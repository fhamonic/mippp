#pragma once
// Skeptic's harness for GLPK. Build with -fno-access-control: model_params and the
// protected helpers of glpk_milp. M mirrors glpk_milp::solve() (which overwrites
// cb_func) and adds the recipe's root LP. Default construction leaves presolve on,
// so a baseline with no attach is the wrapper's real default.
#include "mippp/solvers/glpk/all.hpp"
#include "cut_conformance.hpp"
#include <cstring>
#include <fstream>
#include <sstream>
#include <sys/wait.h>
#include <unistd.h>

namespace v1 = mippp::glpk::impl::v1;
using v1::glp_prob;
using v1::glp_tree;
using Base = mippp::glpk_milp;

namespace vg {
enum { R_ROWGEN = 1, R_BINGO = 2, R_HEUR = 3, R_CUTGEN = 4, R_BRANCH = 5, R_SELECT = 6, R_PREPRO = 7 };
enum { SF_PATH = 0x01 | 0x10 | 0x20 | 0x40 };
struct attr_t { int level, origin, klass; double foo_bar[7]; };
struct natives {
    int (*ios_add_row)(glp_tree *, const char *, int, int, int, const int *, const double *, int, double);
    int (*curr_node)(glp_tree *);
    int (*node_level)(glp_tree *, int);
    int (*up_node)(glp_tree *, int);
    int (*pool_size)(glp_tree *);
    void (*row_attr)(glp_tree *, int, attr_t *);
    void (*term_hook)(int (*)(void *, const char *), void *);
    void (*scale_prob)(glp_prob *, int);
    void (*unscale_prob)(glp_prob *);
    void (*adv_basis)(glp_prob *, int);
    double (*get_row_prim)(glp_prob *, int);
    const char * (*get_row_name)(glp_prob *, int);
    int (*get_status)(glp_prob *);
    int (*read_mps)(glp_prob *, int, const void *, const char *);
    void (*set_prob_name)(glp_prob *, const char *);
    int (*get_num_int)(glp_prob *);
    int (*get_num_bin)(glp_prob *);
    static natives & of(const Base & m) {
        static natives n = [&] {
            const auto l = m.native_api().library_path();
            natives r{};
#define V(field, name) r.field = cp::symf<decltype(r.field)>(l, name)
            V(ios_add_row, "glp_ios_add_row"); V(curr_node, "glp_ios_curr_node"); V(node_level, "glp_ios_node_level");
            V(up_node, "glp_ios_up_node"); V(pool_size, "glp_ios_pool_size"); V(row_attr, "glp_ios_row_attr");
            V(term_hook, "glp_term_hook"); V(scale_prob, "glp_scale_prob"); V(unscale_prob, "glp_unscale_prob");
            V(adv_basis, "glp_adv_basis"); V(get_row_prim, "glp_get_row_prim"); V(get_row_name, "glp_get_row_name");
            V(get_status, "glp_get_status"); V(read_mps, "glp_read_mps"); V(set_prob_name, "glp_set_prob_name");
            V(get_num_int, "glp_get_num_int"); V(get_num_bin, "glp_get_num_bin");
#undef V
            return r;
        }();
        return n;
    }
};

// What the solve does, one removable piece at a time.
struct path {
    bool presolve_off = false;     // the recipe sets this in attach
    bool root_lp = true;           // run scale/adv/simplex when presolve is off
    bool scale = true;
    bool adv_basis = true;
    bool subtract_root_time = true;
    bool clear_stale = true;       // a failed root has no MIP solution, whatever glp_mip_status says
    bool empty_shortcut = true;    // n == 0: keep the presolve path
    bool keep_log = false;         // collect the term-hook text
};

// Every run prints GLPK's own error text: an abort is diagnosed from it.
struct M : Base {
    void (*hook)(glp_tree *, void *) = nullptr;
    void * hook_info = nullptr;
    path p;
    long active = -1, closed = -1, iters = 0;
    int native_ret = -1, last_mip_status = 0;
    double root_seconds = 0, solve_seconds = 0;
    bool stopped_at_root = false;
    std::string log;

    static int on_term(void * self, const char * s) {
        auto & m = *static_cast<M *>(self);
        if(m.p.keep_log) { m.log += s; std::fputs(s, stdout); }
        if(s[0] == '+') {
            if(const char * q = std::strrchr(s, '(')) {
                long a = 0, d = 0;
                if(std::sscanf(q, "(%ld; %ld)", &a, &d) == 2) { m.active = a; m.closed = d; }
            }
        } else if(std::strncmp(s, "glp_", 4) == 0 || std::strncmp(s, "ios_", 4) == 0 || std::strncmp(s, "npp_", 4) == 0 ||
                  std::strstr(s, "Error detected") || std::strstr(s, "Assertion failed")) {
            std::fprintf(stderr, "   [glpk says] %s", s);
        }
        return 1;
    }
    int root_lp() {
        auto & n = natives::of(*this);
        if(p.scale) n.scale_prob(model, SF_PATH);
        if(p.adv_basis) n.adv_basis(model, 0);
        v1::glp_smcp s;
        glp->init_smcp(&s);
        s.msg_lev = v1::GLP_MSG_OFF;
        s.tm_lim = model_params.tm_lim;
        const auto t0 = clk::now();
        const int r = glp->simplex(model, &s);
        root_seconds = std::chrono::duration<double>(clk::now() - t0).count();
        if(r != 0) return r;
        const int st = glp->get_status(model);
        if(st == v1::GLP_OPT) return 0;
        if(st == v1::GLP_NOFEAS) return v1::GLP_ENOPFS;
        if(st == v1::GLP_UNBND) return v1::GLP_ENODFS;
        return v1::GLP_EFAIL;
    }
    // A control can abort the process: the search then runs in a child, one verdict per case.
    bool fork_solve = false;
    double forked_value = NAN;
    bool forked_abort = false;
    void solve() {
        if(!fork_solve || !hook) { native_solve(); return; }
        struct out { int ret, mip, closed, crossed; double value; long iters; } r{};
        int fd[2];
        if(pipe(fd) != 0) std::abort();
        std::fflush(stdout); std::fflush(stderr);
        const pid_t pid = fork();
        if(pid == 0) {
            close(fd[0]);
            native_solve();
            r.ret = native_ret; r.mip = last_mip_status; r.closed = static_cast<int>(closed);
            r.crossed = stopped_at_root; r.value = glp->mip_obj_val(model) + objective_offset; r.iters = iters;
            if(write(fd[1], &r, sizeof r) != static_cast<ssize_t>(sizeof r)) _exit(3);
            _exit(0);
        }
        close(fd[1]);
        const ssize_t got = read(fd[0], &r, sizeof r);
        close(fd[0]);
        int st = 0; waitpid(pid, &st, 0);
        reset_status();
        if(WIFSIGNALED(st) || got != static_cast<ssize_t>(sizeof r)) {
            forked_abort = true;
            std::println(stderr, "   CHILD ABORTED signal={}", WIFSIGNALED(st) ? WTERMSIG(st) : -1);
            _status = mippp::status::failed{false};
            forked_value = NAN;
        } else {
            forked_value = r.value; native_ret = r.ret; closed = r.closed; iters = r.iters;
            _status = _intopt_status(r.ret, r.crossed ? v1::GLP_UNDEF : r.mip);
        }
        _mark_results_valid();
    }
    double get_solution_value() { return fork_solve && hook ? forked_value : Base::get_solution_value(); }
    void native_solve() {
        reset_status();
        const auto saved = _round_integral_sides();
        auto & n = natives::of(*this);
        active = closed = -1; log.clear(); stopped_at_root = false;
        n.term_hook(on_term, this);
        const int term_out = glp->term_out(v1::GLP_ON);
        const int lev = model_params.msg_lev, frq = model_params.out_frq, presolve = model_params.presolve, tm = model_params.tm_lim;
        model_params.msg_lev = v1::GLP_MSG_ON;
        if(const char * d = std::getenv("VG_DBG")) { model_params.msg_lev = v1::GLP_MSG_DBG; p.keep_log = true; (void)d; }
        model_params.out_frq = 100000000;
        model_params.cb_func = hook;
        model_params.cb_info = hook_info;
        const int it0 = glp->get_it_cnt(model);
        const auto t0 = clk::now();
        int ret = 0;
        const bool off = p.presolve_off && !(p.empty_shortcut && glp->get_num_cols(model) == 0);
        if(off) {
            model_params.presolve = v1::GLP_OFF;
            if(p.root_lp) ret = root_lp();
            if(ret != 0) stopped_at_root = true;
            if(ret == 0 && p.subtract_root_time && tm != std::numeric_limits<int>::max()) {
                const int used = static_cast<int>(root_seconds * 1000);
                model_params.tm_lim = std::max(tm - used, 0);
            }
        }
        if(ret == 0) ret = glp->intopt(model, &model_params);
        solve_seconds = std::chrono::duration<double>(clk::now() - t0).count();
        native_ret = ret;
        iters = glp->get_it_cnt(model) - it0;
        model_params.msg_lev = lev; model_params.out_frq = frq; model_params.presolve = presolve; model_params.tm_lim = tm;
        const bool crossed = (ret == v1::GLP_EBOUND) && _has_crossed_bounds();
        _restore_sides(saved);
        glp->term_out(term_out);
        n.term_hook(nullptr, nullptr);
        last_mip_status = glp->mip_status(model);
        if(crossed) _status = mippp::status::infeasible{};
        else if(stopped_at_root && p.clear_stale) _status = _intopt_status(ret, v1::GLP_UNDEF);
        else _status = _intopt_status(ret, last_mip_status);
        _mark_results_valid();
    }
    std::size_t get_node_count() { return closed < 0 ? 0u : static_cast<std::size_t>(closed); }
    using ctr = mippp::model_constraint_t<Base>;
};

// ---------------------------------------------------------------- recipe ----
// One removable piece at a time: cf::sanitize is all of them at once.
struct cut_rules {
    bool fold = true, strip_zeros = true, skip_empty = true, drop_satisfied = true, split_eq = true;
    bool normalize = false;      // divide by max |a| before anything looks at the violation
    int stall_k = 0;             // offer nothing after k rounds at a node without LP progress
    double stall_eps = 1e-4;
    bool direct_rows = false;    // glp_add_rows instead of the pool
    bool irowgen = false;        // hook on IROWGEN instead of ICUTGEN (direct rows only)
    double tol = 1e-9;
};
inline std::optional<cp::raw_cut> tidy(const cp::raw_cut & in, const std::vector<double> & x, const cut_rules & r) {
    cp::raw_cut out{{}, {}, in.sense, in.rhs};
    if(r.fold) {
        std::map<int, double> folded;
        for(std::size_t k = 0; k < in.ind.size(); ++k) folded[in.ind[k]] += in.val[k];
        for(auto [j, v] : folded) { out.ind.push_back(j); out.val.push_back(v); }
    } else {
        out.ind = in.ind; out.val = in.val;
    }
    if(r.strip_zeros) {
        std::vector<int> i2; std::vector<double> v2;
        for(std::size_t k = 0; k < out.ind.size(); ++k) if(out.val[k] != 0.0) { i2.push_back(out.ind[k]); v2.push_back(out.val[k]); }
        out.ind = std::move(i2); out.val = std::move(v2);
    }
    if(r.skip_empty && out.ind.empty()) return std::nullopt;
    if(r.normalize) {
        double big = 0;
        for(double v : out.val) big = std::max(big, std::fabs(v));
        if(big > 0) { for(double & v : out.val) v /= big; out.rhs /= big; }
    }
    if(r.drop_satisfied && out.violation(x) <= r.tol) return std::nullopt;
    return out;
}

struct ctx {
    cp::separator sep;
    const v1::glpk_api * api = nullptr;
    natives * n = nullptr;
    cut_rules rules;
    int cols = 0;           // columns of the model, as the wrapper knows them
    long offered = 0, passed = 0, threw = 0, rounds_cap = 0, rounds = 0;
    std::vector<int> ind;
    std::vector<double> val, x;
    std::function<void(glp_tree *, int)> also;  // sees every reason
    int st_node = -1, st_count = 0; double st_last = 0; long st_stops = 0;
    const char * row_name = nullptr;
};

inline void add_native(ctx & c, glp_tree * t, glp_prob * P, const cp::raw_cut & r, const char sense) {
    const auto len = r.ind.size();
    c.ind.assign(len + 1, 0); c.val.assign(len + 1, 0.0);
    for(std::size_t k = 0; k < len; ++k) { c.ind[k + 1] = r.ind[k]; c.val[k + 1] = r.val[k]; }
    const int type = sense == 'L' ? v1::GLP_UP : sense == 'G' ? v1::GLP_LO : v1::GLP_FX;
    if(c.rules.direct_rows) {
        const int i = c.api->add_rows(P, 1);
        if(c.row_name) c.api->set_row_name(P, i, c.row_name);
        c.api->set_mat_row(P, i, static_cast<int>(len), c.ind.data(), c.val.data());
        c.api->set_row_bnds(P, i, type, r.rhs, r.rhs);
    } else {
        c.n->ios_add_row(t, c.row_name, 0, 0, static_cast<int>(len), c.ind.data(), c.val.data(), type, r.rhs);
    }
    ++c.passed;
}
inline void hook(glp_tree * t, void * u) {
    auto & c = *static_cast<ctx *>(u);
    try {
        const int why = c.api->ios_reason(t);
        if(c.also) c.also(t, why);
        if(why != (c.rules.irowgen ? R_ROWGEN : R_CUTGEN)) return;
        if(c.rounds_cap && ++c.rounds > c.rounds_cap) { c.api->ios_terminate(t); return; }
        glp_prob * P = c.api->ios_get_prob(t);
        if(c.rules.stall_k) {
            const int node = c.n->curr_node(t);
            const double lp = c.api->get_obj_val(P);
            if(node == c.st_node && std::fabs(lp - c.st_last) < c.rules.stall_eps * (1 + std::fabs(c.st_last))) ++c.st_count; else c.st_count = 0;
            c.st_node = node; c.st_last = lp;
            if(c.st_count >= c.rules.stall_k) { ++c.st_stops; return; }
        }
        const int n = c.api->get_num_cols(P);
        c.x.assign(static_cast<std::size_t>(std::max(n, c.cols)) + 1, 0.0);
        for(int j = 1; j <= n; ++j) c.x[static_cast<std::size_t>(j)] = c.api->get_col_prim(P, j);
        const std::vector<double> & x = c.x;
        c.sep(x, [&](const cp::raw_cut & raw) {
            ++c.offered;
            if(c.rules.split_eq && raw.sense == 'E') {
                for(const char s : {'L', 'G'}) {
                    cp::raw_cut one = raw; one.sense = s;
                    if(const auto r = tidy(one, x, c.rules)) add_native(c, t, P, *r, s);
                }
                return 0;
            }
            if(const auto r = tidy(raw, x, c.rules)) add_native(c, t, P, *r, r->sense);
            return 0;
        });
    } catch(...) {
        ++c.threw;
        c.api->ios_terminate(t);
    }
}
inline std::vector<std::unique_ptr<ctx>> & kept() { static std::vector<std::unique_ptr<ctx>> k; return k; }
inline ctx & attach(M & m, cp::separator sep, const cut_rules & rules = {}) {
    kept().push_back(std::make_unique<ctx>());
    ctx & c = *kept().back();
    c.sep = std::move(sep);
    c.api = &m.native_api();
    c.n = &natives::of(m);
    c.rules = rules;
    c.cols = static_cast<int>(m.num_variables());
    m.hook = hook; m.hook_info = &c;
    return c;
}
inline const char * vers(const Base & m) { return m.native_api().version(); }
}  // namespace vg
