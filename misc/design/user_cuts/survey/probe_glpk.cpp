// Built with -fno-access-control: glpk_milp::model_params holds the iocp.
// glpk_milp::solve() writes cb_func itself on every solve, so a hook set from
// outside is overwritten; M below mirrors that solve() with its own hook, the
// presolve-off root LP, and a read of the last progress line through
// glp_term_hook, which gives nodes and simplex iterations with no cb_func at all.
#include "mippp/solvers/glpk/all.hpp"
#include "cut_common.hpp"

#include <cstring>

namespace v1 = mippp::glpk::impl::v1;
using v1::glp_prob;
using v1::glp_tree;
using Base = mippp::glpk_milp;

// glpk.h 4.65
enum { R_ROWGEN = 1, R_BINGO = 2, R_HEUR = 3, R_CUTGEN = 4, R_BRANCH = 5, R_SELECT = 6, R_PREPRO = 7 };
enum { RF_REG = 0, RF_LAZY = 1, RF_CUT = 2 };
enum { SF_PATH = 0x01 | 0x10 | 0x20 | 0x40, PP_NONE = 0, PP_ROOT = 1 };
struct attr_t { int level, origin, klass; double foo_bar[7]; };

struct natives {
    int (*ios_add_row)(glp_tree *, const char *, int, int, int, const int *, const double *, int, double);
    int (*curr_node)(glp_tree *);
    int (*node_level)(glp_tree *, int);
    int (*up_node)(glp_tree *, int);
    void * (*node_data)(glp_tree *, int);
    int (*pool_size)(glp_tree *);
    void (*row_attr)(glp_tree *, int, attr_t *);
    void (*term_hook)(int (*)(void *, const char *), void *);
    void (*scale_prob)(glp_prob *, int);
    void (*adv_basis)(glp_prob *, int);
    void (*branch_upon)(glp_tree *, int, int);
    int (*get_num_int)(glp_prob *);
    double (*get_row_prim)(glp_prob *, int);
    int (*heur_sol)(glp_tree *, const double *);
    static natives & of(const Base & m) {
        static natives n = [&] {
            const auto l = m.native_api().library_path();
            natives r{};
            r.ios_add_row = cp::symf<decltype(r.ios_add_row)>(l, "glp_ios_add_row");
            r.curr_node = cp::symf<decltype(r.curr_node)>(l, "glp_ios_curr_node");
            r.node_level = cp::symf<decltype(r.node_level)>(l, "glp_ios_node_level");
            r.up_node = cp::symf<decltype(r.up_node)>(l, "glp_ios_up_node");
            r.node_data = cp::symf<decltype(r.node_data)>(l, "glp_ios_node_data");
            r.pool_size = cp::symf<decltype(r.pool_size)>(l, "glp_ios_pool_size");
            r.row_attr = cp::symf<decltype(r.row_attr)>(l, "glp_ios_row_attr");
            r.term_hook = cp::symf<decltype(r.term_hook)>(l, "glp_term_hook");
            r.scale_prob = cp::symf<decltype(r.scale_prob)>(l, "glp_scale_prob");
            r.adv_basis = cp::symf<decltype(r.adv_basis)>(l, "glp_adv_basis");
            r.branch_upon = cp::symf<decltype(r.branch_upon)>(l, "glp_ios_branch_upon");
            r.get_num_int = cp::symf<decltype(r.get_num_int)>(l, "glp_get_num_int");
            r.get_row_prim = cp::symf<decltype(r.get_row_prim)>(l, "glp_get_row_prim");
            r.heur_sol = cp::symf<decltype(r.heur_sol)>(l, "glp_ios_heur_sol");
            return r;
        }();
        return n;
    }
};

struct M : Base {
    void (*hook)(glp_tree *, void *) = nullptr;
    void * hook_info = nullptr;
    std::function<void()> after_solve;
    long it_cnt = -1, active = -1, closed = -1;
    double last_bound = NAN;
    int native_ret = -1;
    bool scale = true;
    double root_seconds = 0;

    static int on_term(void * self, const char * s) {
        auto & m = *static_cast<M *>(self);
        if(s[0] == '+') {
            long it = 0;
            if(std::sscanf(s + 1, "%ld", &it) == 1) m.it_cnt = it;
            if(const char * p = std::strrchr(s, '(')) {
                long a = 0, d = 0;
                if(std::sscanf(p, "(%ld; %ld)", &a, &d) == 2) { m.active = a; m.closed = d; }
            }
            const char * rel = std::strstr(s, ">=");
            if(!rel) rel = std::strstr(s, "<=");
            if(rel) {
                char * end = nullptr;
                const double v = std::strtod(rel + 2, &end);
                m.last_bound = end != rel + 2 ? v : NAN;
            }
        } else if(std::strncmp(s, "glp_", 4) == 0 || std::strncmp(s, "ios_", 4) == 0 ||
                  std::strstr(s, "Error detected") || std::strstr(s, "Assertion failed") || std::strstr(s, "npp_")) {
            std::fprintf(stderr, "   [glpk says] %s", s);
        }
        return 1;
    }
    // what preprocess_and_solve_mip does to its transformed problem (glpapi09.c), here on P
    int root_lp() {
        auto & n = natives::of(*this);
        if(scale) n.scale_prob(model, SF_PATH);
        n.adv_basis(model, 0);
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
    void solve() {
        reset_status();
        const auto saved = _round_integral_sides();
        auto & n = natives::of(*this);
        it_cnt = active = closed = -1; last_bound = NAN;
        n.term_hook(on_term, this);
        const int term_out = glp->term_out(v1::GLP_ON);
        const int lev = model_params.msg_lev;
        const int frq = model_params.out_frq;
        model_params.msg_lev = v1::GLP_MSG_ON;
        model_params.out_frq = 1000000;
        model_params.cb_func = hook;
        model_params.cb_info = hook_info;
        int ret = 0;
        if(!model_params.presolve) ret = root_lp();
        if(ret == 0) ret = glp->intopt(model, &model_params);
        native_ret = ret;
        model_params.msg_lev = lev;
        model_params.out_frq = frq;
        const bool crossed = (ret == v1::GLP_EBOUND) && _has_crossed_bounds();
        _restore_sides(saved);
        glp->term_out(term_out);
        n.term_hook(nullptr, nullptr);
        if(crossed) _status = mippp::status::infeasible{};
        else _status = _intopt_status(ret, glp->mip_status(model));
        _mark_results_valid();
        if(after_solve) after_solve();
    }
    std::size_t get_node_count() { return closed < 0 ? 0u : static_cast<std::size_t>(closed); }
    double get_dual_bound() { return std::isnan(last_bound) ? get_solution_value() : last_bound; }
    // gadget rows are all <=, so the upper bound is the rhs; the harness P needs these names
    using ctr = mippp::model_constraint_t<Base>;
    void set_constraint_rhs(ctr c, double v) { set_constraint_upper_bound(c, v); }
    double get_constraint_rhs(ctr c) { return get_constraint_upper_bound(c); }
};

// ------------------------------------------------------------------- hook ---
struct ctx {
    cp::separator sep;
    const v1::glpk_api * api = nullptr;
    natives * n = nullptr;
    glp_prob * P = nullptr;
    int want = R_CUTGEN;
    bool pool = true;
    bool guard = false;  // refuse what makes GLPK abort: the matrix must reach its last line
    int P_cols = 0;
    int orig_m = -1;
    long reasons[8] = {};
    long other_prob = 0, same_prob = 0;
    long state_calls[3] = {};
    int rows_min[3] = {1 << 30, 1 << 30, 1 << 30}, rows_max[3] = {-1, -1, -1};
    std::vector<int> ind;
    std::vector<double> val, x;
    bool gadget_like = false;
    std::string reasons_line() const {
        return std::format("reasons ISELECT={} IPREPRO={} IROWGEN={} IHEUR={} ICUTGEN={} IBRANCH={} IBINGO={}",
                           reasons[R_SELECT], reasons[R_PREPRO], reasons[R_ROWGEN], reasons[R_HEUR], reasons[R_CUTGEN], reasons[R_BRANCH], reasons[R_BINGO]);
    }
    std::string rows_line() const {
        const auto one = [&](int s) { return state_calls[s] ? std::format("{} calls, extra rows {}..{}", state_calls[s], rows_min[s], rows_max[s]) : std::string("none"); };
        return std::format("rows beyond the model by y0 state: frac[{}] y0=0[{}] y0=1[{}]", one(0), one(1), one(2));
    }
};
static std::vector<std::unique_ptr<ctx>> keep;

static bool g_strip_zeros = true;  // the pool of GLPK < 4.65 aborts on a zero coefficient; hull_cut(3) has one
static int add_cut(ctx & c, glp_tree * t, glp_prob * p, const cp::raw_cut & r0) {
    const int n = c.api->get_num_cols(p);
    cp::raw_cut stripped;
    if(c.pool && g_strip_zeros) {
        stripped.sense = r0.sense; stripped.rhs = r0.rhs;
        for(std::size_t k = 0; k < r0.ind.size(); ++k) if(r0.val[k] != 0.0) { stripped.ind.push_back(r0.ind[k]); stripped.val.push_back(r0.val[k]); }
    }
    const cp::raw_cut & r = c.pool && g_strip_zeros ? stripped : r0;
    const auto len = r.ind.size();
    if(c.guard) {
        if(c.pool && r.sense == 'E') return -1;
        std::set<int> seen;
        for(int j : r.ind) {
            if(j < 1 || j > n) return -2;
            if(!seen.insert(j).second) return -3;
        }
    }
    c.ind.assign(len + 1, 0);
    c.val.assign(len + 1, 0.0);
    for(std::size_t k = 0; k < len; ++k) { c.ind[k + 1] = r.ind[k]; c.val[k + 1] = r.val[k]; }
    const int type = r.sense == 'L' ? v1::GLP_UP : r.sense == 'G' ? v1::GLP_LO : v1::GLP_FX;
    if(c.pool) {
        c.n->ios_add_row(t, nullptr, 0, 0, static_cast<int>(len), c.ind.data(), c.val.data(), type, r.rhs);
    } else {
        const int i = c.api->add_rows(p, 1);
        c.api->set_mat_row(p, i, static_cast<int>(len), c.ind.data(), c.val.data());
        c.api->set_row_bnds(p, i, type, r.rhs, r.rhs);
    }
    return 0;
}
static void read_point(ctx & c, glp_prob * p) {
    const int n = c.api->get_num_cols(p);
    c.x.assign(static_cast<std::size_t>(std::max(n, c.P_cols)) + 1, 0.0);
    for(int j = 1; j <= n; ++j) c.x[static_cast<std::size_t>(j)] = c.api->get_col_prim(p, j);
}
static void hook(glp_tree * t, void * u) {
    auto & c = *static_cast<ctx *>(u);
    const int why = c.api->ios_reason(t);
    if(why >= 1 && why <= 7) c.reasons[why]++;
    glp_prob * p = c.api->ios_get_prob(t);
    if(c.orig_m < 0) c.orig_m = c.api->get_num_rows(p);
    if(why != c.want) return;
    (p == c.P ? c.same_prob : c.other_prob)++;
    read_point(c, p);
    if(c.gadget_like) {
        const double v = c.x[1];
        const int s = v < 1e-6 ? 1 : v > 1 - 1e-6 ? 2 : 0;
        const int extra = c.api->get_num_rows(p) - c.orig_m;
        c.state_calls[s]++;
        c.rows_min[s] = std::min(c.rows_min[s], extra);
        c.rows_max[s] = std::max(c.rows_max[s], extra);
    }
    c.sep(c.x, [&](const cp::raw_cut & r) { return add_cut(c, t, p, r); });
}
static bool g_guard = false;
static bool g_keep_gmi = false;
static std::string tail(const M & m);
static ctx & make_ctx(M & m, cp::separator sep, const std::string & mode) {
    keep.push_back(std::make_unique<ctx>());
    ctx & c = *keep.back();
    c.sep = std::move(sep);
    c.api = &m.native_api();
    c.n = &natives::of(m);
    c.P = m.native_model();
    c.P_cols = c.api->get_num_cols(c.P);
    c.guard = g_guard;
    c.pool = mode == "pool" || mode == "presolve_on";
    c.want = mode == "rows_irowgen" ? R_ROWGEN : R_CUTGEN;
    c.gadget_like = c.P_cols == 3 * 10 + 1 + 9;
    return c;
}
static void attach(M & m, cp::separator sep, const std::string & mode) {
    ctx & c = make_ctx(m, std::move(sep), mode);
    m.model_params.presolve = mode == "presolve_on" ? v1::GLP_ON : v1::GLP_OFF;
    // glp_gmi_gen and the MIR generator need a factorization, which glp_add_rows at
    // ICUTGEN has just invalidated: GLPK aborts the process (out_glpk_gen.txt).
    if(mode == "rows_icutgen" && !g_keep_gmi) m.model_params.gmi_cuts = m.model_params.mir_cuts = v1::GLP_OFF;
    m.hook = hook;
    m.hook_info = &c;
    m.after_solve = [&c, &m, mode] {
        std::println(stderr, "   [hook {}] {} point on P {} / on another problem {} | {}", mode, c.reasons_line(), c.same_prob, c.other_prob, tail(m));
        if(c.gadget_like) std::println(stderr, "   [hook {}] {}", mode, c.rows_line());
    };
}
static void one_thread(M &) {}  // GLPK has no solver thread setting: one search thread by design
static void scope_settings(M & m) {
    auto & q = m.model_params;
    q.presolve = v1::GLP_OFF;
    q.pp_tech = PP_NONE;
    q.mir_cuts = q.gmi_cuts = q.cov_cuts = q.clq_cuts = v1::GLP_OFF;
    q.fp_heur = q.ps_heur = q.sr_heur = v1::GLP_OFF;
    q.binarize = v1::GLP_OFF;
    q.mip_gap = 0.0;
}
static std::string tail(const M & m) {
    return std::format("it={} closed={} active={} ret=0x{:x} root_lp={:.3f}s", m.it_cnt, m.closed, m.active, m.native_ret, m.root_seconds);
}
static void TL(M & m, double s) { m.set_time_limit(std::chrono::duration<double>(s)); }

static const int ATSP_N = std::getenv("CUT_ATSP_N") ? std::atoi(std::getenv("CUT_ATSP_N")) : 14;

static cp::backend<M> make_backend() {
    cp::backend<M> B;
    B.attach = attach; B.scope_settings = scope_settings; B.one_thread = one_thread;
    B.cut_modes = {"pool", "rows_icutgen", "rows_irowgen"};
    B.atsp_n = ATSP_N;
    B.atsp_complement_n = ATSP_N;
    B.atsp_limit = 60;
    return B;
}

// ------------------------------------------------- does the hook change the path?
static void noop_hook(glp_tree * t, void * u) {
    auto & c = *static_cast<ctx *>(u);
    const int why = c.api->ios_reason(t);
    if(why >= 1 && why <= 7) c.reasons[why]++;
}
static void reading_hook(glp_tree * t, void * u) {
    auto & c = *static_cast<ctx *>(u);
    const int why = c.api->ios_reason(t);
    if(why >= 1 && why <= 7) c.reasons[why]++;
    if(why == R_CUTGEN || why == R_ROWGEN) read_point(c, c.api->ios_get_prob(t));
}
template <class Build>
static void pathcheck_on(const char * name, Build build, const double limit) {
    for(const int pre : {1, 0}) for(const int kind : {0, 1, 2}) {
        M m;
        m.model_params.presolve = pre;
        build(m);
        TL(m, limit);
        ctx c;
        c.api = &m.native_api(); c.n = &natives::of(m); c.P = m.native_model(); c.P_cols = c.api->get_num_cols(c.P);
        if(kind == 1) { m.hook = noop_hook; m.hook_info = &c; }
        if(kind == 2) { m.hook = reading_hook; m.hook_info = &c; }
        const double t = cp::timed_solve(m);
        std::println(stderr, "== PATH {} presolve={} {}: {} {}{}", name, pre ? "on" : "off", kind == 0 ? "no cb_func" : kind == 1 ? "no-op cb_func" : "reading cb_func",
                     cp::summary(m, t), tail(m), kind ? " " + c.reasons_line() : std::string());
    }
}
static void pathcheck() {
    pathcheck_on("atsp", [](M & m) { cp::build_atsp(m, ATSP_N); }, 60);
    pathcheck_on("atsp+complements", [](M & m) { cp::build_atsp(m, ATSP_N, true); }, 60);
    pathcheck_on("mkp 5x30", [](M & m) { cp::build_mkp(m, 5, 30); }, 30);
    pathcheck_on("mkp 30x100 (time limit)", [](M & m) { cp::build_mkp(m, 30, 100); }, 10);
    pathcheck_on("gadgets k=10", [](M & m) { cp::build_gadgets(m, 10, false); }, 30);
}

// ---------------------------------------------- presolve on: what does the hook see?
struct map_ctx { const v1::glpk_api * api; natives * n; glp_prob * P; bool printed = false; std::string name; };
static void map_hook(glp_tree * t, void * u) {
    auto & c = *static_cast<map_ctx *>(u);
    if(c.api->ios_reason(t) != R_CUTGEN && c.api->ios_reason(t) != R_ROWGEN) return;
    if(c.printed) return;
    c.printed = true;
    glp_prob * p = c.api->ios_get_prob(t);
    std::string names;
    for(int j = 1; j <= std::min(3, c.api->get_num_cols(p)); ++j) {
        const char * nm = c.api->get_col_name(p, j);
        names += std::format(" {}={}", j, nm ? nm : "(null)");
    }
    std::string pnames;
    for(int j = 1; j <= 3; ++j) { const char * nm = c.api->get_col_name(c.P, j); pnames += std::format(" {}={}", j, nm ? nm : "(null)"); }
    std::println(stderr, "== MAP {}: tree problem {} P; P rows={} cols={} ints={}; tree problem rows={} cols={} ints={}; tree col names:{}; P col names:{}; tree LP obj={:.6g}",
                 c.name, p == c.P ? "==" : "!=", c.api->get_num_rows(c.P), c.api->get_num_cols(c.P), c.n->get_num_int(c.P), c.api->get_num_rows(p), c.api->get_num_cols(p), c.n->get_num_int(p), names, pnames, c.api->get_obj_val(p));
}
template <class Build>
static void map_on(const char * name, Build build, double limit) {
    for(const int pre : {1, 0}) {
        M m;
        m.model_params.presolve = pre;
        build(m);
        TL(m, limit);
        const int cols = m.native_api().get_num_cols(m.native_model());
        for(int j = 1; j <= cols; ++j) { const auto s = std::format("c{}", j); m.native_api().set_col_name(m.native_model(), j, s.c_str()); }
        map_ctx c{&m.native_api(), &natives::of(m), m.native_model(), false, std::format("{} presolve={}", name, pre ? "on" : "off")};
        m.hook = map_hook; m.hook_info = &c;
        const double t = cp::timed_solve(m);
        std::println(stderr, "   {} {}", cp::summary(m, t), tail(m));
    }
}
static void map_probe() {
    map_on("atsp", [](M & m) { cp::build_atsp(m, ATSP_N); }, 20);
    map_on("atsp+complements", [](M & m) { cp::build_atsp(m, ATSP_N, true); }, 20);
    map_on("gadgets", [](M & m) { cp::build_gadgets(m, 10, false); }, 20);
    map_on("mkp 5x30", [](M & m) { cp::build_mkp(m, 5, 30); }, 20);
}
// Presolve left on, cuts written over P's columns as a careless wrapper would. Unguarded.
// zab[2] + 6 yb[2] <= 9 is valid in P's numbering. With the presolver on, the zero
// column dummy (31) is gone, so P's 37 and 34 name other columns of the tree problem.
static void presolve_silent() {
    // scope settings: GLPK's heuristics run before ICUTGEN and find the optimum first otherwise
    for(const char * mode : {"presolve_on", "pool"}) for(const bool flip : {false, true}) for(const bool heur : {false, true}) {
        M m; scope_settings(m);
        if(heur) { m.model_params.fp_heur = v1::GLP_ON; m.model_params.sr_heur = v1::GLP_ON; }
        auto g = cp::build_gadgets(m, 10, flip); TL(m, 30);
        auto s = std::make_shared<cp::stats>();
        bool once = false;
        const cp::raw_cut cut{{g.za[12], g.dummy + 3}, {1.0, 6.0}, 'L', 9.0};
        attach(m, cp::watched([&](const auto &, const cp::add_fn & add) { if(!once) { once = true; add(cut); } }, s), mode);
        const double t = cp::timed_solve(m);
        std::println(stderr, "== SILENT mode {} flip={} heuristics={} valid cut over P's columns, cols {} and {}: {} known opt {} {} => {}", mode, flip, heur, cut.ind[0], cut.ind[1], cp::summary(m, t), g.opt, tail(m),
                     std::fabs(m.get_solution_value() - g.opt) < 1e-4 ? "optimum KEPT" : "optimum LOST, no error reported");
    }
}
// Loose big-M facility location: what presolve can actually tighten.
static void build_cflp(M & m, const int fac, const int cus, const unsigned seed, const double big) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> F(50, 150), C(1, 20), D(1, 10);
    const auto Fa = static_cast<std::size_t>(fac), Cu = static_cast<std::size_t>(cus);
    auto y = m.add_binary_variables(Fa);
    auto x = m.add_variables(Fa * Cu);
    std::vector<double> f(Fa), d(Cu), c(Fa * Cu);
    for(auto & v : f) v = std::round(F(rng));
    for(auto & v : d) v = std::round(D(rng));
    for(auto & v : c) v = std::round(C(rng));
    const auto Is = std::views::iota(std::size_t{0}, Fa);
    const auto Js = std::views::iota(std::size_t{0}, Cu);
    double total = 0;
    for(double v : d) total += v;
    m.set_minimization();
    m.set_objective(xsum(Is, [&](std::size_t i) { return f[i] * y(i); }) + xsum(Is, [&](std::size_t i) { return xsum(Js, [&, i](std::size_t j) { return c[i * Cu + j] * x(i * Cu + j); }); }));
    for(std::size_t j : Js) m.add_constraint(xsum(Is, [&, j](std::size_t i) { return 1.0 * x(i * Cu + j); }) == d[j]);
    for(std::size_t i : Is) m.add_constraint(xsum(Js, [&, i](std::size_t j) { return 1.0 * x(i * Cu + j); }) - big * total * y(i) <= 0);
    for(std::size_t i : Is) for(std::size_t j : Js) m.add_constraint(x(i * Cu + j) <= big * total);
}
static void presolve_cost() {
    for(const auto & [fac, cus, big] : {std::tuple{8, 20, 1.0}, {8, 20, 100.0}, {10, 30, 100.0}, {12, 30, 100.0}}) for(const unsigned seed : {1u, 2u}) for(const int pre : {1, 0}) {
        M m; m.model_params.presolve = pre; build_cflp(m, fac, cus, seed, big); TL(m, 25);
        map_ctx c{&m.native_api(), &natives::of(m), m.native_model(), false, std::format("cflp {}x{} big={} seed={} presolve={}", fac, cus, big, seed, pre ? "on" : "off")};
        m.hook = map_hook; m.hook_info = &c;
        const double t = cp::timed_solve(m);
        std::println(stderr, "   {} {}", cp::summary(m, t), tail(m));
    }
}
static void presolve_case(const std::string & which) {
    if(which == "silent") { presolve_silent(); return; }
    if(which == "cost") { presolve_cost(); return; }
    if(which == "e3" || which == "e4") {
        M m; auto a = cp::build_atsp(m, ATSP_N, which == "e4"); TL(m, 60);
        auto s = std::make_shared<cp::stats>();
        attach(m, cp::watched([&a](const auto & x, const cp::add_fn & add) { for(const auto & c : a.separate(x)) add(c); }, s,
            [&a](const auto & x) { return a.objective(x); }, [&a](const auto & x) { return a.point_ok(x); }, [&a](const auto & x) { return a.integral(x); }), "presolve_on");
        std::println(stderr, "== PRESOLVE-ON {} starts", which);
        const double t = cp::timed_solve(m);
        std::println(stderr, "== PRESOLVE-ON {}: {} {}\n   {}", which, cp::summary(m, t), tail(m), s->line());
    } else {
        // hull cut of gadget 3 over P's columns, default presolve, otherwise default settings
        const bool flip = which == "gadgets_flip";
        M m; auto g = cp::build_gadgets(m, 10, flip); TL(m, 30);
        auto s = std::make_shared<cp::stats>();
        bool once = false;
        const auto cut = g.hull_cut(3);
        attach(m, cp::watched([&](const auto & x, const cp::add_fn & add) { if(!once) { once = true; add(cut); } }, s, {}, [&g](const auto & x) { return g.point_ok(x); }), "presolve_on");
        const double t = cp::timed_solve(m);
        std::println(stderr, "== PRESOLVE-ON gadgets flip={} hull cut once: {} (known opt {}) {}\n   {}", flip, cp::summary(m, t), g.opt, tail(m), s->line());
    }
}

// ------------------------------------------------- invalid cuts and non-optimal cuts
static void rootcut() {
    for(const char * mode : {"pool", "rows_icutgen", "rows_irowgen"}) for(const bool flip : {false, true}) {
        { // invalid at the root: inherited by every node, so a global cut in force
            M m; scope_settings(m); auto g = cp::build_gadgets(m, 10, flip); TL(m, 30);
            auto s = std::make_shared<cp::stats>();
            bool once = false; long after = 0, viol = 0;
            const auto cut = g.invalid_global_cut();
            attach(m, cp::watched([&](const auto & x, const cp::add_fn & add) {
                if(!once) { once = true; add(cut); return; }
                ++after; viol += cut.violation(x) > 1e-6;
            }, s, [&g](const auto & x) { return g.objective(x); }, [&g](const auto & x) { return g.point_ok(x); }), mode);
            const double t = cp::timed_solve(m);
            std::println(stderr, "== ROOTCUT {} flip={} invalid cut at the first call (root): {} known opt {} {}; later calls {} violating it {} => {}", mode, flip, cp::summary(m, t), g.opt, tail(m), after, viol,
                         std::fabs(m.get_solution_value() - g.opt) < 1e-4 ? "optimum KEPT" : "optimum LOST, no error reported");
        }
        { // cut off the optimum inside the BETTER subtree: invalid for that subtree
            M m; scope_settings(m); auto g = cp::build_gadgets(m, 10, flip); TL(m, 30);
            auto s = std::make_shared<cp::stats>();
            bool once = false;
            const auto cut = g.invalid_global_cut();
            const int better = 1 - g.worse_state();
            attach(m, cp::watched([&](const auto & x, const cp::add_fn & add) {
                if(!once && g.state(x) == better && cut.violation(x) > 1e-4) { once = true; add(cut); }
            }, s, [&g](const auto & x) { return g.objective(x); }, [&g](const auto & x) { return g.point_ok(x); }), mode);
            const double t = cp::timed_solve(m);
            std::println(stderr, "== BADLOCAL {} flip={} invalid cut in the subtree that holds the optimum (added={}): {} known opt {} {} => {}", mode, flip, once, cp::summary(m, t), g.opt, tail(m),
                         std::fabs(m.get_solution_value() - g.opt) < 1e-4 ? "optimum KEPT" : "optimum LOST, no error reported");
        }
        { // violated by the LP, removes only non-optimal integer points: the worse value of y0 is forbidden
            M m; scope_settings(m); auto g = cp::build_gadgets(m, 10, flip); TL(m, 30);
            auto s = std::make_shared<cp::stats>();
            bool once = false;
            const cp::raw_cut cut = flip ? cp::raw_cut{{g.y[0]}, {1.0}, 'G', 1.0} : cp::raw_cut{{g.y[0]}, {1.0}, 'L', 0.0};
            attach(m, cp::watched([&](const auto &, const cp::add_fn & add) { if(!once) { once = true; add(cut); } }, s), mode);
            const double t = cp::timed_solve(m);
            std::println(stderr, "== DOMINANCE {} flip={} y0 fixed to its better value at the root, integer feasible non-optimal points removed: {} known opt {} {} => {}", mode, flip, cp::summary(m, t), g.opt, tail(m),
                         std::fabs(m.get_solution_value() - g.opt) < 1e-4 ? "optimum KEPT" : "optimum LOST");
        }
        { // removes non-optimal integer points only: Sum z >= opt - 1, at the root
            M m; scope_settings(m); auto g = cp::build_gadgets(m, 10, flip); TL(m, 30);
            auto s = std::make_shared<cp::stats>();
            bool once = false;
            const cp::raw_cut cut{g.z, std::vector<double>(g.z.size(), 1.0), 'G', g.opt - 1};
            attach(m, cp::watched([&](const auto &, const cp::add_fn & add) { if(!once) { once = true; add(cut); } }, s), mode);
            const double t = cp::timed_solve(m);
            std::println(stderr, "== CUTOFF {} flip={} objective cutoff at the root, removes only non-optimal points: {} known opt {} {}", mode, flip, cp::summary(m, t), g.opt, tail(m));
        }
    }
}

// ------------------------------------------------------------ node-local reads
struct reads_ctx { const v1::glpk_api * api; natives * n; int y0; long fires = 0; long mismatch = 0; long fixed_seen = 0; int max_level = 0; };
static void reads_hook(glp_tree * t, void * u) {
    auto & c = *static_cast<reads_ctx *>(u);
    if(c.api->ios_reason(t) != R_CUTGEN) return;
    glp_prob * p = c.api->ios_get_prob(t);
    const int cur = c.n->curr_node(t);
    const int lvl = c.n->node_level(t, cur);
    const int up = c.n->up_node(t, cur);
    const double bnd = c.api->ios_node_bound(t, cur);
    const double lb = c.api->get_col_lb(p, c.y0), ub = c.api->get_col_ub(p, c.y0), v = c.api->get_col_prim(p, c.y0);
    auto * data = static_cast<int *>(c.n->node_data(t, cur));
    const int inherited = data ? *data : -1;
    if(data) *data = cur;
    int a, n_, tot; c.api->ios_tree_size(t, &a, &n_, &tot);
    if(++c.fires <= 24)
        std::println(stderr, "   node={} level={} parent={} node_bound={:.6g} lp_obj={:.6g} y0 local bounds=[{},{}] y0={:.4g} node_data_on_entry={} tree active={} in_tree={} total={}", cur, lvl, up, bnd, c.api->get_obj_val(p), lb, ub, v, inherited, a, n_, tot);
    if(lb == ub) { ++c.fixed_seen; if(std::fabs(v - lb) > 1e-6) ++c.mismatch; }
    c.max_level = std::max(c.max_level, lvl);
}
static void reads() {
    M m; scope_settings(m); auto g = cp::build_gadgets(m, 10, false); TL(m, 30);
    m.model_params.cb_size = sizeof(int);
    reads_ctx c{&m.native_api(), &natives::of(m), g.y[0]};
    m.hook = reads_hook; m.hook_info = &c;
    const double t = cp::timed_solve(m);
    std::println(stderr, "== READS {} {}; fires={} max level={} y0 fixed by local bounds at {} fires, point off those bounds at {}", cp::summary(m, t), tail(m), c.fires, c.max_level, c.fixed_seen, c.mismatch);
}

// -------------------------- a wrapper-side list re-applied at every node: does it pay?
static void reinject() {
    for(const bool flip : {false, true}) {
        const auto run = [&](const std::string & how) {
            M m; scope_settings(m); auto g = cp::build_gadgets(m, 10, flip); TL(m, 30);
            auto s = std::make_shared<cp::stats>();
            std::vector<cp::raw_cut> hulls;
            for(int i = 1; i < 10; ++i) hulls.push_back(g.hull_cut(i));
            std::vector<char> added(hulls.size(), 0);
            bool started = false;
            attach(m, cp::watched([&](const auto & x, const cp::add_fn & add) {
                if(how == "none") return;
                if(how == "root_once") { if(!started) { started = true; for(const auto & c : hulls) add(c); } return; }
                // discovered only inside the worse subtree
                if(!started && g.state(x) != g.worse_state()) return;
                started = true;
                for(std::size_t i = 0; i < hulls.size(); ++i) {
                    if(hulls[i].violation(x) <= 1e-6) continue;
                    if(how == "subtree_once" && added[i]) continue;
                    added[i] = 1;
                    add(hulls[i]);
                }
            }, s, {}, [&g](const auto & x) { return g.point_ok(x); }), "pool");
            const double t = cp::timed_solve(m);
            std::println(stderr, "== REINJECT flip={} {}: {} {} cuts={} calls={}", flip, how, cp::summary(m, t), tail(m), s->cuts.load(), s->calls.load());
        };
        for(const char * how : {"none", "root_once", "subtree_once", "subtree_reinjected"}) run(how);
    }
}

// --------------------------------------------- the pool's filter, and the purge
// maximize Sum x on an odd cycle with x_i + x_{i+1} <= 1: the LP point is all 1/2
template <class X>
static auto odd_cycle(M & m, const int n) {
    auto x = m.add_binary_variables(static_cast<std::size_t>(n));
    const auto ids = std::views::iota(std::size_t{0}, static_cast<std::size_t>(n));
    m.set_maximization();
    m.set_objective(xsum(ids, [&](std::size_t i) { return 1.0 * x(i); }));
    for(std::size_t i : ids) m.add_constraint(x(i) + x((i + 1) % static_cast<std::size_t>(n)) <= 1);
    return x;
}
struct filter_ctx {
    const v1::glpk_api * api; natives * n; bool pool; int orig_m = -1;
    long fires = 0; bool root_done = false, child_done = false; int pending = -1, pending_level = -1, want = 0; std::mt19937 rng{3};
};
static void filter_hook(glp_tree * t, void * u) {
    auto & c = *static_cast<filter_ctx *>(u);
    if(c.api->ios_reason(t) != R_CUTGEN) return;
    glp_prob * p = c.api->ios_get_prob(t);
    if(c.orig_m < 0) c.orig_m = c.api->get_num_rows(p);
    const int cur = c.n->curr_node(t), lvl = c.n->node_level(t, cur);
    ++c.fires;
    if(c.pending >= 0) {
        std::println(stderr, "   after {} violated cuts offered at level {}: rows before={} now={} kept={} (this level={})", c.want, c.pending_level, c.pending, c.api->get_num_rows(p), c.api->get_num_rows(p) - c.pending, lvl);
        c.pending = -1;
        if(c.root_done && c.child_done) { c.api->ios_terminate(t); return; }
    }
    if((lvl == 0 && c.root_done) || (lvl > 0 && c.child_done)) return;
    const int n = c.api->get_num_cols(p);
    std::vector<double> x(static_cast<std::size_t>(n) + 1);
    for(int j = 1; j <= n; ++j) x[static_cast<std::size_t>(j)] = c.api->get_col_prim(p, j);
    std::uniform_int_distribution<int> J(1, n);
    int offered = 0;
    for(int tries = 0; offered < 200 && tries < 200000; ++tries) {
        const int a = J(c.rng), b = J(c.rng), d = J(c.rng);
        if(a == b || a == d || b == d) continue;
        if(x[static_cast<std::size_t>(a)] + x[static_cast<std::size_t>(b)] + x[static_cast<std::size_t>(d)] <= 1.2) continue;
        const int ind[4] = {0, a, b, d};
        const double val[4] = {0, 1, 1, 1};
        if(c.pool) c.n->ios_add_row(t, nullptr, 0, 0, 3, ind, val, v1::GLP_UP, 1.0);
        else { const int i = c.api->add_rows(p, 1); c.api->set_mat_row(p, i, 3, ind, val); c.api->set_row_bnds(p, i, v1::GLP_UP, 1.0, 1.0); }
        ++offered;
    }
    c.pending = c.api->get_num_rows(p) - (c.pool ? 0 : offered);
    c.pending_level = lvl;
    c.want = offered;
    (lvl == 0 ? c.root_done : c.child_done) = true;
}
// how violated must a cut be for the pool to keep it?
struct thresh_ctx { const v1::glpk_api * api; natives * n; bool pool; double v; int fires = 0; int orig_m = -1; int rows_after = -1; };
static void thresh_hook(glp_tree * t, void * u) {
    auto & c = *static_cast<thresh_ctx *>(u);
    const int why = c.api->ios_reason(t);
    if(why != R_CUTGEN && why != R_ROWGEN) return;
    glp_prob * p = c.api->ios_get_prob(t);
    if(c.orig_m < 0) c.orig_m = c.api->get_num_rows(p);
    if(c.fires == 1 && why == R_ROWGEN) { c.rows_after = c.api->get_num_rows(p) - c.orig_m; c.api->ios_terminate(t); return; }
    if(why != R_CUTGEN) return;
    if(c.fires++ == 0) {
        const int ind[4] = {0, 1, 2, 3};
        const double val[4] = {0, 1, 1, 1};
        const double a = c.api->get_col_prim(p, 1) + c.api->get_col_prim(p, 2) + c.api->get_col_prim(p, 3);
        if(c.pool) c.n->ios_add_row(t, nullptr, 0, 0, 3, ind, val, v1::GLP_UP, a - c.v);
        else { const int i = c.api->add_rows(p, 1); c.api->set_mat_row(p, i, 3, ind, val); c.api->set_row_bnds(p, i, v1::GLP_UP, a - c.v, a - c.v); }
    }
}
struct par_ctx { const v1::glpk_api * api; natives * n; double second; int fires = 0; int orig_m = -1; int rows_after = -1; };
static void par_hook(glp_tree * t, void * u) {
    auto & c = *static_cast<par_ctx *>(u);
    const int why = c.api->ios_reason(t);
    if(why != R_CUTGEN && why != R_ROWGEN) return;
    glp_prob * p = c.api->ios_get_prob(t);
    if(c.orig_m < 0) c.orig_m = c.api->get_num_rows(p);
    if(c.fires == 1 && why == R_ROWGEN) { c.rows_after = c.api->get_num_rows(p) - c.orig_m; c.api->ios_terminate(t); return; }
    if(why != R_CUTGEN || c.fires++) return;
    // same terms, or one more term: cosine 1, or 3/sqrt(12) = 0.866
    int ind[5] = {0, 1, 2, 3, 5};
    double val[5] = {0, 1, 1, 1, 1};
    c.n->ios_add_row(t, nullptr, 0, 0, 3, ind, val, v1::GLP_UP, 1.3);
    c.n->ios_add_row(t, nullptr, 0, 0, c.second > 0 ? 3 : 4, ind, val, v1::GLP_UP, 1.2);
}
static void parallel_cuts() {
    for(const double same : {1.0, -1.0}) {
        M m; scope_settings(m); (void)odd_cycle<int>(m, 21); TL(m, 5);
        par_ctx c{&m.native_api(), &natives::of(m), same};
        m.hook = par_hook; m.hook_info = &c;
        (void)cp::timed_solve(m);
        std::println(stderr, "== PARALLEL pool, two violated cuts offered, {}: rows kept at the next LP = {}", same > 0 ? "same terms (cosine 1)" : "one extra term (cosine 0.866)", c.rows_after);
    }
}
static void thresh() {
    parallel_cuts();
    for(const bool pool : {true, false}) for(const double v : {1e-6, 1e-4, 1e-3, 5e-3, 0.01, 0.015, 0.02, 0.05}) {
        M m; scope_settings(m); (void)odd_cycle<int>(m, 21); TL(m, 5);
        thresh_ctx c{&m.native_api(), &natives::of(m), pool, v};
        m.hook = thresh_hook; m.hook_info = &c;
        (void)cp::timed_solve(m);
        std::println(stderr, "== THRESH {} violation {:g} (norm sqrt 3): rows kept at the next LP = {}", pool ? "pool" : "rows_icutgen", v, c.rows_after);
    }
}
// how many rounds at one node? A fresh violated cut per call, on the objective's terms.
struct rounds_ctx { const v1::glpk_api * api; natives * n; std::vector<int> ind; std::vector<double> val; long at_root = 0, cap = 2000; };
static void rounds_hook(glp_tree * t, void * u) {
    auto & c = *static_cast<rounds_ctx *>(u);
    if(c.api->ios_reason(t) != R_CUTGEN) return;
    if(c.n->node_level(t, c.n->curr_node(t)) > 0) { c.api->ios_terminate(t); return; }
    if(++c.at_root >= c.cap) { c.api->ios_terminate(t); return; }
    glp_prob * p = c.api->ios_get_prob(t);
    double obj = 0;
    for(std::size_t k = 1; k < c.ind.size(); ++k) obj += c.api->get_col_prim(p, c.ind[k]);
    c.n->ios_add_row(t, nullptr, 0, 0, static_cast<int>(c.ind.size()) - 1, c.ind.data(), c.val.data(), v1::GLP_UP, obj - 0.02);
}
static void rounds() {
    M m; scope_settings(m); auto g = cp::build_gadgets(m, 10, false); TL(m, 20);
    rounds_ctx c{&m.native_api(), &natives::of(m)};
    c.ind.push_back(0); c.val.push_back(0.0);
    for(int j : g.z) { c.ind.push_back(j); c.val.push_back(1.0); }
    m.hook = rounds_hook; m.hook_info = &c;
    const double t = cp::timed_solve(m);
    std::println(stderr, "== ROUNDS pool, one fresh violated cut per ICUTGEN at the root: rounds at the root={} (cap {}), rows left on P={}; {} {}", c.at_root, c.cap, m.native_api().get_num_rows(m.native_model()), cp::summary(m, t), tail(m));
}
static void filter_probe() {
    for(const bool pool : {true, false}) {
        M m; scope_settings(m); (void)odd_cycle<int>(m, 61); TL(m, 20);
        filter_ctx c{&m.native_api(), &natives::of(m), pool};
        m.hook = filter_hook; m.hook_info = &c;
        std::println(stderr, "== FILTER {}", pool ? "pool" : "rows_icutgen");
        const double t = cp::timed_solve(m);
        std::println(stderr, "   {} {}", cp::summary(m, t), tail(m));
    }
}
struct purge_ctx { const v1::glpk_api * api; natives * n; bool pool; int want; int orig_m = -1; int round = 0; long fires = 0; };
static void purge_hook(glp_tree * t, void * u) {
    auto & c = *static_cast<purge_ctx *>(u);
    const int why = c.api->ios_reason(t);
    if(why != R_ROWGEN && why != R_CUTGEN) return;
    glp_prob * p = c.api->ios_get_prob(t);
    if(c.orig_m < 0) c.orig_m = c.api->get_num_rows(p);
    const int cur = c.n->curr_node(t), lvl = c.n->node_level(t, cur);
    const int m = c.api->get_num_rows(p);
    std::string rows;
    for(int i = c.orig_m + 1; i <= m; ++i) {
        attr_t a{};
        c.n->row_attr(t, i, &a);
        rows += std::format(" [row {} origin={} level={} ub={} {}]", i, a.origin == RF_LAZY ? "LAZY" : a.origin == RF_CUT ? "CUT" : "REG", a.level, c.api->get_row_ub(p, i), c.api->get_row_stat(p, i) == v1::GLP_BS ? "basic" : "nonbasic");
    }
    if(++c.fires <= 12) std::println(stderr, "   {} node={} level={} x0+x1+x2={:.3f} rows beyond the model: {}{}", why == R_ROWGEN ? "IROWGEN" : "ICUTGEN", cur, lvl, c.api->get_col_prim(p, 1) + c.api->get_col_prim(p, 2) + c.api->get_col_prim(p, 3), m - c.orig_m, rows);
    if(why != c.want || c.round >= 2 || lvl > 0) return;
    // first a weak cut, then a stronger one on the same terms: the weak one goes basic
    const double rhs = c.round == 0 ? 1.4 : 1.2;
    ++c.round;
    const int ind[4] = {0, 1, 2, 3};
    const double val[4] = {0, 1, 1, 1};
    if(c.pool) c.n->ios_add_row(t, nullptr, 0, 0, 3, ind, val, v1::GLP_UP, rhs);
    else { const int i = c.api->add_rows(p, 1); c.api->set_mat_row(p, i, 3, ind, val); c.api->set_row_bnds(p, i, v1::GLP_UP, rhs, rhs); }
}
static void purge() {
    for(const auto & [name, pool, want] : {std::tuple{"pool", true, int(R_CUTGEN)}, {"rows_icutgen", false, int(R_CUTGEN)}, {"rows_irowgen", false, int(R_ROWGEN)}}) {
        M m; scope_settings(m); (void)odd_cycle<int>(m, 21); TL(m, 20);
        purge_ctx c{&m.native_api(), &natives::of(m), pool, want};
        m.hook = purge_hook; m.hook_info = &c;
        std::println(stderr, "== PURGE {}", name);
        const double t = cp::timed_solve(m);
        std::println(stderr, "   {} {}", cp::summary(m, t), tail(m));
    }
}

// ---------------------------------------------------- odd cuts, native, unguarded
static void weird(const std::string & mode, const std::string & name) {
    if(name == "zerocoef") g_strip_zeros = false;
    M m; scope_settings(m); auto g = cp::build_gadgets(m, 10, false); TL(m, 5);
    auto s = std::make_shared<cp::stats>();
    cp::raw_cut cut;
    bool every = false;
    if(name == "equality") cut = {{g.dummy}, {1.0}, 'E', 1.0};
    else if(name == "noterms") cut = {{}, {}, 'L', 1.0};
    else if(name == "noterms_infeasible") cut = {{}, {}, 'L', -1.0};
    else if(name == "zerocoef") { cut = g.hull_cut(3); cut.ind.push_back(g.za[1]); cut.val.push_back(0.0); }
    else if(name == "dupcol") { cut = g.hull_cut(3); cut.val[0] = 0.5; cut.ind.push_back(cut.ind[0]); cut.val.push_back(0.5); }
    else if(name == "outofrange") { cut = g.hull_cut(3); cut.ind.push_back(1000); cut.val.push_back(1.0); }
    else if(name == "index0") { cut = g.hull_cut(3); cut.ind.push_back(0); cut.val.push_back(1.0); }
    else if(name == "notviolated") cut = {{g.za[3]}, {1.0}, 'L', 1e6};
    else if(name == "samecut") { cut = g.hull_cut(3); every = true; }
    else if(name == "empties") cut = {{g.za[3]}, {1.0}, 'L', -1.0};
    else { std::println(stderr, "unknown odd cut {}", name); std::exit(2); }
    bool once = false;
    attach(m, cp::watched([&](const auto &, const cp::add_fn & add) { if(every || !once) { once = true; add(cut); } }, s, [&g](const auto & x) { return g.objective(x); }), mode);
    std::println(stderr, "== W {} {} starts", mode, name);
    const double t = cp::timed_solve(m);
    std::println(stderr, "== W {} {}: {} (known opt {}) {}\n   {}", mode, name, cp::summary(m, t), g.opt, tail(m), s->line());
}
// which calls are allowed at which reason? Each abort is reported by GLPK itself.
struct forbid_ctx { const v1::glpk_api * api; natives * n; int reason; bool pool; int done = 0; };
static void forbid_hook(glp_tree * t, void * u) {
    auto & c = *static_cast<forbid_ctx *>(u);
    if(c.done || c.api->ios_reason(t) != c.reason) return;
    c.done = 1;
    glp_prob * p = c.api->ios_get_prob(t);
    const int ind[3] = {0, 1, 2};
    const double val[3] = {0, 1, 1};
    std::println(stderr, "   calling {} at reason {}", c.pool ? "glp_ios_add_row" : "glp_add_rows", c.reason);
    if(c.pool) c.n->ios_add_row(t, nullptr, 0, 0, 2, ind, val, v1::GLP_UP, 1e6);
    else { const int i = c.api->add_rows(p, 1); c.api->set_mat_row(p, i, 2, ind, val); c.api->set_row_bnds(p, i, v1::GLP_UP, 1e6, 1e6); }
    std::println(stderr, "   returned");
}
static void forbidden(const int reason, const bool pool) {
    M m; scope_settings(m); (void)cp::build_gadgets(m, 10, false); TL(m, 5);
    forbid_ctx c{&m.native_api(), &natives::of(m), reason, pool};
    m.hook = forbid_hook; m.hook_info = &c;
    const double t = cp::timed_solve(m);
    std::println(stderr, "== FORBID reason={} {}: fired={} {}", reason, pool ? "glp_ios_add_row" : "glp_add_rows", c.done, cp::summary(m, t));
}

// ------------------------------------------------- root LP paths with presolve off
static void root_cases(const int which) {
    using std::chrono::duration;
    const auto show = [](const char * name, M & m) {
        std::println(stderr, "== ROOT {} starts", name);
        const double t = cp::timed_solve(m);
        std::println(stderr, "== ROOT {}: {} ret=0x{:x} glp_get_status={} glp_mip_status={}", name, cp::status_name(m.get_status()), m.native_ret, m.native_api().get_status(m.native_model()), m.native_api().mip_status(m.native_model()));
        (void)t;
    };
    const auto X = std::size_t{0}, Y = std::size_t{1};
    if(which == 0) { M m; m.model_params.presolve = 0; auto x = m.add_binary_variables(2); m.set_maximization(); m.set_objective(x(X) + x(Y)); m.add_constraint(x(X) + x(Y) >= 5); show("LP infeasible", m); }
    if(which == 1) { M m; m.model_params.presolve = 0; auto x = m.add_integer_variables(1); m.set_maximization(); m.set_objective(1.0 * x(X)); show("LP unbounded", m); }
    if(which == 2) { M m; m.model_params.presolve = 0; auto x = m.add_integer_variables(1); m.set_maximization(); m.set_objective(1.0 * x(X)); m.add_constraint(x(X) <= 3); m.add_constraint(x(X) >= 5); show("rows without a common point", m); }
    if(which == 3) { M m; m.model_params.presolve = 0; auto x = m.add_binary_variables(1); m.set_variable_lower_bound(x(X), 2.0); m.set_variable_upper_bound(x(X), 1.0); show("crossed column bounds", m); }
    if(which == 4) { M m; m.model_params.presolve = 0; show("empty model, scale and adv_basis as the presolve path does", m); }
    if(which == 5) { M m; m.model_params.presolve = 0; m.scale = false; show("empty model, no scaling", m); }
    if(which == 6) { M m; show("empty model, presolve on", m); }
    if(which == 7) { M m; m.model_params.presolve = 0; auto x = m.add_binary_variables(2); m.set_maximization(); m.set_objective(x(X) + 2.0 * x(Y)); show("no rows", m); std::println(stderr, "   value={}", m.get_solution_value()); }
    if(which == 8) { // a stale incumbent status when the second root LP fails
        M m; m.model_params.presolve = 0; auto x = m.add_binary_variables(2); m.set_maximization(); m.set_objective(x(X) + x(Y)); auto c = m.add_constraint(x(X) + x(Y) >= 1);
        show("feasible first", m);
        m.set_constraint_lower_bound(c, 5.0);
        show("then infeasible root: glp_intopt never runs, mip status is the previous solve's", m);
    }
    if(which == 9) { M m; m.model_params.presolve = 0; cp::build_mkp(m, 30, 100); TL(m, 0.0); show("time limit 0", m); }
    if(which == 10) { // the same with a real re-solve after a stopped root
        M m; m.model_params.presolve = 0; auto x = m.add_binary_variables(2); m.set_maximization(); m.set_objective(x(X) + x(Y)); (void)m.add_constraint(x(X) + x(Y) <= 1);
        show("first", m); TL(m, 0.0); show("time limit 0 on the re-solve: stale incumbent reported as a solution?", m);
    }
}

// presolve left on: which column of P is which column of the tree problem?
struct cols_ctx { const v1::glpk_api * api; natives * n; glp_prob * P; bool printed = false; };
static void cols_hook(glp_tree * t, void * u) {
    auto & c = *static_cast<cols_ctx *>(u);
    if(c.api->ios_reason(t) != R_ROWGEN || c.printed) return;
    c.printed = true;
    glp_prob * p = c.api->ios_get_prob(t);
    const auto show = [&](const char * name, glp_prob * q) {
        std::string line;
        for(int j = 1; j <= c.api->get_num_cols(q); ++j)
            line += std::format(" {}:{}[{:g},{:g}]c{:g}", j, c.api->get_col_kind(q, j) == v1::GLP_CV ? "C" : "I", c.api->get_col_lb(q, j), c.api->get_col_ub(q, j), c.api->get_obj_coef(q, j));
        std::println(stderr, "   {}:{}", name, line);
    };
    show("P   ", c.P);
    show("tree", p);
}
static void cols() {
    M m; scope_settings(m); m.model_params.presolve = 1;
    (void)cp::build_gadgets(m, 10, false); TL(m, 10);
    cols_ctx c{&m.native_api(), &natives::of(m), m.native_model()};
    m.hook = cols_hook; m.hook_info = &c;
    const double t = cp::timed_solve(m);
    std::println(stderr, "== COLS {} {}", cp::summary(m, t), tail(m));
}

// ------------------------------------------------- lazy constraints and user cuts
template <class M_>
static cp::atsp build_dfj(M_ & m, const int n, const unsigned seed) {
    cp::atsp a;
    a.n = n;
    const auto N = static_cast<std::size_t>(n);
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> P(0, 100), noise(1.0, 1.3);
    std::vector<std::pair<double, double>> pts(N);
    for(auto & p : pts) p = {P(rng), P(rng)};
    a.d.assign(N, std::vector<double>(N, 0));
    for(std::size_t i = 0; i < N; ++i) for(std::size_t j = 0; j < N; ++j) if(i != j)
        a.d[i][j] = std::round(std::hypot(pts[i].first - pts[j].first, pts[i].second - pts[j].second) * noise(rng));
    const auto arcs = N * (N - 1);
    const auto id = [N](std::size_t i, std::size_t j) { return i * (N - 1) + (j < i ? j : j - 1); };
    auto x = m.add_binary_variables(arcs);
    a.col.assign(N, std::vector<int>(N, -1));
    a.comp.assign(N, std::vector<int>(N, -1));
    for(std::size_t i = 0; i < N; ++i) for(std::size_t j = 0; j < N; ++j) if(i != j) a.col[i][j] = m.native_id(x(id(i, j)));
    m.set_minimization();
    const auto nodes = std::views::iota(std::size_t{0}, N);
    m.set_objective(xsum(nodes, [&](std::size_t i) {
        return xsum(nodes | std::views::filter([i](std::size_t j) { return j != i; }), [&, i](std::size_t j) { return a.d[i][j] * x(id(i, j)); });
    }));
    for(std::size_t i : nodes) {
        m.add_constraint(xsum(nodes | std::views::filter([i](std::size_t j) { return j != i; }), [&, i](std::size_t j) { return 1.0 * x(id(i, j)); }) == 1);
        m.add_constraint(xsum(nodes | std::views::filter([i](std::size_t j) { return j != i; }), [&, i](std::size_t j) { return 1.0 * x(id(j, i)); }) == 1);
    }
    return a;
}
struct lazy_ctx {
    const v1::glpk_api * api; natives * n; const cp::atsp * a;
    int lazy_reason;      // R_ROWGEN
    bool user_pool, user_rows, user_at_rowgen;  // fractional points
    long lazy_rows = 0, user_cuts = 0, int_calls = 0, frac_calls = 0, cut_calls = 0, bingos = 0, bingo_subtour = 0;
    std::map<std::vector<int>, long> seen;
    long repeats = 0;
    std::vector<int> ind; std::vector<double> val, x;
};
static bool has_subtour(const cp::atsp & a, const std::vector<double> & x) {
    int count = 0, v = 0;
    do {
        int next = -1;
        for(int j = 0; j < a.n; ++j) if(j != v && a.X(x, v, j) > 0.5) { next = j; break; }
        if(next < 0) return true;
        v = next; ++count;
    } while(v != 0 && count <= a.n);
    return count != a.n;
}
static void lazy_add(lazy_ctx & c, glp_tree * t, glp_prob * p, const cp::raw_cut & r, const bool pool) {
    const auto len = r.ind.size();
    c.ind.assign(len + 1, 0); c.val.assign(len + 1, 0.0);
    for(std::size_t k = 0; k < len; ++k) { c.ind[k + 1] = r.ind[k]; c.val[k + 1] = r.val[k]; }
    auto & times = c.seen[r.ind];
    if(times++) ++c.repeats;
    if(pool) c.n->ios_add_row(t, nullptr, 0, 0, static_cast<int>(len), c.ind.data(), c.val.data(), v1::GLP_LO, r.rhs);
    else { const int i = c.api->add_rows(p, 1); c.api->set_mat_row(p, i, static_cast<int>(len), c.ind.data(), c.val.data()); c.api->set_row_bnds(p, i, v1::GLP_LO, r.rhs, r.rhs); }
}
static void lazy_hook(glp_tree * t, void * u) {
    auto & c = *static_cast<lazy_ctx *>(u);
    const int why = c.api->ios_reason(t);
    glp_prob * p = c.api->ios_get_prob(t);
    const int n = c.api->get_num_cols(p);
    if(why == R_BINGO) {
        ++c.bingos;
        c.x.assign(static_cast<std::size_t>(n) + 1, 0.0);
        for(int j = 1; j <= n; ++j) c.x[static_cast<std::size_t>(j)] = c.api->mip_col_val(p, j);
        c.bingo_subtour += has_subtour(*c.a, c.x);
        return;
    }
    if(why != R_ROWGEN && why != R_CUTGEN) return;
    c.x.assign(static_cast<std::size_t>(n) + 1, 0.0);
    for(int j = 1; j <= n; ++j) c.x[static_cast<std::size_t>(j)] = c.api->get_col_prim(p, j);
    if(why == R_ROWGEN) {
        const bool integral = c.a->integral(c.x);
        (integral ? c.int_calls : c.frac_calls)++;
        if(!integral && !c.user_at_rowgen) return;
        for(const auto & r : c.a->separate(c.x, integral ? 20u : 5u)) { lazy_add(c, t, p, r, false); integral ? ++c.lazy_rows : ++c.user_cuts; }
    } else {
        ++c.cut_calls;
        if(!c.user_pool && !c.user_rows) return;
        for(const auto & r : c.a->separate(c.x, 5u)) { lazy_add(c, t, p, r, c.user_pool); ++c.user_cuts; }
    }
}
static void lazy(const int n, const unsigned seed, const std::string & config) {
    // MTZ reference, complete model, wrapper defaults
    double ref = NAN;
    if(config == "reference" || std::getenv("CUT_LAZY_REF")) {
        M m; auto a = cp::build_atsp(m, n, false, seed); TL(m, 120);
        const double t = cp::timed_solve(m);
        ref = m.get_solution_value();
        std::println(stderr, "== LAZY n={} seed={} reference MTZ, wrapper defaults: {} {}", n, seed, cp::summary(m, t), tail(m));
        if(config == "reference") return;
    }
    M m;
    scope_settings(m);
    auto a = build_dfj(m, n, seed);
    TL(m, 60);
    lazy_ctx c{&m.native_api(), &natives::of(m), &a, R_ROWGEN, false, false, false};
    auto & q = m.model_params;
    if(config == "lazy_only") {}
    else if(config == "lazy_user_pool") c.user_pool = true;
    else if(config == "lazy_user_rows") c.user_rows = true;
    else if(config == "rowgen_both") c.user_at_rowgen = true;
    else if(config == "lazy_only+sr") q.sr_heur = v1::GLP_ON;
    else if(config == "lazy_only+fp") q.fp_heur = v1::GLP_ON;
    else if(config == "lazy_only+ps") q.ps_heur = v1::GLP_ON;
    else if(config == "lazy_only+cuts") q.mir_cuts = q.gmi_cuts = q.cov_cuts = q.clq_cuts = v1::GLP_ON;
    else if(config == "lazy_only+pp") q.pp_tech = PP_ROOT;
    else if(config == "lazy_user_pool+wrapper_defaults") {
        c.user_pool = true;
        q.pp_tech = PP_ROOT; q.mip_gap = 1e-4;
        q.mir_cuts = q.gmi_cuts = q.cov_cuts = q.clq_cuts = v1::GLP_ON;
        q.fp_heur = q.sr_heur = v1::GLP_ON;
    } else { std::println(stderr, "unknown config"); std::exit(2); }
    m.hook = lazy_hook; m.hook_info = &c;
    const double t = cp::timed_solve(m);
    std::string tour = "none";
    bool sol = false;
    std::visit([&](const auto & st) { if constexpr(requires { st.solution_available; }) sol = bool(st.solution_available); }, m.get_status());
    std::vector<double> x(static_cast<std::size_t>(m.native_api().get_num_cols(m.native_model())) + 1, 0.0);
    if(sol) {
        for(std::size_t j = 1; j < x.size(); ++j) x[j] = m.native_api().mip_col_val(m.native_model(), static_cast<int>(j));
        tour = has_subtour(a, x) ? "HAS A SUBTOUR" : "one tour";
    }
    std::println(stderr, "== LAZY n={} seed={} {}: {} {} rows_left={}; IROWGEN integer={} fractional={} ICUTGEN={} IBINGO={} (with subtour {}); lazy rows={} user cuts={} repeated rows={}; final: {}{}",
                 n, seed, config, cp::summary(m, t), tail(m), m.native_api().get_num_rows(m.native_model()), c.int_calls, c.frac_calls, c.cut_calls, c.bingos, c.bingo_subtour, c.lazy_rows, c.user_cuts, c.repeats, tour,
                 std::isnan(ref) ? std::string() : std::format(" {} reference {}", std::fabs(m.get_solution_value() - ref) < 1e-6 ? "MATCHES" : "DIFFERS FROM", ref));
}

// ------------------------- which built-in generator survives direct rows at ICUTGEN?
static void generators(const std::string & gen, const std::string & mode, const int n) {
    g_keep_gmi = true;
    M m;
    auto & q = m.model_params;
    q.gmi_cuts = gen == "gmi" || gen == "all" ? v1::GLP_ON : v1::GLP_OFF;
    q.mir_cuts = gen == "mir" || gen == "all" ? v1::GLP_ON : v1::GLP_OFF;
    q.cov_cuts = gen == "cov" || gen == "all" ? v1::GLP_ON : v1::GLP_OFF;
    q.clq_cuts = gen == "clq" || gen == "all" ? v1::GLP_ON : v1::GLP_OFF;
    auto a = cp::build_atsp(m, n);
    TL(m, 30);
    auto s = std::make_shared<cp::stats>();
    attach(m, cp::watched([&a](const auto & x, const cp::add_fn & add) { for(const auto & c : a.separate(x)) add(c); }, s,
        [&a](const auto & x) { return a.objective(x); }, [&a](const auto & x) { return a.point_ok(x); }, [&a](const auto & x) { return a.integral(x); }), mode);
    std::println(stderr, "== GEN {} mode {} n={} starts", gen, mode, n);
    const double t = cp::timed_solve(m);
    std::println(stderr, "== GEN {} mode {} n={}: {} {}\n   {}", gen, mode, n, cp::summary(m, t), tail(m), s->line());
}

// ---------------------- lazy rows against GLPK's heuristics, which never pass IROWGEN
// Multidimensional knapsack, rows beyond the first enforced lazily at integer points.
struct mkp_data { std::vector<std::vector<double>> a; std::vector<double> b, c; };
static mkp_data make_mkp(const int rows, const int cols, const unsigned seed) {
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> A(1, 1000), R(0, 500);
    const auto C = static_cast<std::size_t>(cols);
    mkp_data d;
    d.a.assign(static_cast<std::size_t>(rows), std::vector<double>(C));
    for(auto & row : d.a) for(auto & v : row) v = A(rng);
    d.c.resize(C);
    for(std::size_t j = 0; j < C; ++j) {
        double sum = 0;
        for(const auto & row : d.a) sum += row[j];
        d.c[j] = std::floor(sum / rows + R(rng));
    }
    for(const auto & row : d.a) { double sum = 0; for(double v : row) sum += v; d.b.push_back(std::floor(sum / 2)); }
    return d;
}
struct lmkp_ctx {
    const v1::glpk_api * api; const mkp_data * d; std::vector<int> col;
    long rowgen = 0, added = 0, int_calls = 0, bingo = 0, heur_calls = 0;
    std::vector<int> ind; std::vector<double> val;
    bool only_integer = true;
    bool reinject = false;  // a list kept by the wrapper: rows found at integer points, re-applied at every LP
    std::set<std::size_t> known;
};
static void lmkp_hook(glp_tree * t, void * u) {
    auto & c = *static_cast<lmkp_ctx *>(u);
    const int why = c.api->ios_reason(t);
    if(why == R_BINGO) ++c.bingo;
    if(why == R_HEUR) ++c.heur_calls;
    if(why != R_ROWGEN) return;
    ++c.rowgen;
    glp_prob * p = c.api->ios_get_prob(t);
    const auto C = c.col.size();
    std::vector<double> x(C);
    bool integral = true;
    for(std::size_t j = 0; j < C; ++j) { x[j] = c.api->get_col_prim(p, c.col[j]); integral = integral && std::fabs(x[j] - std::round(x[j])) <= 1e-6; }
    if(integral) ++c.int_calls;
    if(c.only_integer && !integral && !c.reinject) return;
    for(std::size_t r = 1; r < c.d->a.size(); ++r) {
        if(c.reinject && !integral && !c.known.count(r)) continue;
        double act = 0;
        for(std::size_t j = 0; j < C; ++j) act += c.d->a[r][j] * x[j];
        if(act <= c.d->b[r] + 1e-6) continue;
        c.known.insert(r);
        c.ind.assign(C + 1, 0); c.val.assign(C + 1, 0.0);
        for(std::size_t j = 0; j < C; ++j) { c.ind[j + 1] = c.col[j]; c.val[j + 1] = c.d->a[r][j]; }
        const int i = c.api->add_rows(p, 1);
        c.api->set_mat_row(p, i, static_cast<int>(C), c.ind.data(), c.val.data());
        c.api->set_row_bnds(p, i, v1::GLP_UP, c.d->b[r], c.d->b[r]);
        ++c.added;
    }
}
static void lazy_mkp(const int rows, const int cols, const unsigned seed, const std::string & config) {
    const auto d = make_mkp(rows, cols, seed);
    const auto C = static_cast<std::size_t>(cols);
    const auto ids = std::views::iota(std::size_t{0}, C);
    double ref = NAN;
    {
        M m; auto x = m.add_binary_variables(C);
        m.set_maximization(); m.set_objective(xsum(ids, [&](std::size_t j) { return d.c[j] * x(j); }));
        for(std::size_t r = 0; r < d.a.size(); ++r) m.add_constraint(xsum(ids, [&](std::size_t j) { return d.a[r][j] * x(j); }) <= d.b[r]);
        m.model_params.mip_gap = 0.0; TL(m, 60);
        (void)cp::timed_solve(m);
        ref = m.get_solution_value();
        std::println(stderr, "== LMKP {}x{} seed={} complete model, wrapper defaults, gap 0: {} value={} {}", rows, cols, seed, cp::status_name(m.get_status()), ref, tail(m));
    }
    M m; scope_settings(m);
    auto x = m.add_binary_variables(C);
    m.set_maximization(); m.set_objective(xsum(ids, [&](std::size_t j) { return d.c[j] * x(j); }));
    m.add_constraint(xsum(ids, [&](std::size_t j) { return d.a[0][j] * x(j); }) <= d.b[0]);
    TL(m, 60);
    lmkp_ctx c{&m.native_api(), &d};
    for(std::size_t j = 0; j < C; ++j) c.col.push_back(m.native_id(x(j)));
    auto & q = m.model_params;
    if(config.find("+sr") != std::string::npos) q.sr_heur = v1::GLP_ON;
    if(config.find("+fp") != std::string::npos) q.fp_heur = v1::GLP_ON;
    if(config.find("+ps") != std::string::npos) q.ps_heur = v1::GLP_ON;
    if(config.find("all_points") != std::string::npos) c.only_integer = false;
    if(config.find("reinject") != std::string::npos) c.reinject = true;
    m.hook = lmkp_hook; m.hook_info = &c;
    const double t = cp::timed_solve(m);
    long bad = 0; double worst = 0;
    for(std::size_t r = 0; r < d.a.size(); ++r) {
        double act = 0;
        for(std::size_t j = 0; j < C; ++j) act += d.a[r][j] * m.native_api().mip_col_val(m.native_model(), c.col[j]);
        if(act > d.b[r] + 1e-6) { ++bad; worst = std::max(worst, act - d.b[r]); }
    }
    const double v = m.get_solution_value();
    std::println(stderr, "== LMKP {}x{} seed={} {}: {} {} IROWGEN={} integer={} IBINGO={} lazy rows={}; final solution violates {} lazy rows (worst {:g}); reference {} => {}", rows, cols, seed, config, cp::summary(m, t), tail(m), c.rowgen, c.int_calls, c.bingo, c.added, bad, worst, ref,
                 bad ? "WRONG: an incumbent that breaks lazy rows" : std::fabs(v - ref) < 1e-6 ? "MATCHES" : "DIFFERS");
}

// ---------- the public preprocessor with names kept: is that a column map?
template <class Build>
static void npp_on(const char * name, Build build) {
    M m; build(m);
    const auto & api = m.native_api();
    glp_prob * P = m.native_model();
    const auto l = api.library_path();
    using prep = void;
    auto alloc = cp::symf<prep * (*)()>(l, "glp_npp_alloc_wksp");
    auto load = cp::symf<void (*)(prep *, glp_prob *, int, int)>(l, "glp_npp_load_prob");
    auto pre1 = cp::symf<int (*)(prep *, int)>(l, "glp_npp_preprocess1");
    auto build_q = cp::symf<void (*)(prep *, glp_prob *)>(l, "glp_npp_build_prob");
    auto freew = cp::symf<void (*)(prep *)>(l, "glp_npp_free_wksp");
    const int cols = api.get_num_cols(P);
    std::map<std::string, int> by_name;
    for(int j = 1; j <= cols; ++j) { const auto nm = std::format("c{}", j); api.set_col_name(P, j, nm.c_str()); by_name[nm] = j; }
    prep * w = alloc();
    load(w, P, 3 /* GLP_MIP */, 1 /* names on */);
    const int r = pre1(w, 0);
    glp_prob * Q = api.create_prob();
    if(r == 0) build_q(w, Q);
    const int qc = api.get_num_cols(Q);
    int named = 0, same_index = 0, bounds_same = 0, obj_same = 0;
    for(int j = 1; j <= qc; ++j) {
        const char * nm = api.get_col_name(Q, j);
        if(!nm) continue;
        ++named;
        const auto it = by_name.find(nm);
        if(it == by_name.end()) continue;
        same_index += it->second == j;
        bounds_same += api.get_col_lb(Q, j) == api.get_col_lb(P, it->second) && api.get_col_ub(Q, j) == api.get_col_ub(P, it->second);
        obj_same += api.get_obj_coef(Q, j) == api.get_obj_coef(P, it->second);
    }
    std::println(stderr, "== NPP {} preprocess1 ret={}: P rows={} cols={}; Q rows={} cols={}; Q columns with a name={} at P's index={} same bounds={} same objective coefficient={}", name, r, api.get_num_rows(P), cols, api.get_num_rows(Q), qc, named, same_index, bounds_same, obj_same);
    api.delete_prob(Q);
    freew(w);
}
// End to end: preprocess with names kept, search Q with the presolver off, cuts mapped by name, solution postsolved to P.
struct e2e_ctx { const v1::glpk_api * api; natives * n; std::vector<int> map_to_q; cp::raw_cut cut; bool once = false; long unmapped = 0; long fires = 0; };
static void e2e_hook(glp_tree * t, void * u) {
    auto & c = *static_cast<e2e_ctx *>(u);
    if(c.api->ios_reason(t) != R_CUTGEN) return;
    ++c.fires;
    if(c.once) return;
    c.once = true;
    std::vector<int> ind{0}; std::vector<double> val{0.0};
    for(std::size_t k = 0; k < c.cut.ind.size(); ++k) {
        const int q = c.map_to_q[static_cast<std::size_t>(c.cut.ind[k])];
        if(q == 0) { ++c.unmapped; continue; }
        ind.push_back(q); val.push_back(c.cut.val[k]);
    }
    c.n->ios_add_row(t, nullptr, 0, 0, static_cast<int>(ind.size()) - 1, ind.data(), val.data(), c.cut.sense == 'L' ? v1::GLP_UP : v1::GLP_LO, c.cut.rhs);
}
static void npp_e2e() {
    for(const char * which : {"valid cut on shifted columns", "invalid cut, must bind", "no cut", "cut touching the eliminated column"}) for(const bool flip : {false, true}) {
        M m; scope_settings(m); auto g = cp::build_gadgets(m, 10, flip);
        const auto & api = m.native_api();
        glp_prob * P = m.native_model();
        const auto l = api.library_path();
        using prep = void;
        auto alloc = cp::symf<prep * (*)()>(l, "glp_npp_alloc_wksp");
        auto load = cp::symf<void (*)(prep *, glp_prob *, int, int)>(l, "glp_npp_load_prob");
        auto pre1 = cp::symf<int (*)(prep *, int)>(l, "glp_npp_preprocess1");
        auto build_q = cp::symf<void (*)(prep *, glp_prob *)>(l, "glp_npp_build_prob");
        auto post = cp::symf<void (*)(prep *, glp_prob *)>(l, "glp_npp_postprocess");
        auto obtain = cp::symf<void (*)(prep *, glp_prob *)>(l, "glp_npp_obtain_sol");
        auto freew = cp::symf<void (*)(prep *)>(l, "glp_npp_free_wksp");
        auto & n = natives::of(m);
        const int term = api.term_out(v1::GLP_OFF);
        const int cols = api.get_num_cols(P);
        std::map<std::string, int> by_name;
        for(int j = 1; j <= cols; ++j) { const auto nm = std::format("c{}", j); api.set_col_name(P, j, nm.c_str()); by_name[nm] = j; }
        prep * w = alloc();
        load(w, P, 3, 1);
        const int r = pre1(w, 0);
        glp_prob * Q = api.create_prob();
        build_q(w, Q);
        e2e_ctx c{&api, &n};
        c.map_to_q.assign(static_cast<std::size_t>(cols) + 1, 0);
        for(int j = 1; j <= api.get_num_cols(Q); ++j) c.map_to_q[static_cast<std::size_t>(by_name.at(api.get_col_name(Q, j)))] = j;
        const std::string how = which;
        if(how == "valid cut on shifted columns") c.cut = cp::raw_cut{{g.za[12], g.dummy + 3}, {1.0, 6.0}, 'L', 9.0};
        else if(how == "invalid cut, must bind") c.cut = g.invalid_global_cut();
        else if(how == "cut touching the eliminated column") c.cut = cp::raw_cut{{g.za[12], g.dummy + 3, g.dummy}, {1.0, 6.0, 1.0}, 'L', 10.0};
        else c.once = true;
        n.scale_prob(Q, SF_PATH);
        n.adv_basis(Q, 0);
        v1::glp_smcp sm; api.init_smcp(&sm); sm.msg_lev = v1::GLP_MSG_OFF;
        const int sret = api.simplex(Q, &sm);
        v1::glp_iocp q = m.model_params;
        q.msg_lev = v1::GLP_MSG_OFF; q.presolve = v1::GLP_OFF; q.tm_lim = 20000; q.cb_func = e2e_hook; q.cb_info = &c;
        const int ret = api.intopt(Q, &q);
        const int qstat = api.mip_status(Q);
        double value = NAN;
        if(qstat == v1::GLP_OPT || qstat == v1::GLP_FEAS) { post(w, Q); obtain(w, P); value = api.mip_obj_val(P); }
        std::println(stderr, "== NPP-E2E {} flip={}: preprocess={} simplex={} intopt=0x{:x} Q cols={} (P {}) fires={} terms lost to the elimination={} value on P={:.6g} known opt {:.6g} P mip status={} => {}", which, flip, r, sret, ret, api.get_num_cols(Q), cols, c.fires, c.unmapped, value, g.opt, api.mip_status(P),
                     std::fabs(value - g.opt) < 1e-4 ? "optimum KEPT" : "optimum LOST");
        api.delete_prob(Q);
        freew(w);
        api.term_out(term);
    }
}
static void npp_probe() {
    npp_on("gadgets", [](M & m) { cp::build_gadgets(m, 10, false); });
    npp_on("atsp+complements", [](M & m) { cp::build_atsp(m, 20, true); });
    npp_on("cflp 12x30", [](M & m) { build_cflp(m, 12, 30, 1, 100.0); });
}

// -------------------------- rows on P before and after a solve with cuts
static void persist() {
    for(const char * mode : {"pool", "rows_icutgen", "rows_irowgen"}) {
        M m; scope_settings(m); auto g = cp::build_gadgets(m, 10, false); TL(m, 30);
        auto s = std::make_shared<cp::stats>();
        std::vector<cp::raw_cut> hulls;
        for(int i = 1; i < 10; ++i) hulls.push_back(g.hull_cut(i));
        bool once = false;
        attach(m, cp::watched([&](const auto &, const cp::add_fn & add) { if(!once) { once = true; for(const auto & c : hulls) add(c); } }, s), mode);
        const int before = m.native_api().get_num_rows(m.native_model());
        const double t = cp::timed_solve(m);
        std::println(stderr, "== PERSIST {} rows on P before={} after={} cuts={}: {} {}", mode, before, m.native_api().get_num_rows(m.native_model()), s->cuts.load(), cp::summary(m, t), tail(m));
    }
}

int main(int argc, char ** argv) {
    const std::string cmd = argc > 1 ? argv[1] : "matrix";
    const auto arg = [&](int i, const char * d = "") { return std::string(argc > i ? argv[i] : d); };
    if(cmd == "matrix") { g_guard = true; auto B = make_backend(); cp::run_matrix(B); return 0; }
    if(cmd == "version") { M m; std::println(stderr, "library {} version {}", m.native_api().library_path().string(), m.native_api().version()); return 0; }
    if(cmd == "sizing") {
        for(int n : {8, 10, 12, 14, 16, 18, 20}) {
            M m; cp::build_atsp(m, n); TL(m, 20);
            const double t = cp::timed_solve(m);
            std::println(stderr, "== SIZING n={} defaults: {} {}", n, cp::summary(m, t), tail(m));
            M o; o.model_params.presolve = 0; cp::build_atsp(o, n); TL(o, 20);
            const double t2 = cp::timed_solve(o);
            std::println(stderr, "   presolve off: {} {}", cp::summary(o, t2), tail(o));
            if(t > 20 && t2 > 20) break;
        }
        return 0;
    }
    if(cmd == "pathcheck") { pathcheck(); return 0; }
    if(cmd == "map") { map_probe(); return 0; }
    if(cmd == "presolve") { presolve_case(arg(2, "e3")); return 0; }
    if(cmd == "rootcut") { rootcut(); return 0; }
    if(cmd == "reads") { reads(); return 0; }
    if(cmd == "reinject") { reinject(); return 0; }
    if(cmd == "filter") { filter_probe(); return 0; }
    if(cmd == "purge") { purge(); return 0; }
    if(cmd == "weird") { weird(arg(2, "pool"), arg(3, "equality")); return 0; }
    if(cmd == "forbid") { forbidden(std::atoi(arg(2, "1").c_str()), arg(3, "pool") == "pool"); return 0; }
    if(cmd == "root") { root_cases(std::atoi(arg(2, "0").c_str())); return 0; }
    if(cmd == "cols") { cols(); return 0; }
    if(cmd == "lazy") { lazy(std::atoi(arg(2, "12").c_str()), static_cast<unsigned>(std::atoi(arg(3, "5").c_str())), arg(4, "lazy_only")); return 0; }
    if(cmd == "persist") { persist(); return 0; }
    if(cmd == "npp") { npp_probe(); return 0; }
    if(cmd == "npp_e2e") { npp_e2e(); return 0; }
    if(cmd == "plain") {
        // the wrapper's own solve(), untouched: same search as M's, and a hook set from outside is lost
        static long fired = 0;
        Base m; (void)cp::build_atsp(m, ATSP_N);
        m.set_time_limit(std::chrono::duration<double>(60));
        m.model_params.cb_func = [](glp_tree *, void *) { ++fired; };
        m.solve();
        std::println(stderr, "== PLAIN glpk_milp n={}: {} value={} glp_get_it_cnt={}; hook written to model_params from outside fired {} times; presolve in model_params={}", ATSP_N, cp::status_name(m.get_status()), m.get_solution_value(), m.native_api().get_it_cnt(m.native_model()), fired, m.model_params.presolve);
        return 0;
    }
    if(cmd == "thresh") { thresh(); return 0; }
    if(cmd == "rounds") { rounds(); return 0; }
    if(cmd == "lmkp") { lazy_mkp(std::atoi(arg(2, "5").c_str()), std::atoi(arg(3, "30").c_str()), static_cast<unsigned>(std::atoi(arg(4, "7").c_str())), arg(5, "off")); return 0; }
    if(cmd == "gen") { generators(arg(2, "all"), arg(3, "rows_icutgen"), std::atoi(arg(4, "20").c_str())); return 0; }
    std::println(stderr, "unknown command {}", cmd);
    return 2;
}
