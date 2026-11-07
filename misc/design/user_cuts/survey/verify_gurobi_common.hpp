#pragma once
// Skeptic's own native layer for Gurobi: a raw model builder, a hook with counters.
// Hooks never let an exception out and never return non-zero.
#include "mippp/solvers/gurobi/all.hpp"
#include "cut_conformance.hpp"
using M = mippp::gurobi_milp;
namespace g = mippp::gurobi::impl::v1;
using cp::raw_cut;
using std::size_t;

enum : int { W_MIP = 3, W_MIPSOL = 4, W_MIPNODE = 5, W_MESSAGE = 6,
             SOL_SOL = 4001, SOL_OBJ = 4002,
             NODE_STATUS = 5001, NODE_REL = 5002, NODE_OBJBST = 5003, NODE_OBJBND = 5004, NODE_NODCNT = 5005, NODE_SOLCNT = 5006,
             MSG_STRING = 6001, ST_OPTIMAL = 2 };

inline void seti(M & m, const char * n, int v) { auto [env, model] = m.native_model(); m.native_api()._check(env, m.native_api().setintparam(env, n, v)); }
inline void setd(M & m, const char * n, double v) { auto [env, model] = m.native_model(); m.native_api()._check(env, m.native_api().setdblparam(env, n, v)); }
inline int geti_attr(M & m, const char * n) { auto [env, model] = m.native_model(); int v = 0; m.native_api().getintattr(model, n, &v); return v; }
inline double getd_attr(M & m, const char * n) { auto [env, model] = m.native_model(); double v = NAN; if(m.native_api().getdblattr(model, n, &v)) return NAN; return v; }
inline int num_vars(M & m) { auto [env, model] = m.native_model(); m.native_api().updatemodel(model); return geti_attr(m, "NumVars"); }
inline void print_release(M & m) {
    int a = 0, b = 0, c = 0;
    cp::symf<void (*)(int *, int *, int *)>(m.native_api().library_path(), "GRBversion")(&a, &b, &c);
    std::println(stderr, "== library {} version {}.{}.{}", m.native_api().library_path().string(), a, b, c);
}

// ------------------------------------------------------------------- raw model
struct rawm {
    std::vector<double> obj, lb, ub; std::vector<char> vt;
    struct row { std::vector<int> ind; std::vector<double> val; char sense; double rhs; };
    std::vector<row> rows;
    int sense = -1; // -1 maximize, +1 minimize
    double offset = 0;
    int var(double o, double l, double u, char t) { obj.push_back(o); lb.push_back(l); ub.push_back(u); vt.push_back(t); return static_cast<int>(obj.size()) - 1; }
    int bin(double o = 0) { return var(o, 0, 1, 'B'); }
    void add(std::vector<int> i, std::vector<double> v, char s, double r) { rows.push_back({std::move(i), std::move(v), s, r}); }
    double eval(const std::vector<double> & x) const { double s = offset; for(size_t j = 0; j < obj.size(); ++j) s += obj[j] * x[j]; return s; }
    void load(M & m) const {
        auto [env, model] = m.native_model();
        auto & api = m.native_api();
        std::vector<int> beg(obj.size(), 0), ind; std::vector<double> val;
        auto o = obj, l = lb, u = ub; auto t = vt;
        api._check(env, api.addvars(model, static_cast<int>(obj.size()), 0, beg.data(), ind.data(), val.data(), o.data(), l.data(), u.data(), t.data(), nullptr));
        for(const auto & r : rows) {
            auto ri = r.ind; auto rv = r.val;
            api._check(env, api.addconstr(model, static_cast<int>(ri.size()), ri.data(), rv.data(), r.sense, r.rhs, nullptr));
        }
        api._check(env, api.setintattr(model, "ModelSense", sense));
        api._check(env, api.setdblattr(model, "ObjCon", offset));
        api._check(env, api.updatemodel(model));
    }
};

struct sol { int status = -1, solcount = 0; double value = NAN, bound = NAN, nodes = NAN, iters = NAN, work = NAN, runtime = NAN; std::vector<double> x; };
inline const char * sname(int s) {
    switch(s) { case 2: return "OPTIMAL"; case 3: return "INFEASIBLE"; case 4: return "INF_OR_UNBD"; case 5: return "UNBOUNDED"; case 7: return "ITERLIM"; case 8: return "NODELIM"; case 9: return "TIMELIM"; case 10: return "SOLLIM"; case 11: return "INTERRUPTED"; case 12: return "NUMERIC"; case 13: return "SUBOPTIMAL"; default: return "?"; }
}
inline sol solve_native(M & m, bool read_x = true) {
    auto [env, model] = m.native_model();
    auto & api = m.native_api();
    sol s;
    const int e = api.optimize(model);
    if(e) std::println(stderr, "   GRBoptimize error {}", e);
    api.getintattr(model, "Status", &s.status);
    api.getintattr(model, "SolCount", &s.solcount);
    s.value = getd_attr(m, "ObjVal"); s.bound = getd_attr(m, "ObjBound"); s.nodes = getd_attr(m, "NodeCount");
    s.iters = getd_attr(m, "IterCount"); s.work = getd_attr(m, "Work"); s.runtime = getd_attr(m, "Runtime");
    if(read_x && s.solcount > 0) { s.x.resize(static_cast<size_t>(num_vars(m))); api.getdblattrarray(model, "X", 0, static_cast<int>(s.x.size()), s.x.data()); }
    return s;
}
inline std::string line(const sol & s) {
    return std::format("status={} value={:.10g} bound={:.10g} nodes={} iters={} work={:.4f} t={:.2f}s", sname(s.status), s.value, s.bound, s.nodes, s.iters, s.work, s.runtime);
}

// ------------------------------------------------------------------- the hook
struct hctx;
using node_fn = std::function<void(hctx &, const std::vector<double> &)>;
struct hctx {
    const g::gurobi_api * api = nullptr;
    int cols = 0;
    node_fn on_node;
    std::function<void(hctx &, const std::vector<double> &, double)> on_sol;
    std::function<void(hctx &, const char *)> on_msg;
    bool filter = true;     // status filter of the recipe
    bool lazy = false;      // GRBcblazy instead of GRBcbcut
    bool drop_satisfied = true, san = true;
    std::chrono::microseconds spin{0};
    // per-call state
    void * cbdata = nullptr; int where = 0; double nod = 0, best = NAN, bnd = NAN; int status = 0;
    std::vector<double> x;
    // counters
    long fires = 0, opt_calls = 0, root_calls = 0, tree_calls = 0, cuts = 0, root_cuts = 0, tree_cuts = 0, native_err = 0, swallowed = 0, rel_err = 0, sanitized_out = 0;
    int first_err = 0;
    std::map<int, long> status_hist;
    double last_nod = -1; bool added_this_call = false, last_added = false;
    long same_after_cut = 0, same_after_nocut = 0, next_after_cut = 0, next_after_nocut = 0, tree_same_after_cut = 0, tree_same_after_nocut = 0;
    std::map<double, long> tree_calls_per_nod;  // optimal calls per NODCNT > 0
    std::map<double, long> tree_fires_per_nod;  // every status
    std::thread::id main_thread = std::this_thread::get_id();
    std::atomic<long> off_main{0}, overlaps{0}; std::atomic<int> inflight{0}, max_inflight{0};
    std::set<std::thread::id> threads; std::mutex mx;
    int add(const raw_cut & in) {
        raw_cut tmp;
        const raw_cut * c = &in;
        if(san) {
            auto r = cf::sanitize(in, x, drop_satisfied);
            if(!r) { ++sanitized_out; return 0; }
            tmp = std::move(*r); c = &tmp;
        }
        const char s = c->sense == 'E' ? '=' : c->sense == 'L' ? '<' : '>';
        const int n = static_cast<int>(c->ind.size());
        const int e = lazy ? api->cblazy(cbdata, n, c->ind.data(), c->val.data(), s, c->rhs)
                           : api->cbcut(cbdata, n, c->ind.data(), c->val.data(), s, c->rhs);
        if(e) { ++native_err; if(!first_err) first_err = e; }
        else { ++cuts; ++(nod == 0 ? root_cuts : tree_cuts); added_this_call = true; }
        return e;
    }
    std::string counters() const {
        std::string st; for(auto [k, v] : status_hist) st += std::format(" {}:{}", k, v);
        long mx_tree = 0, mult = 0; for(auto [k, v] : tree_calls_per_nod) { mx_tree = std::max(mx_tree, v); if(v > 1) ++mult; }
        return std::format("fires={} status{{{} }} root_calls={} tree_calls={} distinct_tree_nodcnt={} tree_nodcnt_with_repeats={} max_calls_at_tree_nodcnt={} root_cuts={} tree_cuts={} after_cut(same/next)={}/{} after_nocut(same/next)={}/{} tree_same_after_cut={} tree_same_after_nocut={} native_err={}(first {}) rel_err={} sanitized_out={} swallowed={}",
            fires, st, root_calls, tree_calls, tree_calls_per_nod.size(), mult, mx_tree, root_cuts, tree_cuts, same_after_cut, next_after_cut, same_after_nocut, next_after_nocut, tree_same_after_cut, tree_same_after_nocut, native_err, first_err, rel_err, sanitized_out, swallowed);
    }
};
inline int hook(g::GRBmodel *, void * cbdata, int where, void * usr) {
    auto & c = *static_cast<hctx *>(usr);
    try {
        if(where == W_MESSAGE && c.on_msg) {
            char * msg = nullptr;
            if(!c.api->cbget(cbdata, where, MSG_STRING, &msg) && msg) c.on_msg(c, msg);
            return 0;
        }
        if(where == W_MIPSOL && c.on_sol) {
            c.cbdata = cbdata; c.where = where;
            std::vector<double> x(static_cast<size_t>(c.cols));
            double o = NAN;
            c.api->cbget(cbdata, where, SOL_SOL, x.data());
            c.api->cbget(cbdata, where, SOL_OBJ, &o);
            c.on_sol(c, x, o);
            return 0;
        }
        if(where != W_MIPNODE) return 0;
        const int now = c.inflight.fetch_add(1) + 1;
        if(now > 1) c.overlaps.fetch_add(1);
        { int seen = c.max_inflight.load(); while(now > seen && !c.max_inflight.compare_exchange_weak(seen, now)) {} }
        if(std::this_thread::get_id() != c.main_thread) c.off_main.fetch_add(1);
        { std::lock_guard l(c.mx); c.threads.insert(std::this_thread::get_id()); }
        ++c.fires;
        c.cbdata = cbdata; c.where = where;
        c.api->cbget(cbdata, where, NODE_STATUS, &c.status);
        c.api->cbget(cbdata, where, NODE_NODCNT, &c.nod);
        c.api->cbget(cbdata, where, NODE_OBJBST, &c.best);
        c.api->cbget(cbdata, where, NODE_OBJBND, &c.bnd);
        c.status_hist[c.status]++;
        if(c.nod > 0) c.tree_fires_per_nod[c.nod]++;
        if(c.spin.count()) { const auto until = clk::now() + c.spin; while(clk::now() < until) {} }
        if(c.status == ST_OPTIMAL || !c.filter) {
            ++c.opt_calls;
            ++(c.nod == 0 ? c.root_calls : c.tree_calls);
            if(c.nod > 0) c.tree_calls_per_nod[c.nod]++;
            if(c.last_nod >= 0) {
                const bool same = c.nod == c.last_nod;
                if(c.last_added) { ++(same ? c.same_after_cut : c.next_after_cut); if(same && c.nod > 0) ++c.tree_same_after_cut; }
                else { ++(same ? c.same_after_nocut : c.next_after_nocut); if(same && c.nod > 0) ++c.tree_same_after_nocut; }
            }
            c.last_nod = c.nod;
            c.added_this_call = false;
            c.x.assign(static_cast<size_t>(c.cols), -7.0);
            if(c.api->cbget(cbdata, where, NODE_REL, c.x.data())) ++c.rel_err;
            if(c.on_node) c.on_node(c, c.x);
            c.last_added = c.added_this_call;
        }
        c.inflight.fetch_sub(1);
    } catch(...) { ++c.swallowed; c.inflight.store(0); }
    return 0;
}
inline int noop_hook(g::GRBmodel *, void *, int, void *) { return 0; }
inline void attach(M & m, hctx & c) {
    auto [env, model] = m.native_model();
    auto & api = m.native_api();
    c.api = &api;
    c.cols = num_vars(m);
    api._check(env, api.setcallbackfunc(model, hook, &c));
}
inline void detach(M & m) { auto [env, model] = m.native_model(); m.native_api()._check(env, m.native_api().setcallbackfunc(model, nullptr, nullptr)); }
inline void recipe(M & m) { seti(m, "PreCrush", 1); }
inline void scope(M & m) { seti(m, "Presolve", 0); seti(m, "Cuts", 0); setd(m, "Heuristics", 0.0); seti(m, "Symmetry", 0); seti(m, "Disconnected", 0); seti(m, "Threads", 1); }

// ---------------------------------------------------------------- instances
// Maximize, binary, rows with some negative coefficients: same family as C10.
struct cgm {
    rawm r; std::vector<int> x;
    std::vector<std::vector<double>> a; std::vector<double> b, c;
};
inline cgm make_cg(unsigned seed, size_t cols = 35, size_t rows = 15) {
    cgm M_;
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> A(-5, 20), C(1, 40);
    M_.a.assign(rows, std::vector<double>(cols));
    for(auto & r : M_.a) for(auto & v : r) v = A(rng);
    M_.b.resize(rows); M_.c.resize(cols);
    for(size_t i = 0; i < rows; ++i) { double t = 0; for(double v : M_.a[i]) t += std::max(v, 0.0); M_.b[i] = std::floor(t * 0.4); }
    for(auto & v : M_.c) v = C(rng);
    for(size_t j = 0; j < cols; ++j) M_.x.push_back(M_.r.bin(M_.c[j]));
    for(size_t i = 0; i < rows; ++i) { std::vector<double> v = M_.a[i]; M_.r.add(M_.x, v, '<', M_.b[i]); }
    return M_;
}
// Covers of the rows of a cgm at p, valid for every integer point.
inline std::vector<raw_cut> covers(const cgm & G, const std::vector<double> & p, double min_viol = 1e-4) {
    std::vector<raw_cut> out;
    for(size_t i = 0; i < G.a.size(); ++i) {
        std::vector<size_t> order; double best = 0;
        for(size_t j = 0; j < G.x.size(); ++j) { if(G.a[i][j] > 0) order.push_back(j); else best += G.a[i][j]; }
        std::ranges::sort(order, [&](size_t l, size_t r) { return p[static_cast<size_t>(G.x[l])] > p[static_cast<size_t>(G.x[r])]; });
        raw_cut cover{{}, {}, 'L', 0};
        for(size_t j : order) {
            best += G.a[i][j]; cover.ind.push_back(G.x[j]); cover.val.push_back(1);
            if(best > G.b[i] + 1e-9) { cover.rhs = static_cast<double>(cover.ind.size()) - 1; if(cover.violation(p) > min_viol) out.push_back(cover); break; }
        }
    }
    return out;
}
// CG cuts from rows tight at p, multipliers k/d; valid since x >= 0.
inline std::vector<raw_cut> cg_cuts(const cgm & G, const std::vector<double> & p, std::mt19937 & gen, int want = 3, double min_viol = 1e-4) {
    std::vector<raw_cut> out;
    std::vector<size_t> tight;
    for(size_t i = 0; i < G.a.size(); ++i) { double act = 0; for(size_t j = 0; j < G.x.size(); ++j) act += G.a[i][j] * p[static_cast<size_t>(G.x[j])]; if(act >= G.b[i] - 1e-6) tight.push_back(i); }
    if(tight.empty()) return out;
    const int dens[] = {2, 3, 4, 5, 7, 11};
    std::uniform_int_distribution<size_t> D(0, std::size(dens) - 1);
    for(int attempt = 0; attempt < 300 && static_cast<int>(out.size()) < want; ++attempt) {
        const int d = dens[D(gen)];
        std::uniform_int_distribution<int> K(0, d - 1);
        std::vector<double> lam(G.a.size(), 0.0);
        for(size_t i : tight) lam[i] = static_cast<double>(K(gen)) / d;
        raw_cut cut{{}, {}, 'L', 0}; double rhs = 0;
        for(size_t i = 0; i < G.a.size(); ++i) rhs += lam[i] * G.b[i];
        cut.rhs = std::floor(rhs + 1e-9);
        for(size_t j = 0; j < G.x.size(); ++j) { double t = 0; for(size_t i = 0; i < G.a.size(); ++i) t += lam[i] * G.a[i][j]; cut.ind.push_back(G.x[j]); cut.val.push_back(std::floor(t + 1e-9)); }
        if(cut.violation(p) > min_viol) out.push_back(std::move(cut));
    }
    return out;
}
