// The experiments beyond cut_common.hpp. Usage: probe_copt_x <experiment> [args].
// Every solve has a time limit; run each experiment under `timeout -k 5`.
#include <cstring>
#include "probe_copt_common.cpp"
using namespace std::chrono;
constexpr int R = c::COPT_CBCONTEXT_MIPRELAX, N = c::COPT_CBCONTEXT_MIPNODE, S = c::COPT_CBCONTEXT_MIPSOL, I = c::COPT_CBCONTEXT_INCUMBENT;
using params = std::vector<std::pair<const char *, int>>;
static void set(M & m, const params & p) {
    auto & api = m.native_api();
    auto [env, prob] = m.native_model();
    for(auto [name, v] : p) api._check(env, api.SetIntParam(prob, name, v));
}
static int iattr(M & m, const char * a) { int v = -1; m.native_api().GetIntAttr(m.native_model().second, a, &v); return v; }
static double dattr(M & m, const char * a) { double v = NAN; m.native_api().GetDblAttr(m.native_model().second, a, &v); return v; }
static std::string mask_name(int mk) {
    std::string s;
    if(mk & R) s += "R";
    if(mk & N) s += "N";
    if(mk & S) s += "S";
    if(mk & I) s += "I";
    return s.empty() ? "0" : s;
}
template <class F>
static void attach_raw(M & m, int mask, F f) {
    attach(m, [](const auto &, const auto &) {}, "usercut");
    m.hook->mask = mask;
    m.hook->raw = f;
}
static std::vector<double> relax(const hook_ctx & h, void * cbdata) {
    std::vector<double> x(static_cast<std::size_t>(h.cols));
    const int e = h.api->GetCallbackInfo(cbdata, c::COPT_CBINFO_RELAXSOLUTION, x.data());
    if(e) std::println(stderr, "RelaxSolution read rc={}", e);
    return x;
}
static int addcut(const hook_ctx & h, void * cbdata, const cp::raw_cut & r) {
    return h.api->AddCallbackUserCut(cbdata, static_cast<int>(r.ind.size()), r.ind.data(), r.val.data(), r.sense, r.rhs);
}
static void heading(const std::string & t) {
    std::println(stderr, "--- {}", t);
    std::println("--- {}", t);
    std::fflush(stdout);
}

// Q6: does a context move the search? no-op hooks, deterministic at one thread.
static void ctx() {
    const std::vector<std::pair<std::string, int>> masks = {{"no callback", -1}, {"N (wrapper today)", N}, {"R", R}, {"RN", R | N}, {"RNI", R | N | I}, {"S", S}, {"RNS", R | N | S}, {"RNS LazyConstraints=0", 256 | R | N | S}, {"R LazyConstraints=1", 512 | R}, {"RN LazyConstraints=1", 512 | R | N}, {"N LazyConstraints=1", 512 | N}};
    const auto run = [&](const char * inst, auto build, const params & p) {
        for(auto & [label, mk] : masks) {
            heading(std::format("ctx {} mask {}", inst, label));
            M m; m.set_thread_limit(1); build(m);
            m.set_time_limit(duration<double>(30.0));
            set(m, p);
            if(mk == -1) m.no_callback = true;
            else {
                if(mk & 256) set(m, {{"LazyConstraints", 0}});
                if(mk & 512) set(m, {{"LazyConstraints", 1}});
                attach_raw(m, mk & 15, [](c::copt_prob *, void *, int) { return 0; });
            }
            if(std::getenv("CUT_LOG")) set(m, {{"Logging", 1}});
            const double t = cp::timed_solve(m);
            std::println(stderr, "{:10} {:24} {} simplex={} relax? rows={}", inst, label, cp::summary(m, t), iattr(m, "SimplexIter"), iattr(m, "Rows"));
        }
    };
    run("atsp26", [](M & m) { cp::build_atsp(m, 26); }, {});
    run("mkp", [](M & m) { cp::build_mkp(m, 30, 100); }, {{"NodeLimit", 800}});
    run("gadgets", [](M & m) { cp::build_gadgets(m, 10, false); }, {{"Presolve", 0}, {"RootCutLevel", 0}, {"HeurLevel", 0}});
}

// Q1, Q12: what the hook can read, with return codes.
static void reads(int argc, char ** argv) {
    const bool threads = argc > 2 && std::string(argv[2]) == "threads";
    M m;
    if(!threads) scope_settings(m);
    auto g = cp::build_gadgets(m, 10, false); m.set_time_limit(duration<double>(10.0));
    auto & api = m.native_api();
    auto * model_prob = m.native_model().second;
    const int cols = static_cast<int>(m.num_variables());
    const int y0 = g.y[0];
    auto get_col_info = cp::symf<int (*)(c::copt_prob *, const char *, int, const int *, double *)>(api.library_path(), "COPT_GetColInfo");
    const std::vector<std::string> info = {"BestObj", "BestBnd", "HasIncumbent", "Incumbent", "MipCandObj", "MipCandidate", "RelaxSolObj", "RelaxSolution", "NodeStatus",
        "NodeDepth", "Depth", "NodeID", "NodeId", "NodeNum", "NodeCnt", "NodeCount", "NodeLeft", "NodeObj", "LpIter", "CutRound", "LB", "UB", "RelaxLB", "RelaxUB", "LocalLB", "LocalUB", "LpObjval", "Cols", "Rows"};
    int shown = 0, shown_node = 0;
    std::atomic<long> diff_prob{0}, relax_calls{0}, lb_moved{0}, lb_stayed{0};
    attach_raw(m, R | N | I, [&](c::copt_prob * p, void * cb, int ctx) {
        if(p != model_prob) ++diff_prob;
        if(ctx == R) ++relax_calls;
        const bool show = ctx == R ? shown < 3 : ctx == N ? shown_node < 1 : false;
        if(ctx == R) {
            auto x = relax(*m.hook, cb);
            double lb = NAN, ub = NAN;
            int e1 = get_col_info(p, "LB", 1, &y0, &lb), e2 = get_col_info(p, "UB", 1, &y0, &ub);
            int nodecnt = -1, cols2 = -1, rows2 = -1; double bnd = NAN;
            int e3 = api.GetIntAttr(p, "NodeCnt", &nodecnt), e4 = api.GetIntAttr(p, "Cols", &cols2), e5 = api.GetIntAttr(p, "Rows", &rows2), e6 = api.GetDblAttr(p, "BestBnd", &bnd);
            double robj = NAN; int e7 = api.GetCallbackInfo(cb, "RelaxSolObj", &robj);
            const int st = g.state(x);
            if((st == 0 && ub < 0.5) || (st == 1 && lb > 0.5)) ++lb_moved; else if(st != -1) ++lb_stayed;
            if(shown < 12) std::println(stderr, "R call: y0={} state={} GetColInfo LB rc={} ={} UB rc={} ={} | attr NodeCnt rc={} ={} Cols rc={} ={} Rows rc={} ={} BestBnd rc={} ={} | RelaxSolObj rc={} ={} c'x={} | thread prob==model {}", x[static_cast<std::size_t>(y0)], st, e1, lb, e2, ub, e3, nodecnt, e4, cols2, e5, rows2, e6, bnd, e7, robj, g.objective(x), p == model_prob);
        }
        if(show) {
            std::string line = std::format("ctx {}: ", ctx == R ? "MIPRELAX" : ctx == N ? "MIPNODE" : "INCUMBENT");
            for(const auto & n : info) {
                std::vector<double> buf(static_cast<std::size_t>(cols) + 64, -12345.0);
                const int e = api.GetCallbackInfo(cb, n.c_str(), buf.data());
                std::int32_t asint = 0; std::memcpy(&asint, buf.data(), 4);
                if(e == 0) line += std::format("{}=rc0(dbl {:.5g} int {}) ", n, buf[0], asint);
                else line += std::format("{}=rc{} ", n, e);
            }
            std::println(stderr, "{}", line);
        }
        if(ctx == R) ++shown; else if(ctx == N) ++shown_node;
        return 0;
    });
    const double t = cp::timed_solve(m);
    std::println(stderr, "reads {}: {} relax={} prob-differs={} y0 bound reflects branching: moved={} stayed={}", threads ? "default" : "scope", cp::summary(m, t), relax_calls.load(), diff_prob.load(), lb_moved.load(), lb_stayed.load());
}

// RelaxSolObj against c'x, sense, objective constant, presolve on.
static void objcheck() {
    M m; m.set_thread_limit(1);
    auto a = cp::build_atsp(m, 26); m.set_time_limit(duration<double>(20.0));
    double maxdiff = 0; long calls = 0;
    attach_raw(m, R | N, [&](c::copt_prob *, void * cb, int ctx) {
        if(ctx != R) return 0;
        auto x = relax(*m.hook, cb);
        double o = NAN; m.native_api().GetCallbackInfo(cb, "RelaxSolObj", &o);
        maxdiff = std::max(maxdiff, std::fabs(o - a.objective(x)));
        ++calls;
        return 0;
    });
    const double t = cp::timed_solve(m);
    std::println(stderr, "objcheck atsp min presolve-on: {} calls={} max|RelaxSolObj - c'x|={}", cp::summary(m, t), calls, maxdiff);
    M n; scope_settings(n);
    auto g = cp::build_gadgets(n, 10, false); n.set_time_limit(duration<double>(20.0));
    maxdiff = 0; calls = 0;
    attach_raw(n, R | N, [&](c::copt_prob *, void * cb, int ctx) {
        if(ctx != R) return 0;
        auto x = relax(*n.hook, cb);
        double o = NAN; n.native_api().GetCallbackInfo(cb, "RelaxSolObj", &o);
        maxdiff = std::max(maxdiff, std::fabs(o - g.objective(x)));
        ++calls;
        return 0;
    });
    const double t2 = cp::timed_solve(n);
    std::println(stderr, "objcheck gadgets max: {} calls={} max|RelaxSolObj - c'x|={}", cp::summary(n, t2), calls, maxdiff);
}

// Add functions in contexts the guide forbids, read info in contexts the guide does not list.
static void contexts() {
    M m; m.set_thread_limit(1);
    auto a = cp::build_atsp(m, 26); m.set_time_limit(duration<double>(20.0));
    const cp::raw_cut any{{a.col[0][1], a.col[1][0]}, {1, 1}, 'L', 1.0};
    bool sn = false, ss = false, si = false, sr = false;
    auto & api = m.native_api();
    const int cols = static_cast<int>(m.num_variables());
    attach_raw(m, R | N | S | I, [&](c::copt_prob *, void * cb, int ctx) {
        std::vector<double> buf(static_cast<std::size_t>(cols) + 8);
        const int n = static_cast<int>(any.ind.size());
        if(ctx == N && !sn) { sn = true; std::println(stderr, "MIPNODE: AddCallbackUserCut rc={} AddCallbackLazyConstr rc={} RelaxSolution rc={} NodeStatus rc={} MipCandidate rc={}", (int)api.AddCallbackUserCut(cb, n, any.ind.data(), any.val.data(), 'L', 1.0), (int)api.AddCallbackLazyConstr(cb, n, any.ind.data(), any.val.data(), 'L', 1.0), (int)api.GetCallbackInfo(cb, "RelaxSolution", buf.data()), (int)api.GetCallbackInfo(cb, "NodeStatus", buf.data()), (int)api.GetCallbackInfo(cb, "MipCandidate", buf.data())); }
        if(ctx == S && !ss) { ss = true; std::println(stderr, "MIPSOL: AddCallbackUserCut rc={} RelaxSolution rc={} NodeStatus rc={} MipCandidate rc={}", (int)api.AddCallbackUserCut(cb, n, any.ind.data(), any.val.data(), 'L', 1.0), (int)api.GetCallbackInfo(cb, "RelaxSolution", buf.data()), (int)api.GetCallbackInfo(cb, "NodeStatus", buf.data()), (int)api.GetCallbackInfo(cb, "MipCandidate", buf.data())); }
        if(ctx == I && !si) { si = true; std::println(stderr, "INCUMBENT: AddCallbackUserCut rc={} AddCallbackLazyConstr rc={} RelaxSolution rc={} Incumbent rc={}", (int)api.AddCallbackUserCut(cb, n, any.ind.data(), any.val.data(), 'L', 1.0), (int)api.AddCallbackLazyConstr(cb, n, any.ind.data(), any.val.data(), 'L', 1.0), (int)api.GetCallbackInfo(cb, "RelaxSolution", buf.data()), (int)api.GetCallbackInfo(cb, "Incumbent", buf.data())); }
        if(ctx == R && !sr) { sr = true; std::println(stderr, "MIPRELAX: NodeStatus rc={} MipCandidate rc={} RelaxSolution rc={}", (int)api.GetCallbackInfo(cb, "NodeStatus", buf.data()), (int)api.GetCallbackInfo(cb, "MipCandidate", buf.data()), (int)api.GetCallbackInfo(cb, "RelaxSolution", buf.data())); }
        return 0;
    });
    const double t = cp::timed_solve(m);
    std::println(stderr, "contexts: {}", cp::summary(m, t));
}

// Valid in the worse subtree only, violated by its LPs, not parallel to the objective:
// gadget 0 at its worse total plus gadgets 1..k-1 at their integer total. S1's cut is the
// objective itself, which a solver could also read as a cutoff. Global semantics: opt - 2 c0.
template <class G>
static cp::raw_cut partial_cut(const G & g, const bool flip) {
    cp::raw_cut c; c.sense = 'L';
    c.rhs = g.c[0] * (flip ? g.total0[0] : g.total1[0]);
    c.ind = {g.za[0], g.zb[0]}; c.val = {1, 1};
    for(int i = 1; i < g.k; ++i) {
        const auto I = static_cast<std::size_t>(i);
        c.ind.push_back(g.za[I]); c.ind.push_back(g.zb[I]); c.val.push_back(1); c.val.push_back(1);
        c.rhs += g.c[I] * g.total0[I];
    }
    return c;
}

// Q2: a locally valid cut which is not parallel to the objective, under each cut setting.
static void s1b(int argc, char ** argv) {
    const int k = argc > 2 ? std::atoi(argv[2]) : 10;
    const std::string mode = argc > 3 ? argv[3] : "usercut";
    for(const char * v : {"noroot", "noroot_r1", "tree1", "tree1r1", "tree2", "tree3", "root_r1", "root_r1_tree1", "root1_tree1"})
    for(const bool every : {false, true})
    for(const bool flip : {false, true}) {
        setenv("CUT_SCOPE", v, 1);
        M m; scope_settings(m); auto g = cp::build_gadgets(m, k, flip); m.set_time_limit(duration<double>(25.0));
        if(std::string(v) == "tree2") set(m, {{"CutLevel", 0}, {"TreeCutLevel", 2}});
        if(std::string(v) == "tree3") set(m, {{"CutLevel", 0}, {"TreeCutLevel", 3}});
        const cp::raw_cut cut = partial_cut(g, flip);
        const int trigger = g.worse_state();
        bool added = false; int rc = 0; long adds = 0, same = 0, same_v = 0, sib = 0, sib_v = 0;
        attach(m, [&](const std::vector<double> & x, const cp::add_fn & add) {
            const int st = g.state(x);
            const double v = cut.violation(x);
            if((!added || every) && st == trigger && v > 1e-4) { rc = add(cut); added = true; ++adds; return; }
            if(!added) return;
            if(st == trigger) { ++same; same_v += v > 1e-6; }
            else if(st == 1 - trigger) { ++sib; sib_v += v > 1e-6; }
        }, mode);
        const double t = cp::timed_solve(m);
        double value = NAN; try { value = m.get_solution_value(); } catch(...) {}
        std::println(stderr, "S1b k={} scope={:14} every={} flip={}: {} (opt {:.6g}, a global cut gives {:.6g}) {} adds={} rc={} subtree viol/checks={}/{} sibling viol/checks={}/{} relax={} node={}", k, v, every, flip, cp::summary(m, t), g.opt, g.opt - g.c[0] * (flip ? (g.total1[0] - g.total0[0]) : (g.total0[0] - g.total1[0])), std::fabs(value - g.opt) < 1e-4 ? "KEPT" : "LOST", adds, rc, same_v, same, sib_v, sib, m.hook->relax_calls.load(), m.hook->node_calls.load());
    }
}

// A cut at the root (RootCutRounds=1 lets the root fire and keeps a tree).
static void rootadd() {
    for(const char * v : {"root_r1", "root_r1_tree1"}) {
        setenv("CUT_SCOPE", v, 1);
        for(const bool flip : {false, true}) {
            // the hull cut of gadget 3 at the first call, and no more
            M m; scope_settings(m); auto g = cp::build_gadgets(m, 10, flip); m.set_time_limit(duration<double>(20.0));
            const cp::raw_cut cut{{g.za[3], g.zb[3]}, {1, 1}, 'L', g.c[3] * g.total0[3]};
            bool added = false; long zero = 0, zero_v = 0, one = 0, one_v = 0, f = 0, f_v = 0; int rc = 0; double v0 = 0;
            attach(m, [&](const std::vector<double> & x, const cp::add_fn & add) {
                const int st = g.state(x);
                const double vv = cut.violation(x);
                if(!added && st == -1) { v0 = vv; rc = add(cut); added = true; return; }
                if(!added) return;
                (st == 0 ? zero : st == 1 ? one : f)++;
                (st == 0 ? zero_v : st == 1 ? one_v : f_v) += vv > 1e-6;
            }, "usercut");
            const double t = cp::timed_solve(m);
            std::println(stderr, "rootadd {} flip={} valid hull(3) at the root: viol at add={:.4f} rc={}: {} (opt {:.6g}) later viol/checks y0=0 {}/{} y0=1 {}/{} frac {}/{} relax={} node={}", v, flip, v0, rc, cp::summary(m, t), g.opt, zero_v, zero, one_v, one, f_v, f, m.hook->relax_calls.load(), m.hook->node_calls.load());
        }
        for(const bool flip : {false, true}) {
            // the cut valid only in the worse subtree, at the root: must lose
            M m; scope_settings(m); auto g = cp::build_gadgets(m, 10, flip); m.set_time_limit(duration<double>(20.0));
            const cp::raw_cut cut = g.invalid_global_cut();
            bool added = false; int rc = 0;
            attach(m, [&](const std::vector<double> & x, const cp::add_fn & add) {
                if(!added && cut.violation(x) > 1e-4) { rc = add(cut); added = true; }
            }, "usercut");
            const double t = cp::timed_solve(m);
            std::println(stderr, "rootadd {} flip={} the invalid objective cut at first call, rc={}: {} (opt {:.6g})", v, flip, rc, cp::summary(m, t), g.opt);
        }
    }
}

// Q3: a cut removing only non-optimal integer points; objective cutoff.
static void dominance() {
    for(const bool flip : {false, true}) {
        { M m; scope_settings(m); auto g = cp::build_gadgets(m, 10, flip); m.set_time_limit(duration<double>(20.0));
          const double better = flip ? g.total1[0] : g.total0[0];
          const cp::raw_cut cut{{g.za[0], g.zb[0]}, {1, 1}, 'G', g.c[0] * better};
          bool added = false; int rc = 0; long calls_after = 0;
          attach(m, [&](const std::vector<double> & x, const cp::add_fn & add) {
              if(!added && cut.violation(x) > 1e-4) { rc = add(cut); added = true; } else if(added) ++calls_after; }, "usercut");
          const double t = cp::timed_solve(m);
          std::println(stderr, "dominance flip={} za0+zb0 >= better total, added={} rc={}: {} (opt {:.6g}) relax={}", flip, added, rc, cp::summary(m, t), g.opt, m.hook->relax_calls.load()); }
        { M m; scope_settings(m); auto g = cp::build_gadgets(m, 10, flip); m.set_time_limit(duration<double>(20.0));
          const cp::raw_cut cut{g.z, std::vector<double>(g.z.size(), 1.0), 'L', g.opt + 0.01};
          bool added = false; int rc = 0;
          attach(m, [&](const std::vector<double> & x, const cp::add_fn & add) {
              if(!added && cut.violation(x) > 1e-4) { rc = add(cut); added = true; } }, "usercut");
          const double t = cp::timed_solve(m);
          std::println(stderr, "dominance flip={} objective cap at opt+0.01, added={} rc={}: {} (opt {:.6g}) relax={}", flip, added, rc, cp::summary(m, t), g.opt, m.hook->relax_calls.load()); }
    }
    // ATSP at default settings: the partial objective (no arc leaving node 0) floored above the optimal tour's
    for(const int root_level : {-1, 0}) {
        std::vector<double> best;
        { M b; b.set_thread_limit(1); auto a0 = cp::build_atsp(b, 26); b.set_time_limit(duration<double>(40.0)); cp::timed_solve(b);
          best.resize(b.num_variables()); b.native_api().GetSolution(b.native_model().second, best.data()); }
        M m; m.set_thread_limit(1); auto a = cp::build_atsp(m, 26); m.set_time_limit(duration<double>(40.0));
        if(root_level == 0) set(m, {{"RootCutLevel", 0}});
        cp::raw_cut cut; cut.sense = 'G';
        for(int i = 1; i < a.n; ++i) for(int j = 0; j < a.n; ++j) if(i != j) { cut.ind.push_back(a.col[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]); cut.val.push_back(a.d[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]); }
        cut.rhs = cut.activity(best) + 1;
        long done_at_add = -1; int rc = 0; double v0 = 0;
        attach(m, [&](const std::vector<double> & x, const cp::add_fn & add) {
            const long nd = m.hook->node_calls.load();
            if(done_at_add < 0 && nd >= 20 && cut.violation(x) > 1e-4) { v0 = cut.violation(x); rc = add(cut); done_at_add = nd; } }, "usercut");
        const double t = cp::timed_solve(m);
        std::vector<double> sol(m.num_variables()); m.native_api().GetSolution(m.native_model().second, sol.data());
        std::println(stderr, "atsp default RootCutLevel={} partial-objective floor {} (optimal tour violates it by 1) added after {} nodes viol={:.3f} rc={}: {} final solution violates it by {:.3f}", root_level, cut.rhs, done_at_add, v0, rc, cp::summary(m, t), cut.violation(sol));
    }
    // the same trick on the ATSP at default settings: an objective floor above the true optimum (invalid), at a tree node
    for(const int root_level : {-1, 0}) {
        M m; m.set_thread_limit(1); auto a = cp::build_atsp(m, 26); m.set_time_limit(duration<double>(40.0));
        if(root_level == 0) set(m, {{"RootCutLevel", 0}});
        cp::raw_cut cut; cut.sense = 'G'; cut.rhs = 550;
        for(int i = 0; i < a.n; ++i) for(int j = 0; j < a.n; ++j) if(i != j) { cut.ind.push_back(a.col[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]); cut.val.push_back(a.d[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]); }
        long done_at_add = -1; int rc = 0;
        attach(m, [&](const std::vector<double> & x, const cp::add_fn & add) {
            const long nd = m.hook->node_calls.load();
            if(done_at_add < 0 && nd >= 20 && cut.violation(x) > 1e-4) { rc = add(cut); done_at_add = nd; } }, "usercut");
        const double t = cp::timed_solve(m);
        std::println(stderr, "atsp default RootCutLevel={} floor obj>=550 (true opt 549) added after {} nodes rc={}: {}", root_level, done_at_add, rc, cp::summary(m, t));
    }
}

// Q4: the guide says non-violated cuts are discarded. y3 + 0.01 za3 <= activity - eps at the first
// call with y3 fractional: two real terms, because a one-term cut is dropped in the tree anyway.
// In force, the descendants obey it; dropped, later points exceed it. The cut itself is invalid on
// purpose: only whether it takes matters.
static void discard() {
    for(const double eps : {1.0, 1e-1, 1e-2, 1e-3, 1e-4, 1e-5, 1e-6, 1e-7, 1e-8, 1e-9, 1e-10, 0.0, -1e-9, -1e-6, -1e-4, -1e-2, -1e-1}) {
        M m; scope_settings(m); auto g = cp::build_gadgets(m, 10, false); m.set_time_limit(duration<double>(20.0));
        const int y3 = g.y[3], z3 = g.za[3];
        bool added = false; int rc = 0; long after = 0, exceeded = 0, same_node = 0, first_nodes = 0, first_exceed_after = -1; cp::raw_cut cut;
        attach(m, [&](const std::vector<double> & x, const cp::add_fn & add) {
            const double v = x[static_cast<std::size_t>(y3)];
            if(!added) {
                if(v > 1e-3 && v < 1 - 1e-3) {
                    cut = cp::raw_cut{{y3, z3}, {1.0, 0.01}, 'L', 0.0};
                    cut.rhs = cut.activity(x) - eps;
                    added = true; first_nodes = m.hook->node_calls.load();
                    rc = add(cut);
                }
                return;
            }
            ++after;
            if(m.hook->node_calls.load() == first_nodes) ++same_node;
            if(cut.violation(x) > 1e-6) { ++exceeded; if(first_exceed_after < 0) first_exceed_after = after; }
        }, "usercut");
        const double t = cp::timed_solve(m);
        std::println(stderr, "discard violation at add={:9.1e} rc={}: {} relax={} refired on the node={} later points violating={}/{} first at later call {}", eps, rc, cp::summary(m, t), m.hook->relax_calls.load(), same_node, exceeded, after, first_exceed_after);
    }
    // the same on a dense cut with large coefficients: efficacy, not raw violation
    for(const double delta : {0.01, 0.1, 1.0, 5.0, 20.0}) {
        M m; m.set_thread_limit(1); auto a = cp::build_atsp(m, 26); m.set_time_limit(duration<double>(20.0));
        cp::raw_cut base; base.sense = 'G';
        for(int i = 0; i < a.n; ++i) for(int j = 0; j < a.n; ++j) if(i != j) { base.ind.push_back(a.col[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]); base.val.push_back(a.d[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]); }
        double norm = 0; for(double v : base.val) norm += v * v; norm = std::sqrt(norm);
        long adds = 0, rc_bad = 0, root = 0; double last = 0, first = NAN;
        attach(m, [&](const std::vector<double> & x, const cp::add_fn & add) {
            if(m.hook->node_calls.load() == 0) ++root;
            auto c = base; c.rhs = a.objective(x) + delta;
            if(std::isnan(first)) first = a.objective(x);
            last = a.objective(x);
            if(c.rhs < 549) { ++adds; if(add(c)) ++rc_bad; }
        }, "usercut");
        const double t = cp::timed_solve(m);
        std::println(stderr, "ladder atsp26 obj >= LP + {} (efficacy {:.2e}): {} relax={} node={} adds={} rc errors={} calls before first MIPNODE={}", delta, delta / norm, cp::summary(m, t), m.hook->relax_calls.load(), m.hook->node_calls.load(), adds, rc_bad, root);
    }
}

// Q4: many cuts at once; are they all used?
static void burst() {
    {   M m; m.set_thread_limit(1); auto a = cp::build_atsp(m, 40); m.set_time_limit(duration<double>(40.0));
        std::vector<cp::raw_cut> added; long call = 0;
        attach(m, [&](const std::vector<double> & x, const cp::add_fn & add) {
            ++call;
            if(call == 1) {
                added = a.separate(x, 100000);
                int bad = 0; for(const auto & c : added) if(add(c)) ++bad;
                std::println(stderr, "burst atsp40 first call: {} violated DFJ cuts added, rc errors={}, max viol={:.3f}", added.size(), bad, added.empty() ? 0.0 : added.front().violation(x));
            } else if(call <= 8) {
                long held = 0; for(const auto & c : added) if(c.violation(x) <= 1e-6) ++held;
                std::println(stderr, "   call {} nodes_done={} obj={:.4f}: {}/{} of the burst hold", call, m.hook->node_calls.load(), a.objective(x), held, added.size());
            }
        }, "usercut");
        const double t = cp::timed_solve(m);
        std::println(stderr, "burst atsp40: {} relax={} node={}", cp::summary(m, t), m.hook->relax_calls.load(), m.hook->node_calls.load());
    }
    for(const char * v : {"root_r1", "noroot"}) {
        setenv("CUT_SCOPE", v, 1);
        M m; scope_settings(m); auto g = cp::build_gadgets(m, 10, false); m.set_time_limit(duration<double>(20.0));
        std::vector<cp::raw_cut> added; long call = 0;
        attach(m, [&](const std::vector<double> & x, const cp::add_fn & add) {
            ++call;
            if(call == 1) {
                for(int i = 1; i < 10; ++i) { auto c = g.hull_cut(i); c.ind.resize(2); c.val.resize(2); if(c.violation(x) > 1e-6) added.push_back(c); }
                int bad = 0; for(const auto & c : added) if(add(c)) ++bad;
                std::println(stderr, "burst gadgets {} first call: {} violated cuts, rc errors={}", v, added.size(), bad);
            } else if(call <= 6) {
                long held = 0; for(const auto & c : added) if(c.violation(x) <= 1e-6) ++held;
                std::println(stderr, "   call {} nodes_done={} obj={:.4f}: {}/{} hold", call, m.hook->node_calls.load(), g.objective(x), held, added.size());
            }
        }, "usercut");
        const double t = cp::timed_solve(m);
        std::println(stderr, "burst gadgets {}: {} relax={} node={}", v, cp::summary(m, t), m.hook->relax_calls.load(), m.hook->node_calls.load());
    }
}

// Q4: are cuts purged? Root cuts only, watched over the whole tree.
static void purge(int argc, char ** argv) {
    const int n = argc > 2 ? std::atoi(argv[2]) : 40;
    const int max_rounds = argc > 3 ? std::atoi(argv[3]) : 5;
    const std::size_t per_round = argc > 4 ? static_cast<std::size_t>(std::atoi(argv[4])) : 20;
    M m; m.set_thread_limit(1); auto a = cp::build_atsp(m, n); m.set_time_limit(duration<double>(40.0));
    std::vector<cp::raw_cut> added; long call = 0, rounds = 0, later = 0, later_any_viol = 0, viol_total = 0, stop_at = 0;
    std::map<long, std::pair<long, long>> by_node;  // bucketed per 200 nodes done
    std::vector<long> viol_per_cut;
    attach(m, [&](const std::vector<double> & x, const cp::add_fn & add) {
        ++call;
        const long nd = m.hook->node_calls.load();
        if(nd == 0 && rounds < max_rounds) {  // the first root rounds add
            ++rounds;
            for(const auto & c : a.separate(x, per_round)) { if(!add(c)) { added.push_back(c); viol_per_cut.push_back(0); } }
            return;
        }
        if(stop_at == 0) { stop_at = call; std::println(stderr, "purge: stopped adding at call {} with {} cuts after {} rounds, nodes_done={}", call, added.size(), rounds, nd); }
        ++later;
        long v = 0;
        for(std::size_t k = 0; k < added.size(); ++k) if(added[k].violation(x) > 1e-6) { ++v; ++viol_per_cut[k]; }
        viol_total += v; if(v) ++later_any_viol;
        auto & b = by_node[nd / 500]; b.first += 1; b.second += v ? 1 : 0;
    }, "usercut");
    const double t = cp::timed_solve(m);
    long cuts_ever = 0; for(long w : viol_per_cut) if(w) ++cuts_ever;
    std::println(stderr, "purge atsp{}: {} relax={} node={} later calls={} with a root cut violated={} violations={} distinct root cuts ever violated={}/{}", n, cp::summary(m, t), m.hook->relax_calls.load(), m.hook->node_calls.load(), later, later_any_viol, viol_total, cuts_ever, added.size());
    for(auto & [b, p] : by_node) std::println(stderr, "   nodes {:6}..: calls={} with a root cut violated={}", b * 500, p.first, p.second);
}

// Q7: rounds against nodes.
static void fire() {
    const auto hist = [](M & m, const char * label, auto sep, const params & p, double limit) {
        set(m, p);
        m.set_time_limit(duration<double>(limit));
        std::map<long, long> per_node; long last = -1, cnt = 0, root = 0;
        std::atomic<long> calls{0};
        attach(m, [&](const std::vector<double> & x, const cp::add_fn & add) {
            const long nd = m.hook->node_calls.load();
            if(nd != last) { if(last >= 0) per_node[cnt]++; last = nd; cnt = 0; }
            ++cnt; ++calls;
            if(nd == 0) ++root;
            sep(x, add);
        }, "usercut");
        const double t = cp::timed_solve(m);
        if(last >= 0) per_node[cnt]++;
        std::string h; long nodes_with = 0;
        for(auto [k, v] : per_node) { h += std::format(" {}x:{}", k, v); nodes_with += v; }
        std::println(stderr, "fire {}: {} relax={} node={} calls before the first MIPNODE={} per-node-with-a-call histogram:{} (nodes that saw a call: {})", label, cp::summary(m, t), m.hook->relax_calls.load(), m.hook->node_calls.load(), root, h, nodes_with);
    };
    { M m; m.set_thread_limit(1); auto a = cp::build_atsp(m, 26); hist(m, "atsp26 default no cuts", [](const auto &, const auto &) {}, {}, 30.0); }
    { M m; m.set_thread_limit(1); auto a = cp::build_atsp(m, 26); hist(m, "atsp26 default DFJ", [&a](const auto & x, const cp::add_fn & add) { for(auto & c : a.separate(x)) add(c); }, {}, 30.0); }
    { M m; m.set_thread_limit(1); auto a = cp::build_atsp(m, 40); hist(m, "atsp40 default DFJ", [&a](const auto & x, const cp::add_fn & add) { for(auto & c : a.separate(x)) add(c); }, {}, 30.0); }
    { M m; m.set_thread_limit(1); cp::build_mkp(m, 30, 100); hist(m, "mkp default no cuts", [](const auto &, const auto &) {}, {}, 8.0); }
    { M m; scope_settings(m); auto g = cp::build_gadgets(m, 10, false); hist(m, "gadgets noroot no cuts", [](const auto &, const auto &) {}, {}, 10.0); }
    // Ladder: always one more violated cut. How many rounds does COPT grant? Objective floor at LP + 0.01.
    for(const params & p : {params{}, params{{"RootCutRounds", 3}}, params{{"RootCutRounds", 50}}, params{{"RootCutRounds", 1000}}, params{{"NodeCutRounds", 1}}, params{{"NodeCutRounds", 100}}, params{{"RootCutLevel", 0}, {"NodeCutRounds", 100}}, params{{"RootCutLevel", 3}}}) {
        M m; m.set_thread_limit(1); auto a = cp::build_atsp(m, 26);
        std::string lab = "atsp26 ladder";
        for(auto [k, v] : p) lab += std::format(" {}={}", k, v);
        cp::raw_cut base; base.sense = 'G';
        for(int i = 0; i < a.n; ++i) for(int j = 0; j < a.n; ++j) if(i != j) { base.ind.push_back(a.col[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]); base.val.push_back(a.d[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]); }
        hist(m, lab.c_str(), [&](const auto & x, const cp::add_fn & add) { auto c = base; c.rhs = a.objective(x) + 0.01; if(c.rhs < 549) add(c); }, p, 15.0);
    }
}

// Q8: overlaps across contexts, at default threads.
static void threads() {
    const auto mkp = [](M & m) { cp::build_mkp(m, 30, 100); };
    for(const long us : {0L, 200L, 2000L, 10000L}) {
        M m; mkp(m); m.set_time_limit(duration<double>(8.0));
        std::atomic<int> inflight{0}, maxin{0}; std::atomic<long> over{0}, rc{0}, nc{0}, cross{0};
        std::mutex mu; std::set<std::thread::id> tid; std::set<void *> cbs; std::set<void *> probs;
        std::atomic<int> in_relax{0}, in_node{0};
        attach_raw(m, R | N, [&](c::copt_prob * p, void * cb, int ctx) {
            const int now = inflight.fetch_add(1) + 1;
            if(now > 1) over.fetch_add(1);
            int seen = maxin.load(); while(now > seen && !maxin.compare_exchange_weak(seen, now)) {}
            { std::lock_guard l(mu); tid.insert(std::this_thread::get_id()); cbs.insert(cb); probs.insert(p); }
            (ctx == R ? rc : nc).fetch_add(1);
            if(us) { const auto until = clk::now() + microseconds(us); while(clk::now() < until) {} }
            inflight.fetch_sub(1);
            return 0;
        });
        const double t = cp::timed_solve(m);
        std::println(stderr, "threads default spin={}us: {} relax={} node={} os_threads={} distinct cbdata={} distinct prob={} (model prob one of them: {}) overlaps={} max_inflight={}", us, cp::summary(m, t), rc.load(), nc.load(), tid.size(), cbs.size(), probs.size(), probs.contains(m.native_model().second), over.load(), maxin.load());
    }
    {   // does a separator work at default threads
        M m; auto a = cp::build_atsp(m, 40); m.set_time_limit(duration<double>(40.0));
        auto s = std::make_shared<cp::stats>();
        attach(m, cp::watched([&a](const auto & x, const cp::add_fn & add) { for(const auto & c : a.separate(x)) add(c); }, s, [&a](const auto & x) { return a.objective(x); }, [&a](const auto & x) { return a.point_ok(x); }, [&a](const auto & x) { return a.integral(x); }), "usercut");
        const double t = cp::timed_solve(m);
        std::println(stderr, "threads atsp40 DFJ default threads: {}\n   {}", cp::summary(m, t), s->line());
    }
    {   M m; m.set_thread_limit(1); auto a = cp::build_atsp(m, 40); m.set_time_limit(duration<double>(40.0));
        auto s = std::make_shared<cp::stats>();
        attach(m, cp::watched([&a](const auto & x, const cp::add_fn & add) { for(const auto & c : a.separate(x)) add(c); }, s, [&a](const auto & x) { return a.objective(x); }, [&a](const auto & x) { return a.point_ok(x); }, [&a](const auto & x) { return a.integral(x); }), "usercut");
        const double t = cp::timed_solve(m);
        std::println(stderr, "threads atsp40 DFJ 1 thread: {}\n   {}", cp::summary(m, t), s->line());
    }
    {   // a tree: a long search with cuts at default threads, watching the spin overlap, RootCutLevel=0 so calls come from tree workers
        M m; auto a = cp::build_atsp(m, 26); m.set_time_limit(duration<double>(20.0)); set(m, {{"RootCutLevel", 0}});
        auto s = std::make_shared<cp::stats>();
        attach(m, cp::watched([](const auto &, const auto &) {}, s, {}, {}, {}, microseconds(2000)), "usercut");
        const double t = cp::timed_solve(m);
        std::println(stderr, "threads atsp26 RootCutLevel=0 default threads 2 ms spin: {}\n   {}", cp::summary(m, t), s->line());
    }
}

// Q11: DFJ model with only the degree constraints.
static cp::atsp build_dfj(M & m, const int n) {
    cp::atsp a; a.n = n;
    const auto Nn = static_cast<std::size_t>(n);
    std::mt19937 rng(5);
    std::uniform_real_distribution<double> P(0, 100), noise(1.0, 1.3);
    std::vector<std::pair<double, double>> pts(Nn);
    for(auto & p : pts) p = {P(rng), P(rng)};
    a.d.assign(Nn, std::vector<double>(Nn, 0));
    for(std::size_t i = 0; i < Nn; ++i) for(std::size_t j = 0; j < Nn; ++j) if(i != j)
        a.d[i][j] = std::round(std::hypot(pts[i].first - pts[j].first, pts[i].second - pts[j].second) * noise(rng));
    const auto arcs = Nn * (Nn - 1);
    const auto id = [Nn](std::size_t i, std::size_t j) { return i * (Nn - 1) + (j < i ? j : j - 1); };
    auto x = m.add_binary_variables(arcs);
    a.col.assign(Nn, std::vector<int>(Nn, -1));
    a.comp.assign(Nn, std::vector<int>(Nn, -1));
    for(std::size_t i = 0; i < Nn; ++i) for(std::size_t j = 0; j < Nn; ++j) if(i != j) a.col[i][j] = m.native_id(x(id(i, j)));
    m.set_minimization();
    const auto nodes = std::views::iota(std::size_t{0}, Nn);
    m.set_objective(xsum(nodes, [&](std::size_t i) { return xsum(nodes | std::views::filter([i](std::size_t j) { return j != i; }), [&, i](std::size_t j) { return a.d[i][j] * x(id(i, j)); }); }));
    for(std::size_t i : nodes) {
        m.add_constraint(xsum(nodes | std::views::filter([i](std::size_t j) { return j != i; }), [&, i](std::size_t j) { return 1.0 * x(id(i, j)); }) == 1);
        m.add_constraint(xsum(nodes | std::views::filter([i](std::size_t j) { return j != i; }), [&, i](std::size_t j) { return 1.0 * x(id(j, i)); }) == 1);
    }
    return a;
}
static bool is_tour(const cp::atsp & a, const std::vector<double> & x) {
    int at = 0, len = 0;
    do {
        int nxt = -1;
        for(int j = 0; j < a.n; ++j) if(j != at && a.X(x, at, j) > 0.5) { nxt = j; break; }
        if(nxt < 0) return false;
        at = nxt; ++len;
    } while(at != 0 && len <= a.n);
    return at == 0 && len == a.n;
}
static void lazy(int argc, char ** argv) {
    const int n = argc > 2 ? std::atoi(argv[2]) : 26;
    struct cfg { const char * name; bool lazy_sol, user_relax, lazy_relax; int mask; };
    const std::vector<cfg> cfgs = {
        {"lazy on MIPSOL only", true, false, false, S},
        {"lazy on MIPSOL + user cuts on MIPRELAX", true, true, false, S | R},
        {"same + MIPNODE bit", true, true, false, S | R | N},
        {"lazy on MIPSOL + lazy on MIPRELAX", true, false, true, S | R},
        {"user cuts on MIPRELAX only, no MIPSOL", false, true, false, R | N},
        {"lazy on MIPRELAX only, no MIPSOL bit", false, false, true, R | N},
        {"lazy on MIPRELAX only, MIPSOL bit on but no check there", false, false, true, R | N | S},
    };
    for(const auto & cf : cfgs) {
        M m; m.set_thread_limit(1); auto a = build_dfj(m, n); m.set_time_limit(duration<double>(60.0));
        long sol = 0, sol_cuts = 0, rel = 0, rel_cuts = 0, rel_errs = 0, sol_errs = 0, rel_int = 0; int last_rel_rc = 0, last_sol_rc = 0;
        attach_raw(m, cf.mask, [&](c::copt_prob *, void * cb, int ctx) {
            auto & h = *m.hook;
            if(ctx == S) {
                ++sol;
                if(!cf.lazy_sol) return 0;
                std::vector<double> x(static_cast<std::size_t>(h.cols));
                const int e = h.api->GetCallbackInfo(cb, c::COPT_CBINFO_MIPCANDIDATE, x.data());
                if(e) { ++sol_errs; last_sol_rc = e; return 0; }
                for(const auto & r : a.separate(x, 3)) {
                    const int rc = h.api->AddCallbackLazyConstr(cb, static_cast<int>(r.ind.size()), r.ind.data(), r.val.data(), r.sense, r.rhs);
                    if(rc) { ++sol_errs; last_sol_rc = rc; } else ++sol_cuts;
                }
            } else if(ctx == R) {
                ++rel;
                auto x = relax(h, cb);
                if(a.integral(x)) ++rel_int;
                for(const auto & r : a.separate(x, 3)) {
                    int rc = 0;
                    if(cf.lazy_relax) rc = h.api->AddCallbackLazyConstr(cb, static_cast<int>(r.ind.size()), r.ind.data(), r.val.data(), r.sense, r.rhs);
                    else if(cf.user_relax) rc = addcut(h, cb, r);
                    if(rc) { ++rel_errs; last_rel_rc = rc; } else if(cf.lazy_relax || cf.user_relax) ++rel_cuts;
                }
            }
            return 0;
        });
        const double t = cp::timed_solve(m);
        std::string tour = "no solution";
        try { std::vector<double> x(static_cast<std::size_t>(m.hook->cols)); m.native_api().GetSolution(m.native_model().second, x.data()); tour = is_tour(a, x) ? "a tour" : "NOT A TOUR (subtours)"; } catch(...) {}
        std::println(stderr, "lazy n={} [{}]: {} solution is {} | MIPSOL calls={} lazy cuts={} errors={}(rc {}) | MIPRELAX calls={} integral={} cuts={} errors={}(rc {})", n, cf.name, cp::summary(m, t), tour, sol, sol_cuts, sol_errs, last_sol_rc, rel, rel_int, rel_cuts, rel_errs, last_rel_rc);
    }
}

// P: the harness skips it, because COPT has set_constraint_upper_bound and no set_constraint_rhs.
static void persist() {
    for(const int add_how : {0, 1}) {  // 0 = from the hook, 1 = COPT_AddUserCut before the solve
        M m; scope_settings(m); auto g = cp::build_gadgets(m, 10, false); m.set_time_limit(duration<double>(20.0));
        const cp::raw_cut cut{{g.za[3], g.zb[3]}, {1, 1}, 'L', g.c[3] * g.total0[3]};
        bool once = false;
        auto s = std::make_shared<cp::stats>();
        if(add_how == 1) {
            auto f = cp::symf<addcut_row_fn>(m.native_api().library_path(), "COPT_AddUserCut");
            std::println(stderr, "AddUserCut rc={}", f(m.native_model().second, static_cast<int>(cut.ind.size()), cut.ind.data(), cut.val.data(), cut.sense, cut.rhs, 0.0, nullptr));
        }
        attach(m, cp::watched([&](const auto & x, const cp::add_fn & add) { if(add_how == 0 && !once && cut.violation(x) > 1e-4) { once = true; add(cut); } }, s), "usercut");
        double t = cp::timed_solve(m);
        std::println(stderr, "P how={} first solve with the hull cut of gadget 3: {} (opt {:.6g}) relax={} rows={}", add_how ? "AddUserCut" : "hook", cp::summary(m, t), g.opt, m.hook->relax_calls.load(), iattr(m, "Rows"));
        t = cp::timed_solve(m);
        std::println(stderr, "P how={} re-solve with nothing changed: {} (opt {:.6g}) relax={} rows={}", add_how ? "AddUserCut" : "hook", cp::summary(m, t), g.opt, m.hook->relax_calls.load(), iattr(m, "Rows"));
        for(std::size_t r = 12; r < 16; ++r) m.set_constraint_upper_bound(g.rows[r], m.get_constraint_upper_bound(g.rows[r]) + 50);
        t = cp::timed_solve(m);
        double value = NAN; try { value = m.get_solution_value(); } catch(const std::exception &) {}
        std::println(stderr, "P how={} re-solve, rhs raised so the cut is invalid: {} (new opt {:.6g}) => {}", add_how ? "AddUserCut" : "hook", cp::summary(m, t), g.opt + 100, std::fabs(value - (g.opt + 100)) < 1e-4 ? "cut did NOT persist" : "cut PERSISTED (wrong optimum)");
    }
}

// COPT_AddUserCut before the solve, against the hook.
static void pre() {
    const std::vector<std::pair<std::string, std::function<void(M &)>>> settings = {
        {"noroot", [](M & m) { setenv("CUT_SCOPE", "noroot", 1); scope_settings(m); }},
        {"silent (CutLevel=0)", [](M & m) { setenv("CUT_SCOPE", "silent", 1); scope_settings(m); }},
        {"tree1", [](M & m) { setenv("CUT_SCOPE", "tree1", 1); scope_settings(m); }},
        {"defaults", [](M &) {}},
    };
    for(auto & [label, setup] : settings) for(const bool flip : {false, true}) {
        const auto run = [&](const char * what, auto cutf) {
            M m; setup(m); m.set_thread_limit(1); auto g = cp::build_gadgets(m, 10, flip); m.set_time_limit(duration<double>(20.0));
            const cp::raw_cut cut = cutf(g);
            auto f = cp::symf<addcut_row_fn>(m.native_api().library_path(), "COPT_AddUserCut");
            const int rc = f(m.native_model().second, static_cast<int>(cut.ind.size()), cut.ind.data(), cut.val.data(), cut.sense, cut.rhs, 0.0, nullptr);
            long v = 0, chk = 0;
            attach(m, [&](const std::vector<double> & x, const cp::add_fn &) { ++chk; v += cut.violation(x) > 1e-6; }, "usercut");
            const double t = cp::timed_solve(m);
            double value = NAN; try { value = m.get_solution_value(); } catch(...) {}
            std::println(stderr, "pre {:20} flip={} {:28} rc={}: {} (opt {:.6g}) {} hook viol/checks={}/{} node={}", label, flip, what, rc, cp::summary(m, t), g.opt, std::fabs(value - g.opt) < 1e-4 ? "KEPT" : "LOST", v, chk, m.hook->node_calls.load());
        };
        run("invalid: objective cap", [](const auto & g) { return g.invalid_global_cut(); });
        run("invalid: gadget 0 cap", [flip](const auto & g) { return cp::raw_cut{{g.za[0], g.zb[0]}, {1, 1}, 'L', g.c[0] * (flip ? g.total0[0] : g.total1[0])}; });
        run("valid: hull(3)", [](const auto & g) { return cp::raw_cut{{g.za[3], g.zb[3]}, {1, 1}, 'L', g.c[3] * g.total0[3]}; });
    }
}

// What a nonzero return and an interrupt do from MIPRELAX.
static void ret() {
    for(const int r : {1, -1, 5}) {
        M m; scope_settings(m); auto g = cp::build_gadgets(m, 10, false); m.set_time_limit(duration<double>(10.0));
        long calls = 0;
        attach_raw(m, R | N, [&](c::copt_prob *, void *, int ctx) { if(ctx == R) { ++calls; return r; } return 0; });
        const double t = cp::timed_solve(m);
        std::println(stderr, "ret hook returns {} on MIPRELAX: {} calls={} native rc={}", r, cp::summary(m, t), calls, m.hook->native_rc);
    }
    {   M m; scope_settings(m); auto g = cp::build_gadgets(m, 10, false); m.set_time_limit(duration<double>(10.0));
        long calls = 0; int rc = 0;
        attach_raw(m, R | N, [&](c::copt_prob * p, void *, int ctx) { if(ctx == R) { ++calls; rc = m.native_api().Interrupt(p); } return 0; });
        const double t = cp::timed_solve(m);
        std::println(stderr, "ret COPT_Interrupt from MIPRELAX rc={}: {} calls={} native rc={}", rc, cp::summary(m, t), calls, m.hook->native_rc);
    }
}

// The hint-or-binding question at the sibling: what fraction of sibling LPs contain a tree-added cut,
// and does a candidate violating the cut get accepted? Records every incumbent against the cut.
static void candidates(int argc, char ** argv) {
    const char * v = argc > 2 ? argv[2] : "noroot";
    setenv("CUT_SCOPE", v, 1);
    for(const bool flip : {false, true}) {
        M m; scope_settings(m); auto g = cp::build_gadgets(m, 10, flip); m.set_time_limit(duration<double>(20.0));
        const cp::raw_cut cut = partial_cut(g, flip);
        const int trigger = g.worse_state();
        bool added = false; long sols = 0, sols_viol = 0, incs = 0, incs_viol = 0, rel = 0, rel_int_viol = 0;
        const bool with_sol = argc > 3;
        attach_raw(m, R | N | I | (with_sol ? S : 0), [&](c::copt_prob *, void * cb, int ctx) {
            auto & h = *m.hook;
            std::vector<double> x(static_cast<std::size_t>(h.cols));
            if(ctx == R) {
                ++rel;
                h.api->GetCallbackInfo(cb, c::COPT_CBINFO_RELAXSOLUTION, x.data());
                if(!added && g.state(x) == trigger && cut.violation(x) > 1e-4) { added = true; addcut(h, cb, cut); }
            } else if(ctx == S) {
                h.api->GetCallbackInfo(cb, c::COPT_CBINFO_MIPCANDIDATE, x.data());
                ++sols; if(added && cut.violation(x) > 1e-6) ++sols_viol;
            } else if(ctx == I) {
                h.api->GetCallbackInfo(cb, c::COPT_CBINFO_INCUMBENT, x.data());
                ++incs; if(added && cut.violation(x) > 1e-6) ++incs_viol;
            }
            return 0;
        });
        const double t = cp::timed_solve(m);
        std::println(stderr, "candidates scope={} MIPSOL_bit={} flip={}: {} (opt {:.6g}) added={} MIPSOL candidates={} after-add violating the cut={} | incumbents={} violating={}", v, with_sol, flip, cp::summary(m, t), g.opt, added, sols, sols_viol, incs, incs_viol);
    }
}

// AddCallbackLazyConstr from MIPRELAX, with and without the MIPSOL bit, on a model where the hook fires.
static void contexts2() {
    for(const int mk : {R | N, R, R | N | S, R | S}) {
        M m; m.set_thread_limit(1); auto a = cp::build_atsp(m, 26); m.set_time_limit(duration<double>(20.0));
        long calls = 0, ok = 0, bad = 0; int rc = 0;
        attach_raw(m, mk, [&](c::copt_prob *, void * cb, int ctx) {
            if(ctx != R) return 0;
            ++calls;
            auto x = relax(*m.hook, cb);
            for(const auto & r : a.separate(x, 3)) {
                const int e = m.native_api().AddCallbackLazyConstr(cb, static_cast<int>(r.ind.size()), r.ind.data(), r.val.data(), r.sense, r.rhs);
                if(e) { ++bad; rc = e; } else ++ok;
            }
            return 0;
        });
        const double t = cp::timed_solve(m);
        std::println(stderr, "lazy from MIPRELAX mask {}: {} calls={} accepted={} refused={} (rc {})", mask_name(mk), cp::summary(m, t), calls, ok, bad, rc);
    }
}

// Is the undocumented callback info NodeCnt the node count? Against counted MIPNODE events.
static void nodecnt() {
    for(const int th : {1, 0}) {
        M m; if(th) m.set_thread_limit(1);
        auto a = cp::build_atsp(m, 26); m.set_time_limit(duration<double>(20.0));
        std::atomic<long> nodes{0}, rcalls{0}, bad_rc{0}, maxdiff{0}; int last_info = -1; std::mutex mu;
        std::vector<std::tuple<int, long, int>> sample;
        attach_raw(m, R | N | I, [&](c::copt_prob *, void * cb, int ctx) {
            if(ctx == N) nodes.fetch_add(1);
            if(ctx == R) rcalls.fetch_add(1);
            int v = -1;
            const int e = m.native_api().GetCallbackInfo(cb, "NodeCnt", &v);
            if(e) { bad_rc.fetch_add(1); return 0; }
            std::lock_guard l(mu);
            last_info = v;
            long seen = maxdiff.load(); const long d = std::labs(long(v) - nodes.load()); if(d > seen) maxdiff = d;
            if(sample.size() < 6 || (nodes.load() % 400 == 0 && sample.size() < 12)) sample.emplace_back(ctx, nodes.load(), v);
            return 0;
        });
        const double t = cp::timed_solve(m);
        std::string smp; for(auto [ctx, n, v] : sample) smp += std::format(" (ctx {} counted {} info {})", ctx, n, v);
        std::println(stderr, "nodecnt threads={}: {} MIPNODE events={} relax={} rc errors={} last info={} max|info - counted|={}{}", th ? "1" : "default", cp::summary(m, t), nodes.load(), rcalls.load(), bad_rc.load(), last_info, maxdiff.load(), smp);
    }
}

// Does anything clear a model-level user cut?
static void clearcut() {
    for(const int reset : {-1, 0, 1}) {
        M m; scope_settings(m); auto g = cp::build_gadgets(m, 10, false); m.set_time_limit(duration<double>(20.0));
        const cp::raw_cut cut = g.invalid_global_cut();
        auto f = cp::symf<addcut_row_fn>(m.native_api().library_path(), "COPT_AddUserCut");
        f(m.native_model().second, static_cast<int>(cut.ind.size()), cut.ind.data(), cut.val.data(), cut.sense, cut.rhs, 0.0, nullptr);
        attach(m, [](const auto &, const auto &) {}, "usercut");
        double t = cp::timed_solve(m);
        std::print(stderr, "clearcut first: {} ", cp::summary(m, t));
        int rc = 0;
        if(reset >= 0) rc = m.native_api().Reset(m.native_model().second, reset);
        t = cp::timed_solve(m);
        std::println(stderr, "| COPT_Reset({}) rc={} then re-solve: {} (opt {:.6g})", reset, rc, cp::summary(m, t), g.opt);
    }
}

// A no-op MIPRELAX on more instances: same node count and simplex count as an empty slot?
static void ctx2() {
    struct inst { const char * name; std::function<void(M &)> build; params p; };
    const std::vector<inst> list = {
        {"atsp30", [](M & m) { cp::build_atsp(m, 30, false, 11); }, {}},
        {"atsp26 Presolve=0", [](M & m) { cp::build_atsp(m, 26); }, {{"Presolve", 0}}},
        {"atsp26 RootCutLevel=0", [](M & m) { cp::build_atsp(m, 26); }, {{"RootCutLevel", 0}}},
        {"atsp26 TreeCutLevel=0", [](M & m) { cp::build_atsp(m, 26); }, {{"TreeCutLevel", 0}}},
        {"atsp26 RootCutRounds=0", [](M & m) { cp::build_atsp(m, 26); }, {{"RootCutRounds", 0}}},
        {"mkp 5x60", [](M & m) { cp::build_mkp(m, 5, 60); }, {{"NodeLimit", 3000}}},
        {"mkp 30x100 Presolve=0", [](M & m) { cp::build_mkp(m, 30, 100); }, {{"Presolve", 0}, {"NodeLimit", 800}}},
        {"gadgets Presolve=0", [](M & m) { cp::build_gadgets(m, 10, false); }, {{"Presolve", 0}}},
        {"gadgets k=16 Presolve=0 Root=0", [](M & m) { cp::build_gadgets(m, 16, false); }, {{"Presolve", 0}, {"RootCutLevel", 0}}},
    };
    for(const auto & it : list) for(const int mk : {-1, N, R | N}) {
        M m; m.set_thread_limit(1); it.build(m); m.set_time_limit(duration<double>(25.0)); set(m, it.p);
        if(mk == -1) m.no_callback = true; else attach_raw(m, mk, [](c::copt_prob *, void *, int) { return 0; });
        const double t = cp::timed_solve(m);
        std::println(stderr, "ctx2 {:32} mask {:3} {} simplex={}", it.name, mk == -1 ? "0" : mask_name(mk), cp::summary(m, t), iattr(m, "SimplexIter"));
    }
}

// E4's instance: how many columns does presolve keep?
static void e4log() {
    M m; m.set_thread_limit(1); cp::build_atsp(m, 26, true); m.set_time_limit(duration<double>(40.0));
    set(m, {{"Logging", 1}});
    int cols_in_cb = -1;
    attach_raw(m, R | N, [&](c::copt_prob * p, void *, int ctx) { if(ctx == R) m.native_api().GetIntAttr(p, "Cols", &cols_in_cb); return 0; });
    const double t = cp::timed_solve(m);
    std::println(stderr, "e4log: {} model cols={} Cols read in the hook={}", cp::summary(m, t), m.num_variables(), cols_in_cb);
}

// Do callbacks of all four contexts overlap?
static void threads_all() {
    M m; cp::build_mkp(m, 30, 100); m.set_time_limit(duration<double>(8.0));
    std::atomic<int> inflight{0}, maxin{0}; std::atomic<long> over{0}; std::atomic<long> cnt[16];
    std::mutex mu; std::set<std::thread::id> tid;
    attach_raw(m, R | N | S | I, [&](c::copt_prob *, void *, int ctx) {
        const int now = inflight.fetch_add(1) + 1;
        if(now > 1) over.fetch_add(1);
        int seen = maxin.load(); while(now > seen && !maxin.compare_exchange_weak(seen, now)) {}
        { std::lock_guard l(mu); tid.insert(std::this_thread::get_id()); }
        cnt[ctx].fetch_add(1);
        const auto until = clk::now() + microseconds(2000); while(clk::now() < until) {}
        inflight.fetch_sub(1);
        return 0;
    });
    const double t = cp::timed_solve(m);
    std::println(stderr, "threads_all RNSI 2 ms spin: {} relax={} node={} sol={} inc={} os_threads={} overlaps={} max_inflight={}", cp::summary(m, t), cnt[R].load(), cnt[N].load(), cnt[S].load(), cnt[I].load(), tid.size(), over.load(), maxin.load());
}

// Why did the y3 cut hold in the sibling? Where it was added, and what later points look like.
static void dtrace(int argc, char ** argv) {
    const double eps = argc > 2 ? std::atof(argv[2]) : 1e-2;
    M m; scope_settings(m); auto g = cp::build_gadgets(m, 10, false); m.set_time_limit(duration<double>(20.0));
    const int y3 = g.y[3], z3 = g.za[3];
    bool added = false; long after = 0, v_0 = 0, c_0 = 0, v_1 = 0, c_1 = 0, y3nz = 0; cp::raw_cut cut;
    attach(m, [&](const std::vector<double> & x, const cp::add_fn & add) {
        const double v = x[static_cast<std::size_t>(y3)];
        if(!added) {
            if(v > 1e-3 && v < 1 - 1e-3) {
                cut = cp::raw_cut{{y3, z3}, {1.0, 0.01}, 'L', 0.0};
                cut.rhs = cut.activity(x) - eps;
                added = true;
                std::println(stderr, "added at y0={} y3={} za3={} rhs={} rc={}", g.state(x), v, x[static_cast<std::size_t>(z3)], cut.rhs, add(cut));
            }
            return;
        }
        ++after;
        const bool vv = cut.violation(x) > 1e-6;
        if(v > 1e-6) ++y3nz;
        (g.state(x) == 0 ? c_0 : c_1)++; (g.state(x) == 0 ? v_0 : v_1) += vv;
        if(after <= 25) std::println(stderr, "  later {} nodes_done={} y0={} y3={:.4f} za3={:.4f} viol={:.4f}", after, m.hook->node_calls.load(), g.state(x), v, x[static_cast<std::size_t>(z3)], cut.violation(x));
    }, "usercut");
    const double t = cp::timed_solve(m);
    std::println(stderr, "dtrace eps={}: {} y0=0 viol/checks={}/{} y0=1 viol/checks={}/{} later points with y3>0: {}", eps, cp::summary(m, t), v_0, c_0, v_1, c_1, y3nz);
}
// A real ladder: how many rounds does COPT grant, and which parameter limits them?
static void ladder2() {
    for(const params & p : {params{}, params{{"RootCutRounds", 3}}, params{{"RootCutRounds", 10}}, params{{"RootCutRounds", 50}}, params{{"RootCutRounds", 1000}}, params{{"RootCutLevel", 3}}, params{{"RootCutLevel", 3}, {"RootCutRounds", 1000}}, params{{"NodeCutRounds", 1}}, params{{"NodeCutRounds", 100}}, params{{"CutLevel", 0}, {"RootCutLevel", 1}, {"TreeCutLevel", 1}}}) {
        M m; m.set_thread_limit(1); auto a = cp::build_atsp(m, 26); m.set_time_limit(duration<double>(15.0)); set(m, p);
        std::string lab;
        for(auto [k, v] : p) lab += std::format(" {}={}", k, v);
        cp::raw_cut base; base.sense = 'G';
        for(int i = 0; i < a.n; ++i) for(int j = 0; j < a.n; ++j) if(i != j) { base.ind.push_back(a.col[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]); base.val.push_back(a.d[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]); }
        std::map<long, long> per; long adds = 0, root = 0; double last_root = NAN; long last_nd = -1, cnt = 0; long maxper = 0;
        attach(m, [&](const std::vector<double> & x, const cp::add_fn & add) {
            const long nd = m.hook->node_calls.load();
            if(nd != last_nd) { if(last_nd > 0) { per[cnt]++; maxper = std::max(maxper, cnt); } last_nd = nd; cnt = 0; }
            ++cnt;
            if(nd == 0) { ++root; last_root = a.objective(x); }
            auto c = base; c.rhs = a.objective(x) + 1.0;
            if(c.rhs < 549) { ++adds; add(c); }
        }, "usercut");
        const double t = cp::timed_solve(m);
        std::println(stderr, "ladder2{}: {} relax={} node={} adds={} root calls={} root obj at last call={:.2f} max calls on a tree node={}", lab, cp::summary(m, t), m.hook->relax_calls.load(), m.hook->node_calls.load(), adds, root, last_root, maxper);
    }
}
static void nodecnt2() {
    M m; m.set_thread_limit(1);
    auto a = cp::build_atsp(m, 26); m.set_time_limit(duration<double>(20.0));
    long nodes = 0, step = 0; std::string out;
    attach_raw(m, R | N, [&](c::copt_prob *, void * cb, int ctx) {
        if(ctx == N) ++nodes;
        if(ctx == N && nodes % 100 == 0) { int v = -1; m.native_api().GetCallbackInfo(cb, "NodeCnt", &v); out += std::format(" {}:{}", nodes, v); }
        return 0;
    });
    const double t = cp::timed_solve(m);
    std::println(stderr, "nodecnt2 (counted:info): {}\n {}", cp::summary(m, t), out);
}

// Nodes with many calls: a long cut loop, or a restart? The objective tells: a restart falls back to the root's.
static void fire2(int argc, char ** argv) {
    M m; m.set_thread_limit(1); auto a = cp::build_atsp(m, 26); m.set_time_limit(duration<double>(30.0));
    if(argc > 2) set(m, {{"NodeCutRounds", std::atoi(argv[2])}});
    set(m, {{"Logging", 1}});
    long last = -1; std::vector<double> objs; double first_root = NAN; long multi = 0;
    const auto flush = [&] {
        if(objs.size() >= 8) { std::string t; for(std::size_t i = 0; i < objs.size(); i += std::max<std::size_t>(1, objs.size() / 12)) t += std::format(" {:.2f}", objs[i]); std::println(stderr, "node index {} calls={} objective:{}", last, objs.size(), t); }
    };
    attach(m, [&](const std::vector<double> & x, const cp::add_fn &) {
        const long nd = m.hook->node_calls.load();
        if(nd != last) { flush(); last = nd; objs.clear(); }
        objs.push_back(a.objective(x));
    }, "usercut");
    const double t = cp::timed_solve(m);
    flush();
    std::println(stderr, "fire2: {}", cp::summary(m, t));
}
// The existing wrapper path, untouched: does an explicit zero in add_lazy_constraint hurt?
static void lazyzero() {
    for(const bool zero : {false, true}) {
        c::copt_milp m; m.set_thread_limit(1);
        const std::size_t n = 26, arcs = n * (n - 1);
        std::mt19937 rng(5);
        std::uniform_real_distribution<double> P(0, 100), noise(1.0, 1.3);
        std::vector<std::pair<double, double>> pts(n);
        for(auto & p : pts) p = {P(rng), P(rng)};
        std::vector<std::vector<double>> d(n, std::vector<double>(n, 0));
        for(std::size_t i = 0; i < n; ++i) for(std::size_t j = 0; j < n; ++j) if(i != j) d[i][j] = std::round(std::hypot(pts[i].first - pts[j].first, pts[i].second - pts[j].second) * noise(rng));
        const auto id = [n](std::size_t i, std::size_t j) { return i * (n - 1) + (j < i ? j : j - 1); };
        auto x = m.add_binary_variables(arcs);
        m.set_minimization();
        const auto nodes = std::views::iota(std::size_t{0}, n);
        m.set_objective(xsum(nodes, [&](std::size_t i) { return xsum(nodes | std::views::filter([i](std::size_t j) { return j != i; }), [&, i](std::size_t j) { return d[i][j] * x(id(i, j)); }); }));
        for(std::size_t i : nodes) {
            m.add_constraint(xsum(nodes | std::views::filter([i](std::size_t j) { return j != i; }), [&, i](std::size_t j) { return 1.0 * x(id(i, j)); }) == 1);
            m.add_constraint(xsum(nodes | std::views::filter([i](std::size_t j) { return j != i; }), [&, i](std::size_t j) { return 1.0 * x(id(j, i)); }) == 1);
        }
        m.set_time_limit(duration<double>(40.0));
        long added = 0;
        m.set_candidate_solution_callback([&](auto & h) {
            auto sol = h.get_solution();
            std::vector<int> comp(n, -1); int nc = 0;
            for(std::size_t s = 0; s < n; ++s) if(comp[s] < 0) {
                std::size_t at = s;
                while(comp[at] < 0) { comp[at] = nc; for(std::size_t j = 0; j < n; ++j) if(j != at && sol[x(id(at, j))] > 0.5) { at = j; break; } }
                ++nc;
            }
            if(nc == 1) return;
            for(int k = 0; k < nc; ++k) {
                std::size_t a0 = 0; while(comp[a0] != k) ++a0;
                std::size_t b0 = 0; while(comp[b0] == k) ++b0;
                // the expression names one arc twice with opposite signs when zero is set: it coalesces to 0
                auto out = xsum(nodes | std::views::filter([&](std::size_t i) { return comp[i] == k; }), [&](std::size_t i) { return xsum(nodes | std::views::filter([&](std::size_t j) { return comp[j] != k; }), [&, i](std::size_t j) { return 1.0 * x(id(i, j)); }); });
                if(zero) {
                    std::size_t a1 = a0 == 0 ? 1 : 0; if(comp[a1] != k) a1 = a0; // an arc inside the component, not in the cut
                    const std::size_t c0 = a0, c1 = [&] { for(std::size_t j = 0; j < n; ++j) if(j != a0 && comp[j] == k) return j; return b0; }();
                    h.add_lazy_constraint(out + 1.0 * x(id(c0, c1)) - 1.0 * x(id(c0, c1)) >= 1);
                } else h.add_lazy_constraint(out >= 1);
                ++added;
            }
        });
        const auto t0 = clk::now();
        try { m.solve(); } catch(const std::exception & e) { std::println(stderr, "solve threw {}", e.what()); }
        std::println(stderr, "lazyzero real wrapper, explicit zero={}: status={} value={} nodes={} lazy added={} in {:.2f}s (opt 549)", zero, cp::status_name(m.get_status()), m.get_solution_value(), m.get_node_count(), added, duration<double>(clk::now() - t0).count());
    }
}
static void objconst() {
    M m; scope_settings(m); auto g = cp::build_gadgets(m, 10, false); m.set_time_limit(duration<double>(10.0));
    m.native_api().SetObjConst(m.native_model().second, 1000.0);
    double maxd = 0; long calls = 0;
    attach_raw(m, R | N, [&](c::copt_prob *, void * cb, int ctx) {
        if(ctx != R) return 0;
        auto x = relax(*m.hook, cb);
        double o = NAN; m.native_api().GetCallbackInfo(cb, "RelaxSolObj", &o);
        maxd = std::max(maxd, std::fabs(o - g.objective(x) - 1000.0));
        if(++calls == 1) std::println(stderr, "objconst: first RelaxSolObj={} c'x={}", o, g.objective(x));
        return 0;
    });
    const double t = cp::timed_solve(m);
    std::println(stderr, "objconst: {} calls={} max|RelaxSolObj - (c'x + 1000)|={}", cp::summary(m, t), calls, maxd);
}

// Q9: at MIPNODE event 512 a "node" fires dozens of calls starting from the root's objective. Is it a
// restart, and do user cuts added before it survive? A few cuts in the first root round, then watch.
static void restart(int argc, char ** argv) {
    const int ncuts = argc > 2 ? std::atoi(argv[2]) : 3;
    const long rounds = argc > 3 ? std::atol(argv[3]) : 1;
    M m; m.set_thread_limit(1); auto a = cp::build_atsp(m, 26); m.set_time_limit(duration<double>(40.0));
    if(argc > 4) set(m, {{"NodeCutRounds", std::atoi(argv[4])}});
    std::vector<cp::raw_cut> added; long last = -1, cnt = 0, r = 0, shown = 0; double prev = NAN;
    long early_viol = 0, early_checks = 0, late_viol = 0, late_checks = 0;
    attach(m, [&](const std::vector<double> & x, const cp::add_fn & add) {
        const long nd = m.hook->node_calls.load();
        const double o = a.objective(x);
        if(nd == 0 && r < rounds) { ++r; for(const auto & c : a.separate(x, static_cast<std::size_t>(ncuts))) if(std::getenv("SHADOW") || !add(c)) added.push_back(c); return; }
        long v = 0; for(const auto & c : added) if(c.violation(x) > 1e-6) ++v;
        (nd < 512 ? early_viol : late_viol) += v ? 1 : 0;
        (nd < 512 ? early_checks : late_checks) += 1;
        if(nd != last) { last = nd; cnt = 0; }
        ++cnt;
        if(nd > 0 && o < prev - 25 && o < 475 && shown < 8) { ++shown; std::println(stderr, "objective fell back: MIPNODE events={} call {} on this index obj={:.3f} (previous call {:.3f}); {} of the {} early cuts violated", nd, cnt, o, prev, v, added.size()); }
        prev = o;
    }, "usercut");
    const double t = cp::timed_solve(m);
    std::println(stderr, "restart: {} relax={} node={} early cuts={} calls with an early cut violated: before event 512 {}/{}, after {}/{}", cp::summary(m, t), m.hook->relax_calls.load(), m.hook->node_calls.load(), added.size(), early_viol, early_checks, late_viol, late_checks);
}

int main(int argc, char ** argv) {
    const std::string e = argc > 1 ? argv[1] : "";
    if(e == "restart") { restart(argc, argv); return 0; }
    if(e == "fire2") { fire2(argc, argv); return 0; }
    if(e == "lazyzero") { lazyzero(); return 0; }
    if(e == "objconst") { objconst(); return 0; }
    if(e == "dtrace") { dtrace(argc, argv); return 0; }
    if(e == "ladder2") { ladder2(); return 0; }
    if(e == "nodecnt2") { nodecnt2(); return 0; }
    if(e == "contexts2") { contexts2(); return 0; }
    if(e == "nodecnt") { nodecnt(); return 0; }
    if(e == "clearcut") { clearcut(); return 0; }
    if(e == "ctx2") { ctx2(); return 0; }
    if(e == "e4log") { e4log(); return 0; }
    if(e == "threads_all") { threads_all(); return 0; }
    if(e == "ctx") ctx();
    else if(e == "reads") reads(argc, argv);
    else if(e == "objcheck") objcheck();
    else if(e == "contexts") contexts();
    else if(e == "s1b") s1b(argc, argv);
    else if(e == "rootadd") rootadd();
    else if(e == "dominance") dominance();
    else if(e == "discard") discard();
    else if(e == "burst") burst();
    else if(e == "purge") purge(argc, argv);
    else if(e == "fire") fire();
    else if(e == "threads") threads();
    else if(e == "lazy") lazy(argc, argv);
    else if(e == "persist") persist();
    else if(e == "pre") pre();
    else if(e == "ret") ret();
    else if(e == "candidates") candidates(argc, argv);
    else { std::println(stderr, "unknown experiment"); return 2; }
}
