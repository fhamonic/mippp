// Experiments the shared matrix does not have. One subcommand per question.
#define PROBE_SCIP_NO_MAIN
#include "probe_scip.cpp"
using std::chrono::duration;
using cp::raw_cut;

static void TL(M & m, double l) { m.set_time_limit(duration<double>(l)); }
static std::string line_for(M & m, double t) { return cp::summary(m, t); }

// ---------------------------------------------------------------- firing -----
// Q7: what the separator's own parameters and the global limits do to a hook that
// never adds a cut, on a tree of known shape (gadgets) and on the default ATSP.
static void firing() {
    struct cfg { std::string name; sepa_params p; std::vector<std::pair<const char *, int>> global; bool sep_off_after = false; };
    const std::vector<cfg> cfgs = {
        {"freq=1 expbackoff=default(4) maxbounddist=1", {1000000, 1, 0, 1.0, false}, {}},
        {"freq=1 expbackoff=1 maxbounddist=1", {1000000, 1, 1, 1.0, false}, {}},
        {"freq=1 expbackoff=1 maxbounddist=0", {1000000, 1, 1, 0.0, false}, {}},
        {"freq=0 (root only)", {1000000, 0, 1, 1.0, false}, {}},
        {"freq=3 expbackoff=1", {1000000, 3, 1, 1.0, false}, {}},
        {"freq=1 expbackoff=1 delay=TRUE", {1000000, 1, 1, 1.0, true}, {}},
        {"freq=1 expbackoff=1 priority=-1000000", {-1000000, 1, 1, 1.0, false}, {}},
        {"freq=-1", {1000000, -1, 1, 1.0, false}, {}},
        {"+ SCIPsetSeparating(OFF) after the include", {1000000, 1, 1, 1.0, false}, {}, true},
        {"+ separating/maxrounds=0", {1000000, 1, 1, 1.0, false}, {{"separating/maxrounds", 0}}},
        {"+ separating/maxroundsroot=0", {1000000, 1, 1, 1.0, false}, {{"separating/maxroundsroot", 0}}},
        {"+ separating/maxcuts=0 maxcutsroot=0", {1000000, 1, 1, 1.0, false}, {{"separating/maxcuts", 0}, {"separating/maxcutsroot", 0}}},
        {"+ separating/maxruns=0", {1000000, 1, 1, 1.0, false}, {{"separating/maxruns", 0}}},
    };
    for(const auto & c : cfgs) {
        M m; scope_settings(m); auto g = cp::build_gadgets(m, 10, false); TL(m, 30);
        auto s = std::make_shared<cp::stats>();
        ctx * h = attach_with(m, cp::watched([](const auto &, const auto &) {}, s), "row_g", c.p);
        h->mode = c.name;
        for(auto [p, v] : c.global) setint(m.native_model(), p, v, m.native_api());
        if(c.sep_off_after) N.setSeparating(m.native_model(), PS_OFF, 1);
        const double t = cp::timed_solve(m);
        std::println(stderr, "== firing gadgets [{}]: {}\n   hook calls={}", c.name, line_for(m, t), s->calls.load());
    }
    for(int eb : {0, 1}) {
        M m; auto a = cp::build_atsp(m, 26); TL(m, 60);
        auto s = std::make_shared<cp::stats>();
        ctx * h = attach_with(m, cp::watched([](const auto &, const auto &) {}, s, {}, {}, [&a](const auto & x) { return a.integral(x); }), "row_g", {1000000, 1, eb, 1.0, false});
        h->mode = std::format("atsp expbackoff={}", eb ? "1" : "default");
        const double t = cp::timed_solve(m);
        std::println(stderr, "== firing atsp default settings expbackoff={}: {}\n   hook calls={} on integral points={}", eb ? "1" : "default(4)", line_for(m, t), s->calls.load(), s->integral.load());
    }
    {   // parameter values on this release
        M m; auto s = m.native_model(); auto & api = m.native_api();
        std::string out;
        for(const char * p : {"separating/maxrounds", "separating/maxroundsroot", "separating/maxstallrounds", "separating/maxstallroundsroot", "separating/maxaddrounds", "separating/maxcuts", "separating/maxcutsroot", "separating/poolfreq", "separating/cutagelimit", "lp/rowagelimit", "separating/maxruns"}) {
            int v = -999; const int rc = api.getIntParam(s, p, &v);
            out += std::format(" {}={}{}", p, v, rc == RC_OKAY ? "" : " (missing)");
        }
        for(const char * p : {"separating/maxbounddist", "separating/maxlocalbounddist", "separating/minefficacy", "separating/minefficacyroot", "separating/minactivityquot", "numerics/feastol"}) {
            double v = -999; const int rc = api.getRealParam(s, p, &v);
            out += std::format(" {}={}{}", p, v, rc == RC_OKAY ? "" : " (missing)");
        }
        std::println(stderr, "== defaults on this release:{}", out);
    }
}

// ---------------------------------------------------------------- rounds -----
// Q7: rounds at a non-root node. Cuts only below the root, so a tree exists.
static void rounds() {
    const std::vector<std::pair<std::string, std::vector<std::pair<const char *, int>>>> cfgs = {
        {"defaults", {}},
        {"separating/maxrounds=1", {{"separating/maxrounds", 1}}},
        {"separating/maxrounds=3", {{"separating/maxrounds", 3}}},
        {"separating/maxstallrounds=-1", {{"separating/maxstallrounds", -1}}},
        {"separating/maxaddrounds=-1", {{"separating/maxaddrounds", -1}}},
    };
    for(const auto & [name, params] : cfgs) {
        M m; auto a = cp::build_atsp(m, 26); TL(m, 60);
        void * s = m.native_model();
        auto st = std::make_shared<cp::stats>();
        ctx * h = attach_with(m, cp::watched([&a, s](const auto & x, const cp::add_fn & add) {
            if(N.getDepth(s) == 0) return;
            for(const auto & c : a.separate(x)) add(c);
        }, st, [&a](const auto & x) { return a.objective(x); }, [&a](const auto & x) { return a.point_ok(x); }, [&a](const auto & x) { return a.integral(x); }), "row_l");
        h->mode = "rounds " + name;
        for(auto [p, v] : params) setint(s, p, v, m.native_api());
        const double t = cp::timed_solve(m);
        std::println(stderr, "== rounds at non-root nodes [{}]: {}\n   {}", name, line_for(m, t), st->line());
    }
    {   // integral points, no incumbent to cut the node off first
        M m; auto a = cp::build_atsp(m, 26); TL(m, 60);
        void * s = m.native_model();
        N.lib.clear(); load_natives(m);
        N.setHeuristics(s, PS_OFF, 1);
        auto st = std::make_shared<cp::stats>();
        ctx * h = attach_with(m, cp::watched([&a](const auto & x, const cp::add_fn & add) { for(const auto & c : a.separate(x)) add(c); }, st,
            [&a](const auto & x) { return a.objective(x); }, [&a](const auto & x) { return a.point_ok(x); }, [&a](const auto & x) { return a.integral(x); }), "row_g");
        h->mode = "dfj, heuristics off";
        const double t = cp::timed_solve(m);
        std::println(stderr, "== E3 with heuristics off, so an integral LP point reaches the hook: {}\n   {}", line_for(m, t), st->line());
    }
}

// -------------------------------------------------------------------- e4 -----
// Q5: original pointer, transformed pointer, or active variable by hand.
static void e4() {
    for(int map : {0, 1, 2, 3}) {
        M m; auto a = cp::build_atsp(m, 26, true); TL(m, 60);
        auto s = std::make_shared<cp::stats>();
        ctx * h = attach_with(m, cp::watched([&a](const auto & x, const cp::add_fn & add) { for(const auto & c : a.separate(x)) add(c); }, s,
            [&a](const auto & x) { return a.objective(x); }, [&a](const auto & x) { return a.point_ok(x); }, [&a](const auto & x) { return a.integral(x); }), "row_g");
        h->var_map = map;
        h->mode = std::format("var_map={}", map);
        bool once = false;
        M * mp = &m;
        h->inspect = [&once, &a, mp](void * sp, SEPA *, unsigned, int) {
            if(once) return;
            once = true;
            std::map<int, int> xs, cs;
            double maxdiff_orig_trans = 0, maxdiff_vals = 0;
            int null_trans = 0;
            for(int i = 0; i < a.n; ++i) for(int j = 0; j < a.n; ++j) if(i != j) {
                const auto I = static_cast<std::size_t>(i); const auto J = static_cast<std::size_t>(j);
                for(int col : {a.col[I][J], a.comp[I][J]}) {
                    SCIP_VAR * o = mp->var_of(col); SCIP_VAR * t = nullptr;
                    N.getTransformedVar(sp, o, &t);
                    if(!t) { ++null_trans; continue; }
                    (col == a.col[I][J] ? xs : cs)[N.varGetStatus(t)]++;
                    maxdiff_orig_trans = std::max(maxdiff_orig_trans, std::fabs(N.getVarSol(sp, o) - N.getVarSol(sp, t)));
                }
            }
            const auto show = [](const std::map<int, int> & h2) { std::string r; for(auto [k, v] : h2) r += std::format(" {}:{}", k, v); return r; };
            std::println(stderr, "   first call: status of transformed x (2=COLUMN 4=AGGREGATED 6=NEGATED):{} complements:{} null transformed={} max|getVarSol(orig)-getVarSol(trans)|={}", show(xs), show(cs), null_trans, maxdiff_orig_trans);
        };
        const double t = cp::timed_solve(m);
        std::println(stderr, "== E4 var_map={} (0 orig pointer, 1 SCIPgetTransformedVar, 2 SCIPgetProbvarSum by hand, 3 SCIPaddVarsToRow): {} (baseline 549)\n   {}", map, line_for(m, t), s->line());
    }
}

// ------------------------------------------------------------- nodereads -----
// Q12: what the hook can read about the node, on original and transformed pointers.
static void nodereads() {
    {
        M m; auto a = cp::build_atsp(m, 26, true); TL(m, 10);
        auto s = std::make_shared<cp::stats>();
        ctx * h = attach_with(m, cp::watched([](const auto &, const auto &) {}, s), "row_g");
        h->mode = "nodereads atsp";
        long checked = 0, orig_stale = 0, compl_stale = 0, lp_outside = 0, depth_arg_mismatch = 0, deep = 0;
        M * mp = &m;
        h->inspect = [&, mp](void * sp, SEPA *, unsigned, int depth) {
            if(depth != N.getDepth(sp)) ++depth_arg_mismatch;
            if(N.getDepth(sp) == 0) return;
            ++deep;
            for(int i = 0; i < a.n; ++i) for(int j = 0; j < a.n; ++j) if(i != j) {
                const auto I = static_cast<std::size_t>(i); const auto J = static_cast<std::size_t>(j);
                SCIP_VAR * ox = mp->var_of(a.col[I][J]); SCIP_VAR * oc = mp->var_of(a.comp[I][J]);
                SCIP_VAR * tx = nullptr; SCIP_VAR * tc = nullptr;
                N.getTransformedVar(sp, ox, &tx); N.getTransformedVar(sp, oc, &tc);
                ++checked;
                if(N.varGetLbLocal(ox) != N.varGetLbLocal(tx) || N.varGetUbLocal(ox) != N.varGetUbLocal(tx)) ++orig_stale;
                if(std::fabs(N.varGetLbLocal(tx) + N.varGetUbLocal(tc) - 1) > 1e-9 || std::fabs(N.varGetUbLocal(tx) + N.varGetLbLocal(tc) - 1) > 1e-9) ++compl_stale;
                const double v = N.getVarSol(sp, tx);
                if(v < N.varGetLbLocal(tx) - 1e-6 || v > N.varGetUbLocal(tx) + 1e-6) ++lp_outside;
            }
        };
        const double t = cp::timed_solve(m);
        std::println(stderr, "== nodereads atsp+complements, default presolve, no cuts: {}\n   calls below the root={} columns checked={} local bound differs on the ORIGINAL pointer={} x vs 1-xbar local bounds differ={} LP value outside transformed local bounds={} depth argument != SCIPgetDepth={}",
            line_for(m, t), deep, checked, orig_stale, compl_stale, lp_outside, depth_arg_mismatch);
    }
    for(const char * mode : {"row_l", "row_g"}) {   // a local cut decided from the local bounds, not the LP value
        M m; scope_settings(m); auto g = cp::build_gadgets(m, 10, false); TL(m, 30);
        auto s = std::make_shared<cp::stats>();
        M * mp = &m;
        void * sp = m.native_model();
        bool added = false;
        long agree = 0, disagree = 0;
        std::string first;
        const raw_cut cut = g.invalid_global_cut();
        ctx * h = attach_with(m, cp::watched([&](const auto & x, const cp::add_fn & add) {
            SCIP_VAR * ty = nullptr;
            N.getTransformedVar(sp, mp->var_of(g.y[0]), &ty);
            const double lb = N.varGetLbLocal(ty), ub = N.varGetUbLocal(ty);
            const int by_bounds = lb > 0.5 ? 1 : ub < 0.5 ? 0 : -1;
            if(by_bounds == g.state(x)) ++agree; else ++disagree;
            if(!added && by_bounds == g.worse_state() && cut.violation(x) > 1e-4) {
                added = true;
                auto * node = N.getCurrentNode(sp);
                first = std::format("depth={} node={} y0 local=[{},{}] orig=[{},{}] global=[{},{}]", N.getDepth(sp), N.nodeGetNumber(node), lb, ub,
                    N.varGetLbLocal(mp->var_of(g.y[0])), N.varGetUbLocal(mp->var_of(g.y[0])), N.varGetLbGlobal(ty), N.varGetUbGlobal(ty));
                add(cut);
            }
        }, s), mode);
        h->mode = std::string("nodereads gadgets ") + mode;
        const double t = cp::timed_solve(m);
        double v = NAN; try { v = m.get_solution_value(); } catch(const std::exception &) {}
        std::println(stderr, "== local cut triggered by the transformed local bounds, mode {}: {} (known opt {}) => {}\n   at add: {} | state from bounds agrees with LP point {} times, differs {}", mode, line_for(m, t), g.opt, std::fabs(v - g.opt) < 1e-4 ? "KEPT" : "LOST", first, agree, disagree);
    }
}

// --------------------------------------------------------------- restart -----
// Q9: SCIPrestartSolve after a cut. Does the next run still have it?
static void restart() {
    struct cfg { std::string mode; int depth; bool invalid; };
    const std::vector<cfg> cfgs = {
        {"row_g", 0, false}, {"row_g_force", 0, false}, {"row_l", 0, false}, {"pool_g", 0, false}, {"row_force+pool", 0, false},
        {"row_g", 1, false}, {"row_l", 1, false}, {"pool_g", 1, false}, {"pool_g_freq1", 1, false}, {"row_force+pool", 1, false},
        {"row_g", 1, true}, {"row_l", 1, true}, {"pool_g", 1, true}, {"row_force+pool", 1, true}, {"row_force+pool_freq1", 1, true},
    };
    for(const auto & c : cfgs) {
        M m; scope_settings(m); auto g = cp::build_gadgets(m, 10, false); TL(m, 30);
        void * sp = m.native_model();
        setint(sp, "presolving/maxrestarts", -1, m.native_api());
        setint(sp, "presolving/maxrounds", 0, m.native_api());
        auto s = std::make_shared<cp::stats>();
        const raw_cut cut = c.invalid ? g.invalid_global_cut() : g.hull_cut(3);
        bool added = false;
        long calls2 = 0, viol2 = 0; double first2 = NAN; int conss1 = 0, conss2 = -1;
        ctx * h = attach_with(m, cp::watched([&](const auto & x, const cp::add_fn & add) {
            if(N.getNRuns(sp) >= 2) {
                ++calls2;
                if(conss2 < 0) { conss2 = N.getNConss(sp); first2 = cut.violation(x); }
                viol2 += cut.violation(x) > 1e-6;
                return;
            }
            const bool here = N.getDepth(sp) == c.depth && cut.violation(x) > 1e-4 && (!c.invalid || g.state(x) == g.worse_state());
            if(!added && here) {
                added = true;
                conss1 = N.getNConss(sp);
                const int e = add(cut);
                if(e) std::println(stderr, "   add error {}", e);
                const int rc = N.restartSolve(sp);
                if(rc != RC_OKAY) std::println(stderr, "   restart retcode {}", rc);
            }
        }, s), c.mode);
        h->mode = std::format("restart {} depth {}", c.mode, c.depth);
        const double t = cp::timed_solve(m);
        double v = NAN; try { v = m.get_solution_value(); } catch(const std::exception &) {}
        std::println(stderr, "== restart mode={} added at depth {} cut={}: {} => {} | run 2: first LP violation={:.4g} viol/checks={}/{} active conss {} -> {}", c.mode, c.depth, c.invalid ? "valid-only-in-worse-subtree" : "hull(globally valid)",
            line_for(m, t), std::fabs(v - g.opt) < 1e-4 ? "optimum KEPT" : "optimum LOST", first2, viol2, calls2, conss1, conss2);
    }
}

// ----------------------------------------------------------------- mixed -----
// Q11: DFJ with degree constraints only. Lazy subtours through a constraint handler,
// user cuts through the separator.
struct lazy_ctx {
    cp::atsp * a; const sc::scip_api * api; SCIP_VAR ** vars; int cols;
    int (*createRowConshdlr)(void *, ROW **, sc::SCIP_CONSHDLR *, const char *, double, double, unsigned, unsigned, unsigned);
    int (*addLocks)(void *, SCIP_VAR *, int, int, int);
    long enfolp = 0, enfolp_frac = 0, enfolp_cuts = 0, enfops = 0, check = 0, check_rejected = 0, lock = 0;
};
static lazy_ctx * current_lazy = nullptr;
static int lazy_cuts(lazy_ctx & c, sc::SCIP * s, sc::SCIP_CONSHDLR * h, const std::vector<raw_cut> & cuts, SCIP_RESULT * result) {
    bool infeasible = false;
    for(const auto & r : cuts) {
        ROW * row = nullptr;
        int rc = c.createRowConshdlr(s, &row, h, "lazy", r.rhs, c.api->infinity(s), 0, 0, 1);
        if(rc != RC_OKAY) return rc;
        N.cacheRow(s, row);
        for(std::size_t k = 0; k < r.ind.size(); ++k) N.addVarToRow(s, row, c.vars[static_cast<std::size_t>(r.ind[k])], r.val[k]);
        N.flushRow(s, row);
        unsigned inf = 0;
        rc = N.addRow(s, row, 1, &inf);
        N.releaseRow(s, &row);
        if(rc != RC_OKAY) return rc;
        infeasible = infeasible || inf;
        ++c.enfolp_cuts;
    }
    *result = infeasible ? sc::SCIP_CUTOFF : sc::SCIP_SEPARATED;
    return RC_OKAY;
}
static std::vector<double> lazy_point(lazy_ctx & c, sc::SCIP * s, sc::SCIP_SOL * sol) {
    std::vector<double> x(static_cast<std::size_t>(c.cols));
    c.api->getSolVals(s, sol, c.cols, c.vars, x.data());
    return x;
}
static sc::SCIP_RETCODE lazy_enfolp(sc::SCIP * s, sc::SCIP_CONSHDLR * h, sc::SCIP_CONS **, int, int, unsigned, SCIP_RESULT * result) {
    auto & c = *current_lazy;
    ++c.enfolp;
    const auto x = lazy_point(c, s, nullptr);
    if(!c.a->integral(x)) ++c.enfolp_frac;
    const auto cuts = c.a->separate(x);
    *result = sc::SCIP_FEASIBLE;
    if(cuts.empty()) return sc::SCIP_OKAY;
    return static_cast<sc::SCIP_RETCODE>(lazy_cuts(c, s, h, cuts, result));
}
static sc::SCIP_RETCODE lazy_enfops(sc::SCIP * s, sc::SCIP_CONSHDLR *, sc::SCIP_CONS **, int, int, unsigned, unsigned, SCIP_RESULT * result) {
    auto & c = *current_lazy;
    ++c.enfops;
    const auto x = lazy_point(c, s, nullptr);
    *result = c.a->separate(x).empty() ? sc::SCIP_FEASIBLE : sc::SCIP_SOLVELP;
    return sc::SCIP_OKAY;
}
static sc::SCIP_RETCODE lazy_check(sc::SCIP * s, sc::SCIP_CONSHDLR *, sc::SCIP_CONS **, int, sc::SCIP_SOL * sol, unsigned, unsigned, unsigned, unsigned, SCIP_RESULT * result) {
    auto & c = *current_lazy;
    ++c.check;
    const auto x = lazy_point(c, s, sol);
    const bool bad = !c.a->separate(x).empty();
    if(bad) ++c.check_rejected;
    *result = bad ? sc::SCIP_INFEASIBLE : sc::SCIP_FEASIBLE;
    return sc::SCIP_OKAY;
}
static sc::SCIP_RETCODE lazy_lock(sc::SCIP * s, sc::SCIP_CONSHDLR *, sc::SCIP_CONS *, sc::SCIP_LOCKTYPE locktype, int nlockspos, int nlocksneg) {
    auto & c = *current_lazy;
    ++c.lock;
    for(int j = 0; j < c.cols; ++j) {
        const int rc = c.addLocks(s, c.vars[j], static_cast<int>(locktype), nlockspos + nlocksneg, nlockspos + nlocksneg);
        if(rc != RC_OKAY) return static_cast<sc::SCIP_RETCODE>(rc);
    }
    return sc::SCIP_OKAY;
}
// Same distances as cp::build_atsp, so the same optimum, without the MTZ rows.
static cp::atsp build_degree_only(M & m, const int n, const unsigned seed = 5) {
    cp::atsp a;
    a.n = n;
    const auto N_ = static_cast<std::size_t>(n);
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> P(0, 100), noise(1.0, 1.3);
    std::vector<std::pair<double, double>> pts(N_);
    for(auto & p : pts) p = {P(rng), P(rng)};
    a.d.assign(N_, std::vector<double>(N_, 0));
    for(std::size_t i = 0; i < N_; ++i) for(std::size_t j = 0; j < N_; ++j) if(i != j)
        a.d[i][j] = std::round(std::hypot(pts[i].first - pts[j].first, pts[i].second - pts[j].second) * noise(rng));
    const auto arcs = N_ * (N_ - 1);
    const auto id = [N_](std::size_t i, std::size_t j) { return i * (N_ - 1) + (j < i ? j : j - 1); };
    auto x = m.add_binary_variables(arcs);
    a.col.assign(N_, std::vector<int>(N_, -1));
    a.comp.assign(N_, std::vector<int>(N_, -1));
    for(std::size_t i = 0; i < N_; ++i) for(std::size_t j = 0; j < N_; ++j) if(i != j) a.col[i][j] = m.native_id(x(id(i, j)));
    m.set_minimization();
    const auto nodes = std::views::iota(std::size_t{0}, N_);
    m.set_objective(xsum(nodes, [&](std::size_t i) {
        return xsum(nodes | std::views::filter([i](std::size_t j) { return j != i; }), [&, i](std::size_t j) { return a.d[i][j] * x(id(i, j)); });
    }));
    for(std::size_t i : nodes) {
        m.add_constraint(xsum(nodes | std::views::filter([i](std::size_t j) { return j != i; }), [&, i](std::size_t j) { return 1.0 * x(id(i, j)); }) == 1);
        m.add_constraint(xsum(nodes | std::views::filter([i](std::size_t j) { return j != i; }), [&, i](std::size_t j) { return 1.0 * x(id(j, i)); }) == 1);
    }
    return a;
}
static void mixed() {
    const int size = std::getenv("MIXED_N") ? std::atoi(std::getenv("MIXED_N")) : 20;
    const unsigned seed = std::getenv("MIXED_SEED") ? static_cast<unsigned>(std::atoi(std::getenv("MIXED_SEED"))) : 5;
    const double tour = size > 26 ? NAN : [size, seed] { M m; auto a = cp::build_atsp(m, size, false, seed); TL(m, 60); cp::timed_solve(m); std::println(stderr, "== MTZ n={} reference: {}", size, cp::status_name(m.get_status())); return m.get_solution_value(); }();
    struct cfg { std::string name; bool lazy; const char * sepa_mode; bool fractional_only; };
    const std::vector<cfg> cfgs = {
        {"lazy conshdlr only", true, nullptr, false},
        {"lazy conshdlr + separator row_g on fractional points", true, "row_g", true},
        {"lazy conshdlr + separator row_l on fractional points", true, "row_l", true},
        {"separator row_g alone, all points", false, "row_g", false},
        {"separator row_g_force alone, all points", false, "row_g_force", false},
        {"separator cons_g_model alone, all points", false, "cons_g_model", false},
    };
    for(const auto & c : cfgs) {
        // Declared before the model: SCIP calls the lock callback again when it frees it.
        lazy_ctx l{};
        cp::atsp a;
        std::vector<SCIP_VAR *> own_vars;
        M m; a = build_degree_only(m, size, seed); TL(m, 60);
        load_natives(m);
        auto & api = m.native_api();
        // A private copy: ~scip_milp nulls _scip_vars through SCIPreleaseVar before
        // SCIPfree, and SCIPfree runs the lock callback once more. MIXED_NOCOPY shows it.
        own_vars.assign(m.vars(), m.vars() + m.num_variables());
        l.a = &a; l.api = &api; l.vars = std::getenv("MIXED_NOCOPY") ? m.vars() : own_vars.data(); l.cols = static_cast<int>(m.num_variables());
        l.createRowConshdlr = cp::symf<decltype(l.createRowConshdlr)>(N.lib, "SCIPcreateEmptyRowConshdlr");
        l.addLocks = cp::symf<decltype(l.addLocks)>(N.lib, "SCIPaddVarLocksType");
        current_lazy = &l;
        if(c.lazy) {
            sc::SCIP_CONSHDLR * h = nullptr;
            const int rc = api.includeConshdlrBasic(m.native_model(), &h, "probe_lazy", "lazy subtours", -2000000, -2000000, -1, 0, lazy_enfolp, lazy_enfops, lazy_check, lazy_lock, nullptr);
            if(rc != RC_OKAY) { std::println(stderr, "include conshdlr: retcode {}", rc); std::exit(3); }
        }
        auto s = std::make_shared<cp::stats>();
        if(c.sepa_mode) {
            ctx * h = attach_with(m, cp::watched([&a, frac = c.fractional_only](const auto & x, const cp::add_fn & add) {
                if(frac && a.integral(x)) return;
                for(const auto & r : a.separate(x)) add(r);
            }, s, [&a](const auto & x) { return a.objective(x); }, {}, [&a](const auto & x) { return a.integral(x); }), c.sepa_mode);
            h->mode = c.name;
        }
        const double t = cp::timed_solve(m);
        // The tour check is on the returned point: a heuristic solution with subtours is wrong.
        std::string verdict = "no solution";
        try {
            const auto sol = m.get_solution();
            std::vector<double> x(static_cast<std::size_t>(l.cols));
            for(int j = 0; j < l.cols; ++j) x[static_cast<std::size_t>(j)] = api.getSolVal(m.native_model(), api.getBestSol(m.native_model()), m.vars()[j]);
            verdict = a.separate(x).empty() ? "returned point is a tour" : "returned point HAS SUBTOURS";
        } catch(const std::exception &) {}
        std::println(stderr, "== mixed [{}]: {} (tour optimum {}) => {} | lazy: lock={} enfolp={} (fractional {}) cuts={} enfops={} check={} rejected={} | sepa: {}", c.name, line_for(m, t), tour, verdict, l.lock, l.enfolp, l.enfolp_frac, l.enfolp_cuts, l.enfops, l.check, l.check_rejected, s->line());
    }
}

// ---------------------------------------------------------------- filter -----
// Q4: one cut at the root whose violation we choose. Non-forced against forced.
static void filter() {
    for(const char * mode : {"row_g", "row_g_force"}) for(double delta : {1e-1, 1e-3, 1e-4, 1e-5, 1e-6, 1e-7, 1e-8}) {
        M m; scope_settings(m); auto g = cp::build_gadgets(m, 10, false); TL(m, 30);
        auto s = std::make_shared<cp::stats>();
        raw_cut base = g.hull_cut(3);
        bool once = false;
        double applied_rhs = 0;
        long later = 0, later_viol = 0;
        std::shared_ptr<raw_cut> cut;
        ctx * h = attach_with(m, cp::watched([&](const auto & x, const cp::add_fn & add) {
            if(!once) {
                once = true;
                cut = std::make_shared<raw_cut>(base);
                cut->rhs = base.activity(x) - delta;  // violated by exactly delta
                applied_rhs = cut->rhs;
                add(*cut);
                return;
            }
            ++later;
            later_viol += cut->violation(x) > delta * 0.5;
        }, s), mode);
        h->mode = std::format("{} delta={}", mode, delta);
        const double t = cp::timed_solve(m);
        std::println(stderr, "== filter mode={} violation={} at the root: {} | later calls violating it {}/{}", mode, delta, line_for(m, t), later_viol, later);
    }
    for(const char * mode : {"row_g", "row_g_force"}) {
        // at most so many of the cuts of one round
        M m; auto a = cp::build_atsp(m, 26); TL(m, 60);
        void * sp = m.native_model();
        auto s = std::make_shared<cp::stats>();
        ctx * h = attach_with(m, cp::watched([&a](const auto & x, const cp::add_fn & add) { for(const auto & c : a.separate(x, 50)) add(c); }, s), mode);
        h->mode = std::format("atsp many cuts per round {} maxcutsroot=2", mode);
        setint(sp, "separating/maxcutsroot", 2, m.native_api());
        const double t = cp::timed_solve(m);
        std::println(stderr, "== filter atsp, up to 50 cuts a round, separating/maxcutsroot=2, mode={}: {}\n   {}", mode, line_for(m, t), s->line());
    }
}

// ----------------------------------------------------------------- aging -----
// Q4: do rows stay in the LP? Rows captured for the whole root phase.
static void aging() {
    auto isInLP = static_cast<unsigned (*)(ROW *)>(nullptr);
    auto getAge = static_cast<int (*)(ROW *)>(nullptr);
    auto capture = static_cast<int (*)(void *, ROW *)>(nullptr);
    for(const bool removable : {true, false}) for(const bool force : {false, true}) {
        M m; auto a = cp::build_atsp(m, 26); TL(m, 60);
        load_natives(m);
        isInLP = cp::symf<decltype(isInLP)>(N.lib, "SCIProwIsInLP");
        getAge = cp::symf<decltype(getAge)>(N.lib, "SCIProwGetAge");
        capture = cp::symf<decltype(capture)>(N.lib, "SCIPcaptureRow");
        void * sp = m.native_model();
        auto & api = m.native_api();
        auto s = std::make_shared<cp::stats>();
        std::vector<ROW *> rows;
        long entered = 0, left = 0;
        std::map<ROW *, bool> seen_in;
        int maxage = 0;
        ctx * h = nullptr;
        M * mp = &m;
        h = attach_with(m, cp::watched([&](const auto & x, const cp::add_fn &) {
            for(ROW * r : rows) {
                const bool in = isInLP(r);
                auto & was = seen_in[r];
                if(in && !was) { was = true; ++entered; }
                if(!in && was) { was = false; ++left; }
                maxage = std::max(maxage, getAge(r));
            }
            for(const auto & c : a.separate(x)) {
                ROW * row = nullptr;
                N.createEmptyRowSepa(sp, &row, h->sepa, "held", c.rhs, api.infinity(static_cast<Scip *>(sp)), 0, 0, removable);
                N.cacheRow(sp, row);
                for(std::size_t k = 0; k < c.ind.size(); ++k) N.addVarToRow(sp, row, mp->var_of(c.ind[k]), c.val[k]);
                N.flushRow(sp, row);
                unsigned inf = 0;
                N.addRow(sp, row, force, &inf);
                rows.push_back(row);  // keep the creation capture
            }
        }, s), "row_g");
        h->mode = std::format("aging removable={} force={}", removable, force);
        h->on_exit = [&](void * q) {
            long in = 0;
            for(ROW * r : rows) { in += isInLP(r) != 0; N.releaseRow(q, &r); }
            std::println(stderr, "   rows created={} entered the LP at some call={} left it={} max age seen={} still in the LP at exit={}", rows.size(), entered, left, maxage, in);
            rows.clear();
        };
        const double t = cp::timed_solve(m);
        std::println(stderr, "== aging removable={} force={}: {}\n   {}", removable, force, line_for(m, t), s->line());
        api.freeTransform(m.native_model());  // runs the exit callback while the locals above live
    }
}

// ------------------------------------------------------------- lifecycle -----
// Q14: when a plugin can be included, a dormant plugin, a moved model, resuming, threads.
static void lifecycle() {
    {   // include after a solve
        M m; auto a = cp::build_atsp(m, 10); TL(m, 20);
        cp::timed_solve(m);
        load_natives(m);
        SEPA * sepa = nullptr;
        ctx dummy;
        int rc = N.includeSepaBasic(m.native_model(), &sepa, "late", "late", 0, 1, 1.0, 0, 0, execlp, nullptr, &dummy);
        std::println(stderr, "== include a separator at stage SOLVED: retcode {} (1 is OKAY, -8 INVALIDCALL)", rc);
        rc = m.native_api().freeTransform(m.native_model());
        std::println(stderr, "   SCIPfreeTransform: retcode {}", rc);
        rc = N.includeSepaBasic(m.native_model(), &sepa, "late2", "late2", 0, -1, 1.0, 0, 0, execlp, nullptr, &dummy);
        std::println(stderr, "   include after SCIPfreeTransform: retcode {}", rc);
    }
    {   // a dormant plugin must not change the path, and can be woken by parameter
        M a0; auto a = cp::build_atsp(a0, 26); TL(a0, 60);
        const double t0 = cp::timed_solve(a0);
        std::println(stderr, "== dormant: no plugin: {}", line_for(a0, t0));
        M b0; auto b = cp::build_atsp(b0, 26); TL(b0, 60);
        auto s = std::make_shared<cp::stats>();
        ctx * h = attach_with(b0, cp::watched([](const auto &, const auto &) {}, s), "row_g", {1000000, -1, 0, 1.0, false});
        h->mode = "dormant freq=-1";
        const double t1 = cp::timed_solve(b0);
        std::println(stderr, "   plugin in place with freq=-1: {} hook calls={}", line_for(b0, t1), s->calls.load());
        M c0; auto c = cp::build_atsp(c0, 26); TL(c0, 60);
        auto s2 = std::make_shared<cp::stats>();
        ctx * h2 = attach_with(c0, cp::watched([](const auto &, const auto &) {}, s2), "row_g", {1000000, -1, 1, 1.0, false});
        h2->mode = "woken";
        const int rc = setint(c0.native_model(), "separating/probe/freq", 1, c0.native_api());
        const double t2 = cp::timed_solve(c0);
        std::println(stderr, "   freq set to 1 before the solve (retcode {}): {} hook calls={}", rc, line_for(c0, t2), s2->calls.load());
        // and after the solve, without freeing the transform
        const int rc2 = setint(c0.native_model(), "separating/probe/freq", -1, c0.native_api());
        std::println(stderr, "   freq set at stage SOLVED, no freeTransform: retcode {}", rc2);
    }
    {   // a dormant constraint handler, no locks and everything feasible
        M d; auto a = cp::build_atsp(d, 26); TL(d, 60);
        sc::SCIP_CONSHDLR * h = nullptr;
        static long calls = 0;
        sc::SCIP_CONSENFOLP * e1 = +[](sc::SCIP *, sc::SCIP_CONSHDLR *, sc::SCIP_CONS **, int, int, unsigned, SCIP_RESULT * r) { ++calls; *r = sc::SCIP_FEASIBLE; return sc::SCIP_OKAY; };
        sc::SCIP_CONSENFOPS * e2 = +[](sc::SCIP *, sc::SCIP_CONSHDLR *, sc::SCIP_CONS **, int, int, unsigned, unsigned, SCIP_RESULT * r) { ++calls; *r = sc::SCIP_FEASIBLE; return sc::SCIP_OKAY; };
        sc::SCIP_CONSCHECK * e3 = +[](sc::SCIP *, sc::SCIP_CONSHDLR *, sc::SCIP_CONS **, int, sc::SCIP_SOL *, unsigned, unsigned, unsigned, unsigned, SCIP_RESULT * r) { ++calls; *r = sc::SCIP_FEASIBLE; return sc::SCIP_OKAY; };
        sc::SCIP_CONSLOCK * e4 = +[](sc::SCIP *, sc::SCIP_CONSHDLR *, sc::SCIP_CONS *, sc::SCIP_LOCKTYPE, int, int) { return sc::SCIP_OKAY; };
        const int rc = d.native_api().includeConshdlrBasic(d.native_model(), &h, "dormant", "dormant", -2000000, -2000000, -1, 0, e1, e2, e3, e4, nullptr);
        const double t = cp::timed_solve(d);
        std::println(stderr, "== dormant constraint handler, needscons=FALSE, no locks (retcode {}): {} callback calls={}", rc, line_for(d, t), calls);
    }
    {   // moved model: the plugin data sits on the heap
        M a; auto x = cp::build_atsp(a, 26); TL(a, 60);
        auto s = std::make_shared<cp::stats>();
        ctx * h = attach_with(a, cp::watched([&x](const auto & p, const cp::add_fn & add) { for(const auto & c : x.separate(p)) add(c); }, s), "row_g");
        h->mode = "moved";
        M b(std::move(a));
        h->vars = b.vars();
        const double t = cp::timed_solve(b);
        std::println(stderr, "== model moved after attach: {} hook calls={}", line_for(b, t), s->calls.load());
    }
    {   // resume after a limit: same transformed problem, the cuts stay
        M m; auto a = cp::build_atsp(m, 26); TL(m, 0.25);
        void * sp = m.native_model();
        auto s = std::make_shared<cp::stats>();
        ctx * h = attach_with(m, cp::watched([&a, sp](const auto & x, const cp::add_fn & add) {
            if(N.getDepth(sp) == 0) return;
            for(const auto & c : a.separate(x)) add(c);
        }, s), "row_l");
        h->mode = "resume";
        const double t = cp::timed_solve(m);
        const long c1 = s->calls.load(), k1 = s->cuts.load();
        std::println(stderr, "== resume: first solve with a 0.25 s limit: {} calls={} cuts={}", line_for(m, t), c1, k1);
        TL(m, 60);
        const double t2 = cp::timed_solve(m);
        std::println(stderr, "   second solve(): {} calls={} cuts={}", line_for(m, t2), s->calls.load() - c1, s->cuts.load() - k1);
    }
    {   // the hook runs on the thread that called solve()
        std::thread::id solver, hook;
        std::thread([&] {
            solver = std::this_thread::get_id();
            M m; auto a = cp::build_atsp(m, 12); TL(m, 20);
            auto s = std::make_shared<cp::stats>();
            ctx * h = attach_with(m, cp::watched([&hook](const auto &, const auto &) { hook = std::this_thread::get_id(); }, s), "row_g");
            h->mode = "thread";
            cp::timed_solve(m);
            std::println(stderr, "== hook on the solving thread: {} (calls {})", solver == hook, s->calls.load());
        }).join();
    }
    {   // with the wrapper's own progress plugin
        M m; auto a = cp::build_atsp(m, 26); TL(m, 60);
        long progress = 0;
        m.set_progress_callback([&progress](auto &) { ++progress; });
        auto s = std::make_shared<cp::stats>();
        ctx * h = attach_with(m, cp::watched([&a](const auto & x, const cp::add_fn & add) { for(const auto & c : a.separate(x)) add(c); }, s), "row_g");
        h->mode = "with progress";
        const double t = cp::timed_solve(m);
        std::println(stderr, "== with set_progress_callback: {} progress calls={} hook calls={}", line_for(m, t), progress, s->calls.load());
    }
}

// -------------------------------------------------------------- equality -----
// W: why `dummy == 1` reports infeasible.
static void equality() {
    struct cfg { std::string name; bool strong, weak; const char * mode; char sense; };
    const std::vector<cfg> cfgs = {
        {"dummy == 1, dual reductions on, row_g", true, true, "row_g", 'E'},
        {"dummy >= 1, dual reductions on, row_g", true, true, "row_g", 'G'},
        {"dummy == 1, misc/allowstrongdualreds=0 misc/allowweakdualreds=0, row_g", false, false, "row_g", 'E'},
        {"dummy >= 1, misc/allowstrongdualreds=0 misc/allowweakdualreds=0, row_g", false, false, "row_g", 'G'},
        {"dummy == 1, dual reductions on, row_l", true, true, "row_l", 'E'},
        {"dummy == 1, misc/allowstrongdualreds=0 only, row_g", false, true, "row_g", 'E'},
        {"dummy == 1, misc/allowweakdualreds=0 only, row_g", true, false, "row_g", 'E'},
    };
    for(const auto & c : cfgs) {
        M m; scope_settings(m); auto g = cp::build_gadgets(m, 10, false); TL(m, 15);
        void * sp = m.native_model();
        M * mp = &m;
        if(!c.strong) N.setBoolParam(sp, "misc/allowstrongdualreds", 0);
        if(!c.weak) N.setBoolParam(sp, "misc/allowweakdualreds", 0);
        auto s = std::make_shared<cp::stats>();
        bool once = false;
        std::string seen;
        ctx * h = attach_with(m, cp::watched([&](const auto & x, const cp::add_fn & add) {
            if(once) return;
            once = true;
            SCIP_VAR * t = nullptr; N.getTransformedVar(sp, mp->var_of(g.dummy), &t);
            seen = std::format("transformed status={} global=[{},{}] local=[{},{}] original=[{},{}] LP value={}", N.varGetStatus(t), N.varGetLbGlobal(t), N.varGetUbGlobal(t), N.varGetLbLocal(t), N.varGetUbLocal(t), N.varGetLbLocal(mp->var_of(g.dummy)), N.varGetUbLocal(mp->var_of(g.dummy)), x[static_cast<std::size_t>(g.dummy)]);
            add(raw_cut{{g.dummy}, {1.0}, c.sense, 1.0});
        }, s), c.mode);
        h->mode = c.name;
        const double t = cp::timed_solve(m);
        std::println(stderr, "== [{}]: {} (known opt {})\n   at the first hook: {}\n   {}", c.name, line_for(m, t), g.opt, seen, s->line());
    }
    {   // the single-variable cut is a bound change, on which scope?
        for(const char * mode : {"row_g", "row_l"}) for(const bool invalid : {false, true}) {
            M m; scope_settings(m); auto g = cp::build_gadgets(m, 10, false); TL(m, 15);
            void * sp = m.native_model();
            M * mp = &m;
            auto s = std::make_shared<cp::stats>();
            bool added = false;
            long sib_checks = 0, sib_at_bound = 0, sub_checks = 0, sub_at_bound = 0;
            double bound = 0;
            std::string at;
            const int col = g.za[3];
            ctx * h = attach_with(m, cp::watched([&](const auto & x, const cp::add_fn & add) {
                SCIP_VAR * t = nullptr; N.getTransformedVar(sp, mp->var_of(col), &t);
                const int st = g.state(x);
                if(!added && st == g.worse_state()) {
                    added = true;
                    // Below the local bound by a little, or below the value the optimum needs.
                    bound = invalid ? 1.0 : N.varGetUbLocal(t) - 0.001;
                    at = std::format("za3 local ub before={} bound={} LP value={} depth={}", N.varGetUbLocal(t), bound, x[static_cast<std::size_t>(col)], N.getDepth(sp));
                    add(raw_cut{{col}, {1.0}, 'L', bound});
                    return;
                }
                if(!added) return;
                const bool tight = N.varGetUbLocal(t) <= bound + 1e-9;
                if(st == g.worse_state()) { ++sub_checks; sub_at_bound += tight; }
                else if(st == 1 - g.worse_state()) { ++sib_checks; sib_at_bound += tight; }
            }, s), mode);
            h->mode = std::format("single var {} invalid={}", mode, invalid);
            const double t = cp::timed_solve(m);
            double v = NAN; try { v = m.get_solution_value(); } catch(const std::exception &) {}
            std::println(stderr, "== single-variable cut added in a subtree, mode {}, {}: {} (known opt {}) => {} | {} | local ub at the bound: subtree {}/{} sibling {}/{}", mode, invalid ? "za3 <= 1, cuts off the optimum" : "just below the local bound", line_for(m, t), g.opt, std::fabs(v - g.opt) < 1e-4 ? "KEPT" : "LOST", at, sub_at_bound, sub_checks, sib_at_bound, sib_checks);
        }
    }
    {   // an infeasible empty row
        for(const char * mode : {"row_g", "row_l", "row_l_force"}) {
            M m; scope_settings(m); auto g = cp::build_gadgets(m, 10, false); TL(m, 15);
            auto s = std::make_shared<cp::stats>();
            bool added = false;
            ctx * h = attach_with(m, cp::watched([&](const auto & x, const cp::add_fn & add) {
                if(!added && g.state(x) == g.worse_state()) { added = true; add(raw_cut{{}, {}, 'L', -1.0}); }
            }, s), mode);
            h->mode = std::string("empty infeasible ") + mode;
            const double t = cp::timed_solve(m);
            std::println(stderr, "== no terms, 0 <= -1, added in a subtree, mode {}: {} (known opt {})\n   {}", mode, line_for(m, t), g.opt, s->line());
        }
    }
}

// ------------------------------------------------------------------ s1age ----
// The mechanism behind S1: is the row in the LP and at age 0 when the search jumps to
// the sibling? One row, held, read at the first call in the sibling.
static void s1age() {
    for(const char * sel : {"default", "breadthfirst"}) for(const bool local : {false, true}) {
        M m; scope_settings(m); auto g = cp::build_gadgets(m, 10, false); TL(m, 30);
        load_natives(m);
        auto isInLP = cp::symf<unsigned (*)(ROW *)>(N.lib, "SCIProwIsInLP");
        auto getAge = cp::symf<int (*)(ROW *)>(N.lib, "SCIProwGetAge");
        auto isLocal = cp::symf<unsigned (*)(ROW *)>(N.lib, "SCIProwIsLocal");
        void * sp = m.native_model();
        auto & api = m.native_api();
        if(std::string(sel) == "breadthfirst") setint(sp, "nodeselection/breadthfirst/stdpriority", 1000000, api);
        M * mp = &m;
        ROW * row = nullptr;
        ctx * h = nullptr;
        const raw_cut cut = g.invalid_global_cut();
        bool first_sib = true;
        int age_last_subtree = -1; bool in_last_subtree = false;
        std::string report = "sibling never reached after the cut";
        auto s = std::make_shared<cp::stats>();
        h = attach_with(m, cp::watched([&](const auto & x, const cp::add_fn &) {
            const int st = g.state(x);
            if(!row) {
                if(st != g.worse_state() || cut.violation(x) < 1e-4) return;
                N.createEmptyRowSepa(sp, &row, h->sepa, "held", -api.infinity(static_cast<Scip *>(sp)), cut.rhs, local, 0, 1);
                N.cacheRow(sp, row);
                for(std::size_t k = 0; k < cut.ind.size(); ++k) N.addVarToRow(sp, row, mp->var_of(cut.ind[k]), cut.val[k]);
                N.flushRow(sp, row);
                unsigned inf = 0;
                N.addRow(sp, row, 0, &inf);
                return;
            }
            if(st == g.worse_state()) { age_last_subtree = getAge(row); in_last_subtree = isInLP(row); }
            else if(st == 1 - g.worse_state() && first_sib) {
                first_sib = false;
                report = std::format("last call in the subtree: in LP={} age={} | first call in the sibling: in LP={} age={} violation={:.4g}", in_last_subtree, age_last_subtree, bool(isInLP(row)), getAge(row), cut.violation(x));
            }
        }, s), "row_g");
        h->mode = std::format("s1age {} local={}", sel, local);
        const double t = cp::timed_solve(m);
        double v = NAN; try { v = m.get_solution_value(); } catch(const std::exception &) {}
        std::println(stderr, "== s1age selection={} local={}: {} (known opt {}) => {}\n   {}", sel, local, line_for(m, t), g.opt, std::fabs(v - g.opt) < 1e-4 ? "KEPT" : "LOST", report);
        if(row) N.releaseRow(sp, &row);
        api.freeTransform(m.native_model());
        (void)isLocal;
    }
    {   // the local flag at the root
        M m; scope_settings(m); auto g = cp::build_gadgets(m, 10, false); TL(m, 30);
        load_natives(m);
        auto isLocal = cp::symf<unsigned (*)(ROW *)>(N.lib, "SCIProwIsLocal");
        void * sp = m.native_model();
        auto & api = m.native_api();
        M * mp = &m;
        ctx * h = nullptr;
        bool once = false;
        auto s = std::make_shared<cp::stats>();
        h = attach_with(m, cp::watched([&](const auto & x, const cp::add_fn &) {
            if(once) return;
            once = true;
            const raw_cut cut = g.hull_cut(3);
            ROW * row = nullptr;
            N.createEmptyRowSepa(sp, &row, h->sepa, "root", -api.infinity(static_cast<Scip *>(sp)), cut.rhs, 1, 0, 1);
            for(std::size_t k = 0; k < cut.ind.size(); ++k) N.addVarToRow(sp, row, mp->var_of(cut.ind[k]), cut.val[k]);
            const unsigned before = isLocal(row);
            unsigned inf = 0;
            N.addRow(sp, row, 0, &inf);
            std::println(stderr, "== a local row added at depth {}: SCIProwIsLocal before SCIPaddRow={} after={}", N.getDepth(sp), before, isLocal(row));
            N.releaseRow(sp, &row);
        }, s), "row_l");
        h->mode = "local at root";
        cp::timed_solve(m);
    }
}

// -------------------------------------------------------------- unbounded ----
// Q1: an unbounded LP relaxation reaches the separator with another LP status.
static void unbounded() {
    M m; scope_settings(m);
    auto x = m.add_variables(2, {.obj_coef = 1.0, .lower_bound = 0.0});
    auto y = m.add_binary_variables(2);
    m.set_maximization();
    m.add_constraint(x(std::size_t{0}) + y(std::size_t{0}) >= 1.0);
    m.add_constraint(x(std::size_t{1}) - x(std::size_t{0}) + y(std::size_t{1}) <= 1.5);
    TL(m, 5);
    auto s = std::make_shared<cp::stats>();
    ctx * h = attach_with(m, cp::watched([](const auto &, const auto &) {}, s), "row_g");
    h->mode = "unbounded LP";
    const double t = cp::timed_solve(m);
    std::println(stderr, "== unbounded relaxation: {} hook reached the separator {} times, LP not optimal at entry {} times", line_for(m, t), s->calls.load(), h->not_optimal);
}

// ---------------------------------------------------------------- objread ----
// What a handle's get_solution_value() would read, on a maximization with an offset.
static void objread() {
    for(const double offset : {0.0, 7.5}) {
        M m; scope_settings(m); auto g = cp::build_gadgets(m, 10, false); TL(m, 15);
        m.set_objective_offset(offset);
        load_natives(m);
        auto lpObj = cp::symf<double (*)(void *)>(N.lib, "SCIPgetLPObjval");
        auto retrans = cp::symf<double (*)(void *, double)>(N.lib, "SCIPretransformObj");
        auto solObj = cp::symf<double (*)(void *, void *)>(N.lib, "SCIPgetSolOrigObj");
        void * sp = m.native_model();
        auto s = std::make_shared<cp::stats>();
        double max_raw = 0, max_ret = 0, max_sol = 0, raw0 = NAN, ret0 = NAN, sol0 = NAN, mine0 = NAN;
        ctx * h = attach_with(m, cp::watched([&](const auto & x, const auto &) {
            const double mine = g.objective(x) + offset;
            const double raw = lpObj(sp), ret = retrans(sp, raw), sol = solObj(sp, nullptr);
            if(std::isnan(raw0)) { raw0 = raw; ret0 = ret; sol0 = sol; mine0 = mine; }
            max_raw = std::max(max_raw, std::fabs(raw - mine));
            max_ret = std::max(max_ret, std::fabs(ret - mine));
            max_sol = std::max(max_sol, std::fabs(sol - mine));
        }, s), "row_g");
        h->mode = "objread";
        cp::timed_solve(m);
        std::println(stderr, "== objread offset={}: first call point objective={} SCIPgetLPObjval={} SCIPretransformObj(it)={} SCIPgetSolOrigObj(NULL)={} | max diff over {} calls: raw={} retransformed={} solorigobj={}", offset, mine0, raw0, ret0, sol0, s->calls.load(), max_raw, max_ret, max_sol);
    }
}

// ----------------------------------------------------------------- cutoff ----
// Q3: a global cut that removes only non-optimal integer points, an objective cutoff.
static void cutoff() {
    for(const char * mode : {"row_g", "row+pool", "cons_g"}) {
        M m; scope_settings(m); auto g = cp::build_gadgets(m, 10, false); TL(m, 15);
        auto s = std::make_shared<cp::stats>();
        raw_cut cut{g.z, std::vector<double>(g.z.size(), 1.0), 'G', g.opt - 1};
        bool once = false;
        ctx * h = attach_with(m, cp::watched([&](const auto &, const cp::add_fn & add) {
            if(!once) { once = true; add(cut); }
        }, s), mode);
        h->mode = std::string("objective cutoff ") + mode;
        const double t = cp::timed_solve(m);
        double v = NAN; try { v = m.get_solution_value(); } catch(const std::exception &) {}
        std::println(stderr, "== objective cutoff Sum z >= opt-1 at the root, mode {}: {} (known opt {}) => {}", mode, line_for(m, t), g.opt, std::fabs(v - g.opt) < 1e-4 ? "KEPT" : "LOST");
    }
}

int main(int argc, char ** argv) {
    { M m; std::println(stderr, "library {} SCIP {}.{}.{}", m.native_api().library_path().string(), m.native_api().majorVersion(), m.native_api().minorVersion(), m.native_api().techVersion()); }
    const std::string what = argc > 1 ? argv[1] : "";
    if(what == "firing") firing();
    else if(what == "rounds") rounds();
    else if(what == "e4") e4();
    else if(what == "nodereads") nodereads();
    else if(what == "restart") restart();
    else if(what == "mixed") mixed();
    else if(what == "filter") filter();
    else if(what == "aging") aging();
    else if(what == "lifecycle") lifecycle();
    else if(what == "equality") equality();
    else if(what == "s1age") s1age();
    else if(what == "unbounded") unbounded();
    else if(what == "objread") objread();
    else if(what == "cutoff") cutoff();
    else std::println(stderr, "unknown subcommand");
}
