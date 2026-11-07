// Q11: lazy constraints and user cuts in one native slot, on a DFJ model with only degree constraints.
#include "probe_cplex_common.cpp"
using cp::raw_cut;
struct dfj_model { cp::atsp a; };
template <class MM>
static cp::atsp build_degree_only(MM & m, const int n, const unsigned seed = 5) {
    cp::atsp a; a.n = n;
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
    m.set_objective(xsum(nodes, [&](std::size_t i) { return xsum(nodes | std::views::filter([i](std::size_t j) { return j != i; }), [&, i](std::size_t j) { return a.d[i][j] * x(id(i, j)); }); }));
    for(std::size_t i : nodes) {
        m.add_constraint(xsum(nodes | std::views::filter([i](std::size_t j) { return j != i; }), [&, i](std::size_t j) { return 1.0 * x(id(i, j)); }) == 1);
        m.add_constraint(xsum(nodes | std::views::filter([i](std::size_t j) { return j != i; }), [&, i](std::size_t j) { return 1.0 * x(id(j, i)); }) == 1);
    }
    return a;
}
struct L {
    const c::cplex_api * api; const syms * s; const cp::atsp * a; int cols;
    bool lazy = true, user = true; int purgeable = USECUT_PURGE, local = 0;
    int (*rejlocal)(ctxptr, int, int, const double *, const char *, const int *, const int *, const double *) = nullptr;
    std::atomic<long> rel_calls{0}, rel_integral{0}, rel_cuts{0}, cand_calls{0}, cand_rejected{0}, cand_lazy_cuts{0}, cand_accept{0}, overlaps{0}, rel_err{0}, cand_err{0};
    std::atomic<int> inflight{0}, maxinflight{0};
    int cross_rel_reject = 12345, cross_rel_rejectlocal = 12345, cross_cand_usercut = 12345, cross_rel_cand_point = 12345, cross_cand_rel_point = 12345, cross_cand_status = 12345;
    std::map<int, long> cand_sources; std::mutex mtx;
    std::set<std::string> added_lazy;
};
static int lhook(ctxptr ctx, cpxlong id, void * usr) {
    auto & l = *static_cast<L *>(usr);
    const int now = l.inflight.fetch_add(1) + 1;
    if(now > 1) l.overlaps++;
    { int s = l.maxinflight.load(); while(now > s && !l.maxinflight.compare_exchange_weak(s, now)) {} }
    std::vector<double> x(static_cast<std::size_t>(l.cols));
    const auto one = [&](const raw_cut & r, auto fn) {
        const int beg = 0;
        return fn(ctx, 1, static_cast<int>(r.ind.size()), &r.rhs, &r.sense, &beg, r.ind.data(), r.val.data());
    };
    const auto user_cut = [&](const raw_cut & r) {
        const int beg = 0;
        return l.api->callbackaddusercuts(ctx, 1, static_cast<int>(r.ind.size()), &r.rhs, &r.sense, &beg, r.ind.data(), r.val.data(), &l.purgeable, &l.local);
    };
    if(id == c::CPX_CALLBACKCONTEXT_RELAXATION) {
        l.rel_calls++;
        if(l.s->point(ctx, x.data(), 0, l.cols - 1, nullptr)) { l.rel_err++; l.inflight.fetch_sub(1); return 0; }
        if(l.a->integral(x)) l.rel_integral++;
        const auto cuts = l.a->separate(x);
        if(!cuts.empty() && l.cross_rel_reject == 12345) {
            // from the wrong context: the error codes, no effect wanted
            l.cross_rel_reject = one(cuts[0], l.api->callbackrejectcandidate);
            l.cross_rel_rejectlocal = l.rejlocal ? one(cuts[0], l.rejlocal) : -1;
            l.cross_rel_cand_point = l.api->callbackgetcandidatepoint(ctx, x.data(), 0, l.cols - 1, nullptr);
            l.s->point(ctx, x.data(), 0, l.cols - 1, nullptr);
        }
        if(l.user) for(const auto & r : cuts) { if(user_cut(r)) l.rel_err++; else l.rel_cuts++; }
    } else if(id == c::CPX_CALLBACKCONTEXT_CANDIDATE) {
        l.cand_calls++;
        int source = -1; l.s->infoint(ctx, INFO_CANDIDATE_SOURCE, &source);
        { std::lock_guard g(l.mtx); l.cand_sources[source]++; }
        if(l.api->callbackgetcandidatepoint(ctx, x.data(), 0, l.cols - 1, nullptr)) { l.cand_err++; l.inflight.fetch_sub(1); return 0; }
        const auto cuts = l.a->separate(x, 1000);
        if(!cuts.empty() && l.cross_cand_usercut == 12345) {
            l.cross_cand_usercut = user_cut(cuts[0]);
            std::vector<double> y(static_cast<std::size_t>(l.cols));
            l.cross_cand_rel_point = l.s->point(ctx, y.data(), 0, l.cols - 1, nullptr);
            int st = 0; l.cross_cand_status = l.s->status(ctx, &st, RELAX_NOSOLVE);
        }
        if(l.lazy && !cuts.empty()) {
            std::vector<double> rhs; std::vector<char> sense; std::vector<int> beg, ind; std::vector<double> val;
            for(const auto & r : cuts) {
                rhs.push_back(r.rhs); sense.push_back(r.sense); beg.push_back(static_cast<int>(ind.size()));
                ind.insert(ind.end(), r.ind.begin(), r.ind.end()); val.insert(val.end(), r.val.begin(), r.val.end());
            }
            if(l.api->callbackrejectcandidate(ctx, static_cast<int>(cuts.size()), static_cast<int>(ind.size()), rhs.data(), sense.data(), beg.data(), ind.data(), val.data())) l.cand_err++;
            l.cand_rejected++; l.cand_lazy_cuts += static_cast<long>(cuts.size());
        } else l.cand_accept++;
    }
    l.inflight.fetch_sub(1);
    return 0;
}
static bool no_solver_help = false;
static void only_user_cuts(M & m) {
    for(int p : {P_CLIQUES, P_COVERS, P_DISJCUTS, P_FLOWCOVERS, P_FLOWPATHS, P_FRACCUTS, P_GUBCOVERS, P_IMPLBD, P_LANDP, P_MCF, P_ZEROHALF, P_LOCALIMPLBD, P_BQP, P_RLT})
        param(m, p, -1);
    param(m, P_EACHCUTLIM, 0);
    param(m, P_HEURFREQ, -1); param(m, P_RINS, -1); param(m, P_FPHEUR, -1); param(m, P_LBHEUR, 0);
    auto [env, lp] = m.native_model(); m.native_api()._check(env, m.native_api().setdblparam(env, P_HEUREFFORT, 0.0));
}
static double run(const std::string & name, int n, unsigned seed, bool lazy, bool user, const std::string & mode, cpxlong mask, int threads, double expect = NAN, bool log = false) {
    M m; if(threads) param(m, P_THREADS, threads);
    if(no_solver_help) only_user_cuts(m);
    if(log) { param(m, P_SCRIND, 1); param(m, P_MIPDISPLAY, 1); std::println("#### {}", name); std::fflush(stdout); }
    auto a = build_degree_only(m, n, seed);
    m.set_time_limit(std::chrono::duration<double>(60));
    L l; l.api = &m.native_api(); l.s = &syms::get(m); l.a = &a; l.cols = static_cast<int>(m.num_variables());
    l.lazy = lazy; l.user = user;
    auto [pg, lc] = mode_flags(mode); l.purgeable = pg; l.local = lc;
    l.rejlocal = cp::symf<decltype(l.rejlocal)>(m.native_api().library_path(), "CPXcallbackrejectcandidatelocal");
    auto [env, lp] = m.native_model();
    m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, mask, lhook, &l));
    const double t = cp::timed_solve(m);
    double value = NAN; try { value = m.get_solution_value(); } catch(const std::exception &) {}
    std::string src; for(auto [k, v] : l.cand_sources) src += std::format(" {}:{}", k, v);
    std::println(stderr, "== {} n={}: {}{}\n   relaxation calls={} integral={} user_cuts={} | candidate calls={} rejected={} lazy_rows={} accepted={} sources(1=node? see cpxconst):{} | overlaps={} max_inflight={} errs rel={} cand={}\n   from RELAXATION: rejectcandidate={} rejectcandidatelocal={} getcandidatepoint={}; from CANDIDATE: addusercuts={} getrelaxationpoint={} getrelaxationstatus={}",
                 name, n, cp::summary(m, t), std::isnan(expect) ? "" : std::format(" (expected {})", expect), l.rel_calls.load(), l.rel_integral.load(), l.rel_cuts.load(), l.cand_calls.load(), l.cand_rejected.load(), l.cand_lazy_cuts.load(), l.cand_accept.load(), src, l.overlaps.load(), l.maxinflight.load(), l.rel_err.load(), l.cand_err.load(), l.cross_rel_reject, l.cross_rel_rejectlocal, l.cross_rel_cand_point, l.cross_cand_usercut, l.cross_cand_rel_point, l.cross_cand_status);
    std::fflush(stderr);
    return value;
}
int main() {
    std::println(stderr, "library {}", M{}.native_api().library_path().string());
    const cpxlong R = c::CPX_CALLBACKCONTEXT_RELAXATION, C = c::CPX_CALLBACKCONTEXT_CANDIDATE;
    // reference optimum: the MTZ model with DFJ user cuts of the same instance, plain, and lazy only
    double ref = NAN;
    for(unsigned seed : {5u, 6u, 7u}) for(bool help : {true, false}) {
        no_solver_help = !help;
        const int n = 31;
        { M m; param(m, P_THREADS, 1); auto a = cp::build_atsp(m, n, false, seed); m.set_time_limit(std::chrono::duration<double>(60));
          auto st = std::make_shared<cp::stats>();
          attach(m, cp::watched([&a](const auto & x, const cp::add_fn & add) { for(const auto & c : a.separate(x)) add(c); }, st), "global-force");
          cp::timed_solve(m); keep.back()->sep = {}; ref = m.get_solution_value(); std::println(stderr, "== reference, MTZ + DFJ cuts, n={} seed={}: {}; {}", n, seed, ref, help ? "SOLVER HEURISTICS/CUTS ON" : "SOLVER HEURISTICS/CUTS OFF"); }
        run("lazy only, 1 thread", n, seed, true, false, "global-purge", C, 1, ref);
        run("lazy + user global-purge, 1 thread", n, seed, true, true, "global-purge", C | R, 1, ref);
        run("lazy + user global-force, 1 thread", n, seed, true, true, "global-force", C | R, 1, ref);
        run("lazy + user global-filter, 1 thread", n, seed, true, true, "global-filter", C | R, 1, ref);
        run("lazy + user local-purge, 1 thread", n, seed, true, true, "local-purge", C | R, 1, ref);
        run("lazy + user global-purge, default threads", n, seed, true, true, "global-purge", C | R, 0, ref);
        run("lazy only, default threads", n, seed, true, false, "global-purge", C, 0, ref);
    }
    no_solver_help = false;
    ref = 549;
    run("user cuts ONLY on the incomplete model (no lazy, no candidate context)", 26, 5, false, true, "global-force", R, 1, ref);
    run("user cuts ONLY, candidate context registered but accepts", 26, 5, false, true, "global-force", C | R, 1, ref);

    // the wrapper's solve() overwrites a native registration once a MIP++ callback is set
    {
        M m; param(m, P_THREADS, 1);
        auto a = cp::build_atsp(m, 26);
        m.set_time_limit(std::chrono::duration<double>(30));
        auto s = std::make_shared<cp::stats>();
        attach(m, cp::watched([](const auto &, const auto &) {}, s), "global-purge");
        std::atomic<long> cand{0};
        m.set_candidate_solution_callback([&cand](auto &) { cand++; });
        const double t = cp::timed_solve(m);
        std::println(stderr, "== native RELAXATION hook + MIP++ candidate callback: {}\n   wrapper candidate calls={} my relaxation calls={}", cp::summary(m, t), cand.load(), s->calls.load());
        // after solve(): the wrapper detached everything, mine included
        auto [env, lp] = m.native_model();
        cpxlong mask = -1; c::CPXCALLBACKFUNC * f = nullptr; void * u = nullptr;
        const int e = cp::symf<int (*)(c::CPXCENVptr, c::CPXCLPptr, cpxlong *, c::CPXCALLBACKFUNC **, void **)>(m.native_api().library_path(), "CPXcallbackgetfunc")(env, lp, &mask, &f, &u);
        std::println(stderr, "   after solve(): CPXcallbackgetfunc err={} mask={} func set={}", e, mask, f != nullptr);
    }
    // what the candidate context costs in presolve (the manual: dual and nonlinear reductions off)
    for(auto [nm, mask] : std::initializer_list<std::pair<const char *, cpxlong>>{{"no hook", 0}, {"R", R}, {"C", C}, {"C|R", C | R}}) {
        for(bool dups : {false, true}) {
            M m; param(m, P_THREADS, 1); param(m, P_SCRIND, 1); param(m, P_MIPDISPLAY, 1);
            std::println("#### {} {}", dups ? "duplicate columns" : "dual-fixable column", nm); std::fflush(stdout);
            const std::size_t Y = 20;
            std::mt19937 rng(3); std::uniform_int_distribution<int> A(5, 60), P(5, 80);
            auto y = m.add_binary_variables(Y);
            auto z = dups ? m.add_integer_variables(2, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 7}) : m.add_variables(2, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 7});
            std::vector<double> p(Y), a1(Y), a2(Y); for(std::size_t j = 0; j < Y; ++j) { p[j] = P(rng); a1[j] = A(rng); a2[j] = A(rng); }
            const auto ids = std::views::iota(std::size_t{0}, Y);
            m.set_maximization();
            const auto zz0 = z(std::size_t{0}); const auto zz1 = z(std::size_t{1});
            // duplicates: same columns, same cost. Dual-fixable: z costs in a max and only eats capacity.
            const double zo = dups ? 9.0 : -3.0;
            m.set_objective(xsum(ids, [&](std::size_t j) { return p[j] * y(j); }) + zo * zz0 + zo * zz1);
            m.add_constraint(xsum(ids, [&](std::size_t j) { return a1[j] * y(j); }) + 7.0 * zz0 + 7.0 * zz1 <= 203.5);
            m.add_constraint(xsum(ids, [&](std::size_t j) { return a2[j] * y(j); }) + 11.0 * zz0 + 11.0 * zz1 <= 231.5);
            int err = 12345; long calls = 0; const int cz = m.native_id(zz0), cols = static_cast<int>(m.num_variables());
            struct U { int * err; long * calls; int cz; int cols; const c::cplex_api * api; const syms * s; } u{&err, &calls, cz, cols, &m.native_api(), &syms::get(m)};
            if(mask) {
                auto [env, lp] = m.native_model();
                m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, mask, [](ctxptr ctx, cpxlong id, void * usr) -> int {
                    auto & uu = *static_cast<U *>(usr);
                    if(id != c::CPX_CALLBACKCONTEXT_RELAXATION) return 0;
                    ++*uu.calls;
                    if(*uu.err == 12345) {
                        const double rhs = 1000, val = 1; const char sense = 'L'; const int beg = 0, pg = USECUT_PURGE, lc = 0;
                        *uu.err = uu.api->callbackaddusercuts(ctx, 1, 1, &rhs, &sense, &beg, &uu.cz, &val, &pg, &lc);
                    }
                    return 0;
                }, &u));
            }
            m.set_time_limit(std::chrono::duration<double>(10));
            const double t = cp::timed_solve(m);
            std::println(stderr, "== presolve side effects, {} mask={}: {} relaxation calls={} addusercuts on one column err={}", dups ? "duplicate columns" : "dual-fixable column", nm, cp::summary(m, t), calls, err);
        }
    }
}
