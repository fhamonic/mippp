// Q13: on one release, the entry points and the behaviours a wrapper would rely on.
#include "probe_cplex_common.cpp"
using cp::raw_cut;
static void family_off(M & m) { for(int p : {P_CLIQUES, P_COVERS, P_DISJCUTS, P_FLOWCOVERS, P_FLOWPATHS, P_FRACCUTS, P_GUBCOVERS, P_IMPLBD, P_LANDP, P_MCF, P_MIRCUTS, P_ZEROHALF, P_LOCALIMPLBD, P_BQP, P_RLT}) param(m, p, -1); }
struct Q {
    const c::cplex_api * api; const syms * s; int cols;
    std::function<void(ctxptr, const std::vector<double> &, cpxlong)> body;
    long calls = 0, depth0 = 0;
};
static int qhook(ctxptr ctx, cpxlong id, void * usr) {
    auto & q = *static_cast<Q *>(usr);
    if(id != c::CPX_CALLBACKCONTEXT_RELAXATION) return 0;
    std::vector<double> x(static_cast<std::size_t>(q.cols));
    q.s->point(ctx, x.data(), 0, q.cols - 1, nullptr);
    cpxlong d = -1; q.s->infolong(ctx, INFO_NODEDEPTH, &d);
    ++q.calls; q.depth0 += d == 0;
    if(q.body) q.body(ctx, x, d);
    return 0;
}
static int add1(const Q & q, ctxptr ctx, const raw_cut & r, int pg, int lc) {
    const int beg = 0;
    return q.api->callbackaddusercuts(ctx, 1, static_cast<int>(r.ind.size()), &r.rhs, &r.sense, &beg, r.ind.data(), r.val.data(), &pg, &lc);
}
int main() {
    M m0; auto & api0 = m0.native_api(); const auto & path = api0.library_path();
    auto [env0, lp0] = m0.native_model();
    int number = 0; api0.versionnumber(env0, &number);
    std::println(stderr, "library {}\nCPXversion {} CPXversionnumber {}", path.string(), api0.version(env0), number);
    void * h = dlopen(path.c_str(), RTLD_NOW | RTLD_NOLOAD);
    for(const char * n : {"CPXcallbacksetfunc", "CPXcallbackaddusercuts", "CPXcallbackgetrelaxationpoint", "CPXcallbackgetrelaxationstatus", "CPXcallbackgetrelaxationpi", "CPXcallbackexitcutloop", "CPXcallbackgetlocallb", "CPXcallbackgetlocalub", "CPXcallbackgetgloballb", "CPXcallbackgetglobalub", "CPXcallbackgetinfoint", "CPXcallbackgetinfolong", "CPXcallbackgetinfodbl", "CPXcallbackgetincumbent", "CPXcallbackrejectcandidate", "CPXcallbackrejectcandidatelocal", "CPXcallbackpostheursoln", "CPXgetnumcuts", "CPXaddusercuts", "CPXsetusercutcallbackfunc"})
        std::println(stderr, "  entry point {}: {}", n, dlsym(h, n) ? "present" : "MISSING");
    {
        int d = 0, mn = 0, mx = 0;
        std::println(stderr, "  Reformulations(1167) infointparam err={} default={} min={} max={}; PRELINEAR(1058) err={}", api0.infointparam(env0, P_REFORM, &d, &mn, &mx), d, mn, mx, api0.getintparam(env0, P_PRELINEAR, &d));
    }
    // info queries in the hook; families on so the root fires
    {
        M m; scope_settings(m); param(m, P_MIRCUTS, 1); auto g = cp::build_gadgets(m, 8, false); m.set_time_limit(std::chrono::duration<double>(5));
        Q q{&m.native_api(), &syms::get(m), static_cast<int>(m.num_variables())};
        std::map<std::string, int> errs; int st = -1;
        q.body = [&](ctxptr ctx, const std::vector<double> &, cpxlong) {
            int i = 0; cpxlong l = 0; double db = 0;
            errs["AFTERCUTLOOP"] |= q.s->infoint(ctx, INFO_AFTERCUTLOOP, &i);
            errs["RESTARTS"] |= q.s->infoint(ctx, INFO_RESTARTS, &i);
            errs["THREADID"] |= q.s->infoint(ctx, INFO_THREADID, &i);
            errs["NODEUID"] |= q.s->infolong(ctx, INFO_NODEUID, &l);
            errs["NODEDEPTH"] |= q.s->infolong(ctx, INFO_NODEDEPTH, &l);
            errs["NODESLEFT"] |= q.s->infolong(ctx, INFO_NODESLEFT, &l);
            errs["BEST_BND"] |= q.s->infodbl(ctx, INFO_BEST_BND, &db);
            errs["STATUS"] |= q.s->status(ctx, &st, RELAX_NOSOLVE);
        };
        auto [env, lp] = m.native_model();
        m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, c::CPX_CALLBACKCONTEXT_RELAXATION, qhook, &q));
        cp::timed_solve(m);
        std::string e; for(auto [k, v] : errs) e += std::format(" {}={}", k, v);
        std::println(stderr, "== info queries in the hook, calls={} depth0={} errors:{} last status={}", q.calls, q.depth0, e, st);
    }
    // root firing with every family off
    for(bool all_off : {true, false}) {
        M m; scope_settings(m); if(!all_off) param(m, P_MIRCUTS, 1); auto g = cp::build_gadgets(m, 8, false); m.set_time_limit(std::chrono::duration<double>(5));
        Q q{&m.native_api(), &syms::get(m), static_cast<int>(m.num_variables())};
        auto [env, lp] = m.native_model();
        m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, c::CPX_CALLBACKCONTEXT_RELAXATION, qhook, &q));
        cp::timed_solve(m);
        std::println(stderr, "== families {}: calls={} root calls={}", all_off ? "all off" : "MIR on", q.calls, q.depth0);
    }
    // duplicate columns, crushing
    for(auto [nm, mask, reform] : std::initializer_list<std::tuple<const char *, cpxlong, int>>{{"R reform=3", c::CPX_CALLBACKCONTEXT_RELAXATION, 3}, {"R reform=2", c::CPX_CALLBACKCONTEXT_RELAXATION, 2}, {"R reform=0", c::CPX_CALLBACKCONTEXT_RELAXATION, 0}, {"C|R reform=3", c::CPX_CALLBACKCONTEXT_RELAXATION | c::CPX_CALLBACKCONTEXT_CANDIDATE, 3}}) {
        M m; param(m, P_THREADS, 1); param(m, P_REFORM, reform); param(m, P_HEURFREQ, -1); param(m, P_FPHEUR, -1); param(m, P_RINS, -1);
        const std::size_t Y = 25; std::mt19937 rng(11); std::uniform_int_distribution<int> A(5, 60), P(5, 80);
        auto y = m.add_binary_variables(Y); auto x = m.add_integer_variables(2, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 9});
        std::vector<double> p(Y), a1(Y), a2(Y); for(std::size_t j = 0; j < Y; ++j) { p[j] = P(rng); a1[j] = A(rng); a2[j] = A(rng); }
        const auto ids = std::views::iota(std::size_t{0}, Y);
        m.set_maximization();
        m.set_objective(xsum(ids, [&](std::size_t j) { return p[j] * y(j); }) + 17.0 * x(std::size_t{0}) + 17.0 * x(std::size_t{1}));
        m.add_constraint(xsum(ids, [&](std::size_t j) { return a1[j] * y(j); }) + 11.0 * x(std::size_t{0}) + 11.0 * x(std::size_t{1}) <= 263.5);
        m.add_constraint(xsum(ids, [&](std::size_t j) { return a2[j] * y(j); }) + 13.0 * x(std::size_t{0}) + 13.0 * x(std::size_t{1}) <= 281.5);
        m.set_time_limit(std::chrono::duration<double>(5));
        const int col = m.native_id(x(std::size_t{0}));
        Q q{&m.native_api(), &syms::get(m), static_cast<int>(m.num_variables())};
        int err = 12345;
        q.body = [&](ctxptr ctx, const std::vector<double> &, cpxlong) { if(err == 12345) err = add1(q, ctx, raw_cut{{col}, {1.0}, 'L', 100.0}, USECUT_PURGE, 0); };
        auto [env, lp] = m.native_model();
        m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, mask, qhook, &q));
        cp::timed_solve(m);
        std::println(stderr, "== crush, cut on one of two duplicate columns, {}: addusercuts err={} (calls={})", nm, err, q.calls);
    }
    // the root loop and a satisfied cut
    for(const std::string mode : {"global-purge", "global-force", "global-filter"}) {
        M m; scope_settings(m); param(m, P_MIRCUTS, 1); param(m, P_EACHCUTLIM, 0); auto g = cp::build_gadgets(m, 10, false); m.set_time_limit(std::chrono::duration<double>(2));
        Q q{&m.native_api(), &syms::get(m), static_cast<int>(m.num_variables())};
        const auto [pg, lc] = mode_flags(mode);
        q.body = [&, pg, lc](ctxptr ctx, const std::vector<double> &, cpxlong d) { if(d == 0) add1(q, ctx, raw_cut{{g.za[3]}, {1.0}, 'L', 1e6}, pg, lc); };
        auto [env, lp] = m.native_model();
        m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, c::CPX_CALLBACKCONTEXT_RELAXATION, qhook, &q));
        const double t = cp::timed_solve(m);
        std::println(stderr, "== satisfied cut at every root call, {}: {} calls={} root calls={}", mode, cp::summary(m, t), q.calls, q.depth0);
    }
    // filter refuses a dense weak cut at the root
    for(const std::string mode : {"global-filter", "global-purge"}) {
        M m; scope_settings(m); param(m, P_MIRCUTS, 1); param(m, P_EACHCUTLIM, 0);
        cp::build_mkp(m, 30, 100); m.set_time_limit(std::chrono::duration<double>(2));
        auto [env, lp] = m.native_model();
        std::vector<double> obj(100);
        cp::symf<int (*)(c::CPXCENVptr, c::CPXCLPptr, double *, int, int)>(m.native_api().library_path(), "CPXgetobj")(env, lp, obj.data(), 0, 99);
        Q q{&m.native_api(), &syms::get(m), 100};
        const auto [pg, lc] = mode_flags(mode);
        std::optional<raw_cut> cut; long ck = 0, v = 0;
        q.body = [&, pg, lc](ctxptr ctx, const std::vector<double> & x, cpxlong d) {
            if(d != 0) return;
            if(cut) { ++ck; v += cut->violation(x) > 1e-5; return; }
            double o = 0; for(std::size_t j = 0; j < 100; ++j) o += obj[j] * x[j];
            raw_cut r; r.sense = 'L'; r.rhs = std::floor(o); for(int j = 0; j < 100; ++j) { r.ind.push_back(j); r.val.push_back(obj[static_cast<std::size_t>(j)]); }
            cut = r; add1(q, ctx, r, pg, lc);
        };
        m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, c::CPX_CALLBACKCONTEXT_RELAXATION, qhook, &q));
        cp::timed_solve(m);
        int n = -1; syms::get(m).numcuts(env, lp, CUT_USER, &n);
        std::println(stderr, "== dense objective-floor cut, {}: later root points violating it {}/{} numcuts={}", mode, v, ck, n);
    }
    // legacy mix
    {
        M m; scope_settings(m); auto g = cp::build_gadgets(m, 6, false);
        auto [env, lp] = m.native_model(); auto & api = m.native_api();
        Q q{&api, &syms::get(m), static_cast<int>(m.num_variables())};
        api._check(env, api.callbacksetfunc(env, lp, c::CPX_CALLBACKCONTEXT_RELAXATION, qhook, &q));
        using lcb = int (*)(c::CPXCENVptr, void *, int, void *, int *);
        const auto f = cp::symf<int (*)(c::CPXENVptr, lcb, void *)>(api.library_path(), "CPXsetusercutcallbackfunc");
        f(env, [](c::CPXCENVptr, void *, int, void *, int * a) -> int { *a = 0; return 0; }, nullptr);
        std::println(stderr, "== legacy + generic: CPXmipopt={}", api.mipopt(env, lp));
        f(env, nullptr, nullptr);
    }
    // cross-context codes and the lazy + user optimum, with the MTZ model as a complete model and a run of the same on x-only cuts
    {
        M m; param(m, P_THREADS, 1);
        auto a = cp::build_atsp(m, 26);
        m.set_time_limit(std::chrono::duration<double>(30));
        const int cols = static_cast<int>(m.num_variables());
        Q q{&m.native_api(), &syms::get(m), cols};
        int rel_rej = 12345, cand_add = 12345; long cands = 0;
        struct U { Q * q; int * rel_rej; int * cand_add; long * cands; const cp::atsp * a; } u{&q, &rel_rej, &cand_add, &cands, &a};
        auto [env, lp] = m.native_model();
        m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, c::CPX_CALLBACKCONTEXT_RELAXATION | c::CPX_CALLBACKCONTEXT_CANDIDATE, [](ctxptr ctx, cpxlong id, void * usr) -> int {
            auto & uu = *static_cast<U *>(usr);
            const raw_cut r{{0}, {1.0}, 'L', 1e6};
            const int beg = 0;
            if(id == c::CPX_CALLBACKCONTEXT_CANDIDATE) {
                ++*uu.cands;
                if(*uu.cand_add == 12345) *uu.cand_add = add1(*uu.q, ctx, r, USECUT_PURGE, 0);
                return 0;
            }
            if(*uu.rel_rej == 12345) *uu.rel_rej = uu.q->api->callbackrejectcandidate(ctx, 1, 1, &r.rhs, &r.sense, &beg, r.ind.data(), r.val.data());
            return qhook(ctx, id, uu.q);
        }, &u));
        q.body = [&](ctxptr ctx, const std::vector<double> & x, cpxlong) { for(const auto & r : a.separate(x)) add1(q, ctx, r, USECUT_PURGE, 0); };
        const double t = cp::timed_solve(m);
        std::println(stderr, "== MTZ atsp26 with C|R, DFJ user cuts: {} calls={} candidates={}; rejectcandidate from RELAXATION={} addusercuts from CANDIDATE={}", cp::summary(m, t), q.calls, cands, rel_rej, cand_add);
    }
}
