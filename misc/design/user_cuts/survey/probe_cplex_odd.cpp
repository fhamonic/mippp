// Q10 extras, legacy mixing, return codes.
#include "probe_cplex_common.cpp"
using cp::raw_cut;
struct O {
    const c::cplex_api * api; const syms * s; int cols;
    std::function<int(ctxptr, const std::vector<double> &, cpxlong, O &)> body;
    long calls = 0;
};
static int ohook(ctxptr ctx, cpxlong id, void * usr) {
    auto & o = *static_cast<O *>(usr);
    if(id != c::CPX_CALLBACKCONTEXT_RELAXATION) return 0;
    std::vector<double> x(static_cast<std::size_t>(o.cols));
    o.s->point(ctx, x.data(), 0, o.cols - 1, nullptr);
    cpxlong d = 0; o.s->infolong(ctx, INFO_NODEDEPTH, &d);
    ++o.calls;
    return o.body(ctx, x, d, o);
}
static int add1(const O & o, ctxptr ctx, const raw_cut & r, int pg = USECUT_PURGE, int lc = 0) {
    const int beg = 0;
    return o.api->callbackaddusercuts(ctx, 1, static_cast<int>(r.ind.size()), &r.rhs, &r.sense, &beg, r.ind.data(), r.val.data(), &pg, &lc);
}
static std::string errtext(M & m, int e) {
    if(e == 0) return "0";
    char buf[1024] = {0}; auto [env, lp] = m.native_model();
    const char * t = m.native_api().geterrorstring(env, e, buf);
    std::string s = t ? t : "?"; while(!s.empty() && (s.back() == '\n' || s.back() == ' ')) s.pop_back();
    return std::format("{} ({})", e, s);
}
// a small model: x continuous in [0,5], a knapsack in y, fractional root; only the root call matters
struct tiny { int x; std::vector<int> y; int cols; };
static tiny build_tiny(M & m) {
    const std::size_t Y = 18;
    std::mt19937 rng(9); std::uniform_int_distribution<int> A(5, 60), P(5, 80);
    auto y = m.add_binary_variables(Y);
    auto x = m.add_variables(1, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 5});
    std::vector<double> a(Y), p(Y); for(std::size_t j = 0; j < Y; ++j) { a[j] = A(rng); p[j] = P(rng); }
    const auto ids = std::views::iota(std::size_t{0}, Y);
    m.set_maximization();
    m.set_objective(xsum(ids, [&](std::size_t j) { return p[j] * y(j); }) + 1.0 * x(std::size_t{0}));
    m.add_constraint(xsum(ids, [&](std::size_t j) { return a[j] * y(j); }) <= 203.5);
    tiny t; t.x = m.native_id(x(std::size_t{0})); t.cols = static_cast<int>(m.num_variables());
    for(std::size_t j = 0; j < Y; ++j) t.y.push_back(m.native_id(y(j)));
    return t;
}
static void root_setup(M & m) { scope_settings(m); param(m, P_MIRCUTS, 1); param(m, P_EACHCUTLIM, 0); }
int main() {
    std::println(stderr, "library {}", M{}.native_api().library_path().string());
    // 1. duplicate columns: 0.5 x + 0.5 x <= 1 ; what does the next LP read?
    for(int datacheck : {0, 1, 2}) for(const auto & [name, vals] : std::initializer_list<std::pair<const char *, std::pair<double, double>>>{{"0.5x + 0.5x <= 1", {0.5, 0.5}}, {"0.2x + 0.8x <= 1", {0.2, 0.8}}}) {
        M m; root_setup(m); param(m, 1056, datacheck);
        auto t = build_tiny(m);
        m.set_time_limit(std::chrono::duration<double>(5));
        O o{&m.native_api(), &syms::get(m), t.cols};
        int e = 12345; double first = NAN, after = NAN; int st = 0;
        const auto [v0, v1] = vals;
        o.body = [&, v0, v1](ctxptr ctx, const std::vector<double> & x, cpxlong, O & oo) {
            if(st == 0) { first = x[static_cast<std::size_t>(t.x)]; st = 1; e = add1(oo, ctx, raw_cut{{t.x, t.x}, {v0, v1}, 'L', 1.0}, USECUT_FORCE); return 0; }
            if(st == 1) { after = x[static_cast<std::size_t>(t.x)]; st = 2; }
            return 0;
        };
        auto [env, lp] = m.native_model();
        m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, c::CPX_CALLBACKCONTEXT_RELAXATION, ohook, &o));
        const double s = cp::timed_solve(m);
        std::println(stderr, "== duplicate {} DATACHECK={}: add={} x before={} x at next call={} (sum: 1; last wins: {}; first wins: {}; dropped: 5) | {}", name, datacheck, errtext(m, e), first, after, 1.0 / v1, 1.0 / v0, cp::summary(m, s));
    }
    // 2. malformed data: native codes
    struct bad { const char * name; raw_cut r; };
    const auto badtest = [&](const char * name, std::function<raw_cut(const tiny &)> make, int datacheck) {
        M m; root_setup(m); param(m, 1056, datacheck);
        auto t = build_tiny(m);
        m.set_time_limit(std::chrono::duration<double>(5));
        O o{&m.native_api(), &syms::get(m), t.cols};
        int e = 12345;
        o.body = [&](ctxptr ctx, const std::vector<double> &, cpxlong, O & oo) { if(e == 12345) e = add1(oo, ctx, make(t)); return 0; };
        auto [env, lp] = m.native_model();
        m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, c::CPX_CALLBACKCONTEXT_RELAXATION, ohook, &o));
        const double s = cp::timed_solve(m);
        std::println(stderr, "== malformed {} DATACHECK={}: add={} | {} calls={}", name, datacheck, errtext(m, e), cp::summary(m, s), o.calls);
    };
    for(int dc : {0, 1}) {
        badtest("column -1", [](const tiny &) { return raw_cut{{-1}, {1.0}, 'L', 1e6}; }, dc);
        badtest("column == num cols", [](const tiny & t) { return raw_cut{{t.cols}, {1.0}, 'L', 1e6}; }, dc);
        badtest("sense 'X'", [](const tiny & t) { return raw_cut{{t.x}, {1.0}, 'X', 1e6}; }, dc);
        badtest("sense 'R'", [](const tiny & t) { return raw_cut{{t.x}, {1.0}, 'R', 1e6}; }, dc);
        badtest("rhs infinity", [](const tiny & t) { return raw_cut{{t.x}, {1.0}, 'L', std::numeric_limits<double>::infinity()}; }, dc);
        badtest("rhs NaN", [](const tiny & t) { return raw_cut{{t.x}, {1.0}, 'L', std::numeric_limits<double>::quiet_NaN()}; }, dc);
        badtest("coefficient NaN", [](const tiny & t) { return raw_cut{{t.x}, {std::numeric_limits<double>::quiet_NaN()}, 'L', 1e6}; }, dc);
    }
    // 3. equality: enforced?
    {
        M m; root_setup(m);
        auto g = cp::build_gadgets(m, 6, false);
        m.set_time_limit(std::chrono::duration<double>(5));
        O o{&m.native_api(), &syms::get(m), static_cast<int>(m.num_variables())};
        int e = 12345; long checks = 0, is_one = 0;
        o.body = [&](ctxptr ctx, const std::vector<double> & x, cpxlong, O & oo) {
            if(e == 12345) { e = add1(oo, ctx, raw_cut{{g.dummy}, {1.0}, 'E', 1.0}, USECUT_FORCE); return 0; }
            ++checks; is_one += std::fabs(x[static_cast<std::size_t>(g.dummy)] - 1) < 1e-6;
            return 0;
        };
        auto [env, lp] = m.native_model();
        m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, c::CPX_CALLBACKCONTEXT_RELAXATION, ohook, &o));
        const double s = cp::timed_solve(m);
        std::println(stderr, "== equality dummy == 1, force: add={} later calls with dummy==1: {}/{} | {}", errtext(m, e), is_one, checks, cp::summary(m, s));
    }
    // 4. batching: rcnt=2 in one call, and two calls in one invocation
    {
        M m; root_setup(m);
        auto g = cp::build_gadgets(m, 8, false);
        m.set_time_limit(std::chrono::duration<double>(5));
        O o{&m.native_api(), &syms::get(m), static_cast<int>(m.num_variables())};
        int e1 = 12345, e2 = 12345; long ck = 0, v = 0;
        const auto a = g.hull_cut(1), b = g.hull_cut(2), d = g.hull_cut(3);
        o.body = [&](ctxptr ctx, const std::vector<double> & x, cpxlong, O & oo) {
            if(e1 == 12345) {
                std::vector<double> rhs{a.rhs, b.rhs}, val; std::vector<char> sense{a.sense, b.sense}; std::vector<int> beg{0, static_cast<int>(a.ind.size())}, ind, pg{USECUT_FORCE, USECUT_FORCE}, lc{0, 0};
                ind = a.ind; ind.insert(ind.end(), b.ind.begin(), b.ind.end()); val = a.val; val.insert(val.end(), b.val.begin(), b.val.end());
                e1 = oo.api->callbackaddusercuts(ctx, 2, static_cast<int>(ind.size()), rhs.data(), sense.data(), beg.data(), ind.data(), val.data(), pg.data(), lc.data());
                e2 = add1(oo, ctx, d, USECUT_FORCE);
                return 0;
            }
            ++ck; v += a.violation(x) > 1e-6 || b.violation(x) > 1e-6 || d.violation(x) > 1e-6;
            return 0;
        };
        auto [env, lp] = m.native_model();
        m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, c::CPX_CALLBACKCONTEXT_RELAXATION, ohook, &o));
        const double s = cp::timed_solve(m);
        int n = -1; syms::get(m).numcuts(env, lp, CUT_USER, &n);
        std::println(stderr, "== batch rcnt=2 then rcnt=1 in one invocation: errs={},{} later viol/checks={}/{} numcuts={} | {}", e1, e2, v, ck, n, cp::summary(m, s));
    }
    // 5. what the hook returns
    for(int rc : {1, 1006, -1}) {
        M m; root_setup(m);
        auto g = cp::build_gadgets(m, 8, false);
        m.set_time_limit(std::chrono::duration<double>(5));
        O o{&m.native_api(), &syms::get(m), static_cast<int>(m.num_variables())};
        o.body = [rc](ctxptr, const std::vector<double> &, cpxlong, O &) { return rc; };
        auto [env, lp] = m.native_model(); auto & api = m.native_api();
        api._check(env, api.callbacksetfunc(env, lp, c::CPX_CALLBACKCONTEXT_RELAXATION, ohook, &o));
        const int e = api.mipopt(env, lp);
        std::println(stderr, "== hook returns {}: CPXmipopt={} CPXgetstat={} calls={}", rc, errtext(m, e), api.getstat(env, lp), o.calls);
    }
    {
        M m; root_setup(m);
        auto g = cp::build_gadgets(m, 8, false);
        m.set_time_limit(std::chrono::duration<double>(5));
        O o{&m.native_api(), &syms::get(m), static_cast<int>(m.num_variables())};
        o.body = [](ctxptr ctx, const std::vector<double> &, cpxlong, O & oo) { oo.api->callbackabort(ctx); return 0; };
        auto [env, lp] = m.native_model(); auto & api = m.native_api();
        api._check(env, api.callbacksetfunc(env, lp, c::CPX_CALLBACKCONTEXT_RELAXATION, ohook, &o));
        const double s = cp::timed_solve(m);
        std::println(stderr, "== CPXcallbackabort in the hook: {} calls={}", cp::summary(m, s), o.calls);
    }
    // 6. legacy mix: a legacy user cut callback plus a generic callback
    using legacy_cb = int (*)(c::CPXCENVptr, void *, int, void *, int *);
    using setlegacy = int (*)(c::CPXENVptr, legacy_cb, void *);
    {
        M m; root_setup(m);
        auto g = cp::build_gadgets(m, 8, false);
        m.set_time_limit(std::chrono::duration<double>(5));
        O o{&m.native_api(), &syms::get(m), static_cast<int>(m.num_variables())};
        o.body = [](ctxptr, const std::vector<double> &, cpxlong, O &) { return 0; };
        auto [env, lp] = m.native_model(); auto & api = m.native_api();
        api._check(env, api.callbacksetfunc(env, lp, c::CPX_CALLBACKCONTEXT_RELAXATION, ohook, &o));
        const auto f = cp::symf<setlegacy>(api.library_path(), "CPXsetusercutcallbackfunc");
        static long legacy_calls = 0;
        const int es = f(env, [](c::CPXCENVptr, void *, int, void *, int * action) -> int { ++legacy_calls; *action = 0; return 0; }, nullptr);
        const int e = api.mipopt(env, lp);
        std::println(stderr, "== legacy user cut callback + generic: setter={} CPXmipopt={} generic calls={} legacy calls={}", errtext(m, es), errtext(m, e), o.calls, legacy_calls);
        api._check(env, f(env, nullptr, nullptr));
        api._check(env, api.callbacksetfunc(env, lp, 0, nullptr, nullptr));
        const int e2 = api.mipopt(env, lp);
        std::println(stderr, "   after clearing both: CPXmipopt={}", errtext(m, e2));
    }
    // 7. a user cut pool before the search plus a generic hook
    using addpool = int (*)(c::CPXCENVptr, c::CPXLPptr, int, int, const double *, const char *, const int *, const int *, const double *, char **);
    using numpool = int (*)(c::CPXCENVptr, c::CPXCLPptr);
    {
        M m; scope_settings(m); param(m, P_MIRCUTS, 1); param(m, P_EACHCUTLIM, 0); param(m, P_SCRIND, 0);
        auto g = cp::build_gadgets(m, 10, false);
        m.set_time_limit(std::chrono::duration<double>(10));
        O o{&m.native_api(), &syms::get(m), static_cast<int>(m.num_variables())};
        const auto pool = g.hull_cut(3);
        long v = 0, ck = 0;
        o.body = [&](ctxptr, const std::vector<double> & x, cpxlong, O &) { ++ck; v += pool.violation(x) > 1e-6; return 0; };
        auto [env, lp] = m.native_model(); auto & api = m.native_api();
        const int beg = 0;
        const int ea = cp::symf<addpool>(api.library_path(), "CPXaddusercuts")(env, lp, 1, static_cast<int>(pool.ind.size()), &pool.rhs, &pool.sense, &beg, pool.ind.data(), pool.val.data(), nullptr);
        api._check(env, api.callbacksetfunc(env, lp, c::CPX_CALLBACKCONTEXT_RELAXATION, ohook, &o));
        const double s = cp::timed_solve(m);
        int n = -1; syms::get(m).numcuts(env, lp, CUT_USER, &n);
        std::println(stderr, "== CPXaddusercuts pool + generic hook: add={} {} calls={} pool-cut viol/checks at the hook={}/{} numcuts={}", errtext(m, ea), cp::summary(m, s), o.calls, v, ck, n);
    }
    // 8. lpopt with a generic callback registered; the model has no integers
    {
        M m;
        auto x = m.add_variables(2, {.obj_coef = 1, .lower_bound = 0, .upper_bound = 1});
        m.set_maximization();
        m.add_constraint(1.0 * x(std::size_t{0}) + 1.0 * x(std::size_t{1}) <= 1.5);
        O o{&m.native_api(), &syms::get(m), 2};
        o.body = [](ctxptr, const std::vector<double> &, cpxlong, O &) { return 0; };
        auto [env, lp] = m.native_model(); auto & api = m.native_api();
        api._check(env, api.callbacksetfunc(env, lp, c::CPX_CALLBACKCONTEXT_RELAXATION, ohook, &o));
        std::println(stderr, "== no integers, probtype={}: CPXlpopt with the hook registered={} calls={}", api.getprobtype(env, lp), errtext(m, api.lpopt(env, lp)), o.calls);
    }
    // 9. a non-violated cut at every call at the root, at plain defaults: does a realistic model hang?
    for(const std::string mode : {"global-purge", "global-force", "global-filter"}) for(int cutpass : {0, 5}) {
        M m; param(m, P_THREADS, 1); param(m, P_CUTPASS, cutpass);
        auto a = cp::build_atsp(m, 26);
        m.set_time_limit(std::chrono::duration<double>(5));
        O o{&m.native_api(), &syms::get(m), static_cast<int>(m.num_variables())};
        const auto [pg, lc] = mode_flags(mode);
        std::optional<raw_cut> kept; long root = 0;
        o.body = [&, pg, lc](ctxptr ctx, const std::vector<double> & x, cpxlong d, O & oo) {
            root += d == 0;
            if(!kept) { auto cs = a.separate(x, 1); if(!cs.empty()) kept = cs[0]; else return 0; }
            add1(oo, ctx, *kept, pg, lc);  // the same cut, satisfied from the second call on
            return 0;
        };
        auto [env, lp] = m.native_model();
        m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, c::CPX_CALLBACKCONTEXT_RELAXATION, ohook, &o));
        const double s = cp::timed_solve(m);
        std::println(stderr, "== re-adding one satisfied DFJ cut at every call, defaults, {} CUTPASS={}: {} calls={} root calls={}", mode, cutpass, cp::summary(m, s), o.calls, root);
    }
    // 10. dominance: a cut that removes non-optimal integer points only, y0 fixed to its optimal value
    for(const std::string mode : {"global-purge", "global-force"}) for(bool presolve_on : {false, true}) {
        M m; scope_settings(m);
        if(presolve_on) { param(m, P_PREIND, 1); param(m, P_REDUCE, 3); param(m, P_REFORM, 3); param(m, P_AGGIND, -1); }
        auto g = cp::build_gadgets(m, 10, false);
        m.set_time_limit(std::chrono::duration<double>(5));
        O o{&m.native_api(), &syms::get(m), static_cast<int>(m.num_variables())};
        const auto [pg, lc] = mode_flags(mode);
        const raw_cut cut{{g.y[0]}, {1.0}, 'L', 0.0};  // y0 <= 0: flip=false, the optimum has y0=0; all y0=1 points are worse
        int e = 12345;
        o.body = [&, pg, lc](ctxptr ctx, const std::vector<double> & x, cpxlong, O & oo) { if(e == 12345 && cut.violation(x) > 1e-6) e = add1(oo, ctx, cut, pg, lc); return 0; };
        auto [env, lp] = m.native_model();
        m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, c::CPX_CALLBACKCONTEXT_RELAXATION, ohook, &o));
        const double s = cp::timed_solve(m);
        std::println(stderr, "== dominance cut y0<=0 {} presolve={}: add={} {} calls={} (known opt {})", mode, presolve_on, e == 12345 ? "never added" : errtext(m, e), cp::summary(m, s), o.calls, g.opt);
    }
}
