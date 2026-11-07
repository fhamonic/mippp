// Q5/Q6: column space, crushing, and what the hook does to presolve.
#include "probe_cplex_common.cpp"
using cp::raw_cut;
static void banner(const std::string & s) { std::println("#### {}", s); std::fflush(stdout); }
// families off except MIR, so that the root fires; EACHCUTLIM=0 so that MIR yields nothing: only DFJ acts
static void only_user_cuts(M & m) {
    for(int p : {P_CLIQUES, P_COVERS, P_DISJCUTS, P_FLOWCOVERS, P_FLOWPATHS, P_FRACCUTS, P_GUBCOVERS, P_IMPLBD, P_LANDP, P_MCF, P_ZEROHALF, P_LOCALIMPLBD, P_BQP, P_RLT})
        param(m, p, -1);
    param(m, P_EACHCUTLIM, 0);
    param(m, P_HEURFREQ, -1); param(m, P_RINS, -1); param(m, P_FPHEUR, -1); param(m, P_LBHEUR, 0);
    auto [env, lp] = m.native_model(); m.native_api()._check(env, m.native_api().setdblparam(env, P_HEUREFFORT, 0.0));
    param(m, P_THREADS, 1);
}
static void logon(M & m) { param(m, P_SCRIND, 1); param(m, P_MIPDISPLAY, 1); }

static void e4(const std::string & name, int n, bool complements, bool hook_on, std::function<void(M &)> cfg, double tl = 60) {
    banner(name);
    M m; only_user_cuts(m); cfg(m); logon(m);
    auto a = cp::build_atsp(m, n, complements);
    m.set_time_limit(std::chrono::duration<double>(tl));
    auto s = std::make_shared<cp::stats>();
    auto added = std::make_shared<std::vector<raw_cut>>();
    long prev_viol = 0;
    if(hook_on) attach(m, cp::watched([&](const auto & x, const cp::add_fn & add) {
        for(const auto & old : *added) if(old.violation(x) > 1e-5) prev_viol++;
        for(const auto & c : a.separate(x)) { if(add(c) == 0) added->push_back(c); }
    }, s, [&a](const auto & x) { return a.objective(x); }, [&a](const auto & x) { return a.point_ok(x); }, [&a](const auto & x) { return a.integral(x); }), "global-purge");
    const double t = cp::timed_solve(m);
    std::println(stderr, "== {}: {}", name, cp::summary(m, t));
    if(hook_on) { std::println(stderr, "   {} previously-added-cuts-violated={}", s->line(), prev_viol); flush_diag(); }
    std::fflush(stderr);
}

// duplicate general-integer columns: x1, x2 identical everywhere
struct dupm { int x1, x2; std::vector<int> y; };
static dupm build_dup(M & m, bool continuous_dups) {
    dupm d;
    const std::size_t Y = 25;
    std::mt19937 rng(11);
    std::uniform_int_distribution<int> A(5, 60), P(5, 80);
    auto y = m.add_binary_variables(Y);
    auto add_x = [&](std::size_t n) {
        if(continuous_dups) return m.add_variables(n, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 9});
        return m.add_integer_variables(n, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 9});
    };
    auto x = add_x(2);
    d.x1 = m.native_id(x(std::size_t{0})); d.x2 = m.native_id(x(std::size_t{1}));
    for(std::size_t j = 0; j < Y; ++j) d.y.push_back(m.native_id(y(j)));
    std::vector<double> p(Y), a1(Y), a2(Y);
    for(std::size_t j = 0; j < Y; ++j) { p[j] = P(rng); a1[j] = A(rng); a2[j] = A(rng); }
    const auto ids = std::views::iota(std::size_t{0}, Y);
    m.set_maximization();
    m.set_objective(xsum(ids, [&](std::size_t j) { return p[j] * y(j); }) + 17.0 * x(std::size_t{0}) + 17.0 * x(std::size_t{1}));
    m.add_constraint(xsum(ids, [&](std::size_t j) { return a1[j] * y(j); }) + 11.0 * x(std::size_t{0}) + 11.0 * x(std::size_t{1}) <= 263.5);
    m.add_constraint(xsum(ids, [&](std::size_t j) { return a2[j] * y(j); }) + 13.0 * x(std::size_t{0}) + 13.0 * x(std::size_t{1}) <= 281.5);
    return d;
}
using presolve_fn = int (*)(c::CPXCENVptr, c::CPXLPptr, int);
using prestat_fn = int (*)(c::CPXCENVptr, c::CPXCLPptr, int *, int *, int *, int *, int *);
using crush_fn = int (*)(c::CPXCENVptr, c::CPXCLPptr, int, const int *, const double *, int *, double *, int *, double *);
static void crush_check(const std::string & name, bool continuous_dups, std::function<void(M &)> cfg) {
    M m; param(m, P_THREADS, 1); cfg(m);
    auto d = build_dup(m, continuous_dups);
    auto [env, lp] = m.native_model(); auto & api = m.native_api();
    const auto & path = api.library_path();
    const int n = static_cast<int>(m.num_variables());
    const int pe = cp::symf<presolve_fn>(path, "CPXpresolve")(env, lp, 0);
    int pre = 0; std::vector<int> pc(static_cast<std::size_t>(n));
    const int ps = cp::symf<prestat_fn>(path, "CPXgetprestat")(env, lp, &pre, pc.data(), nullptr, nullptr, nullptr);
    const int ind = d.x1; const double val = 1;
    int plen = 0; double off = 0; std::vector<int> pind(static_cast<std::size_t>(n)); std::vector<double> pval(static_cast<std::size_t>(n));
    const int ce = cp::symf<crush_fn>(path, "CPXcrushform")(env, lp, 1, &ind, &val, &plen, &off, pind.data(), pval.data());
    std::println(stderr, "== crush {}: CPXpresolve={} prestat_err={} prestat={} pcstat[x1]={} pcstat[x2]={} CPXcrushform(x1)={} plen={} off={}", name, pe, ps, pre, pc[static_cast<std::size_t>(d.x1)], pc[static_cast<std::size_t>(d.x2)], ce, plen, off);
    std::fflush(stderr);
}
static void dup_run(const std::string & name, bool continuous_dups, std::function<void(M &)> cfg) {
    banner(name);
    M m; only_user_cuts(m); cfg(m); logon(m);
    auto d = build_dup(m, continuous_dups);
    m.set_time_limit(std::chrono::duration<double>(10));
    auto s = std::make_shared<cp::stats>();
    int err = 0; long tried = 0, kept_viol = 0, checks = 0; double first_x1 = NAN, first_x2 = NAN;
    std::optional<raw_cut> cut;
    attach(m, cp::watched([&](const auto & x, const cp::add_fn & add) {
        const double x1 = x[static_cast<std::size_t>(d.x1)], x2 = x[static_cast<std::size_t>(d.x2)];
        if(cut) { ++checks; kept_viol += cut->violation(x) > 1e-5; return; }
        first_x1 = x1; first_x2 = x2;
        if(x1 > 0.5) {
            cut = raw_cut{{d.x1}, {1.0}, 'L', std::max(0.0, std::floor(x1) - 0.0 > x1 - 1e-6 ? x1 - 0.5 : std::floor(x1))};
            ++tried; err = add(*cut);
        } else if(x2 > 0.5) {  // the split went to x2: same form on x2 alone
            cut = raw_cut{{d.x2}, {1.0}, 'L', std::max(0.0, std::floor(x2) > x2 - 1e-6 ? x2 - 0.5 : std::floor(x2))};
            ++tried; err = add(*cut);
        }
    }, s), "global-force");
    const double t = cp::timed_solve(m);
    std::println(stderr, "== {}: {}\n   first point x1={} x2={} cut tried={} native error={} then viol/checks={}/{}\n   {}", name, cp::summary(m, t), first_x1, first_x2, tried, err, kept_viol, checks, s->line());
    flush_diag();
    std::fflush(stderr);
}
int main() {
    std::println(stderr, "library {}", M{}.native_api().library_path().string());
    const auto none = [](M &) {};
    for(int n : {20, 22}) {
        e4(std::format("atsp n={} no complements, no hook, only_user_cuts", n), n, false, false, none);
        e4(std::format("atsp n={} complements, no hook, only_user_cuts", n), n, true, false, none);
        e4(std::format("atsp n={} no complements, DFJ on originals, only_user_cuts", n), n, false, true, none);
        e4(std::format("atsp n={} complements, DFJ on complements, default reformulations", n), n, true, true, none);
    }
    e4("atsp n=22 complements, DFJ on complements, Reformulations=0", 22, true, true, [](M & m) { param(m, P_REFORM, 0); });
    e4("atsp n=22 complements, DFJ on complements, PRELINEAR=0", 22, true, true, [](M & m) { param(m, P_PRELINEAR, 0); });
    e4("atsp n=22 complements, DFJ on complements, MIPCBREDLP=0", 22, true, true, [](M & m) { param(m, P_MIPCBREDLP, 0); });
    e4("atsp n=22 complements, DFJ on complements, MIPCBREDLP=1 explicit", 22, true, true, [](M & m) { param(m, P_MIPCBREDLP, 1); });
    e4("atsp n=22 complements, DFJ on complements, presolve off", 22, true, true, [](M & m) { param(m, P_PREIND, 0); });
    for(bool cont : {false, true}) {
        const std::string k = cont ? "continuous" : "integer";
        crush_check(k + " dups, default", cont, none);
        crush_check(k + " dups, Reformulations=0", cont, [](M & m) { param(m, P_REFORM, 0); });
        crush_check(k + " dups, PRELINEAR=0", cont, [](M & m) { param(m, P_PRELINEAR, 0); });
        dup_run(k + " dups, hook, default", cont, none);
        dup_run(k + " dups, hook, Reformulations=0", cont, [](M & m) { param(m, P_REFORM, 0); });
        dup_run(k + " dups, hook, Reformulations=2 (no form-crushing obstacles)", cont, [](M & m) { param(m, P_REFORM, 2); });
        dup_run(k + " dups, hook, PRELINEAR=0", cont, [](M & m) { param(m, P_PRELINEAR, 0); });
    }
}
