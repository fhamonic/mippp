// Q6: what the log says with no hook, a RELAXATION hook, a CANDIDATE hook.
#include "probe_cplex_common.cpp"
static int noop(ctxptr, cpxlong, void *) { return 0; }
int main() {
    const c::CPXCALLBACKFUNC * none = nullptr; (void)none;
    for(const char * inst : {"atsp26", "mkp"}) for(auto [nm, mask] : std::initializer_list<std::pair<const char *, cpxlong>>{{"no hook", 0}, {"R", c::CPX_CALLBACKCONTEXT_RELAXATION}, {"C", c::CPX_CALLBACKCONTEXT_CANDIDATE}, {"C|R", c::CPX_CALLBACKCONTEXT_CANDIDATE | c::CPX_CALLBACKCONTEXT_RELAXATION}}) {
        M m; param(m, P_SCRIND, 1); param(m, P_MIPDISPLAY, 1);
        std::println("#### {} {}", inst, nm); std::fflush(stdout);
        if(std::string(inst) == "mkp") cp::build_mkp(m, 30, 100); else cp::build_atsp(m, 26);
        m.set_time_limit(std::chrono::duration<double>(1));
        if(mask) { auto [env, lp] = m.native_model(); m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, mask, noop, nullptr)); }
        const double t = cp::timed_solve(m);
        std::println("#### result {}", cp::summary(m, t)); std::fflush(stdout);
    }
    // the pool, at settings where it can be used
    using addpool = int (*)(c::CPXCENVptr, c::CPXLPptr, int, int, const double *, const char *, const int *, const int *, const double *, char **);
    for(bool hook_on : {false, true}) {
        M m; scope_settings(m); param(m, P_MIRCUTS, 1); 
        auto g = cp::build_gadgets(m, 10, false);
        m.set_time_limit(std::chrono::duration<double>(10));
        const auto pool = g.hull_cut(3);
        auto [env, lp] = m.native_model(); auto & api = m.native_api();
        const int beg = 0;
        const int ea = cp::symf<addpool>(api.library_path(), "CPXaddusercuts")(env, lp, 1, static_cast<int>(pool.ind.size()), &pool.rhs, &pool.sense, &beg, pool.ind.data(), pool.val.data(), nullptr);
        if(hook_on) api._check(env, api.callbacksetfunc(env, lp, c::CPX_CALLBACKCONTEXT_RELAXATION, noop, nullptr));
        const double t = cp::timed_solve(m);
        int n = -1; syms::get(m).numcuts(env, lp, CUT_USER, &n);
        std::println(stderr, "== pool without EACHCUTLIM=0, hook={}: add={} {} numcuts={}", hook_on, ea, cp::summary(m, t), n);
    }
}
