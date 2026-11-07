// Q7: when does the RELAXATION context fire at the root? Bisects the cut families.
#include "probe_cplex_common.cpp"
static const std::vector<std::pair<const char *, int>> families = {
    {"Cliques", P_CLIQUES}, {"Covers", P_COVERS}, {"Disj", P_DISJCUTS}, {"FlowCovers", P_FLOWCOVERS}, {"FlowPaths", P_FLOWPATHS},
    {"Gomory", P_FRACCUTS}, {"GUB", P_GUBCOVERS}, {"Implied", P_IMPLBD}, {"LiftProj", P_LANDP}, {"MCF", P_MCF}, {"MIR", P_MIRCUTS},
    {"ZeroHalf", P_ZEROHALF}, {"LocalImplied", P_LOCALIMPLBD}, {"BQP", P_BQP}, {"RLT", P_RLT}};
struct C { const syms * s; long calls = 0, depth0 = 0; cpxlong first_depth = -1; };
static int h(ctxptr ctx, cpxlong id, void * usr) {
    auto & t = *static_cast<C *>(usr);
    if(id != c::CPX_CALLBACKCONTEXT_RELAXATION) return 0;
    cpxlong d = -1; t.s->infolong(ctx, INFO_NODEDEPTH, &d);
    if(t.calls == 0) t.first_depth = d;
    t.calls++; t.depth0 += d == 0;
    return 0;
}
template <class Build>
static void run(const std::string & name, Build build, std::function<void(M &)> cfg) {
    M m; cfg(m); build(m);
    m.set_time_limit(std::chrono::duration<double>(20));
    C t{&syms::get(m)};
    auto [env, lp] = m.native_model();
    m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, c::CPX_CALLBACKCONTEXT_RELAXATION, h, &t));
    const double s = cp::timed_solve(m);
    std::println(stderr, "{:<55} calls={} depth0_calls={} first_depth={} | {}", name, t.calls, t.depth0, t.first_depth, cp::summary(m, s));
}
int main() {
    auto gad = [](M & m) { cp::build_gadgets(m, 10, false); };
    auto ats = [](M & m) { cp::build_atsp(m, 26); };
    auto off = [](M & m) { for(auto [n, p] : families) param(m, p, -1); };
    run("gadgets, all defaults", gad, [](M & m) { param(m, P_THREADS, 1); });
    run("gadgets, defaults, all families -1", gad, [&](M & m) { param(m, P_THREADS, 1); off(m); });
    run("atsp26, all defaults", ats, [](M & m) { param(m, P_THREADS, 1); });
    run("atsp26, defaults, all families -1", ats, [&](M & m) { param(m, P_THREADS, 1); off(m); });
    run("atsp26, scope settings", ats, [](M & m) { scope_settings(m); });
    for(auto [n, p] : families) {
        const int pp = p;
        run(std::format("gadgets, scope, only {} on (0)", n), gad, [pp](M & m) { scope_settings(m); param(m, pp, 0); });
    }
    run("gadgets, scope, Covers=2", gad, [](M & m) { scope_settings(m); param(m, P_COVERS, 2); });
    run("gadgets, scope, MIR=1", gad, [](M & m) { scope_settings(m); param(m, P_MIRCUTS, 1); });
    run("gadgets, scope, MIR=1, CUTPASS=1", gad, [](M & m) { scope_settings(m); param(m, P_MIRCUTS, 1); param(m, P_CUTPASS, 1); });
    run("gadgets, scope, MIR=1, EACHCUTLIM=0", gad, [](M & m) { scope_settings(m); param(m, P_MIRCUTS, 1); param(m, P_EACHCUTLIM, 0); });
    run("gadgets, scope, MIR=1, CUTPASS=-1", gad, [](M & m) { scope_settings(m); param(m, P_MIRCUTS, 1); param(m, P_CUTPASS, -1); });
    run("atsp26, default, CUTPASS=-1", ats, [](M & m) { param(m, P_THREADS, 1); param(m, P_CUTPASS, -1); });
    run("atsp26, default, EACHCUTLIM=0", ats, [](M & m) { param(m, P_THREADS, 1); param(m, P_EACHCUTLIM, 0); });
    run("atsp26, default, Nodecuts=-1", ats, [](M & m) { param(m, P_THREADS, 1); param(m, P_NODECUTS, -1); });
}
