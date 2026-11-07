// Which setting silences CBCONTEXT_MIPRELAX? S1/S2 under the full scope_settings got
// calls=0 with 2535 nodes. No-op hook, one thread.
#include "probe_copt_common.cpp"
using namespace std::chrono;
using params = std::vector<std::pair<const char *, int>>;
template <class B, class O, class P>
static void run(const char * inst, const std::string & label, const params & base, const params & extra, B build, O obj, P ok) {
    M m; m.set_thread_limit(1);
    auto g = build(m);
    m.set_time_limit(duration<double>(20.0));
    auto & api = m.native_api();
    auto [env, prob] = m.native_model();
    for(auto [name, v] : base) api._check(env, api.SetIntParam(prob, name, v));
    for(auto [name, v] : extra) api._check(env, api.SetIntParam(prob, name, v));
    auto s = std::make_shared<cp::stats>();
    attach(m, cp::watched([](const auto &, const auto &) {}, s, [&g, obj](const auto & x) { return obj(g, x); }, [&g, ok](const auto & x) { return ok(g, x); }), "usercut");
    const double t = cp::timed_solve(m);
    std::println(stderr, "{} {:40} {} | relax={} node={} bad_points={} obj_range={}", inst, label, cp::summary(m, t), m.hook->relax_calls.load(), m.hook->node_calls.load(), s->bad_points.load(), s->line().substr(s->line().find("obj_first")));
}
int main() {
    const params heur_off = {{"HeurLevel", 0}, {"PreRootHeurLevel", 0}, {"RoundingHeurLevel", 0}, {"DivingHeurLevel", 0}, {"SubMipHeurLevel", 0}, {"FAPHeurLevel", 0}, {"ConflictAnalysis", 0}, {"MipTasks", 1}};
    params base = heur_off; base.push_back({"Presolve", 0});
    const std::vector<std::pair<std::string, params>> grid = {
        {"auto cuts", {}},
        {"CutLevel=0", {{"CutLevel", 0}}},
        {"RootCutLevel=0", {{"RootCutLevel", 0}}},
        {"TreeCutLevel=0", {{"TreeCutLevel", 0}}},
        {"RootCutLevel=0 TreeCutLevel=0", {{"RootCutLevel", 0}, {"TreeCutLevel", 0}}},
        {"CutLevel=0 RootCutLevel=1", {{"CutLevel", 0}, {"RootCutLevel", 1}}},
        {"CutLevel=0 TreeCutLevel=1", {{"CutLevel", 0}, {"TreeCutLevel", 1}}},
        {"CutLevel=1", {{"CutLevel", 1}}},
        {"RootCutRounds=0", {{"RootCutRounds", 0}}},
        {"NodeCutRounds=0", {{"NodeCutRounds", 0}}},
        {"RootCutRounds=0 NodeCutRounds=0", {{"RootCutRounds", 0}, {"NodeCutRounds", 0}}},
        {"RootCutRounds=1", {{"RootCutRounds", 1}}},
        {"NodeCutRounds=1", {{"NodeCutRounds", 1}}},
        {"RootCutLevel=0 NodeCutRounds=1", {{"RootCutLevel", 0}, {"NodeCutRounds", 1}}},
        {"RootCutLevel=0 TreeCutLevel=0 rounds=5", {{"RootCutLevel", 0}, {"TreeCutLevel", 0}, {"RootCutRounds", 5}, {"NodeCutRounds", 5}}},
    };
    const auto gb = [](M & m) { return cp::build_gadgets(m, 10, false); };
    const auto gobj = [](const auto & g, const auto & x) { return g.objective(x); };
    const auto gok = [](const auto & g, const auto & x) { return g.point_ok(x); };
    for(auto & [label, p] : grid) run("gadgets presolve-off", label, base, p, gb, gobj, gok);
    const auto ab = [](M & m) { return cp::build_atsp(m, 26); };
    const auto aobj = [](const auto & a, const auto & x) { return a.objective(x); };
    const auto aok = [](const auto & a, const auto & x) { return a.point_ok(x); };
    for(auto & [label, p] : grid) run("atsp26 presolve-off", label, base, p, ab, aobj, aok);
    for(auto & [label, p] : grid) run("atsp26 presolve-on", label, {}, p, ab, aobj, aok);
}
