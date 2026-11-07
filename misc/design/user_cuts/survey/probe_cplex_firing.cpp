// Q7: per-call trace. Which nodes fire, in which order, at which depth?
#include "probe_cplex_common.cpp"
struct rec { cpxlong uid, depth, nodecount; int after, restarts; double y0, obj; };
struct tr_ctx {
    const c::cplex_api * api; const syms * s; int cols; int y0 = -1;
    std::function<int(ctxptr, const std::vector<double> &, const rec &)> on_call;
    std::vector<rec> recs;
    long fired = 0;
};
static int tr_hook(ctxptr ctx, cpxlong id, void * usr) {
    auto & t = *static_cast<tr_ctx *>(usr);
    if(id != c::CPX_CALLBACKCONTEXT_RELAXATION) return 0;
    std::vector<double> x(static_cast<std::size_t>(t.cols));
    rec r{};
    t.s->infolong(ctx, INFO_NODEUID, &r.uid);
    t.s->infolong(ctx, INFO_NODEDEPTH, &r.depth);
    t.s->infolong(ctx, INFO_NODECOUNT, &r.nodecount);
    t.s->infoint(ctx, INFO_AFTERCUTLOOP, &r.after);
    t.s->infoint(ctx, INFO_RESTARTS, &r.restarts);
    t.s->point(ctx, x.data(), 0, t.cols - 1, &r.obj);
    r.y0 = t.y0 >= 0 ? x[static_cast<std::size_t>(t.y0)] : NAN;
    t.recs.push_back(r);
    return t.on_call ? t.on_call(ctx, x, r) : 0;
}
static void set_families(M & m, int v) {
    for(int p : {P_CLIQUES, P_COVERS, P_DISJCUTS, P_FLOWCOVERS, P_FLOWPATHS, P_FRACCUTS, P_GUBCOVERS, P_IMPLBD, P_LANDP, P_MCF, P_MIRCUTS, P_ZEROHALF, P_LOCALIMPLBD, P_BQP, P_RLT})
        param(m, p, v);
}
static void report(const std::string & name, tr_ctx & t, M & m, double secs, int show) {
    std::map<cpxlong, int> per_node;
    long depth0 = 0, after = 0;
    for(auto & r : t.recs) { per_node[r.uid]++; depth0 += r.depth == 0; after += r.after; }
    std::map<int, long> hist;
    for(auto [u, n] : per_node) hist[n]++;
    std::string h;
    for(auto [n, k] : hist) h += std::format(" {}x:{}", n, k);
    std::println(stderr, "== {}: {}\n   calls={} depth0_calls={} after_cutloop={} distinct_nodes={} calls-per-node histogram:{}", name, cp::summary(m, secs), t.recs.size(), depth0, after, per_node.size(), h);
    for(int i = 0; i < show && i < static_cast<int>(t.recs.size()); ++i) {
        auto & r = t.recs[static_cast<std::size_t>(i)];
        std::println(stderr, "     call {:2}: uid={} depth={} nodecount={} after_cutloop={} restarts={} y0={:.4f} obj={:.5g}", i, r.uid, r.depth, r.nodecount, r.after, r.restarts, r.y0, r.obj);
    }
}
static void run_gadgets(const std::string & name, std::function<void(M &)> cfg, int show = 6) {
    M m; scope_settings(m); cfg(m);
    auto g = cp::build_gadgets(m, 10, false);
    m.set_time_limit(std::chrono::duration<double>(30));
    tr_ctx t{&m.native_api(), &syms::get(m), static_cast<int>(m.num_variables()), g.y[0]};
    auto [env, lp] = m.native_model();
    m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, c::CPX_CALLBACKCONTEXT_RELAXATION, tr_hook, &t));
    const double s = cp::timed_solve(m);
    report(name, t, m, s, show);
}
int main() {
    std::println(stderr, "library {}", M{}.native_api().library_path().string());
    // the known LP optimum of the gadget instance: nothing in the hook, solve the relaxation natively
    {
        M m; scope_settings(m); auto g = cp::build_gadgets(m, 10, false);
        auto [env, lp] = m.native_model(); auto & api = m.native_api();
        api._check(env, api.chgprobtype(env, lp, c::CPXPROB_LP));
        api._check(env, api.lpopt(env, lp));
        double v = 0, y0 = 0; 
        cp::symf<int (*)(c::CPXCENVptr, c::CPXCLPptr, double *)>(api.library_path(), "CPXgetobjval")(env, lp, &v);
        cp::symf<int (*)(c::CPXCENVptr, c::CPXCLPptr, double *, int, int)>(api.library_path(), "CPXgetx")(env, lp, &y0, g.y[0], g.y[0]);
        std::println(stderr, "== root LP of gadgets k=10 flip=false: obj={:.6g} y0={:.5f}", v, y0);
    }
    run_gadgets("A scope settings, traditional search, families -1", [](M &) {});
    run_gadgets("B same, search auto", [](M & m) { param(m, P_MIPSEARCH, 0); });
    run_gadgets("C same, search dynamic", [](M & m) { param(m, P_MIPSEARCH, 2); });
    run_gadgets("D traditional, families back to 0 (auto)", [](M & m) { set_families(m, 0); });
    run_gadgets("E traditional, families -1, EACHCUTLIM untouched, Nodecuts default, presolve on", [](M & m) { param(m, P_PREIND, 1); });
    run_gadgets("F traditional, families -1, CUTPASS=5", [](M & m) { param(m, P_CUTPASS, 5); });
    run_gadgets("G traditional, families -1, Nodecuts=-1", [](M & m) { param(m, P_NODECUTS, -1); });
    run_gadgets("H traditional, families -1, CUTPASS=-1", [](M & m) { param(m, P_CUTPASS, -1); });
    run_gadgets("I traditional, families -1, EACHCUTLIM=0", [](M & m) { param(m, P_EACHCUTLIM, 0); });
    run_gadgets("J traditional, families -1, strong branching off (VARSEL=-1 min infeas)", [](M & m) { param(m, 2028, -1); });
    run_gadgets("K traditional, families -1, BBINTERVAL 0 / NODESEL 0 (depth-first)", [](M & m) { param(m, P_NODESEL, 0); });
}
