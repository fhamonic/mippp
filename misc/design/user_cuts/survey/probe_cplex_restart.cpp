// Q9: does a cut added before a restart survive it, inside one solve?
#include "probe_cplex_common.cpp"
using cp::raw_cut;
struct S {
    const c::cplex_api * api; const syms * s; int cols; int purgeable, local; bool do_add;
    const cp::atsp * a; std::optional<raw_cut> cut; int err = 0; int K; int min_depth = 0; long at_nodecount = -1;
    std::vector<raw_cut> cuts;
    long calls = 0, c0 = 0, v0 = 0, c1 = 0, v1 = 0, first_after_d = -1; int maxr = 0; long root0 = 0, root1 = 0;
};
static int shook(ctxptr ctx, cpxlong id, void * usr) {
    auto & t = *static_cast<S *>(usr);
    if(id != c::CPX_CALLBACKCONTEXT_RELAXATION) return 0;
    std::vector<double> x(static_cast<std::size_t>(t.cols));
    t.s->point(ctx, x.data(), 0, t.cols - 1, nullptr);
    int restarts = 0; t.s->infoint(ctx, INFO_RESTARTS, &restarts); t.maxr = std::max(t.maxr, restarts);
    cpxlong depth = -1; t.s->infolong(ctx, INFO_NODEDEPTH, &depth);
    ++t.calls;
    (restarts ? t.root1 : t.root0) += depth == 0;
    if(t.cuts.empty()) {
        if(depth < t.min_depth) return 0;
        cpxlong nc = 0; t.s->infolong(ctx, INFO_NODECOUNT, &nc); t.at_nodecount = nc;
        // the first point deep enough: the K most violated subtour cuts
        t.cuts = t.a->separate(x, static_cast<std::size_t>(t.K));
        if(t.do_add) for(const auto & r : t.cuts) {
            const int beg = 0;
            t.err |= t.api->callbackaddusercuts(ctx, 1, static_cast<int>(r.ind.size()), &r.rhs, &r.sense, &beg, r.ind.data(), r.val.data(), &t.purgeable, &t.local);
        }
        return 0;
    }
    for(const auto & r : t.cuts) {
        const bool v = r.violation(x) > 1e-5;
        if(restarts) { ++t.c1; t.v1 += v; } else { ++t.c0; t.v0 += v; }
    }
    return 0;
}
static void run(const std::string & mode, bool add, int K, int repeat, double tl, int min_depth = 0) {
    M m; param(m, P_THREADS, 1); param(m, P_REPEATPRESOLVE, repeat);
    auto a = cp::build_atsp(m, 26);
    m.set_time_limit(std::chrono::duration<double>(tl));
    const auto [pg, lc] = mode_flags(mode);
    S t{&m.native_api(), &syms::get(m), static_cast<int>(m.num_variables()), pg, lc, add, &a};
    t.K = K; t.min_depth = min_depth;
    auto [env, lp] = m.native_model();
    m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, c::CPX_CALLBACKCONTEXT_RELAXATION, shook, &t));
    const double s = cp::timed_solve(m);
    int n = -1; syms::get(m).numcuts(env, lp, CUT_USER, &n);
    std::println(stderr, "== repeatpresolve={} {} K={} min_depth={} (added at nodecount {}): {}\n   calls={} restarts={} root calls before/after restart={}/{} err={} violations of the first cuts: before restart {}/{}, after restart {}/{}; CPXgetnumcuts(USER)={}", repeat, add ? "ADD " + mode : "no add (power baseline)", K, min_depth, t.at_nodecount, cp::summary(m, s), t.calls, t.maxr, t.root0, t.root1, t.err, t.v0, t.c0, t.v1, t.c1, n);
    std::fflush(stderr);
}
int main() {
    std::println(stderr, "library {}", M{}.native_api().library_path().string());
    if(std::getenv("RESTART_DEEP")) {
        for(int d : {3, 8, 12}) {
            run("", false, 1, -1, 20, d);
            for(const std::string mode : {"global-purge", "global-force", "local-purge", "local-force"}) run(mode, true, 1, -1, 20, d);
        }
        return 0;
    }
    for(int repeat : {-1, 1, 3}) for(int K : {1, 3}) {
        run("", false, K, repeat, 20);
        for(const std::string mode : {"global-purge", "global-filter", "global-force", "local-purge", "local-force"}) run(mode, true, K, repeat, 20);
    }
}
