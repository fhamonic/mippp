// Same node, stateless separator: every call runs the cover separator and adds what the point violates (recipe, sanitize on).
// Per node: calls, identical points, violated re-adds. Does a post-loop add reopen the loop at >=2 threads?
#include "verify_cplex_common.hpp"
using namespace cp;
using std::chrono::duration;
struct nd { int calls = 0, after = 0, added = 0, added_after = 0, same_as_prev = 0; std::vector<double> prev; };
struct sc {
    const vsyms * s; const cx::cplex_api * api; int cols, purgeable, local = 0; bool skip_post;
    std::vector<std::vector<double>> a; std::vector<double> b;
    std::mutex mx; std::unordered_map<cxlong, nd> nodes;
};
static int hk(ctxptr context, cxlong id, void * usr) {
    auto & h = *static_cast<sc *>(usr);
    if(id != CPX_CALLBACKCONTEXT_RELAXATION) return 0;
    try {
        std::vector<double> x(static_cast<std::size_t>(h.cols));
        if(h.s->point(context, x.data(), 0, h.cols - 1, nullptr)) return 0;
        cxlong uid = -1; int after = 0;
        h.s->infolong(context, CPXCALLBACKINFO_NODEUID, &uid);
        h.s->infoint(context, CPXCALLBACKINFO_AFTERCUTLOOP, &after);
        int added = 0;
        if(!(after && h.skip_post)) {
            std::vector<std::size_t> order(x.size()); for(std::size_t j = 0; j < order.size(); ++j) order[j] = j;
            std::ranges::sort(order, [&](std::size_t p, std::size_t r) { return x[p] > x[r]; });
            for(std::size_t i = 0; i < h.a.size(); ++i) {
                raw_cut cut{{}, {}, 'L', 0}; double w = 0;
                for(std::size_t j : order) { w += h.a[i][j]; cut.ind.push_back(static_cast<int>(j)); cut.val.push_back(1.0); if(w > h.b[i]) { cut.rhs = static_cast<double>(cut.ind.size()) - 1; break; } }
                if(cut.violation(x) <= 1e-3) continue;
                int beg = 0;
                if(!h.api->callbackaddusercuts(context, 1, static_cast<int>(cut.ind.size()), &cut.rhs, &cut.sense, &beg, cut.ind.data(), cut.val.data(), &h.purgeable, &h.local)) ++added;
            }
        }
        std::lock_guard l(h.mx);
        auto & n = h.nodes[uid];
        ++n.calls; n.after += after; n.added += added; if(after) n.added_after += added;
        if(n.prev == x) ++n.same_as_prev;
        n.prev = std::move(x);
    } catch(...) { return 1; }
    return 0;
}
int main() {
    { M m; print_version(m); }
    for(const auto & [rows, cols, seed] : std::initializer_list<std::tuple<int, int, unsigned>>{{30, 100, 7}, {5, 60, 3}}) for(const int th : {1, 2, 4}) for(const int par : {1, -1}) for(const int search : {0}) for(const int skip : {0, 1}) {
        if(th == 1 && par == -1) continue;
        M m; build_mkp(m, rows, cols, seed); set_threads(m, th); vparam(m, CPXPARAM_Parallel, par); vparam(m, CPX_PARAM_MIPSEARCH, search); m.set_time_limit(duration<double>(3));
        sc h; h.s = &vsyms::get(m); h.api = &m.native_api(); h.cols = static_cast<int>(m.num_variables()); h.purgeable = CPX_USECUT_PURGE; h.skip_post = skip;
        {
            auto [env, lp] = m.native_model();
            const int n = h.cols; const int nr = static_cast<int>(m.num_constraints());
            h.a.assign(static_cast<std::size_t>(nr), std::vector<double>(static_cast<std::size_t>(n), 0.0)); h.b.assign(static_cast<std::size_t>(nr), 0.0);
            std::vector<int> ind(static_cast<std::size_t>(n)); std::vector<double> val(static_cast<std::size_t>(n)); int nz, beg, surplus;
            for(int i = 0; i < nr; ++i) {
                m.native_api()._check(env, m.native_api().getrows(env, lp, &nz, &beg, ind.data(), val.data(), n, &surplus, i, i));
                m.native_api()._check(env, m.native_api().getrhs(env, lp, &h.b[static_cast<std::size_t>(i)], i, i));
                for(int k = 0; k < nz; ++k) h.a[static_cast<std::size_t>(i)][static_cast<std::size_t>(ind[static_cast<std::size_t>(k)])] = val[static_cast<std::size_t>(k)];
            }
            m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, CPX_CALLBACKCONTEXT_RELAXATION, hk, &h));
        }
        const double t = timed_solve(m);
        long calls = 0, same = 0, added = 0, added_after = 0, maxcalls = 0, after = 0, nodes_with_add = 0, only_post = 0, only_inloop = 0; std::map<int, long> dist;
        for(const auto & [u, n] : h.nodes) { calls += n.calls; same += n.same_as_prev; added += n.added; added_after += n.added_after; maxcalls = std::max<long>(maxcalls, n.calls); after += n.after; if(n.added) ++nodes_with_add; if(n.calls == n.after) ++only_post; if(n.after == 0) ++only_inloop; dist[std::min(n.calls, 9)]++; }
        std::string ds; for(auto [k, v] : dist) ds += std::format(" {}:{}", k, v);
        std::println(stderr, "SN2 mkp{}x{}s{} threads={} par={} search={} skip_post={}: {} | nodes_seen={} calls={} post_loop={} same_point_as_prev={} added={} added_in_post={} nodes_with_add={} max_calls_at_node={} nodes_only_post_loop={} nodes_without_post={} dist{}", rows, cols, seed, th, par, search, skip, summary(m, t), h.nodes.size(), calls, after, same, added, added_after, nodes_with_add, maxcalls, only_post, only_inloop, ds);
    }
}
