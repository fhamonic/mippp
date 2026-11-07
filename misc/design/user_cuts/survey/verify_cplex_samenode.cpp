// With more than one thread, does a cut added in the tree act on the same node? Per node: first add, then the next call there.
#include "verify_cplex_common.hpp"
using namespace cp;
using std::chrono::duration;
struct st { int calls = 0; bool added = false; raw_cut cut; int next_seen = 0; };
struct sc {
    const vsyms * s; const cx::cplex_api * api; int cols, purgeable, local = 0;
    std::vector<std::vector<double>> a; std::vector<double> b;
    std::mutex mx; std::unordered_map<cxlong, st> nodes;
    long added = 0, next_calls = 0, next_viol = 0, next_after = 0, next_viol_after = 0, next_viol_inloop = 0, next_ok_after = 0, next_ok_inloop = 0, nodes_no_next = 0;
    long root_added = 0, root_next = 0, root_next_viol = 0, tree_next = 0, tree_next_viol = 0;
    int max_cuts = 1000000;
};
static int hk(ctxptr context, cxlong id, void * usr) {
    auto & h = *static_cast<sc *>(usr);
    if(id != CPX_CALLBACKCONTEXT_RELAXATION) return 0;
    try {
        std::vector<double> x(static_cast<std::size_t>(h.cols));
        if(h.s->point(context, x.data(), 0, h.cols - 1, nullptr)) return 0;
        cxlong uid = -1, depth = -1; int after = 0;
        h.s->infolong(context, CPXCALLBACKINFO_NODEUID, &uid);
        h.s->infolong(context, CPXCALLBACKINFO_NODEDEPTH, &depth);
        h.s->infoint(context, CPXCALLBACKINFO_AFTERCUTLOOP, &after);
        std::unique_lock l(h.mx);
        auto & n = h.nodes[uid];
        ++n.calls;
        if(n.added && n.next_seen == 0) {
            ++n.next_seen;
            const bool v = n.cut.violation(x) > 1e-6;
            ++h.next_calls; h.next_viol += v; h.next_after += after;
            if(v) (after ? h.next_viol_after : h.next_viol_inloop)++; else (after ? h.next_ok_after : h.next_ok_inloop)++;
            if(depth == 0) { ++h.root_next; h.root_next_viol += v; } else { ++h.tree_next; h.tree_next_viol += v; }
        }
        if(n.added) return 0;
        l.unlock();
        std::vector<std::size_t> order(x.size()); for(std::size_t j = 0; j < order.size(); ++j) order[j] = j;
        std::ranges::sort(order, [&](std::size_t p, std::size_t r) { return x[p] > x[r]; });
        for(std::size_t i = 0; i < h.a.size(); ++i) {
            raw_cut cut{{}, {}, 'L', 0}; double w = 0;
            for(std::size_t j : order) { w += h.a[i][j]; cut.ind.push_back(static_cast<int>(j)); cut.val.push_back(1.0); if(w > h.b[i]) { cut.rhs = static_cast<double>(cut.ind.size()) - 1; break; } }
            if(cut.violation(x) <= 1e-3) continue;
            int beg = 0;
            const int e = h.api->callbackaddusercuts(context, 1, static_cast<int>(cut.ind.size()), &cut.rhs, &cut.sense, &beg, cut.ind.data(), cut.val.data(), &h.purgeable, &h.local);
            if(!e) { l.lock(); auto & nn = h.nodes[uid]; nn.added = true; nn.cut = cut; ++h.added; if(depth == 0) ++h.root_added; l.unlock(); break; }
        }
    } catch(...) { return 1; }
    return 0;
}
int main() {
    { M m; print_version(m); }
    for(const auto & [rows, cols, seed] : std::initializer_list<std::tuple<int, int, unsigned>>{{30, 100, 7}, {10, 200, 3}}) for(const int th : {1, 2, 4, 8, 16}) for(const int par : {1, -1}) for(const auto & [mn, mp] : std::initializer_list<std::pair<const char *, int>>{{"PURGE", CPX_USECUT_PURGE}, {"FORCE", CPX_USECUT_FORCE}}) {
        if(th == 1 && par == -1) continue;
        M m; build_mkp(m, rows, cols, seed); set_threads(m, th); vparam(m, CPXPARAM_Parallel, par); m.set_time_limit(duration<double>(4));
        sc h; h.s = &vsyms::get(m); h.api = &m.native_api(); h.cols = static_cast<int>(m.num_variables()); h.purgeable = mp;
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
        std::println(stderr, "SN mkp{}x{}s{} threads={} par={} {}: {} | added={} next-call-at-node viol/calls root {}/{} tree {}/{} | next call flagged post-loop: viol {} ok {} ; in-loop: viol {} ok {}", rows, cols, seed, th, par, mn, summary(m, t), h.added, h.root_next_viol, h.root_next, h.tree_next_viol, h.tree_next, h.next_viol_after, h.next_ok_after, h.next_viol_inloop, h.next_ok_inloop);
    }
}
