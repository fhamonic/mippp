// C1: what local=0 and local=1 mean. Classifies by local bounds at each call, not by the
// point: descendant of the adding node, cousin inside the same y0 subtree, other y0 subtree.
#include "verify_cplex_common.hpp"
using namespace cp;
struct bucket { long checks = 0, viol = 0; };
struct sc {
    gadgets<M> * g; const vsyms * s; const cx::cplex_api * api; int cols; int purgeable, local;
    bool hull;   // hull cut of a free gadget (valid), otherwise the invalid opt-1 cut
    int want_depth;
    std::mutex mx;
    bool added = false; int add_err = 0; raw_cut cut; int gadget = -1;
    std::vector<double> box_lb, box_ub;
    long calls = 0; cxlong add_depth = -1;
    bucket desc, cous, other, adding;
    std::map<int,long> depth_hist;
    cxlong add_uid = -1;
};
static std::vector<std::unique_ptr<sc>> keep;
static int hook(ctxptr context, cxlong id, void * usr) {
    auto & h = *static_cast<sc *>(usr);
    if(id != CPX_CALLBACKCONTEXT_RELAXATION) return 0;
    try {
        const auto N = static_cast<std::size_t>(h.cols);
        std::vector<double> x(N), lb(N), ub(N);
        if(h.s->point(context, x.data(), 0, h.cols - 1, nullptr)) return 0;
        h.s->locallb(context, lb.data(), 0, h.cols - 1);
        h.s->localub(context, ub.data(), 0, h.cols - 1);
        cxlong depth = -1, uid = -1;
        h.s->infolong(context, CPXCALLBACKINFO_NODEDEPTH, &depth);
        h.s->infolong(context, CPXCALLBACKINFO_NODEUID, &uid);
        std::lock_guard l(h.mx);
        ++h.calls; h.depth_hist[static_cast<int>(depth)]++;
        auto & g = *h.g;
        const auto Y0 = static_cast<std::size_t>(g.y[0]);
        const int y0state = lb[Y0] > 0.5 ? 1 : ub[Y0] < 0.5 ? 0 : -1;
        if(!h.added) {
            if(y0state != g.worse_state() || depth < h.want_depth) return 0;
            if(h.hull) {
                for(int i = 1; i < g.k; ++i) {
                    const auto Yi = static_cast<std::size_t>(g.y[static_cast<std::size_t>(i)]);
                    if(lb[Yi] < 0.5 && ub[Yi] > 0.5 && g.hull_cut(i).violation(x) > 1e-4) { h.gadget = i; h.cut = g.hull_cut(i); break; }
                }
                if(h.gadget < 0) return 0;
            } else {
                h.cut = g.invalid_global_cut();
                if(h.cut.violation(x) <= 1e-4) return 0;
            }
            const int beg = 0;
            h.add_err = h.api->callbackaddusercuts(context, 1, static_cast<int>(h.cut.ind.size()), &h.cut.rhs, &h.cut.sense, &beg, h.cut.ind.data(), h.cut.val.data(), &h.purgeable, &h.local);
            h.added = true; h.box_lb = lb; h.box_ub = ub; h.add_depth = depth; h.add_uid = uid;
            return 0;
        }
        if(h.hull) {
            const auto Yi = static_cast<std::size_t>(g.y[static_cast<std::size_t>(h.gadget)]);
            if(!(lb[Yi] < 0.5 && ub[Yi] > 0.5)) return 0;
        }
        const bool v = h.cut.violation(x) > 1e-6;
        bool inside = true;
        for(std::size_t J = 0; J < N; ++J) if(lb[J] < h.box_lb[J] - 1e-6 || ub[J] > h.box_ub[J] + 1e-6) { inside = false; break; }
        auto & b = uid == h.add_uid ? h.adding : inside ? h.desc : y0state == g.worse_state() ? h.cous : h.other;
        ++b.checks; b.viol += v;
    } catch(...) { return 1; }
    return 0;
}
int main() {
    { M m; print_version(m); }
    using std::chrono::duration;
    struct cfg { const char * name; int search; int threads; bool scope; int parallel; };
    const cfg cfgs[] = {{"traditional", 1, 1, true, 0}, {"auto", 0, 1, true, 0}, {"dynamic", 2, 1, true, 0}, {"presolve-on-REDUCE=0-nocuts-noheur", -2, 1, false, 0}, {"traditional-4threads-det", 1, 4, true, 1}};
    const char * only = std::getenv("ONLY_CFG");
    for(const auto & f : cfgs) {
        if(only && f.name != std::string(only)) continue;
        for(const auto & [mode, purge, local] : std::initializer_list<std::tuple<const char *, int, int>>{
                {"global-purge", CPX_USECUT_PURGE, 0}, {"global-force", CPX_USECUT_FORCE, 0}, {"global-filter", CPX_USECUT_FILTER, 0},
                {"local-purge", CPX_USECUT_PURGE, 1}, {"local-force", CPX_USECUT_FORCE, 1}, {"local-filter", CPX_USECUT_FILTER, 1}}) {
            for(const bool flip : {false, true}) {
                for(const bool hull : {false, true}) {
                    M m;
                    if(f.scope) scope_settings(m, f.search); else {
                        vparam(m, CPX_PARAM_REDUCE, 0); all_families_off(m);
                        vparam(m, CPX_PARAM_HEURFREQ, -1); vparam(m, CPX_PARAM_RINSHEUR, -1); vparam(m, CPX_PARAM_FPHEUR, -1); vparam(m, CPX_PARAM_LBHEUR, 0);
                        vparamd(m, CPXPARAM_MIP_Strategy_HeuristicEffort, 0.0);
                    }
                    set_threads(m, f.threads);
                    if(f.parallel) vparam(m, CPXPARAM_Parallel, f.parallel);
                    auto g = build_gadgets(m, 10, flip);
                    m.set_time_limit(duration<double>(30));
                    keep.push_back(std::make_unique<sc>());
                    auto & h = *keep.back();
                    h.g = &g; h.s = &vsyms::get(m); h.api = &m.native_api(); h.cols = static_cast<int>(m.num_variables()); h.purgeable = purge; h.local = local; h.hull = hull;
                    h.want_depth = hull ? 2 : 1;
                    auto [env, lp] = m.native_model();
                    m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, CPX_CALLBACKCONTEXT_RELAXATION, hook, &h));
                    const double t = timed_solve(m);
                    double value = cf::value_of(m);
                    if(!hull) {
                        std::println(stderr, "S1 cfg={} {} flip={}: {} known_opt={} added={} err={} depth={} calls={} => {}", f.name, mode, flip, summary(m, t), g.opt, h.added, h.add_err, h.add_depth, h.calls,
                            !h.added ? "NO POWER" : std::fabs(value - g.opt) < 1e-4 ? "KEPT" : "LOST");
                    } else {
                        std::println(stderr, "S2 cfg={} {} flip={}: {} added={} err={} depth={} gadget={} calls={} | viol/checks adding-node {}/{} descendants {}/{} cousins {}/{} other-y0 {}/{}", f.name, mode, flip, summary(m, t), h.added, h.add_err, h.add_depth, h.gadget, h.calls,
                            h.adding.viol, h.adding.checks, h.desc.viol, h.desc.checks, h.cous.viol, h.cous.checks, h.other.viol, h.other.checks);
                    }
                }
            }
        }
    }
}
