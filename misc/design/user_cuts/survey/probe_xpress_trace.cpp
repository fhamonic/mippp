// A firing-by-firing trace of the S1 instance, to see whether a managed "global" cut ever
// reaches a node outside the subtree it was added in. argv: k flip global(1/0) action(-100,-1,0,1,2) [maxlines]
#include "probe_xpress_hook.hpp"
struct tctx {
    const x::xpress_api * api; std::shared_ptr<xp_fns> fn; cp::gadgets<M> * g; int global, action, cols; cp::raw_cut cut;
    bool added = false; long lines = 0, maxlines = 200; bool poke = false; long poked = 0; int poke_global = -1; int rows0 = -1; long sib_after = 0, sib_viol = 0, sib_rows_up = 0, same_after = 0, same_viol = 0;
};
static void cutround(prob_t cb, void * d, int ifx, int * p_action) {
    auto & c = *static_cast<tctx *>(d);
    int node = 0, parent = 0, depth = 0, rounds = 0, cuts = 0, rows = 0;
    c.api->getintattrib(cb, A_CURRENTNODE, &node); c.api->getintattrib(cb, A_PARENTNODE, &parent);
    c.api->getintattrib(cb, A_NODEDEPTH, &depth); c.api->getintattrib(cb, A_CUTROUNDS, &rounds);
    c.api->getintattrib(cb, A_CUTS, &cuts); c.api->getintattrib(cb, A_ROWS, &rows);
    if(c.rows0 < 0) c.rows0 = rows;
    double obj = 0; c.api->getdblattrib(cb, D_LPOBJVAL, &obj);
    std::vector<double> xv(static_cast<std::size_t>(c.cols));
    c.api->getcallbacksolution(cb, nullptr, xv.data(), 0, c.cols - 1);
    const int st = c.g->state(xv);
    const double viol = c.cut.violation(xv);
    bool now = false;
    if(!c.added && st == c.g->worse_state() && viol > 1e-4) {
        const int start[2] = {0, static_cast<int>(c.cut.ind.size())};
        const int e = c.fn->addmanaged(cb, c.global, 1, &c.cut.sense, &c.cut.rhs, start, c.cut.ind.data(), c.cut.val.data());
        c.added = !e; now = true;
        if(e) std::println(stderr, "add error {}", e);
    } else if(c.added) {
        if(st == c.g->worse_state()) { ++c.same_after; c.same_viol += viol > 1e-6; }
        else if(st == 1 - c.g->worse_state()) { ++c.sib_after; c.sib_viol += viol > 1e-6; c.sib_rows_up += rows > c.rows0; }
    }
    // after the cut: at every firing outside its subtree add one valid hull cut this point violates,
    // so that a round is processed there. A hull cut is valid everywhere, so it cannot lose the optimum.
    bool poked_now = false;
    if(c.poke && c.added && st != c.g->worse_state())
        for(int i = 1; i < c.g->k; ++i) {
            const auto h = c.g->hull_cut(i);
            if(h.violation(xv) > 1e-4) {
                const int start[2] = {0, static_cast<int>(h.ind.size())};
                const int e = c.fn->addmanaged(cb, c.poke_global < 0 ? c.global : c.poke_global, 1, &h.sense, &h.rhs, start, h.ind.data(), h.val.data());
                if(!e) { ++c.poked; poked_now = true; }
                break;
            }
        }
    const int in = *p_action;
    if(c.action != -100 && rounds == 0) *p_action = c.action;
    if(now || poked_now) *p_action = 2;
    if(c.lines++ < c.maxlines)
        std::println(stderr, "node={} parent={} depth={} ifx={} rounds={} cuts={} rows={}(+{}) y0state={} lp={:.4f} viol={:.4f} action_in={} out={} {}", node, parent, depth, ifx, rounds, cuts, rows, rows - c.rows0, st, obj, viol, in, *p_action, now ? "<-- CUT ADDED" : "");
}
int main(int argc, char ** argv) {
    const int k = argc > 1 ? std::atoi(argv[1]) : 6;
    const bool flip = argc > 2 && std::atoi(argv[2]);
    tctx c; c.global = argc > 3 ? std::atoi(argv[3]) : 1; c.action = argc > 4 ? std::atoi(argv[4]) : -100; if(argc > 5) c.maxlines = std::atol(argv[5]);
    c.poke = argc > 6 && std::atoi(argv[6]) != 0;
    if(argc > 6 && std::atoi(argv[6]) == 2) c.poke_global = 1;  // 2: the poke is global, whatever the cut
    if(argc > 6 && std::atoi(argv[6]) == 3) c.poke_global = 0;  // 3: the poke is local, whatever the cut
    M m; scope_settings(m);
    auto g = cp::build_gadgets(m, k, flip);
    m.set_time_limit(std::chrono::duration<double>(20.0));
    c.api = &m.native_api(); c.fn = fns_of(m); c.g = &g; c.cut = g.invalid_global_cut(); c.cols = static_cast<int>(m.num_variables());
    m.native_api()._check(m.native_model(), c.fn->addcbcutround(m.native_model(), cutround, &c, 0));
    const double t = cp::timed_solve(m);
    std::println(stderr, "RESULT k={} flip={} global={} action={}: {} (known opt {}) firings={} after the cut: same-subtree viol/checks={}/{}, sibling viol/checks={}/{} sibling firings with rows>rows0={} poked={} => {}", k, flip, c.global, c.action, cp::summary(m, t), g.opt, c.lines, c.same_viol, c.same_after, c.sib_viol, c.sib_after, c.sib_rows_up, c.poked, std::fabs(m.get_solution_value() - g.opt) < 1e-4 ? "optimum KEPT" : "optimum LOST");
}
