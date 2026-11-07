// Power test for globalvalid. DFJ cuts are separated only at depth >= D, so that a cut is never
// added at the root: if globalvalid mattered, a cut found in one subtree would serve the others,
// and the node and cut counts would differ between 1 and 0. One thread, so the path is deterministic.
// argv: n D settings(default|cutfreq1|treecuts_only)
#include "probe_xpress_hook.hpp"
struct gctx {
    const x::xpress_api * api; std::shared_ptr<xp_fns> fn; cp::atsp * a; int global, D, cols;
    long fired = 0, cuts = 0, skipped_depth = 0, errs = 0; std::set<int> nodes; int maxrows = 0, rows0 = -1;
};
static void cutround(prob_t cb, void * d, int, int * p_action) {
    auto & c = *static_cast<gctx *>(d);
    int depth = 0, node = 0, rows = 0, lp = 0;
    c.api->getintattrib(cb, A_NODEDEPTH, &depth); c.api->getintattrib(cb, A_CURRENTNODE, &node); c.api->getintattrib(cb, A_ROWS, &rows); c.api->getintattrib(cb, A_LPSTATUS, &lp);
    if(c.rows0 < 0) c.rows0 = rows;
    c.maxrows = std::max(c.maxrows, rows);
    ++c.fired; c.nodes.insert(node);
    if(depth < c.D) { ++c.skipped_depth; return; }
    if(lp != LP_OPTIMAL) return;
    std::vector<double> xv(static_cast<std::size_t>(c.cols));
    c.api->getcallbacksolution(cb, nullptr, xv.data(), 0, c.cols - 1);
    bool added = false;
    for(const auto & r : c.a->separate(xv)) {
        const int start[2] = {0, static_cast<int>(r.ind.size())};
        const int e = c.fn->addmanaged(cb, c.global, 1, &r.sense, &r.rhs, start, r.ind.data(), r.val.data());
        if(e) ++c.errs; else { ++c.cuts; added = true; }
    }
    if(added) *p_action = 2;
}
int main(int argc, char ** argv) {
    const int n = argc > 1 ? std::atoi(argv[1]) : 26;
    const int D = argc > 2 ? std::atoi(argv[2]) : 2;
    const std::string st = argc > 3 ? argv[3] : "default";
    for(int global : {1, 0}) {
        M m; one_thread(m);
        auto a = cp::build_atsp(m, n);
        m.set_time_limit(std::chrono::duration<double>(60.0));
        if(st == "cutfreq1") set_int(m, C_CUTFREQ, 1);
        if(st == "treecuts_only") { set_int(m, C_COVERCUTS, 0); set_int(m, C_GOMCUTS, 0); set_int(m, C_CUTFREQ, 1); }
        if(st == "nocuts") { set_int(m, C_COVERCUTS, 0); set_int(m, C_GOMCUTS, 0); set_int(m, C_TREECOVERCUTS, 0); set_int(m, C_TREEGOMCUTS, 0); }
        gctx c; c.api = &m.native_api(); c.fn = fns_of(m); c.a = &a; c.global = global; c.D = D; c.cols = static_cast<int>(m.num_variables());
        m.native_api()._check(m.native_model(), c.fn->addcbcutround(m.native_model(), cutround, &c, 0));
        const double t = cp::timed_solve(m);
        std::println(stderr, "n={} D={} settings={} globalvalid={}: {} fired={} distinct_nodes={} cuts={} errs={} skipped(depth<D)={} max_rows_seen={} (first {})", n, D, st, global, cp::summary(m, t), c.fired, c.nodes.size(), c.cuts, c.errs, c.skipped_depth, c.maxrows, c.rows0);
    }
}
