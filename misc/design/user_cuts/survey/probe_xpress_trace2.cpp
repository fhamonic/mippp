// Which cuts is a node really carrying? Every DFJ cut is logged with the node it came from, and at
// each firing the rows of the node are compared with the cuts of its ancestors. A managed global
// cut loaded outside its subtree shows as rows beyond what the ancestors explain, and as a foreign
// cut with slack 0 (tight) while an unloaded one can be violated. argv: n D settings global maxlines
#include "probe_xpress_hook.hpp"
struct rec { cp::raw_cut cut; int origin; };
struct tctx {
    const x::xpress_api * api; std::shared_ptr<xp_fns> fn; cp::atsp * a; int global, D, cols; long maxlines, lines = 0;
    int rows0 = -1; std::vector<rec> log; std::map<int, int> parent; long foreign_viol = 0, foreign_ok = 0, firings_with_foreign_rows = 0, firings = 0, cuts = 0;
};
static void cutround(prob_t cb, void * d, int ifx, int * p_action) {
    auto & c = *static_cast<tctx *>(d);
    int depth = 0, node = 0, par = 0, rows = 0, lp = 0, rounds = 0;
    c.api->getintattrib(cb, A_NODEDEPTH, &depth); c.api->getintattrib(cb, A_CURRENTNODE, &node); c.api->getintattrib(cb, A_PARENTNODE, &par);
    c.api->getintattrib(cb, A_ROWS, &rows); c.api->getintattrib(cb, A_LPSTATUS, &lp); c.api->getintattrib(cb, A_CUTROUNDS, &rounds);
    if(c.rows0 < 0) c.rows0 = rows;
    c.parent[node] = par; ++c.firings;
    std::set<int> anc;
    for(int v = node; v > 0; ) { anc.insert(v); auto it = c.parent.find(v); if(it == c.parent.end()) break; v = it->second; }
    std::vector<double> xv(static_cast<std::size_t>(c.cols));
    c.api->getcallbacksolution(cb, nullptr, xv.data(), 0, c.cols - 1);
    long inherited = 0, fviol = 0, fok = 0, ftight = 0;
    for(const auto & r : c.log) {
        if(anc.contains(r.origin)) { ++inherited; continue; }
        const double v = r.cut.violation(xv);
        if(v > 1e-6) ++fviol; else ++fok;
        if(std::fabs(v) < 1e-6) ++ftight;
    }
    c.foreign_viol += fviol; c.foreign_ok += fok;
    const long extra = rows - c.rows0 - inherited;
    if(extra > 0) ++c.firings_with_foreign_rows;
    long added = 0;
    if(depth >= c.D && lp == LP_OPTIMAL)
        for(const auto & r : c.a->separate(xv)) {
            const int start[2] = {0, static_cast<int>(r.ind.size())};
            if(!c.fn->addmanaged(cb, c.global, 1, &r.sense, &r.rhs, start, r.ind.data(), r.val.data())) { ++added; ++c.cuts; c.log.push_back({r, node}); }
        }
    if(added) *p_action = 2;
    if(c.lines++ < c.maxlines)
        std::println(stderr, "node={} parent={} depth={} ifx={} round={} rows=+{} ancestors'cuts={} extra_rows={} foreign: viol={} ok={} tight={} added={}", node, par, depth, ifx, rounds, rows - c.rows0, inherited, extra, fviol, fok, ftight, added);
}
int main(int argc, char ** argv) {
    const int n = argc > 1 ? std::atoi(argv[1]) : 26;
    const int D = argc > 2 ? std::atoi(argv[2]) : 2;
    const std::string st = argc > 3 ? argv[3] : "nocuts";
    const int global = argc > 4 ? std::atoi(argv[4]) : 1;
    const long ml = argc > 5 ? std::atol(argv[5]) : 60;
    const bool shuffle = argc > 6;
    M m; one_thread(m);
    auto a = cp::build_atsp(m, n, false, shuffle ? static_cast<unsigned>(std::atoi(argv[6])) : 5u);
    m.set_time_limit(std::chrono::duration<double>(60.0));
    if(st == "nocuts" || st == "nocuts_nopresolve") { set_int(m, C_COVERCUTS, 0); set_int(m, C_GOMCUTS, 0); set_int(m, C_TREECOVERCUTS, 0); set_int(m, C_TREEGOMCUTS, 0); }
    if(st == "nocuts_nopresolve") { set_int(m, C_PRESOLVE, 0); set_int(m, C_MIPPRESOLVE, 0); }
    if(st == "scope") scope_settings(m); else apply_xp_set(m);
    tctx c; c.api = &m.native_api(); c.fn = fns_of(m); c.a = &a; c.global = global; c.D = D; c.cols = static_cast<int>(m.num_variables()); c.maxlines = ml;
    m.native_api()._check(m.native_model(), c.fn->addcbcutround(m.native_model(), cutround, &c, 0));
    const double t = cp::timed_solve(m);
    std::println(stderr, "RESULT n={} D={} settings={} globalvalid={}: {} firings={} cuts={} firings_with_rows_beyond_ancestors={} foreign-cut checks viol={} ok={}", n, D, st, global, cp::summary(m, t), c.firings, c.cuts, c.firings_with_foreign_rows, c.foreign_viol, c.foreign_ok);
}
