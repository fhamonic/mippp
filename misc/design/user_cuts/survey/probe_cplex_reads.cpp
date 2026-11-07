// Q12: what the hook can read about its node, and a local cut written from those reads.
#include "probe_cplex_common.cpp"
using cp::raw_cut;
struct R {
    const c::cplex_api * api; const syms * s; int cols; int purgeable = USECUT_PURGE, local = 0;
    std::function<void(ctxptr, const std::vector<double> &, const std::vector<double> &, const std::vector<double> &, cpxlong, cpxlong, R &)> body;
    int add(ctxptr ctx, const raw_cut & r) {
        const int beg = 0;
        return api->callbackaddusercuts(ctx, 1, static_cast<int>(r.ind.size()), &r.rhs, &r.sense, &beg, r.ind.data(), r.val.data(), &purgeable, &local);
    }
    long errs = 0;
};
static int rhook(ctxptr ctx, cpxlong id, void * usr) {
    auto & r = *static_cast<R *>(usr);
    if(id != c::CPX_CALLBACKCONTEXT_RELAXATION) return 0;
    const auto N = static_cast<std::size_t>(r.cols);
    std::vector<double> x(N), lb(N), ub(N);
    cpxlong uid = -1, depth = -1;
    int e = r.s->point(ctx, x.data(), 0, r.cols - 1, nullptr);
    e |= r.s->locallb(ctx, lb.data(), 0, r.cols - 1);
    e |= r.s->localub(ctx, ub.data(), 0, r.cols - 1);
    e |= r.s->infolong(ctx, INFO_NODEUID, &uid);
    e |= r.s->infolong(ctx, INFO_NODEDEPTH, &depth);
    if(e) { r.errs++; return 0; }
    r.body(ctx, x, lb, ub, uid, depth, r);
    return 0;
}
int main() {
    std::println(stderr, "library {}", M{}.native_api().library_path().string());
    // A: the reads, on the scope instance
    {
        M m; scope_settings(m); param(m, P_MIRCUTS, 1); param(m, P_EACHCUTLIM, 0);
        auto g = cp::build_gadgets(m, 10, false);
        R r{&m.native_api(), &syms::get(m), static_cast<int>(m.num_variables())};
        long calls = 0, xout = 0, fixed_agree = 0, fixed_disagree = 0, y0_free = 0; int shown = 0;
        std::vector<double> glb(static_cast<std::size_t>(r.cols)), gub(static_cast<std::size_t>(r.cols));
        long g_err = 0, nodesleft_err = 0, inc_err = 0, bnd_err = 0;
        r.body = [&](ctxptr ctx, const std::vector<double> & x, const std::vector<double> & lb, const std::vector<double> & ub, cpxlong uid, cpxlong depth, R & rr) {
            ++calls;
            for(int j = 0; j < rr.cols; ++j) { const auto J = static_cast<std::size_t>(j); if(x[J] < lb[J] - 1e-6 || x[J] > ub[J] + 1e-6) ++xout; }
            const auto Y = static_cast<std::size_t>(g.y[0]);
            if(lb[Y] == ub[Y]) { (std::fabs(x[Y] - lb[Y]) < 1e-6 ? fixed_agree : fixed_disagree)++; } else ++y0_free;
            g_err += rr.s->globallb(ctx, glb.data(), 0, rr.cols - 1) != 0;
            g_err += rr.s->globalub(ctx, gub.data(), 0, rr.cols - 1) != 0;
            cpxlong left = 0; nodesleft_err += rr.s->infolong(ctx, INFO_NODESLEFT, &left) != 0;
            double bb = 0, bs = 0; bnd_err += rr.s->infodbl(ctx, INFO_BEST_BND, &bb) != 0; inc_err += rr.s->infodbl(ctx, INFO_BEST_SOL, &bs) != 0;
            if(shown++ < 8) std::println(stderr, "   call {}: uid={} depth={} x[y0]={:.4f} local[y0]=[{},{}] global[y0]=[{},{}] nodesleft={} best_bnd={:.5g} best_sol={:.5g}", calls, uid, depth, x[Y], lb[Y], ub[Y], glb[Y], gub[Y], left, bb, bs);
        };
        auto [env, lp] = m.native_model();
        m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, c::CPX_CALLBACKCONTEXT_RELAXATION, rhook, &r));
        m.set_time_limit(std::chrono::duration<double>(10));
        const double t = cp::timed_solve(m);
        std::println(stderr, "== A reads: {}\n   calls={} read errors={} x outside local bounds={} y0 fixed by local bounds and x agrees={} disagrees={} y0 free={} ; globallb/ub errors={} nodesleft errors={} best_bnd errors={} best_sol errors={}", cp::summary(m, t), calls, r.errs, xout, fixed_agree, fixed_disagree, y0_free, g_err, nodesleft_err, bnd_err, inc_err);
    }
    // B: local bounds drive the cut: S1 where the trigger comes from the bounds, not from x
    for(const std::string mode : {"global-purge", "local-purge", "local-force"}) for(bool flip : {false, true}) {
        M m; scope_settings(m);
        auto g = cp::build_gadgets(m, 10, flip);
        const auto [pg, lc] = mode_flags(mode);
        R r{&m.native_api(), &syms::get(m), static_cast<int>(m.num_variables()), pg, lc};
        bool added = false; int err = 0; const raw_cut cut = g.invalid_global_cut();
        r.body = [&](ctxptr ctx, const std::vector<double> & x, const std::vector<double> & lb, const std::vector<double> & ub, cpxlong, cpxlong, R & rr) {
            const auto Y = static_cast<std::size_t>(g.y[0]);
            const double w = g.worse_state();
            if(!added && lb[Y] == w && ub[Y] == w && cut.violation(x) > 1e-4) { added = true; err = rr.add(ctx, cut); }
        };
        auto [env, lp] = m.native_model();
        m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, c::CPX_CALLBACKCONTEXT_RELAXATION, rhook, &r));
        m.set_time_limit(std::chrono::duration<double>(10));
        const double t = cp::timed_solve(m);
        double v = NAN; try { v = m.get_solution_value(); } catch(const std::exception &) {}
        std::println(stderr, "== B trigger from local bounds, mode {} flip={}: {} (known opt {}) => {} add err={}", mode, flip, cp::summary(m, t), g.opt, std::fabs(v - g.opt) < 1e-4 ? "KEPT" : "LOST", err);
    }
    // C: a local cut at a deeper node: its descendants, its cousins, the far sibling
    for(const std::string mode : {"local-purge", "local-force", "local-filter", "global-purge"}) for(int want_depth : {2, 3}) {
        M m; scope_settings(m);
        auto g = cp::build_gadgets(m, 10, false);
        const auto [pg, lc] = mode_flags(mode);
        R r{&m.native_api(), &syms::get(m), static_cast<int>(m.num_variables()), pg, lc};
        const raw_cut cut = g.invalid_global_cut();
        bool added = false; int err = 0; cpxlong at_uid = -1, at_depth = -1;
        std::vector<double> L, U;
        long desc_c = 0, desc_v = 0, cous_c = 0, cous_v = 0, sib_c = 0, sib_v = 0;
        const auto ints = g.y;
        r.body = [&](ctxptr ctx, const std::vector<double> & x, const std::vector<double> & lb, const std::vector<double> & ub, cpxlong uid, cpxlong depth, R & rr) {
            const auto Y = static_cast<std::size_t>(g.y[0]);
            const double w = g.worse_state();
            const bool in_worse = lb[Y] == w && ub[Y] == w;
            if(!added) {
                if(in_worse && depth >= want_depth && cut.violation(x) > 1e-4) { added = true; L = lb; U = ub; at_uid = uid; at_depth = depth; err = rr.add(ctx, cut); }
                return;
            }
            bool desc = true;
            for(std::size_t j = 0; j < L.size(); ++j) if(lb[j] < L[j] - 1e-9 || ub[j] > U[j] + 1e-9) { desc = false; break; }
            const bool v = cut.violation(x) > 1e-6;
            if(desc) { desc_c++; desc_v += v; }
            else if(in_worse) { cous_c++; cous_v += v; }
            else { sib_c++; sib_v += v; }
        };
        auto [env, lp] = m.native_model();
        m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, c::CPX_CALLBACKCONTEXT_RELAXATION, rhook, &r));
        m.set_time_limit(std::chrono::duration<double>(10));
        const double t = cp::timed_solve(m);
        std::println(stderr, "== C cut at a depth>={} node, mode {}: {}\n   added at uid={} depth={} err={} viol/checks: descendants {}/{} cousins (same y0 subtree) {}/{} other y0 subtree {}/{}", want_depth, mode, cp::summary(m, t), at_uid, at_depth, err, desc_v, desc_c, cous_v, cous_c, sib_v, sib_c);
    }
    // C2: same, but a cut every integer point obeys, so that descendants are not pruned at once:
    // sum z <= opt + 1 at the node. Far more descendant checks.
    for(const std::string mode : {"local-purge", "local-force", "global-purge"}) for(int want_depth : {2, 3}) {
        M m; scope_settings(m);
        auto g = cp::build_gadgets(m, 10, false);
        const auto [pg, lc] = mode_flags(mode);
        R r{&m.native_api(), &syms::get(m), static_cast<int>(m.num_variables()), pg, lc};
        raw_cut cut = g.invalid_global_cut(); cut.rhs = g.opt + 1;
        bool added = false; int err = 0; cpxlong at_uid = -1, at_depth = -1;
        std::vector<double> L, U;
        long desc_c = 0, desc_v = 0, cous_c = 0, cous_v = 0, sib_c = 0, sib_v = 0;
        r.body = [&](ctxptr ctx, const std::vector<double> & x, const std::vector<double> & lb, const std::vector<double> & ub, cpxlong uid, cpxlong depth, R & rr) {
            const auto Y = static_cast<std::size_t>(g.y[0]);
            const double w = g.worse_state();
            const bool in_worse = lb[Y] == w && ub[Y] == w;
            if(!added) {
                if(in_worse && depth >= want_depth && cut.violation(x) > 1e-4) { added = true; L = lb; U = ub; at_uid = uid; at_depth = depth; err = rr.add(ctx, cut); }
                return;
            }
            bool desc = true;
            for(std::size_t j = 0; j < L.size(); ++j) if(lb[j] < L[j] - 1e-9 || ub[j] > U[j] + 1e-9) { desc = false; break; }
            const bool v = cut.violation(x) > 1e-6;
            if(desc) { desc_c++; desc_v += v; }
            else if(in_worse) { cous_c++; cous_v += v; }
            else { sib_c++; sib_v += v; }
        };
        auto [env, lp] = m.native_model();
        m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, c::CPX_CALLBACKCONTEXT_RELAXATION, rhook, &r));
        m.set_time_limit(std::chrono::duration<double>(10));
        const double t = cp::timed_solve(m);
        std::println(stderr, "== C2 sum z <= opt+1 at a depth>={} node, mode {}: {}\n   added at uid={} depth={} err={} viol/checks: descendants {}/{} cousins (same y0 subtree) {}/{} other y0 subtree {}/{}", want_depth, mode, cp::summary(m, t), at_uid, at_depth, err, desc_v, desc_c, cous_v, cous_c, sib_v, sib_c);
    }
    // D: bounds on presolved-away columns
    {
        M m; 
        for(int p : {P_CLIQUES, P_COVERS, P_DISJCUTS, P_FLOWCOVERS, P_FLOWPATHS, P_FRACCUTS, P_GUBCOVERS, P_IMPLBD, P_LANDP, P_MCF, P_ZEROHALF, P_LOCALIMPLBD, P_BQP, P_RLT}) param(m, p, -1);
        param(m, P_EACHCUTLIM, 0); param(m, P_HEURFREQ, -1); param(m, P_RINS, -1); param(m, P_FPHEUR, -1); param(m, P_LBHEUR, 0); param(m, P_THREADS, 1);
        auto a = cp::build_atsp(m, 22, true);
        R r{&m.native_api(), &syms::get(m), static_cast<int>(m.num_variables())};
        long calls = 0, xout = 0, comp_ok = 0, comp_bad = 0, fixed_x = 0, comp_fixed = 0;
        r.body = [&](ctxptr, const std::vector<double> & x, const std::vector<double> & lb, const std::vector<double> & ub, cpxlong, cpxlong, R & rr) {
            ++calls;
            for(int j = 0; j < rr.cols; ++j) { const auto J = static_cast<std::size_t>(j); if(x[J] < lb[J] - 1e-6 || x[J] > ub[J] + 1e-6) ++xout; }
            for(int i = 0; i < a.n; ++i) for(int j = 0; j < a.n; ++j) if(i != j) {
                const auto cx = static_cast<std::size_t>(a.col[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]), cc = static_cast<std::size_t>(a.comp[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]);
                const bool xf = lb[cx] == ub[cx], cf = lb[cc] == ub[cc];
                fixed_x += xf; comp_fixed += cf;
                (std::fabs(lb[cc] - (1 - ub[cx])) < 1e-9 && std::fabs(ub[cc] - (1 - lb[cx])) < 1e-9 ? comp_ok : comp_bad)++;
            }
        };
        auto [env, lp] = m.native_model();
        m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, c::CPX_CALLBACKCONTEXT_RELAXATION, rhook, &r));
        m.set_time_limit(std::chrono::duration<double>(20));
        const double t = cp::timed_solve(m);
        std::println(stderr, "== D atsp22 complements, default presolve, local bounds on eliminated columns: {}\n   calls={} read errors={} x outside local bounds={} complement bounds agree with 1-x bounds: {} disagree: {}; fixed x bounds seen={} fixed complement bounds seen={}", cp::summary(m, t), calls, r.errs, xout, comp_ok, comp_bad, fixed_x, comp_fixed);
    }
}
