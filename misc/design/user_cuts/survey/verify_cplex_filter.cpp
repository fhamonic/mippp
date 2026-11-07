// C10: FILTER at the root and in the tree; RepeatPresolve and FORCE.
#include "verify_cplex_common.hpp"
using namespace cp;
using std::chrono::duration;
struct wctx {
    const vsyms * s; const cx::cplex_api * api; int cols, purgeable, local = 0;
    std::function<std::optional<raw_cut>(const std::vector<double> &, cxlong depth)> maker;
    bool add = true;
    std::optional<raw_cut> cut; std::mutex mx;
    int err = 0; cxlong add_depth = -1;
    long root_checks = 0, root_viol = 0, tree_checks = 0, tree_viol = 0, r0_checks = 0, r0_viol = 0, r1_checks = 0, r1_viol = 0;
    int maxrestarts = 0; double viol_at_add = 0;
};
static int whook(ctxptr context, cxlong id, void * usr) {
    auto & h = *static_cast<wctx *>(usr);
    if(id != CPX_CALLBACKCONTEXT_RELAXATION) return 0;
    try {
        std::vector<double> x(static_cast<std::size_t>(h.cols));
        if(h.s->point(context, x.data(), 0, h.cols - 1, nullptr)) return 0;
        cxlong d = 0; int rs = 0;
        h.s->infolong(context, CPXCALLBACKINFO_NODEDEPTH, &d);
        h.s->infoint(context, CPXCALLBACKINFO_RESTARTS, &rs);
        std::lock_guard l(h.mx);
        h.maxrestarts = std::max(h.maxrestarts, rs);
        if(!h.cut) {
            auto c = h.maker(x, d);
            if(!c) return 0;
            h.cut = std::move(c); h.add_depth = d; h.viol_at_add = h.cut->violation(x);
            if(h.add) {
                int beg = 0;
                h.err = h.api->callbackaddusercuts(context, 1, static_cast<int>(h.cut->ind.size()), &h.cut->rhs, &h.cut->sense, &beg, h.cut->ind.data(), h.cut->val.data(), &h.purgeable, &h.local);
            }
            return 0;
        }
        const bool v = h.cut->violation(x) > 1e-6;
        if(d == 0) { ++h.root_checks; h.root_viol += v; } else { ++h.tree_checks; h.tree_viol += v; }
        if(rs == 0) { ++h.r0_checks; h.r0_viol += v; } else { ++h.r1_checks; h.r1_viol += v; }
    } catch(...) { return 1; }
    return 0;
}
static std::vector<std::unique_ptr<wctx>> keep;
static wctx & attach(M & m, const int purgeable, std::function<std::optional<raw_cut>(const std::vector<double> &, cxlong)> maker, const bool add = true) {
    keep.push_back(std::make_unique<wctx>()); auto & h = *keep.back();
    h.s = &vsyms::get(m); h.api = &m.native_api(); h.cols = static_cast<int>(m.num_variables()); h.purgeable = purgeable; h.maker = std::move(maker); h.add = add;
    auto [env, lp] = m.native_model();
    m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, CPX_CALLBACKCONTEXT_RELAXATION, whook, &h));
    return h;
}
// mkp with integer objective, so that Sum c x <= floor(root LP value) is valid.
static std::vector<double> objective_of(M & m) {
    auto [env, lp] = m.native_model();
    const int n = static_cast<int>(m.num_variables());
    std::vector<double> c(static_cast<std::size_t>(n));
    m.native_api()._check(env, m.native_api().getobj(env, lp, c.data(), 0, n - 1));
    return c;
}
int main(int, char ** argv) {
    { M m; print_version(m); }
    const std::string what = argv[1] ? argv[1] : "";
    const std::pair<const char *, int> modes[] = {{"FILTER", CPX_USECUT_FILTER}, {"PURGE", CPX_USECUT_PURGE}, {"FORCE", CPX_USECUT_FORCE}};
    if(what == "filter") {
        // A: dense objective floor at the root. B: same family in the tree. C: sparse cut at the root.
        for(const auto & [inst, rows, cols, seed] : std::initializer_list<std::tuple<const char *, int, int, unsigned>>{{"mkp30x100s7", 30, 100, 7}, {"mkp30x100s8", 30, 100, 8}, {"mkp5x100s3", 5, 100, 3}, {"mkp10x200s3", 10, 200, 3}}) {
            for(const auto & [mn, mp] : modes) {
                M m; m.set_thread_limit(1); build_mkp(m, rows, cols, seed); m.set_time_limit(duration<double>(3));
                const auto c = objective_of(m);
                auto & h = attach(m, mp, [c](const std::vector<double> & x, cxlong) -> std::optional<raw_cut> {
                    raw_cut cut{{}, {}, 'L', 0}; double act = 0;
                    for(std::size_t j = 0; j < c.size(); ++j) if(c[j] != 0) { cut.ind.push_back(static_cast<int>(j)); cut.val.push_back(c[j]); act += c[j] * x[j]; }
                    cut.rhs = std::floor(act + 1e-9);
                    if(act - cut.rhs < 1e-4) return std::nullopt;
                    return cut;
                });
                const double t = timed_solve(m);
                std::println(stderr, "C10 root floor {} {}: {} | added at depth {} err={} viol_at_add={:.3f} nz={} later root viol/checks {}/{} tree {}/{} numcuts_user={}", inst, mn, summary(m, t), h.add_depth, h.err, h.viol_at_add, h.cut ? h.cut->ind.size() : 0, h.root_viol, h.root_checks, h.tree_viol, h.tree_checks, vnumcuts(m));
            }
            // tree: the root LP floor, added at the first point in the tree that violates it
            for(const auto & [mn, mp] : modes) {
                M m; m.set_thread_limit(1); build_mkp(m, rows, cols, seed); m.set_time_limit(duration<double>(3)); all_families_off(m);
                vparam(m, CPX_PARAM_NODESEL, 1);
                const auto c = objective_of(m);
                // the root LP value is known only to a separate LP: solve one first
                double floorv;
                { M r; r.set_thread_limit(1); build_mkp(r, rows, cols, seed); all_families_off(r); r.set_time_limit(duration<double>(1)); timed_solve(r); floorv = r.get_dual_bound(); }
                (void)floorv;
                auto first = std::make_shared<std::optional<double>>();
                auto & h = attach(m, mp, [c, first](const std::vector<double> & x, cxlong d) -> std::optional<raw_cut> {
                    raw_cut cut{{}, {}, 'L', 0}; double act = 0;
                    for(std::size_t j = 0; j < c.size(); ++j) if(c[j] != 0) { cut.ind.push_back(static_cast<int>(j)); cut.val.push_back(c[j]); act += c[j] * x[j]; }
                    if(!*first) { *first = act; }   // first call: depth 1, the highest value met so far is a valid-ish probe
                    if(d < 1) return std::nullopt;
                    return std::nullopt;
                }, false);
                (void)h;
            }
            // sparse cut at the root under FILTER: a two-term clique-free cut x_i + x_j <= 1 is not valid, use a cover of row 0
            for(const auto & [mn, mp] : modes) {
                M m; m.set_thread_limit(1);
                std::mt19937 rng(seed);
                (void)rng;
                build_mkp(m, rows, cols, seed); m.set_time_limit(duration<double>(3));
                // row 0 read back natively: coefficients a, rhs b
                auto [env, lp] = m.native_model();
                const int n = static_cast<int>(m.num_variables());
                std::vector<int> ind(static_cast<std::size_t>(n)); std::vector<double> val(static_cast<std::size_t>(n)); int nz, beg, surplus; double b;
                m.native_api()._check(env, m.native_api().getrows(env, lp, &nz, &beg, ind.data(), val.data(), n, &surplus, 0, 0));
                m.native_api()._check(env, m.native_api().getrhs(env, lp, &b, 0, 0));
                std::vector<double> a(static_cast<std::size_t>(n), 0.0);
                for(int k = 0; k < nz; ++k) a[static_cast<std::size_t>(ind[static_cast<std::size_t>(k)])] = val[static_cast<std::size_t>(k)];
                auto & h = attach(m, mp, [a, b](const std::vector<double> & x, cxlong) -> std::optional<raw_cut> {
                    std::vector<std::size_t> order(a.size()); for(std::size_t j = 0; j < order.size(); ++j) order[j] = j;
                    std::ranges::sort(order, [&](std::size_t l, std::size_t r) { return x[l] > x[r]; });
                    raw_cut cut{{}, {}, 'L', 0}; double w = 0;
                    for(std::size_t j : order) { w += a[j]; cut.ind.push_back(static_cast<int>(j)); cut.val.push_back(1.0); if(w > b) { cut.rhs = static_cast<double>(cut.ind.size()) - 1; break; } }
                    if(cut.violation(x) <= 1e-4) return std::nullopt;
                    return cut;
                });
                const double t = timed_solve(m);
                std::println(stderr, "C10 first violated cover {} {}: {} | added at depth {} err={} viol_at_add={:.3f} nz={} later root viol/checks {}/{} tree {}/{} numcuts_user={}", inst, mn, summary(m, t), h.add_depth, h.err, h.viol_at_add, h.cut ? h.cut->ind.size() : 0, h.root_viol, h.root_checks, h.tree_viol, h.tree_checks, vnumcuts(m));
            }
        }
    }
    if(what == "filter2") {
        // No solver cut can move the root LP: one family on, EACHCUTLIM=0. A later root point then differs only if the user cut is in.
        for(const auto & [inst, rows, cols, seed] : std::initializer_list<std::tuple<const char *, int, int, unsigned>>{{"mkp30x100s7", 30, 100, 7}, {"mkp30x100s8", 30, 100, 8}, {"mkp5x100s3", 5, 100, 3}, {"mkp10x200s3", 10, 200, 3}, {"mkp5x60s3", 5, 60, 3}}) {
            for(const int kind : {0, 1}) for(const auto & [mn, mp, add] : std::initializer_list<std::tuple<const char *, int, bool>>{{"NOT-ADDED", CPX_USECUT_PURGE, false}, {"FILTER", CPX_USECUT_FILTER, true}, {"PURGE", CPX_USECUT_PURGE, true}, {"FORCE", CPX_USECUT_FORCE, true}}) {
                M m; m.set_thread_limit(1); build_mkp(m, rows, cols, seed); m.set_time_limit(duration<double>(3));
                all_families_off(m); vparam(m, CPX_PARAM_MIRCUTS, 2); vparam(m, CPX_PARAM_EACHCUTLIM, 0);
                const auto c = objective_of(m);
                std::vector<double> a(c.size(), 0.0); double b = 0;
                {
                    auto [env, lp] = m.native_model();
                    const int n = static_cast<int>(m.num_variables());
                    std::vector<int> ind(static_cast<std::size_t>(n)); std::vector<double> val(static_cast<std::size_t>(n)); int nz, beg, surplus;
                    m.native_api()._check(env, m.native_api().getrows(env, lp, &nz, &beg, ind.data(), val.data(), n, &surplus, 0, 0));
                    m.native_api()._check(env, m.native_api().getrhs(env, lp, &b, 0, 0));
                    for(int k = 0; k < nz; ++k) a[static_cast<std::size_t>(ind[static_cast<std::size_t>(k)])] = val[static_cast<std::size_t>(k)];
                }
                auto & h = attach(m, mp, [c, a, b, kind](const std::vector<double> & x, cxlong) -> std::optional<raw_cut> {
                    if(kind == 0) {
                        raw_cut cut{{}, {}, 'L', 0}; double act = 0;
                        for(std::size_t j = 0; j < c.size(); ++j) if(c[j] != 0) { cut.ind.push_back(static_cast<int>(j)); cut.val.push_back(c[j]); act += c[j] * x[j]; }
                        cut.rhs = std::floor(act + 1e-9);
                        if(act - cut.rhs < 1e-4) return std::nullopt;
                        return cut;
                    }
                    std::vector<std::size_t> order(a.size()); for(std::size_t j = 0; j < order.size(); ++j) order[j] = j;
                    std::ranges::sort(order, [&](std::size_t l, std::size_t r) { return x[l] > x[r]; });
                    raw_cut cut{{}, {}, 'L', 0}; double w = 0;
                    for(std::size_t j : order) { w += a[j]; cut.ind.push_back(static_cast<int>(j)); cut.val.push_back(1.0); if(w > b) { cut.rhs = static_cast<double>(cut.ind.size()) - 1; break; } }
                    if(cut.violation(x) <= 1e-4) return std::nullopt;
                    return cut;
                }, add);
                const double t = timed_solve(m);
                std::println(stderr, "C10 {} {} {}: {} | depth {} err={} viol_at_add={:.3f} nz={} later root viol/checks {}/{} tree {}/{} numcuts_user={}", kind ? "cover" : "floor", inst, mn, summary(m, t), h.add_depth, h.err, h.viol_at_add, h.cut ? h.cut->ind.size() : 0, h.root_viol, h.root_checks, h.tree_viol, h.tree_checks, vnumcuts(m));
            }
        }
    }
    if(what == "repeat") {
        for(const auto & [iname, n, seed] : std::initializer_list<std::tuple<const char *, int, unsigned>>{{"atsp26s5", 26, 5}, {"atsp31s5", 31, 5}}) for(const int rp : {-1, 0, 1, 2, 3}) {
            for(const auto & [mn, mp, add] : std::initializer_list<std::tuple<const char *, int, bool>>{{"NOT-ADDED baseline", CPX_USECUT_FORCE, false}, {"FORCE", CPX_USECUT_FORCE, true}, {"PURGE", CPX_USECUT_PURGE, true}}) {
                M m; m.set_thread_limit(1); auto a = build_atsp(m, n, false, seed); vparam(m, CPX_PARAM_REPEATPRESOLVE, rp); m.set_time_limit(duration<double>(20));
                auto & h = attach(m, mp, [a](const std::vector<double> & x, cxlong d) -> std::optional<raw_cut> {
                    if(d != 0) return std::nullopt;
                    auto cuts = a.separate(x, 1);
                    if(cuts.empty()) return std::nullopt;
                    return cuts.front();
                }, add);
                if(std::getenv("WITH_LOG")) { vparam(m, CPX_PARAM_SCRIND, 1); std::println("#### {} rp={} {}", iname, rp, mn); std::fflush(stdout); }
                const double t = timed_solve(m);
                std::println(stderr, "C10 RepeatPresolve={} {} {}: {} | err={} depth {} later root viol/checks {}/{} tree {}/{} before_restart {}/{} after {}/{} restarts={} numcuts_user={}", rp, iname, mn, summary(m, t), h.err, h.add_depth, h.root_viol, h.root_checks, h.tree_viol, h.tree_checks, h.r0_viol, h.r0_checks, h.r1_viol, h.r1_checks, h.maxrestarts, vnumcuts(m));
            }
        }
    }
}
