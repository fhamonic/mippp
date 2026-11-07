// C11: how late does a global cut reach another thread, and can a globally invalid cut miss the optimum?
#include "verify_cplex_common.hpp"
using namespace cp;
using std::chrono::duration;
struct reg { raw_cut c; std::thread::id tid; clk::time_point at; };
struct lctx {
    const vsyms * s; const cx::cplex_api * api; int cols, purgeable, local = 0; const atsp * a;
    std::mutex mx; std::vector<reg> cuts;
    long same_checks = 0, same_viol = 0, other_checks = 0, other_viol = 0;
    long other_age[5] = {0, 0, 0, 0, 0}, same_age[5] = {0, 0, 0, 0, 0}; // viol by age: <1ms <10ms <100ms <1s >=1s
    long calls = 0, added = 0;
    std::set<std::thread::id> tids;
};
static int bucket(const clk::duration d) { const double s = duration<double>(d).count(); return s < 1e-3 ? 0 : s < 1e-2 ? 1 : s < 1e-1 ? 2 : s < 1 ? 3 : 4; }
static int lhook(ctxptr context, cxlong id, void * usr) {
    auto & h = *static_cast<lctx *>(usr);
    if(id != CPX_CALLBACKCONTEXT_RELAXATION) return 0;
    try {
        std::vector<double> x(static_cast<std::size_t>(h.cols));
        if(h.s->point(context, x.data(), 0, h.cols - 1, nullptr)) return 0;
        const auto me = std::this_thread::get_id();
        const auto now = clk::now();
        std::vector<raw_cut> fresh;
        {
            std::lock_guard l(h.mx);
            ++h.calls; h.tids.insert(me);
            for(const auto & r : h.cuts) {
                const bool v = r.c.violation(x) > 1e-6;
                if(r.tid == me) { ++h.same_checks; h.same_viol += v; if(v) h.same_age[bucket(now - r.at)]++; }
                else { ++h.other_checks; h.other_viol += v; if(v) h.other_age[bucket(now - r.at)]++; }
            }
        }
        for(const auto & c : h.a->separate(x)) fresh.push_back(c);
        for(const auto & c : fresh) {
            if(c.violation(x) <= 1e-9) continue;
            int beg = 0;
            const int e = h.api->callbackaddusercuts(context, 1, static_cast<int>(c.ind.size()), &c.rhs, &c.sense, &beg, c.ind.data(), c.val.data(), &h.purgeable, &h.local);
            if(!e) { std::lock_guard l(h.mx); ++h.added; h.cuts.push_back({c, me, clk::now()}); }
        }
    } catch(...) { return 1; }
    return 0;
}
static std::vector<std::unique_ptr<lctx>> keep;
struct s1ctx { const vsyms * s; const cx::cplex_api * api; int cols, purgeable, local = 0; const gadgets<M> * g; std::atomic<bool> added{false}; std::atomic<long> calls{0}; int spin_us = 0; };
static int s1hook(ctxptr context, cxlong id, void * usr) {
    auto & h = *static_cast<s1ctx *>(usr);
    if(id != CPX_CALLBACKCONTEXT_RELAXATION) return 0;
    try {
        ++h.calls;
        std::vector<double> x(static_cast<std::size_t>(h.cols));
        if(h.s->point(context, x.data(), 0, h.cols - 1, nullptr)) return 0;
        if(h.spin_us) { const auto u = clk::now() + std::chrono::microseconds(h.spin_us); while(clk::now() < u) {} }
        if(h.added.load() || h.g->state(x) != h.g->worse_state()) return 0;
        const raw_cut c = h.g->invalid_global_cut();
        if(c.violation(x) <= 1e-4) return 0;
        if(h.added.exchange(true)) return 0;
        int beg = 0;
        h.api->callbackaddusercuts(context, 1, static_cast<int>(c.ind.size()), &c.rhs, &c.sense, &beg, c.ind.data(), c.val.data(), &h.purgeable, &h.local);
    } catch(...) { return 1; }
    return 0;
}

// late2: a long search, the first 100 cuts only, then watch how old a cut can be and still be violated.
struct l2 {
    const vsyms * s; const cx::cplex_api * api; int cols, purgeable, local = 0;
    std::vector<std::vector<double>> a; std::vector<double> b;
    struct rec { raw_cut c; std::thread::id tid; clk::time_point at; cxlong uid; };
    std::mutex mx; std::vector<rec> cuts; long calls = 0; int max_cuts = 100;
    long ck[3][5] = {}, vi[3][5] = {}; // 0 same thread same node, 1 same thread other node, 2 other thread
};
static int l2hook(ctxptr context, cxlong id, void * usr) {
    auto & h = *static_cast<l2 *>(usr);
    if(id != CPX_CALLBACKCONTEXT_RELAXATION) return 0;
    try {
        std::vector<double> x(static_cast<std::size_t>(h.cols));
        if(h.s->point(context, x.data(), 0, h.cols - 1, nullptr)) return 0;
        cxlong uid = -1; h.s->infolong(context, CPXCALLBACKINFO_NODEUID, &uid);
        const auto me = std::this_thread::get_id();
        const auto now = clk::now();
        bool want = false;
        {
            std::lock_guard l(h.mx); ++h.calls;
            for(const auto & r : h.cuts) {
                const int who = r.tid != me ? 2 : r.uid == uid ? 0 : 1;
                const int b = bucket(now - r.at);
                h.ck[who][b]++; h.vi[who][b] += r.c.violation(x) > 1e-6;
            }
            want = static_cast<int>(h.cuts.size()) < h.max_cuts;
        }
        if(!want) return 0;
        std::vector<std::size_t> order(x.size()); for(std::size_t j = 0; j < order.size(); ++j) order[j] = j;
        std::ranges::sort(order, [&](std::size_t l, std::size_t r) { return x[l] > x[r]; });
        for(std::size_t i = 0; i < h.a.size(); ++i) {
            raw_cut cut{{}, {}, 'L', 0}; double w = 0;
            for(std::size_t j : order) { w += h.a[i][j]; cut.ind.push_back(static_cast<int>(j)); cut.val.push_back(1.0); if(w > h.b[i]) { cut.rhs = static_cast<double>(cut.ind.size()) - 1; break; } }
            if(cut.violation(x) <= 1e-4) continue;
            int beg = 0;
            const int e = h.api->callbackaddusercuts(context, 1, static_cast<int>(cut.ind.size()), &cut.rhs, &cut.sense, &beg, cut.ind.data(), cut.val.data(), &h.purgeable, &h.local);
            if(!e) { std::lock_guard l(h.mx); h.cuts.push_back({std::move(cut), me, clk::now(), uid}); }
        }
    } catch(...) { return 1; }
    return 0;
}
static void late2() {
    for(const auto & [rows, cols, seed] : std::initializer_list<std::tuple<int, int, unsigned>>{{30, 100, 7}, {10, 200, 3}}) for(const int th : {1, 4, 8}) for(const int par : {1, -1}) for(const auto & [mn, mp] : std::initializer_list<std::pair<const char *, int>>{{"FORCE", CPX_USECUT_FORCE}, {"PURGE", CPX_USECUT_PURGE}}) {
        if(th == 1 && par == -1) continue;
        M m; build_mkp(m, rows, cols, seed); set_threads(m, th); vparam(m, CPXPARAM_Parallel, par); m.set_time_limit(duration<double>(8));
        l2 h; h.s = &vsyms::get(m); h.api = &m.native_api(); h.cols = static_cast<int>(m.num_variables()); h.purgeable = mp;
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
            m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, CPX_CALLBACKCONTEXT_RELAXATION, l2hook, &h));
        }
        const double t = timed_solve(m);
        std::string line;
        const char * who[] = {"same-thread-same-node", "same-thread-other-node", "other-thread"};
        for(int w = 0; w < 3; ++w) {
            line += std::format(" | {}:", who[w]);
            for(int b = 0; b < 5; ++b) line += std::format(" {}/{}", h.vi[w][b], h.ck[w][b]);
        }
        std::println(stderr, "C11b mkp{}x{}s{} threads={} parallel={} {}: {} calls={} cuts={} viol/checks by age <1ms <10ms <100ms <1s >=1s{}", rows, cols, seed, th, par, mn, summary(m, t), h.calls, h.cuts.size(), line);
    }
}
int main(int, char ** argv) {
    { M m; print_version(m); }
    const std::string what = argv[1] ? argv[1] : "";
    if(what == "late") {
        for(const auto & [n, seed] : std::initializer_list<std::pair<int, unsigned>>{{31, 11}, {28, 11}, {25, 11}, {30, 1}}) for(const int th : {4, 8}) for(const int par : {1, -1}) for(const auto & [mn, mp] : std::initializer_list<std::pair<const char *, int>>{{"FORCE", CPX_USECUT_FORCE}, {"PURGE", CPX_USECUT_PURGE}}) {
            M m; auto a = build_atsp(m, n, false, seed); set_threads(m, th); vparam(m, CPXPARAM_Parallel, par); m.set_time_limit(duration<double>(20));
            keep.push_back(std::make_unique<lctx>()); auto & h = *keep.back();
            h.s = &vsyms::get(m); h.api = &m.native_api(); h.cols = static_cast<int>(m.num_variables()); h.purgeable = mp; h.a = &a;
            auto [env, lp] = m.native_model();
            m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, CPX_CALLBACKCONTEXT_RELAXATION, lhook, &h));
            const double t = timed_solve(m);
            std::println(stderr, "C11 atsp{}s{} threads={} parallel={} {}: {} | calls={} cuts={} os_threads={} viol/checks same-thread {}/{} other-thread {}/{} | same by age <1ms:{} <10ms:{} <100ms:{} <1s:{} >=1s:{} | other by age <1ms:{} <10ms:{} <100ms:{} <1s:{} >=1s:{}", n, seed, th, par, mn, summary(m, t), h.calls, h.added, h.tids.size(), h.same_viol, h.same_checks, h.other_viol, h.other_checks, h.same_age[0], h.same_age[1], h.same_age[2], h.same_age[3], h.same_age[4], h.other_age[0], h.other_age[1], h.other_age[2], h.other_age[3], h.other_age[4]);
        }
    }
    if(what == "late2") late2();
    if(what == "s1") {
        for(const auto & [th, par, spin, reps] : std::initializer_list<std::tuple<int, int, int, int>>{{1, 0, 0, 200}, {4, 1, 0, 500}, {4, -1, 0, 1500}, {4, -1, 300, 300}, {8, -1, 0, 800}, {16, -1, 0, 400}, {16, 1, 0, 200}}) for(const auto & [mn, mp, lo] : std::initializer_list<std::tuple<const char *, int, int>>{{"global-purge", CPX_USECUT_PURGE, 0}, {"global-force", CPX_USECUT_FORCE, 0}, {"local-purge", CPX_USECUT_PURGE, 1}}) {
            if(lo && reps > 200) { }
            const int R = lo ? std::min(reps, 100) : reps;
            int kept = 0, lost = 0, noadd = 0, notopt = 0; long calls = 0;
            const auto t0 = clk::now();
            for(int r = 0; r < R; ++r) {
                const bool flip = r % 2;
                M m; scope_settings(m); set_threads(m, th); if(par) vparam(m, CPXPARAM_Parallel, par); auto g = build_gadgets(m, 10, flip); m.set_time_limit(duration<double>(10));
                s1ctx h; h.s = &vsyms::get(m); h.api = &m.native_api(); h.cols = static_cast<int>(m.num_variables()); h.purgeable = mp; h.local = lo; h.g = &g; h.spin_us = spin;
                auto [env, lp] = m.native_model();
                m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, CPX_CALLBACKCONTEXT_RELAXATION, s1hook, &h));
                timed_solve(m);
                const double v = cf::value_of(m);
                calls += h.calls.load();
                if(!h.added.load()) ++noadd;
                else if(!is_a<status::optimal>(m.get_status())) ++notopt;
                else if(std::fabs(v - g.opt) < 1e-4) ++kept;
                else ++lost;
            }
            std::println(stderr, "C11 S1 repeated {} threads={} parallel={} spin={}us: runs={} LOST={} KEPT={} not_added={} not_optimal={} calls={} wall={:.1f}s", mn, th, par, spin, R, lost, kept, noadd, notopt, calls, duration<double>(clk::now() - t0).count());
        }
    }
}
