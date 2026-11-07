// Q8: overlap, thread ids, what a global cut does across threads, the wrapper's candidate path.
#include "probe_cplex_common.cpp"
using cp::raw_cut;
struct OV {
    const syms * s; int cols; std::chrono::microseconds spin;
    std::atomic<long> calls{0}, overlaps{0}, in_cand{0}, thread_err{0}, threads_err{0};
    std::atomic<int> inflight{0}, maxinflight{0}, threads_reported{0};
    std::mutex m; std::set<int> tids; std::set<std::thread::id> os;
};
static int ovhook(ctxptr ctx, cpxlong id, void * usr) {
    auto & o = *static_cast<OV *>(usr);
    if(id != c::CPX_CALLBACKCONTEXT_RELAXATION && id != c::CPX_CALLBACKCONTEXT_CANDIDATE) return 0;
    const int now = o.inflight.fetch_add(1) + 1;
    if(now > 1) o.overlaps++;
    { int s = o.maxinflight.load(); while(now > s && !o.maxinflight.compare_exchange_weak(s, now)) {} }
    o.calls++;
    if(id == c::CPX_CALLBACKCONTEXT_CANDIDATE) o.in_cand++;
    int tid = -1, n = 0;
    if(o.s->infoint(ctx, INFO_THREADID, &tid)) o.thread_err++;
    if(o.s->infoint(ctx, INFO_THREADS, &n)) o.threads_err++; else o.threads_reported = n;
    { std::lock_guard l(o.m); o.tids.insert(tid); o.os.insert(std::this_thread::get_id()); }
    if(o.spin.count()) { const auto until = clk::now() + o.spin; while(clk::now() < until) {} }
    o.inflight.fetch_sub(1);
    return 0;
}
static void overlap(const std::string & name, int threads, int parallel, cpxlong mask, std::chrono::microseconds spin, double tl = 5) {
    M m; if(threads) param(m, P_THREADS, threads); param(m, P_PARALLEL, parallel);
    cp::build_mkp(m, 30, 100);
    m.set_time_limit(std::chrono::duration<double>(tl));
    OV o; o.s = &syms::get(m); o.cols = 100; o.spin = spin;
    auto [env, lp] = m.native_model();
    m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, mask, ovhook, &o));
    const double t = cp::timed_solve(m);
    std::println(stderr, "== overlap {}: {}\n   calls={} candidate calls={} overlaps={} max_inflight={} CPXCALLBACKINFO_THREADS={} THREADID values={} (min {} max {}) OS threads={}", name, cp::summary(m, t), o.calls.load(), o.in_cand.load(), o.overlaps.load(), o.maxinflight.load(), o.threads_reported.load(), o.tids.size(), o.tids.empty() ? -1 : *o.tids.begin(), o.tids.empty() ? -1 : *o.tids.rbegin(), o.os.size());
    std::fflush(stderr);
}
// A cut added by one thread: which threads then read points violating it?
struct VIS {
    const c::cplex_api * api; const syms * s; int cols, purgeable, local; const cp::gadgets<M> * g;
    std::mutex m; std::vector<std::pair<raw_cut, int>> added; std::set<int> done;  // cut, adding thread
    long same_c = 0, same_v = 0, other_c = 0, other_v = 0, calls = 0, cuts = 0; std::set<int> tids;
};
static int vishook(ctxptr ctx, cpxlong id, void * usr) {
    auto & v = *static_cast<VIS *>(usr);
    if(id != c::CPX_CALLBACKCONTEXT_RELAXATION) return 0;
    std::vector<double> x(static_cast<std::size_t>(v.cols));
    v.s->point(ctx, x.data(), 0, v.cols - 1, nullptr);
    int tid = -1; v.s->infoint(ctx, INFO_THREADID, &tid);
    std::lock_guard l(v.m);
    v.calls++; v.tids.insert(tid);
    for(auto & [c0, by] : v.added) {
        const bool bad = c0.violation(x) > 1e-5;
        if(by == tid) { v.same_c++; v.same_v += bad; } else { v.other_c++; v.other_v += bad; }
    }
    for(int i = 1; i < v.g->k; ++i) {
        if(v.done.count(i)) continue;
        const auto r = v.g->hull_cut(i);
        if(r.violation(x) > 1e-4) {
            v.done.insert(i);
            const int beg = 0;
            if(v.api->callbackaddusercuts(ctx, 1, static_cast<int>(r.ind.size()), &r.rhs, &r.sense, &beg, r.ind.data(), r.val.data(), &v.purgeable, &v.local) == 0) { v.added.emplace_back(r, tid); v.cuts++; }
            break;
        }
    }
    return 0;
}
static void visibility(const std::string & mode, int threads, int parallel, int k) {
    M m; scope_settings(m); param(m, P_THREADS, threads); param(m, P_PARALLEL, parallel);
    auto g = cp::build_gadgets(m, k, false);
    m.set_time_limit(std::chrono::duration<double>(20));
    const auto [pg, lc] = mode_flags(mode);
    VIS v{&m.native_api(), &syms::get(m), static_cast<int>(m.num_variables()), pg, lc, &g};
    auto [env, lp] = m.native_model();
    m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, c::CPX_CALLBACKCONTEXT_RELAXATION, vishook, &v));
    const double t = cp::timed_solve(m);
    std::println(stderr, "== hull cuts across {} threads parallel={} k={} {}: {} (known opt {})\n   calls={} cuts={} threads seen={} older-cut violations: by the adding thread {}/{}, by another thread {}/{}", threads, parallel, k, mode, cp::summary(m, t), g.opt, v.calls, v.cuts, v.tids.size(), v.same_v, v.same_c, v.other_v, v.other_c);
    std::fflush(stderr);
}
static void s1(const std::string & mode, int threads, int parallel, bool flip) {
    M m; scope_settings(m); param(m, P_THREADS, threads); param(m, P_PARALLEL, parallel);
    auto g = cp::build_gadgets(m, 12, flip);
    m.set_time_limit(std::chrono::duration<double>(20));
    auto s = std::make_shared<cp::stats>();
    auto tr = std::make_shared<cp::scope_tracker>();
    tr->cut = g.invalid_global_cut(); tr->trigger = g.worse_state(); tr->state = [&g](const auto & x) { return g.state(x); };
    attach(m, cp::watched([tr](const auto & x, const cp::add_fn & add) { if(tr->wants_add(x)) tr->add_error = add(tr->cut); }, s, [&g](const auto & x) { return g.objective(x); }, [&g](const auto & x) { return g.point_ok(x); }), mode);
    const double t = cp::timed_solve(m);
    double value = NAN; try { value = m.get_solution_value(); } catch(const std::exception &) {}
    std::println(stderr, "== S1 at {} threads parallel={} k=12 mode {} flip={}: {} (known opt {}) => {}\n   {}\n   {}", threads, parallel, mode, flip, cp::summary(m, t), g.opt, std::fabs(value - g.opt) < 1e-4 ? "KEPT" : "LOST", tr->line(), s->line());
    flush_diag();
    std::fflush(stderr);
}
int main() {
    std::println(stderr, "library {}", M{}.native_api().library_path().string());
    const cpxlong R = c::CPX_CALLBACKCONTEXT_RELAXATION, C = c::CPX_CALLBACKCONTEXT_CANDIDATE;
    using us = std::chrono::microseconds;
    overlap("default threads, deterministic default, R, 200us", 0, 0, R, us(200));
    overlap("4 threads, deterministic explicit, R, 200us", 4, 1, R, us(200));
    overlap("4 threads, opportunistic, R, 200us", 4, -1, R, us(200));
    overlap("2 threads, deterministic, R, 200us", 2, 1, R, us(200));
    overlap("1 thread, R, 200us", 1, 0, R, us(200));
    overlap("4 threads, deterministic, R|C no spin", 4, 1, R | C, us(0));
    overlap("4 threads, deterministic, C only, 2ms", 4, 1, C, us(2000));
    // path invariance at more than one thread, deterministic: the run is fixed by a node limit, compared on incumbents
    for(int threads : {4}) {
        for(int pass = 0; pass < 2; ++pass) for(bool hook_on : {false, true}) {
            M m; param(m, P_THREADS, threads); param(m, P_PARALLEL, 1);
            cp::build_atsp(m, 26);
            m.set_time_limit(std::chrono::duration<double>(40));
            OV o; o.s = &syms::get(m); o.cols = static_cast<int>(m.num_variables()); o.spin = us(0);
            if(hook_on) { auto [env, lp] = m.native_model(); m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, R, ovhook, &o)); }
            const double t = cp::timed_solve(m);
            std::println(stderr, "== atsp26 {} threads deterministic pass {} {}: {} calls={}", threads, pass, hook_on ? "no-op hook" : "no hook", cp::summary(m, t), o.calls.load());
            std::fflush(stderr);
        }
    }
    for(const std::string mode : {"global-purge", "global-force", "global-filter", "local-purge", "local-force"}) for(int th : {4, 8}) visibility(mode, th, 1, 13);
    for(const std::string mode : {"global-purge", "global-force"}) visibility(mode, 4, -1, 13);
    for(const std::string mode : {"global-purge", "global-force", "local-purge", "local-force"}) for(int par : {1, -1}) for(bool flip : {false, true}) s1(mode, 4, par, flip);
    // the wrapper's candidate path at default threads: its own callback_fun, no lock, native calls in the handle
    {
        M m; 
        auto a = cp::build_atsp(m, 26);
        m.set_time_limit(std::chrono::duration<double>(20));
        std::atomic<int> in{0}, mx{0}; std::atomic<long> calls{0}, ov{0};
        m.set_candidate_solution_callback([&](auto & h) {
            const int now = in.fetch_add(1) + 1;
            if(now > 1) ov++;
            int s = mx.load(); while(now > s && !mx.compare_exchange_weak(s, now)) {}
            calls++;
            const auto until = clk::now() + std::chrono::microseconds(3000);
            while(clk::now() < until) {}
            (void)h.get_solution_value();
            in.fetch_sub(1);
        });
        const double t = cp::timed_solve(m);
        std::println(stderr, "== wrapper candidate callback, default threads, 3ms spin: {}\n   calls={} overlaps={} max_inflight={}", cp::summary(m, t), calls.load(), ov.load(), mx.load());
    }
}
