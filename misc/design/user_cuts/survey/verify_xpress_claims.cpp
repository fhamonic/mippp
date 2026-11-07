// X1, X2, X4, X8, X9, X10 and the fall-back: each subcommand is one claim. My own models where it matters.
#include "verify_xpress_hook.hpp"
using namespace mippp; using namespace mippp::operators;
using std::chrono::duration;
static void scope(M & m) {
    for(auto [c, v] : std::vector<std::pair<int, int>>{{vx::C_PRESOLVE, 0}, {vx::C_MIPPRESOLVE, 0}, {vx::C_COVERCUTS, 0}, {vx::C_GOMCUTS, 0}, {vx::C_TREECOVERCUTS, 0}, {vx::C_TREEGOMCUTS, 0}, {vx::C_HEUREMPHASIS, 0}, {vx::C_FEASIBILITYPUMP, 0}, {vx::C_HEURBEFORELP, 0}, {vx::C_SYMMETRY, 0}, {vx::C_PRECOMPONENTS, 0}, {vx::C_MIPRESTART, 0}, {vx::C_THREADS, 1}})
        vx::setint(m, c, v);
}
// no presolve-off, no restart-off: only what stops solver cuts and heuristics, so the gadget tree stays.
static void nosolvercuts(M & m) {
    for(auto [c, v] : std::vector<std::pair<int, int>>{{vx::C_COVERCUTS, 0}, {vx::C_GOMCUTS, 0}, {vx::C_TREECOVERCUTS, 0}, {vx::C_TREEGOMCUTS, 0}, {vx::C_HEUREMPHASIS, 0}, {vx::C_FEASIBILITYPUMP, 0}, {vx::C_HEURBEFORELP, 0}, {vx::C_THREADS, 1}})
        vx::setint(m, c, v);
}
// S1: an invalid cut in the worse subtree. KEPT = local or not loaded elsewhere; LOST = in force in the sibling.
// S2: a valid hull cut added once in the worse subtree: its violation at sibling nodes.
static void s1s2(const std::string & label, const std::string & spec, std::function<void(M &)> settings, int which, bool flip = false, double limit = 30) {
    M m; settings(m);
    auto g = cp::build_gadgets(m, 10, flip);
    m.set_time_limit(duration<double>(limit));
    auto s = std::make_shared<cp::stats>();
    auto tr = std::make_shared<cp::scope_tracker>();
    tr->cut = which == 1 ? g.invalid_global_cut() : g.hull_cut(3);
    tr->trigger = g.worse_state(); tr->state = [&g](const auto & x) { return g.state(x); };
    vx::recipe r = vx::parse(spec);
    r.drop_satisfied = false;  // the tracker decides when
    auto & c = vx::attach(m, cp::watched([tr](const auto & x, const cp::add_fn & add) { if(tr->wants_add(x)) tr->add_error = add(tr->cut); }, s, {}, [&g](const auto & x) { return g.point_ok(x); }), r);
    const double t = cp::timed_solve(m);
    double v = NAN; try { v = m.get_solution_value(); } catch(const std::exception &) {}
    std::println(stderr, "{} S{} {} flip={}: {} (opt {}) {}\n   {}\n   calls={} cuts={} errs={}", label, which, spec, flip, cp::summary(m, t), g.opt, which == 1 ? (std::fabs(v - g.opt) < 1e-4 ? "optimum KEPT" : "optimum LOST") : "", tr->line(), s->calls.load(), s->cuts.load(), s->add_errors.load());
    vx::quiet_flush = true; vx::flush_stats(); std::println(stderr, "   {}", c.line());
}
// X1/X2 matrix. "open" = everything of scope except PRESOLVE and MIPRESTART: presolve runs, restarts allowed, the gadget tree survives.
static void open_scope(M & m) {
    for(auto [c, v] : std::vector<std::pair<int, int>>{{vx::C_MIPPRESOLVE, 0}, {8238 /*PREPROBING*/, 0}, {vx::C_COVERCUTS, 0}, {vx::C_GOMCUTS, 0}, {vx::C_TREECOVERCUTS, 0}, {vx::C_TREEGOMCUTS, 0}, {vx::C_HEUREMPHASIS, 0}, {vx::C_FEASIBILITYPUMP, 0}, {vx::C_HEURBEFORELP, 0}, {vx::C_SYMMETRY, 0}, {vx::C_PRECOMPONENTS, 0}, {vx::C_MIPDUALREDUCTIONS, 0}, {vx::C_THREADS, 1}})
        vx::setint(m, c, v);
}
static void x1x2() {
    const auto pres0 = [](M & mm) { open_scope(mm); vx::setint(mm, vx::C_PRESOLVE, 0); };
    const auto rest0 = [](M & mm) { open_scope(mm); vx::setint(mm, vx::C_MIPRESTART, 0); };
    const auto both0 = [](M & mm) { open_scope(mm); vx::setint(mm, vx::C_PRESOLVE, 0); vx::setint(mm, vx::C_MIPRESTART, 0); };
    for(const char * spec : {"R2", "R2cr", "R1", "R1,add2", "R1,first2", "R1,add2,first2", "R1,add2,first2,local"}) for(const int which : {1, 2}) for(const bool flip : {false, true}) {
        s1s2("open", spec, open_scope, which, flip);
        if(std::string(spec) == "R1,add2,first2" || std::string(spec) == "R1,first2") {
            s1s2("PRESOLVE=0", spec, pres0, which, flip);
            s1s2("MIPRESTART=0", spec, rest0, which, flip);
            s1s2("both 0", spec, both0, which, flip);
        }
    }
    // X1 "takes presolved columns": the same model with shifted columns, a one-column cut whose index moves.
    for(const char * spec : {"R2,no_presolverow", "R2", "R1,add2"}) {
        M m; vx::setint(m, vx::C_THREADS, 1); m.set_time_limit(duration<double>(20));
        auto f = m.add_binary_variables(40);
        for(std::size_t i = 0; i < 40; ++i) m.add_constraint(1.0 * f(i) == 1.0);
        auto a = cp::build_atsp(m, 26, false, 5);
        auto s = std::make_shared<cp::stats>();
        vx::recipe r = vx::parse(spec);
        auto & c = vx::attach(m, cp::watched([&a](const auto & x, const cp::add_fn & add) { for(const auto & cut : a.separate(x)) add(cut); }, s, [&a](const auto & x) { return a.objective(x); }, [&a](const auto & x) { return a.point_ok(x); }), r);
        const double t = cp::timed_solve(m);
        std::println(stderr, "X1 columns first shifted by 40, truth 549, {}: {} | {}\n   {}", spec, cp::summary(m, t), s->line(), c.line());
    }
}
// X4: the same cut at every call, with and without sanitize's drop_satisfied.
static void x4() {
    const auto one = [](const std::string & spec, const std::string & settings_name, std::function<void(M &)> st, double limit) {
        M m; st(m);
        auto g = cp::build_gadgets(m, 10, false);
        m.set_time_limit(duration<double>(limit));
        const cp::raw_cut cut = g.hull_cut(3);
        auto s = std::make_shared<cp::stats>();
        auto & c = vx::attach(m, cp::watched([&](const auto &, const cp::add_fn & add) { add(cut); }, s), vx::parse(spec));
        const double t = cp::timed_solve(m);
        std::println(stderr, "X4 same cut at every call, {} under {}: {} (opt {}) | calls={} cuts={}\n   {}", spec, settings_name, cp::summary(m, t), g.opt, s->calls.load(), s->cuts.load(), c.line());
    };
    for(const char * spec : {"R1,keep_satisfied", "R1,keep_satisfied,add2", "R1,keep_satisfied,add0", "R2,keep_satisfied", "R2cr,keep_satisfied", "R1", "R1,add2", "R2", "R2cr"}) {
        one(spec, "scope", scope, 5);
        one(spec, "defaults", [](M & m) { vx::setint(m, vx::C_THREADS, 1); }, 5);
    }
}
// X8: getlpsol vs getcallbacksolution, on a model where presolve moves columns. Separator compares both.
static void x8() {
    M m; vx::setint(m, vx::C_THREADS, 1); m.set_time_limit(duration<double>(15));
    auto f = m.add_binary_variables(40);
    for(std::size_t i = 0; i < 40; ++i) m.add_constraint(1.0 * f(i) == 1.0);
    auto a = cp::build_atsp(m, 26, true, 5);  // complements too
    struct q { const xn::xpress_api * api; vx::fns * fn; const cp::atsp * a; long calls = 0, same = 0, lp_ok = 0, pres_ok = 0, cb_ok = 0, lp_err = 0; int input = 0; double maxdiff = 0; } q_;
    vx::fns fn(m.native_api().library_path());
    q_.api = &m.native_api(); q_.fn = &fn; q_.a = &a;
    const auto hook = +[](prob_t cb, void * d, int *) {
        auto & c = *static_cast<q *>(d);
        int input = 0, cols = 0; c.api->getintattrib(cb, vx::A_INPUTCOLS, &input); c.api->getintattrib(cb, vx::A_COLS, &cols);
        c.input = input;
        std::vector<double> xc(static_cast<std::size_t>(input)), xl(static_cast<std::size_t>(input)), xp(static_cast<std::size_t>(input));
        int avail = 0;
        c.api->getcallbacksolution(cb, &avail, xc.data(), 0, input - 1);
        if(c.fn->getlpsol(cb, xl.data(), nullptr, nullptr, nullptr)) { ++c.lp_err; return; }
        int e = c.fn->getpresolved(cb, &avail, xp.data(), 0, cols - 1);
        ++c.calls;
        double d0 = 0; for(int j = 0; j < input; ++j) d0 = std::max(d0, std::fabs(xc[static_cast<std::size_t>(j)] - xl[static_cast<std::size_t>(j)]));
        c.maxdiff = std::max(c.maxdiff, d0);
        if(d0 < 1e-9) ++c.same;
        c.cb_ok += c.a->point_ok(xc); c.lp_ok += c.a->point_ok(xl); if(!e) c.pres_ok += c.a->point_ok(xp);
    };
    const int e = m.native_api().addcboptnode(m.native_model(), hook, &q_, 0);
    const double t = cp::timed_solve(m);
    std::println(stderr, "X8 shifted+complement ATSP n=26 no cuts: e={} {} | calls={} getlpsol==getcallbacksolution on {} (max diff {}) point_ok: callback {} getlpsol {} presolved {} lp_err={} input={}", e, cp::summary(m, t), q_.calls, q_.same, q_.maxdiff, q_.cb_ok, q_.lp_ok, q_.pres_ok, q_.lp_err, q_.input);
}
// X9
static void x9() {
    for(const int cs : {-1, 0}) for(const char * spec : {"R1", "R1,add2", "R2", "R2cr"}) {
        M m; vx::setint(m, vx::C_THREADS, 1); if(cs == 0) vx::setint(m, vx::C_CUTSTRATEGY, 0);
        m.set_time_limit(duration<double>(20));
        auto a = cp::build_atsp(m, 26, false, 5);
        auto s = std::make_shared<cp::stats>();
        auto & c = vx::attach(m, cp::watched([&a](const auto & x, const cp::add_fn & add) { for(const auto & cut : a.separate(x)) add(cut); }, s), vx::parse(spec));
        const double t = cp::timed_solve(m);
        std::println(stderr, "X9 CUTSTRATEGY={} {}: {} | calls={} cuts={}\n   {}", cs, spec, cp::summary(m, t), s->calls.load(), s->cuts.load(), c.line());
    }
    M m; vx::setint(m, vx::C_THREADS, 1); vx::setint(m, vx::C_CUTSTRATEGY, 0); m.set_time_limit(duration<double>(20));
    auto a = cp::build_atsp(m, 26, false, 5);
    const double t = cp::timed_solve(m);
    std::println(stderr, "X9 CUTSTRATEGY=0 no hook: {}", cp::summary(m, t));
}
// X10: overlap, per hook kind and across kinds: one model with both hooks at once, shared counter.
namespace ov {
struct S { std::atomic<int> in{0}, peak{0}; std::atomic<long> calls{0}, overlaps{0}, cr{0}, on{0}; std::chrono::microseconds spin{200}; std::mutex m; std::set<std::thread::id> ids; std::map<std::string, long> kind_overlap; std::atomic<int> who{0}; };
inline void enter(S & s, int kind) {
    const int now = s.in.fetch_add(1) + 1;
    if(now > 1) s.overlaps.fetch_add(1);
    int seen = s.peak.load(); while(now > seen && !s.peak.compare_exchange_weak(seen, now)) {}
    s.calls.fetch_add(1); (kind == 0 ? s.cr : s.on).fetch_add(1);
    { std::lock_guard l(s.m); s.ids.insert(std::this_thread::get_id()); }
    const auto until = clk::now() + s.spin; while(clk::now() < until) {}
    s.in.fetch_sub(1);
}
inline void cr(prob_t, void * d, int, int *) { enter(*static_cast<S *>(d), 0); }
inline void on(prob_t, void * d, int *) { enter(*static_cast<S *>(d), 1); }
}
static void x10() {
    for(const int mutex : {1, 0}) for(const int kinds : {1, 2, 3}) {
        M m; cp::build_mkp(m, 30, 100); m.set_time_limit(duration<double>(10));
        if(!mutex) vx::setint(m, vx::C_MUTEXCALLBACKS, 0);
        ov::S s;
        vx::fns fn(m.native_api().library_path());
        if(kinds & 1) fn.addcbcutround(m.native_model(), ov::cr, &s, 0);
        if(kinds & 2) m.native_api().addcboptnode(m.native_model(), ov::on, &s, 0);
        const double t = cp::timed_solve(m);
        std::println(stderr, "X10 MUTEXCALLBACKS={} hooks={}: {} | calls={} (cutround {} optnode {}) overlaps={} peak={} os_threads={}", mutex, kinds == 1 ? "cutround" : kinds == 2 ? "optnode" : "both", cp::summary(m, t), s.calls.load(), s.cr.load(), s.on.load(), s.overlaps.load(), s.peak.load(), s.ids.size());
    }
}
int main(int argc, char ** argv) {
    const std::string w = argc > 1 ? argv[1] : "";
    { M m; std::println(stderr, "release {}", vx::version(m)); }
    if(w == "x1x2") x1x2();
    if(w == "x4") x4();
    if(w == "x8") x8();
    if(w == "x9") x9();
    if(w == "x10") x10();
}
