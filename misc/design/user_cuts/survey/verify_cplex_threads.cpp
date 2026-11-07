// C2: overlap by context. C3: the shipped candidate callback, and CPXgetnumcols inside a callback.
#include "verify_cplex_common.hpp"
using namespace cp;
using std::chrono::duration;
struct ov {
    std::atomic<long> rel_calls{0}, cand_calls{0};
    std::atomic<int> in_rel{0}, in_cand{0}, max_rel{0}, max_cand{0};
    std::atomic<long> rel_rel{0}, cand_cand{0}, rel_saw_cand{0}, cand_saw_rel{0};
    std::chrono::microseconds spin_rel{0}, spin_cand{0};
    std::set<int> * tids = nullptr;
    const vsyms * s = nullptr;
    std::atomic<long> getnumcols_calls{0}, getnumcols_bad{0};
    int cols = 0;
    cx::CPXCENVptr env = nullptr; cx::CPXCLPptr lp = nullptr;
    bool hammer = false;
};
static void up(std::atomic<int> & m, int v) { int c = m.load(); while(v > c && !m.compare_exchange_weak(c, v)) {} }
static void spin(std::chrono::microseconds d) { if(!d.count()) return; const auto u = clk::now() + d; while(clk::now() < u) {} }
static int ohook(ctxptr, cxlong id, void * usr) {
    auto & h = *static_cast<ov *>(usr);
    if(id == CPX_CALLBACKCONTEXT_RELAXATION) {
        ++h.rel_calls;
        const int n = h.in_rel.fetch_add(1) + 1; up(h.max_rel, n);
        if(n > 1) ++h.rel_rel;
        if(h.in_cand.load() > 0) ++h.rel_saw_cand;
        spin(h.spin_rel);
        if(h.hammer) { // documented undefined: env and lp from inside a callback, on purpose, counted
            for(int k = 0; k < 200; ++k) { ++h.getnumcols_calls; if(h.s->getnumcols(h.env, h.lp) != h.cols) ++h.getnumcols_bad; }
        }
        if(h.in_cand.load() > 0) ++h.rel_saw_cand;
        h.in_rel.fetch_sub(1);
    } else if(id == CPX_CALLBACKCONTEXT_CANDIDATE) {
        ++h.cand_calls;
        const int n = h.in_cand.fetch_add(1) + 1; up(h.max_cand, n);
        if(n > 1) ++h.cand_cand;
        if(h.in_rel.load() > 0) ++h.cand_saw_rel;
        spin(h.spin_cand);
        if(h.hammer) { for(int k = 0; k < 200; ++k) { ++h.getnumcols_calls; if(h.s->getnumcols(h.env, h.lp) != h.cols) ++h.getnumcols_bad; } }
        if(h.in_rel.load() > 0) ++h.cand_saw_rel;
        h.in_cand.fetch_sub(1);
    }
    return 0;
}
static void c2() {
    struct cf_ { const char * name; int threads; int parallel; int spin_r; int spin_c; bool cand; };
    const cf_ cfgs[] = {
        {"R only, default threads, no spin", 0, 0, 0, 0, false},
        {"R only, default threads, 200us", 0, 0, 200, 0, false},
        {"R only, 4 threads det, 200us", 4, 1, 200, 0, false},
        {"R only, 4 threads opp, 200us", 4, -1, 200, 0, false},
        {"R only, 2 threads det, 200us", 2, 1, 200, 0, false},
        {"R only, 1 thread, 200us", 1, 0, 200, 0, false},
        {"C|R, default threads, no spin", 0, 0, 0, 0, true},
        {"C|R, default threads, 200us/200us", 0, 0, 200, 200, true},
        {"C|R, 4 threads opp, 200us/200us", 4, -1, 200, 200, true},
        {"C|R, 1 thread, 200us/200us", 1, 0, 200, 200, true},
    };
    for(const auto & f : cfgs) for(const char * inst : {"mkp30x100", "mkp10x200"}) {
        M m; if(inst[3] == '3') build_mkp(m, 30, 100, 7); else build_mkp(m, 10, 200, 3);
        m.set_time_limit(duration<double>(5));
        if(f.threads) set_threads(m, f.threads);
        if(f.parallel) vparam(m, CPXPARAM_Parallel, f.parallel);
        static std::vector<std::unique_ptr<ov>> keep;
        keep.push_back(std::make_unique<ov>()); auto & h = *keep.back();
        h.spin_rel = std::chrono::microseconds(f.spin_r); h.spin_cand = std::chrono::microseconds(f.spin_c);
        auto [env, lp] = m.native_model();
        const long mask = CPX_CALLBACKCONTEXT_RELAXATION | (f.cand ? CPX_CALLBACKCONTEXT_CANDIDATE : 0);
        m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, mask, ohook, &h));
        const double t = timed_solve(m);
        std::println(stderr, "C2 {} {}: {} | rel={} rel_overlap={} max_rel={} cand={} cand_overlap={} max_cand={} rel_saw_cand={} cand_saw_rel={}", inst, f.name, summary(m, t), h.rel_calls.load(), h.rel_rel.load(), h.max_rel.load(), h.cand_calls.load(), h.cand_cand.load(), h.max_cand.load(), h.rel_saw_cand.load(), h.cand_saw_rel.load());
    }
}
// Shipped API only: set_candidate_solution_callback, handle.num_variables, get_solution, add_lazy_constraint.
template <class Build>
static void shipped(const char * name, Build build, const int threads, const int spin_us, const bool lazy) {
    M m; if(threads) set_threads(m, threads);
    auto st = build(m); // returns atsp
    const std::size_t want = m.num_variables();
    std::atomic<long> calls{0}, overlaps{0}, bad_numvars{0}, lazy_added{0}; std::atomic<int> in{0}, mx{0};
    std::set<std::thread::id> tids; std::mutex tm;
    m.set_time_limit(duration<double>(20));
    m.set_candidate_solution_callback([&](auto & handle) {
        const int n = in.fetch_add(1) + 1; up(mx, n); if(n > 1) ++overlaps; ++calls;
        { std::lock_guard l(tm); tids.insert(std::this_thread::get_id()); }
        if(handle.num_variables() != want) ++bad_numvars;
        spin(std::chrono::microseconds(spin_us));
        if(lazy) {
            // get_solution() and add_lazy_constraint() both go through CPXgetnumcols
            auto sol = handle.get_solution(); (void)sol;
            (void)sol;
        }
        in.fetch_sub(1);
    });
    const double t = timed_solve(m);
    std::println(stderr, "C3 shipped {} threads={} spin={}us: {} | calls={} overlaps={} max={} bad_numvars={} os_threads={}", name, threads, spin_us, summary(m, t), calls.load(), overlaps.load(), mx.load(), bad_numvars.load(), tids.size());
}
static void c3() {
    const auto mk = [](M & m) { build_mkp(m, 10, 200, 3); return 0; };
    const auto mk2 = [](M & m) { build_mkp(m, 5, 60, 3); return 0; };
    for(int rep = 0; rep < 2; ++rep) {
        shipped("mkp10x200", mk, 0, 0, true);
        shipped("mkp10x200", mk, 0, 3000, true);
        shipped("mkp10x200", mk, 4, 3000, true);
        shipped("mkp5x60", mk2, 0, 3000, true);
    }
    // the hammer: getnumcols(env, lp) directly from inside both contexts, 200 reads per call, at 16 threads.
    for(const char * inst : {"mkp10x200", "mkp30x100"}) {
        M m; if(inst[3] == '3') build_mkp(m, 30, 100, 7); else build_mkp(m, 10, 200, 3);
        m.set_time_limit(duration<double>(10));
        static std::vector<std::unique_ptr<ov>> keep;
        keep.push_back(std::make_unique<ov>()); auto & h = *keep.back();
        auto [env, lp] = m.native_model();
        h.s = &vsyms::get(m); h.env = env; h.lp = lp; h.cols = static_cast<int>(m.num_variables()); h.hammer = true;
        m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, CPX_CALLBACKCONTEXT_RELAXATION | CPX_CALLBACKCONTEXT_CANDIDATE, ohook, &h));
        const double t = timed_solve(m);
        std::println(stderr, "C3 hammer {}: {} | rel={} cand={} getnumcols reads={} wrong={}", inst, summary(m, t), h.rel_calls.load(), h.cand_calls.load(), h.getnumcols_calls.load(), h.getnumcols_bad.load());
    }
}
int main(int, char ** argv) {
    { M m; print_version(m); }
    const std::string what = argv[1] ? argv[1] : "";
    if(what == "c2") c2();
    if(what == "c3") c3();
}
