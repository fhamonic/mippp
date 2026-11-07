// Is a one-term cut honoured at the root? Tightening cuts, deliberately not valid, so that being honoured shows
// in the value. usage: verify_copt_oneterm3 <nd|gadgets> <one|two> <after_call>
#include "verify_copt_common.hpp"
using namespace std::chrono;
int main(int argc, char ** argv) {
    const std::string inst = argv[1], form = argv[2];
    const long after = std::atol(argv[3]);
    const int after_node = argc > 4 ? std::atoi(argv[4]) : 0;
    const double frac = argc > 5 ? std::atof(argv[5]) : 0.5;
    std::atomic<int> add_node{-1}; std::atomic<double> add_val{0};
    double base = NAN;
    const auto build = [&](V & m) {
        m.iparam("Threads", 1); m.set_time_limit(duration<double>(20.0));
        if(inst == "gadgets") { m.iparam("Presolve", 0); m.iparam("HeurLevel", 0); auto g = cp::build_gadgets(m, 10, false); return g.za; }
        setenv("CAPF", "0.2", 1);
        auto v = build_mcnd(m, 14, 30, 16, 1);
        return v.g.all_cont();
    };
    { V b; build(b); const double t = cp::timed_solve(b); base = b.get_solution_value(); std::println(stderr, "{} base {}", inst, cp::summary(b, t)); }
    V m; const auto cols = build(m);
    std::atomic<long> calls{0}, adds{0}, later{0}, viol{0}; std::atomic<int> rc0{0};
    cp::raw_cut cut; std::mutex mu;
    m.raw_mask = c::COPT_CBCONTEXT_MIPRELAX | c::COPT_CBCONTEXT_MIPNODE;
    m.raw = [&](c::copt_prob *, void * cb, int ctx) {
        if(ctx != c::COPT_CBCONTEXT_MIPRELAX) return 0;
        const long k = ++calls;
        std::vector<double> p(m.num_variables());
        if(m.native_api().GetCallbackInfo(cb, c::COPT_CBINFO_RELAXSOLUTION, p.data())) return 0;
        if(adds > 0) { ++later; if(cut.violation(p) > 1e-6) ++viol; return 0; }
        if(k <= after) return 0;
        int nc = 0; m.native_api().GetCallbackInfo(cb, "NodeCnt", &nc);
        if(nc < after_node) return 0;
        int j1 = -1, j2 = -1; for(int j : cols) if(p[(std::size_t)j] > 1e-3) { if(j1 < 0 || p[(std::size_t)j] > p[(std::size_t)j1]) { j2 = j1; j1 = j; } else if(j2 < 0 || p[(std::size_t)j] > p[(std::size_t)j2]) j2 = j; }
        if(j1 < 0 || j2 < 0) return 0;
        if(form == "one") cut = cp::raw_cut{{j1}, {1.0}, 'L', frac * p[(std::size_t)j1]};
        else cut = cp::raw_cut{{j1, j2}, {1.0, 1.0}, 'L', frac * (p[(std::size_t)j1] + p[(std::size_t)j2])};
        rc0 = m.native_api().AddCallbackUserCut(cb, (int)cut.ind.size(), cut.ind.data(), cut.val.data(), cut.sense, cut.rhs);
        ++adds; add_node = nc; add_val = p[(std::size_t)j1];
        return 0;
    };
    const double t = cp::timed_solve(m);
    std::println(stderr, "{} {} after_call={} after_node={} frac={} add_node={} val={:.4g}: {} base {} changed={} | calls={} adds={} rc={} later={} later_violated={}", inst, form, after, after_node, frac, add_node.load(), add_val.load(), cp::summary(m, t), base, !cf::same(cf::value_of(m), base), calls.load(), adds.load(), rc0.load(), later.load(), viol.load());
}
