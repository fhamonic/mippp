// C: one piece of the recipe removed at a time, on the case built to have power, in one process per run.
// usage: verify_copt_ctrl <case> <recipe|neg> [after_calls] [seed]
//   zero_L   each link cut gains s - s, in L form    neg = no zero strip
//   zero_G   same in G form                          neg = no zero strip (expected no power)
//   zero_L_tree: zero only after 'after_calls' calls
//   dup      each link cut carries x as .5 + .5      neg = no folding
//   empty    a cut folding to nothing, x - x <= 1   neg = no empty skip and no filter: rc=3 reaches the native call
//   empty_thrown  as empty, a wrapper's check() in the hook: rc!=0 throws across the C frames
//   filter   a separator that re-offers all it ever found at every call  neg = no violation filter
//   hook1    any hook return value: neg returns 1
//   dispatch a progress callback set; neg lets the shipped path see MIPRELAX first
#include "verify_copt_common.hpp"
using namespace std::chrono;
int main(int argc, char ** argv) {
    const std::string kase = argv[1], mode = argv[2];
    const long after = argc > 3 ? std::atol(argv[3]) : 0;
    const unsigned seed = argc > 4 ? std::atoi(argv[4]) : 1;
    const bool neg = mode == "neg";
    setenv("CAPF", "0.2", 1);
    const auto bld = [&](V & m) { m.iparam("Threads", 1); m.set_time_limit(duration<double>(20.0)); auto v = build_mcnd(m, 14, 30, 16, seed); return v.g; };
    double base; std::string base_s;
    { V b; bld(b); const double t = cp::timed_solve(b); base = b.get_solution_value(); base_s = cp::summary(b, t); }
    V m; auto g = bld(m);
    const auto cont = g.all_cont();
    recipe_opts o;
    if(neg) {
        if(kase.starts_with("zero")) o.strip_zero = false;
        if(kase == "dup") o.fold = false;
        if(kase.starts_with("empty")) { o.skip_empty = false; o.drop_satisfied = false; }
        if(kase == "filter") o.drop_satisfied = false;
        if(kase == "hook1") o.hook_return = 1;
        if(kase == "dispatch") o.pass_relax_to_shipped_first = true;
    }
    std::atomic<long> calls{0}, offered{0}, prog{0};
    auto all = std::make_shared<std::vector<cp::raw_cut>>();
    if(kase == "dispatch") m.set_progress_callback([&](auto &) { ++prog; });
    m.attach([&](const std::vector<double> & p, const cp::add_fn & add) {
        const long k = ++calls;
        if(k <= after) return;
        if(kase == "filter") {
            for(const auto & c : g.links(p, 5)) all->push_back(c);
            for(const auto & c : *all) { add(c); ++offered; }
            return;
        }
        for(auto c : g.links(p, 10)) {
            int s = cont[(std::size_t)(offered * 7 + 3) % cont.size()];
            if(s == c.ind[0]) s = cont[(std::size_t)(offered * 7 + 4) % cont.size()];
            if(kase == "zero_L" || kase == "zero_L_tree") { c.ind.push_back(s); c.val.push_back(1.0); c.ind.push_back(s); c.val.push_back(-1.0); }
            if(kase == "zero_G") { for(auto & w : c.val) w = -w; c.sense = 'G'; c.rhs = -c.rhs + 0.0; c.ind.push_back(s); c.val.push_back(1.0); c.ind.push_back(s); c.val.push_back(-1.0); }
            if(kase == "dup") { const double v0 = c.val[0]; c.val[0] = v0 / 2; c.ind.push_back(c.ind[0]); c.val.push_back(v0 / 2); }
            add(c); ++offered;
        }
        if(kase.starts_with("empty")) { const int rc = add(cp::raw_cut{{cont[0], cont[0]}, {1.0, -1.0}, 'L', 1.0}); if(rc && kase == "empty_thrown") m.native_api()._check(nullptr, static_cast<c::ret_code>(rc)); }
    }, o);
    const double t = cp::timed_solve(m);
    const double v = cf::value_of(m);
    const bool ok = cf::same(v, base) && is_a<status::optimal>(m.get_status());
    std::println(stderr, "{:12} {:6} after={} seed={} {} {} | base {} | calls={} offered={} progress_calls={} | {}", kase, mode, after, seed, ok ? "OK   " : "WRONG", cp::summary(m, t), base_s, calls.load(), offered.load(), prog.load(), m.last->line());
}
