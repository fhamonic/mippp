// Q8: a global cut, invalid outside its subtree, at several threads: how often is the optimum lost?
#include "probe_cplex_common.cpp"
static void rep(const std::string & mode, int threads, int parallel, bool flip, int runs, int k, int spin_us = 0) {
    int lost = 0, kept = 0, sib_viol_runs = 0; long sib_v = 0, sib_c = 0;
    double opt = 0;
    for(int r = 0; r < runs; ++r) {
        M m; scope_settings(m); param(m, P_THREADS, threads); param(m, P_PARALLEL, parallel);
        auto g = cp::build_gadgets(m, k, flip); opt = g.opt;
        m.set_time_limit(std::chrono::duration<double>(20));
        auto s = std::make_shared<cp::stats>();
        auto tr = std::make_shared<cp::scope_tracker>();
        tr->cut = g.invalid_global_cut(); tr->trigger = g.worse_state(); tr->state = [&g](const auto & x) { return g.state(x); };
        attach(m, cp::watched([tr](const auto & x, const cp::add_fn & add) { if(tr->wants_add(x)) tr->add_error = add(tr->cut); }, s, {}, {}, {}, std::chrono::microseconds(spin_us)), mode);
        cp::timed_solve(m);
        double value = NAN; try { value = m.get_solution_value(); } catch(const std::exception &) {}
        (std::fabs(value - g.opt) < 1e-4 ? kept : lost)++;
        sib_v += tr->sib_viol; sib_c += tr->sib_checks; sib_viol_runs += tr->sib_viol > 0;
        keep.clear();
    }
    std::println(stderr, "== S1 repeated {} threads={} parallel={} flip={} k={} spin={}us: runs={} LOST={} KEPT={} (opt {}); sibling viol/checks total {}/{}, runs with a sibling violation {}", mode, threads, parallel, flip, k, spin_us, runs, lost, kept, opt, sib_v, sib_c, sib_viol_runs);
    std::fflush(stderr);
}
int main() {
    std::println(stderr, "library {}", M{}.native_api().library_path().string());
    if(std::getenv("REP_LONG")) {
        for(int spin : {0, 300}) for(int par : {-1, 1}) for(bool flip : {false, true}) for(const std::string mode : {"global-purge", "global-force"})
            rep(mode, 4, par, flip, 150, 12, spin);
        return 0;
    }
    for(int th : {1, 4, 8, 16}) for(int par : {1, -1}) for(bool flip : {false, true}) for(const std::string mode : {"global-purge", "global-force", "local-purge"})
        rep(mode, th, par, flip, 20, 12);
}
