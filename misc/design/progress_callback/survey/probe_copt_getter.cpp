// The model's getter after a stop, on the instances of probe_copt_bound.
#include "mippp/solvers/copt/all.hpp"
#include "probe_common.hpp"
using M = mippp::copt_milp;
template <class B> static void run(const char * label, B build, double limit) {
    M m; build(m); m.set_time_limit(std::chrono::duration<double>(limit)); m.solve();
    std::println(stderr, "{}: status={} get_dual_bound={:g}", label, status_name(m.get_status()), m.get_dual_bound());
}
int main() {
    run("no incumbent min 2s", [](M & m) { build_no_incumbent(m, 7); }, 2);
    run("no incumbent max 2s", [](M & m) { build_no_incumbent(m, 7); m.set_maximization(); }, 2);
    run("no incumbent min, 1 ms limit", [](M & m) { build_no_incumbent(m, 7); }, 0.001);
    run("no incumbent max, 1 ms limit", [](M & m) { build_no_incumbent(m, 7); m.set_maximization(); }, 0.001);
    run("incumbent min 2s", [](M & m) { build_mkp(m, make_mkp(30, 100), +1, 100.0); }, 2);
    run("solved", [](M & m) { build_mkp(m, make_mkp(5, 25), +1, 100.0); }, 30);
    run("infeasible", [](M & m) { auto x = m.add_binary_variables(2); m.set_objective(1.0 * x(0)); m.add_constraint(x(0) + x(1) >= 3); }, 5);
}
