// Plain wrapper, no hook, no opening of the class: a stop before any solution on master.
#include "mippp/solvers/cbc/all.hpp"
#include "cut_common.hpp"
int main() {
    mippp::cbc_milp m; cp::build_atsp(m, 26, false, 9);
    m.set_time_limit(std::chrono::duration<double>(0.02));
    m.solve();
    std::println(stderr, "version {} status {}", m.native_api().getVersion(), cp::status_name(m.get_status()));
    std::fflush(stderr);
    const double v = m.get_solution_value();
    std::println(stderr, "value {}", v);
}
