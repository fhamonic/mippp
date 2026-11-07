// max_bounded of lp_status.hpp under IPM without crossover: what HiGHS says
#include "mippp/solvers/highs/all.hpp"
#include "probe_common.hpp"
using M = mippp::highs_lp;
int main(int, char ** argv) {
    M m;
    auto & api = m.native_api();
    api.setStringOptionValue(m.native_model(), "solver", argv[1]);
    api.setStringOptionValue(m.native_model(), "run_crossover", argv[2]);
    auto x = m.add_variable({.upper_bound = 3});
    auto y = m.add_variable({.lower_bound = 1});
    m.set_maximization();
    m.set_objective(2 * x + 1.5 * y);
    m.add_constraint(x + y <= 5);
    std::string r = attempt([&] { m.solve(); });
    int ps = -1, ds = -1, it = -1;
    api.getIntInfoValue(m.native_model(), "primal_solution_status", &ps);
    api.getIntInfoValue(m.native_model(), "dual_solution_status", &ds);
    api.getIntInfoValue(m.native_model(), "ipm_iteration_count", &it);
    std::println("{} {}: solve {} status={} model_status={} primal_sol={} dual_sol={} ipm={}", argv[1], argv[2], r,
                 status_name(m.get_status()), api.getModelStatus(m.native_model()), ps, ds, it);
    if(is<status::unknown>(m.get_status()) || true) {
        auto s = attempt([&] { auto sol = m.get_solution(); std::print("  x={} y={} ", sol[x], sol[y]); });
        std::println("{}", s);
    }
}
