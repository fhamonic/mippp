#include <print>

#include "mippp/solvers/highs/all.hpp"

using namespace mippp;
using namespace mippp::operators;

int main() {
    highs_api api;        // loads the HiGHS C API at runtime
    highs_lp model(api);  // swap for gurobi_*, cplex_*, cbc_*, ...

    auto x1 = model.add_variable();
    auto x2 = model.add_variable({.upper_bound = 3});

    model.set_maximization();
    model.set_objective(4 * x1 + 5 * x2);
    model.add_constraint(x1 <= 4);
    model.add_constraint(2 * x1 + x2 <= 9);

    model.solve();
    auto sol = model.get_solution();

    std::println("objective = {}", model.get_solution_value());
    std::println("x1 = {}, x2 = {}", sol[x1], sol[x2]);
    return 0;
}
