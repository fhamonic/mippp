// The smallest possible MIP++ program: build a tiny LP and read its solution.
//
// The model is written once against the generic API; pick any backend by
// changing the two aliases below. Here we use HiGHS (open source).

#include <format>
#include <iostream>

#include "mippp/solvers/highs/all.hpp"

using namespace mippp;
using namespace mippp::operators;

using api_type = highs_api;
using lp_type = highs_lp;

int main() {
    api_type api;        // loads the HiGHS C API at runtime
    lp_type model(api);

    auto x1 = model.add_variable();
    auto x2 = model.add_variable({.upper_bound = 3});

    model.set_maximization();
    model.set_objective(4 * x1 + 5 * x2);
    model.add_constraint(x1 <= 4);
    model.add_constraint(2 * x1 + x2 <= 9);

    model.solve();
    auto sol = model.get_solution();

    std::cout << std::format("objective = {}\n", model.get_solution_value());
    std::cout << std::format("x1 = {}, x2 = {}\n", sol[x1], sol[x2]);
    return 0;
}
