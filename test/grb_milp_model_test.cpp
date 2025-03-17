#include <gtest/gtest.h>

#include "assert_eq_ranges.hpp"

#include "mippp/grb_milp_model.hpp"

using namespace fhamonic::mippp;

using Var = model_variable<int, double>;

GTEST_TEST(grb_milp_model, test) {
    using namespace fhamonic::mippp::operators;
    grb_api api;
    grb_milp_model model(api);

    auto x1 = model.add_variable();
    auto x2 = model.add_variable({.upper_bound = 3});
    model.set_maximization();
    model.set_objective(2 * x1 + 5 * x2);
    model.add_constraint(x1 <= 4);
    model.add_constraint(3 * x1 + x2 <= 9);

    model.optimize();
    auto solution = model.get_primal_solution();

    ASSERT_EQ(model.get_objective_value(), 19.0);
    ASSERT_EQ(solution[x1], 2.0);
    ASSERT_EQ(solution[x2], 3.0);
}