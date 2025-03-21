#include <gtest/gtest.h>

#include "assert_helper.hpp"

#include "mippp/model/cbc_milp_model.hpp"

using namespace fhamonic::mippp;

using Var = model_variable<int, double>;

GTEST_TEST(cbc_lp_model, test) {
    using namespace fhamonic::mippp::operators;
    cbc_api api;
    cbc_milp_model model(api);

    auto x1 = model.add_variable();
    auto x2 = model.add_variable({.upper_bound = 3});
    model.set_maximization();
    model.set_objective(2 * x1 + 5 * x2);
    model.add_constraint(x1 <= 4);
    auto c = model.add_constraint(3 * x1 + x2 <= 9);

    std::cout << "#vars = " << model.num_variables() << std::endl;
    std::cout << "#constrs = " << model.num_constraints() << std::endl;
    model.add_ranged_constraint(-x1, -3, -1);
    std::cout << "#vars = " << model.num_variables() << std::endl;
    std::cout << "#constrs = " << model.num_constraints() << std::endl;

    // int added_variable = static_cast<int>(model.num_variables()) - 1;

    // std::cout << "lb = " << model.get_variable_lower_bound(Var(added_variable))
    //           << std::endl;
    // std::cout << "ub = " << model.get_variable_upper_bound(Var(added_variable))
    //           << std::endl;



    // std::cout << model.get_constraint_rhs(c) << " "
    //           << model.get_constraint_sense(c) << std::endl;

    ASSERT_FALSE(true);

    model.optimize();
    auto solution = model.get_primal_solution();

    ASSERT_EQ(model.get_objective_value(), 19.0);
    ASSERT_EQ(solution[x1], 2.0);
    ASSERT_EQ(solution[x2], 3.0);
}