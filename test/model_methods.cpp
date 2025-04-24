#undef NDEBUG
#include <gtest/gtest.h>

#include "assert_helper.hpp"
#include "models.hpp"

using namespace fhamonic::mippp;
using namespace fhamonic::mippp::operators;

#define EPSILON 1e-10
#define INFTY 1e20

// GTEST_TEST(any, test) {
//     // mosek_api api("mosek64",
//     // "/home/plaiseek/Softwares/mosek/11.0/tools/platform/linux64x86/bin");
//     cplex_api api(
//         "/home/plaiseek/Softwares/cplex-community/cplex/bin/x86-64_linux");

//     ASSERT_TRUE(false);
// }

// clang-format off
using Models = ::testing::Types<
        grb_lp_test,
        grb_milp_test,
        clp_lp_test,
        cbc_milp_test,
        glpk_lp_test,
        glpk_milp_test,
        soplex_lp_test,
        scip_milp_test,
        highs_lp_test,
        highs_milp_test,
        cplex_lp_test
        // ,cplex_milp_test
        // ,mosek_lp_test
        // ,mosek_milp_test

        // ,copt_lp_test
        // ,copt_milp_test
        // ,xprs_lp_test
        // ,xprs_milp_test
        >;
// clang-format on

template <typename T>
class ModelMethods : public ::testing::Test, public T {};

TYPED_TEST_SUITE(ModelMethods, Models);

TYPED_TEST(ModelMethods, add_variable) {
    using T = TypeParam::model_type;
    auto model = this->construct_model();
    auto x = model.add_variable();
    ASSERT_EQ(model.num_variables(), 1);
    ASSERT_EQ(x.id(), 0);
    if constexpr(has_readable_objective<T>) {
        ASSERT_EQ(model.get_objective_coefficient(x), 0.0);
    }
    if constexpr(has_readable_variables_bounds<T>) {
        ASSERT_EQ(model.get_variable_lower_bound(x), 0.0);
        ASSERT_GE(model.get_variable_upper_bound(x), INFTY);
    }
}
TYPED_TEST(ModelMethods, add_variable_2) {
    using T = TypeParam::model_type;
    auto model = this->construct_model();
    auto x = model.add_variable();
    auto y = model.add_variable();
    ASSERT_EQ(model.num_variables(), 2);
    ASSERT_EQ(x.id(), 0);
    ASSERT_EQ(y.id(), 1);
    if constexpr(has_readable_objective<T>) {
        ASSERT_EQ(model.get_objective_coefficient(x), 0.0);
        ASSERT_EQ(model.get_objective_coefficient(y), 0.0);
    }
    if constexpr(has_readable_variables_bounds<T>) {
        ASSERT_EQ(model.get_variable_lower_bound(x), 0.0);
        ASSERT_GE(model.get_variable_upper_bound(x), INFTY);
        ASSERT_EQ(model.get_variable_lower_bound(y), 0.0);
        ASSERT_GE(model.get_variable_upper_bound(y), INFTY);
    }
}
TYPED_TEST(ModelMethods, add_variable_w_params) {
    using T = TypeParam::model_type;
    T model = this->construct_model();
    auto x = model.add_variable({.obj_coef = 7.0, .upper_bound = 13.0});
    auto y = model.add_variable({.lower_bound = 2});
    ASSERT_EQ(model.num_variables(), 2);
    ASSERT_EQ(x.id(), 0);
    ASSERT_EQ(y.id(), 1);
    if constexpr(has_readable_objective<T>) {
        ASSERT_EQ(model.get_objective_coefficient(x), 7.0);
        ASSERT_EQ(model.get_objective_coefficient(y), 0.0);
    }
    if constexpr(has_readable_variables_bounds<T>) {
        ASSERT_EQ(model.get_variable_upper_bound(x), 13.0);
        ASSERT_LE(model.get_variable_lower_bound(x), -INFTY);
        ASSERT_EQ(model.get_variable_lower_bound(y), 2.0);
        ASSERT_GE(model.get_variable_upper_bound(y), INFTY);
    }
}
TYPED_TEST(ModelMethods, add_variables) {
    using T = TypeParam::model_type;
    auto model = this->construct_model();

    auto F = model.add_variable();
    auto X_vars = model.add_variables(
        4, {.obj_coef = 1, .lower_bound = 2, .upper_bound = 5});
    auto Y_vars = model.add_variables(3);

    ASSERT_EQ(model.num_variables(), 8);
    ASSERT_EQ(F.id(), 0);
    ASSERT_EQ(X_vars[0].id(), 1);
    ASSERT_EQ(X_vars[1].id(), 2);
    ASSERT_EQ(X_vars[2].id(), 3);
    ASSERT_EQ(X_vars[3].id(), 4);
    ASSERT_EQ(Y_vars[0].id(), 5);
    ASSERT_EQ(Y_vars[1].id(), 6);
    ASSERT_EQ(Y_vars[2].id(), 7);

    ASSERT_EQ_RANGES(X_vars, {X_vars[0], X_vars[1], X_vars[2], X_vars[3]});
    ASSERT_EQ_RANGES(Y_vars, {Y_vars[0], Y_vars[1], Y_vars[2]});

    // ASSERT_DEATH creates subprocesses that increase computation time
    ASSERT_DEATH(X_vars[-1], ".*Assertion.*failed.*");
    ASSERT_DEATH(X_vars[4], ".*Assertion.*failed.*");
    ASSERT_DEATH(Y_vars[-1], ".*Assertion.*failed.*");
    ASSERT_DEATH(Y_vars[3], ".*Assertion.*failed.*");

    if constexpr(has_readable_objective<T>) {
        for(auto && x : X_vars)
            ASSERT_EQ(model.get_objective_coefficient(x), 1);
        for(auto && y : Y_vars)
            ASSERT_EQ(model.get_objective_coefficient(y), 0);
    }
    if constexpr(has_readable_variables_bounds<T>) {
        for(auto && x : X_vars) {
            ASSERT_EQ(model.get_variable_lower_bound(x), 2.0);
            ASSERT_EQ(model.get_variable_upper_bound(x), 5.0);
        }
        for(auto && y : Y_vars) {
            ASSERT_EQ(model.get_variable_lower_bound(y), 0.0);
            ASSERT_GE(model.get_variable_upper_bound(y), INFTY);
        }
    }
}
TYPED_TEST(ModelMethods, add_indexed_variables) {
    using T = TypeParam::model_type;
    auto model = this->construct_model();

    auto F = model.add_variable();
    auto X_vars = model.add_variables(
        4, [](int a) { return 3 - a; },
        {.obj_coef = 1, .lower_bound = 2, .upper_bound = 5});
    auto Y_vars =
        model.add_variables(4, [](int a, int b) { return a * 2 + b; });

    ASSERT_EQ(model.num_variables(), 9);
    ASSERT_EQ(F.id(), 0);
    ASSERT_EQ(X_vars(0).id(), 4);
    ASSERT_EQ(X_vars(1).id(), 3);
    ASSERT_EQ(X_vars(2).id(), 2);
    ASSERT_EQ(X_vars(3).id(), 1);
    ASSERT_EQ(Y_vars(0, 0).id(), 5);
    ASSERT_EQ(Y_vars(0, 1).id(), 6);
    ASSERT_EQ(Y_vars(1, 0).id(), 7);
    ASSERT_EQ(Y_vars(1, 1).id(), 8);

    // ASSERT_DEATH creates subprocesses that increase computation time
    ASSERT_DEATH(X_vars(-1), ".*Assertion.*failed.*");
    ASSERT_DEATH(X_vars(4), ".*Assertion.*failed.*");
    ASSERT_DEATH(Y_vars(0, -1), ".*Assertion.*failed.*");
    ASSERT_DEATH(Y_vars(2, 0), ".*Assertion.*failed.*");

    ASSERT_EQ_RANGES(X_vars, {X_vars(3), X_vars(2), X_vars(1), X_vars(0)});
    ASSERT_EQ_RANGES(Y_vars,
                     {Y_vars(0, 0), Y_vars(0, 1), Y_vars(1, 0), Y_vars(1, 1)});

    if constexpr(has_readable_objective<T>) {
        for(auto && x : X_vars)
            ASSERT_EQ(model.get_objective_coefficient(x), 1);
        for(auto && y : Y_vars)
            ASSERT_EQ(model.get_objective_coefficient(y), 0);
    }
    if constexpr(has_readable_variables_bounds<T>) {
        for(auto && x : X_vars) {
            ASSERT_EQ(model.get_variable_lower_bound(x), 2.0);
            ASSERT_EQ(model.get_variable_upper_bound(x), 5.0);
        }
        for(auto && y : Y_vars) {
            ASSERT_EQ(model.get_variable_lower_bound(y), 0.0);
            ASSERT_GE(model.get_variable_upper_bound(y), INFTY);
        }
    }
}
TYPED_TEST(ModelMethods, optimize_empty_min) {
    using T = TypeParam::model_type;
    T model = this->construct_model();
    model.set_minimization();
    model.solve();
    if constexpr(has_lp_status<T>) {
        ASSERT_EQ(model.get_lp_status().value(), lp_status::optimal);
    }
    ASSERT_DOUBLE_EQ(model.get_solution_value(), 0.0);
}
TYPED_TEST(ModelMethods, optimize_empty_max) {
    using T = TypeParam::model_type;
    T model = this->construct_model();
    model.set_maximization();
    model.solve();
    if constexpr(has_lp_status<T>) {
        ASSERT_EQ(model.get_lp_status().value(), lp_status::optimal);
    }
    ASSERT_DOUBLE_EQ(model.get_solution_value(), 0.0);
}
TYPED_TEST(ModelMethods, add_constraint_and_get) {
    using T = TypeParam::model_type;
    T model = this->construct_model();
    auto x = model.add_variable();
    auto y = model.add_variable();
    auto c = model.add_constraint(1.5 * x + 4 * y <= 5);
    ASSERT_EQ(model.num_constraints(), 1);
    // if constexpr(has_readable_constraint_lhs<T>) {
    //     ASSERT_EQ_SETS(model.get_constraint_lhs(c),
    //                    {{x.id(), 1.5}, {y.id(), 4.0}});
    // }
    if constexpr(has_readable_constraint_sense<T>) {
        ASSERT_EQ(model.get_constraint_sense(c),
                  constraint_relation::less_equal_zero);
    }
    if constexpr(has_readable_constraint_rhs<T>) {
        ASSERT_DOUBLE_EQ(model.get_constraint_rhs(c), 5.0);
    }
}
TYPED_TEST(ModelMethods, add_constraint_and_optimize_max_bounded) {
    using T = TypeParam::model_type;
    T model = this->construct_model();
    auto x = model.add_variable({.upper_bound = 3});
    auto y = model.add_variable({.lower_bound = 1});
    model.set_maximization();
    model.set_objective(2 * x + 1.5 * y);
    auto c = model.add_constraint(x + y <= 5);
    model.solve();
    if constexpr(has_lp_status<T>) {
        ASSERT_EQ(model.get_lp_status().value(), lp_status::optimal);
    }
    ASSERT_DOUBLE_EQ(model.get_solution_value(), 9.0);
    auto solution = model.get_solution();
    ASSERT_DOUBLE_EQ(solution[x], 3.0);
    ASSERT_DOUBLE_EQ(solution[y], 2.0);
    if constexpr(has_dual_solution<T>) {
        auto dual_solution = model.get_dual_solution();
        ASSERT_DOUBLE_EQ(dual_solution[c], 1.5);
    }
}
TYPED_TEST(ModelMethods, add_constraint_and_optimize_min_bounded) {
    using T = TypeParam::model_type;
    T model = this->construct_model();
    auto x = model.add_variable({.upper_bound = 3});
    auto y = model.add_variable({.lower_bound = 1});
    model.set_minimization();
    model.set_objective(-2 * x - 1.5 * y);
    auto c = model.add_constraint(x + y <= 5);
    model.solve();
    if constexpr(has_lp_status<T>) {
        ASSERT_EQ(model.get_lp_status().value(), lp_status::optimal);
    }
    ASSERT_DOUBLE_EQ(model.get_solution_value(), -9.0);
    auto solution = model.get_solution();
    ASSERT_DOUBLE_EQ(solution[x], 3.0);
    ASSERT_DOUBLE_EQ(solution[y], 2.0);
    if constexpr(has_dual_solution<T>) {
        auto dual_solution = model.get_dual_solution();
        ASSERT_DOUBLE_EQ(dual_solution[c], -1.5);
    }
}
TYPED_TEST(ModelMethods, add_constraint_and_optimize_max_unbounded) {
    using T = TypeParam::model_type;
    T model = this->construct_model();
    auto x = model.add_variable(
        {.lower_bound = -std::numeric_limits<double>::infinity(),
         .upper_bound = 3});
    auto y = model.add_variable({.lower_bound = 1});
    model.set_maximization();
    model.set_objective(-2 * x + -1.5 * y);
    model.add_constraint(x + y <= 5);
    model.solve();
    if constexpr(has_lp_status<T>) {
        ASSERT_EQ(model.get_lp_status().value(), lp_status::unbounded);
    }
}
TYPED_TEST(ModelMethods, add_constraint_and_optimize_min_unbounded) {
    using T = TypeParam::model_type;
    T model = this->construct_model();
    auto x = model.add_variable(
        {.lower_bound = -std::numeric_limits<double>::infinity(),
         .upper_bound = 3});
    auto y = model.add_variable({.lower_bound = 1});
    model.set_minimization();
    model.set_objective(2 * x + 1.5 * y);
    model.add_constraint(x + y <= 5);
    model.solve();
    if constexpr(has_lp_status<T>) {
        ASSERT_EQ(model.get_lp_status().value(), lp_status::unbounded);
    }
}
TYPED_TEST(ModelMethods, add_constraint_and_optimize_max_infeasible) {
    using T = TypeParam::model_type;
    T model = this->construct_model();
    auto x = model.add_variable(
        {.lower_bound = -std::numeric_limits<double>::infinity(),
         .upper_bound = 3});
    auto y = model.add_variable({.lower_bound = 1});
    model.set_maximization();
    model.set_objective(-2 * x + -1.5 * y);
    model.add_constraint(x + y <= 4);
    model.add_constraint(x + y >= 5);
    model.solve();
    if constexpr(has_lp_status<T>) {
        ASSERT_EQ(model.get_lp_status().value(), lp_status::infeasible);
    }
}
TYPED_TEST(ModelMethods, add_constraint_and_optimize_min_infeasible) {
    using T = TypeParam::model_type;
    T model = this->construct_model();
    auto x = model.add_variable(
        {.lower_bound = -std::numeric_limits<double>::infinity(),
         .upper_bound = 3});
    auto y = model.add_variable({.lower_bound = 1});
    model.set_minimization();
    model.set_objective(2 * x + 1.5 * y);
    model.add_constraint(x + y <= 4);
    model.add_constraint(x + y >= 5);
    model.solve();
    if constexpr(has_lp_status<T>) {
        ASSERT_EQ(model.get_lp_status().value(), lp_status::infeasible);
    }
}
TYPED_TEST(ModelMethods, lp_example) {
    using T = TypeParam::model_type;
    T model = this->construct_model();
    auto x1 = model.add_variable();
    auto x2 = model.add_variable();
    auto x3 = model.add_variable();
    model.set_maximization();
    model.set_objective(5 * x1 + 4 * x2 + 3 * x3);
    auto c1 = model.add_constraint(2 * x1 + 3 * x2 + x3 <= 5);
    auto c2 = model.add_constraint(4 * x1 + x2 + 2 * x3 <= 11);
    auto c3 = model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 <= 8);
    model.solve();
    if constexpr(has_lp_status<T>) {
        ASSERT_EQ(model.get_lp_status().value(), lp_status::optimal);
    }
    ASSERT_NEAR(model.get_solution_value(), 13.0, EPSILON);
    auto solution = model.get_solution();
    ASSERT_NEAR(solution[x1], 2.0, EPSILON);
    ASSERT_NEAR(solution[x2], 0.0, EPSILON);
    ASSERT_NEAR(solution[x3], 1.0, EPSILON);
    if constexpr(has_dual_solution<T>) {
        auto dual_solution = model.get_dual_solution();
        ASSERT_NEAR(dual_solution[c1], 1.0, EPSILON);
        ASSERT_NEAR(dual_solution[c2], 0.0, EPSILON);
        ASSERT_NEAR(dual_solution[c3], 1.0, EPSILON);
    }
}
TYPED_TEST(ModelMethods, lp_example_set_objective_redundant_terms) {
    using T = TypeParam::model_type;
    T model = this->construct_model();
    auto x1 = model.add_variable();
    auto x2 = model.add_variable();
    auto x3 = model.add_variable();
    model.set_maximization();
    model.set_objective(2 * x1 + 4 * x2 + 3 * x1 + 3 * x3);
    auto c1 = model.add_constraint(2 * x1 + 3 * x2 + x3 <= 5);
    auto c2 = model.add_constraint(4 * x1 + x2 + 2 * x3 <= 11);
    auto c3 = model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 <= 8);
    model.solve();
    if constexpr(has_lp_status<T>) {
        ASSERT_EQ(model.get_lp_status().value(), lp_status::optimal);
    }
    ASSERT_NEAR(model.get_solution_value(), 13.0, EPSILON);
    auto solution = model.get_solution();
    ASSERT_NEAR(solution[x1], 2.0, EPSILON);
    ASSERT_NEAR(solution[x2], 0.0, EPSILON);
    ASSERT_NEAR(solution[x3], 1.0, EPSILON);
    if constexpr(has_dual_solution<T>) {
        auto dual_solution = model.get_dual_solution();
        ASSERT_NEAR(dual_solution[c1], 1.0, EPSILON);
        ASSERT_NEAR(dual_solution[c2], 0.0, EPSILON);
        ASSERT_NEAR(dual_solution[c3], 1.0, EPSILON);
    }
}
TYPED_TEST(ModelMethods, lp_example_add_constraint_redundant_terms) {
    using T = TypeParam::model_type;
    T model = this->construct_model();
    auto x1 = model.add_variable();
    auto x2 = model.add_variable();
    auto x3 = model.add_variable();
    model.set_maximization();
    model.set_objective(5 * x1 + 4 * x2 + 3 * x3);
    auto c1 = model.add_constraint(3 * x1 - x1 + 3 * x2 + x3 <= 5);
    auto c2 = model.add_constraint(4 * x1 + x3 + x2 + x3 <= 11);
    auto c3 = model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 <= 8);
    model.solve();
    if constexpr(has_lp_status<T>) {
        ASSERT_EQ(model.get_lp_status().value(), lp_status::optimal);
    }
    ASSERT_NEAR(model.get_solution_value(), 13.0, EPSILON);
    auto solution = model.get_solution();
    ASSERT_NEAR(solution[x1], 2.0, EPSILON);
    ASSERT_NEAR(solution[x2], 0.0, EPSILON);
    ASSERT_NEAR(solution[x3], 1.0, EPSILON);
    if constexpr(has_dual_solution<T>) {
        auto dual_solution = model.get_dual_solution();
        ASSERT_NEAR(dual_solution[c1], 1.0, EPSILON);
        ASSERT_NEAR(dual_solution[c2], 0.0, EPSILON);
        ASSERT_NEAR(dual_solution[c3], 1.0, EPSILON);
    }
}
//*/