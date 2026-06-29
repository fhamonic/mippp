#pragma once
#undef NDEBUG
#include <gtest/gtest.h>

#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"

#include "assert_helper.hpp"

namespace fhamonic::mippp {

template <typename T>
struct RemoveVariableTest : public T {
    using typename T::model_type;
    static_assert(has_remove_variable<model_type>);
};
TYPED_TEST_SUITE_P(RemoveVariableTest);

TYPED_TEST_P(RemoveVariableTest, remove_solve) {
    using namespace operators;
    auto model = this->new_model();
    auto x1 = model.add_variable();
    auto x2 = model.add_variable();
    auto x3 = model.add_variable();
    model.set_minimization();
    model.set_objective(5 * x1 + 4 * x2 + 3 * x3);
    auto c1 = model.add_constraint(2 * x1 + 2 * x2 - x3 >= 5);
    auto c2 = model.add_constraint(4 * x1 + x2 + 2 * x3 <= 11);
    auto c3 = model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 == 8);

    model.remove_variable(x2);
    ASSERT_EQ(model.num_variables(), 2);

    model.solve();
    ASSERT_NEAR(model.get_solution_value(), 93.0 / 7.0, TEST_EPSILON);
    {
        auto solution = model.get_solution();
        ASSERT_NEAR(solution[x1], 18.0 / 7.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x3], 1.0 / 7.0, TEST_EPSILON);
    }
}

TYPED_TEST_P(RemoveVariableTest, solve_remove_solve) {
    using namespace operators;
    auto model = this->new_model();
    auto x1 = model.add_variable();
    auto x2 = model.add_variable();
    auto x3 = model.add_variable();
    model.set_minimization();
    model.set_objective(5 * x1 + 4 * x2 + 3 * x3);
    auto c1 = model.add_constraint(2 * x1 + 2 * x2 - x3 >= 5);
    auto c2 = model.add_constraint(4 * x1 + x2 + 2 * x3 <= 11);
    auto c3 = model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 == 8);
    model.solve();
    ASSERT_NEAR(model.get_solution_value(), 12.0, TEST_EPSILON);
    {
        auto solution = model.get_solution();
        ASSERT_NEAR(solution[x1], 2.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x2], 0.5, TEST_EPSILON);
        ASSERT_NEAR(solution[x3], 0.0, TEST_EPSILON);
    }

    model.remove_variable(x2);
    ASSERT_EQ(model.num_variables(), 2);

    model.solve();
    ASSERT_NEAR(model.get_solution_value(), 93.0 / 7.0, TEST_EPSILON);
    {
        auto solution = model.get_solution();
        ASSERT_NEAR(solution[x1], 18.0 / 7.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x3], 1.0 / 7.0, TEST_EPSILON);
    }
}

TYPED_TEST_P(RemoveVariableTest, remove_addvar_solve) {
    using namespace operators;
    auto model = this->new_model();
    auto x1 = model.add_variable();
    auto x2 = model.add_variable();
    auto x3 = model.add_variable();
    model.set_minimization();
    model.set_objective(5 * x1 + 4 * x2 + 3 * x3);
    auto c1 = model.add_constraint(2 * x1 + 2 * x2 - x3 >= 5);
    auto c2 = model.add_constraint(4 * x1 + x2 + 2 * x3 <= 11);
    auto c3 = model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 == 8);

    model.remove_variable(x2);
    ASSERT_EQ(model.num_variables(), 2);
    auto x4 = model.add_variable(
        {.obj_coef = 2, .lower_bound = -1, .upper_bound = std::nullopt});
    ASSERT_EQ(model.num_variables(), 3);
    ASSERT_EQ(x4, x2);

    model.solve();
    ASSERT_NEAR(model.get_solution_value(), 93.0 / 7.0 - 2, TEST_EPSILON);
    {
        auto solution = model.get_solution();
        ASSERT_NEAR(solution[x1], 18.0 / 7.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x3], 1.0 / 7.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x4], -1.0, TEST_EPSILON);
    }
}

TYPED_TEST_P(RemoveVariableTest, solve_remove_addvar_solve) {
    using namespace operators;
    auto model = this->new_model();
    auto x1 = model.add_variable();
    auto x2 = model.add_variable();
    auto x3 = model.add_variable();
    model.set_minimization();
    model.set_objective(5 * x1 + 4 * x2 + 3 * x3);
    auto c1 = model.add_constraint(2 * x1 + 2 * x2 - x3 >= 5);
    auto c2 = model.add_constraint(4 * x1 + x2 + 2 * x3 <= 11);
    auto c3 = model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 == 8);
    model.solve();
    ASSERT_NEAR(model.get_solution_value(), 12.0, TEST_EPSILON);
    {
        auto solution = model.get_solution();
        ASSERT_NEAR(solution[x1], 2.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x2], 0.5, TEST_EPSILON);
        ASSERT_NEAR(solution[x3], 0.0, TEST_EPSILON);
    }

    model.remove_variable(x2);
    ASSERT_EQ(model.num_variables(), 2);
    auto x4 = model.add_variable(
        {.obj_coef = 2, .lower_bound = -1, .upper_bound = std::nullopt});
    ASSERT_EQ(model.num_variables(), 3);
    ASSERT_EQ(x4, x2);

    model.solve();
    ASSERT_NEAR(model.get_solution_value(), 93.0 / 7.0 - 2, TEST_EPSILON);
    {
        auto solution = model.get_solution();
        ASSERT_NEAR(solution[x1], 18.0 / 7.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x3], 1.0 / 7.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x4], -1.0, TEST_EPSILON);
    }
}

TYPED_TEST_P(RemoveVariableTest, remove_addcol_solve) {
    using namespace operators;
    auto model = this->new_model();
    if constexpr(!has_add_column<decltype(model)>) GTEST_SKIP();
    auto x1 = model.add_variable();
    auto x2 = model.add_variable();
    auto x3 = model.add_variable();
    model.set_minimization();
    model.set_objective(5 * x1 + 4 * x2 + 3 * x3);
    auto c1 = model.add_constraint(2 * x1 + 2 * x2 - x3 >= 5);
    auto c2 = model.add_constraint(4 * x1 + x2 + 2 * x3 <= 11);
    auto c3 = model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 == 8);

    model.remove_variable(x2);
    ASSERT_EQ(model.num_variables(), 2);
    auto x4 =
        model.add_column({{c1, 2.0}, {c2, 1.0}, {c3, 4.0}}, {.obj_coef = 4});
    ASSERT_EQ(model.num_variables(), 3);
    ASSERT_EQ(x4, x2);

    model.solve();
    ASSERT_NEAR(model.get_solution_value(), 12.0, TEST_EPSILON);
    {
        auto solution = model.get_solution();
        ASSERT_NEAR(solution[x1], 2.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x2], 0.5, TEST_EPSILON);
        ASSERT_NEAR(solution[x3], 0.0, TEST_EPSILON);
    }
}

TYPED_TEST_P(RemoveVariableTest, solve_remove_addcol_solve) {
    using namespace operators;
    auto model = this->new_model();
    if constexpr(!has_add_column<decltype(model)>) GTEST_SKIP();
    auto x1 = model.add_variable();
    auto x2 = model.add_variable();
    auto x3 = model.add_variable();
    model.set_minimization();
    model.set_objective(5 * x1 + 4 * x2 + 3 * x3);
    auto c1 = model.add_constraint(2 * x1 + 2 * x2 - x3 >= 5);
    auto c2 = model.add_constraint(4 * x1 + x2 + 2 * x3 <= 11);
    auto c3 = model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 == 8);
    model.solve();
    ASSERT_NEAR(model.get_solution_value(), 12.0, TEST_EPSILON);
    {
        auto solution = model.get_solution();
        ASSERT_NEAR(solution[x1], 2.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x2], 0.5, TEST_EPSILON);
        ASSERT_NEAR(solution[x3], 0.0, TEST_EPSILON);
    }

    model.remove_variable(x2);
    ASSERT_EQ(model.num_variables(), 2);
    auto x4 =
        model.add_column({{c1, 2.0}, {c2, 1.0}, {c3, 4.0}}, {.obj_coef = 4});
    ASSERT_EQ(model.num_variables(), 3);
    ASSERT_EQ(x4, x2);

    model.solve();
    ASSERT_NEAR(model.get_solution_value(), 12.0, TEST_EPSILON);
    {
        auto solution = model.get_solution();
        ASSERT_NEAR(solution[x1], 2.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x2], 0.5, TEST_EPSILON);
        ASSERT_NEAR(solution[x3], 0.0, TEST_EPSILON);
    }
}

TYPED_TEST_P(RemoveVariableTest, remove_addnamedvar_solve) {
    using namespace operators;
    auto model = this->new_model();
    if constexpr(!has_named_variables<decltype(model)>) GTEST_SKIP();
    auto x1 = model.add_variable();
    auto x2 = model.add_named_variable("x2");
    auto x3 = model.add_variable();
    model.set_minimization();
    model.set_objective(5 * x1 + 4 * x2 + 3 * x3);
    auto c1 = model.add_constraint(2 * x1 + 2 * x2 - x3 >= 5);
    auto c2 = model.add_constraint(4 * x1 + x2 + 2 * x3 <= 11);
    auto c3 = model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 == 8);

    model.remove_variable(x2);
    ASSERT_EQ(model.num_variables(), 2);
    auto x4 = model.add_named_variable(
        "x4", {.obj_coef = 2, .lower_bound = -1, .upper_bound = std::nullopt});
    ASSERT_EQ(model.num_variables(), 3);
    ASSERT_EQ(x4, x2);
    ASSERT_STREQ(model.get_variable_name(x4).c_str(), "x4");

    model.solve();
    ASSERT_NEAR(model.get_solution_value(), 93.0 / 7.0 - 2, TEST_EPSILON);
    {
        auto solution = model.get_solution();
        ASSERT_NEAR(solution[x1], 18.0 / 7.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x3], 1.0 / 7.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x4], -1.0, TEST_EPSILON);
    }
}

TYPED_TEST_P(RemoveVariableTest, solve_remove_addnamedvar_solve) {
    using namespace operators;
    auto model = this->new_model();
    if constexpr(!has_named_variables<decltype(model)>) GTEST_SKIP();
    auto x1 = model.add_variable();
    auto x2 = model.add_named_variable("x2");
    auto x3 = model.add_variable();
    model.set_minimization();
    model.set_objective(5 * x1 + 4 * x2 + 3 * x3);
    auto c1 = model.add_constraint(2 * x1 + 2 * x2 - x3 >= 5);
    auto c2 = model.add_constraint(4 * x1 + x2 + 2 * x3 <= 11);
    auto c3 = model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 == 8);
    model.solve();
    ASSERT_NEAR(model.get_solution_value(), 12.0, TEST_EPSILON);
    {
        auto solution = model.get_solution();
        ASSERT_NEAR(solution[x1], 2.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x2], 0.5, TEST_EPSILON);
        ASSERT_NEAR(solution[x3], 0.0, TEST_EPSILON);
    }

    model.remove_variable(x2);
    ASSERT_EQ(model.num_variables(), 2);
    auto x4 = model.add_named_variable(
        "x4", {.obj_coef = 2, .lower_bound = -1, .upper_bound = std::nullopt});
    ASSERT_EQ(model.num_variables(), 3);
    ASSERT_EQ(x4, x2);
    ASSERT_STREQ(model.get_variable_name(x4).c_str(), "x4");

    model.solve();
    ASSERT_NEAR(model.get_solution_value(), 93.0 / 7.0 - 2, TEST_EPSILON);
    {
        auto solution = model.get_solution();
        ASSERT_NEAR(solution[x1], 18.0 / 7.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x3], 1.0 / 7.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x4], -1.0, TEST_EPSILON);
    }
}

REGISTER_TYPED_TEST_SUITE_P(RemoveVariableTest, remove_solve,
                            solve_remove_solve, remove_addvar_solve,
                            solve_remove_addvar_solve, remove_addcol_solve,
                            solve_remove_addcol_solve,
                            remove_addnamedvar_solve,
                            solve_remove_addnamedvar_solve);

}  // namespace fhamonic::mippp