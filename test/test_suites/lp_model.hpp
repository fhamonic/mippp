#pragma once
#undef NDEBUG
#include <gtest/gtest.h>

#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"

namespace fhamonic::mippp {

template <typename T>
struct LpModelTest : public T {
    using typename T::model_type;
    static_assert(lp_model<model_type>);
};
TYPED_TEST_SUITE_P(LpModelTest);

TYPED_TEST_P(LpModelTest, construct) {
    auto model = this->new_model();
    ASSERT_EQ(model.num_variables(), 0);
    ASSERT_EQ(model.num_constraints(), 0);
}
TYPED_TEST_P(LpModelTest, add_variable) {
    auto model = this->new_model();
    auto x1 = model.add_variable();
    auto x2 = model.add_variable();
    ASSERT_EQ(model.num_variables(), 2);
    ASSERT_EQ(model.num_constraints(), 0);
    ASSERT_EQ(x1.id(), 0);
    ASSERT_EQ(x2.id(), 1);
}
TYPED_TEST_P(LpModelTest, add_variable_params) {
    auto model = this->new_model();
    auto x1 = model.add_variable({});
    auto x2 = model.add_variable(
        {.obj_coef = 1, .lower_bound = 2, .upper_bound = 10});
    auto x3 = model.add_variable({.obj_coef = 1, .lower_bound = 2});
    auto x4 = model.add_variable({.upper_bound = 10});
    ASSERT_EQ(model.num_variables(), 4);
    ASSERT_EQ(model.num_constraints(), 0);
    ASSERT_EQ(x1.id(), 0);
    ASSERT_EQ(x2.id(), 1);
    ASSERT_EQ(x3.id(), 2);
    ASSERT_EQ(x4.id(), 3);
}
TYPED_TEST_P(LpModelTest, add_zero_variables) {
    auto model = this->new_model();
    auto x = model.add_variables(0);
    ASSERT_EQ(model.num_variables(), 0);
    ASSERT_EQ(model.num_constraints(), 0);
    ASSERT_THROW(x[0], std::out_of_range);
}
TYPED_TEST_P(LpModelTest, add_variables) {
    auto model = this->new_model();
    auto x = model.add_variables(1);
    auto y = model.add_variables(3);
    ASSERT_EQ(model.num_variables(), 4);
    ASSERT_EQ(model.num_constraints(), 0);
    ASSERT_EQ(x[0].id(), 0);
    ASSERT_EQ(y[0].id(), 1);
    ASSERT_EQ(y[1].id(), 2);
    ASSERT_EQ(y[2].id(), 3);
    ASSERT_THROW(y[3], std::out_of_range);
}
TYPED_TEST_P(LpModelTest, add_variables_params) {
    auto model = this->new_model();
    auto x = model.add_variables(1, {});
    auto y = model.add_variables(
        3, {.obj_coef = 1, .lower_bound = 2, .upper_bound = 10});
    ASSERT_EQ(model.num_variables(), 4);
    ASSERT_EQ(model.num_constraints(), 0);
    ASSERT_EQ(x[0].id(), 0);
    ASSERT_EQ(y[0].id(), 1);
    ASSERT_EQ(y[1].id(), 2);
    ASSERT_EQ(y[2].id(), 3);
    ASSERT_THROW(y[3], std::out_of_range);
}
TYPED_TEST_P(LpModelTest, add_variable_and_variables) {
    auto model = this->new_model();
    auto x = model.add_variable();
    auto y = model.add_variables(2);
    auto z = model.add_variable();
    ASSERT_EQ(model.num_variables(), 4);
    ASSERT_EQ(model.num_constraints(), 0);
    ASSERT_EQ(x.id(), 0);
    ASSERT_EQ(y[0].id(), 1);
    ASSERT_EQ(y[1].id(), 2);
    ASSERT_EQ(z.id(), 3);
}
TYPED_TEST_P(LpModelTest, add_zero_indexed_variables) {
    auto model = this->new_model();
    auto x = model.add_variables(0, [](int i, int j) { return i + j; });
    ASSERT_EQ(model.num_variables(), 0);
    ASSERT_EQ(model.num_constraints(), 0);
    ASSERT_THROW(x(0, 0), std::out_of_range);
}
TYPED_TEST_P(LpModelTest, add_indexed_variables) {
    auto model = this->new_model();
    auto x = model.add_variables(1, [](int i) { return i; });
    auto y = model.add_variables(3, [](int i, int j) { return 2 * i + j; });
    ASSERT_EQ(model.num_variables(), 4);
    ASSERT_EQ(model.num_constraints(), 0);
    ASSERT_EQ(x(0).id(), 0);
    ASSERT_EQ(y(0, 0).id(), 1);
    ASSERT_EQ(y(0, 1).id(), 2);
    ASSERT_EQ(y(1, 0).id(), 3);
    ASSERT_THROW(y(0, -1), std::out_of_range);
    ASSERT_THROW(y(1, 1), std::out_of_range);
}
TYPED_TEST_P(LpModelTest, add_indexed_variables_params) {
    auto model = this->new_model();
    auto x = model.add_variables(1, [](int i) { return i; }, {});
    auto y = model.add_variables(
        3, [](int i, int j) { return 2 * i + j; },
        {.obj_coef = 1, .lower_bound = 2, .upper_bound = 10});
    ASSERT_EQ(model.num_variables(), 4);
    ASSERT_EQ(model.num_constraints(), 0);
    ASSERT_EQ(x(0).id(), 0);
    ASSERT_EQ(y(0, 0).id(), 1);
    ASSERT_EQ(y(0, 1).id(), 2);
    ASSERT_EQ(y(1, 0).id(), 3);
    ASSERT_THROW(y(0, -1), std::out_of_range);
    ASSERT_THROW(y(1, 1), std::out_of_range);
}
TYPED_TEST_P(LpModelTest, add_capturing_indexed_variables) {
    auto model = this->new_model();
    int side_effect = 0;
    auto x = model.add_variables(2, [&](int i) { return side_effect + i; });
    ASSERT_EQ(model.num_variables(), 2);
    ASSERT_EQ(model.num_constraints(), 0);
    ASSERT_EQ(x(0).id(), 0);
    ASSERT_EQ(x(1).id(), 1);
    side_effect = 1;
    ASSERT_EQ(x(0).id(), 1);
    side_effect = 2;
    ASSERT_THROW(x(0), std::out_of_range);
}
TYPED_TEST_P(LpModelTest, add_variable_and_indexed_variables) {
    auto model = this->new_model();
    auto x = model.add_variable();
    auto y = model.add_variables(2, [](int i, int j) { return i + j; });
    auto z = model.add_variable();
    ASSERT_EQ(model.num_variables(), 4);
    ASSERT_EQ(model.num_constraints(), 0);
    ASSERT_EQ(x.id(), 0);
    ASSERT_EQ(y(0, 0).id(), 1);
    ASSERT_EQ(y(0, 1).id(), 2);
    ASSERT_EQ(z.id(), 3);
}
TYPED_TEST_P(LpModelTest, set_objective) {
    using namespace operators;
    auto model = this->new_model();
    auto x = model.add_variable();
    auto y = model.add_variable();
    model.set_objective(x + y);
    ASSERT_EQ(model.num_variables(), 2);
    ASSERT_EQ(model.num_constraints(), 0);
}
TYPED_TEST_P(LpModelTest, add_constraint) {
    using namespace operators;
    auto model = this->new_model();
    auto x = model.add_variable();
    auto y = model.add_variable();
    auto c1 = model.add_constraint(x + y >= 1);
    auto c2 = model.add_constraint(x + y == 2);
    auto c3 = model.add_constraint(x + y <= 3);
    ASSERT_EQ(model.num_variables(), 2);
    ASSERT_EQ(model.num_constraints(), 3);
    ASSERT_EQ(c1.id(), 0);
    ASSERT_EQ(c2.id(), 1);
    ASSERT_EQ(c3.id(), 2);
}
TYPED_TEST_P(LpModelTest, add_constraints) {
    using namespace operators;
    auto model = this->new_model();
    auto x = model.add_variable();
    auto y = model.add_variable();
    auto c1 = model.add_constraint(x + y >= 1);
    auto c = model.add_constraints(std::views::iota(0, 3), [&](auto i) {
        return (3 - i) * x + i * y <= 5;
    });
    ASSERT_EQ(model.num_variables(), 2);
    ASSERT_EQ(model.num_constraints(), 4);
    ASSERT_EQ(c1.id(), 0);
    ASSERT_EQ(c(0).id(), 1);
    ASSERT_EQ(c(1).id(), 2);
    ASSERT_EQ(c(2).id(), 3);
    ASSERT_THROW(c(-1), std::out_of_range);
    ASSERT_THROW(c(3), std::out_of_range);
}
TYPED_TEST_P(LpModelTest, add_opt_constraints) {
    using namespace operators;
    auto model = this->new_model();
    auto x = model.add_variable();
    auto y = model.add_variable();
    auto c1 = model.add_constraint(x + y >= 1);
    auto c = model.add_constraints(
        std::views::iota(0, 3),
        [&](auto i) { return OPT((i == 0), 3 * x <= 5); },
        [&](auto i) { return OPT((i == 1), 2 * x + y <= 5); },
        [&](auto i) { return x + 2 * y <= 5; });
    ASSERT_EQ(model.num_variables(), 2);
    ASSERT_EQ(model.num_constraints(), 4);
    ASSERT_EQ(c1.id(), 0);
    ASSERT_EQ(c(0).id(), 1);
    ASSERT_EQ(c(1).id(), 2);
    ASSERT_EQ(c(2).id(), 3);
    ASSERT_THROW(c(-1), std::out_of_range);
    ASSERT_THROW(c(3), std::out_of_range);
}
TYPED_TEST_P(LpModelTest, solve_empty_no_sense) {
    auto model = this->new_model();
    model.solve();
    ASSERT_DOUBLE_EQ(model.get_solution_value(), 0.0);
    auto solution = model.get_solution();
}
TYPED_TEST_P(LpModelTest, solve_empty_max) {
    auto model = this->new_model();
    model.set_maximization();
    model.solve();
    ASSERT_DOUBLE_EQ(model.get_solution_value(), 0.0);
    auto solution = model.get_solution();
}
TYPED_TEST_P(LpModelTest, solve_empty_min) {
    auto model = this->new_model();
    model.set_minimization();
    model.solve();
    ASSERT_DOUBLE_EQ(model.get_solution_value(), 0.0);
    auto solution = model.get_solution();
}
TYPED_TEST_P(LpModelTest, solve_bounded_variables_max) {
    using namespace operators;
    auto model = this->new_model();
    auto x = model.add_variable({.upper_bound = 3});
    auto y = model.add_variable({.lower_bound = 1});
    model.set_maximization();
    model.set_objective(-2 * x - y);
    auto z = model.add_variable({.obj_coef = 2, .upper_bound = 1});
    model.add_constraint(x >= -2);
    model.add_constraint(x - y <= 5);
    model.solve();
    ASSERT_DOUBLE_EQ(model.get_solution_value(), 5.0);
    auto solution = model.get_solution();
    ASSERT_DOUBLE_EQ(solution[x], -2.0);
    ASSERT_DOUBLE_EQ(solution[y], 1.0);
    ASSERT_DOUBLE_EQ(solution[z], 1.0);
}
TYPED_TEST_P(LpModelTest, solve_bounded_variables_min) {
    using namespace operators;
    auto model = this->new_model();
    auto x = model.add_variable({.upper_bound = 3});
    auto y = model.add_variable({.lower_bound = 1});
    model.set_minimization();
    model.set_objective(2 * x + y);
    auto z = model.add_variable({.obj_coef = -2, .upper_bound = 1});
    model.add_constraint(x >= -2);
    model.add_constraint(x - y <= 5);
    model.solve();
    ASSERT_DOUBLE_EQ(model.get_solution_value(), -5.0);
    auto solution = model.get_solution();
    ASSERT_DOUBLE_EQ(solution[x], -2.0);
    ASSERT_DOUBLE_EQ(solution[y], 1.0);
    ASSERT_DOUBLE_EQ(solution[z], 1.0);
}
TYPED_TEST_P(LpModelTest, solve_lp) {
    using namespace operators;
    auto model = this->new_model();
    auto x1 = model.add_variable();
    auto x2 = model.add_variable();
    auto x3 = model.add_variable();
    model.set_maximization();
    model.set_objective(5 * x1 + 4 * x2 + 3 * x3);
    auto c1 = model.add_constraint(2 * x1 + 3 * x2 + x3 <= 5);
    auto c2 = model.add_constraint(4 * x1 + x2 + 2 * x3 <= 11);
    auto c3 = model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 <= 8);
    model.solve();
    ASSERT_NEAR(model.get_solution_value(), 13.0, TEST_EPSILON);
    auto solution = model.get_solution();
    ASSERT_NEAR(solution[x1], 2.0, TEST_EPSILON);
    ASSERT_NEAR(solution[x2], 0.0, TEST_EPSILON);
    ASSERT_NEAR(solution[x3], 1.0, TEST_EPSILON);
}
TYPED_TEST_P(LpModelTest, solve_lp_add_constraints) {
    using namespace operators;
    auto model = this->new_model();
    auto x1 = model.add_variable();
    auto x2 = model.add_variable();
    auto x3 = model.add_variable();
    model.set_maximization();
    model.set_objective(5 * x1 + 4 * x2 + 3 * x3);
    auto c1 = model.add_constraints(
        std::views::iota(0, 3),
        [&](int i) { return OPT((i == 0), 2 * x1 + 3 * x2 + x3 <= 5); },
        [&](int i) { return OPT((i == 1), 4 * x1 + x2 + 2 * x3 <= 11); },
        [&](int i) { return 3 * x1 + 4 * x2 + 2 * x3 <= 8; });
    model.solve();
    ASSERT_NEAR(model.get_solution_value(), 13.0, TEST_EPSILON);
    auto solution = model.get_solution();
    ASSERT_NEAR(solution[x1], 2.0, TEST_EPSILON);
    ASSERT_NEAR(solution[x2], 0.0, TEST_EPSILON);
    ASSERT_NEAR(solution[x3], 1.0, TEST_EPSILON);
}
TYPED_TEST_P(LpModelTest, solve_lp_with_objective_offset_min) {
    using namespace operators;
    auto model = this->new_model();
    auto x1 = model.add_variable();
    auto x2 = model.add_variable();
    auto x3 = model.add_variable();
    model.set_minimization();
    model.set_objective(15 - 5 * x1 - 4 * x2 - 3 * x3);
    auto c1 = model.add_constraint(2 * x1 + 3 * x2 + x3 <= 5);
    auto c2 = model.add_constraint(4 * x1 + x2 + 2 * x3 <= 11);
    auto c3 = model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 <= 8);
    model.solve();
    ASSERT_NEAR(model.get_solution_value(), 2.0, TEST_EPSILON);
    auto solution = model.get_solution();
    ASSERT_NEAR(solution[x1], 2.0, TEST_EPSILON);
    ASSERT_NEAR(solution[x2], 0.0, TEST_EPSILON);
    ASSERT_NEAR(solution[x3], 1.0, TEST_EPSILON);
}
TYPED_TEST_P(LpModelTest, solve_lp_with_objective_offset_max) {
    using namespace operators;
    auto model = this->new_model();
    auto x1 = model.add_variable();
    auto x2 = model.add_variable();
    auto x3 = model.add_variable();
    model.set_maximization();
    model.set_objective(15 + 5 * x1 + 4 * x2 + 3 * x3);
    auto c1 = model.add_constraint(2 * x1 + 3 * x2 + x3 <= 5);
    auto c2 = model.add_constraint(4 * x1 + x2 + 2 * x3 <= 11);
    auto c3 = model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 <= 8);
    model.solve();
    ASSERT_NEAR(model.get_solution_value(), 28.0, TEST_EPSILON);
    auto solution = model.get_solution();
    ASSERT_NEAR(solution[x1], 2.0, TEST_EPSILON);
    ASSERT_NEAR(solution[x2], 0.0, TEST_EPSILON);
    ASSERT_NEAR(solution[x3], 1.0, TEST_EPSILON);
}
TYPED_TEST_P(LpModelTest, solve_lp_set_objective_offset) {
    using namespace operators;
    auto model = this->new_model();
    auto x1 = model.add_variable();
    auto x2 = model.add_variable();
    auto x3 = model.add_variable();
    model.set_minimization();
    model.set_objective(15 - 5 * x1 - 4 * x2 - 3 * x3);
    model.set_objective_offset(9);
    auto c1 = model.add_constraint(2 * x1 + 3 * x2 + x3 <= 5);
    auto c2 = model.add_constraint(4 * x1 + x2 + 2 * x3 <= 11);
    auto c3 = model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 <= 8);
    model.solve();
    ASSERT_NEAR(model.get_solution_value(), -4.0, TEST_EPSILON);
    auto solution = model.get_solution();
    ASSERT_NEAR(solution[x1], 2.0, TEST_EPSILON);
    ASSERT_NEAR(solution[x2], 0.0, TEST_EPSILON);
    ASSERT_NEAR(solution[x3], 1.0, TEST_EPSILON);
}
TYPED_TEST_P(LpModelTest, solve_lp_objective_redundant_terms) {
    using namespace operators;
    auto model = this->new_model();
    auto x1 = model.add_variable();
    auto x2 = model.add_variable();
    auto x3 = model.add_variable();
    model.set_maximization();
    model.set_objective(2 * x1 + 4 * x2 + 3 * x1 + 3 * x3);
    auto c1 = model.add_constraint(2 * x1 + 3 * x2 + x3 <= 5);
    auto c2 = model.add_constraint(4 * x1 + x2 + 2 * x3 <= 11);
    auto c3 = model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 <= 8);
    model.solve();
    ASSERT_NEAR(model.get_solution_value(), 13.0, TEST_EPSILON);
    auto solution = model.get_solution();
    ASSERT_NEAR(solution[x1], 2.0, TEST_EPSILON);
    ASSERT_NEAR(solution[x2], 0.0, TEST_EPSILON);
    ASSERT_NEAR(solution[x3], 1.0, TEST_EPSILON);
}
TYPED_TEST_P(LpModelTest, solve_lp_constraint_redundant_terms) {
    using namespace operators;
    auto model = this->new_model();
    auto x1 = model.add_variable();
    auto x2 = model.add_variable();
    auto x3 = model.add_variable();
    model.set_maximization();
    model.set_objective(5 * x1 + 4 * x2 + 3 * x3);
    auto c1 = model.add_constraint(3 * x1 - x1 + 3 * x2 + x3 <= 5);
    auto c2 = model.add_constraint(4 * x1 + x3 + x2 + x3 <= 11);
    auto c3 = model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 <= 8);
    model.solve();
    ASSERT_NEAR(model.get_solution_value(), 13.0, TEST_EPSILON);
    auto solution = model.get_solution();
    ASSERT_NEAR(solution[x1], 2.0, TEST_EPSILON);
    ASSERT_NEAR(solution[x2], 0.0, TEST_EPSILON);
    ASSERT_NEAR(solution[x3], 1.0, TEST_EPSILON);
}
TYPED_TEST_P(LpModelTest, solve_lp_non_standard_form_max) {
    using namespace operators;
    auto model = this->new_model();
    auto x1 = model.add_variable();
    auto x2 = model.add_variable();
    auto x3 = model.add_variable();
    model.set_maximization();
    model.set_objective(5 * x1 + 4 * x2 + 3 * x3);
    auto c1 = model.add_constraint(2 * x1 + 2 * x2 - x3 >= 5);
    auto c2 = model.add_constraint(4 * x1 + x2 + 2 * x3 <= 11);
    auto c3 = model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 == 8);
    model.solve();
    ASSERT_NEAR(model.get_solution_value(), 13.3333333333, TEST_EPSILON);
    auto solution = model.get_solution();
    ASSERT_NEAR(solution[x1], 2.6666666666, TEST_EPSILON);
    ASSERT_NEAR(solution[x2], 0.0, TEST_EPSILON);
    ASSERT_NEAR(solution[x3], 0.0, TEST_EPSILON);
}
TYPED_TEST_P(LpModelTest, solve_lp_non_standard_form_min) {
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
    auto solution = model.get_solution();
    ASSERT_NEAR(solution[x1], 2.0, TEST_EPSILON);
    ASSERT_NEAR(solution[x2], 0.5, TEST_EPSILON);
    ASSERT_NEAR(solution[x3], 0.0, TEST_EPSILON);
}

REGISTER_TYPED_TEST_SUITE_P(
    LpModelTest, construct, add_variable, add_variable_params,
    add_zero_variables, add_variables, add_variables_params,
    add_variable_and_variables, add_zero_indexed_variables,
    add_indexed_variables, add_indexed_variables_params,
    add_capturing_indexed_variables, add_variable_and_indexed_variables,
    set_objective, add_constraint, add_constraints, add_opt_constraints,
    solve_empty_no_sense, solve_empty_max, solve_empty_min,
    solve_bounded_variables_max, solve_bounded_variables_min, solve_lp,
    solve_lp_add_constraints, solve_lp_with_objective_offset_min,
    solve_lp_with_objective_offset_max, solve_lp_set_objective_offset,
    solve_lp_objective_redundant_terms, solve_lp_constraint_redundant_terms,
    solve_lp_non_standard_form_max, solve_lp_non_standard_form_min);

}  // namespace fhamonic::mippp