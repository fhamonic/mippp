#pragma once
#undef NDEBUG
#include <gtest/gtest.h>

#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"

namespace fhamonic::mippp {

template <typename T>
struct MilpModelTest : public T {
    using typename T::model_type;
    static_assert(milp_model<model_type>);
};
TYPED_TEST_SUITE_P(MilpModelTest);

TYPED_TEST_P(MilpModelTest, add_integer_variable) {
    auto model = this->new_model();
    auto x1 = model.add_integer_variable();
    auto x2 = model.add_integer_variable();
    ASSERT_EQ(model.num_variables(), 2);
    ASSERT_EQ(model.num_constraints(), 0);
    ASSERT_EQ(x1.id(), 0);
    ASSERT_EQ(x2.id(), 1);
}
TYPED_TEST_P(MilpModelTest, add_integer_variable_params) {
    auto model = this->new_model();
    auto x1 = model.add_integer_variable({});
    auto x2 = model.add_integer_variable(
        {.obj_coef = 1, .lower_bound = 2, .upper_bound = 10});
    auto x3 = model.add_integer_variable({.obj_coef = 1, .lower_bound = 2});
    auto x4 = model.add_integer_variable({.upper_bound = 10});
    ASSERT_EQ(model.num_variables(), 4);
    ASSERT_EQ(model.num_constraints(), 0);
    ASSERT_EQ(x1.id(), 0);
    ASSERT_EQ(x2.id(), 1);
    ASSERT_EQ(x3.id(), 2);
    ASSERT_EQ(x4.id(), 3);
}
TYPED_TEST_P(MilpModelTest, add_zero_integer_variables) {
    auto model = this->new_model();
    auto x = model.add_integer_variables(0);
    ASSERT_EQ(model.num_variables(), 0);
    ASSERT_EQ(model.num_constraints(), 0);
    ASSERT_THROW(x[0], std::out_of_range);
}
TYPED_TEST_P(MilpModelTest, add_integer_variables) {
    auto model = this->new_model();
    auto x = model.add_integer_variables(1);
    auto y = model.add_integer_variables(3);
    ASSERT_EQ(model.num_variables(), 4);
    ASSERT_EQ(model.num_constraints(), 0);
    ASSERT_EQ(x[0].id(), 0);
    ASSERT_EQ(y[0].id(), 1);
    ASSERT_EQ(y[1].id(), 2);
    ASSERT_EQ(y[2].id(), 3);
    ASSERT_THROW(y[3], std::out_of_range);
}
TYPED_TEST_P(MilpModelTest, add_integer_variables_params) {
    auto model = this->new_model();
    auto x = model.add_integer_variables(1, {});
    auto y = model.add_integer_variables(
        3, {.obj_coef = 1, .lower_bound = 2, .upper_bound = 10});
    ASSERT_EQ(model.num_variables(), 4);
    ASSERT_EQ(model.num_constraints(), 0);
    ASSERT_EQ(x[0].id(), 0);
    ASSERT_EQ(y[0].id(), 1);
    ASSERT_EQ(y[1].id(), 2);
    ASSERT_EQ(y[2].id(), 3);
    ASSERT_THROW(y[3], std::out_of_range);
}
TYPED_TEST_P(MilpModelTest, add_integer_variable_and_variables) {
    auto model = this->new_model();
    auto x = model.add_integer_variable();
    auto y = model.add_integer_variables(2);
    auto z = model.add_integer_variable();
    ASSERT_EQ(model.num_variables(), 4);
    ASSERT_EQ(model.num_constraints(), 0);
    ASSERT_EQ(x.id(), 0);
    ASSERT_EQ(y[0].id(), 1);
    ASSERT_EQ(y[1].id(), 2);
    ASSERT_EQ(z.id(), 3);
}
TYPED_TEST_P(MilpModelTest, add_zero_indexed_integer_variables) {
    auto model = this->new_model();
    auto x = model.add_integer_variables(0, [](int i, int j) { return i + j; });
    ASSERT_EQ(model.num_variables(), 0);
    ASSERT_EQ(model.num_constraints(), 0);
    ASSERT_THROW(x(0, 0), std::out_of_range);
}
TYPED_TEST_P(MilpModelTest, add_indexed_integer_variables) {
    auto model = this->new_model();
    auto x = model.add_integer_variables(1, [](int i) { return i; });
    auto y =
        model.add_integer_variables(3, [](int i, int j) { return 2 * i + j; });
    ASSERT_EQ(model.num_variables(), 4);
    ASSERT_EQ(model.num_constraints(), 0);
    ASSERT_EQ(x(0).id(), 0);
    ASSERT_EQ(y(0, 0).id(), 1);
    ASSERT_EQ(y(0, 1).id(), 2);
    ASSERT_EQ(y(1, 0).id(), 3);
    ASSERT_THROW(y(0, -1), std::out_of_range);
    ASSERT_THROW(y(1, 1), std::out_of_range);
}
TYPED_TEST_P(MilpModelTest, add_indexed_integer_variables_params) {
    auto model = this->new_model();
    auto x = model.add_integer_variables(1, [](int i) { return i; }, {});
    auto y = model.add_integer_variables(
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
TYPED_TEST_P(MilpModelTest, add_capturing_indexed_integer_variables) {
    auto model = this->new_model();
    int side_effect = 0;
    auto x =
        model.add_integer_variables(2, [&](int i) { return side_effect + i; });
    ASSERT_EQ(model.num_variables(), 2);
    ASSERT_EQ(model.num_constraints(), 0);
    ASSERT_EQ(x(0).id(), 0);
    ASSERT_EQ(x(1).id(), 1);
    side_effect = 1;
    ASSERT_EQ(x(0).id(), 1);
    side_effect = 2;
    ASSERT_THROW(x(0), std::out_of_range);
}
TYPED_TEST_P(MilpModelTest,
             add_integer_variable_and_indexed_integer_variables) {
    auto model = this->new_model();
    auto x = model.add_integer_variable();
    auto y = model.add_integer_variables(2, [](int i, int j) { return i + j; });
    auto z = model.add_integer_variable();
    ASSERT_EQ(model.num_variables(), 4);
    ASSERT_EQ(model.num_constraints(), 0);
    ASSERT_EQ(x.id(), 0);
    ASSERT_EQ(y(0, 0).id(), 1);
    ASSERT_EQ(y(0, 1).id(), 2);
    ASSERT_EQ(z.id(), 3);
}
TYPED_TEST_P(MilpModelTest, solve_bounded_integer_variables_max) {
    using namespace operators;
    auto model = this->new_model();
    auto x = model.add_integer_variable({.upper_bound = 3});
    auto y = model.add_integer_variable({.lower_bound = 1});
    model.set_maximization();
    model.set_objective(-2 * x - y);
    auto z = model.add_integer_variable({.obj_coef = 2, .upper_bound = 1});
    model.add_constraint(x >= -2);
    model.add_constraint(x - y <= 5);
    model.solve();
    ASSERT_DOUBLE_EQ(model.get_solution_value(), 5.0);
    auto solution = model.get_solution();
    ASSERT_DOUBLE_EQ(solution[x], -2.0);
    ASSERT_DOUBLE_EQ(solution[y], 1.0);
    ASSERT_DOUBLE_EQ(solution[z], 1.0);
}
TYPED_TEST_P(MilpModelTest, solve_bounded_integer_variables_min) {
    using namespace operators;
    auto model = this->new_model();
    auto x = model.add_integer_variable({.upper_bound = 3});
    auto y = model.add_integer_variable({.lower_bound = 1});
    model.set_minimization();
    model.set_objective(2 * x + y);
    auto z = model.add_integer_variable({.obj_coef = -2, .upper_bound = 1});
    model.add_constraint(x >= -2);
    model.add_constraint(x - y <= 5);
    model.solve();
    ASSERT_DOUBLE_EQ(model.get_solution_value(), -5.0);
    auto solution = model.get_solution();
    ASSERT_DOUBLE_EQ(solution[x], -2.0);
    ASSERT_DOUBLE_EQ(solution[y], 1.0);
    ASSERT_DOUBLE_EQ(solution[z], 1.0);
}
TYPED_TEST_P(MilpModelTest, solve_unbounded_knapsack) {
    using namespace operators;
    std::vector<double> values = {50.0, 120.0, 150.0, 210.0, 240.0};
    std::vector<double> costs = {10.0, 20.0, 30.0, 40.0, 50.0};
    const auto N = values.size();
    const auto items = ranges::views::iota(0u, N);
    const double budget = 50.0;
    auto model = this->new_model();
    auto x = model.add_integer_variables(N);
    model.set_maximization();
    model.set_objective(xsum(items, [&](auto i) { return values[i] * x[i]; }));
    model.add_constraint(xsum(items, [&](auto i) { return costs[i] * x[i]; }) <=
                         budget);
    model.solve();
    ASSERT_DOUBLE_EQ(model.get_solution_value(), 290.0);
    auto solution = model.get_solution();
    ASSERT_DOUBLE_EQ(solution[x[0]], 1.0);
    ASSERT_DOUBLE_EQ(solution[x[1]], 2.0);
    ASSERT_DOUBLE_EQ(solution[x[2]], 0.0);
    ASSERT_DOUBLE_EQ(solution[x[3]], 0.0);
    ASSERT_DOUBLE_EQ(solution[x[4]], 0.0);
}
TYPED_TEST_P(MilpModelTest, add_binary_variable) {
    auto model = this->new_model();
    auto x1 = model.add_binary_variable();
    auto x2 = model.add_binary_variable();
    ASSERT_EQ(model.num_variables(), 2);
    ASSERT_EQ(model.num_constraints(), 0);
    ASSERT_EQ(x1.id(), 0);
    ASSERT_EQ(x2.id(), 1);
}
TYPED_TEST_P(MilpModelTest, add_zero_binary_variables) {
    auto model = this->new_model();
    auto x = model.add_binary_variables(0);
    ASSERT_EQ(model.num_variables(), 0);
    ASSERT_EQ(model.num_constraints(), 0);
    ASSERT_THROW(x[0], std::out_of_range);
}
TYPED_TEST_P(MilpModelTest, add_binary_variables) {
    auto model = this->new_model();
    auto x = model.add_binary_variables(1);
    auto y = model.add_binary_variables(3);
    ASSERT_EQ(model.num_variables(), 4);
    ASSERT_EQ(model.num_constraints(), 0);
    ASSERT_EQ(x[0].id(), 0);
    ASSERT_EQ(y[0].id(), 1);
    ASSERT_EQ(y[1].id(), 2);
    ASSERT_EQ(y[2].id(), 3);
    ASSERT_THROW(y[3], std::out_of_range);
}
TYPED_TEST_P(MilpModelTest, add_binary_variable_and_variables) {
    auto model = this->new_model();
    auto x = model.add_binary_variable();
    auto y = model.add_binary_variables(2);
    auto z = model.add_binary_variable();
    ASSERT_EQ(model.num_variables(), 4);
    ASSERT_EQ(model.num_constraints(), 0);
    ASSERT_EQ(x.id(), 0);
    ASSERT_EQ(y[0].id(), 1);
    ASSERT_EQ(y[1].id(), 2);
    ASSERT_EQ(z.id(), 3);
}
TYPED_TEST_P(MilpModelTest, add_zero_indexed_binary_variables) {
    auto model = this->new_model();
    auto x = model.add_binary_variables(0, [](int i, int j) { return i + j; });
    ASSERT_EQ(model.num_variables(), 0);
    ASSERT_EQ(model.num_constraints(), 0);
    ASSERT_THROW(x(0, 0), std::out_of_range);
}
TYPED_TEST_P(MilpModelTest, add_indexed_binary_variables) {
    auto model = this->new_model();
    auto x = model.add_binary_variables(1, [](int i) { return i; });
    auto y =
        model.add_binary_variables(3, [](int i, int j) { return 2 * i + j; });
    ASSERT_EQ(model.num_variables(), 4);
    ASSERT_EQ(model.num_constraints(), 0);
    ASSERT_EQ(x(0).id(), 0);
    ASSERT_EQ(y(0, 0).id(), 1);
    ASSERT_EQ(y(0, 1).id(), 2);
    ASSERT_EQ(y(1, 0).id(), 3);
    ASSERT_THROW(y(0, -1), std::out_of_range);
    ASSERT_THROW(y(1, 1), std::out_of_range);
}
TYPED_TEST_P(MilpModelTest, add_capturing_indexed_binary_variables) {
    auto model = this->new_model();
    int side_effect = 0;
    auto x =
        model.add_binary_variables(2, [&](int i) { return side_effect + i; });
    ASSERT_EQ(model.num_variables(), 2);
    ASSERT_EQ(model.num_constraints(), 0);
    ASSERT_EQ(x(0).id(), 0);
    ASSERT_EQ(x(1).id(), 1);
    side_effect = 1;
    ASSERT_EQ(x(0).id(), 1);
    side_effect = 2;
    ASSERT_THROW(x(0), std::out_of_range);
}
TYPED_TEST_P(MilpModelTest, solve_binary_knapsack) {
    using namespace operators;
    std::vector<double> values = {5.0, 3.0, 2.0, 7.0, 4.0};
    std::vector<double> costs = {2.0, 8.0, 4.0, 2.0, 5.0};
    const auto N = values.size();
    const auto items = ranges::views::iota(0u, N);
    const double budget = 10.0;
    auto model = this->new_model();
    auto x = model.add_binary_variables(N);
    model.set_maximization();
    model.set_objective(xsum(items, [&](auto i) { return values[i] * x[i]; }));
    model.add_constraint(xsum(items, [&](auto i) { return costs[i] * x[i]; }) <=
                         budget);
    model.solve();
    ASSERT_DOUBLE_EQ(model.get_solution_value(), 16.0);
    auto solution = model.get_solution();
    ASSERT_DOUBLE_EQ(solution[x[0]], 1.0);
    ASSERT_DOUBLE_EQ(solution[x[1]], 0.0);
    ASSERT_DOUBLE_EQ(solution[x[2]], 0.0);
    ASSERT_DOUBLE_EQ(solution[x[3]], 1.0);
    ASSERT_DOUBLE_EQ(solution[x[4]], 1.0);
}
TYPED_TEST_P(MilpModelTest, set_integer_to_continuous) {
    using namespace operators;
    auto model = this->new_model();
    auto x = model.add_integer_variable();
    auto y = model.add_integer_variable();
    model.set_maximization();
    model.set_objective(y);
    model.add_constraint(y <= 3.25 - x / 2);
    model.add_constraint(y <= 2 * x - 0.5);
    model.set_continuous(x);
    model.set_continuous(y);
    model.solve();
    ASSERT_DOUBLE_EQ(model.get_solution_value(), 2.5);
    auto solution = model.get_solution();
    ASSERT_DOUBLE_EQ(solution[x], 1.5);
    ASSERT_DOUBLE_EQ(solution[y], 2.5);
}
TYPED_TEST_P(MilpModelTest, set_continuous_to_integer) {
    using namespace operators;
    auto model = this->new_model();
    auto x = model.add_variable();
    auto y = model.add_variable();
    model.set_maximization();
    model.set_objective(y);
    model.add_constraint(y <= 3.25 - x / 2);
    model.add_constraint(y <= 2 * x - 0.5);
    model.set_integer(x);
    model.set_integer(y);
    model.solve();
    ASSERT_DOUBLE_EQ(model.get_solution_value(), 2.0);
    auto solution = model.get_solution();
    ASSERT_DOUBLE_EQ(solution[x], 2.0);
    ASSERT_DOUBLE_EQ(solution[y], 2.0);
}
TYPED_TEST_P(MilpModelTest, set_continuous_to_binary) {
    using namespace operators;
    auto model = this->new_model();
    auto x = model.add_variable();
    auto y = model.add_variable();
    model.set_maximization();
    model.set_objective(y);
    model.add_constraint(y <= 3.25 - x / 2);
    model.add_constraint(y <= 2 * x - 0.5);
    model.set_binary(x);
    model.set_binary(y);
    model.solve();
    ASSERT_DOUBLE_EQ(model.get_solution_value(), 1.0);
    auto solution = model.get_solution();
    ASSERT_DOUBLE_EQ(solution[x], 1.0);
    ASSERT_DOUBLE_EQ(solution[y], 1.0);
}
TYPED_TEST_P(MilpModelTest, set_binary_to_continuous) {
    using namespace operators;
    std::vector<double> values = {5.0, 3.0, 2.0, 7.0, 4.0};
    std::vector<double> costs = {2.0, 8.0, 4.0, 2.0, 5.0};
    const auto N = values.size();
    const auto items = ranges::views::iota(0u, N);
    const double budget = 10.0;
    auto model = this->new_model();
    auto x = model.add_binary_variables(N);
    model.set_maximization();
    model.set_objective(xsum(items, [&](auto i) { return values[i] * x[i]; }));
    model.add_constraint(xsum(items, [&](auto i) { return costs[i] * x[i]; }) <=
                         budget);
    for(auto && var : x) model.set_continuous(var);
    model.solve();
    ASSERT_DOUBLE_EQ(model.get_solution_value(), 16.5);
    auto solution = model.get_solution();
    ASSERT_DOUBLE_EQ(solution[x[0]], 1.0);
    ASSERT_DOUBLE_EQ(solution[x[1]], 0.0);
    ASSERT_DOUBLE_EQ(solution[x[2]], 0.25);
    ASSERT_DOUBLE_EQ(solution[x[3]], 1.0);
    ASSERT_DOUBLE_EQ(solution[x[4]], 1.0);
}

REGISTER_TYPED_TEST_SUITE_P(
    MilpModelTest, add_integer_variable, add_integer_variable_params,
    add_zero_integer_variables, add_integer_variables,
    add_integer_variables_params, add_integer_variable_and_variables,
    add_zero_indexed_integer_variables, add_indexed_integer_variables,
    add_indexed_integer_variables_params,
    add_capturing_indexed_integer_variables,
    add_integer_variable_and_indexed_integer_variables,
    solve_bounded_integer_variables_max, solve_bounded_integer_variables_min,
    solve_unbounded_knapsack, add_binary_variable, add_zero_binary_variables,
    add_binary_variables, add_binary_variable_and_variables,
    add_zero_indexed_binary_variables, add_indexed_binary_variables,
    add_capturing_indexed_binary_variables, solve_binary_knapsack,
    set_integer_to_continuous, set_continuous_to_integer,
    set_continuous_to_binary, set_binary_to_continuous);

}  // namespace fhamonic::mippp