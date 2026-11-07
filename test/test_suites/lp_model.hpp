#pragma once

#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <forward_list>
#include <functional>
#include <ranges>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "mippp/detail/cartesian_product_view.hpp"
#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/utility/solver_exceptions.hpp"

namespace mippp {

template <typename T>
struct LpModelTest : public T {
    using typename T::model_type;
    static_assert(lp_model<model_type>);
    static_assert(constraints_range<std::vector<model_constraint_t<model_type>>,
                                    model_type>);
    static_assert(
        !constraints_range<std::forward_list<model_constraint_t<model_type>>,
                           model_type>);
    static_assert(!constraints_range<std::vector<model_variable_t<model_type>>,
                                     model_type>);
};
TYPED_TEST_SUITE_P(LpModelTest);
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(LpModelTest);

TYPED_TEST_P(LpModelTest, construct) {
    this->SkipOnLicenseError([this]() {
        auto model = this->new_model();
        ASSERT_EQ(model.num_variables(), 0);
        ASSERT_EQ(model.num_constraints(), 0);
    });
}
TYPED_TEST_P(LpModelTest, add_variable) {
    this->SkipOnLicenseError([this]() {
        auto model = this->new_model();
        auto x1 = model.add_variable();
        auto x2 = model.add_variable();
        ASSERT_EQ(model.num_variables(), 2);
        ASSERT_EQ(model.num_constraints(), 0);
        ASSERT_EQ(x1.id(), 0);
        ASSERT_EQ(x2.id(), 1);
    });
}
TYPED_TEST_P(LpModelTest, add_variable_params) {
    this->SkipOnLicenseError([this]() {
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
    });
}
TYPED_TEST_P(LpModelTest, add_zero_variables) {
    this->SkipOnLicenseError([this]() {
        auto model = this->new_model();
        auto x = model.add_variables(0);
        ASSERT_EQ(model.num_variables(), 0);
        ASSERT_EQ(model.num_constraints(), 0);
        ASSERT_THROW(x[0], std::out_of_range);
    });
}
TYPED_TEST_P(LpModelTest, add_variables) {
    this->SkipOnLicenseError([this]() {
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
    });
}
TYPED_TEST_P(LpModelTest, add_variables_params) {
    this->SkipOnLicenseError([this]() {
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
    });
}
TYPED_TEST_P(LpModelTest, add_variable_and_variables) {
    this->SkipOnLicenseError([this]() {
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
    });
}
TYPED_TEST_P(LpModelTest, add_zero_indexed_variables) {
    this->SkipOnLicenseError([this]() {
        auto model = this->new_model();
        auto x = model.add_variables(0, [](int i, int j) { return i + j; });
        ASSERT_EQ(model.num_variables(), 0);
        ASSERT_EQ(model.num_constraints(), 0);
        ASSERT_THROW(x(0, 0), std::out_of_range);
    });
}
TYPED_TEST_P(LpModelTest, add_indexed_variables) {
    this->SkipOnLicenseError([this]() {
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
    });
}
TYPED_TEST_P(LpModelTest, add_indexed_variables_params) {
    this->SkipOnLicenseError([this]() {
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
    });
}
TYPED_TEST_P(LpModelTest, add_capturing_indexed_variables) {
    this->SkipOnLicenseError([this]() {
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
    });
}
TYPED_TEST_P(LpModelTest, add_variable_and_indexed_variables) {
    this->SkipOnLicenseError([this]() {
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
    });
}
TYPED_TEST_P(LpModelTest, set_objective) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x = model.add_variable();
        auto y = model.add_variable();
        model.set_objective(x + y);
        ASSERT_EQ(model.num_variables(), 2);
        ASSERT_EQ(model.num_constraints(), 0);
    });
}
TYPED_TEST_P(LpModelTest, set_objective_distinct_variables) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x = model.add_variable();
        auto y = model.add_variable();
        model.set_objective(distinct_variables, x + y);
        ASSERT_EQ(model.num_variables(), 2);
        ASSERT_EQ(model.num_constraints(), 0);
    });
}
TYPED_TEST_P(LpModelTest, distinct_variables_repeat_is_asserted) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        // GTEST_FLAG, not GTEST_FLAG_SET: the recipe allows gtest 1.10
        ::testing::GTEST_FLAG(death_test_style) = "threadsafe";
        auto model = this->new_model();
        auto x = model.add_variable();
        auto y = model.add_variable();
        EXPECT_DEATH(
            model.add_constraint(distinct_variables, 2 * x + 3 * x + y <= 1),
            "appears twice");
        EXPECT_DEATH(
            model.add_constraints(distinct_variables, std::views::iota(0, 1),
                                  [&](int) { return 2 * x + 3 * x + y <= 1; }),
            "appears twice");
    });
}
TYPED_TEST_P(LpModelTest, add_constraint) {
    this->SkipOnLicenseError([this]() {
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
    });
}
TYPED_TEST_P(LpModelTest, add_constraint_distinct_variables) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x = model.add_variable();
        auto y = model.add_variable();
        auto c1 = model.add_constraint(distinct_variables, x + y >= 1);
        auto c2 = model.add_constraint(distinct_variables, x + y == 2);
        auto c3 = model.add_constraint(distinct_variables, x + y <= 3);
        ASSERT_EQ(model.num_variables(), 2);
        ASSERT_EQ(model.num_constraints(), 3);
        ASSERT_EQ(c1.id(), 0);
        ASSERT_EQ(c2.id(), 1);
        ASSERT_EQ(c3.id(), 2);
    });
}
TYPED_TEST_P(LpModelTest, add_constraints) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x = model.add_variable();
        auto y = model.add_variable();
        auto c1 = model.add_constraint(x + y >= 1);
        auto c = model.add_constraints(std::views::iota(0, 3), [&](auto i) {
            return (3 - i) * x + i * y <= 5;
        });
        static_assert(
            constraints_range<decltype(c), typename TestFixture::model_type>);
        static_assert(std::ranges::sized_range<decltype(c)>);
        ASSERT_EQ(c.size(), 3);
        ASSERT_EQ(c[1].id(), 2);
        ASSERT_EQ(model.num_variables(), 2);
        ASSERT_EQ(model.num_constraints(), 4);
        ASSERT_EQ(c1.id(), 0);
        ASSERT_EQ(c(0).id(), 1);
        ASSERT_EQ(c(1).id(), 2);
        ASSERT_EQ(c(2).id(), 3);
        ASSERT_THROW(c(-1), std::out_of_range);
        ASSERT_THROW(c(3), std::out_of_range);
    });
}
TYPED_TEST_P(LpModelTest, add_constraints_distinct_variables) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x = model.add_variable();
        auto y = model.add_variable();
        auto c1 = model.add_constraint(x + y >= 1);
        auto c = model.add_constraints(
            distinct_variables, std::views::iota(0, 3),
            [&](auto i) { return (3 - i) * x + i * y <= 5; });
        ASSERT_EQ(model.num_variables(), 2);
        ASSERT_EQ(model.num_constraints(), 4);
        ASSERT_EQ(c1.id(), 0);
        ASSERT_EQ(c(0).id(), 1);
        ASSERT_EQ(c(1).id(), 2);
        ASSERT_EQ(c(2).id(), 3);
        ASSERT_THROW(c(-1), std::out_of_range);
        ASSERT_THROW(c(3), std::out_of_range);
    });
}
TYPED_TEST_P(LpModelTest, add_opt_constraints) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x = model.add_variable();
        auto y = model.add_variable();
        auto c1 = model.add_constraint(x + y >= 1);
        auto c = model.add_constraints(
            std::views::iota(0, 3),
            [&](int i) { return OPT((i == 0), 3 * x <= 5); },
            [&](int i) { return OPT((i == 1), 2 * x + y <= 5); },
            [&](int) { return x + 2 * y <= 5; });
        ASSERT_EQ(model.num_variables(), 2);
        ASSERT_EQ(model.num_constraints(), 4);
        ASSERT_EQ(c1.id(), 0);
        ASSERT_EQ(c(0).id(), 1);
        ASSERT_EQ(c(1).id(), 2);
        ASSERT_EQ(c(2).id(), 3);
        ASSERT_THROW(c(-1), std::out_of_range);
        ASSERT_THROW(c(3), std::out_of_range);
    });
}
TYPED_TEST_P(LpModelTest, add_opt_constraints_distinct_variables) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x = model.add_variable();
        auto y = model.add_variable();
        auto c1 = model.add_constraint(x + y >= 1);
        auto c = model.add_constraints(
            distinct_variables, std::views::iota(0, 3),
            [&](int i) { return OPT((i == 0), 3 * x <= 5); },
            [&](int i) { return OPT((i == 1), 2 * x + y <= 5); },
            [&](int) { return x + 2 * y <= 5; });
        ASSERT_EQ(model.num_variables(), 2);
        ASSERT_EQ(model.num_constraints(), 4);
        ASSERT_EQ(c1.id(), 0);
        ASSERT_EQ(c(0).id(), 1);
        ASSERT_EQ(c(1).id(), 2);
        ASSERT_EQ(c(2).id(), 3);
        ASSERT_THROW(c(-1), std::out_of_range);
        ASSERT_THROW(c(3), std::out_of_range);
    });
}
TYPED_TEST_P(LpModelTest, add_variables_by_key) {
    this->SkipOnLicenseError([this]() {
        auto model = this->new_model();
        auto grid = model.add_variables(
            mippp::detail::cartesian_product(std::views::iota(0, 2),
                                             std::views::iota(0, 3)),
            {.obj_coef = 1});
        ASSERT_EQ(model.num_variables(), 6);
        ASSERT_EQ(grid(1, 2).id(), 5);
        ASSERT_EQ(grid(std::tuple{0, 1}).id(), 1);
        ASSERT_EQ(grid[4].id(), 4);
        ASSERT_THROW(grid(2, 0), std::out_of_range);
        std::vector<std::string> names = {"a", "bb"};
        auto named_keys = model.add_variables(names);
        ASSERT_EQ(named_keys("bb").id(), 7);
        ASSERT_THROW(named_keys("c"), std::out_of_range);
        auto tabled = model.add_variables(
            indexed(std::vector<int>{3, 1}, std::identity{}));
        ASSERT_EQ(tabled(3).id(), 8);
        ASSERT_EQ(tabled(1).id(), 9);
        ASSERT_THROW(tabled(2), std::out_of_range);
        // the id lambda may be generic
        auto generic = model.add_variables(2, [](auto i) { return i - 1; });
        ASSERT_EQ(generic(1).id(), 10);
        ASSERT_THROW(generic(0), std::out_of_range);
        ASSERT_EQ(model.num_variables(), 12);
    });
}
TYPED_TEST_P(LpModelTest, add_constraints_by_key) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x = model.add_variable();
        auto y = model.add_variable();
        auto cells = mippp::detail::cartesian_product(std::views::iota(0, 2),
                                                      std::views::iota(0, 3));
        auto grid = model.add_constraints(
            cells, [&](int i, int j) { return i * x + j * y <= 5; });
        ASSERT_EQ(model.num_constraints(), 6);
        ASSERT_EQ(grid(1, 2).id(), 5);
        ASSERT_EQ(grid(std::tuple{0, 1}).id(), 1);
        ASSERT_THROW(grid(2, 0), std::out_of_range);
        auto grid2 = model.add_constraints(cells, [&](auto && p) {
            auto [i, j] = p;
            return i * x - j * y >= -5;
        });
        ASSERT_EQ(model.num_constraints(), 12);
        ASSERT_EQ(grid2(1, 2).id(), 11);
        std::vector<std::string> names = {"a", "bb"};
        auto named = model.add_constraints(names, [&](const std::string & n) {
            return x + y <= static_cast<double>(n.size());
        });
        ASSERT_EQ(named("bb").id(), 13);
        ASSERT_THROW(named("c"), std::out_of_range);
        auto tabled = model.add_constraints(
            indexed(std::vector<int>{3, 1}, std::identity{}),
            [&](int k) { return k * x <= 1; });
        ASSERT_EQ(tabled(3).id(), 14);
        ASSERT_EQ(tabled(1).id(), 15);
        ASSERT_THROW(tabled(2), std::out_of_range);
    });
}
TYPED_TEST_P(LpModelTest, solve_empty_no_sense) {
    this->SkipOnLicenseError([this]() {
        auto model = this->new_model();
        model.solve();
        ASSERT_DOUBLE_EQ(model.get_solution_value(), 0.0);
        [[maybe_unused]] auto solution = model.get_solution();
    });
}
TYPED_TEST_P(LpModelTest, solve_empty_max) {
    this->SkipOnLicenseError([this]() {
        auto model = this->new_model();
        model.set_maximization();
        model.solve();
        ASSERT_DOUBLE_EQ(model.get_solution_value(), 0.0);
        [[maybe_unused]] auto solution = model.get_solution();
    });
}
TYPED_TEST_P(LpModelTest, solve_empty_min) {
    this->SkipOnLicenseError([this]() {
        auto model = this->new_model();
        model.set_minimization();
        model.solve();
        ASSERT_DOUBLE_EQ(model.get_solution_value(), 0.0);
        [[maybe_unused]] auto solution = model.get_solution();
    });
}
TYPED_TEST_P(LpModelTest, solve_bounded_variables_max) {
    this->SkipOnLicenseError([this]() {
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
    });
}
TYPED_TEST_P(LpModelTest, solve_bounded_variables_min) {
    this->SkipOnLicenseError([this]() {
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
    });
}
TYPED_TEST_P(LpModelTest, solve_lp) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x1 = model.add_variable();
        auto x2 = model.add_variable();
        auto x3 = model.add_variable();
        model.set_maximization();
        model.set_objective(5 * x1 + 4 * x2 + 3 * x3);
        model.add_constraint(2 * x1 + 3 * x2 + x3 <= 5);
        model.add_constraint(4 * x1 + x2 + 2 * x3 <= 11);
        model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 <= 8);
        model.solve();
        ASSERT_NEAR(model.get_solution_value(), 13.0, TEST_EPSILON);
        auto solution = model.get_solution();
        ASSERT_NEAR(solution[x1], 2.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x2], 0.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x3], 1.0, TEST_EPSILON);
    });
}
TYPED_TEST_P(LpModelTest, solve_lp_add_constraints) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x1 = model.add_variable();
        auto x2 = model.add_variable();
        auto x3 = model.add_variable();
        model.set_maximization();
        model.set_objective(5 * x1 + 4 * x2 + 3 * x3);
        model.add_constraints(
            std::views::iota(0, 3),
            [&](int i) { return OPT((i == 0), 2 * x1 + 3 * x2 + x3 <= 5); },
            [&](int i) { return OPT((i == 1), 4 * x1 + x2 + 2 * x3 <= 11); },
            [&](int) { return 3 * x1 + 4 * x2 + 2 * x3 <= 8; });
        model.solve();
        ASSERT_NEAR(model.get_solution_value(), 13.0, TEST_EPSILON);
        auto solution = model.get_solution();
        ASSERT_NEAR(solution[x1], 2.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x2], 0.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x3], 1.0, TEST_EPSILON);
    });
}
TYPED_TEST_P(LpModelTest, solve_lp_with_objective_offset_min) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x1 = model.add_variable();
        auto x2 = model.add_variable();
        auto x3 = model.add_variable();
        model.set_minimization();
        model.set_objective(15 - 5 * x1 - 4 * x2 - 3 * x3);
        model.add_constraint(2 * x1 + 3 * x2 + x3 <= 5);
        model.add_constraint(4 * x1 + x2 + 2 * x3 <= 11);
        model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 <= 8);
        model.solve();
        ASSERT_NEAR(model.get_solution_value(), 2.0, TEST_EPSILON);
        auto solution = model.get_solution();
        ASSERT_NEAR(solution[x1], 2.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x2], 0.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x3], 1.0, TEST_EPSILON);
    });
}
TYPED_TEST_P(LpModelTest, solve_lp_with_objective_offset_max) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x1 = model.add_variable();
        auto x2 = model.add_variable();
        auto x3 = model.add_variable();
        model.set_maximization();
        model.set_objective(15 + 5 * x1 + 4 * x2 + 3 * x3);
        model.add_constraint(2 * x1 + 3 * x2 + x3 <= 5);
        model.add_constraint(4 * x1 + x2 + 2 * x3 <= 11);
        model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 <= 8);
        model.solve();
        ASSERT_NEAR(model.get_solution_value(), 28.0, TEST_EPSILON);
        auto solution = model.get_solution();
        ASSERT_NEAR(solution[x1], 2.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x2], 0.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x3], 1.0, TEST_EPSILON);
    });
}
TYPED_TEST_P(LpModelTest, solve_lp_set_objective_offset) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x1 = model.add_variable();
        auto x2 = model.add_variable();
        auto x3 = model.add_variable();
        model.set_minimization();
        model.set_objective(15 - 5 * x1 - 4 * x2 - 3 * x3);
        model.set_objective_offset(9);
        model.add_constraint(2 * x1 + 3 * x2 + x3 <= 5);
        model.add_constraint(4 * x1 + x2 + 2 * x3 <= 11);
        model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 <= 8);
        model.solve();
        ASSERT_NEAR(model.get_solution_value(), -4.0, TEST_EPSILON);
        auto solution = model.get_solution();
        ASSERT_NEAR(solution[x1], 2.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x2], 0.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x3], 1.0, TEST_EPSILON);
    });
}
TYPED_TEST_P(LpModelTest, solve_lp_objective_redundant_terms) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x1 = model.add_variable();
        auto x2 = model.add_variable();
        auto x3 = model.add_variable();
        model.set_maximization();
        model.set_objective(2 * x1 + 4 * x2 + 3 * x1 + 3 * x3);
        model.add_constraint(2 * x1 + 3 * x2 + x3 <= 5);
        model.add_constraint(4 * x1 + x2 + 2 * x3 <= 11);
        model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 <= 8);
        model.solve();
        ASSERT_NEAR(model.get_solution_value(), 13.0, TEST_EPSILON);
        auto solution = model.get_solution();
        ASSERT_NEAR(solution[x1], 2.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x2], 0.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x3], 1.0, TEST_EPSILON);
    });
}
TYPED_TEST_P(LpModelTest, solve_lp_constraint_redundant_terms) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x1 = model.add_variable();
        auto x2 = model.add_variable();
        auto x3 = model.add_variable();
        model.set_maximization();
        model.set_objective(5 * x1 + 4 * x2 + 3 * x3);
        model.add_constraint(3 * x1 - x1 + 3 * x2 + x3 <= 5);
        model.add_constraint(4 * x1 + x3 + x2 + x3 <= 11);
        model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 <= 8);
        model.solve();
        ASSERT_NEAR(model.get_solution_value(), 13.0, TEST_EPSILON);
        auto solution = model.get_solution();
        ASSERT_NEAR(solution[x1], 2.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x2], 0.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x3], 1.0, TEST_EPSILON);
    });
}
// Every row is zero once merged, from cancelling terms or a written zero,
// through both entry points and both tag forms: only the bounds constrain
// the optimum. Clp 1.17.x reports every column at 0 when the only elements
// it stores are zeros, so a backend must not hand such rows to its solver.
TYPED_TEST_P(LpModelTest, solve_lp_zero_rows) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x1 =
            model.add_variable({.lower_bound = -5.0, .upper_bound = -2.0});
        auto x2 = model.add_variable({.lower_bound = 1.0, .upper_bound = 4.0});
        model.set_minimization();
        model.set_objective(-0.5 * x1 + x2 - 7.0);
        model.add_constraint(-0.5 * x1 + 0.5 * x1 <= 3);
        model.add_constraint(distinct_variables, 0.0 * x2 >= -1);
        model.add_constraints(
            std::views::iota(0, 2),
            [&](int i) { return OPT((i == 0), x1 - x1 + 0.0 * x2 == 0); },
            [&](int) { return 2 * x2 - x2 - x2 <= 1; });
        model.add_constraints(distinct_variables, std::views::iota(0, 1),
                              [&](int) { return 0.0 * x1 + 0.0 * x2 >= -2; });
        model.solve();
        ASSERT_NEAR(model.get_solution_value(), -5.0, TEST_EPSILON);
        auto solution = model.get_solution();
        ASSERT_NEAR(solution[x1], -2.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x2], 1.0, TEST_EPSILON);
    });
}
TYPED_TEST_P(LpModelTest, solve_lp_distinct_variables) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x1 = model.add_variable();
        auto x2 = model.add_variable();
        auto x3 = model.add_variable();
        model.set_maximization();
        model.set_objective(distinct_variables, 5 * x1 + 4 * x2 + 3 * x3);
        model.add_constraint(distinct_variables, 2 * x1 + 3 * x2 + x3 <= 5);
        model.add_constraints(
            distinct_variables, std::views::iota(0, 2),
            [&](int i) { return OPT((i == 0), 4 * x1 + x2 + 2 * x3 <= 11); },
            [&](int) { return 3 * x1 + 4 * x2 + 2 * x3 <= 8; });
        model.solve();
        ASSERT_NEAR(model.get_solution_value(), 13.0, TEST_EPSILON);
        auto solution = model.get_solution();
        ASSERT_NEAR(solution[x1], 2.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x2], 0.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x3], 1.0, TEST_EPSILON);
    });
}
// Interleaves both forms, and inserts a variable after a tagged call: the
// tagged path skips the entry cache the untagged one indexes by variable, so
// the untagged constraint that follows must still see a large enough cache.
TYPED_TEST_P(LpModelTest, solve_lp_mixed_distinct_variables) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x1 = model.add_variable();
        auto x2 = model.add_variable();
        auto x3 = model.add_variable();
        model.set_maximization();
        model.set_objective(5 * x1 + 4 * x2 + 3 * x3);
        model.add_constraint(2 * x1 + 3 * x2 + x3 <= 5);
        model.add_constraint(distinct_variables, 4 * x1 + x2 + 2 * x3 <= 11);
        auto x4 = model.add_variable({.obj_coef = 1, .upper_bound = 2});
        auto c3 = model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 <= 8);
        auto c4 = model.add_constraint(distinct_variables, x1 + x4 <= 10);
        auto c5 = model.add_constraint(x2 + x4 <= 10);
        ASSERT_EQ(model.num_variables(), 4);
        ASSERT_EQ(model.num_constraints(), 5);
        ASSERT_EQ(c3.id(), 2);
        ASSERT_EQ(c4.id(), 3);
        ASSERT_EQ(c5.id(), 4);
        model.solve();
        ASSERT_NEAR(model.get_solution_value(), 15.0, TEST_EPSILON);
        auto solution = model.get_solution();
        ASSERT_NEAR(solution[x1], 2.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x2], 0.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x3], 1.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x4], 2.0, TEST_EPSILON);
    });
}
TYPED_TEST_P(LpModelTest, solve_lp_non_standard_form_max) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x1 = model.add_variable();
        auto x2 = model.add_variable();
        auto x3 = model.add_variable();
        model.set_maximization();
        model.set_objective(5 * x1 + 4 * x2 + 3 * x3);
        model.add_constraint(2 * x1 + 2 * x2 - x3 >= 5);
        model.add_constraint(4 * x1 + x2 + 2 * x3 <= 11);
        model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 == 8);
        model.solve();
        ASSERT_NEAR(model.get_solution_value(), 13.3333333333, TEST_EPSILON);
        auto solution = model.get_solution();
        ASSERT_NEAR(solution[x1], 2.6666666666, TEST_EPSILON);
        ASSERT_NEAR(solution[x2], 0.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x3], 0.0, TEST_EPSILON);
    });
}
TYPED_TEST_P(LpModelTest, solve_lp_non_standard_form_min) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x1 = model.add_variable();
        auto x2 = model.add_variable();
        auto x3 = model.add_variable();
        model.set_minimization();
        model.set_objective(5 * x1 + 4 * x2 + 3 * x3);
        model.add_constraint(2 * x1 + 2 * x2 - x3 >= 5);
        model.add_constraint(4 * x1 + x2 + 2 * x3 <= 11);
        model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 == 8);
        model.solve();
        ASSERT_NEAR(model.get_solution_value(), 12.0, TEST_EPSILON);
        auto solution = model.get_solution();
        ASSERT_NEAR(solution[x1], 2.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x2], 0.5, TEST_EPSILON);
        ASSERT_NEAR(solution[x3], 0.0, TEST_EPSILON);
    });
}

TYPED_TEST_P(LpModelTest, move_preserves_status) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x = model.add_variable({.upper_bound = 3});
        auto y = model.add_variable({.lower_bound = 1});
        model.set_maximization();
        model.set_objective(2 * x + 1.5 * y);
        model.add_constraint(x + y <= 5);
        model.solve();
        ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
        const std::size_t solved = model.get_status().index();
        auto moved = std::move(model);
        ASSERT_EQ(moved.get_status().index(), solved);
        ASSERT_NEAR(moved.get_solution_value(), 9.0, TEST_EPSILON);
    });
}

TYPED_TEST_P(LpModelTest, reset_status_reports_unknown) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        static_assert(has_status_reset<typename TestFixture::model_type>);
        auto model = this->new_model();
        auto x = model.add_variable({.upper_bound = 3});
        auto y = model.add_variable();
        model.set_maximization();
        model.set_objective(x + y);
        model.add_constraint(x + 2 * y <= 4);
        model.solve();
        const auto solved = model.get_status();
        ASSERT_TRUE(is_a<status::optimal>(solved));
        model.reset_status();
        ASSERT_TRUE(is<status::unknown>(model.get_status()));
        ASSERT_FALSE(status::solution_available(model.get_status()));
        model.solve();
        ASSERT_EQ(model.get_status().index(), solved.index());
        ASSERT_NEAR(model.get_solution_value(), 3.5, TEST_EPSILON);
    });
}

template <typename M>
void expect_results_unreadable(M & model) {
    EXPECT_TRUE(is<status::unknown>(model.get_status()));
    EXPECT_THROW((void)model.get_solution_value(), solve_required_error);
    EXPECT_THROW((void)model.get_solution(), solve_required_error);
    if constexpr(has_dual_solution<M>) {
        EXPECT_THROW((void)model.get_dual_solution(), solve_required_error);
    }
    if constexpr(has_reduced_costs<M>) {
        EXPECT_THROW((void)model.get_reduced_costs(), solve_required_error);
    }
    if constexpr(has_lp_basis<M>) {
        EXPECT_THROW((void)model.get_basis(), solve_required_error);
    }
    if constexpr(has_dual_bound<M>) {
        EXPECT_THROW((void)model.get_dual_bound(), solve_required_error);
    }
    if constexpr(has_node_count<M>) {
        EXPECT_THROW((void)model.get_node_count(), solve_required_error);
    }
}

TYPED_TEST_P(LpModelTest, results_need_a_solve) {
    this->SkipOnLicenseError([this]() {
        auto model = this->new_model();
        model.add_variable({.upper_bound = 1});
        expect_results_unreadable(model);
    });
}

// Whether the solver kept its last solution or not: some drop it on a
// rename (SCIP) or on a basis or a MIP start (Gurobi, CPLEX, COPT, HiGHS,
// MOSEK), and the tests of each change must not assume which.
TYPED_TEST_P(LpModelTest, model_changes_invalidate_results) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        using M = typename TestFixture::model_type;
        using V = model_variable_t<M>;
        using C = model_constraint_t<M>;
        // The changes take V and C rather than auto: MSVC loses the
        // directive above inside a generic lambda, and this one, generic in
        // apply, repeats it.
        auto expect_invalidated_by = [this](const char * change, auto apply) {
            using namespace operators;
            SCOPED_TRACE(change);
            auto model = this->new_model();
            auto x = model.add_variable({.lower_bound = 0, .upper_bound = 3});
            auto y = model.add_variable({.lower_bound = 0, .upper_bound = 2});
            model.set_maximization();
            model.set_objective(x + y);
            auto c = model.add_constraint(x + y <= 4);
            model.solve();
            ASSERT_NEAR(model.get_solution_value(), 4.0, TEST_EPSILON);
            apply(model, x, y, c);
            expect_results_unreadable(model);
            model.solve();
            EXPECT_FALSE(is<status::unknown>(model.get_status()));
            EXPECT_NO_THROW((void)model.get_solution_value());
        };
        expect_invalidated_by("add_variable",
                              [](M & m, V, V, C) { m.add_variable(); });
        expect_invalidated_by("add_variables",
                              [](M & m, V, V, C) { m.add_variables(2); });
        expect_invalidated_by("add_constraint", [](M & m, V x, V, C) {
            m.add_constraint(x <= 2);
        });
        expect_invalidated_by("add_constraints", [](M & m, V x, V, C) {
            m.add_constraints(std::views::iota(0, 2),
                              [&](int i) { return x <= 2.0 + i; });
        });
        expect_invalidated_by("set_objective",
                              [](M & m, V x, V, C) { m.set_objective(2 * x); });
        expect_invalidated_by("set_objective_offset", [](M & m, V, V, C) {
            m.set_objective_offset(1.0);
        });
        expect_invalidated_by("set_minimization",
                              [](M & m, V, V, C) { m.set_minimization(); });
        expect_invalidated_by("set_maximization",
                              [](M & m, V, V, C) { m.set_maximization(); });
        if constexpr(has_modifiable_objective<M>) {
            expect_invalidated_by(
                "set_objective_coefficient",
                [](M & m, V x, V, C) { m.set_objective_coefficient(x, 2.0); });
            expect_invalidated_by("add_to_objective", [](M & m, V, V y, C) {
                m.add_to_objective(y);
            });
        }
        if constexpr(has_modifiable_variable_bounds<M>) {
            expect_invalidated_by(
                "set_variable_lower_bound",
                [](M & m, V x, V, C) { m.set_variable_lower_bound(x, 1.0); });
            expect_invalidated_by(
                "set_variable_upper_bound",
                [](M & m, V x, V, C) { m.set_variable_upper_bound(x, 2.0); });
        }
        if constexpr(has_modifiable_constraint_rhs<M>)
            expect_invalidated_by("set_constraint_rhs", [](M & m, V, V, C c) {
                m.set_constraint_rhs(c, 3.0);
            });
        if constexpr(has_modifiable_constraint_sense<M>)
            expect_invalidated_by("set_constraint_sense", [](M & m, V, V, C c) {
                m.set_constraint_sense(c, constraint_sense::greater_equal);
            });
        if constexpr(has_modifiable_constraint_bounds<M>) {
            expect_invalidated_by(
                "set_constraint_lower_bound",
                [](M & m, V, V, C c) { m.set_constraint_lower_bound(c, 1.0); });
            expect_invalidated_by(
                "set_constraint_upper_bound",
                [](M & m, V, V, C c) { m.set_constraint_upper_bound(c, 3.0); });
        }
        if constexpr(has_ranged_constraints<M>)
            expect_invalidated_by("add_ranged_constraint",
                                  [](M & m, V x, V y, C) {
                                      m.add_ranged_constraint(x + y, 0.0, 3.0);
                                  });
        if constexpr(has_column_generation<M>)
            expect_invalidated_by("add_column", [](M & m, V, V, C c) {
                m.add_column({{c, 1.0}}, {.obj_coef = 2, .upper_bound = 1});
            });
        if constexpr(has_remove_variable<M>)
            expect_invalidated_by("remove_variable", [](M & m, V, V y, C) {
                m.remove_variable(y);
            });
        if constexpr(has_named_variables<M>)
            expect_invalidated_by("set_variable_name", [](M & m, V x, V, C) {
                m.set_variable_name(x, "x");
            });
        if constexpr(has_named_constraints<M>)
            expect_invalidated_by("set_constraint_name", [](M & m, V, V, C c) {
                m.set_constraint_name(c, "c");
            });
        if constexpr(has_lp_basis_warm_start<M>)
            expect_invalidated_by("set_basis", [](M & m, V, V, C) {
                m.set_basis(m.get_basis());
            });
        if constexpr(has_mip_start<M>)
            // CPLEX takes a MIP start on a model with integer variables only
            expect_invalidated_by("add_mip_start", [](M & m, V x, V y, C) {
                auto i = m.add_integer_variable({.upper_bound = 1});
                m.solve();
                ASSERT_NO_THROW((void)m.get_solution_value());
                m.add_mip_start({{x, 3.0}, {y, 1.0}, {i, 1.0}});
            });
        if constexpr(milp_model<M>) {
            expect_invalidated_by("add_integer_variable", [](M & m, V, V, C) {
                m.add_integer_variable();
            });
            expect_invalidated_by("add_binary_variable", [](M & m, V, V, C) {
                m.add_binary_variable();
            });
        }
        if constexpr(requires(M & m, model_variable_t<M> v) {
                         m.set_integer(v);
                     })
            expect_invalidated_by("set_integer",
                                  [](M & m, V x, V, C) { m.set_integer(x); });
        if constexpr(requires(M & m, model_variable_t<M> v) {
                         m.set_continuous(v);
                     })
            expect_invalidated_by("set_continuous", [](M & m, V x, V, C) {
                m.set_continuous(x);
            });
        if constexpr(has_indicator_constraints<M>)
            expect_invalidated_by(
                "add_indicator_constraint", [](M & m, V x, V, C) {
                    auto b = m.add_binary_variable();
                    m.solve();
                    ASSERT_NO_THROW((void)m.get_solution_value());
                    m.add_indicator_constraint(b, true, x <= 1.0);
                });
        if constexpr(qp_model<M>)
            expect_invalidated_by("set_quadratic_objective",
                                  [](M & m, V x, V y, C) {
                                      m.set_quadratic_objective(x + y - x * x);
                                  });
    });
}

TYPED_TEST_P(LpModelTest, parameter_setters_keep_results) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        using M = typename TestFixture::model_type;
        auto model = this->new_model();
        auto x = model.add_variable({.upper_bound = 3});
        auto y = model.add_variable({.upper_bound = 2});
        model.set_maximization();
        model.set_objective(x + y);
        model.add_constraint(x + y <= 4);
        model.solve();
        if constexpr(has_time_limit<M>)
            model.set_time_limit(std::chrono::seconds(1000));
        if constexpr(has_verbosity<M>) model.set_verbose(false);
        if constexpr(has_thread_limit<M>) model.set_thread_limit(1);
        if constexpr(has_feasibility_tolerance<M>)
            model.set_feasibility_tolerance(model.get_feasibility_tolerance());
        if constexpr(has_optimality_tolerance<M>)
            model.set_optimality_tolerance(model.get_optimality_tolerance());
        EXPECT_TRUE(is_a<status::optimal>(model.get_status()));
        EXPECT_NEAR(model.get_solution_value(), 4.0, TEST_EPSILON);
    });
}

TYPED_TEST_P(LpModelTest, native_ids) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        static_assert(has_native_handles<typename TestFixture::model_type>);
        auto model = this->new_model();
        auto x = model.add_variable();
        auto y = model.add_variable();
        auto c1 = model.add_constraint(x + y <= 1);
        auto c2 = model.add_constraint(x - y >= 0);
        ASSERT_NE(model.native_id(x), model.native_id(y));
        ASSERT_NE(model.native_id(c1), model.native_id(c2));
    });
}

TYPED_TEST_P(LpModelTest, default_constructed_shares_interned_api) {
    this->SkipOnLicenseError([this]() {
        typename TestFixture::model_type model;
        ASSERT_EQ(&model.native_api(), TestFixture::api);
        ASSERT_EQ(&this->new_model().native_api(), TestFixture::api);
    });
}

REGISTER_TYPED_TEST_SUITE_P(
    LpModelTest, default_constructed_shares_interned_api, native_ids, construct,
    add_variable, add_variable_params, add_zero_variables, add_variables,
    add_variables_params, add_variable_and_variables,
    add_zero_indexed_variables, add_indexed_variables,
    add_indexed_variables_params, add_capturing_indexed_variables,
    add_variable_and_indexed_variables, set_objective,
    set_objective_distinct_variables, distinct_variables_repeat_is_asserted,
    add_constraint, add_constraint_distinct_variables, add_constraints,
    add_constraints_distinct_variables, add_opt_constraints,
    add_opt_constraints_distinct_variables, add_variables_by_key,
    add_constraints_by_key, solve_empty_no_sense, solve_empty_max,
    solve_empty_min, solve_bounded_variables_max, solve_bounded_variables_min,
    solve_lp, solve_lp_add_constraints, solve_lp_with_objective_offset_min,
    solve_lp_with_objective_offset_max, solve_lp_set_objective_offset,
    solve_lp_objective_redundant_terms, solve_lp_constraint_redundant_terms,
    solve_lp_zero_rows, solve_lp_distinct_variables,
    solve_lp_mixed_distinct_variables, solve_lp_non_standard_form_max,
    solve_lp_non_standard_form_min, move_preserves_status,
    reset_status_reports_unknown, results_need_a_solve,
    model_changes_invalidate_results, parameter_setters_keep_results);

}  // namespace mippp