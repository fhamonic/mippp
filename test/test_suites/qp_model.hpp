#pragma once

#include <gtest/gtest.h>

#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/quadratic_expression.hpp"

namespace mippp {

template <typename T>
struct QpModelTest : public T {
    using typename T::model_type;
    static_assert(qp_model<model_type>);
};
TYPED_TEST_SUITE_P(QpModelTest);
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(QpModelTest);

TYPED_TEST_P(QpModelTest, test) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x1 = model.add_variable();
        auto x2 = model.add_variable();
        model.set_minimization();
        model.set_quadratic_objective(2 * x1 * x1 + 2 * x2 * x2 - 4 * x1 -
                                      6 * x2);
        model.add_constraint(x1 + x2 >= 3);
        model.solve();
        EXPECT_NEAR(model.get_solution_value(), -6.25, TEST_EPSILON);
        auto solution = model.get_solution();
        EXPECT_NEAR(solution[x1], 1.25, TEST_EPSILON);
        EXPECT_NEAR(solution[x2], 1.75, TEST_EPSILON);
    });
}
TYPED_TEST_P(QpModelTest, set_objective_distinct_variables) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x1 = model.add_variable();
        auto x2 = model.add_variable();
        model.set_minimization();
        model.set_quadratic_objective(
            distinct_variables, 2 * x1 * x1 + 2 * x2 * x2 - 4 * x1 - 6 * x2);
        model.add_constraint(distinct_variables, x1 + x2 >= 3);
        model.solve();
        EXPECT_NEAR(model.get_solution_value(), -6.25, TEST_EPSILON);
        auto solution = model.get_solution();
        EXPECT_NEAR(solution[x1], 1.25, TEST_EPSILON);
        EXPECT_NEAR(solution[x2], 1.75, TEST_EPSILON);
    });
}

TYPED_TEST_P(QpModelTest, linear_objective_replaces_quadratic) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x1 = model.add_variable({.upper_bound = 10.0});
        auto x2 = model.add_variable({.upper_bound = 10.0});
        model.set_minimization();
        model.add_constraint(x1 + x2 >= 3);
        model.set_quadratic_objective(2 * x1 * x1 + 2 * x2 * x2 - 4 * x1 -
                                      6 * x2);
        model.solve();
        EXPECT_NEAR(model.get_solution_value(), -6.25, TEST_EPSILON);
        // the quadratic part must not survive a linear set_objective
        model.set_objective(-4 * x1 - 6 * x2);
        model.solve();
        ASSERT_TRUE(is<status::optimal>(model.get_status()));
        EXPECT_NEAR(model.get_solution_value(), -100.0, TEST_EPSILON);
        auto solution = model.get_solution();
        EXPECT_NEAR(solution[x1], 10.0, TEST_EPSILON);
        EXPECT_NEAR(solution[x2], 10.0, TEST_EPSILON);
    });
}

// x1² + x1·x2 + x2² - 3x1 - 3x2 has its unique minimum -3 at (1, 1); a
// backend doubling the cross term would report -2.25 instead
TYPED_TEST_P(QpModelTest, cross_terms) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x1 = model.add_variable();
        auto x2 = model.add_variable();
        model.set_minimization();
        model.set_quadratic_objective(x1 * x1 + x1 * x2 + x2 * x2 - 3 * x1 -
                                      3 * x2);
        model.solve();
        EXPECT_NEAR(model.get_solution_value(), -3.0, TEST_EPSILON);
        auto solution = model.get_solution();
        EXPECT_NEAR(solution[x1], 1.0, TEST_EPSILON);
        EXPECT_NEAR(solution[x2], 1.0, TEST_EPSILON);
    });
}
// the same objective written with the pair in both orientations
TYPED_TEST_P(QpModelTest, cross_terms_unordered_pairs) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x1 = model.add_variable();
        auto x2 = model.add_variable();
        model.set_minimization();
        model.set_quadratic_objective(square(x1 + x2) - x2 * x1 - 3 * x1 -
                                      3 * x2);
        model.solve();
        EXPECT_NEAR(model.get_solution_value(), -3.0, TEST_EPSILON);
        auto solution = model.get_solution();
        EXPECT_NEAR(solution[x1], 1.0, TEST_EPSILON);
        EXPECT_NEAR(solution[x2], 1.0, TEST_EPSILON);
    });
}

TYPED_TEST_P(QpModelTest, xsum_objective) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto xs = model.add_variables(2);
        model.set_minimization();
        // sum of (x - 3)^2 : minimum 0 at x = 3, before the row moves it
        model.set_quadratic_objective(
            xsum(xs, [](auto x) { return square(x - 3.0); }));
        model.add_constraint(xs[0] + xs[1] <= 4);
        model.solve();
        EXPECT_NEAR(model.get_solution_value(), 2.0, TEST_EPSILON);
        auto solution = model.get_solution();
        EXPECT_NEAR(solution[xs[0]], 2.0, TEST_EPSILON);
        EXPECT_NEAR(solution[xs[1]], 2.0, TEST_EPSILON);
    });
}

REGISTER_TYPED_TEST_SUITE_P(QpModelTest, test, set_objective_distinct_variables,
                            linear_objective_replaces_quadratic, cross_terms,
                            cross_terms_unordered_pairs, xsum_objective);

}  // namespace mippp