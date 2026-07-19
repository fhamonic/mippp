#pragma once

#undef NDEBUG
#include <gtest/gtest.h>

#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"

namespace mippp {

template <typename T>
struct ReducedCostsTest : public T {
    using typename T::model_type;
    static_assert(has_reduced_costs<model_type>);
};
TYPED_TEST_SUITE_P(ReducedCostsTest);

TYPED_TEST_P(ReducedCostsTest, standard_form_max) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x1 = model.add_variable();
        auto x2 = model.add_variable();
        auto x3 = model.add_variable();
        model.set_maximization();
        model.set_objective(5 * x1 + 4 * x2 + 3 * x3);
        model.add_constraint(-2 * x1 - 2 * x2 + x3 <= -5);
        model.add_constraint(4 * x1 + x2 + 2 * x3 <= 11);
        model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 <= 8);
        model.solve();
        ASSERT_NEAR(model.get_solution_value(), 13.3333333333, TEST_EPSILON);
        auto solution = model.get_solution();
        ASSERT_NEAR(solution[x1], 2.6666666666, TEST_EPSILON);
        ASSERT_NEAR(solution[x2], 0.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x3], 0.0, TEST_EPSILON);
        auto reduced_costs = model.get_reduced_costs();
        ASSERT_NEAR(reduced_costs[x1], 0.0, TEST_EPSILON);
        ASSERT_NEAR(reduced_costs[x2], -2.6666666666, TEST_EPSILON);
        ASSERT_NEAR(reduced_costs[x3], -0.3333333333, TEST_EPSILON);
    });
}
TYPED_TEST_P(ReducedCostsTest, arbitrary_max) {
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
        auto reduced_costs = model.get_reduced_costs();
        ASSERT_NEAR(reduced_costs[x1], 0.0, TEST_EPSILON);
        ASSERT_NEAR(reduced_costs[x2], -2.6666666666, TEST_EPSILON);
        ASSERT_NEAR(reduced_costs[x3], -0.3333333333, TEST_EPSILON);
    });
}
TYPED_TEST_P(ReducedCostsTest, arbitrary_min) {
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
        auto reduced_costs = model.get_reduced_costs();
        ASSERT_NEAR(reduced_costs[x1], 0.0, TEST_EPSILON);
        ASSERT_NEAR(reduced_costs[x2], 0.0, TEST_EPSILON);
        ASSERT_NEAR(reduced_costs[x3], 9.0, TEST_EPSILON);
    });
}
TYPED_TEST_P(ReducedCostsTest, upper_bounded_max) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x1 = model.add_variable({.upper_bound = 3});
        auto x2 = model.add_variable();
        model.set_maximization();
        model.set_objective(2 * x1 + x2);
        model.add_constraint(x1 + x2 <= 4);
        model.solve();
        ASSERT_NEAR(model.get_solution_value(), 7.0, TEST_EPSILON);
        auto solution = model.get_solution();
        ASSERT_NEAR(solution[x1], 3.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x2], 1.0, TEST_EPSILON);
        auto reduced_costs = model.get_reduced_costs();
        ASSERT_NEAR(reduced_costs[x1], 1.0, TEST_EPSILON);
        ASSERT_NEAR(reduced_costs[x2], 0.0, TEST_EPSILON);
    });
}

REGISTER_TYPED_TEST_SUITE_P(ReducedCostsTest, standard_form_max, arbitrary_max,
                            arbitrary_min, upper_bounded_max);

}  // namespace mippp
