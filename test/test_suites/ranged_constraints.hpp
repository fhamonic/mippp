#pragma once

#include <gtest/gtest.h>

#include "mippp/model_concepts.hpp"

namespace mippp {

template <typename T>
struct RangedConstraintsTest : public T {
    using typename T::model_type;
    static_assert(has_ranged_constraints<model_type>);
};
TYPED_TEST_SUITE_P(RangedConstraintsTest);
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(RangedConstraintsTest);

TYPED_TEST_P(RangedConstraintsTest, add_ranged_constraint) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x1 = model.add_variable();
        auto x2 = model.add_variable();
        auto c = model.add_ranged_constraint(x1 + x2, 1.0, 3.0);
        static_assert(
            std::same_as<decltype(c),
                         model_constraint_t<typename TestFixture::model_type>>);
        ASSERT_EQ(model.num_constraints(), 1u);
        model.set_maximization();
        model.set_objective(x1 + 2 * x2);
        model.solve();
        ASSERT_NEAR(model.get_solution_value(), 6.0, TEST_EPSILON);
        model.set_minimization();
        model.solve();
        ASSERT_NEAR(model.get_solution_value(), 1.0, TEST_EPSILON);
    });
}
TYPED_TEST_P(RangedConstraintsTest, constant_moves_to_the_bounds) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x1 = model.add_variable();
        auto x2 = model.add_variable();
        model.add_ranged_constraint(x1 + x2 + 1.0, 2.0, 4.0);
        model.set_maximization();
        model.set_objective(x1 + 2 * x2);
        model.solve();
        ASSERT_NEAR(model.get_solution_value(), 6.0, TEST_EPSILON);
        model.set_minimization();
        model.solve();
        ASSERT_NEAR(model.get_solution_value(), 1.0, TEST_EPSILON);
    });
}
TYPED_TEST_P(RangedConstraintsTest, repeated_variables_are_coalesced) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x1 = model.add_variable();
        auto x2 = model.add_variable();
        model.add_ranged_constraint(x1 + x2 + x1, 1.0, 3.0);  // 2 x1 + x2
        model.set_maximization();
        model.set_objective(x1);
        model.solve();
        ASSERT_NEAR(model.get_solution_value(), 1.5, TEST_EPSILON);
    });
}
TYPED_TEST_P(RangedConstraintsTest, distinct_variables_form) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x1 = model.add_variable();
        auto x2 = model.add_variable();
        model.add_ranged_constraint(distinct_variables, 2 * x1 + x2, 1.0, 3.0);
        model.set_maximization();
        model.set_objective(x1);
        model.solve();
        ASSERT_NEAR(model.get_solution_value(), 1.5, TEST_EPSILON);
    });
}

// see LpModelTest.solve_lp_zero_rows
TYPED_TEST_P(RangedConstraintsTest, zero_rows) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x1 =
            model.add_variable({.lower_bound = -5.0, .upper_bound = -2.0});
        auto x2 = model.add_variable({.lower_bound = 1.0, .upper_bound = 4.0});
        model.add_ranged_constraint(x1 - x1, -1.0, 3.0);
        model.add_ranged_constraint(distinct_variables, 0.0 * x2, -1.0, 3.0);
        model.set_minimization();
        model.set_objective(-0.5 * x1 + x2);
        model.solve();
        ASSERT_NEAR(model.get_solution_value(), 2.0, TEST_EPSILON);
    });
}

REGISTER_TYPED_TEST_SUITE_P(RangedConstraintsTest, add_ranged_constraint,
                            constant_moves_to_the_bounds,
                            repeated_variables_are_coalesced,
                            distinct_variables_form, zero_rows);

}  // namespace mippp
