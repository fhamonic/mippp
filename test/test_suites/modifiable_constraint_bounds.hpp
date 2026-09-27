#pragma once

#include <gtest/gtest.h>

#include "mippp/model_concepts.hpp"

namespace mippp {

template <typename T>
struct ModifiableConstraintBoundsTest : public T {
    using typename T::model_type;
    static_assert(has_modifiable_constraint_bounds<model_type>);
};
TYPED_TEST_SUITE_P(ModifiableConstraintBoundsTest);
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(ModifiableConstraintBoundsTest);

TYPED_TEST_P(ModifiableConstraintBoundsTest, set_constraint_lower_bound) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        using model_type = typename TestFixture::model_type;
        constexpr bool ranged = has_ranged_constraints<model_type>;
        auto model = this->new_model();
        auto x1 =
            model.add_variable({.lower_bound = -10.0, .upper_bound = 10.0});
        auto x2 =
            model.add_variable({.lower_bound = -10.0, .upper_bound = 10.0});
        auto x3 =
            model.add_variable({.lower_bound = -10.0, .upper_bound = 10.0});
        auto x4 =
            model.add_variable({.lower_bound = -10.0, .upper_bound = 10.0});
        auto c1 = model.add_constraint(x1 >= 1.0);
        auto c2 = model.add_constraint(x2 <= 5.0);
        auto c3 = model.add_constraint(x3 == 4.0);
        model.set_constraint_lower_bound(c1, 2.0);
        model.set_constraint_lower_bound(c2, 3.0);
        model.set_constraint_lower_bound(c3, 1.0);
        if constexpr(ranged) {
            auto c4 = model.add_ranged_constraint(x4, 1.0, 6.0);
            model.set_constraint_lower_bound(c4, -2.0);
            if constexpr(has_readable_constraint_bounds<model_type>) {
                ASSERT_EQ(model.get_constraint_lower_bound(c4), -2.0);
                ASSERT_EQ(model.get_constraint_upper_bound(c4), 6.0);
            }
        }
        if constexpr(has_readable_constraint_bounds<model_type>) {
            ASSERT_EQ(model.get_constraint_lower_bound(c1), 2.0);
            ASSERT_TRUE(
                model.is_infinite(model.get_constraint_upper_bound(c1)));
            ASSERT_EQ(model.get_constraint_lower_bound(c2), 3.0);
            ASSERT_EQ(model.get_constraint_upper_bound(c2), 5.0);
            ASSERT_EQ(model.get_constraint_lower_bound(c3), 1.0);
            ASSERT_EQ(model.get_constraint_upper_bound(c3), 4.0);
        }
        model.set_objective(x1 + x2 + x3 + x4);
        model.set_minimization();
        model.solve();
        auto solution = model.get_solution();
        ASSERT_NEAR(solution[x1], 2.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x2], 3.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x3], 1.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x4], ranged ? -2.0 : -10.0, TEST_EPSILON);
        model.set_maximization();
        model.solve();
        solution = model.get_solution();
        ASSERT_NEAR(solution[x1], 10.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x2], 5.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x3], 4.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x4], ranged ? 6.0 : 10.0, TEST_EPSILON);
    });
}
TYPED_TEST_P(ModifiableConstraintBoundsTest, set_constraint_upper_bound) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        using model_type = typename TestFixture::model_type;
        constexpr bool ranged = has_ranged_constraints<model_type>;
        auto model = this->new_model();
        auto x1 =
            model.add_variable({.lower_bound = -10.0, .upper_bound = 10.0});
        auto x2 =
            model.add_variable({.lower_bound = -10.0, .upper_bound = 10.0});
        auto x3 =
            model.add_variable({.lower_bound = -10.0, .upper_bound = 10.0});
        auto x4 =
            model.add_variable({.lower_bound = -10.0, .upper_bound = 10.0});
        auto c1 = model.add_constraint(x1 <= 5.0);
        auto c2 = model.add_constraint(x2 >= 1.0);
        auto c3 = model.add_constraint(x3 == 4.0);
        model.set_constraint_upper_bound(c1, 3.0);
        model.set_constraint_upper_bound(c2, 2.0);
        model.set_constraint_upper_bound(c3, 6.0);
        if constexpr(ranged) {
            auto c4 = model.add_ranged_constraint(x4, 1.0, 6.0);
            model.set_constraint_upper_bound(c4, 8.0);
            if constexpr(has_readable_constraint_bounds<model_type>) {
                ASSERT_EQ(model.get_constraint_lower_bound(c4), 1.0);
                ASSERT_EQ(model.get_constraint_upper_bound(c4), 8.0);
            }
        }
        if constexpr(has_readable_constraint_bounds<model_type>) {
            ASSERT_TRUE(
                model.is_infinite(model.get_constraint_lower_bound(c1)));
            ASSERT_EQ(model.get_constraint_upper_bound(c1), 3.0);
            ASSERT_EQ(model.get_constraint_lower_bound(c2), 1.0);
            ASSERT_EQ(model.get_constraint_upper_bound(c2), 2.0);
            ASSERT_EQ(model.get_constraint_lower_bound(c3), 4.0);
            ASSERT_EQ(model.get_constraint_upper_bound(c3), 6.0);
        }
        model.set_objective(x1 + x2 + x3 + x4);
        model.set_maximization();
        model.solve();
        auto solution = model.get_solution();
        ASSERT_NEAR(solution[x1], 3.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x2], 2.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x3], 6.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x4], ranged ? 8.0 : 10.0, TEST_EPSILON);
        model.set_minimization();
        model.solve();
        solution = model.get_solution();
        ASSERT_NEAR(solution[x1], -10.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x2], 1.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x3], 4.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x4], ranged ? 1.0 : -10.0, TEST_EPSILON);
    });
}

TYPED_TEST_P(ModifiableConstraintBoundsTest, infinity_frees_a_row_side) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x = model.add_variable({.lower_bound = -7.0, .upper_bound = 7.0});
        auto c = model.add_constraint(x == 0.0);
        model.set_constraint_upper_bound(c, model.infinity());
        model.set_constraint_lower_bound(c, -model.infinity());
        if constexpr(has_readable_constraint_bounds<decltype(model)>) {
            ASSERT_TRUE(model.is_infinite(model.get_constraint_upper_bound(c)));
            ASSERT_TRUE(model.is_infinite(model.get_constraint_lower_bound(c)));
        }
        model.set_objective(x);
        model.set_maximization();
        model.solve();
        ASSERT_NEAR(model.get_solution_value(), 7.0, TEST_EPSILON);
        model.set_minimization();
        model.solve();
        ASSERT_NEAR(model.get_solution_value(), -7.0, TEST_EPSILON);
    });
}

TYPED_TEST_P(ModifiableConstraintBoundsTest, resolve_after_moving_row_sides) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x = model.add_variable();
        auto y = model.add_variable();
        auto c1 = model.add_constraint(x + y <= 4.0);
        auto c2 = model.add_constraint(x - y >= -2.0);
        model.set_objective(x + 2 * y);
        model.set_maximization();
        model.solve();
        ASSERT_NEAR(model.get_solution_value(), 7.0, TEST_EPSILON);
        model.set_constraint_upper_bound(c1, 6.0);
        model.solve();
        auto solution = model.get_solution();
        ASSERT_NEAR(solution[x], 2.0, TEST_EPSILON);
        ASSERT_NEAR(solution[y], 4.0, TEST_EPSILON);
        ASSERT_NEAR(model.get_solution_value(), 10.0, TEST_EPSILON);
        model.set_constraint_lower_bound(c2, -4.0);
        model.solve();
        solution = model.get_solution();
        ASSERT_NEAR(solution[x], 1.0, TEST_EPSILON);
        ASSERT_NEAR(solution[y], 5.0, TEST_EPSILON);
        ASSERT_NEAR(model.get_solution_value(), 11.0, TEST_EPSILON);
        model.set_constraint_upper_bound(c1, 4.0);
        model.solve();
        solution = model.get_solution();
        ASSERT_NEAR(solution[x], 0.0, TEST_EPSILON);
        ASSERT_NEAR(solution[y], 4.0, TEST_EPSILON);
        ASSERT_NEAR(model.get_solution_value(), 8.0, TEST_EPSILON);
    });
}

// No ranged row here: a ranged row has no sense to read back.
TYPED_TEST_P(ModifiableConstraintBoundsTest, retightened_row_reads_its_sense) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        using model_type = typename TestFixture::model_type;
        auto model = this->new_model();
        auto x = model.add_variable({.lower_bound = 0.0, .upper_bound = 10.0});
        auto y = model.add_variable({.lower_bound = 0.0, .upper_bound = 10.0});
        auto z = model.add_variable({.lower_bound = 0.0, .upper_bound = 10.0});
        auto c1 = model.add_constraint(x + y <= 4.0);
        auto c2 = model.add_constraint(x - y >= -2.0);
        auto c3 = model.add_constraint(z == 1.0);
        model.set_objective(x + 2 * y + z);
        model.set_maximization();
        model.solve();
        ASSERT_NEAR(model.get_solution_value(), 8.0, TEST_EPSILON);
        model.set_constraint_upper_bound(c1, model.infinity());
        model.set_constraint_lower_bound(c2, -model.infinity());
        model.set_constraint_upper_bound(c3, model.infinity());
        if constexpr(has_readable_constraint_sense<model_type>) {
            ASSERT_EQ(model.get_constraint_sense(c3),
                      constraint_sense::greater_equal);
        }
        model.solve();
        ASSERT_NEAR(model.get_solution_value(), 40.0, TEST_EPSILON);
        model.set_constraint_upper_bound(c1, 4.0);
        model.set_constraint_lower_bound(c2, -2.0);
        model.set_constraint_upper_bound(c3, 1.0);
        if constexpr(has_readable_constraint_sense<model_type>) {
            ASSERT_EQ(model.get_constraint_sense(c1),
                      constraint_sense::less_equal);
            ASSERT_EQ(model.get_constraint_sense(c2),
                      constraint_sense::greater_equal);
            ASSERT_EQ(model.get_constraint_sense(c3), constraint_sense::equal);
        }
        if constexpr(has_readable_constraint_rhs<model_type>) {
            ASSERT_EQ(model.get_constraint_rhs(c1), 4.0);
            ASSERT_EQ(model.get_constraint_rhs(c2), -2.0);
            ASSERT_EQ(model.get_constraint_rhs(c3), 1.0);
        }
        model.solve();
        auto solution = model.get_solution();
        ASSERT_NEAR(solution[x], 1.0, TEST_EPSILON);
        ASSERT_NEAR(solution[y], 3.0, TEST_EPSILON);
        ASSERT_NEAR(solution[z], 1.0, TEST_EPSILON);
        ASSERT_NEAR(model.get_solution_value(), 8.0, TEST_EPSILON);
    });
}

REGISTER_TYPED_TEST_SUITE_P(ModifiableConstraintBoundsTest,
                            set_constraint_lower_bound,
                            set_constraint_upper_bound,
                            infinity_frees_a_row_side,
                            resolve_after_moving_row_sides,
                            retightened_row_reads_its_sense);

}  // namespace mippp
