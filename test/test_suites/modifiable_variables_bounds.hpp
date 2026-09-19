#pragma once

#include <gtest/gtest.h>

#include "mippp/model_concepts.hpp"

namespace mippp {

template <typename T>
struct ModifiableVariablesBoundsTest : public T {
    using typename T::model_type;
    static_assert(has_modifiable_variable_bounds<model_type>);
};
TYPED_TEST_SUITE_P(ModifiableVariablesBoundsTest);
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(ModifiableVariablesBoundsTest);

TYPED_TEST_P(ModifiableVariablesBoundsTest, set_variable_lower_bound) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x1 = model.add_variable();
        auto x2 = model.add_variable({.lower_bound = 2.5});
        auto x3 = model.add_variable({});
        auto xs1 = model.add_variables(1);
        auto xs2 = model.add_variables(1, {.lower_bound = 3.5});
        auto xs3 = model.add_variables(1, {});
        auto xis1 = model.add_variables(1, [](int i) { return i; });
        auto xis2 = model.add_variables(1, [](int i) { return i; },
                                        {.lower_bound = 4.5});
        auto xis3 = model.add_variables(1, [](int i) { return i; }, {});
        model.set_minimization();
        model.set_objective(x1 + x2 + x3 + xs1 + xs2 + xs3 + xis1 + xis2 +
                            xis3);
        model.set_variable_lower_bound(x1, -1.0);
        model.set_variable_lower_bound(x2, 2.0);
        model.set_variable_lower_bound(x3, 3.0);
        model.set_variable_lower_bound(xs1[0], 4.0);
        model.set_variable_lower_bound(xs2[0], -5.0);
        model.set_variable_lower_bound(xs3[0], 6.0);
        model.set_variable_lower_bound(xis1(0), 7.0);
        model.set_variable_lower_bound(xis2(0), 8.0);
        model.set_variable_lower_bound(xis3(0), -9.0);
        model.solve();
        auto solution = model.get_solution();
        ASSERT_NEAR(solution[x1], -1.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x2], 2.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x3], 3.0, TEST_EPSILON);
        ASSERT_NEAR(solution[xs1[0]], 4.0, TEST_EPSILON);
        ASSERT_NEAR(solution[xs2[0]], -5.0, TEST_EPSILON);
        ASSERT_NEAR(solution[xs3[0]], 6.0, TEST_EPSILON);
        ASSERT_NEAR(solution[xis1(0)], 7.0, TEST_EPSILON);
        ASSERT_NEAR(solution[xis2(0)], 8.0, TEST_EPSILON);
        ASSERT_NEAR(solution[xis3(0)], -9.0, TEST_EPSILON);
    });
}
TYPED_TEST_P(ModifiableVariablesBoundsTest, set_variable_upper_bound) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x1 = model.add_variable();
        auto x2 = model.add_variable({.upper_bound = 2.5});
        auto x3 = model.add_variable({});
        auto xs1 = model.add_variables(1);
        auto xs2 = model.add_variables(1, {.upper_bound = 3.5});
        auto xs3 = model.add_variables(1, {});
        auto xis1 = model.add_variables(1, [](int i) { return i; });
        auto xis2 = model.add_variables(1, [](int i) { return i; },
                                        {.upper_bound = 4.5});
        auto xis3 = model.add_variables(1, [](int i) { return i; }, {});
        model.set_maximization();
        model.set_objective(x1 + x2 + x3 + xs1 + xs2 + xs3 + xis1 + xis2 +
                            xis3);
        model.set_variable_upper_bound(x1, 1.0);
        model.set_variable_upper_bound(x2, 2.0);
        model.set_variable_upper_bound(x3, 3.0);
        model.set_variable_upper_bound(xs1[0], 4.0);
        model.set_variable_upper_bound(xs2[0], -5.0);
        model.set_variable_upper_bound(xs3[0], 6.0);
        model.set_variable_upper_bound(xis1(0), 7.0);
        model.set_variable_upper_bound(xis2(0), 8.0);
        model.set_variable_upper_bound(xis3(0), -9.0);
        model.solve();
        auto solution = model.get_solution();
        ASSERT_NEAR(solution[x1], 1.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x2], 2.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x3], 3.0, TEST_EPSILON);
        ASSERT_NEAR(solution[xs1[0]], 4.0, TEST_EPSILON);
        ASSERT_NEAR(solution[xs2[0]], -5.0, TEST_EPSILON);
        ASSERT_NEAR(solution[xs3[0]], 6.0, TEST_EPSILON);
        ASSERT_NEAR(solution[xis1(0)], 7.0, TEST_EPSILON);
        ASSERT_NEAR(solution[xis2(0)], 8.0, TEST_EPSILON);
        ASSERT_NEAR(solution[xis3(0)], -9.0, TEST_EPSILON);
    });
}

// A column without constraint entries whose bound range was infinite at the
// previous solve: a warm-started re-solve must see its new finite range.
TYPED_TEST_P(ModifiableVariablesBoundsTest, resolve_after_bounding_free_range) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x1 = model.add_variable({.upper_bound = 0.0});
        auto x2 = model.add_variable({.lower_bound = 0.0});
        auto x3 = model.add_variable({.lower_bound = 1.0, .upper_bound = 2.0});
        model.set_minimization();
        model.set_objective(-x1 + x2 + x3);
        model.solve();
        auto solution = model.get_solution();
        ASSERT_NEAR(solution[x1], 0.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x2], 0.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x3], 1.0, TEST_EPSILON);
        model.set_objective(x1 - x2 + x3);
        model.set_variable_lower_bound(x1, -4.0);
        model.set_variable_upper_bound(x2, 3.0);
        model.solve();
        solution = model.get_solution();
        ASSERT_NEAR(solution[x1], -4.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x2], 3.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x3], 1.0, TEST_EPSILON);
        ASSERT_NEAR(model.get_solution_value(), -6.0, TEST_EPSILON);
    });
}

TYPED_TEST_P(ModifiableVariablesBoundsTest, infinity_removes_a_bound) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x = model.add_variable({.lower_bound = 0.0, .upper_bound = 3.0});
        model.set_variable_upper_bound(x, model.infinity());
        model.set_variable_lower_bound(x, -model.infinity());
        if constexpr(has_readable_variable_bounds<decltype(model)>) {
            ASSERT_TRUE(model.is_infinite(model.get_variable_upper_bound(x)));
            ASSERT_TRUE(model.is_infinite(model.get_variable_lower_bound(x)));
        }
        model.add_constraint(x <= 7.0);
        model.add_constraint(x >= -7.0);
        model.set_objective(x);
        model.set_maximization();
        model.solve();
        ASSERT_NEAR(model.get_solution_value(), 7.0, TEST_EPSILON);
        model.set_minimization();
        model.solve();
        ASSERT_NEAR(model.get_solution_value(), -7.0, TEST_EPSILON);
    });
}

// GLPK refuses a double-bounded column with lb == ub
TYPED_TEST_P(ModifiableVariablesBoundsTest, equal_bounds_fix_the_variable) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x = model.add_variable();
        auto y = model.add_variable({.lower_bound = 0.0, .upper_bound = 10.0});
        model.set_variable_lower_bound(x, 2.0);
        model.set_variable_upper_bound(x, 2.0);
        model.set_objective(x + y);
        model.set_minimization();
        model.solve();
        ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
        ASSERT_NEAR(model.get_solution_value(), 2.0, TEST_EPSILON);
        ASSERT_NEAR(model.get_solution()[x], 2.0, TEST_EPSILON);
    });
}

REGISTER_TYPED_TEST_SUITE_P(ModifiableVariablesBoundsTest,
                            set_variable_lower_bound, set_variable_upper_bound,
                            resolve_after_bounding_free_range,
                            infinity_removes_a_bound,
                            equal_bounds_fix_the_variable);

}  // namespace mippp