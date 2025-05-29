#pragma once
#undef NDEBUG
#include <gtest/gtest.h>

#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"

#include "assert_helper.hpp"

namespace fhamonic::mippp {

template <typename T>
struct ModifiableObjectiveTest : public T {
    using typename T::model_type;
    static_assert(has_modifiable_objective<model_type>);
};
TYPED_TEST_SUITE_P(ModifiableObjectiveTest);

TYPED_TEST_P(ModifiableObjectiveTest, set_objective_coefficient) {
    using namespace operators;
    auto model = this->new_model();
    auto x1 = model.add_variable();
    auto x2 = model.add_variable({.obj_coef = 2.5, .lower_bound = 0});
    auto x3 = model.add_variable({.lower_bound = 0});
    auto xs1 = model.add_variables(1);
    auto xs2 = model.add_variables(1, {.obj_coef = 3.5, .lower_bound = 0});
    auto xs3 = model.add_variables(1, {.lower_bound = 0});
    auto xis1 = model.add_variables(1, [](int i) { return i; });
    auto xis2 = model.add_variables(1, [](int i) { return i; },
                                    {.obj_coef = 7.5, .lower_bound = 0});
    auto xis3 =
        model.add_variables(1, [](int i) { return i; }, {.lower_bound = 0});
    model.set_maximization();
    model.set_objective(3 * x1 + 5 * xs2 + xis3);
    model.add_constraint(x1 + x2 + x3 <= 1);
    model.add_constraint(xs1 + xs2 + xs3 <= 2);
    model.add_constraint(xis1 + xis2 + xis3 <= 3);
    model.set_objective_coefficient(x1, 0.0);
    model.set_objective_coefficient(x3, 2.0);
    model.set_objective_coefficient(xs1[0], 2.5);
    model.set_objective_coefficient(xs3[0], 4.0);
    model.set_objective_coefficient(xis1(0), 6.0);
    model.set_objective_coefficient(xis3(0), 4.0);
    model.solve();
    ASSERT_NEAR(model.get_solution_value(), 30.0, TEST_EPSILON);
    auto solution = model.get_solution();
    ASSERT_NEAR(solution[x1], 0.0, TEST_EPSILON);
    ASSERT_NEAR(solution[x2], 0.0, TEST_EPSILON);
    ASSERT_NEAR(solution[x3], 1.0, TEST_EPSILON);
    ASSERT_NEAR(solution[xs1[0]], 0.0, TEST_EPSILON);
    ASSERT_NEAR(solution[xs2[0]], 2.0, TEST_EPSILON);
    ASSERT_NEAR(solution[xs3[0]], 0.0, TEST_EPSILON);
    ASSERT_NEAR(solution[xis1(0)], 3.0, TEST_EPSILON);
    ASSERT_NEAR(solution[xis2(0)], 0.0, TEST_EPSILON);
    ASSERT_NEAR(solution[xis3(0)], 0.0, TEST_EPSILON);
}
TYPED_TEST_P(ModifiableObjectiveTest, add_objective) {
    using namespace operators;
    auto model = this->new_model();
    auto x1 = model.add_variable();
    auto x2 = model.add_variable({.obj_coef = 2.5, .lower_bound = 0});
    auto x3 = model.add_variable({.lower_bound = 0});
    auto xs1 = model.add_variables(1);
    auto xs2 = model.add_variables(1, {.obj_coef = 3.5, .lower_bound = 0});
    auto xs3 = model.add_variables(1, {.lower_bound = 0});
    auto xis1 = model.add_variables(1, [](int i) { return i; });
    auto xis2 = model.add_variables(1, [](int i) { return i; },
                                    {.obj_coef = 7.5, .lower_bound = 0});
    auto xis3 =
        model.add_variables(1, [](int i) { return i; }, {.lower_bound = 0});
    model.set_maximization();
    model.set_objective(10 + 3 * x1 + 5 * xs2 + xis3);
    model.add_objective(6 + -3 * x1 + 2 * x3 + 2.5 * xs1 + 4 * xs3 + 6 * xis1 +
                        3 * xis3);
    model.add_constraint(x1 + x2 + x3 <= 1);
    model.add_constraint(xs1 + xs2 + xs3 <= 2);
    model.add_constraint(xis1 + xis2 + xis3 <= 3);
    model.solve();
    ASSERT_NEAR(model.get_solution_value(), 46.0, TEST_EPSILON);
    auto solution = model.get_solution();
    ASSERT_NEAR(solution[x1], 0.0, TEST_EPSILON);
    ASSERT_NEAR(solution[x2], 0.0, TEST_EPSILON);
    ASSERT_NEAR(solution[x3], 1.0, TEST_EPSILON);
    ASSERT_NEAR(solution[xs1[0]], 0.0, TEST_EPSILON);
    ASSERT_NEAR(solution[xs2[0]], 2.0, TEST_EPSILON);
    ASSERT_NEAR(solution[xs3[0]], 0.0, TEST_EPSILON);
    ASSERT_NEAR(solution[xis1(0)], 3.0, TEST_EPSILON);
    ASSERT_NEAR(solution[xis2(0)], 0.0, TEST_EPSILON);
    ASSERT_NEAR(solution[xis3(0)], 0.0, TEST_EPSILON);
}

REGISTER_TYPED_TEST_SUITE_P(ModifiableObjectiveTest, set_objective_coefficient, add_objective);

}  // namespace fhamonic::mippp