#pragma once
#undef NDEBUG
#include <gtest/gtest.h>

#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"

namespace fhamonic::mippp {

template <typename T>
struct DualSolutionTest : public T {
    using typename T::model_type;
    static_assert(has_dual_solution<model_type>);
};
TYPED_TEST_SUITE_P(DualSolutionTest);

TYPED_TEST_P(DualSolutionTest, standard_form_max) {
    using namespace operators;
    auto model = this->construct_model();
    auto x1 = model.add_variable();
    auto x2 = model.add_variable();
    auto x3 = model.add_variable();
    model.set_maximization();
    model.set_objective(5 * x1 + 4 * x2 + 3 * x3);
    auto c1 = model.add_constraint(-2 * x1 - 2 * x2 + x3 <= -5);
    auto c2 = model.add_constraint(4 * x1 + x2 + 2 * x3 <= 11);
    auto c3 = model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 <= 8);
    model.solve();
    ASSERT_NEAR(model.get_solution_value(), 13.3333333333, TEST_EPSILON);
    auto solution = model.get_solution();
    ASSERT_NEAR(solution[x1], 2.6666666666, TEST_EPSILON);
    ASSERT_NEAR(solution[x2], 0.0, TEST_EPSILON);
    ASSERT_NEAR(solution[x3], 0.0, TEST_EPSILON);
    auto dual_solution = model.get_dual_solution();
    ASSERT_NEAR(dual_solution[c1], 0.0, TEST_EPSILON);
    ASSERT_NEAR(dual_solution[c2], 0.0, TEST_EPSILON);
    ASSERT_NEAR(dual_solution[c3], 1.6666666666, TEST_EPSILON);
}
TYPED_TEST_P(DualSolutionTest, arbitrary_max) {
    using namespace operators;
    auto model = this->construct_model();
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
    auto dual_solution = model.get_dual_solution();
    ASSERT_NEAR(dual_solution[c1], 0.0, TEST_EPSILON);
    ASSERT_NEAR(dual_solution[c2], 0.0, TEST_EPSILON);
    ASSERT_NEAR(dual_solution[c3], 1.6666666666, TEST_EPSILON);
}
TYPED_TEST_P(DualSolutionTest, arbitrary_min) {
    using namespace operators;
    auto model = this->construct_model();
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
    auto dual_solution = model.get_dual_solution();
    ASSERT_NEAR(dual_solution[c1], 4.0, TEST_EPSILON);
    ASSERT_NEAR(dual_solution[c2], 0.0, TEST_EPSILON);
    ASSERT_NEAR(dual_solution[c3], -1.0, TEST_EPSILON);
}

TYPED_TEST_P(DualSolutionTest, arbitrary_min_add_constraints) {
    using namespace operators;
    auto model = this->construct_model();
    auto x1 = model.add_variable();
    auto x2 = model.add_variable();
    auto x3 = model.add_variable();
    model.set_minimization();
    model.set_objective(5 * x1 + 4 * x2 + 3 * x3);
    auto c = model.add_constraints(
        ranges::views::iota(0, 3),
        [&](int i) { return OPT((i == 0), 2 * x1 + 2 * x2 - x3 >= 5); },
        [&](int i) { return OPT((i == 1), 4 * x1 + x2 + 2 * x3 <= 11); },
        [&](int i) { return 3 * x1 + 4 * x2 + 2 * x3 == 8; });
    model.solve();
    ASSERT_NEAR(model.get_solution_value(), 12.0, TEST_EPSILON);
    auto solution = model.get_solution();
    ASSERT_NEAR(solution[x1], 2.0, TEST_EPSILON);
    ASSERT_NEAR(solution[x2], 0.5, TEST_EPSILON);
    ASSERT_NEAR(solution[x3], 0.0, TEST_EPSILON);
    auto dual_solution = model.get_dual_solution();
    ASSERT_NEAR(dual_solution[c(0)], 4.0, TEST_EPSILON);
    ASSERT_NEAR(dual_solution[c(1)], 0.0, TEST_EPSILON);
    ASSERT_NEAR(dual_solution[c(2)], -1.0, TEST_EPSILON);
}

REGISTER_TYPED_TEST_SUITE_P(DualSolutionTest, standard_form_max, arbitrary_max,
                            arbitrary_min, arbitrary_min_add_constraints);

}  // namespace fhamonic::mippp