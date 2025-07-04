#pragma once
#undef NDEBUG
#include <gtest/gtest.h>

#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/quadratic_expression.hpp"

namespace fhamonic::mippp {

template <typename T>
struct QpModelTest : public T {
    using typename T::model_type;
    static_assert(qp_model<model_type>);
};
TYPED_TEST_SUITE_P(QpModelTest);

TYPED_TEST_P(QpModelTest, test) {
    using namespace operators;
    auto model = this->new_model();
    auto x1 = model.add_variable();
    auto x2 = model.add_variable();
    model.set_minimization();
    model.set_objective(2 * x1 * x1 + 2 * x2 * x2 - 4 * x1 - 6 * x2);
    auto c1 = model.add_constraint(x1 + x2 >= 3);
    model.solve();
    EXPECT_NEAR(model.get_solution_value(), -6.25, TEST_EPSILON);
    auto solution = model.get_solution();
    EXPECT_NEAR(solution[x1], 1.25, TEST_EPSILON);
    EXPECT_NEAR(solution[x2], 1.75, TEST_EPSILON);
}

REGISTER_TYPED_TEST_SUITE_P(QpModelTest, test);

}  // namespace fhamonic::mippp