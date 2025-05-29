#pragma once
#undef NDEBUG
#include <gtest/gtest.h>

#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"

namespace fhamonic::mippp {

template <typename T>
struct LpStatusTest : public T {
    using typename T::model_type;
    static_assert(lp_model<model_type>);
};
TYPED_TEST_SUITE_P(LpStatusTest);

TYPED_TEST_P(LpStatusTest, not_solved) {
    using namespace operators;
    auto model = this->new_model();
    ASSERT_FALSE(model.is_optimal());
    ASSERT_FALSE(model.is_infeasible());
    ASSERT_FALSE(model.is_unbounded());
}
TYPED_TEST_P(LpStatusTest, max_bounded) {
    using namespace operators;
    auto model = this->new_model();
    auto x = model.add_variable({.upper_bound = 3});
    auto y = model.add_variable({.lower_bound = 1});
    model.set_maximization();
    model.set_objective(2 * x + 1.5 * y);
    auto c = model.add_constraint(x + y <= 5);
    model.solve();
    ASSERT_TRUE(model.is_optimal());
    ASSERT_FALSE(model.is_infeasible());
    ASSERT_FALSE(model.is_unbounded());
    ASSERT_DOUBLE_EQ(model.get_solution_value(), 9.0);
    auto solution = model.get_solution();
    ASSERT_DOUBLE_EQ(solution[x], 3.0);
    ASSERT_DOUBLE_EQ(solution[y], 2.0);
}
TYPED_TEST_P(LpStatusTest, min_bounded) {
    using namespace operators;
    auto model = this->new_model();
    auto x = model.add_variable({.upper_bound = 3});
    auto y = model.add_variable({.lower_bound = 1});
    model.set_minimization();
    model.set_objective(-2 * x - 1.5 * y);
    auto c = model.add_constraint(x + y <= 5);
    model.solve();
    ASSERT_TRUE(model.is_optimal());
    ASSERT_FALSE(model.is_infeasible());
    ASSERT_FALSE(model.is_unbounded());
    ASSERT_DOUBLE_EQ(model.get_solution_value(), -9.0);
    auto solution = model.get_solution();
    ASSERT_DOUBLE_EQ(solution[x], 3.0);
    ASSERT_DOUBLE_EQ(solution[y], 2.0);
}
TYPED_TEST_P(LpStatusTest, max_unbounded) {
    using namespace operators;
    auto model = this->new_model();
    auto x = model.add_variable({.upper_bound = 3});
    auto y = model.add_variable({.lower_bound = 1});
    model.set_maximization();
    model.set_objective(-2 * x + -1.5 * y);
    model.add_constraint(x + y <= 5);
    model.solve();
    ASSERT_FALSE(model.is_optimal());
    ASSERT_FALSE(model.is_infeasible());
    ASSERT_TRUE(model.is_unbounded());
}
TYPED_TEST_P(LpStatusTest, min_unbounded) {
    using namespace operators;
    auto model = this->new_model();
    auto x = model.add_variable({.upper_bound = 3});
    auto y = model.add_variable({.lower_bound = 1});
    model.set_minimization();
    model.set_objective(2 * x + 1.5 * y);
    model.add_constraint(x + y <= 5);
    model.solve();
    ASSERT_FALSE(model.is_optimal());
    ASSERT_FALSE(model.is_infeasible());
    ASSERT_TRUE(model.is_unbounded());
}
TYPED_TEST_P(LpStatusTest, max_infeasible) {
    using namespace operators;
    auto model = this->new_model();
    auto x = model.add_variable();
    auto y = model.add_variable({.upper_bound = 1});
    model.set_maximization();
    model.set_objective(-2 * x + -1.5 * y);
    model.add_constraint(y >= -2 + 2 * x);
    model.add_constraint(y >= 3 - x);
    model.solve();
    ASSERT_FALSE(model.is_optimal());
    ASSERT_TRUE(model.is_infeasible());
    ASSERT_FALSE(model.is_unbounded());
}
TYPED_TEST_P(LpStatusTest, min_infeasible) {
    using namespace operators;
    auto model = this->new_model();
    auto x = model.add_variable();
    auto y = model.add_variable({.upper_bound = 1});
    model.set_minimization();
    model.set_objective(-2 * x + -1.5 * y);
    model.add_constraint(y >= -2 + 2 * x);
    model.add_constraint(y >= 3 - x);
    model.solve();
    ASSERT_FALSE(model.is_optimal());
    ASSERT_TRUE(model.is_infeasible());
    ASSERT_FALSE(model.is_unbounded());
}

REGISTER_TYPED_TEST_SUITE_P(LpStatusTest, not_solved, max_bounded, min_bounded,
                            max_unbounded, min_unbounded, max_infeasible,
                            min_infeasible);

}  // namespace fhamonic::mippp