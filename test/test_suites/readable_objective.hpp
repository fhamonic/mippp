#pragma once
#undef NDEBUG
#include <gtest/gtest.h>

#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"

#include "../assert_helper.hpp"

namespace fhamonic::mippp {

template <typename T>
struct ReadableObjectiveTest : public ::testing::Test, public T {
    using typename T::model_type;
    static_assert(has_readable_objective<model_type>);
};
TYPED_TEST_SUITE_P(ReadableObjectiveTest);

TYPED_TEST_P(ReadableObjectiveTest, get_objective_offset) {
    using namespace operators;
    auto model = this->construct_model();
    auto x = model.add_variable();
    model.set_objective(123 + x);
    ASSERT_EQ(model.get_objective_offset(), 123.0);
}
TYPED_TEST_P(ReadableObjectiveTest, get_objective_coefficient) {
    using namespace operators;
    auto model = this->construct_model();
    auto x = model.add_variable();
    model.set_objective(3 * x);
    auto y = model.add_variable({.obj_coef = 2.0});
    ASSERT_EQ(model.get_objective_coefficient(x), 3.0);
    ASSERT_EQ(model.get_objective_coefficient(y), 2.0);
}
TYPED_TEST_P(ReadableObjectiveTest, get_objective) {
    using namespace operators;
    auto model = this->construct_model();
    auto x = model.add_variable();
    auto y = model.add_variable();
    auto z = model.add_variable();
    model.set_objective(123 + 3 * x - 5 * y + z);
    ASSERT_LIN_EXPR(model.get_objective(), {x, y, z}, {3.0, -5.0, 1.0}, 123.0);
}

REGISTER_TYPED_TEST_SUITE_P(ReadableObjectiveTest, get_objective_offset,
                            get_objective_coefficient, get_objective);

}  // namespace fhamonic::mippp