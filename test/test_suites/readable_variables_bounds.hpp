#pragma once
#undef NDEBUG
#include <gtest/gtest.h>

#include "mippp/model_concepts.hpp"

namespace fhamonic::mippp {

template <typename T>
struct ReadableVariablesBoundsTest : public T {
    using typename T::model_type;
    static_assert(has_readable_variables_bounds<model_type>);
};
TYPED_TEST_SUITE_P(ReadableVariablesBoundsTest);

TYPED_TEST_P(ReadableVariablesBoundsTest, get_variable_lower_bound) {
    auto model = this->construct_model();
    auto x1 = model.add_variable();
    auto x2 = model.add_variable({.lower_bound = 2.5});
    auto x3 = model.add_variable({});
    auto xs1 = model.add_variables(1);
    auto xs2 = model.add_variables(1, {.lower_bound = 3.5});
    auto xs3 = model.add_variables(1, {});
    auto xis1 = model.add_variables(1, [](int i) { return i; });
    auto xis2 =
        model.add_variables(1, [](int i) { return i; }, {.lower_bound = 4.5});
    auto xis3 = model.add_variables(1, [](int i) { return i; }, {});
    ASSERT_EQ(model.get_variable_lower_bound(x1), 0.0);
    ASSERT_EQ(model.get_variable_lower_bound(x2), 2.5);
    ASSERT_LE(model.get_variable_lower_bound(x3), -TEST_INFINITY);
    ASSERT_EQ(model.get_variable_lower_bound(xs1[0]), 0.0);
    ASSERT_EQ(model.get_variable_lower_bound(xs2[0]), 3.5);
    ASSERT_LE(model.get_variable_lower_bound(xs3[0]), -TEST_INFINITY);
    ASSERT_EQ(model.get_variable_lower_bound(xis1(0)), 0.0);
    ASSERT_EQ(model.get_variable_lower_bound(xis2(0)), 4.5);
    ASSERT_LE(model.get_variable_lower_bound(xis3(0)), -TEST_INFINITY);
}
TYPED_TEST_P(ReadableVariablesBoundsTest, get_variable_upper_bound) {
    auto model = this->construct_model();
    auto x1 = model.add_variable();
    auto x2 = model.add_variable({.upper_bound = 2.5});
    auto x3 = model.add_variable({});
    auto xs1 = model.add_variables(1);
    auto xs2 = model.add_variables(1, {.upper_bound = 3.5});
    auto xs3 = model.add_variables(1, {});
    auto xis1 = model.add_variables(1, [](int i) { return i; });
    auto xis2 =
        model.add_variables(1, [](int i) { return i; }, {.upper_bound = 4.5});
    auto xis3 = model.add_variables(1, [](int i) { return i; }, {});
    ASSERT_GE(model.get_variable_upper_bound(x1), TEST_INFINITY);
    ASSERT_EQ(model.get_variable_upper_bound(x2), 2.5);
    ASSERT_GE(model.get_variable_upper_bound(x3), TEST_INFINITY);
    ASSERT_GE(model.get_variable_upper_bound(xs1[0]), TEST_INFINITY);
    ASSERT_EQ(model.get_variable_upper_bound(xs2[0]), 3.5);
    ASSERT_GE(model.get_variable_upper_bound(xs3[0]), TEST_INFINITY);
    ASSERT_GE(model.get_variable_upper_bound(xis1(0)), TEST_INFINITY);
    ASSERT_EQ(model.get_variable_upper_bound(xis2(0)), 4.5);
    ASSERT_GE(model.get_variable_upper_bound(xis3(0)), TEST_INFINITY);
}

REGISTER_TYPED_TEST_SUITE_P(ReadableVariablesBoundsTest,
                            get_variable_lower_bound, get_variable_upper_bound);

}  // namespace fhamonic::mippp