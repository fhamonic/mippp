#pragma once
#undef NDEBUG
#include <gtest/gtest.h>

#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"

namespace fhamonic::mippp {

template <typename T>
struct NamedVariablesTest : public T {
    using typename T::model_type;
    static_assert(has_named_variables<model_type>);
};
TYPED_TEST_SUITE_P(NamedVariablesTest);

TYPED_TEST_P(NamedVariablesTest, set_variable_name) {
    auto model = this->new_model();
    auto x1 = model.add_variable();
    auto x2 = model.add_variable();
    model.set_variable_name(x1, "X1");
    model.set_variable_name(x2, "X2");
    std::cout << model.get_variable_name(x1).length() << std::endl;
    ASSERT_EQ(model.get_variable_name(x1), "X1");
    ASSERT_EQ(model.get_variable_name(x2), "X2");
}
TYPED_TEST_P(NamedVariablesTest, add_named_variable) {
    auto model = this->new_model();
    auto x1 = model.add_named_variable("X1");
    auto x2 = model.add_named_variable(
        "X2", {.obj_coef = 1, .lower_bound = 2, .upper_bound = 10});
    ASSERT_EQ(model.num_variables(), 2);
    ASSERT_EQ(model.num_constraints(), 0);
    ASSERT_EQ(x1.id(), 0);
    ASSERT_EQ(x2.id(), 1);
    ASSERT_EQ(model.get_variable_name(x1), "X1");
    ASSERT_EQ(model.get_variable_name(x2), "X2");
}
TYPED_TEST_P(NamedVariablesTest, add_named_variables) {
    auto model = this->new_model();
    auto x = model.add_named_variables(
        1, [](auto i) { return std::string("X") + std::to_string(i); });
    auto y = model.add_named_variables(
        3, [](auto i) { return std::string("Y") + std::to_string(i); },
        {.obj_coef = 1, .lower_bound = 2, .upper_bound = 10});
    ASSERT_EQ(model.num_variables(), 4);
    ASSERT_EQ(model.num_constraints(), 0);
    ASSERT_EQ(x[0].id(), 0);
    ASSERT_EQ(y[0].id(), 1);
    ASSERT_EQ(y[1].id(), 2);
    ASSERT_EQ(y[2].id(), 3);
    ASSERT_THROW(y[3], std::out_of_range);
    ASSERT_EQ(model.get_variable_name(x[0]), "X0");
    ASSERT_EQ(model.get_variable_name(y[0]), "Y0");
    ASSERT_EQ(model.get_variable_name(y[1]), "Y1");
    ASSERT_EQ(model.get_variable_name(y[2]), "Y2");
}
TYPED_TEST_P(NamedVariablesTest, add_indexed_named_variables) {
    auto model = this->new_model();
    auto x = model.add_named_variables(
        1, [](int i) { return i; },
        [](int i) { return std::string("X") + std::to_string(i); });
    auto y = model.add_named_variables(
        3, [](int i, int j) { return 2 * i + j; },
        [](int i, int j) {
            return std::string("Y") + std::to_string(i) + "_" +
                   std::to_string(j);
        },
        {.obj_coef = 1, .lower_bound = 2, .upper_bound = 10});
    ASSERT_EQ(model.num_variables(), 4);
    ASSERT_EQ(model.num_constraints(), 0);
    ASSERT_EQ(x(0).id(), 0);
    ASSERT_EQ(y(0, 0).id(), 1);
    ASSERT_EQ(y(0, 1).id(), 2);
    ASSERT_EQ(y(1, 0).id(), 3);
    ASSERT_THROW(y(0, -1), std::out_of_range);
    ASSERT_THROW(y(1, 1), std::out_of_range);
    ASSERT_EQ(model.get_variable_name(x(0)), "X0");
    ASSERT_EQ(model.get_variable_name(y(0, 0)), "Y0_0");
    ASSERT_EQ(model.get_variable_name(y(0, 1)), "Y0_1");
    ASSERT_EQ(model.get_variable_name(y(1, 0)), "Y1_0");
}

REGISTER_TYPED_TEST_SUITE_P(NamedVariablesTest, set_variable_name,
                            add_named_variable, add_named_variables,
                            add_indexed_named_variables);

}  // namespace fhamonic::mippp