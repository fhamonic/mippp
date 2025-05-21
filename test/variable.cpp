#undef NDEBUG
#include <gtest/gtest.h>

#include "mippp/linear_expression.hpp"
#include "mippp/model_entities.hpp"

#include "assert_helper.hpp"

using namespace fhamonic::mippp;

using Var = model_variable<int, double>;

GTEST_TEST(variable, ctor) {
    auto v = Var(11);
    ASSERT_LIN_EXPR(v, {Var(11)}, {1.0}, 0);
}

GTEST_TEST(variable, negate_var) {
    ASSERT_LIN_EXPR(linear_expression_negate(Var(11)), {Var(11)}, {-1.0}, 0);
}

GTEST_TEST(variable, var_scalar_add) {
    ASSERT_LIN_EXPR(linear_expression_scalar_add(Var(11), 5), {Var(11)},
                      {1.0}, 5);
}

GTEST_TEST(variable, scalar_mul) {
    ASSERT_LIN_EXPR(linear_expression_scalar_mul(Var(11), -2.5), {Var(11)},
                      {-2.5}, 0);
}

GTEST_TEST(variable, add_variables) {
    ASSERT_LIN_EXPR(linear_expression_add(Var(11), Var(2)), {Var(11), Var(2)},
                      {1.0, 1.0}, 0);
}

GTEST_TEST(variable, add_negative_var) {
    ASSERT_LIN_EXPR(
        linear_expression_add(Var(3), linear_expression_negate(Var(12))),
        {Var(3), Var(12)}, {1.0, -1.0}, 0);
}

using namespace fhamonic::mippp::operators;

GTEST_TEST(variable_operators, negate_var) {
    ASSERT_LIN_EXPR(-Var(11), {Var(11)}, {-1.0}, 0);
}

GTEST_TEST(variable_operators, var_scalar_add) {
    ASSERT_LIN_EXPR(Var(11) + 5, {Var(11)}, {1.0}, 5);
}

GTEST_TEST(variable_operators, var_scalar_add_other_way) {
    ASSERT_LIN_EXPR(5 + Var(11), {Var(11)}, {1.0}, 5);
}

GTEST_TEST(variable_operators, var_scalar_sub) {
    ASSERT_LIN_EXPR(Var(11) - 5, {Var(11)}, {1.0}, -5);
}

GTEST_TEST(variable_operators, var_scalar_sub_other_way) {
    ASSERT_LIN_EXPR(5 - Var(11), {Var(11)}, {-1.0}, 5);
}

GTEST_TEST(variable_operators, scalar_mul) {
    ASSERT_LIN_EXPR(Var(11) * -2.5, {Var(11)}, {-2.5}, 0);
}

GTEST_TEST(variable_operators, scalar_mul_other_way) {
    ASSERT_LIN_EXPR(-2.5 * Var(11), {Var(11)}, {-2.5}, 0);
}

GTEST_TEST(variable_operators, add_variables) {
    ASSERT_LIN_EXPR(Var(11) + Var(2), {Var(11), Var(2)}, {1.0, 1.0}, 0);
}

GTEST_TEST(variable_operators, add_negative_var) {
    ASSERT_LIN_EXPR(Var(3) + (-Var(12)), {Var(3), Var(12)}, {1.0, -1.0}, 0);
}

GTEST_TEST(variable_operators, add_negative_var_other_way) {
    ASSERT_LIN_EXPR(-Var(12) + Var(3), {Var(12), Var(3)}, {-1.0, 1.0}, 0);
}

GTEST_TEST(variable_operators, substract_var) {
    ASSERT_LIN_EXPR(Var(3) - Var(12), {Var(3), Var(12)}, {1.0, -1.0}, 0);
}