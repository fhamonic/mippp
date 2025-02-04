#undef NDEBUG
#include <gtest/gtest.h>

#include "mippp/linear_expression.hpp"
#include "mippp/variable.hpp"

#include "assert_expression.hpp"

using namespace fhamonic::mippp;
using namespace fhamonic::mippp::operators;

using Var = variable<int, double>;

GTEST_TEST(variable_operators, negate_var) {
    ASSERT_EXPRESSION(-Var(11), {11}, {-1.0}, 0);
}

GTEST_TEST(variable_operators, var_scalar_add) {
    ASSERT_EXPRESSION(Var(11) + 5, {11}, {1.0}, 5);
}

GTEST_TEST(variable_operators, var_scalar_add_other_way) {
    ASSERT_EXPRESSION(5 + Var(11), {11}, {1.0}, 5);
}

GTEST_TEST(variable_operators, var_scalar_sub) {
    ASSERT_EXPRESSION(Var(11) - 5, {11}, {1.0}, -5);
}

GTEST_TEST(variable_operators, var_scalar_sub_other_way) {
    ASSERT_EXPRESSION(5 - Var(11), {11}, {-1.0}, 5);
}

GTEST_TEST(variable_operators, scalar_mul) {
    ASSERT_EXPRESSION(Var(11) * -2.5, {11}, {-2.5}, 0);
}

GTEST_TEST(variable_operators, scalar_mul_other_way) {
    ASSERT_EXPRESSION(-2.5 * Var(11), {11}, {-2.5}, 0);
}

GTEST_TEST(variable_operators, add_variables) {
    ASSERT_EXPRESSION(Var(11) + Var(2), {11, 2}, {1.0, 1.0}, 0);
}

GTEST_TEST(variable_operators, add_negative_var) {
    ASSERT_EXPRESSION(Var(3) + (-Var(12)), {3, 12}, {1.0, -1.0}, 0);
}

GTEST_TEST(variable_operators, add_negative_var_other_way) {
    ASSERT_EXPRESSION(-Var(12) + Var(3), {12, 3}, {-1.0, 1.0}, 0);
}

GTEST_TEST(variable_operators, substract_var) {
    ASSERT_EXPRESSION(Var(3) - Var(12), {3, 12}, {1.0, -1.0}, 0);
}