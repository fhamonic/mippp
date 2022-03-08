#include <gtest/gtest.h>

#include "mippp/expressions/linear_expression_operations.hpp"
#include "mippp/expressions/variable.hpp"

#include "assert_eq_ranges.hpp"

using namespace fhamonic::mippp;

using Var = variable<int, double>;

GTEST_TEST(variable, ctor) {
    auto v = Var(11);
    ASSERT_EQ_RANGES(v.variables(), {11});
    ASSERT_EQ_RANGES(v.coefficients(), {1.0});
    ASSERT_EQ(v.constant(), 0);
}

GTEST_TEST(variable, negate_var) {
    ASSERT_EQ_RANGES(linear_expression_negate(Var(11)).variables(), {11});
    ASSERT_EQ_RANGES(linear_expression_negate(Var(11)).coefficients(), {-1.0});
    ASSERT_EQ(linear_expression_negate(Var(11)).constant(), 0);
}

GTEST_TEST(variable, var_scalar_add) {
    ASSERT_EQ_RANGES(linear_expression_scalar_add(Var(11), 5).variables(),
                     {11});
    ASSERT_EQ_RANGES(linear_expression_scalar_add(Var(11), 5).coefficients(),
                     {1.0});
    ASSERT_EQ(linear_expression_scalar_add(Var(11), 5).constant(), 5);
}

GTEST_TEST(variable, scalar_mul) {
    ASSERT_EQ_RANGES(linear_expression_scalar_mul(Var(11), -2.5).variables(),
                     {11});
    ASSERT_EQ_RANGES(
        linear_expression_scalar_mul(Var(11), -2.5).coefficients(), {-2.5});
    ASSERT_EQ(linear_expression_scalar_mul(Var(11), -2.5).constant(), 0);
}

GTEST_TEST(variable, add_vars) {
    ASSERT_EQ_RANGES(
        linear_expression_add(Var(11), Var(2)).variables(), {11, 2});
    ASSERT_EQ_RANGES(
        linear_expression_add(Var(11), Var(2)).coefficients(),
        {1.0, 1.0});
    ASSERT_EQ(linear_expression_add(Var(11), Var(2)).constant(), 0);
}

GTEST_TEST(variable, add_negative_var) {
    ASSERT_EQ_RANGES(linear_expression_add(
                         Var(3), linear_expression_negate(Var(12)))
                         .variables(),
                     {3, 12});
    ASSERT_EQ_RANGES(linear_expression_add(
                         Var(3), linear_expression_negate(Var(12)))
                         .coefficients(),
                     {1.0, -1.0});
    ASSERT_EQ(linear_expression_add(Var(3),
                                    linear_expression_negate(Var(12)))
                  .constant(),
              0);
}