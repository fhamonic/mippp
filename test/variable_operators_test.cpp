#include <gtest/gtest.h>

#include "mippp/expressions/linear_expression_operators.hpp"
#include "mippp/expressions/variable.hpp"

#include "assert_eq_ranges.hpp"

using namespace fhamonic::mippp;

using Var = variable<int, double>;

GTEST_TEST(variable_operators, negate_var) {
    ASSERT_EQ_RANGES((-Var(11)).variables(), {11});
    ASSERT_EQ_RANGES((-Var(11)).coefficients(), {-1.0});
    ASSERT_EQ((-Var(11)).constant(), 0);
}

GTEST_TEST(variable_operators, var_scalar_add) {
    ASSERT_EQ_RANGES((Var(11) + 5).variables(), {11});
    ASSERT_EQ_RANGES((Var(11) + 5).coefficients(), {1.0});
    ASSERT_EQ((Var(11) + 5).constant(), 5);
}
GTEST_TEST(variable_operators, var_scalar_add_other_way) {
    ASSERT_EQ_RANGES((5 + Var(11)).variables(), {11});
    ASSERT_EQ_RANGES((5 + Var(11)).coefficients(), {1.0});
    ASSERT_EQ((5 + Var(11)).constant(), 5);
}

GTEST_TEST(variable_operators, var_scalar_sub) {
    ASSERT_EQ_RANGES((Var(11) - 5).variables(), {11});
    ASSERT_EQ_RANGES((Var(11) - 5).coefficients(), {1.0});
    ASSERT_EQ((Var(11) - 5).constant(), -5);
}

GTEST_TEST(variable_operators, var_scalar_sub_other_way) {
    ASSERT_EQ_RANGES((5 - Var(11)).variables(), {11});
    ASSERT_EQ_RANGES((5 - Var(11)).coefficients(), {-1.0});
    ASSERT_EQ((5 - Var(11)).constant(), 5);
}

GTEST_TEST(variable_operators, scalar_mul) {
    ASSERT_EQ_RANGES((Var(11) * -2.5).variables(), {11});
    ASSERT_EQ_RANGES((Var(11) * -2.5).coefficients(), {-2.5});
    ASSERT_EQ((Var(11) * -2.5).constant(), 0);
}

GTEST_TEST(variable_operators, scalar_mul_other_way) {
    ASSERT_EQ_RANGES((-2.5 * Var(11)).variables(), {11});
    ASSERT_EQ_RANGES((-2.5 * Var(11)).coefficients(), {-2.5});
    ASSERT_EQ((-2.5 * Var(11)).constant(), 0);
}

GTEST_TEST(variable_operators, add_vars) {
    ASSERT_EQ_RANGES((Var(11) + Var(2)).variables(), {11, 2});
    ASSERT_EQ_RANGES((Var(11) + Var(2)).coefficients(), {1.0, 1.0});
    ASSERT_EQ((Var(11) + Var(2)).constant(), 0);
}

GTEST_TEST(variable_operators, add_negative_var) {
    ASSERT_EQ_RANGES((Var(3) + (-Var(12))).variables(), {3, 12});
    ASSERT_EQ_RANGES((Var(3) + (-Var(12))).coefficients(), {1.0, -1.0});
    ASSERT_EQ((Var(3) + (-Var(12))).constant(), 0);
}
GTEST_TEST(variable_operators, add_negative_var_other_way) {
    ASSERT_EQ_RANGES((-Var(12) + Var(3)).variables(), {12, 3});
    ASSERT_EQ_RANGES((-Var(12) + Var(3)).coefficients(), {-1.0, 1.0});
    ASSERT_EQ((-Var(12) + Var(3)).constant(), 0);
}

GTEST_TEST(variable_operators, substract_var) {
    ASSERT_EQ_RANGES((Var(3) - Var(12)).variables(), {3, 12});
    ASSERT_EQ_RANGES((Var(3) - Var(12)).coefficients(), {1.0, -1.0});
    ASSERT_EQ((Var(3) - Var(12)).constant(), 0);
}