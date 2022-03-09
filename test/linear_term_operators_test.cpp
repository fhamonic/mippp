#include <gtest/gtest.h>

#include "mippp/expressions/linear_expression_operators.hpp"
#include "mippp/expressions/variable.hpp"

#include "assert_eq_ranges.hpp"

using namespace fhamonic::mippp;

using Var = variable<int, double>;

GTEST_TEST(linear_terms_operators, negate_term) {
    ASSERT_EQ_RANGES((-(Var(11) * 3.2)).variables(), {11});
    ASSERT_EQ_RANGES((-(Var(11) * 3.2)).coefficients(), {-3.2});
    ASSERT_EQ((-(Var(11) * 3.2)).constant(), 0);
}

GTEST_TEST(linear_terms_operators, scalar_add) {
    ASSERT_EQ_RANGES((Var(11) * 3.2 + 5).variables(), {11});
    ASSERT_EQ_RANGES((Var(11) * 3.2 + 5).coefficients(), {3.2});
    ASSERT_EQ((Var(11) * 3.2 + 5).constant(), 5);
}

GTEST_TEST(linear_terms_operators, scalar_add_other_way) {
    ASSERT_EQ_RANGES((5 + Var(11) * 3.2).variables(), {11});
    ASSERT_EQ_RANGES((5 + Var(11) * 3.2).coefficients(), {3.2});
    ASSERT_EQ((5 + Var(11) * 3.2).constant(), 5);
}

GTEST_TEST(linear_terms_operators, scalar_mul) {
    ASSERT_EQ_RANGES(((Var(11) * 3.2) * -2).variables(), {11});
    ASSERT_EQ_RANGES(((Var(11) * 3.2) * -2).coefficients(), {-6.4});
    ASSERT_EQ(((Var(11) * 3.2) * -2).constant(), 0);
}

GTEST_TEST(linear_terms_operators, scalar_mul_other_way) {
    ASSERT_EQ_RANGES((-2 * (Var(11) * 3.2)).variables(), {11});
    ASSERT_EQ_RANGES((-2 * (Var(11) * 3.2)).coefficients(), {-6.4});
    ASSERT_EQ((-2 * (Var(11) * 3.2)).constant(), 0);
}

GTEST_TEST(linear_terms_operators, add_terms) {
    ASSERT_EQ_RANGES((Var(1) * 3.2 + Var(2) * 1.5).variables(), {1, 2});
    ASSERT_EQ_RANGES((Var(1) * 3.2 + Var(2) * 1.5).coefficients(), {3.2, 1.5});
    ASSERT_EQ((Var(1) * 3.2 + Var(2) * 1.5).constant(), 0);
}

GTEST_TEST(linear_terms_operators, substract_terms) {
    ASSERT_EQ_RANGES((Var(1) * 3.2 - Var(2) * 1.5).variables(), {1, 2});
    ASSERT_EQ_RANGES((Var(1) * 3.2 - Var(2) * 1.5).coefficients(), {3.2, -1.5});
    ASSERT_EQ((Var(1) * 3.2 + Var(2) * 1.5).constant(), 0);
}


GTEST_TEST(linear_terms_operators, lvalues_tests) {
    Var x = Var(27);
    Var y = Var(11);
    auto s = x + y;
    
    ASSERT_EQ_RANGES((3 * s).variables(), {27, 11});
    ASSERT_EQ_RANGES((3 * s).coefficients(), {3.0, 3.0});
    ASSERT_EQ((3 * s).constant(), 0);
}