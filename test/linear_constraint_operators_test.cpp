#include <gtest/gtest.h>

#include "mippp/constraints/linear_constraint_operators.hpp"
#include "mippp/expressions/linear_expression_operators.hpp"
#include "mippp/variable.hpp"

#include "assert_eq_ranges.hpp"

using namespace fhamonic::mippp;

using Var = variable<int, double>;

GTEST_TEST(linear_constraint_operators, lower_bound) {
    ASSERT_EQ_RANGES((3 <= -Var(11) * 3.2).variables(), {11});
    ASSERT_EQ_RANGES((3 <= -Var(11) * 3.2).coefficients(), {-3.2});
    ASSERT_EQ((3 <= -Var(11) * 3.2).lower_bound(), 3);
    ASSERT_EQ((3 <= -Var(11) * 3.2).upper_bound(),
              std::numeric_limits<double>::max());
}

GTEST_TEST(linear_constraint_operators, lower_bound_other_way) {
    ASSERT_EQ_RANGES((-Var(11) * 3.2 >= 3).variables(), {11});
    ASSERT_EQ_RANGES((-Var(11) * 3.2 >= 3).coefficients(), {-3.2});
    ASSERT_EQ((3 <= -Var(11) * 3.2).lower_bound(), 3);
    ASSERT_EQ((3 <= -Var(11) * 3.2).upper_bound(),
              std::numeric_limits<double>::max());
}

GTEST_TEST(linear_constraint_operators, upper_bound) {
    ASSERT_EQ_RANGES((-Var(11) * 3.2 <= 3).variables(), {11});
    ASSERT_EQ_RANGES((-Var(11) * 3.2 <= 3).coefficients(), {-3.2});
    ASSERT_EQ((-Var(11) * 3.2 <= 3).lower_bound(),
              std::numeric_limits<double>::lowest());
    ASSERT_EQ((-Var(11) * 3.2 <= 3).upper_bound(), 3);
}

GTEST_TEST(linear_constraint_operators, upper_bound_other_way) {
    ASSERT_EQ_RANGES((3 >= -Var(11) * 3.2).variables(), {11});
    ASSERT_EQ_RANGES((3 >= -Var(11) * 3.2).coefficients(), {-3.2});
    ASSERT_EQ((3 >= -Var(11) * 3.2).lower_bound(),
              std::numeric_limits<double>::lowest());
    ASSERT_EQ((3 >= -Var(11) * 3.2).upper_bound(), 3);
}

GTEST_TEST(linear_constraint_operators, bounds) {
    ASSERT_EQ_RANGES((1 <= -Var(11) * 3.2 <= 3).variables(), {11});
    ASSERT_EQ_RANGES((1 <= -Var(11) * 3.2 <= 3).coefficients(), {-3.2});
    ASSERT_EQ((3 >= -Var(11) * 3.2 >= 1).lower_bound(), 1);
    ASSERT_EQ((3 >= -Var(11) * 3.2 >= 1).upper_bound(), 3);
}

GTEST_TEST(linear_constraint_operators, bounds_other_way) {
    ASSERT_EQ_RANGES((3 >= -Var(11) * 3.2 >= 1).variables(), {11});
    ASSERT_EQ_RANGES((3 >= -Var(11) * 3.2 >= 1).coefficients(), {-3.2});
    ASSERT_EQ((3 >= -Var(11) * 3.2 >= 1).lower_bound(), 1);
    ASSERT_EQ((3 >= -Var(11) * 3.2 >= 1).upper_bound(), 3);
}