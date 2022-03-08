#include <gtest/gtest.h>

#include "mippp/expressions/linear_expression_operations.hpp"
#include "mippp/expressions/linear_term.hpp"

#include "ranges_test_helper.hpp"

using namespace fhamonic::mippp;

GTEST_TEST(linear_term, ctor) {
    auto t = linear_term(11, 3.2);
    ASSERT_EQ_RANGES(t.variables(), {11});
    ASSERT_EQ_RANGES(t.coefficients(), {3.2});
    ASSERT_EQ(t.constant(), 0);
}

GTEST_TEST(linear_term, negate_term) {
    ASSERT_EQ_RANGES(linear_expression_negate(linear_term(11, 3.2)).variables(),
                     {11});
    ASSERT_EQ_RANGES(
        linear_expression_negate(linear_term(11, 3.2)).coefficients(), {-3.2});
    ASSERT_EQ(linear_expression_negate(linear_term(11, 3.2)).constant(), 0);
}

GTEST_TEST(linear_term, scalar_add) {
    ASSERT_EQ_RANGES(
        linear_expression_scalar_add(linear_term(11, 3.2), 5).variables(),
        {11});
    ASSERT_EQ_RANGES(
        linear_expression_scalar_add(linear_term(11, 3.2), 5).coefficients(),
        {3.2});
    ASSERT_EQ(linear_expression_scalar_add(linear_term(11, 3.2), 5).constant(),
              5);
}

GTEST_TEST(linear_term, scalar_mul) {
    ASSERT_EQ_RANGES(
        linear_expression_scalar_mul(linear_term(11, 3.2), -2).variables(),
        {11});
    ASSERT_EQ_RANGES(
        linear_expression_scalar_mul(linear_term(11, 3.2), -2).coefficients(),
        {-6.4});
    ASSERT_EQ(linear_expression_scalar_mul(linear_term(11, 3.2), -2).constant(),
              0);
}

GTEST_TEST(linear_term, add_terms) {
    ASSERT_EQ_RANGES(
        linear_expression_add(linear_term(1, 3.2), linear_term(2, 1.5))
            .variables(),
        {1, 2});
    ASSERT_EQ_RANGES(
        linear_expression_add(linear_term(1, 3.2), linear_term(2, 1.5))
            .coefficients(),
        {3.2, 1.5});
    ASSERT_EQ(linear_expression_add(linear_term(1, 3.2), linear_term(2, 1.5))
                  .constant(),
              0);
}

GTEST_TEST(linear_term, add_negative_terms) {
    ASSERT_EQ_RANGES(
        linear_expression_add(linear_term(1, 3.2),
                              linear_expression_negate(linear_term(12, 1.5)))
            .variables(),
        {1, 12});
    ASSERT_EQ_RANGES(
        linear_expression_add(linear_term(1, 3.2),
                              linear_expression_negate(linear_term(12, 1.5)))
            .coefficients(),
        {3.2, -1.5});
    ASSERT_EQ(
        linear_expression_add(linear_term(1, 3.2),
                              linear_expression_negate(linear_term(12, 1.5)))
            .constant(),
        0);
}