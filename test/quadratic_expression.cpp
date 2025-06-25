#undef NDEBUG
#include <gtest/gtest.h>

#include "mippp/model_entities.hpp"
#include "mippp/quadratic_expression.hpp"

#include "assert_helper.hpp"

using namespace fhamonic::mippp;
using namespace fhamonic::mippp::operators;

using Var = model_variable<int, double>;

GTEST_TEST(linear_expression_operators, square) {
    auto quadexpr = (Var(2) - Var(11) * 3.2 + 13) ^ 2;
    ASSERT_QUAD_TERMS(quadexpr.quadratic_terms(), {{Var(2), Var(2), 1.0},
                                                   {Var(11), Var(11), 10.24},
                                                   {Var(2), Var(11), -6.4}});
    ASSERT_LIN_EXPR(quadexpr.linear_expression(),
                    {{Var(2), 26.0}, {Var(11), -83.2}}, 169);
}

GTEST_TEST(linear_expression_operators, mul_terms) {
    auto quadexpr = (-Var(11) * 3.2 + 13) * (Var(7) + Var(5) + 2);
    ASSERT_QUAD_TERMS(quadexpr.quadratic_terms(),
                      {{Var(11), Var(7), -3.2}, {Var(11), Var(5), -3.2}});
    ASSERT_LIN_EXPR(quadexpr.linear_expression(),
                    {{Var(11), -6.4}, {Var(7), 13.0}, {Var(5), 13.0}}, 26);
}

GTEST_TEST(quadratic_expression_operators, add_terms) {
    auto e1 = (-Var(11) * 3.2 + 13) * (Var(7) + Var(5) + 2);
    auto e2 = (Var(8) * 7.1 + 9) * (Var(3) + 1);
    auto quadexpr = e1 + e2;
    ASSERT_QUAD_TERMS(quadexpr.quadratic_terms(), {{Var(11), Var(7), -3.2},
                                                   {Var(11), Var(5), -3.2},
                                                   {Var(8), Var(3), 7.1}});
    ASSERT_LIN_EXPR(quadexpr.linear_expression(),
                    {{Var(11), -6.4},
                     {Var(8), 7.1},
                     {Var(7), 13.0},
                     {Var(5), 13.0},
                     {Var(3), 9.0}},
                    35);
}

GTEST_TEST(quadratic_expression_operators, negate) {
    auto e = (-Var(11) * 3.2 + 13) * (Var(7) + Var(5) + 2);
    auto quadexpr = -e;
    ASSERT_QUAD_TERMS(quadexpr.quadratic_terms(),
                      {{Var(11), Var(7), 3.2}, {Var(11), Var(5), 3.2}});
    ASSERT_LIN_EXPR(quadexpr.linear_expression(),
                    {{Var(11), 6.4}, {Var(7), -13.0}, {Var(5), -13.0}}, -26.0);
}

GTEST_TEST(quadratic_expression_operators, scalar_add) {
    auto e = (-Var(11) * 3.2 + 13) * (Var(7) + Var(5) + 2);
    auto quadexpr = e + 18.0;
    ASSERT_QUAD_TERMS(quadexpr.quadratic_terms(),
                      {{Var(11), Var(7), -3.2}, {Var(11), Var(5), -3.2}});
    ASSERT_LIN_EXPR(quadexpr.linear_expression(),
                    {{Var(11), -6.4}, {Var(7), 13.0}, {Var(5), 13.0}}, 44.0);
}
GTEST_TEST(quadratic_expression_operators, scalar_add2) {
    auto e = (-Var(11) * 3.2 + 13) * (Var(7) + Var(5) + 2);
    auto quadexpr = 18.0 + e;
    ASSERT_QUAD_TERMS(quadexpr.quadratic_terms(),
                      {{Var(11), Var(7), -3.2}, {Var(11), Var(5), -3.2}});
    ASSERT_LIN_EXPR(quadexpr.linear_expression(),
                    {{Var(11), -6.4}, {Var(7), 13.0}, {Var(5), 13.0}}, 44.0);
}

GTEST_TEST(quadratic_expression_operators, scalar_sub) {
    auto e = (-Var(11) * 3.2 + 13) * (Var(7) + Var(5) + 2);
    auto quadexpr = e - (-18.0);
    ASSERT_QUAD_TERMS(quadexpr.quadratic_terms(),
                      {{Var(11), Var(7), -3.2}, {Var(11), Var(5), -3.2}});
    ASSERT_LIN_EXPR(quadexpr.linear_expression(),
                    {{Var(11), -6.4}, {Var(7), 13.0}, {Var(5), 13.0}}, 44.0);
}
GTEST_TEST(quadratic_expression_operators, scalar_sub2) {
    auto e = (-Var(11) * 3.2 + 13) * (Var(7) + Var(5) + 2);
    auto quadexpr = 18.0 - (-e);
    ASSERT_QUAD_TERMS(quadexpr.quadratic_terms(),
                      {{Var(11), Var(7), -3.2}, {Var(11), Var(5), -3.2}});
    ASSERT_LIN_EXPR(quadexpr.linear_expression(),
                    {{Var(11), -6.4}, {Var(7), 13.0}, {Var(5), 13.0}}, 44.0);
}

GTEST_TEST(quadratic_expression_operators, scalar_mul) {
    auto e = (-Var(11) * 3.2 + 13) * (Var(7) + Var(5) + 2);
    auto quadexpr = e * 3.0;
    ASSERT_QUAD_TERMS(quadexpr.quadratic_terms(),
                      {{Var(11), Var(7), -9.6}, {Var(11), Var(5), -9.6}});
    ASSERT_LIN_EXPR(quadexpr.linear_expression(),
                    {{Var(11), -19.2}, {Var(7), 39.0}, {Var(5), 39.0}}, 78.0);
}
GTEST_TEST(quadratic_expression_operators, scalar_mul2) {
    auto e = (-Var(11) * 3.2 + 13) * (Var(7) + Var(5) + 2);
    auto quadexpr = 3.0 * e;
    ASSERT_QUAD_TERMS(quadexpr.quadratic_terms(),
                      {{Var(11), Var(7), -9.6}, {Var(11), Var(5), -9.6}});
    ASSERT_LIN_EXPR(quadexpr.linear_expression(),
                    {{Var(11), -19.2}, {Var(7), 39.0}, {Var(5), 39.0}}, 78.0);
}

GTEST_TEST(quadratic_expression_operators, scalar_div) {
    auto e = (-Var(11) * 3.2 + 13) * (Var(7) + Var(5) + 2);
    auto quadexpr = e / (1.0 / 3.0);
    ASSERT_QUAD_TERMS(quadexpr.quadratic_terms(),
                      {{Var(11), Var(7), -9.6}, {Var(11), Var(5), -9.6}});
    ASSERT_LIN_EXPR(quadexpr.linear_expression(),
                    {{Var(11), -19.2}, {Var(7), 39.0}, {Var(5), 39.0}}, 78.0);
}

GTEST_TEST(quadratic_expression_operators, add_linear_expr) {
    auto e = (-Var(11) * 3.2 + 13) * (3 * Var(7) + 2);
    auto quadexpr = e + 6.0 * Var(4);
    ASSERT_QUAD_TERMS(quadexpr.quadratic_terms(), {{Var(11), Var(7), -9.6}});
    ASSERT_LIN_EXPR(quadexpr.linear_expression(),
                    {{Var(11), -6.4}, {Var(7), 39.0}, {Var(4), 6.0}}, 26.0);
}
GTEST_TEST(quadratic_expression_operators, add_linear_expr2) {
    auto e = (-Var(11) * 3.2 + 13) * (3 * Var(7) + 2);
    auto quadexpr = 6.0 * Var(4) + e;
    ASSERT_QUAD_TERMS(quadexpr.quadratic_terms(), {{Var(11), Var(7), -9.6}});
    ASSERT_LIN_EXPR(quadexpr.linear_expression(),
                    {{Var(11), -6.4}, {Var(7), 39.0}, {Var(4), 6.0}}, 26.0);
}
GTEST_TEST(quadratic_expression_operators, sub_linear_expr) {
    auto e = (-Var(11) * 3.2 + 13) * (3 * Var(7) + 2);
    auto quadexpr = e - 6.0 * Var(4);
    ASSERT_QUAD_TERMS(quadexpr.quadratic_terms(), {{Var(11), Var(7), -9.6}});
    ASSERT_LIN_EXPR(quadexpr.linear_expression(),
                    {{Var(11), -6.4}, {Var(7), 39.0}, {Var(4), -6.0}}, 26.0);
}
GTEST_TEST(quadratic_expression_operators, sub_linear_expr2) {
    auto e = (-Var(11) * 3.2 + 13) * (3 * Var(7) + 2);
    auto quadexpr = 6.0 * Var(4) - e;
    ASSERT_QUAD_TERMS(quadexpr.quadratic_terms(), {{Var(11), Var(7), 9.6}});
    ASSERT_LIN_EXPR(quadexpr.linear_expression(),
                    {{Var(11), 6.4}, {Var(7), -39.0}, {Var(4), 6.0}}, -26.0);
}