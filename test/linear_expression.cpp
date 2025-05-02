#undef NDEBUG
#include <gtest/gtest.h>

#include "mippp/linear_expression.hpp"
#include "mippp/model_entities.hpp"

#include "assert_helper.hpp"

using namespace fhamonic::mippp;
using namespace fhamonic::mippp::operators;

using Var = model_variable<int, double>;

GTEST_TEST(linear_expression_operators, negate_term) {
    ASSERT_LIN_EXPR((-Var(11)) * 3.2 + 13, {Var(11)}, {-3.2}, 13);
    ASSERT_LIN_EXPR(-(Var(11) * 3.2 + 13), {Var(11)}, {-3.2}, -13);
}

GTEST_TEST(linear_expression_operators, scalar_add) {
    ASSERT_LIN_EXPR(Var(11) * 3.2 + 5, {Var(11)}, {3.2}, 5);
}
GTEST_TEST(linear_expression_operators, scalar_add2) {
    ASSERT_LIN_EXPR(5 + Var(11) * 3.2, {Var(11)}, {3.2}, 5);
}

GTEST_TEST(linear_expression_operators, scalar_mul) {
    ASSERT_LIN_EXPR((Var(11) * 3.2) * -2, {Var(11)}, {-6.4}, 0);
}
GTEST_TEST(linear_expression_operators, scalar_mul2) {
    ASSERT_LIN_EXPR(-2 * (Var(11) * 3.2), {Var(11)}, {-6.4}, 0);
}
GTEST_TEST(linear_expression_operators, scalar_div) {
    ASSERT_LIN_EXPR((Var(11) * 3.2) / (-0.5), {Var(11)}, {-6.4}, 0);
}

GTEST_TEST(linear_expression_operators, add_terms) {
    ASSERT_LIN_EXPR(Var(1) * 3.2 + Var(2) * 1.5, {Var(1), Var(2)}, {3.2, 1.5},
                      0);
}
GTEST_TEST(linear_expression_operators, substract_terms) {
    ASSERT_LIN_EXPR(Var(1) * 3.2 - Var(2) * 1.5, {Var(1), Var(2)},
                      {3.2, -1.5}, 0);
}

GTEST_TEST(linear_expression_operators, lvalues_tests) {
    Var x = Var(27);
    Var y = Var(11);
    auto s = x + y;
    ASSERT_LIN_EXPR(3 * s, {Var(27), Var(11)}, {3.0, 3.0}, 0);
}

GTEST_TEST(linear_expression_operators, xsum_test) {
    std::vector<Var> vars;
    auto e = xsum(vars);
    static_assert(linear_expression<decltype(e)>);
    static_assert(linear_expression<decltype(Var(2))>);
    auto e1 = e + Var(13);
    ASSERT_LIN_EXPR(e1, {Var(13)}, {1.0}, 0);
}
