#undef NDEBUG
#include <gtest/gtest.h>

#include "mippp/linear_expression.hpp"
#include "mippp/variable.hpp"

#include "assert_expression.hpp"

using namespace fhamonic::mippp;
using namespace fhamonic::mippp::operators;

using Var = variable<int, double>;

GTEST_TEST(linear_expression_operators, negate_term) {
    ASSERT_EXPRESSION((-Var(11)) * 3.2 + 13, {11}, {-3.2}, 13);
    ASSERT_EXPRESSION(-(Var(11) * 3.2 + 13), {11}, {-3.2}, -13);
}

GTEST_TEST(linear_expression_operators, scalar_add) {
    ASSERT_EXPRESSION(Var(11) * 3.2 + 5, {11}, {3.2}, 5);
}

GTEST_TEST(linear_expression_operators, scalar_add_other_way) {
    ASSERT_EXPRESSION(5 + Var(11) * 3.2, {11}, {3.2}, 5);
}

GTEST_TEST(linear_expression_operators, scalar_mul) {
    ASSERT_EXPRESSION((Var(11) * 3.2) * -2, {11}, {-6.4}, 0);
}

GTEST_TEST(linear_expression_operators, scalar_mul_other_way) {
    ASSERT_EXPRESSION(-2 * (Var(11) * 3.2), {11}, {-6.4}, 0);
}

GTEST_TEST(linear_expression_operators, add_terms) {
    ASSERT_EXPRESSION(Var(1) * 3.2 + Var(2) * 1.5, {1, 2}, {3.2, 1.5}, 0);
}

GTEST_TEST(linear_expression_operators, substract_terms) {
    ASSERT_EXPRESSION(Var(1) * 3.2 - Var(2) * 1.5, {1, 2}, {3.2, -1.5}, 0);
}

GTEST_TEST(linear_expression_operators, lvalues_tests) {
    Var x = Var(27);
    Var y = Var(11);
    auto s = x + y;
    ASSERT_EXPRESSION(3 * s, {27, 11}, {3.0, 3.0}, 0);
}

GTEST_TEST(linear_expression_operators, xsum_test) {
    std::vector<Var> vars;
    auto e = xsum(vars);
    static_assert(linear_expression<decltype(e)>);
    static_assert(linear_expression<decltype(Var(2))>);
    auto e1 = e + Var(13);
    ASSERT_EXPRESSION(e1, {13}, {1.0}, 0);
}
