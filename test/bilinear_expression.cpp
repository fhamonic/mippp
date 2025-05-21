#undef NDEBUG
#include <gtest/gtest.h>

#include "mippp/bilinear_expression.hpp"
#include "mippp/model_entities.hpp"

#include "assert_helper.hpp"

using namespace fhamonic::mippp;
using namespace fhamonic::mippp::operators;

using Var = model_variable<int, double>;

GTEST_TEST(linear_expression_operators, mul_terms) {
    auto bilin = (-Var(11) * 3.2 + 13) * (Var(7) + Var(5) + 2);
    ASSERT_BILIN_TERMS(bilin.bilinear_terms(), {Var(11), Var(11)},
                       {Var(7), Var(5)}, {-3.2, -3.2});
    ASSERT_LIN_TERMS(bilin.left_linear_terms(), {Var(11)}, {-6.4});
    ASSERT_LIN_TERMS(bilin.right_linear_terms(), {Var(7), Var(5)},
                     {13.0, 13.0});
    ASSERT_EQ(bilin.constant(), 26);
}

GTEST_TEST(bilinear_expression_operators, add_terms) {
    auto e1 = (-Var(11) * 3.2 + 13) * (Var(7) + Var(5) + 2);
    auto e2 = (Var(8) * 7.1 + 9) * (Var(3) + 1);
    auto bilin = e1 + e2;
    ASSERT_BILIN_TERMS(bilin.bilinear_terms(), {Var(11), Var(11), Var(8)},
                       {Var(7), Var(5), Var(3)}, {-3.2, -3.2, 7.1});
    ASSERT_LIN_TERMS(bilin.left_linear_terms(), {Var(11), Var(8)}, {-6.4, 7.1});
    ASSERT_LIN_TERMS(bilin.right_linear_terms(), {Var(7), Var(5), Var(3)},
                     {13.0, 13.0, 9.0});
    ASSERT_EQ(bilin.constant(), 35);
}

GTEST_TEST(bilinear_expression_operators, negate) {
    auto e = (-Var(11) * 3.2 + 13) * (Var(7) + Var(5) + 2);
    auto bilin = -e;
    ASSERT_BILIN_TERMS(bilin.bilinear_terms(), {Var(11), Var(11)},
                       {Var(7), Var(5)}, {3.2, 3.2});
    ASSERT_LIN_TERMS(bilin.left_linear_terms(), {Var(11)}, {6.4});
    ASSERT_LIN_TERMS(bilin.right_linear_terms(), {Var(7), Var(5)},
                     {-13.0, -13.0});
    ASSERT_EQ(bilin.constant(), -26);
}

GTEST_TEST(bilinear_expression_operators, scalar_add) {
    auto e = (-Var(11) * 3.2 + 13) * (Var(7) + Var(5) + 2);
    auto bilin = e + 18.0;
    ASSERT_BILIN_TERMS(bilin.bilinear_terms(), {Var(11), Var(11)},
                       {Var(7), Var(5)}, {-3.2, -3.2});
    ASSERT_LIN_TERMS(bilin.left_linear_terms(), {Var(11)}, {-6.4});
    ASSERT_LIN_TERMS(bilin.right_linear_terms(), {Var(7), Var(5)},
                     {13.0, 13.0});
    ASSERT_EQ(bilin.constant(), 44);
}
GTEST_TEST(bilinear_expression_operators, scalar_add2) {
    auto e = (-Var(11) * 3.2 + 13) * (Var(7) + Var(5) + 2);
    auto bilin = 18.0 + e;
    ASSERT_BILIN_TERMS(bilin.bilinear_terms(), {Var(11), Var(11)},
                       {Var(7), Var(5)}, {-3.2, -3.2});
    ASSERT_LIN_TERMS(bilin.left_linear_terms(), {Var(11)}, {-6.4});
    ASSERT_LIN_TERMS(bilin.right_linear_terms(), {Var(7), Var(5)},
                     {13.0, 13.0});
    ASSERT_EQ(bilin.constant(), 44);
}
GTEST_TEST(bilinear_expression_operators, scalar_sub) {
    auto e = (-Var(11) * 3.2 + 13) * (Var(7) + Var(5) + 2);
    auto bilin = e - (-18.0);
    ASSERT_BILIN_TERMS(bilin.bilinear_terms(), {Var(11), Var(11)},
                       {Var(7), Var(5)}, {-3.2, -3.2});
    ASSERT_LIN_TERMS(bilin.left_linear_terms(), {Var(11)}, {-6.4});
    ASSERT_LIN_TERMS(bilin.right_linear_terms(), {Var(7), Var(5)},
                     {13.0, 13.0});
    ASSERT_EQ(bilin.constant(), 44);
}
GTEST_TEST(bilinear_expression_operators, scalar_sub2) {
    auto e = (-Var(11) * 3.2 + 13) * (Var(7) + Var(5) + 2);
    auto bilin = 18.0 - (-e);
    ASSERT_BILIN_TERMS(bilin.bilinear_terms(), {Var(11), Var(11)},
                       {Var(7), Var(5)}, {-3.2, -3.2});
    ASSERT_LIN_TERMS(bilin.left_linear_terms(), {Var(11)}, {-6.4});
    ASSERT_LIN_TERMS(bilin.right_linear_terms(), {Var(7), Var(5)},
                     {13.0, 13.0});
    ASSERT_EQ(bilin.constant(), 44);
}

GTEST_TEST(bilinear_expression_operators, scalar_mul) {
    auto e = (-Var(11) * 3.2 + 13) * (Var(7) + Var(5) + 2);
    auto bilin = e * 3.0;
    ASSERT_BILIN_TERMS(bilin.bilinear_terms(), {Var(11), Var(11)},
                       {Var(7), Var(5)}, {-9.6, -9.6});
    ASSERT_LIN_TERMS(bilin.left_linear_terms(), {Var(11)}, {-19.2});
    ASSERT_LIN_TERMS(bilin.right_linear_terms(), {Var(7), Var(5)},
                     {39.0, 39.0});
    ASSERT_EQ(bilin.constant(), 78);
}
GTEST_TEST(bilinear_expression_operators, scalar_mul2) {
    auto e = (-Var(11) * 3.2 + 13) * (Var(7) + Var(5) + 2);
    auto bilin = 3.0 * e;
    ASSERT_BILIN_TERMS(bilin.bilinear_terms(), {Var(11), Var(11)},
                       {Var(7), Var(5)}, {-9.6, -9.6});
    ASSERT_LIN_TERMS(bilin.left_linear_terms(), {Var(11)}, {-19.2});
    ASSERT_LIN_TERMS(bilin.right_linear_terms(), {Var(7), Var(5)},
                     {39.0, 39.0});
    ASSERT_EQ(bilin.constant(), 78);
}
GTEST_TEST(bilinear_expression_operators, scalar_div) {
    auto e = (-Var(11) * 3.2 + 13) * (Var(7) + Var(5) + 2);
    auto bilin = e / (1.0/3.0);
    ASSERT_BILIN_TERMS(bilin.bilinear_terms(), {Var(11), Var(11)},
                       {Var(7), Var(5)}, {-9.6, -9.6});
    ASSERT_LIN_TERMS(bilin.left_linear_terms(), {Var(11)}, {-19.2});
    ASSERT_LIN_TERMS(bilin.right_linear_terms(), {Var(7), Var(5)},
                     {39.0, 39.0});
    ASSERT_EQ(bilin.constant(), 78);
}