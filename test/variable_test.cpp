#undef NDEBUG
#include <gtest/gtest.h>

#include "mippp/linear_expression.hpp"
#include "mippp/model_entities.hpp"

#include "assert_helper.hpp"

using namespace fhamonic::mippp;

using Var = model_variable<int, double>;

GTEST_TEST(variable, ctor) {
    auto v = Var(11);
    ASSERT_EXPRESSION(v, {Var(11)}, {1.0}, 0);
}

GTEST_TEST(variable, negate_var) {
    ASSERT_EXPRESSION(linear_expression_negate(Var(11)), {Var(11)}, {-1.0}, 0);
}

GTEST_TEST(variable, var_scalar_add) {
    ASSERT_EXPRESSION(linear_expression_scalar_add(Var(11), 5), {Var(11)},
                      {1.0}, 5);
}

GTEST_TEST(variable, scalar_mul) {
    ASSERT_EXPRESSION(linear_expression_scalar_mul(Var(11), -2.5), {Var(11)},
                      {-2.5}, 0);
}

GTEST_TEST(variable, add_variables) {
    ASSERT_EXPRESSION(linear_expression_add(Var(11), Var(2)), {Var(11), Var(2)},
                      {1.0, 1.0}, 0);
}

GTEST_TEST(variable, add_negative_var) {
    ASSERT_EXPRESSION(
        linear_expression_add(Var(3), linear_expression_negate(Var(12))),
        {Var(3), Var(12)}, {1.0, -1.0}, 0);
}