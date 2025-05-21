#include <vector>

#undef NDEBUG
#include <gtest/gtest.h>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_entities.hpp"

#include "assert_helper.hpp"

using namespace fhamonic::mippp;
using namespace fhamonic::mippp::operators;

using Var = model_variable<int, double>;

GTEST_TEST(linear_constraint_operators, equal_scalar) {
    ASSERT_CONSTRAINT(Var(3) * 6 + 7 - Var(2) == -2, {Var(3), Var(2)},
                      {6.0, -1.0}, constraint_sense::equal, -9);
}

GTEST_TEST(linear_constraint_operators, equal_scalar2) {
    ASSERT_CONSTRAINT(-2 == Var(3) * 6 + 7 - Var(2), {Var(3), Var(2)},
                      {-6.0, 1.0}, constraint_sense::equal, 9);
}

GTEST_TEST(linear_constraint_operators, equal_expressions) {
    ASSERT_CONSTRAINT(Var(3) * 6 + 7 - Var(2) == -Var(11) * 3.2 - 2,
                      {Var(3), Var(2), Var(11)}, {6.0, -1.0, 3.2},
                      constraint_sense::equal, -9);
}

GTEST_TEST(linear_constraint_operators, lower_bound) {
    ASSERT_CONSTRAINT(-Var(11) * 3.2 >= 3, {Var(11)}, {-3.2},
                      constraint_sense::greater_equal, 3);
}

GTEST_TEST(linear_constraint_operators, lower_bound2) {
    ASSERT_CONSTRAINT(-3 >= Var(11) * 3.2, {Var(11)}, {-3.2},
                      constraint_sense::greater_equal, 3);
}

GTEST_TEST(linear_constraint_operators, upper_bound) {
    ASSERT_CONSTRAINT(-Var(11) * 3.2 <= 3, {Var(11)}, {-3.2},
                      constraint_sense::less_equal, 3);
}

GTEST_TEST(linear_constraint_operators, upper_bound2) {
    ASSERT_CONSTRAINT(-3 <= Var(11) * 3.2, {Var(11)}, {-3.2},
                      constraint_sense::less_equal, 3);
}