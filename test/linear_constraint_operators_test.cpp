#include <vector>

#undef NDEBUG
#include <gtest/gtest.h>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_variable.hpp"

#include "assert_constraint.hpp"
#include "assert_eq_ranges.hpp"

using namespace fhamonic::mippp;
using namespace fhamonic::mippp::operators;

using Var = model_variable<int, double>;

GTEST_TEST(linear_constraint_operators, less_equal) {
    ASSERT_CONSTRAINT((-Var(11) * 3.2 - 2 <= Var(3) * 6 + 7 - Var(2)),
                      {11, 3, 2}, {-3.2, -6.0, 1.0},
                      -std::numeric_limits<double>::infinity(), 9);
}

GTEST_TEST(linear_constraint_operators, less_equal_other_way) {
    ASSERT_CONSTRAINT((Var(3) * 6 + 7 - Var(2) >= -Var(11) * 3.2 - 2),
                      {3, 2, 11}, {6.0, -1.0, 3.2}, -9,
                      std::numeric_limits<double>::infinity());
}

GTEST_TEST(linear_constraint_operators, equal_scalar) {
    ASSERT_CONSTRAINT((Var(3) * 6 + 7 - Var(2) == -2), {3, 2}, {6.0, -1.0}, -9,
                      -9);
}

GTEST_TEST(linear_constraint_operators, equal_scalar_other_way) {
    ASSERT_CONSTRAINT((-2 == Var(3) * 6 + 7 - Var(2)), {3, 2}, {6.0, -1.0}, -9,
                      -9);
}

GTEST_TEST(linear_constraint_operators, equal_expressions) {
    ASSERT_CONSTRAINT((Var(3) * 6 + 7 - Var(2) == -Var(11) * 3.2 - 2),
                      {3, 2, 11}, {6.0, -1.0, 3.2}, -9, -9);
}

GTEST_TEST(linear_constraint_operators, lower_bound) {
    ASSERT_CONSTRAINT((3 <= -Var(11) * 3.2), {11}, {-3.2}, 3,
                      std::numeric_limits<double>::infinity());
}

GTEST_TEST(linear_constraint_operators, lower_bound_other_way) {
    ASSERT_CONSTRAINT((-Var(11) * 3.2 >= 3), {11}, {-3.2}, 3,
                      std::numeric_limits<double>::infinity());
}

GTEST_TEST(linear_constraint_operators, upper_bound) {
    ASSERT_CONSTRAINT((-Var(11) * 3.2 <= 3), {11}, {-3.2},
                      -std::numeric_limits<double>::infinity(), 3);
}

GTEST_TEST(linear_constraint_operators, upper_bound_other_way) {
    ASSERT_CONSTRAINT((-Var(11) * 3.2 <= 3), {11}, {-3.2},
                      -std::numeric_limits<double>::infinity(), 3);
}

// GTEST_TEST(linear_constraint_operators, bounds) {
//     ASSERT_CONSTRAINT((1 <= -Var(11) * 3.2 <= 3), {11}, {-3.2}, 1, 3);
// }

// GTEST_TEST(linear_constraint_operators, bounds_other_way) {
//     ASSERT_CONSTRAINT(3 >= -Var(11) * 3.2 >= 1, {11}, {-3.2}, 1, 3);
// }