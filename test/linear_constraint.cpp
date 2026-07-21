#include <vector>

#undef NDEBUG
#include <gtest/gtest.h>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_entities.hpp"

#include "assert_helper.hpp"

using namespace mippp;
using namespace mippp::operators;

using Var = model_variable<int, double>;

GTEST_TEST(linear_constraint_operators, equal_scalar) {
    ASSERT_CONSTRAINT(Var(3) * 6 + 7 - Var(2) == -2,
                      {{Var(3), 6.0}, {Var(2), -1.0}}, constraint_sense::equal,
                      -9);
}

GTEST_TEST(linear_constraint_operators, equal_scalar2) {
    ASSERT_CONSTRAINT(-2 == Var(3) * 6 + 7 - Var(2),
                      {{Var(3), -6.0}, {Var(2), 1.0}}, constraint_sense::equal,
                      9);
}

GTEST_TEST(linear_constraint_operators, equal_expressions) {
    ASSERT_CONSTRAINT(Var(3) * 6 + 7 - Var(2) == -Var(11) * 3.2 - 2,
                      {{Var(3), 6.0}, {Var(2), -1.0}, {Var(11), 3.2}},
                      constraint_sense::equal, -9);
}

GTEST_TEST(linear_constraint_operators, lower_bound) {
    ASSERT_CONSTRAINT(-Var(11) * 3.2 >= 3, {{Var(11), -3.2}},
                      constraint_sense::greater_equal, 3);
}

GTEST_TEST(linear_constraint_operators, lower_bound2) {
    ASSERT_CONSTRAINT(-3 >= Var(11) * 3.2, {{Var(11), -3.2}},
                      constraint_sense::greater_equal, 3);
}

GTEST_TEST(linear_constraint_operators, upper_bound) {
    ASSERT_CONSTRAINT(-Var(11) * 3.2 <= 3, {{Var(11), -3.2}},
                      constraint_sense::less_equal, 3);
}

GTEST_TEST(linear_constraint_operators, upper_bound2) {
    ASSERT_CONSTRAINT(-3 <= Var(11) * 3.2, {{Var(11), -3.2}},
                      constraint_sense::less_equal, 3);
}
///////////////////////////////////////////////////////////////////////////////
/////////////////////// Constraints must be re-readable ///////////////////////
///////////////////////////////////////////////////////////////////////////////

// `linear_constraint` probes its requirements through a `const &`: reading a
// constraint must not consume it. An expression owning a move-only term range
// therefore cannot back one, and `linear_constraint_view` rejects it on its
// own template parameter -- not merely in the operators -- so that
// `std::is_constructible_v` and friends do not lie.

namespace {
using term = std::pair<Var, double>;
// terms = owning_view over an rvalue vector: move-only
using owning_expr = decltype(linear_expression_view(std::vector<term>{}));
// terms = concat of transforms over named operands: copyable
using named_expr = decltype(Var(1) * 3.2 + 9);

template <typename LE>
concept constrainable = requires { typename linear_constraint_view<LE>; };
}  // namespace

GTEST_TEST(linear_constraint_concepts, rejects_move_only_term_ranges) {
    static_assert(!const_readable_linear_terms<owning_expr>);
    static_assert(const_readable_linear_terms<named_expr>);

    static_assert(!constrainable<owning_expr>);
    static_assert(constrainable<named_expr>);
}

GTEST_TEST(linear_constraint_concepts, constraints_model_the_concept) {
    // the whole point of constraining the class rather than the member:
    // asking the question must yield an answer, not a hard error
    static_assert(linear_constraint<decltype(Var(1) * 3.2 <= 9.0)>);
    static_assert(linear_constraint<decltype(9.0 <= Var(1) * 3.2)>);
    static_assert(linear_constraint<decltype(Var(1) == Var(2) * 2.0)>);
}

GTEST_TEST(linear_constraint_view_test, is_readable_twice) {
    // a constraint is consumed by `add_constraint` through a `const &`, so
    // reading its terms must leave it intact
    const auto c = (Var(1) * 3.2 - Var(2) <= 9.0);
    ASSERT_LIN_TERMS(c.linear_terms(), {{Var(1), 3.2}, {Var(2), -1.0}});
    ASSERT_LIN_TERMS(c.linear_terms(), {{Var(1), 3.2}, {Var(2), -1.0}});
    ASSERT_EQ(c.sense(), constraint_sense::less_equal);
    ASSERT_EQ(c.rhs(), 9.0);
}
