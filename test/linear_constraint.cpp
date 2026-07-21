#undef NDEBUG
#include <gtest/gtest.h>

#include <concepts>
#include <utility>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_entities.hpp"

#include "assert_helper.hpp"

using namespace mippp;
using namespace mippp::operators;

using Var = model_variable<int, double>;

///////////////////////////////////////////////////////////////////////////////
///////////////////// Comparisons build lazy constraints //////////////////////
///////////////////////////////////////////////////////////////////////////////

GTEST_TEST(linear_constraint_operators, equal) {
    ASSERT_CONSTRAINT(Var(3) * 6 + 7 - Var(2) == -2,
                      {{Var(3), 6.0}, {Var(2), -1.0}}, constraint_sense::equal,
                      -9);
    ASSERT_CONSTRAINT(-2 == Var(3) * 6 + 7 - Var(2),
                      {{Var(3), -6.0}, {Var(2), 1.0}}, constraint_sense::equal,
                      9);
    ASSERT_CONSTRAINT(Var(3) * 6 + 7 - Var(2) == -Var(11) * 3.2 - 2,
                      {{Var(3), 6.0}, {Var(2), -1.0}, {Var(11), 3.2}},
                      constraint_sense::equal, -9);
}

GTEST_TEST(linear_constraint_operators, greater_equal) {
    ASSERT_CONSTRAINT(-Var(11) * 3.2 >= 3, {{Var(11), -3.2}},
                      constraint_sense::greater_equal, 3);
    ASSERT_CONSTRAINT(-3 >= Var(11) * 3.2, {{Var(11), -3.2}},
                      constraint_sense::greater_equal, 3);
}

GTEST_TEST(linear_constraint_operators, less_equal) {
    ASSERT_CONSTRAINT(-Var(11) * 3.2 <= 3, {{Var(11), -3.2}},
                      constraint_sense::less_equal, 3);
    ASSERT_CONSTRAINT(-3 <= Var(11) * 3.2, {{Var(11), -3.2}},
                      constraint_sense::less_equal, 3);
}

///////////////////////////////////////////////////////////////////////////////
/////////////////////// Constraints must be re-readable ///////////////////////
///////////////////////////////////////////////////////////////////////////////

// Reading a constraint must never consume it, but that does not require
// `const`: the `&` overload hands back a reference too. So an expression
// owning a move-only term range *can* back a constraint -- it just cannot be
// read through a `const &`. `linear_constraint` probes the operand as it was
// passed (`T &&`), answering per value category.

namespace {
using term = std::pair<Var, double>;
// terms = owning_view over an rvalue vector: move-only
using owning_expr = decltype(linear_expression_view(std::vector<term>{}));
// terms = concat of transforms over named operands: copyable
using named_expr = decltype(Var(1) * 3.2 + 9);

template <typename LE>
concept constrainable = requires { typename linear_constraint_view<LE>; };
}  // namespace

GTEST_TEST(linear_constraint_concepts, move_only_terms_back_a_constraint) {
    static_assert(!const_readable_linear_terms<owning_expr>);
    static_assert(const_readable_linear_terms<named_expr>);

    // both can back a constraint ...
    static_assert(constrainable<owning_expr>);
    static_assert(constrainable<named_expr>);

    // ... but only the copyable one is readable through a `const &`
    using owning_c = linear_constraint_view<owning_expr>;
    using named_c = linear_constraint_view<named_expr>;
    static_assert(linear_constraint<owning_c &>);
    static_assert(!linear_constraint<const owning_c &>);
    static_assert(linear_constraint<named_c &>);
    static_assert(linear_constraint<const named_c &>);
}

GTEST_TEST(linear_constraint_concepts, constraints_model_the_concept) {
    // asking the question must yield an answer, not a hard error
    static_assert(linear_constraint<decltype(Var(1) * 3.2 <= 9.0)>);
    static_assert(linear_constraint<decltype(9.0 <= Var(1) * 3.2)>);
    static_assert(linear_constraint<decltype(Var(1) == Var(2) * 2.0)>);
}

GTEST_TEST(linear_constraint_view_test, is_readable_twice) {
    // `add_constraint` consumes a constraint through a `const &`: reading the
    // terms must leave it intact
    const auto c = (Var(1) * 3.2 - Var(2) <= 9.0);
    ASSERT_LIN_TERMS(c.linear_terms(), {{Var(1), 3.2}, {Var(2), -1.0}});
    ASSERT_LIN_TERMS(c.linear_terms(), {{Var(1), 3.2}, {Var(2), -1.0}});
    ASSERT_EQ(c.sense(), constraint_sense::less_equal);
    ASSERT_EQ(c.rhs(), 9.0);
}

///////////////////////////////////////////////////////////////////////////////
////////////////////// Accessors forward, they don't copy /////////////////////
///////////////////////////////////////////////////////////////////////////////

// A constraint wraps an expression rather than owning terms, so each accessor
// forwards to the matching overload of the wrapped expression and propagates
// its return category -- decaying it would turn a container-backed read into
// a full copy of the container.

namespace {
using view_backed = decltype(Var(1) * 3.2 - Var(2) <= 9.0);
using rt_expr = decltype(materialize(Var(1) * 3.2 + 9.0));
// a constraint sitting directly on a container-backed expression
using rt_backed = decltype(linear_constraint_view(std::declval<rt_expr &>(),
                                                  constraint_sense::equal));

template <typename C>
using const_terms_t = decltype(std::declval<const C &>().linear_terms());
template <typename C>
using lvalue_terms_t = decltype(std::declval<C &>().linear_terms());
template <typename C>
using rvalue_terms_t = decltype(std::declval<C &&>().linear_terms());
}  // namespace

GTEST_TEST(linear_constraint_view_test, terms_accessors_forward) {
    static_assert(linear_constraint<view_backed>);
    static_assert(linear_constraint<rt_backed>);

    // the wrapped view returns a copy of itself from `const &`, a reference
    // from the other two -- the constraint propagates each unchanged
    static_assert(!std::is_reference_v<const_terms_t<view_backed>>);
    static_assert(std::is_lvalue_reference_v<lvalue_terms_t<view_backed>>);
    static_assert(std::is_rvalue_reference_v<rvalue_terms_t<view_backed>>);

    // container-backed: reading through `const &` must hand back a reference,
    // never a copy of the vector
    static_assert(std::is_lvalue_reference_v<const_terms_t<rt_backed>>);
    static_assert(std::same_as<const_terms_t<rt_backed>,
                               const std::vector<std::pair<Var, double>> &>);
    static_assert(std::is_lvalue_reference_v<lvalue_terms_t<rt_backed>>);
    static_assert(std::is_rvalue_reference_v<rvalue_terms_t<rt_backed>>);
}

GTEST_TEST(linear_constraint_view_test, rhs_never_returns_a_reference) {
    // `-constant()` is a prvalue: returning a reference to it would dangle
    static_assert(!std::is_reference_v<
                  decltype(std::declval<const view_backed &>().rhs())>);
    static_assert(
        !std::is_reference_v<decltype(std::declval<view_backed &>().rhs())>);
    static_assert(
        !std::is_reference_v<decltype(std::declval<view_backed &&>().rhs())>);
    static_assert(
        !std::is_reference_v<decltype(std::declval<rt_backed &&>().rhs())>);

    // and it survives being read off an rvalue constraint
    ASSERT_EQ((Var(1) * 3.2 <= 9.0).rhs(), 9.0);
    auto rhs = (Var(1) * 3.2 - Var(2) >= -2.0).rhs();
    ASSERT_EQ(rhs, -2.0);
}

GTEST_TEST(linear_constraint_view_test, container_backed_is_readable_twice) {
    auto a = materialize(Var(1) * 3.2 + 9.0);
    auto b = materialize(Var(2) * 1.5);
    const auto c = (a <= b);
    ASSERT_LIN_TERMS(c.linear_terms(), {{Var(1), 3.2}, {Var(2), -1.5}});
    ASSERT_LIN_TERMS(c.linear_terms(), {{Var(1), 3.2}, {Var(2), -1.5}});
    ASSERT_EQ(c.sense(), constraint_sense::less_equal);
    ASSERT_EQ(c.rhs(), -9.0);
    // the operands are only referenced, so they stay usable afterwards
    ASSERT_LIN_EXPR(a, {{Var(1), 3.2}}, 9.0);
    ASSERT_LIN_EXPR(b, {{Var(2), 1.5}}, 0);
}

///////////////////////////////////////////////////////////////////////////////
///////////////// Const-readability is a property of the sum //////////////////
///////////////////////////////////////////////////////////////////////////////

// What must be const-readable is the expression backing the constraint, not
// the operands handed to the comparison: a container-backed operand is itself
// const-readable, but forwarding it as an rvalue hands the container over and
// the sum is left behind a move-only `owning_view`.

namespace {
using rt_lvalue_sum = decltype(linear_expression_add(
    std::declval<rt_expr &>(),
    linear_expression_negate(std::declval<rt_expr &>())));
using rt_rvalue_sum = decltype(linear_expression_add(
    std::declval<rt_expr>(),
    linear_expression_negate(std::declval<rt_expr>())));
}  // namespace

GTEST_TEST(linear_constraint_concepts, operand_readability_is_not_the_sums) {
    // the operand is const-readable either way ...
    static_assert(const_readable_linear_terms<rt_expr>);

    // ... but only the lvalue sum keeps that property
    static_assert(const_readable_linear_terms<rt_lvalue_sum>);
    static_assert(!const_readable_linear_terms<rt_rvalue_sum>);

    // both still back a constraint; the difference shows up in how it may be
    // held, which is exactly what `linear_constraint` reports
    static_assert(constrainable<rt_lvalue_sum>);
    static_assert(constrainable<rt_rvalue_sum>);

    using lvalue_c = linear_constraint_view<rt_lvalue_sum>;
    using rvalue_c = linear_constraint_view<rt_rvalue_sum>;
    static_assert(linear_constraint<const lvalue_c &>);
    static_assert(linear_constraint<rvalue_c &>);
    static_assert(!linear_constraint<const rvalue_c &>);
}

GTEST_TEST(linear_constraint_view_test, moved_operands_stay_readable) {
    auto a = materialize(Var(1) * 3.2 + 9.0);
    auto b = materialize(Var(2) * 1.5);
    auto c = (std::move(a) <= std::move(b));
    // non-const reads return a reference, so they do not consume
    ASSERT_LIN_TERMS(c.linear_terms(), {{Var(1), 3.2}, {Var(2), -1.5}});
    ASSERT_LIN_TERMS(c.linear_terms(), {{Var(1), 3.2}, {Var(2), -1.5}});
    ASSERT_EQ(c.sense(), constraint_sense::less_equal);
    ASSERT_EQ(c.rhs(), -9.0);
}
