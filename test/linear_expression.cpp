#undef NDEBUG
#include <gtest/gtest.h>

#include <concepts>
#include <utility>
#include <vector>

#include "mippp/linear_expression.hpp"
#include "mippp/model_entities.hpp"

#include "assert_helper.hpp"

using namespace mippp;
using namespace mippp::operators;

using Var = model_variable<int, double>;

///////////////////////////////////////////////////////////////////////////////
///////////////////// Operators build lazy expressions ////////////////////////
///////////////////////////////////////////////////////////////////////////////

// a variable is the smallest linear expression
static_assert(linear_expression<Var>);

GTEST_TEST(linear_expression_operators, negate) {
    ASSERT_LIN_EXPR((-Var(11)) * 3.2 + 13, {{Var(11), -3.2}}, 13);
    ASSERT_LIN_EXPR(-(Var(11) * 3.2 + 13), {{Var(11), -3.2}}, -13);
}

GTEST_TEST(linear_expression_operators, scalar_add) {
    ASSERT_LIN_EXPR(Var(11) * 3.2 + 5, {{Var(11), 3.2}}, 5);
    ASSERT_LIN_EXPR(5 + Var(11) * 3.2, {{Var(11), 3.2}}, 5);
}

GTEST_TEST(linear_expression_operators, scalar_mul_div) {
    ASSERT_LIN_EXPR((Var(11) * 3.2) * -2, {{Var(11), -6.4}}, 0);
    ASSERT_LIN_EXPR(-2 * (Var(11) * 3.2), {{Var(11), -6.4}}, 0);
    ASSERT_LIN_EXPR((Var(11) * 3.2) / (-0.5), {{Var(11), -6.4}}, 0);
}

GTEST_TEST(linear_expression_operators, add_subtract_expressions) {
    ASSERT_LIN_EXPR(Var(1) * 3.2 + Var(2) * 1.5, {{Var(1), 3.2}, {Var(2), 1.5}},
                    0);
    ASSERT_LIN_EXPR(Var(1) * 3.2 - Var(2) * 1.5,
                    {{Var(1), 3.2}, {Var(2), -1.5}}, 0);
}

GTEST_TEST(linear_expression_operators, named_operands) {
    Var x = Var(27);
    Var y = Var(11);
    auto s = x + y;
    ASSERT_LIN_EXPR(3 * s, {{Var(27), 3.0}, {Var(11), 3.0}}, 0);
}

GTEST_TEST(linear_expression_operators, xsum) {
    std::vector<Var> vars = {Var(3)};
    auto e1 = xsum(vars);
    static_assert(linear_expression<decltype(e1)>);
    ASSERT_LIN_EXPR(e1 + 2 * Var(13), {{Var(3), 1.0}, {Var(13), 2.0}}, 0);
    auto e2 = xsum(vars);
    ASSERT_LIN_EXPR(2 * Var(13) + e2, {{Var(3), 1.0}, {Var(13), 2.0}}, 0);
}

GTEST_TEST(empty_linear_expression, test) {
    auto e = empty_linear_expression<int, double>;
    static_assert(linear_expression<decltype(e)>);
    ASSERT_LIN_EXPR(e, {}, 0);
}

///////////////////////////////////////////////////////////////////////////////
//////////////// Materialized (container-backed) expressions //////////////////
///////////////////////////////////////////////////////////////////////////////

GTEST_TEST(runtime_linear_expression, accumulate) {
    runtime_linear_expression<Var, double> e;
    e += Var(13) - 2;
    e += 4.5;
    static_assert(linear_expression<decltype(e)>);
    ASSERT_LIN_EXPR(e, {{Var(13), 1.0}}, 2.5);
}

GTEST_TEST(runtime_linear_expression, add_two_materialized) {
    // regression: unordered_concat must accept two plain containers
    // (`const std::vector &`), not only views
    auto a = materialize(Var(1) * 3.2 + Var(2) * 1.5);
    auto b = materialize(Var(2) * 2.0 - 4.0);
    auto e = a + b;
    static_assert(linear_expression<decltype(e)>);
    ASSERT_LIN_EXPR(e, {{Var(1), 3.2}, {Var(2), 3.5}}, -4.0);
    // the operands are only referenced, so they stay usable afterwards
    ASSERT_LIN_EXPR(a, {{Var(1), 3.2}, {Var(2), 1.5}}, 0);
    ASSERT_LIN_EXPR(b, {{Var(2), 2.0}}, -4.0);
}

GTEST_TEST(runtime_linear_expression, add_two_materialized_rvalues) {
    // linear_terms() && hands the containers over: the sum must own them
    auto a = materialize(Var(1) * 3.2 + Var(2) * 1.5);
    auto b = materialize(Var(2) * 2.0 - 4.0);
    auto e = std::move(a) + std::move(b);
    ASSERT_LIN_EXPR(e, {{Var(1), 3.2}, {Var(2), 3.5}}, -4.0);
}

GTEST_TEST(runtime_linear_expression, add_materialized_and_view) {
    auto a = materialize(Var(1) * 3.2 + 1.0);
    ASSERT_LIN_EXPR(a + Var(2) * 1.5, {{Var(1), 3.2}, {Var(2), 1.5}}, 1.0);
    ASSERT_LIN_EXPR(Var(2) * 1.5 + a, {{Var(1), 3.2}, {Var(2), 1.5}}, 1.0);
    ASSERT_LIN_EXPR(a - Var(2) * 1.5, {{Var(1), 3.2}, {Var(2), -1.5}}, 1.0);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////// Term-range concepts /////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// Two independent properties of an expression's term range, plus one property
// of how an operand was passed:
//
//   const_readable_linear_terms      terms reachable through `const &`, so
//                                    reading them cannot consume the expression
//   multipass_linear_terms           readable, and the range is restartable
//   forwardable_linear_expression    the operand may be handed to a consuming
//                                    operation; the only value-category
//                                    dependent one
//
// "owns its terms" and "is single-pass" are independent axes; each
// representative below isolates one.

namespace {
using term = std::pair<Var, double>;

// terms = concat_view of transforms: copyable and forward -- the control
using eager_expr = decltype(Var(1) * 3.2 - Var(2) * 1.5 + 9);
// terms = owning_view: random-access, but MOVE-ONLY
using owning_expr = decltype(linear_expression_view(std::vector<term>{}));
// terms = join_view over temporaries: copyable, but SINGLE-PASS
using lazy_expr = decltype(xsum(std::declval<std::vector<Var> &>()));
// terms = const std::vector &: copyable and random-access
using owned_expr = decltype(materialize(std::declval<lazy_expr>()));
}  // namespace

GTEST_TEST(linear_expression_concepts, representatives) {
    // all four model linear_expression: the negative assertions below are
    // about the concepts that separate them, never vacuous
    static_assert(linear_expression<eager_expr>);
    static_assert(linear_expression<owning_expr>);
    static_assert(linear_expression<lazy_expr>);
    static_assert(linear_expression<owned_expr>);
}

GTEST_TEST(linear_expression_concepts, const_readable_linear_terms) {
    static_assert(const_readable_linear_terms<Var>);
    static_assert(const_readable_linear_terms<eager_expr>);
    static_assert(const_readable_linear_terms<owned_expr>);
    // single-pass is irrelevant here: the terms view is still copyable
    static_assert(const_readable_linear_terms<lazy_expr>);
    // move-only terms can only be reached by gutting the expression
    static_assert(!const_readable_linear_terms<owning_expr>);

    // a property of the type, not of the operand: cv/ref make no difference
    static_assert(const_readable_linear_terms<eager_expr &>);
    static_assert(const_readable_linear_terms<const eager_expr &>);
    static_assert(const_readable_linear_terms<eager_expr &&>);
    static_assert(!const_readable_linear_terms<owning_expr &>);
    static_assert(!const_readable_linear_terms<owning_expr &&>);
}

GTEST_TEST(linear_expression_concepts, multipass_linear_terms) {
    static_assert(multipass_linear_terms<Var>);
    static_assert(multipass_linear_terms<eager_expr>);
    static_assert(multipass_linear_terms<owned_expr>);

    // fails on each axis independently
    static_assert(!multipass_linear_terms<lazy_expr>);    // readable, 1-pass
    static_assert(!multipass_linear_terms<owning_expr>);  // restartable, unread.

    // materialize() repairs both
    static_assert(multipass_linear_terms<decltype(materialize(
                      std::declval<lazy_expr>()))>);
    static_assert(multipass_linear_terms<decltype(materialize(
                      std::declval<owning_expr>()))>);

    static_assert(multipass_linear_terms<const eager_expr &>);
    static_assert(!multipass_linear_terms<lazy_expr &>);
}

GTEST_TEST(linear_expression_concepts, forwardable_linear_expression) {
    // an rvalue may always be gutted, whatever its term range looks like
    static_assert(detail::forwardable_linear_expression<owning_expr>);
    static_assert(detail::forwardable_linear_expression<owning_expr &&>);
    // ...but the same expression as an lvalue is single-use: it must be moved
    static_assert(!detail::forwardable_linear_expression<owning_expr &>);
    static_assert(!detail::forwardable_linear_expression<const owning_expr &>);

    // expressions that only reference their terms survive repeated use
    static_assert(detail::forwardable_linear_expression<eager_expr &>);
    static_assert(detail::forwardable_linear_expression<lazy_expr &>);
    static_assert(detail::forwardable_linear_expression<owned_expr &>);
}

namespace {
// forwardable's lvalue branch is *defined as* const_readable_linear_terms;
// keep asserting the equivalence so a rewrite cannot make them drift apart
template <typename E>
constexpr bool lvalue_branch_is_readable =
    detail::forwardable_linear_expression<E &> ==
        const_readable_linear_terms<E> &&
    detail::forwardable_linear_expression<const E &> ==
        const_readable_linear_terms<E>;

template <typename E>
constexpr bool rvalue_is_always_forwardable =
    detail::forwardable_linear_expression<E> &&
    detail::forwardable_linear_expression<E &&>;

template <typename E>
constexpr bool multipass_implies_readable =
    !multipass_linear_terms<E> || const_readable_linear_terms<E>;
}  // namespace

GTEST_TEST(linear_expression_concepts, concept_ordering) {
    static_assert(lvalue_branch_is_readable<eager_expr>);
    static_assert(lvalue_branch_is_readable<owning_expr>);
    static_assert(lvalue_branch_is_readable<lazy_expr>);
    static_assert(lvalue_branch_is_readable<owned_expr>);

    static_assert(rvalue_is_always_forwardable<eager_expr>);
    static_assert(rvalue_is_always_forwardable<owning_expr>);
    static_assert(rvalue_is_always_forwardable<lazy_expr>);
    static_assert(rvalue_is_always_forwardable<owned_expr>);

    static_assert(multipass_implies_readable<eager_expr>);
    static_assert(multipass_implies_readable<owning_expr>);
    static_assert(multipass_implies_readable<lazy_expr>);
    static_assert(multipass_implies_readable<owned_expr>);

    // multipass is *strictly* stronger than readable, hence also than
    // forwardable-for-lvalues -- lazy_expr witnesses both
    static_assert(const_readable_linear_terms<lazy_expr> &&
                  !multipass_linear_terms<lazy_expr>);
    static_assert(detail::forwardable_linear_expression<lazy_expr &> &&
                  !multipass_linear_terms<lazy_expr>);

    // but *incomparable* with forwardable-for-rvalues: owning_expr is
    // forwardable as an rvalue yet never multipass
    static_assert(detail::forwardable_linear_expression<owning_expr &&> &&
                  !multipass_linear_terms<owning_expr>);
}

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Evaluate ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

GTEST_TEST(evaluate_linear_expression, test) {
    auto e = Var(1) * 3.2 - Var(2) * 1.5 + 9;
    entity_mapping<Var, std::vector<double>> values_1 =
        std::vector<double>{0, 5.2, 3.1};
    entity_mapping<Var, std::vector<double>> values_2 =
        std::vector<double>{0, 7, 1.5};
    ASSERT_DOUBLE_EQ(evaluate(e, values_1), 20.99);
    ASSERT_DOUBLE_EQ(evaluate(e, values_2), 29.15);
}
