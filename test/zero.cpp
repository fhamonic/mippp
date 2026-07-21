#undef NDEBUG
#include <gtest/gtest.h>

#include <concepts>
#include <ranges>
#include <type_traits>
#include <vector>

#include "mippp/utility/zero.hpp"
//
#include "mippp/linear_expression.hpp"
#include "mippp/model_entities.hpp"

#include "assert_helper.hpp"

// Suite `zero_t_algebra` depends only on utility/zero.hpp; suite
// `zero_t_propagation` covers its integration with the expression layer.

using namespace mippp;
using namespace mippp::operators;

using Var = model_variable<int, double>;

///////////////////////////////////////////////////////////////////////////////
///////////////////////////// zero_t in isolation /////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// a stateless, trivial vocabulary type
static_assert(std::is_empty_v<zero_t>);
static_assert(std::is_trivially_copyable_v<zero_t>);
static_assert(sizeof(zero_t) == 1);

// statically_zero sees through cv-ref qualification
static_assert(statically_zero<zero_t>);
static_assert(statically_zero<zero_t &>);
static_assert(statically_zero<const zero_t &>);
static_assert(statically_zero<zero_t &&>);
static_assert(!statically_zero<double>);
static_assert(!statically_zero<int>);

// converts implicitly to any scalar, and the conversion is SFINAE-friendly
namespace {
struct not_zeroable {
    not_zeroable() = delete;
    explicit not_zeroable(void *) {}
};
}  // namespace
static_assert(std::convertible_to<zero_t, double>);
static_assert(std::convertible_to<zero_t, float>);
static_assert(std::convertible_to<zero_t, int>);
static_assert(static_cast<double>(zero_t{}) == 0.0);
static_assert(static_cast<int>(zero_t{}) == 0);
static_assert(!std::convertible_to<zero_t, not_zeroable>);

// zero-ness is absorbing under * and /, preserved under unary -, and lost
// exactly when a nonzero operand joins under + or - : all checked on TYPES
static_assert(statically_zero<decltype(zero_t{} + zero_t{})>);
static_assert(statically_zero<decltype(zero_t{} - zero_t{})>);
static_assert(statically_zero<decltype(-zero_t{})>);
static_assert(statically_zero<decltype(zero_t{} * zero_t{})>);
static_assert(statically_zero<decltype(zero_t{} * 3.2)>);
static_assert(statically_zero<decltype(3.2 * zero_t{})>);
static_assert(statically_zero<decltype(zero_t{} / 3.2)>);
static_assert(std::same_as<decltype(zero_t{} + 3.2), double>);
static_assert(std::same_as<decltype(3.2 + zero_t{}), double>);
static_assert(std::same_as<decltype(zero_t{} - 3.2), double>);
static_assert(std::same_as<decltype(3.2 - zero_t{}), double>);

// the exact-match friends outrank the builtin operators that would need an
// implicit conversion: the scalar type survives
static_assert(std::same_as<decltype(2 + zero_t{}), int>);
static_assert(std::same_as<decltype(2.5f * zero_t{}), zero_t>);

// the whole algebra is constexpr and noexcept
static_assert(noexcept(zero_t{} + zero_t{}));
static_assert(noexcept(3.2 * zero_t{}));
static_assert(noexcept(static_cast<double>(zero_t{})));

// compound assignment onto a runtime scalar needs its own exact-match friends
// (the conversion template alone leaves `s += zero` ambiguous), and their
// scalar constraint keeps class types with their own `+=` untouched
namespace {
template <typename S>
concept plus_assignable_by_zero = requires(S s) { s += zero_t{}; };
template <typename S>
concept minus_assignable_by_zero = requires(S s) { s -= zero_t{}; };
template <typename S>
concept times_assignable_by_zero = requires(S s) { s *= zero_t{}; };
}  // namespace
static_assert(plus_assignable_by_zero<double>);
static_assert(plus_assignable_by_zero<float>);
static_assert(plus_assignable_by_zero<int>);
static_assert(minus_assignable_by_zero<double>);
static_assert(minus_assignable_by_zero<int>);
static_assert(!plus_assignable_by_zero<not_zeroable>);
// zero_t itself keeps using its member overloads
static_assert(plus_assignable_by_zero<zero_t>);
static_assert(times_assignable_by_zero<zero_t>);
// `s *= zero` and `s / zero` are deliberately absent for scalars: collapsing a
// scalar to zero should be spelled out

GTEST_TEST(zero_t_algebra, arithmetic_values) {
    ASSERT_EQ(zero_t{} + 3.2, 3.2);
    ASSERT_EQ(3.2 + zero_t{}, 3.2);
    ASSERT_EQ(zero_t{} - 3.2, -3.2);
    ASSERT_EQ(3.2 - zero_t{}, 3.2);
    ASSERT_EQ(static_cast<double>(zero_t{} * 3.2), 0.0);
    ASSERT_EQ(static_cast<double>(3.2 * zero_t{}), 0.0);
    ASSERT_EQ(static_cast<double>(zero_t{} / 3.2), 0.0);
    ASSERT_EQ(static_cast<double>(-zero_t{}), 0.0);
    // a runtime scalar consumer sees plain zero
    double offset = zero_t{};
    ASSERT_EQ(offset, 0.0);
}

GTEST_TEST(zero_t_algebra, compound_assignment_onto_scalar) {
    double d = 1.5;
    d += zero_t{};
    d -= zero_t{};
    ASSERT_EQ(d, 1.5);
    int i = 3;
    i += zero_t{};
    ASSERT_EQ(i, 3);
}

///////////////////////////////////////////////////////////////////////////////
////////////////////// propagation through expressions ////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename E>
using constant_t = linear_expression_constant_t<E>;

// variables seed the propagation: their constant is statically zero
static_assert(linear_expression<Var>);
static_assert(statically_zero<constant_t<Var>>);
static_assert(std::same_as<linear_expression_scalar_t<Var>, double>);

// zero-ness propagates through *, /, unary -, and expression +/-
static_assert(statically_zero<constant_t<decltype(3.2 * Var(11))>>);
static_assert(statically_zero<constant_t<decltype(Var(11) / 2.0)>>);
static_assert(statically_zero<constant_t<decltype(-Var(11))>>);
static_assert(
    statically_zero<constant_t<decltype(Var(1) * 3.2 + Var(2) * 1.5)>>);
static_assert(
    statically_zero<constant_t<decltype(Var(1) * 3.2 - Var(2) * 1.5)>>);

// ...and is correctly LOST the moment a scalar constant enters
static_assert(std::same_as<constant_t<decltype(Var(11) * 3.2 + 5)>, double>);
static_assert(std::same_as<constant_t<decltype(5 + Var(11) * 3.2)>, double>);
static_assert(std::same_as<constant_t<decltype(Var(11) - 5.0)>, double>);

// statically-zero expressions still model linear_expression, with the scalar
// type of their terms (not of their constant)
static_assert(linear_expression<decltype(3.2 * Var(11) + Var(2))>);
static_assert(
    std::same_as<linear_expression_scalar_t<decltype(3.2 * Var(11) + Var(2))>,
                 double>);

// the * and / folds are scalar-only: a zero coefficient on an expression goes
// through the expression operators and keeps its terms, never zero_t
namespace {
template <typename E>
concept zero_dividable = requires(E e) { zero / e; };
}  // namespace
static_assert(linear_expression<decltype(zero * Var(1))>);
static_assert(linear_expression<decltype(Var(1) * zero)>);
static_assert(zero_dividable<double>);
static_assert(!zero_dividable<Var>);

GTEST_TEST(zero_t_propagation, existing_helpers_still_work) {
    // ASSERT_LIN_EXPR compares constant() against a double through the
    // implicit conversion
    ASSERT_LIN_EXPR(Var(1) * 3.2 + Var(2) * 1.5, {{Var(1), 3.2}, {Var(2), 1.5}},
                    0);
    ASSERT_LIN_EXPR(Var(11) * 3.2 + 5, {{Var(11), 3.2}}, 5);
}

GTEST_TEST(zero_t_propagation, xsum_static_branch_single_traversal) {
    // no constant to fold: xsum stays fully lazy and traverses exactly once
    std::vector<Var> vars = {Var(1), Var(2), Var(3)};
    int calls = 0;
    auto e = xsum(vars, [&calls](Var v) {
        ++calls;
        return 2.0 * v;
    });
    static_assert(statically_zero<constant_t<decltype(e)>>);
    ASSERT_EQ(calls, 0);
    double coef_sum = 0.0;
    for(auto && [v, c] : e.linear_terms()) coef_sum += c;
    ASSERT_EQ(calls, 3);
    ASSERT_EQ(coef_sum, 6.0);
    ASSERT_EQ(static_cast<double>(e.constant()), 0.0);
}

GTEST_TEST(zero_t_propagation, xsum_fold_branch_constant) {
    std::vector<Var> vars = {Var(1), Var(2), Var(3)};
    auto e = xsum(vars, [](Var v) { return 2.0 * v + 1.5; });
    static_assert(std::same_as<constant_t<decltype(e)>, double>);
    ASSERT_EQ(e.constant(), 4.5);
    ASSERT_LIN_TERMS(e.linear_terms(),
                     {{Var(1), 2.0}, {Var(2), 2.0}, {Var(3), 2.0}});
}

GTEST_TEST(zero_t_propagation, xsum_fold_branch_rvalue_range_regression) {
    // the constant fold must be sequenced before the rvalue range is forwarded
    // into the owning_view, or the sum reads moved-from data
    auto e = xsum(std::vector<Var>{Var(1), Var(2), Var(3)},
                  [](Var v) { return 2.0 * v + 1.5; });
    ASSERT_EQ(e.constant(), 4.5);
    ASSERT_EQ(std::ranges::distance(e.linear_terms()), 3);
}

GTEST_TEST(zero_t_propagation, runtime_expression_accumulates_zero_constant) {
    // `_constant += e.constant()` must accept zero_t on a double constant --
    // incremental construction from constant-free expressions is the common
    // case
    std::vector<Var> vars = {Var(1), Var(2)};
    runtime_linear_expression<Var, double> row;
    for(auto && v : vars) row += 2.0 * v;
    row += 4.5;           // scalar overload
    row += zero;          // scalar overload, statically zero
    row += 3.0 * Var(3);  // expression overload, statically-zero constant
    ASSERT_LIN_EXPR(row, {{Var(1), 2.0}, {Var(2), 2.0}, {Var(3), 3.0}}, 4.5);
}

GTEST_TEST(zero_t_propagation, empty_expression_is_statically_zero) {
    constexpr auto & e = empty_linear_expression<Var, double>;
    static_assert(statically_zero<constant_t<std::decay_t<decltype(e)>>>);
    ASSERT_EQ(std::ranges::distance(e.linear_terms()), 0);
    ASSERT_EQ(static_cast<double>(e.constant()), 0.0);
}
