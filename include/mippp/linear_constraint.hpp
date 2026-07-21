#pragma once

#include <ranges>
#include <type_traits>

#include "mippp/linear_expression.hpp"

namespace mippp {

enum constraint_sense : int { equal = 0, less_equal = -1, greater_equal = 1 };

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Concepts ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename LC>
using linear_constraint_variable_t = linear_term_variable_t<linear_term_t<LC>>;

template <typename LC>
using linear_constraint_scalar_t = linear_term_scalar_t<linear_term_t<LC>>;

template <typename LC>
using linear_constraint_rhs_t =
    std::decay_t<decltype(std::declval<LC &>().rhs())>;

template <typename T>
concept linear_constraint =
    requires(T && t) {
        t.linear_terms();
        { t.sense() } -> std::same_as<constraint_sense>;
        t.rhs();
    } && linear_term<linear_term_t<T>> &&
    std::convertible_to<linear_constraint_rhs_t<T>,
                        linear_constraint_scalar_t<T>>;

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// Views ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <linear_expression LExpr>
class linear_constraint_view {
private:
    LExpr _expression;
    constraint_sense _constraint_sense;

public:
    template <typename E>
        requires std::constructible_from<LExpr, E &&>
    constexpr linear_constraint_view(E && e, constraint_sense rel)
        : _expression(std::forward<E>(e)), _constraint_sense(rel) {}

    // A constraint does not own terms, it wraps an expression that does, so
    // each overload just forwards to the matching one of the wrapped
    // expression. `decltype(auto)` propagates whatever that returns: a view
    // by value, an lvalue reference or an xvalue.
    [[nodiscard]] constexpr decltype(auto) linear_terms() const & noexcept(
        noexcept(_expression.linear_terms()))
        requires const_readable_linear_terms<LExpr>
    {
        return _expression.linear_terms();
    }
    [[nodiscard]] constexpr decltype(auto) linear_terms() & noexcept(
        noexcept(_expression.linear_terms())) {
        return _expression.linear_terms();
    }
    [[nodiscard]] constexpr decltype(auto) linear_terms() && noexcept(
        noexcept(std::move(_expression).linear_terms())) {
        return std::move(_expression).linear_terms();
    }

    [[nodiscard]] constexpr constraint_sense sense() const noexcept {
        return _constraint_sense;
    }

    // The `linear_expression` concept only checks `constant()` on non-const
    // lvalues, so a const-callable overload is required here explicitly --
    // without the guard, an expression lacking one would fail inside the body
    // instead of making the constraint SFINAE-visibly lack `rhs()`.
    [[nodiscard]] constexpr auto rhs() const
        noexcept(noexcept(-_expression.constant()))
        requires requires(const LExpr & e) { -e.constant(); }
    {
        return -_expression.constant();
    }
};

template <typename LE>
linear_constraint_view(LE &&, constraint_sense)
    -> linear_constraint_view<std::decay_t<LE>>;

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Operators //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

namespace operators {

template <linear_expression E1, linear_expression E2>
[[nodiscard]] constexpr auto operator<=(E1 && e1, E2 && e2) {
    detail::assert_compatible_linear_expressions<E1, E2>();
    return linear_constraint_view(
        linear_expression_add(std::forward<E1>(e1),
                              linear_expression_negate(std::forward<E2>(e2))),
        constraint_sense::less_equal);
}
template <linear_expression E1, linear_expression E2>
[[nodiscard]] constexpr auto operator>=(E1 && e1, E2 && e2) {
    detail::assert_compatible_linear_expressions<E1, E2>();
    return linear_constraint_view(
        linear_expression_add(std::forward<E1>(e1),
                              linear_expression_negate(std::forward<E2>(e2))),
        constraint_sense::greater_equal);
}

template <linear_expression E1, linear_expression E2>
[[nodiscard]] constexpr auto operator==(E1 && e1, E2 && e2) {
    detail::assert_compatible_linear_expressions<E1, E2>();
    return linear_constraint_view(
        linear_expression_add(std::forward<E1>(e1),
                              linear_expression_negate(std::forward<E2>(e2))),
        constraint_sense::equal);
}

template <linear_expression E>
[[nodiscard]] constexpr auto operator<=(linear_expression_scalar_t<E> c, E && e) {
    return linear_constraint_view(
        linear_expression_scalar_add(
            linear_expression_negate(std::forward<E>(e)), c),
        constraint_sense::less_equal);
}
template <linear_expression E>
[[nodiscard]] constexpr auto operator<=(E && e, linear_expression_scalar_t<E> c) {
    return linear_constraint_view(
        linear_expression_scalar_add(std::forward<E>(e), -c),
        constraint_sense::less_equal);
}

template <linear_expression E>
[[nodiscard]] constexpr auto operator>=(linear_expression_scalar_t<E> c, E && e) {
    return linear_constraint_view(
        linear_expression_scalar_add(
            linear_expression_negate(std::forward<E>(e)), c),
        constraint_sense::greater_equal);
}
template <linear_expression E>
[[nodiscard]] constexpr auto operator>=(E && e, linear_expression_scalar_t<E> c) {
    return linear_constraint_view(
        linear_expression_scalar_add(std::forward<E>(e), -c),
        constraint_sense::greater_equal);
}

template <linear_expression E>
[[nodiscard]] constexpr auto operator==(E && e, linear_expression_scalar_t<E> c) {
    return linear_constraint_view(
        linear_expression_scalar_add(std::forward<E>(e), -c),
        constraint_sense::equal);
}
template <linear_expression E>
[[nodiscard]] constexpr auto operator==(linear_expression_scalar_t<E> c, E && e) {
    return linear_constraint_view(
        linear_expression_scalar_add(
            linear_expression_negate(std::forward<E>(e)), c),
        constraint_sense::equal);
}

}  // namespace operators

}  // namespace mippp
