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
    requires(const T & t) {
        t.linear_terms();
        { t.sense() } -> std::same_as<constraint_sense>;
        t.rhs();
    } && linear_term<linear_term_t<T>> &&
    std::convertible_to<linear_constraint_rhs_t<T>,
                        linear_constraint_scalar_t<T>>;

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// Views ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// `const_readable_linear_terms` is enforced on the view type itself, not just
// at the operators: a constraint is read through a `const &` (see the concept
// above), so an expression owning a move-only term range cannot back one. Were
// the type constructible anyway, probing `linear_constraint` on it would
// instantiate `linear_terms()` and hard-error outside the immediate context --
// the concept would be ill-formed instead of simply `false`.
template <linear_expression LExpr>
    requires const_readable_linear_terms<LExpr>
class linear_constraint_view {
private:
    LExpr _expression;
    constraint_sense _constraint_sense;

public:
    template <typename E>
        requires std::constructible_from<LExpr, E &&>
    constexpr linear_constraint_view(E && e, constraint_sense rel)
        : _expression(std::forward<E>(e)), _constraint_sense(rel) {}

    constexpr auto linear_terms() const noexcept {
        return _expression.linear_terms();
    }
    constexpr constraint_sense sense() const noexcept {
        return _constraint_sense;
    }
    constexpr auto rhs() const noexcept { return -_expression.constant(); }
};

template <typename LE>
linear_constraint_view(LE &&, constraint_sense)
    -> linear_constraint_view<std::decay_t<LE>>;

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Operators //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

namespace operators {

template <linear_expression E1, linear_expression E2>
    requires compatible_linear_expressions<E1, E2>
constexpr auto operator<=(E1 && e1, E2 && e2) {
    detail::assert_constrainable_linear_expressions<E1, E2>();
    return linear_constraint_view(
        linear_expression_add(std::forward<E1>(e1),
                              linear_expression_negate(std::forward<E2>(e2))),
        constraint_sense::less_equal);
};
template <linear_expression E1, linear_expression E2>
    requires compatible_linear_expressions<E1, E2>
constexpr auto operator>=(E1 && e1, E2 && e2) {
    detail::assert_constrainable_linear_expressions<E1, E2>();
    return linear_constraint_view(
        linear_expression_add(std::forward<E1>(e1),
                              linear_expression_negate(std::forward<E2>(e2))),
        constraint_sense::greater_equal);
};

template <linear_expression E1, linear_expression E2>
    requires compatible_linear_expressions<E1, E2>
constexpr auto operator==(E1 && e1, E2 && e2) {
    detail::assert_constrainable_linear_expressions<E1, E2>();
    return linear_constraint_view(
        linear_expression_add(std::forward<E1>(e1),
                              linear_expression_negate(std::forward<E2>(e2))),
        constraint_sense::equal);
};

template <linear_expression E>
constexpr auto operator<=(linear_expression_scalar_t<E> c, E && e) {
    detail::assert_constrainable_linear_expressions<E>();
    return linear_constraint_view(
        linear_expression_scalar_add(
            linear_expression_negate(std::forward<E>(e)), c),
        constraint_sense::less_equal);
};
template <linear_expression E>
constexpr auto operator<=(E && e, linear_expression_scalar_t<E> c) {
    detail::assert_constrainable_linear_expressions<E>();
    return linear_constraint_view(
        linear_expression_scalar_add(std::forward<E>(e), -c),
        constraint_sense::less_equal);
};

template <linear_expression E>
constexpr auto operator>=(linear_expression_scalar_t<E> c, E && e) {
    detail::assert_constrainable_linear_expressions<E>();
    return linear_constraint_view(
        linear_expression_scalar_add(
            linear_expression_negate(std::forward<E>(e)), c),
        constraint_sense::greater_equal);
};
template <linear_expression E>
constexpr auto operator>=(E && e, linear_expression_scalar_t<E> c) {
    detail::assert_constrainable_linear_expressions<E>();
    return linear_constraint_view(
        linear_expression_scalar_add(std::forward<E>(e), -c),
        constraint_sense::greater_equal);
};

template <linear_expression E>
constexpr auto operator==(E && e, linear_expression_scalar_t<E> c) {
    detail::assert_constrainable_linear_expressions<E>();
    return linear_constraint_view(
        linear_expression_scalar_add(std::forward<E>(e), -c),
        constraint_sense::equal);
};
template <linear_expression E>
constexpr auto operator==(linear_expression_scalar_t<E> c, E && e) {
    detail::assert_constrainable_linear_expressions<E>();
    return linear_constraint_view(
        linear_expression_scalar_add(
            linear_expression_negate(std::forward<E>(e)), c),
        constraint_sense::equal);
};

}  // namespace operators

}  // namespace mippp
