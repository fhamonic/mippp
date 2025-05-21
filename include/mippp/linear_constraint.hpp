#ifndef MIPPP_LINEAR_CONSTRAINT_HPP
#define MIPPP_LINEAR_CONSTRAINT_HPP

#include <type_traits>

#include "mippp/linear_expression.hpp"

namespace fhamonic::mippp {

enum constraint_sense : int { equal = 0, less_equal = -1, greater_equal = 1 };

/////////////////////////////////// CONCEPT ///////////////////////////////////

template <typename _Tp>
using linear_constraint_variable_t = linear_term_variable_t<linear_term_t<_Tp>>;

template <typename _Tp>
using linear_constraint_scalar_t = linear_term_scalar_t<linear_term_t<_Tp>>;

template <typename _Tp>
concept linear_constraint = requires(const _Tp & __t) {
    { __t.linear_terms() } -> ranges::range;
    { __t.sense() } -> std::same_as<constraint_sense>;
    { __t.rhs() } -> std::same_as<linear_constraint_scalar_t<_Tp>>;
};

//////////////////////////////////// CLASS ////////////////////////////////////

template <typename _Expression>
class linear_constraint_view {
private:
    _Expression _expression;
    constraint_sense _constraint_sense;

public:
    constexpr linear_constraint_view(_Expression && e, constraint_sense rel)
        : _expression(std::forward<_Expression>(e)), _constraint_sense(rel) {}

    constexpr auto linear_terms() const noexcept {
        return _expression.linear_terms();
    }
    constexpr constraint_sense sense() const noexcept {
        return _constraint_sense;
    }
    constexpr auto rhs() const noexcept { return -_expression.constant(); }
};

////////////////////////////////// OPERATORS //////////////////////////////////

namespace operators {

template <linear_expression E1, linear_expression E2>
    requires std::same_as<linear_expression_variable_t<E1>,
                          linear_expression_variable_t<E2>> &&
             std::same_as<linear_expression_scalar_t<E1>,
                          linear_expression_scalar_t<E2>>
constexpr auto operator<=(E1 && e1, E2 && e2) {
    return linear_constraint_view(
        linear_expression_add(std::forward<E1>(e1),
                              linear_expression_negate(std::forward<E2>(e2))),
        constraint_sense::less_equal);
};
template <linear_expression E1, linear_expression E2>
    requires std::same_as<linear_expression_variable_t<E1>,
                          linear_expression_variable_t<E2>> &&
             std::same_as<linear_expression_scalar_t<E1>,
                          linear_expression_scalar_t<E2>>
constexpr auto operator>=(E1 && e1, E2 && e2) {
    return linear_constraint_view(
        linear_expression_add(std::forward<E1>(e1),
                              linear_expression_negate(std::forward<E2>(e2))),
        constraint_sense::greater_equal);
};

template <linear_expression E1, linear_expression E2>
    requires std::same_as<linear_expression_variable_t<E1>,
                          linear_expression_variable_t<E2>> &&
             std::same_as<linear_expression_scalar_t<E1>,
                          linear_expression_scalar_t<E2>>
constexpr auto operator==(E1 && e1, E2 && e2) {
    return linear_constraint_view(
        linear_expression_add(std::forward<E1>(e1),
                              linear_expression_negate(std::forward<E2>(e2))),
        constraint_sense::equal);
};

template <linear_expression E>
constexpr auto operator<=(linear_expression_scalar_t<E> c, E && e) {
    return linear_constraint_view(
        linear_expression_scalar_add(
            linear_expression_negate(std::forward<E>(e)), c),
        constraint_sense::less_equal);
};
template <linear_expression E>
constexpr auto operator<=(E && e, linear_expression_scalar_t<E> c) {
    return linear_constraint_view(
        linear_expression_scalar_add(std::forward<E>(e), -c),
        constraint_sense::less_equal);
};

template <linear_expression E>
constexpr auto operator>=(linear_expression_scalar_t<E> c, E && e) {
    return linear_constraint_view(
        linear_expression_scalar_add(
            linear_expression_negate(std::forward<E>(e)), c),
        constraint_sense::greater_equal);
};
template <linear_expression E>
constexpr auto operator>=(E && e, linear_expression_scalar_t<E> c) {
    return linear_constraint_view(
        linear_expression_scalar_add(std::forward<E>(e), -c),
        constraint_sense::greater_equal);
};

template <linear_expression E>
constexpr auto operator==(E && e, linear_expression_scalar_t<E> c) {
    return linear_constraint_view(
        linear_expression_scalar_add(std::forward<E>(e), -c),
        constraint_sense::equal);
};
template <linear_expression E>
constexpr auto operator==(linear_expression_scalar_t<E> c, E && e) {
    return linear_constraint_view(
        linear_expression_scalar_add(
            linear_expression_negate(std::forward<E>(e)), c),
        constraint_sense::equal);
};

}  // namespace operators

}  // namespace fhamonic::mippp

#endif  // MIPPP_LINEAR_CONSTRAINT_HPP