#ifndef MIPPP_LINEAR_CONSTRAINT_HPP
#define MIPPP_LINEAR_CONSTRAINT_HPP

#include <type_traits>

#include "mippp/detail/infinity_helper.hpp"
#include "mippp/linear_expression.hpp"

namespace fhamonic {
namespace mippp {

enum constraint_relation : int {
    equal_zero = 0,
    less_equal_zero = -1,
    greater_equal_zero = 1
};

/////////////////////////////////// CONCEPT ///////////////////////////////////

template <typename _Tp>
using linear_constraint_expression_t =
    decltype(std::declval<_Tp &>().expression());

template <typename _Tp>
using linear_constraint_variable_id_t =
    linear_expression_variable_t<linear_constraint_expression_t<_Tp>>;

template <typename _Tp>
using linear_constraint_scalar_t =
    linear_expression_scalar_t<linear_constraint_expression_t<_Tp>>;

template <typename _Tp>
concept linear_constraint = requires(const _Tp & __t) {
    { __t.expression() } -> linear_expression;
    { __t.relation() } -> std::same_as<constraint_relation>;
};

////////////////////////////// BOUNDS ACCESSORS ///////////////////////////////

template <linear_constraint C>
constexpr auto linear_constraint_lower_bound(C && c) {
    if(c.relation() == constraint_relation::less_equal_zero) {
        return detail::minus_infinity_or_lowest<
            linear_constraint_scalar_t<C>>();
    }
    return -c.expression().constant();
}

template <linear_constraint C>
constexpr auto linear_constraint_upper_bound(C && c) {
    if(c.relation() == constraint_relation::greater_equal_zero) {
        return detail::infinity_or_max<linear_constraint_scalar_t<C>>();
    }
    return -c.expression().constant();
}

//////////////////////////////////// CLASS ////////////////////////////////////

template <typename _Expression>
class linear_constraint_view {
private:
    _Expression _expression;
    constraint_relation _constraint_relation;

public:
    constexpr linear_constraint_view(_Expression && e,
                                     constraint_relation cmp_order)
        : _expression(std::forward<_Expression>(e))
        , _constraint_relation(cmp_order) {}

    constexpr const _Expression & expression() const noexcept {
        return _expression;
    }
    constexpr constraint_relation relation() const noexcept {
        return _constraint_relation;
    }
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
        constraint_relation::less_equal_zero);
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
        constraint_relation::greater_equal_zero);
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
        constraint_relation::equal_zero);
};

template <linear_expression E>
constexpr auto operator<=(linear_expression_scalar_t<E> c, E && e) {
    return linear_constraint_view(
        linear_expression_scalar_add(std::forward<E>(e), -c),
        constraint_relation::greater_equal_zero);
};
template <linear_expression E>
constexpr auto operator>=(E && e, linear_expression_scalar_t<E> c) {
    return linear_constraint_view(
        linear_expression_scalar_add(std::forward<E>(e), -c),
        constraint_relation::greater_equal_zero);
};

template <linear_expression E>
constexpr auto operator<=(E && e, linear_expression_scalar_t<E> c) {
    return linear_constraint_view(
        linear_expression_scalar_add(std::forward<E>(e), -c),
        constraint_relation::less_equal_zero);
};
template <linear_expression E>
constexpr auto operator>=(linear_expression_scalar_t<E> c, E && e) {
    return linear_constraint_view(
        linear_expression_scalar_add(std::forward<E>(e), -c),
        constraint_relation::less_equal_zero);
};

template <linear_expression E>
constexpr auto operator==(E && e, linear_expression_scalar_t<E> c) {
    return linear_constraint_view(
        linear_expression_scalar_add(std::forward<E>(e), -c),
        constraint_relation::equal_zero);
};
template <linear_expression E>
constexpr auto operator==(linear_expression_scalar_t<E> c, E && e) {
    return linear_constraint_view(
        linear_expression_scalar_add(std::forward<E>(e), -c),
        constraint_relation::equal_zero);
};

}  // namespace operators

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_LINEAR_CONSTRAINT_HPP