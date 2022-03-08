#ifndef MIPPP_LINEAR_EXPRESSION_OPERATORS_HPP
#define MIPPP_LINEAR_EXPRESSION_OPERATORS_HPP

#include "mippp/expressions/linear_expression_operations.hpp"

namespace fhamonic {
namespace mippp {

template <linear_expression_c E1, linear_expression_c E2>
requires std::same_as<typename E1::var_id_t, typename E2::var_id_t> &&
    std::same_as<typename E1::scalar_t, typename E2::scalar_t>
constexpr auto operator+(E1 && e1, E2 && e2) {
    return linear_expression_add(std::forward<E1>(e1), std::forward<E2>(e2));
};
template <linear_expression_c E1, linear_expression_c E2>
requires std::same_as<typename E1::var_id_t, typename E2::var_id_t> &&
    std::same_as<typename E1::scalar_t, typename E2::scalar_t>
constexpr auto operator-(E1 && e1, E2 && e2) {
    return linear_expression_add(
        std::forward<E1>(e1), linear_expression_negate(std::forward<E2>(e2)));
};

template <linear_expression_c E>
constexpr auto operator-(E && e) {
    return linear_expression_negate(std::forward<E>(e));
};

template <linear_expression_c E>
constexpr auto operator+(E && e, typename E::scalar_t c) {
    return linear_expression_scalar_add(std::forward<E>(e), c);
};
template <linear_expression_c E>
constexpr auto operator+(typename E::scalar_t c, E && e) {
    return e + c;
};
template <linear_expression_c E>
constexpr auto operator-(E && e, typename E::scalar_t c) {
    return e + (-c);
};
template <linear_expression_c E>
constexpr auto operator-(typename E::scalar_t c, E && e) {
    return e - c;
};

template <linear_expression_c E>
constexpr auto operator*(E && e, typename E::scalar_t c) {
    return linear_expression_scalar_mul(std::forward<E>(e), c);
};
template <linear_expression_c E>
constexpr auto operator*(typename E::scalar_t c, E && e) {
    return e * c;
};
template <linear_expression_c E>
constexpr auto operator/(E && e, typename E::scalar_t c) {
    return e * (typename E::scalar_t{1} / c);
};
template <linear_expression_c E>
constexpr auto operator/(typename E::scalar_t c, E && e) {
    return e / c;
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_LINEAR_EXPRESSION_OPERATORS_HPP