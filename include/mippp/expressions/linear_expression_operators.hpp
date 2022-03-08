#ifndef MIPPP_LINEAR_EXPRESSION_OPERATORS_HPP
#define MIPPP_LINEAR_EXPRESSION_OPERATORS_HPP

#include <type_traits>

#include "mippp/expressions/linear_expression_operations.hpp"

namespace fhamonic {
namespace mippp {

template <linear_expression_c E1, linear_expression_c E2>
requires linear_expression_c<typename std::remove_reference<E1>::type> &&
    linear_expression_c<typename std::remove_reference<E2>::type> &&
    std::same_as<typename E1::var_id_t, typename E2::var_id_t> &&
    std::same_as<typename E1::scalar_t, typename E2::scalar_t>
constexpr auto operator+(const E1 && e1, const E2 && e2) {
    return linear_expression_add(std::forward<const E1>(e1), std::forward<const E2>(e2));
};

template <typename E>
requires linear_expression_c<typename std::remove_reference<E>::type>
constexpr auto operator-(const E && e) {
    return linear_expression_negate(std::forward<const E>(e));
};

template <linear_expression_c E1, linear_expression_c E2>
requires std::same_as<typename E1::var_id_t, typename E2::var_id_t> &&
    std::same_as<typename E1::scalar_t, typename E2::scalar_t>
constexpr auto operator-(const E1 && e1, const E2 && e2) {
    return linear_expression_sub(std::forward<const E1>(e1), std::forward<const E2>(e2));
};

template <typename E>
requires linear_expression_c<typename std::remove_reference<E>::type>
constexpr auto operator+(const E && e, typename E::scalar_t c) {
    return linear_expression_scalar_add(std::forward<const E>(e), c);
};

template <typename E>
requires linear_expression_c<typename std::remove_reference<E>::type>
constexpr auto operator+(typename E::scalar_t c, const E && e) {
    return linear_expression_scalar_add(std::forward<const E>(e), c);
};

template <typename E>
requires linear_expression_c<typename std::remove_reference<E>::type>
constexpr auto operator-(const E && e, typename E::scalar_t c) {
    return linear_expression_scalar_add(std::forward<const E>(e), -c);
};

template <typename E>
requires linear_expression_c<typename std::remove_reference<E>::type>
constexpr auto operator-(typename E::scalar_t c, const E && e) {
    return linear_expression_scalar_sub_other_way(std::forward<const E>(e), c);
};

template <typename E>
requires linear_expression_c<typename std::remove_reference<E>::type>
constexpr auto operator*(const E && e, typename E::scalar_t c) {
    return linear_expression_scalar_mul(std::forward<const E>(e), c);
};

template <typename E>
requires linear_expression_c<typename std::remove_reference<E>::type>
constexpr auto operator*(typename E::scalar_t c, const E && e) {
    return linear_expression_scalar_mul(std::forward<const E>(e), c);
};

template <typename E>
requires linear_expression_c<typename std::remove_reference<E>::type>
constexpr auto operator/(const E && e, typename E::scalar_t c) {
    return linear_expression_scalar_mul(std::forward<const E>(e),
                                        typename E::scalar_t{1} / c);
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_LINEAR_EXPRESSION_OPERATORS_HPP