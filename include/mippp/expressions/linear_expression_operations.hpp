#ifndef MIPPP_LINEAR_EXPRESSION_OPERATIONS_HPP
#define MIPPP_LINEAR_EXPRESSION_OPERATIONS_HPP

#include <functional>

#include <iostream>

#include <range/v3/view/concat.hpp>
#include <range/v3/view/transform.hpp>

#include "mippp/concepts/linear_expression.hpp"
#include "mippp/expressions/linear_expression.hpp"

namespace fhamonic {
namespace mippp {

template <linear_expression_c E1, linear_expression_c E2>
constexpr auto linear_expression_add(E1 && e1, E2 && e2) {
    return linear_expression(
        ranges::views::concat(std::forward<E1>(e1).variables(),
                              std::forward<E2>(e2).variables()),
        ranges::views::concat(std::forward<E1>(e1).coefficients(),
                              std::forward<E2>(e2).coefficients()),
        std::forward<E1>(e1).constant() + std::forward<E2>(e2).constant());
}

template <linear_expression_c E>
constexpr auto linear_expression_negate(E && e) {
    using scalar_t = expression_scalar_t<E>;
    return linear_expression(
        std::forward<E>(e).variables(),
        ranges::views::transform(std::forward<E>(e).coefficients(),
                                 std::negate<scalar_t>()),
        -std::forward<E>(e).constant());
}

template <linear_expression_c E, typename S>
constexpr auto linear_expression_scalar_add(E && e, const S c) {
    using scalar_t = expression_scalar_t<E>;
    return linear_expression(
        std::forward<E>(e).variables(), std::forward<E>(e).coefficients(),
        std::forward<E>(e).constant() + static_cast<scalar_t>(c));
}

template <linear_expression_c E, typename S>
constexpr auto linear_expression_scalar_mul(E && e, const S c) {
    using scalar_t = expression_scalar_t<E>;
    return linear_expression(
        std::forward<E>(e).variables(),
        ranges::views::transform(
            std::forward<E>(e).coefficients(),
            [c](auto && coef) -> scalar_t { return c * coef; }),
        std::forward<E>(e).constant() * c);
}

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_LINEAR_EXPRESSION_OPERATIONS_HPP