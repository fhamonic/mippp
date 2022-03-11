#ifndef MIPPP_LINEAR_CONSTRAINT_OPERATORS_HPP
#define MIPPP_LINEAR_CONSTRAINT_OPERATORS_HPP

#include <type_traits>

#include "mippp/expressions/linear_constraint_operations.hpp"

namespace fhamonic {
namespace mippp {

template <linear_expression_c E1, linear_expression_c E2>
requires std::same_as<expression_var_id_t<E1>, expression_var_id_t<E2>> &&
    std::same_as<expression_scalar_t<E1>, expression_scalar_t<E2>>
constexpr auto operator<=(E1 && e1, E2 && e2) {
    return linear_constraint_less_equal<E1 &&, E2 &&>(std::forward<E1>(e1),
                                                      std::forward<E2>(e2));
};
template <linear_expression_c E1, linear_expression_c E2>
requires std::same_as<expression_var_id_t<E1>, expression_var_id_t<E2>> &&
    std::same_as<expression_scalar_t<E1>, expression_scalar_t<E2>>
constexpr auto operator>=(E1 && e1, E2 && e2) {
    return linear_constraint_less_equal<E2 &&, E1 &&>(std::forward<E2>(e2),
                                                      std::forward<E1>(e1));
};

template <linear_expression_c E>
constexpr auto operator<=(expression_scalar_t<E> c, E && e) {
    return linear_constraint_lower_bound<E &&>(std::forward<E>(e), c);
};
template <linear_expression_c E>
constexpr auto operator>=(E && e, expression_scalar_t<E> c) {
    return linear_constraint_lower_bound<E &&>(std::forward<E>(e), c);
};

template <linear_expression_c E>
constexpr auto operator<=(E && e, expression_scalar_t<E> c) {
    return linear_constraint_upper_bound<E &&>(std::forward<E>(e), c);
};
template <linear_expression_c E>
constexpr auto operator>=(expression_scalar_t<E> c, E && e) {
    return linear_constraint_upper_bound<E &&>(std::forward<E>(e), c);
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_LINEAR_CONSTRAINT_OPERATORS_HPP