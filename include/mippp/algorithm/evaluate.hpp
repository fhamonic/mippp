#ifndef MIPPP_ALGORITHM_EVALUATE_HPP
#define MIPPP_ALGORITHM_EVALUATE_HPP

#include <range/v3/numeric/inner_product.hpp>
#include <range/v3/view/transform.hpp>

#include "mippp/concepts/id_value_map.hpp"
#include "mippp/concepts/linear_expression.hpp"

namespace fhamonic {
namespace mippp {

template <linear_expression_c E, typename M>
requires id_value_map<M, expression_var_id_t<E>, expression_scalar_t<E>>
constexpr expression_scalar_t<E> evaluate(E && e, M && m) {
    return e.constant() +
           ranges::inner_product(
               ranges::views::transform(
                   e.variables(), [&m](auto && var_id) { return m[var_id]; }),
               e.coefficients());
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_ALGORITHM_EVALUATE_HPP