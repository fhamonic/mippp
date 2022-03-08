#ifndef MIPPP_VARIABLE_HPP
#define MIPPP_VARIABLE_HPP

#include <range/v3/numeric/inner_product.hpp>
#include <range/v3/view/transform.hpp>

#include "mippp/concepts/id_value_map.hpp"
#include "mippp/concepts/linear_expression.hpp"

namespace fhamonic {
namespace mippp {

template <linear_expression_c E, typename M>
requires id_value_map<M, typename E::var_id_t, typename E::scalar_t>
constexpr typename E::scalar_t evaluate(E && e, M && m) {
    return e.constant() +
           ranges::inner_product(
               ranges::views::transform(
                   e.variables(), [&m](auto && var_id) { return m[var_id]; }),
               e.coefficients());
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_VARIABLE_HPP