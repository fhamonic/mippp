#ifndef MIPPP_ALGORITHM_SATISFIED_HPP
#define MIPPP_ALGORITHM_SATISFIED_HPP

#include "mippp/algorithm/evaluate.hpp"
#include "mippp/concepts/id_value_map.hpp"
#include "mippp/concepts/linear_constraint.hpp"
#include "mippp/expressions/linear_expression.hpp"

namespace fhamonic {
namespace mippp {

template <linear_constraint_c C, typename M>
requires id_value_map<M, constraint_var_id_t<C>, constraint_scalar_t<C>>
constexpr bool satisfied(C && c, M && m) {
    using scalar_t = constraint_scalar_t<C>;
    scalar_t value =
        evaluate(linear_expression(c.variables(), c.coefficients()), m);
    return c.lower_bound() <= value && value <= c.upper_bound();
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_ALGORITHM_SATISFIED_HPP