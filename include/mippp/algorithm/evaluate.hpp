#ifndef MIPPP_ALGORITHM_EVALUATE_HPP
#define MIPPP_ALGORITHM_EVALUATE_HPP

#include <range/v3/view/transform.hpp>

#include "mippp/concepts/id_value_map.hpp"
#include "mippp/concepts/linear_expression.hpp"

namespace fhamonic {
namespace mippp {

template <expression E, typename M>
    requires id_value_map<M, expression_variable_id_t<E>,
                          expression_scalar_t<E>>
constexpr expression_scalar_t<E> evaluate(E && e, M && m) {
    return ranges::accumulate(
        ranges::views::transform(
            e.terms(),
            [&m](auto && t) { return std::get<0>(t) * m[std::get<1>(t)]; }),
        e.constant());
}

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_ALGORITHM_EVALUATE_HPP