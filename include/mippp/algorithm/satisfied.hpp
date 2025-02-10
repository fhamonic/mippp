#ifndef MIPPP_ALGORITHM_SATISFIED_HPP
#define MIPPP_ALGORITHM_SATISFIED_HPP

#include "mippp/algorithm/evaluate.hpp"
#include "mippp/concepts/id_value_map.hpp"
#include "mippp/concepts/linear_constraint.hpp"

namespace fhamonic {
namespace mippp {

template <linear_constraint C, typename M>
    requires id_value_map<M, constraint_variable_id_t<C>,
                          constraint_scalar_t<C>>
constexpr bool satisfied(C && c, M && m) {
    auto value = evaluate(c.expression(), m);
    switch(c.relation()) {
        case constraint_relation::less_equal_zero:
            return value <= 0;
        case constraint_relation::greater_equal_zero:
            return value >= 0;
        case constraint_relation::equal_zero:
            return value == 0;
    }
}

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_ALGORITHM_SATISFIED_HPP