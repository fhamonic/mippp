#ifndef MIPPP_LINEAR_CONSTRAINT_HPP
#define MIPPP_LINEAR_CONSTRAINT_HPP

#include <cassert>
#include <ranges>
#include <type_traits>

#include <range/v3/view/single.hpp>

#include "mippp/concepts/linear_expression.hpp"

namespace fhamonic {
namespace mippp {

template <linear_expression_c E, typename S>
requires std::same_as<std::remove_reference_t<S>, expression_scalar_t<E>>
class linear_constraint {
public:
    using var_id_t = expression_var_id_t<E>;
    using scalar_t = expression_scalar_t<E>;

private:
    E _expression;
    S _lb;
    S _ub;

public:
    constexpr linear_constraint(E && _expression, S lb = scalar_t{0},
                                S ub = scalar_t{0})
        : _expression(std::forward<E>(expression)), _lb(lb), _ub(ub){};

    constexpr auto expression() const noexcept { return _expression; }
    constexpr scalar_t lower_bound() const noexcept { return _lb; }
    constexpr scalar_t lower_bound() const noexcept { return _ub; }
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_LINEAR_CONSTRAINT_HPP