#ifndef MIPPP_LINEAR_CONSTRAINT_HPP
#define MIPPP_LINEAR_CONSTRAINT_HPP

#include <cassert>
#include <ranges>
#include <type_traits>

#include <range/v3/view/single.hpp>

#include "mippp/concepts/linear_expression.hpp"

namespace fhamonic {
namespace mippp {

template <std::ranges::range V, std::ranges::range C, typename S>
requires std::same_as<std::remove_reference_t<S>,
                      std::remove_reference_t<std::ranges::range_value_t<C>>>
class linear_constraint {
public:
    using var_id_t = std::ranges::range_value_t<V>;
    using scalar_t = std::ranges::range_value_t<C>;

private:
    V _variables;
    C _coefficients;
    S _lb;
    S _ub;

public:
    constexpr linear_constraint(V && variables, C && coefficients,
                                S lb = scalar_t{0}, S ub = scalar_t{0})
        : _variables(std::forward<V>(variables))
        , _coefficients(std::forward<C>(coefficients))
        , _lb(lb)
        , _ub(ub){};

    constexpr auto variables() const noexcept { return _variables; }
    constexpr auto coefficients() const noexcept { return _coefficients; }
    constexpr scalar_t lower_bound() const noexcept { return _lb; }
    constexpr scalar_t upper_bound() const noexcept { return _ub; }
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_LINEAR_CONSTRAINT_HPP