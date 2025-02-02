#ifndef MIPPP_LINEAR_CONSTRAINT_HPP
#define MIPPP_LINEAR_CONSTRAINT_HPP

#include <cassert>
#include <ranges>
#include <type_traits>

#include <range/v3/view/all.hpp>
#include <range/v3/view/single.hpp>

#include "mippp/concepts/linear_expression.hpp"

namespace fhamonic {
namespace mippp {

template <typename _Vars, typename _Coefs>
class linear_constraint {
public:
    using var_id_t = std::ranges::range_value_t<_Vars>;
    using scalar_t = std::ranges::range_value_t<_Coefs>;

private:
    _Vars _variables;
    _Coefs _coefficients;
    scalar_t _lb;
    scalar_t _ub;

public:
    template <std::ranges::range V, std::ranges::range C, typename S>
    constexpr linear_constraint(V && variables, C && coefficients, S lb = 0,
                                S ub = 0)
        : _variables(ranges::views::all(std::forward<V>(variables)))
        , _coefficients(ranges::views::all(std::forward<C>(coefficients)))
        , _lb(static_cast<scalar_t>(lb))
        , _ub(static_cast<scalar_t>(ub)) {}

    constexpr const _Vars & variables() const noexcept { return _variables; }
    constexpr const _Coefs & coefficients() const noexcept {
        return _coefficients;
    }
    constexpr scalar_t lower_bound() const noexcept { return _lb; }
    constexpr scalar_t upper_bound() const noexcept { return _ub; }
};

template <typename V, typename C, typename S>
    requires std::convertible_to<S, std::ranges::range_value_t<C>>
linear_constraint(V &&, C &&, S, S)
    -> linear_constraint<ranges::views::all_t<V>, ranges::views::all_t<C>>;

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_LINEAR_CONSTRAINT_HPP