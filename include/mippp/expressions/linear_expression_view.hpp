#ifndef MIPPP_LINEAR_EXPRESSION_VIEW_HPP
#define MIPPP_LINEAR_EXPRESSION_VIEW_HPP

#include <cassert>
#include <ranges>
#include <type_traits>

#include <range/v3/view/single.hpp>

namespace fhamonic {
namespace mippp {

template <std::ranges::range V, std::ranges::range C, typename S>
requires std::same_as<std::remove_reference_t<S>,
                      std::remove_reference_t<std::ranges::range_value_t<C>>>
class linear_expression_view {
public:
    using var_id_t = std::ranges::range_value_t<V>;
    using scalar_t = std::ranges::range_value_t<C>;

private:
    V _variables;
    C _coefficients;
    S _constant;

public:
    constexpr linear_expression_view(V variables, C coefficients, S constant=scalar_t{0})
        : _variables(variables)
        , _coefficients(coefficients)
        , _constant(constant){};

    constexpr auto variables() const noexcept { return _variables; }
    constexpr auto coefficients() const noexcept { return _coefficients; }
    constexpr scalar_t constant() const noexcept { return _constant; }
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_LINEAR_EXPRESSION_VIEW_HPP