#ifndef MIPPP_LINEAR_EXPRESSION_HPP
#define MIPPP_LINEAR_EXPRESSION_HPP

#include <cassert>
#include <ranges>
#include <type_traits>

#include "mippp/concepts/linear_expression.hpp"

namespace fhamonic {
namespace mippp {

template <std::ranges::range V, std::ranges::range C, typename S>
requires std::same_as<std::remove_reference_t<S>,
                      std::remove_reference_t<std::ranges::range_value_t<C>>>
class linear_expression {
public:
    using var_id_t = std::ranges::range_value_t<V>;
    using scalar_t = std::ranges::range_value_t<C>;

private:
    V _variables;
    C _coefficients;
    S _constant;

public:
    [[nodiscard]] constexpr linear_expression(V && variables, C && coefficients,
                                              S constant = scalar_t{0})
        : _variables(std::forward<V>(variables))
        , _coefficients(std::forward<C>(coefficients))
        , _constant(constant){};

    [[nodiscard]] constexpr const V & variables() const noexcept {
        return _variables;
    }
    [[nodiscard]] constexpr const C & coefficients() const noexcept {
        return _coefficients;
    }
    [[nodiscard]] constexpr scalar_t constant() const noexcept {
        return _constant;
    }
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_LINEAR_EXPRESSION_HPP