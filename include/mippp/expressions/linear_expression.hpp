#ifndef MIPPP_LINEAR_EXPRESSION_HPP
#define MIPPP_LINEAR_EXPRESSION_HPP

#include <cassert>
#include <concepts>
#include <ranges>
#include <type_traits>

#include <range/v3/view/all.hpp>

#include "mippp/concepts/linear_expression.hpp"

namespace fhamonic {
namespace mippp {

template <typename _Vars, typename _Coefs>
class linear_expression {
public:
    using var_id_t = std::ranges::range_value_t<_Vars>;
    using scalar_t = std::ranges::range_value_t<_Coefs>;

private:
    _Vars _variables;
    _Coefs _coefficients;
    scalar_t _constant;

public:
    template <std::ranges::range V, std::ranges::range C>
    [[nodiscard]] constexpr linear_expression(V && variables, C && coefficients)
        : _variables(ranges::views::all(std::forward<V>(variables)))
        , _coefficients(ranges::views::all(std::forward<C>(coefficients)))
        , _constant(static_cast<scalar_t>(0)) {}

    template <std::ranges::range V, std::ranges::range C, typename S>
    [[nodiscard]] constexpr linear_expression(V && variables, C && coefficients,
                                              S constant)
        : _variables(ranges::views::all(std::forward<V>(variables)))
        , _coefficients(ranges::views::all(std::forward<C>(coefficients)))
        , _constant(static_cast<scalar_t>(constant)) {}

    [[nodiscard]] constexpr const _Vars & variables() const & noexcept {
        return _variables;
    }
    [[nodiscard]] constexpr _Vars && variables() && noexcept {
        return std::move(_variables);
    }
    [[nodiscard]] constexpr const _Coefs & coefficients() const & noexcept {
        return _coefficients;
    }
    [[nodiscard]] constexpr _Coefs && coefficients() && noexcept {
        return std::move(_coefficients);
    }
    [[nodiscard]] constexpr const scalar_t & constant() const & noexcept {
        return _constant;
    }
    [[nodiscard]] constexpr scalar_t && constant() && noexcept {
        return std::move(_constant);
    }
};

template <typename V, typename C>
linear_expression(V &&, C &&)
    -> linear_expression<ranges::views::all_t<V>, ranges::views::all_t<C>>;

template <typename V, typename C, typename S>
    requires std::convertible_to<S, std::ranges::range_value_t<C>>
linear_expression(V &&, C &&, S)
    -> linear_expression<ranges::views::all_t<V>, ranges::views::all_t<C>>;

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_LINEAR_EXPRESSION_HPP