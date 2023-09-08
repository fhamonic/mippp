#ifndef MIPPP_XSUM_HPP
#define MIPPP_XSUM_HPP

#include <concepts>
#include <ranges>

#include <range/v3/view/transform.hpp>

#include "expressions/linear_expression.hpp"
#include "variable.hpp"

namespace fhamonic {
namespace mippp {

template <std::ranges::range R, typename V, typename C>
requires requires(std::ranges::range_value_t<R> v, V vars, C coefs) {
    vars(v).id();
    { coefs(v) } -> std::convertible_to<typename decltype(vars(v))::scalar_t>;
}
constexpr auto xsum(R && values, V && vars_map, C && coefs_map) noexcept {
    using variable_t =
        decltype(vars_map(std::declval<std::ranges::range_value_t<R>>()));
    using scalar_t = typename variable_t::scalar_t;
    return linear_expression(
        ranges::views::transform(ranges::views::transform(values, vars_map),
                                 &variable_t::id),
        ranges::views::transform(values,
                                 [&coefs_map](auto && v) {
                                     return static_cast<scalar_t>(coefs_map(v));
                                 }),
        scalar_t{0});
}

template <std::ranges::range R, typename V, typename C>
requires requires(std::ranges::range_value_t<R> v, V vars, C coefs) {
    vars(v).id();
    { coefs[v] } -> std::convertible_to<typename decltype(vars(v))::scalar_t>;
}
constexpr auto xsum(R && values, V && vars_map, C && coefs_map) noexcept {
    using variable_t =
        decltype(vars_map(std::declval<std::ranges::range_value_t<R>>()));
    using scalar_t = typename variable_t::scalar_t;
    return linear_expression(
        ranges::views::transform(ranges::views::transform(values, vars_map),
                                 &variable_t::id),
        ranges::views::transform(values,
                                 [&coefs_map](auto && v) {
                                     return static_cast<scalar_t>(coefs_map[v]);
                                 }),
        scalar_t{0});
}

template <std::ranges::range R, typename V>
constexpr auto xsum(R && values, V && vars_map) noexcept {
    using variable_t =
        decltype(vars_map(std::declval<std::ranges::range_value_t<R>>()));
    using scalar_t = typename variable_t::scalar_t;
    return linear_expression(
        ranges::views::transform(ranges::views::transform(values, vars_map),
                                 &variable_t::id),
        ranges::views::transform(values, [](auto && v) { return scalar_t{1}; }),
        scalar_t{0});
}

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_XSUM_HPP