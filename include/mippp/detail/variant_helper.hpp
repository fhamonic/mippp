#pragma once

#include <concepts>
#include <type_traits>
#include <variant>

namespace mippp {
namespace detail {

template <class... Ts>
struct overloaded : Ts... {
    using Ts::operator()...;
};
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

template <template <typename, typename> typename C, typename T,
          typename Variant>
inline constexpr bool any_alternative_v = false;

template <template <typename, typename> typename C, typename T, typename... Ts>
inline constexpr bool any_alternative_v<C, T, std::variant<Ts...>> =
    (C<T, Ts>::value || ...);

template <template <typename, typename> typename C, typename T,
          typename Variant>
inline constexpr bool all_alternatives_v = false;

template <template <typename, typename> typename C, typename T, typename... Ts>
inline constexpr bool all_alternatives_v<C, T, std::variant<Ts...>> =
    (C<T, Ts>::value && ...);

template <typename Base, typename D>
using is_derived_from = std::bool_constant<std::derived_from<D, Base>>;

}  // namespace detail

template <typename V, typename T>
concept variant_with_alternative =
    detail::any_alternative_v<std::is_same, T, std::remove_cvref_t<V>>;

template <typename V, typename T>
concept variant_of = detail::all_alternatives_v<detail::is_derived_from, T,
                                                std::remove_cvref_t<V>>;

template <typename V, typename T>
concept variant_containing_a =
    detail::any_alternative_v<detail::is_derived_from, T,
                              std::remove_cvref_t<V>>;

template <typename S>
[[nodiscard]] constexpr bool is(
    const variant_with_alternative<S> auto & r) noexcept {
    return std::holds_alternative<S>(r);
}

template <typename S>
[[nodiscard]] constexpr bool is_a(
    const variant_containing_a<S> auto & s) noexcept {
    return std::visit(
        []<typename A>(const A &) { return std::derived_from<A, S>; }, s);
}

}  // namespace mippp
