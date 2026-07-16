#ifndef MIPPP_VARIADIC_HELPER_HPP
#define MIPPP_VARIADIC_HELPER_HPP

#include <array>
#include <concepts>
#include <ranges>
#include <type_traits>
#include <variant>

namespace mippp::detail {

template <typename T, typename... Ts>
inline constexpr bool contains_v = (std::is_same_v<T, Ts> || ...);

template <typename T, typename... Ts>
    requires contains_v<T, Ts...>
inline constexpr std::size_t index_of_v = []() {
    constexpr std::array<bool, sizeof...(Ts)> matches{std::is_same_v<T, Ts>...};
    return static_cast<std::size_t>(std::ranges::find(matches, true) -
                                    matches.begin());
}();

template <typename T, typename Variant>
inline constexpr bool variant_contains_v = false;

template <typename T, typename... Ts>
inline constexpr bool variant_contains_v<T, std::variant<Ts...>> =
    contains_v<T, Ts...>;

template <class... Ts>
struct overloaded : Ts... {
    using Ts::operator()...;
};
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

}  // namespace mippp::detail

#endif  // MIPPP_VARIADIC_HELPER_HPP
