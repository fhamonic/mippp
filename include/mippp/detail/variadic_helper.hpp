#pragma once

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

}  // namespace mippp::detail
