#pragma once

#include <concepts>
#include <cstddef>
#include <functional>
#include <tuple>
#include <type_traits>
#include <utility>

namespace mippp::detail {

// Invoking a user function on a key. The function is called with the key
// itself whenever that is viable; otherwise a std::pair or std::tuple key is
// unpacked into one argument per element, so that a generator over a
// cartesian product may be written [](int i, int j) as well as [](auto && p).
// Trying the key first keeps every generic lambda receiving the whole key.

template <typename T>
inline constexpr bool is_pair_or_tuple_v = false;
template <typename... Ts>
inline constexpr bool is_pair_or_tuple_v<std::tuple<Ts...>> = true;
template <typename A, typename B>
inline constexpr bool is_pair_or_tuple_v<std::pair<A, B>> = true;

template <typename F, typename Key, typename Seq>
struct unpacked_invocable_impl : std::false_type {};
template <typename F, typename Key, std::size_t... I>
struct unpacked_invocable_impl<F, Key, std::index_sequence<I...>>
    : std::bool_constant<
          std::invocable<F, decltype(std::get<I>(std::declval<Key>()))...>> {};

template <typename F, typename Key>
concept unpacked_invocable =
    is_pair_or_tuple_v<std::remove_cvref_t<Key>> &&
    unpacked_invocable_impl<F, Key,
                            std::make_index_sequence<std::tuple_size_v<
                                std::remove_cvref_t<Key>>>>::value;

template <typename F, typename Key>
concept key_invocable = std::invocable<F, Key> || unpacked_invocable<F, Key>;

template <typename F, typename Key, std::size_t... I>
constexpr decltype(auto) invoke_unpacked(F && f, Key && key,
                                         std::index_sequence<I...>) {
    return std::invoke(std::forward<F>(f),
                       std::get<I>(std::forward<Key>(key))...);
}

template <typename F, typename Key>
    requires key_invocable<F, Key>
constexpr decltype(auto) invoke_key(F && f, Key && key) {
    if constexpr(std::invocable<F, Key>)
        return std::invoke(std::forward<F>(f), std::forward<Key>(key));
    else
        return invoke_unpacked(
            std::forward<F>(f), std::forward<Key>(key),
            std::make_index_sequence<
                std::tuple_size_v<std::remove_cvref_t<Key>>>{});
}

template <typename F, typename Key>
    requires key_invocable<F, Key>
using key_invoke_result_t =
    decltype(invoke_key(std::declval<F>(), std::declval<Key>()));

// A function object applying invoke_key, for range adaptors that call their
// function with one argument.
template <typename F>
struct key_fn {
    F f;
    template <typename Key>
        requires key_invocable<F &, Key>
    constexpr decltype(auto) operator()(Key && key) {
        return invoke_key(f, std::forward<Key>(key));
    }
    template <typename Key>
        requires key_invocable<const F &, Key>
    constexpr decltype(auto) operator()(Key && key) const {
        return invoke_key(f, std::forward<Key>(key));
    }
};

}  // namespace mippp::detail
