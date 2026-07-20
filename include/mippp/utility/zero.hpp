#pragma once

#include <concepts>
#include <type_traits>
#include <utility>

namespace mippp {

struct zero_t;

template <typename T>
concept statically_zero = std::same_as<std::remove_cvref_t<T>, zero_t>;

// A compile-time zero. Converts implicitly to any scalar type, so consumers
// expecting a runtime scalar keep working unchanged.
struct zero_t {
    template <typename S>
        requires std::convertible_to<int, S> && (!statically_zero<S>)
    [[nodiscard]] constexpr operator S() const noexcept {
        return S{0};
    }
    constexpr zero_t & operator+=(zero_t) noexcept { return *this; }
    constexpr zero_t & operator-=(zero_t) noexcept { return *this; }
    constexpr zero_t & operator*=(zero_t) noexcept { return *this; }
};

inline constexpr zero_t zero{};

template <typename S>
using scalar_of = std::remove_cvref_t<S>;

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Addition ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

[[nodiscard]] constexpr zero_t operator+(zero_t, zero_t) noexcept { return {}; }

template <typename S>
    requires(!statically_zero<S>)
[[nodiscard]] constexpr scalar_of<S> operator+(zero_t, S && s) noexcept(
    std::is_nothrow_constructible_v<scalar_of<S>, S>) {
    return std::forward<S>(s);
}
template <typename S>
    requires(!statically_zero<S>)
[[nodiscard]] constexpr scalar_of<S> operator+(S && s, zero_t) noexcept(
    std::is_nothrow_constructible_v<scalar_of<S>, S>) {
    return std::forward<S>(s);
}

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// Substraction /////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

[[nodiscard]] constexpr zero_t operator-(zero_t) noexcept { return {}; }
[[nodiscard]] constexpr zero_t operator-(zero_t, zero_t) noexcept { return {}; }

template <typename S>
    requires(!statically_zero<S>)
[[nodiscard]] constexpr auto operator-(zero_t, S && s) noexcept(
    noexcept(-std::forward<S>(s))) {
    return -std::forward<S>(s);
}
template <typename S>
    requires(!statically_zero<S>)
[[nodiscard]] constexpr scalar_of<S> operator-(S && s, zero_t) noexcept(
    std::is_nothrow_constructible_v<scalar_of<S>, S>) {
    return std::forward<S>(s);
}

///////////////////////////////////////////////////////////////////////////////
////////////////////////// Multiplication / Division //////////////////////////
///////////////////////////////////////////////////////////////////////////////

[[nodiscard]] constexpr zero_t operator*(zero_t, zero_t) noexcept { return {}; }
template <typename S>
    requires(!statically_zero<S>)
[[nodiscard]] constexpr zero_t operator*(zero_t, const S &) noexcept {
    return {};
}
template <typename S>
    requires(!statically_zero<S>)
[[nodiscard]] constexpr zero_t operator*(const S &, zero_t) noexcept {
    return {};
}

// note: `s / zero` and `zero / zero` are intentionally not provided.
template <typename S>
    requires(!statically_zero<S>)
[[nodiscard]] constexpr zero_t operator/(zero_t, const S &) noexcept {
    return {};
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////// Comparison //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

[[nodiscard]] constexpr bool operator==(zero_t, zero_t) noexcept {
    return true;
}
template <typename S>
    requires(!statically_zero<S>) && std::convertible_to<int, S> &&
            std::equality_comparable<S>
[[nodiscard]] constexpr bool operator==(zero_t,
                                        const S & s) noexcept(noexcept(s ==
                                                                       S{0})) {
    return s == S{0};
}

}  // namespace mippp
