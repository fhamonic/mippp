#pragma once

#include <concepts>
#include <type_traits>

namespace mippp {

// A compile-time zero. Expressions whose constant() returns zero_t are
// *statically* known to have no constant term, which lets
// linear_expressions_sum skip the eager constant fold (and its second
// range traversal), and lets quadratic products drop identically-zero
// cross terms. zero_t converts implicitly to any scalar type, so any
// consumer that expects a runtime scalar keeps working unchanged.
struct zero_t {
    template <typename S>
        requires std::convertible_to<int, S>
    [[nodiscard]] constexpr operator S() const noexcept {
        return S{0};
    }
};

template <typename T>
concept statically_zero = std::same_as<std::decay_t<T>, zero_t>;

[[nodiscard]] constexpr zero_t operator+(zero_t, zero_t) noexcept { return {}; }
template <typename S>
[[nodiscard]] constexpr S operator+(zero_t, const S & s) noexcept {
    return s;
}
template <typename S>
[[nodiscard]] constexpr S operator+(const S & s, zero_t) noexcept {
    return s;
}

[[nodiscard]] constexpr zero_t operator-(zero_t) noexcept { return {}; }
[[nodiscard]] constexpr zero_t operator-(zero_t, zero_t) noexcept { return {}; }
template <typename S>
[[nodiscard]] constexpr S operator-(zero_t, const S & s) noexcept {
    return -s;
}
template <typename S>
[[nodiscard]] constexpr S operator-(const S & s, zero_t) noexcept {
    return s;
}

[[nodiscard]] constexpr zero_t operator*(zero_t, zero_t) noexcept { return {}; }
template <typename S>
[[nodiscard]] constexpr zero_t operator*(zero_t, const S &) noexcept {
    return {};
}
template <typename S>
[[nodiscard]] constexpr zero_t operator*(const S &, zero_t) noexcept {
    return {};
}

template <typename S>
[[nodiscard]] constexpr zero_t operator/(zero_t, const S &) noexcept {
    return {};
}

[[nodiscard]] constexpr bool operator==(zero_t, zero_t) noexcept {
    return true;
}
template <typename S>
[[nodiscard]] constexpr bool operator==(zero_t,
                                        const S & s) noexcept(noexcept(s ==
                                                                       S{0})) {
    return s == S{0};
}

}  // namespace mippp
