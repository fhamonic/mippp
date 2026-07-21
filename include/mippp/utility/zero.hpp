#pragma once

#include <concepts>
#include <type_traits>
#include <utility>

namespace mippp {

struct zero_t;

template <typename T>
concept statically_zero = std::same_as<std::remove_cvref_t<T>, zero_t>;

// A compile-time zero that converts implicitly to any scalar type, so consumers
// expecting a runtime scalar keep working unchanged.
//
// No type in MIP++ should declare a conversion to zero_t.
// Arithmetic and comparison operators are hidden friends: found by ADL only, so
// they are candidates exactly when an operand *is* a zero_t and don't
// participate to unqualified lookups
struct zero_t {
    template <typename S>
        requires std::convertible_to<int, S> && (!statically_zero<S>)
    [[nodiscard]] constexpr operator S() const noexcept {
        return S{0};
    }

    constexpr zero_t & operator+=(zero_t) noexcept { return *this; }
    constexpr zero_t & operator-=(zero_t) noexcept { return *this; }
    constexpr zero_t & operator*=(zero_t) noexcept { return *this; }

    // Accumulating a zero onto a runtime scalar needs these explicitly: the
    // built-in candidates are `operator+=(S &, R)` for *every* promoted
    // arithmetic R, and the conversion above -- being a template -- satisfies
    // all of them equally well, so `s += zero` would be ambiguous. Binary `+`
    // escapes this only because its friend below is an exact match.
    // `convertible_to<int, S>` keeps class types out; without it these would
    // capture types that define their own `+=`, such as
    // runtime_linear_expression.
    // note: `s *= zero` is intentionally not provided, for the same reason as
    // `s / zero`: silently zeroing a scalar should be spelled out.
    template <typename S>
        requires(!statically_zero<S>) && std::convertible_to<int, S>
    friend constexpr S & operator+=(S & s, zero_t) noexcept {
        return s;
    }
    template <typename S>
        requires(!statically_zero<S>) && std::convertible_to<int, S>
    friend constexpr S & operator-=(S & s, zero_t) noexcept {
        return s;
    }

    template <typename S>
    using scalar_of = std::remove_cvref_t<S>;

    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// Addition /////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    [[nodiscard]] friend constexpr zero_t operator+(zero_t) noexcept {
        return {};
    }
    [[nodiscard]] friend constexpr zero_t operator+(zero_t, zero_t) noexcept {
        return {};
    }

    template <typename S>
        requires(!statically_zero<S>)
    [[nodiscard]] friend constexpr scalar_of<S> operator+(
        zero_t,
        S && s) noexcept(std::is_nothrow_constructible_v<scalar_of<S>, S>) {
        return std::forward<S>(s);
    }
    template <typename S>
        requires(!statically_zero<S>)
    [[nodiscard]] friend constexpr scalar_of<S> operator+(
        S && s,
        zero_t) noexcept(std::is_nothrow_constructible_v<scalar_of<S>, S>) {
        return std::forward<S>(s);
    }

    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////// Substraction ///////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    [[nodiscard]] friend constexpr zero_t operator-(zero_t) noexcept {
        return {};
    }
    [[nodiscard]] friend constexpr zero_t operator-(zero_t, zero_t) noexcept {
        return {};
    }

    template <typename S>
        requires(!statically_zero<S>)
    [[nodiscard]] friend constexpr auto operator-(zero_t, S && s) noexcept(
        noexcept(-std::forward<S>(s))) {
        return -std::forward<S>(s);
    }
    template <typename S>
        requires(!statically_zero<S>)
    [[nodiscard]] friend constexpr scalar_of<S> operator-(
        S && s,
        zero_t) noexcept(std::is_nothrow_constructible_v<scalar_of<S>, S>) {
        return std::forward<S>(s);
    }

    ///////////////////////////////////////////////////////////////////////////
    /////////////////////// Multiplication / Division /////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    [[nodiscard]] friend constexpr zero_t operator*(zero_t, zero_t) noexcept {
        return {};
    }

    // `*` and `/` fold to zero_t, discarding the other operand, so
    // `convertible_to<int, S>` restricts them to scalars: without it
    // `zero * expr` would silently swallow a whole expression (and be
    // ISO-ambiguous with the expression operators, whose scalar overloads
    // also match through the zero_t -> scalar conversion). A zero
    // coefficient on an expression goes through the expression operators
    // instead and keeps its terms, spelled `0.0 * expr`.
    template <typename S>
        requires(!statically_zero<S>) && std::convertible_to<int, S>
    [[nodiscard]] friend constexpr zero_t operator*(zero_t,
                                                    const S &) noexcept {
        return {};
    }
    template <typename S>
        requires(!statically_zero<S>) && std::convertible_to<int, S>
    [[nodiscard]] friend constexpr zero_t operator*(const S &,
                                                    zero_t) noexcept {
        return {};
    }

    // note: `s / zero` and `zero / zero` are intentionally not provided.
    template <typename S>
        requires(!statically_zero<S>) && std::convertible_to<int, S>
    [[nodiscard]] friend constexpr zero_t operator/(zero_t,
                                                    const S &) noexcept {
        return {};
    }

    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////// Comparison /////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    // `s == zero` uses the reversed candidate synthesized from this
    // declaration; `!=` is rewritten from `==`. Both are found by ADL.
    [[nodiscard]] friend constexpr bool operator==(zero_t, zero_t) noexcept {
        return true;
    }

    template <typename S>
        requires(!statically_zero<S>) && std::convertible_to<int, S> &&
                std::equality_comparable<S>
    [[nodiscard]] friend constexpr bool operator==(
        zero_t, const S & s) noexcept(noexcept(s == S{0})) {
        return s == S{0};
    }
};

inline constexpr zero_t zero{};

}  // namespace mippp