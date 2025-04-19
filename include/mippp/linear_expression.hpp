#ifndef MIPPP_LINEAR_EXPRESSION_HPP
#define MIPPP_LINEAR_EXPRESSION_HPP

#include <concepts>
// #include <ranges>
#include <tuple>
#include <type_traits>

#include <range/v3/numeric/accumulate.hpp>
#include <range/v3/view/all.hpp>
#include <range/v3/view/concat.hpp>
#include <range/v3/view/join.hpp>
#include <range/v3/view/transform.hpp>

namespace fhamonic {
namespace mippp {

/////////////////////////////////// CONCEPT ///////////////////////////////////

template <typename _Tp>
concept linear_term = (std::tuple_size_v<_Tp> == 2);

template <typename _Tp>
using linear_term_variable_t = std::decay_t<std::tuple_element_t<0, _Tp>>;

template <typename _Tp>
using linear_term_scalar_t = std::decay_t<std::tuple_element_t<1, _Tp>>;

template <typename _Tp>
using linear_terms_range_t = decltype(std::declval<_Tp &>().linear_terms());

template <typename _Tp>
using linear_term_t = std::ranges::range_value_t<linear_terms_range_t<_Tp>>;

template <typename _Tp>
using linear_expression_scalar_t = linear_term_scalar_t<linear_term_t<_Tp>>;

template <typename _Tp>
using linear_expression_variable_t = linear_term_variable_t<linear_term_t<_Tp>>;

template <typename _Tp>
concept linear_expression = requires(const _Tp & __t) {
    { __t.linear_terms() } -> ranges::range;
    { __t.constant() } -> std::convertible_to<linear_expression_scalar_t<_Tp>>;
} && linear_term<linear_term_t<_Tp>>;

//////////////////////////////////// CLASS ////////////////////////////////////

template <typename _Terms>
class linear_expression_view {
private:
    using scalar_t = linear_term_scalar_t<std::ranges::range_value_t<_Terms>>;
    _Terms _terms;
    scalar_t _constant;

public:
    template <ranges::range T>
    [[nodiscard]] constexpr linear_expression_view(T && terms)
        : _terms(ranges::views::all(std::forward<T>(terms))), _constant(0) {}

    template <ranges::range T, typename S>
    [[nodiscard]] constexpr linear_expression_view(T && terms, S constant)
        : _terms(ranges::views::all(std::forward<T>(terms)))
        , _constant(static_cast<scalar_t>(constant)) {}

    [[nodiscard]] constexpr decltype(auto) linear_terms() const & noexcept {
        return _terms;
    }
    [[nodiscard]] constexpr _Terms && linear_terms() && noexcept {
        return std::move(_terms);
    }
    [[nodiscard]] constexpr decltype(auto) constant() const & noexcept {
        return _constant;
    }
    [[nodiscard]] constexpr scalar_t && constant() && noexcept {
        return std::move(_constant);
    }
};

template <typename T>
linear_expression_view(T &&) -> linear_expression_view<ranges::views::all_t<T>>;

template <typename T, typename S>
    requires std::convertible_to<S,
                                 linear_term_scalar_t<ranges::range_value_t<T>>>
linear_expression_view(T &&, S)
    -> linear_expression_view<ranges::views::all_t<T>>;

///////////////////////////////// OPERATIONS //////////////////////////////////

// Optimisation de la connectivité écologique des réseaux d'habitats : PLNE et
// prétraitemeent de plus courts chemins

template <linear_expression E1, linear_expression E2>
constexpr auto linear_expression_add(E1 && e1, E2 && e2) {
    return linear_expression_view(
        ranges::views::concat(std::forward<E1>(e1).linear_terms(),
                              std::forward<E2>(e2).linear_terms()),
        std::forward<E1>(e1).constant() + std::forward<E2>(e2).constant());
}

template <linear_expression E>
constexpr auto linear_expression_negate(E && e) {
    return linear_expression_view(
        ranges::views::transform(std::forward<E>(e).linear_terms(),
                                 [](auto && t) {
                                     return std::make_pair(std::get<0>(t),
                                                           -std::get<1>(t));
                                 }),
        -std::forward<E>(e).constant());
}

template <linear_expression E, typename S>
constexpr auto linear_expression_scalar_add(E && e, const S c) {
    using scalar_t = linear_expression_scalar_t<E>;
    return linear_expression_view(
        std::forward<E>(e).linear_terms(),
        std::forward<E>(e).constant() + static_cast<scalar_t>(c));
}

template <linear_expression E, typename S>
constexpr auto linear_expression_scalar_mul(E && e, const S c) {
    return linear_expression_view(
        ranges::views::transform(std::forward<E>(e).linear_terms(),
                                 [c](auto && t) {
                                     return std::make_pair(std::get<0>(t),
                                                           c * std::get<1>(t));
                                 }),
        c * std::forward<E>(e).constant());
}

template <ranges::range _Expressions>
    requires linear_expression<std::ranges::range_value_t<_Expressions>>
class linear_expressions_sum {
private:
    using linear_expression = ranges::range_value_t<_Expressions>;
    _Expressions _expressions;

public:
    template <typename E>
    linear_expressions_sum(E && e)
        : _expressions(ranges::views::all(std::forward<E>(e))) {}

    template <typename E, typename F>
    linear_expressions_sum(E && e, F && f)
        : _expressions(ranges::views::transform(std::forward<E>(e),
                                                std::forward<F>(f))) {}

    [[nodiscard]] constexpr auto linear_terms() const & noexcept {
        return ranges::views::join(ranges::views::transform(
            _expressions, [](auto && e) { return e.linear_terms(); }));
    }
    [[nodiscard]] constexpr auto constant() const & noexcept {
        using scalar_t = linear_expression_scalar_t<linear_expression>;
        return ranges::accumulate(
            ranges::views::transform(_expressions,
                                     [](auto && e) { return e.constant(); }),
            scalar_t{0});
    }
};

template <typename _R>
linear_expressions_sum(_R &&)
    -> linear_expressions_sum<ranges::views::all_t<_R>>;

template <typename _R, typename _F>
linear_expressions_sum(_R &&, _F &&)
    -> linear_expressions_sum<std::decay_t<decltype(ranges::views::transform(
        std::declval<_R &&>(), std::declval<_F &&>()))>>;

////////////////////////////////// OPERATORS //////////////////////////////////

namespace operators {

template <linear_expression E1, linear_expression E2>
    requires std::same_as<linear_expression_variable_t<E1>,
                          linear_expression_variable_t<E2>> &&
             std::same_as<linear_expression_scalar_t<E1>,
                          linear_expression_scalar_t<E2>>
[[nodiscard]] constexpr auto operator+(E1 && e1, E2 && e2) {
    return linear_expression_add(std::forward<E1>(e1), std::forward<E2>(e2));
};

template <linear_expression E>
[[nodiscard]] constexpr auto operator-(E && e) {
    return linear_expression_negate(std::forward<E>(e));
};

template <linear_expression E1, linear_expression E2>
    requires std::same_as<linear_expression_variable_t<E1>,
                          linear_expression_variable_t<E2>> &&
             std::same_as<linear_expression_scalar_t<E1>,
                          linear_expression_scalar_t<E2>>
[[nodiscard]] constexpr auto operator-(E1 && e1, E2 && e2) {
    return linear_expression_add(
        std::forward<E1>(e1), linear_expression_negate(std::forward<E2>(e2)));
};

template <linear_expression E>
[[nodiscard]] constexpr auto operator+(E && e,
                                       linear_expression_scalar_t<E> c) {
    return linear_expression_scalar_add(std::forward<E>(e), c);
};

template <linear_expression E>
[[nodiscard]] constexpr auto operator+(linear_expression_scalar_t<E> c,
                                       E && e) {
    return linear_expression_scalar_add(std::forward<E>(e), c);
};

template <linear_expression E>
[[nodiscard]] constexpr auto operator-(E && e,
                                       linear_expression_scalar_t<E> c) {
    return linear_expression_scalar_add(std::forward<E>(e), -c);
};

template <linear_expression E>
[[nodiscard]] constexpr auto operator-(linear_expression_scalar_t<E> c,
                                       E && e) {
    return linear_expression_scalar_add(
        linear_expression_negate(std::forward<E>(e)), c);
};

template <linear_expression E>
[[nodiscard]] constexpr auto operator*(E && e,
                                       linear_expression_scalar_t<E> c) {
    return linear_expression_scalar_mul(std::forward<E>(e), c);
};

template <linear_expression E>
[[nodiscard]] constexpr auto operator*(linear_expression_scalar_t<E> c,
                                       E && e) {
    return linear_expression_scalar_mul(std::forward<E>(e), c);
};

template <linear_expression E>
[[nodiscard]] constexpr auto operator/(E && e,
                                       linear_expression_scalar_t<E> c) {
    return linear_expression_scalar_mul(std::forward<E>(e),
                                        linear_expression_scalar_t<E>{1} / c);
};

template <typename E>
using xsum = linear_expressions_sum<E>;

}  // namespace operators

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_LINEAR_EXPRESSION_HPP