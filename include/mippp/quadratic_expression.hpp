#ifndef MIPPP_BIPARTITE_BILINEAR_EXPRESSION_HPP
#define MIPPP_BIPARTITE_BILINEAR_EXPRESSION_HPP

#include <concepts>
// #include <ranges>
#include <tuple>
#include <type_traits>

#include <range/v3/numeric/accumulate.hpp>
#include <range/v3/view/all.hpp>
#include <range/v3/view/cartesian_product.hpp>
#include <range/v3/view/concat.hpp>
#include <range/v3/view/transform.hpp>

#include "mippp/linear_expression.hpp"

namespace fhamonic::mippp {

/////////////////////////////////// CONCEPT ///////////////////////////////////

template <typename _Tp>
concept quadratic_term = (std::tuple_size_v<_Tp> == 3);

template <typename _Tp>
using quadratic_term_variable_t = std::decay_t<std::tuple_element_t<0, _Tp>>;

template <typename _Tp>
using quadratic_term_scalar_t = std::decay_t<std::tuple_element_t<2, _Tp>>;

template <typename _Tp>
using quadratic_terms_range_t =
    decltype(std::declval<_Tp &>().quadratic_terms());

template <typename _Tp>
using quadratic_term_t =
    std::ranges::range_value_t<quadratic_terms_range_t<_Tp>>;

template <typename _Tp>
using quadratic_expression_scalar_t =
    quadratic_term_scalar_t<quadratic_term_t<_Tp>>;

template <typename _Tp>
using quadratic_expression_variable_t =
    quadratic_term_variable_t<quadratic_term_t<_Tp>>;

template <typename _Tp>
using quadratic_exprresion_linear_part_t =
    decltype(std::declval<_Tp &>().linear_expression());

template <typename _Tp>
using quadratic_expression_linear_term_t =
    linear_term_t<quadratic_exprresion_linear_part_t<_Tp>>;

// template <typename _Tp>
// using right_linear_term_t =
//     std::ranges::range_value_t<right_linear_terms_range_t<_Tp>>;

template <typename _Tp>
concept quadratic_expression = requires(const _Tp & __t) {
    { __t.quadratic_terms() } -> ranges::range;
    { __t.linear_expression() } -> linear_expression;
} && quadratic_term<quadratic_term_t<_Tp>>;

//////////////////////////////////// CLASS ////////////////////////////////////

template <typename _QTerms, typename _LExpr>
class quadratic_expression_view {
private:
    using scalar_t =
        quadratic_term_scalar_t<std::ranges::range_value_t<_QTerms>>;
    _QTerms _quadratic_terms;
    _LExpr _linear_expression;

public:
    template <ranges::range QT, linear_expression LE>
    [[nodiscard]] constexpr quadratic_expression_view(QT && quadratic_terms,
                                                      LE && linear_expression)
        : _quadratic_terms(
              ranges::views::all(std::forward<QT>(quadratic_terms)))
        , _linear_expression(
              ranges::views::all(std::forward<LE>(linear_expression))) {}

    [[nodiscard]] constexpr decltype(auto) quadratic_terms() const & noexcept {
        return _quadratic_terms;
    }
    [[nodiscard]] constexpr _QTerms && quadratic_terms() && noexcept {
        return std::move(_quadratic_terms);
    }
    [[nodiscard]] constexpr decltype(auto) linear_expression()
        const & noexcept {
        return _linear_expression;
    }
    [[nodiscard]] constexpr _LExpr && linear_expression() && noexcept {
        return std::move(_linear_expression);
    }
};

template <typename QT, typename LE>
quadratic_expression_view(QT &&, LE &&)
    -> quadratic_expression_view<ranges::views::all_t<QT>, LE>;

///////////////////////////////// OPERATIONS //////////////////////////////////

template <linear_expression E1, linear_expression E2>
constexpr auto linear_expression_mul(E1 && e1, E2 && e2) {
    auto c1 = e1.constant();
    auto c2 = e2.constant();
    return quadratic_expression_view(
        ranges::views::transform(ranges::views::cartesian_product(
                                     std::forward<E1>(e1).linear_terms(),
                                     std::forward<E2>(e2).linear_terms()),
                                 [](auto && p) {
                                     auto && [t1, t2] = p;
                                     auto && [v1, c1] = t1;
                                     auto && [v2, c2] = t2;
                                     return std::make_tuple(v1, v2, c1 * c2);
                                 }),
        linear_expression_add(
            linear_expression_scalar_mul(std::forward<E2>(e2), c1),
            linear_expression_scalar_mul(std::forward<E1>(e1), c2)));
}

template <quadratic_expression E1, quadratic_expression E2>
constexpr auto quadratic_expression_add(E1 && e1, E2 && e2) {
    return quadratic_expression_view(
        ranges::views::concat(std::forward<E1>(e1).quadratic_terms(),
                              std::forward<E2>(e2).quadratic_terms()),
        linear_expression_add(std::forward<E2>(e2), std::forward<E1>(e1)));
}

template <quadratic_expression E>
constexpr auto quadratic_expression_negate(E && e) {
    return quadratic_expression_view(
        ranges::views::transform(std::forward<E>(e).quadratic_terms(),
                                 [](auto && t) {
                                     return std::make_tuple(std::get<0>(t),
                                                            std::get<1>(t),
                                                            -std::get<2>(t));
                                 }),
        linear_expression_negate(std::forward<E>(e).linear_expression()));
}

template <quadratic_expression E, typename S>
constexpr auto quadratic_expression_scalar_add(E && e, const S c) {
    using scalar_t = quadratic_expression_scalar_t<E>;
    return quadratic_expression_view(
        std::forward<E>(e).quadratic_terms(),
        linear_expression_scalar_add(std::forward<E>(e).left_linear_terms(),
                                     c));
}

template <quadratic_expression E, typename S>
constexpr auto quadratic_expression_scalar_mul(E && e, const S c_) {
    using scalar_t = quadratic_expression_scalar_t<E>;
    scalar_t c = static_cast<scalar_t>(c_);
    return quadratic_expression_view(
        ranges::views::transform(std::forward<E>(e).quadratic_terms(),
                                 [c](auto && t) {
                                     return std::make_tuple(std::get<0>(t),
                                                            std::get<1>(t),
                                                            c * std::get<2>(t));
                                 }),
        linear_expression_scalar_mul(std::forward<E>(e), c));
}

template <quadratic_expression E1, linear_expression E2>
    requires std::same_as<quadratic_term_variable_t<quadratic_term_t<E1>>,
                          linear_term_variable_t<linear_term_t<E2>>>
constexpr auto quadratic_expression_lexpr_add(E1 && e1, E2 && e2) {
    return quadratic_expression_view(
        std::forward<E1>(e1).quadratic_terms(),
        linear_expression_add(std::forward<E1>(e1), std::forward<E2>(e2)));
}

////////////////////////////////// OPERATORS //////////////////////////////////

namespace operators {

template <linear_expression E1, linear_expression E2>
[[nodiscard]] constexpr auto operator*(E1 && e1, E2 && e2) {
    return linear_expression_mul(std::forward<E1>(e1), std::forward<E2>(e2));
};

template <quadratic_expression E1, quadratic_expression E2>
[[nodiscard]] constexpr auto operator+(E1 && e1, E2 && e2) {
    return quadratic_expression_add(std::forward<E1>(e1), std::forward<E2>(e2));
};
template <quadratic_expression E1, quadratic_expression E2>
[[nodiscard]] constexpr auto operator-(E1 && e1, E2 && e2) {
    return quadratic_expression_add(
        std::forward<E1>(e1),
        quadratic_expression_negate(std::forward<E2>(e2)));
};

template <quadratic_expression E>
[[nodiscard]] constexpr auto operator-(E && e) {
    return quadratic_expression_negate(std::forward<E>(e));
};

template <quadratic_expression E>
[[nodiscard]] constexpr auto operator+(E && e,
                                       quadratic_expression_scalar_t<E> c) {
    return quadratic_expression_scalar_add(std::forward<E>(e), c);
};
template <quadratic_expression E>
[[nodiscard]] constexpr auto operator+(quadratic_expression_scalar_t<E> c,
                                       E && e) {
    return quadratic_expression_scalar_add(std::forward<E>(e), c);
};
template <quadratic_expression E>
[[nodiscard]] constexpr auto operator-(E && e,
                                       quadratic_expression_scalar_t<E> c) {
    return quadratic_expression_scalar_add(std::forward<E>(e), -c);
};
template <quadratic_expression E>
[[nodiscard]] constexpr auto operator-(quadratic_expression_scalar_t<E> c,
                                       E && e) {
    return quadratic_expression_scalar_add(
        quadratic_expression_negate(std::forward<E>(e)), c);
};

template <quadratic_expression E>
[[nodiscard]] constexpr auto operator*(E && e,
                                       quadratic_expression_scalar_t<E> c) {
    return quadratic_expression_scalar_mul(std::forward<E>(e), c);
};
template <quadratic_expression E>
[[nodiscard]] constexpr auto operator*(quadratic_expression_scalar_t<E> c,
                                       E && e) {
    return quadratic_expression_scalar_mul(std::forward<E>(e), c);
};
template <quadratic_expression E>
[[nodiscard]] constexpr auto operator/(E && e,
                                       quadratic_expression_scalar_t<E> c) {
    return quadratic_expression_scalar_mul(
        std::forward<E>(e), quadratic_expression_scalar_t<E>{1} / c);
};

}  // namespace operators

// (w[u] + xsum(options(), [](auto i) { return w(u, i) * Y(i); })) * Pi(u) +
//     M(t, a) * Y(i) * C(a, i)

}  // namespace fhamonic::mippp

#endif  // MIPPP_BIPARTITE_BILINEAR_EXPRESSION_HPP