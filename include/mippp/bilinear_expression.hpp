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
concept bilinear_term = (std::tuple_size_v<_Tp> == 3);

template <typename _Tp>
using bilinear_term_left_variable_t =
    std::decay_t<std::tuple_element_t<0, _Tp>>;

template <typename _Tp>
using bilinear_term_right_variable_t =
    std::decay_t<std::tuple_element_t<1, _Tp>>;

template <typename _Tp>
using bilinear_term_scalar_t = std::decay_t<std::tuple_element_t<2, _Tp>>;

template <typename _Tp>
using bilinear_terms_range_t = decltype(std::declval<_Tp &>().bilinear_terms());

template <typename _Tp>
using left_linear_terms_range_t =
    decltype(std::declval<_Tp &>().left_linear_terms());

template <typename _Tp>
using right_linear_terms_range_t =
    decltype(std::declval<_Tp &>().right_linear_terms());

template <typename _Tp>
using bilinear_term_t = std::ranges::range_value_t<bilinear_terms_range_t<_Tp>>;

template <typename _Tp>
using left_linear_term_t =
    std::ranges::range_value_t<left_linear_terms_range_t<_Tp>>;

template <typename _Tp>
using right_linear_term_t =
    std::ranges::range_value_t<right_linear_terms_range_t<_Tp>>;

template <typename _Tp>
using bilinear_expression_scalar_t =
    bilinear_term_scalar_t<bilinear_term_t<_Tp>>;

template <typename _Tp>
using bilinear_expression_left_variable_t =
    bilinear_term_left_variable_t<bilinear_term_t<_Tp>>;

template <typename _Tp>
using bilinear_expression_right_variable_t =
    bilinear_term_right_variable_t<bilinear_term_t<_Tp>>;

template <typename _Tp>
concept bilinear_expression =
    requires(const _Tp & __t) {
        { __t.bilinear_terms() } -> ranges::range;
        { __t.left_linear_terms() } -> ranges::range;
        { __t.right_linear_terms() } -> ranges::range;
        {
            __t.constant()
        } -> std::convertible_to<bilinear_expression_scalar_t<_Tp>>;
    } && bilinear_term<bilinear_term_t<_Tp>> &&
    linear_term<left_linear_term_t<_Tp>> &&
    linear_term<right_linear_term_t<_Tp>> &&
    std::same_as<linear_term_variable_t<left_linear_term_t<_Tp>>,
                 bilinear_term_left_variable_t<bilinear_term_t<_Tp>>> &&
    std::same_as<linear_term_variable_t<right_linear_term_t<_Tp>>,
                 bilinear_term_right_variable_t<bilinear_term_t<_Tp>>>;

//////////////////////////////////// CLASS ////////////////////////////////////

template <typename _BiTerms, typename _LTerms, typename _RTerms>
class bilinear_expression_view {
private:
    using scalar_t =
        bilinear_term_scalar_t<std::ranges::range_value_t<_BiTerms>>;
    _BiTerms _bilinear_terms;
    _LTerms _left_linear_terms;
    _RTerms _right_linear_terms;
    scalar_t _constant;

public:
    template <ranges::range BT, ranges::range LT, ranges::range RT, typename S>
    [[nodiscard]] constexpr bilinear_expression_view(BT && bilinear_terms,
                                                     LT && left_linear_terms,
                                                     RT && right_linear_terms,
                                                     S constant)
        : _bilinear_terms(ranges::views::all(std::forward<BT>(bilinear_terms)))
        , _left_linear_terms(
              ranges::views::all(std::forward<LT>(left_linear_terms)))
        , _right_linear_terms(
              ranges::views::all(std::forward<RT>(right_linear_terms)))
        , _constant(static_cast<scalar_t>(constant)) {}

    [[nodiscard]] constexpr decltype(auto) bilinear_terms() const & noexcept {
        return _bilinear_terms;
    }
    [[nodiscard]] constexpr _BiTerms && bilinear_terms() && noexcept {
        return std::move(_bilinear_terms);
    }
    [[nodiscard]] constexpr decltype(auto) left_linear_terms()
        const & noexcept {
        return _left_linear_terms;
    }
    [[nodiscard]] constexpr _LTerms && left_linear_terms() && noexcept {
        return std::move(_left_linear_terms);
    }
    [[nodiscard]] constexpr decltype(auto) right_linear_terms()
        const & noexcept {
        return _right_linear_terms;
    }
    [[nodiscard]] constexpr _RTerms && right_linear_terms() && noexcept {
        return std::move(_right_linear_terms);
    }
    [[nodiscard]] constexpr decltype(auto) constant() const & noexcept {
        return _constant;
    }
    [[nodiscard]] constexpr scalar_t && constant() && noexcept {
        return std::move(_constant);
    }
};

template <typename BT, typename LT, typename RT, typename S>
bilinear_expression_view(BT &&, LT &&, RT &&, S)
    -> bilinear_expression_view<ranges::views::all_t<BT>,
                                ranges::views::all_t<LT>,
                                ranges::views::all_t<RT>>;

///////////////////////////////// OPERATIONS //////////////////////////////////

template <linear_expression E1, linear_expression E2>
constexpr auto linear_expression_mult(E1 && e1, E2 && e2) {
    return bilinear_expression_view(
        ranges::views::transform(ranges::views::cartesian_product(
                                     std::forward<E1>(e1).linear_terms(),
                                     std::forward<E2>(e2).linear_terms()),
                                 [](auto && p) {
                                     auto && [t1, t2] = p;
                                     auto && [v1, c1] = t1;
                                     auto && [v2, c2] = t2;
                                     return std::make_tuple(v1, v2, c1 * c2);
                                 }),
        ranges::views::transform(std::forward<E1>(e1).linear_terms(),
                                 [c2 = e2.constant()](auto && t1) {
                                     return std::make_pair(
                                         std::get<0>(t1), c2 * std::get<1>(t1));
                                 }),
        ranges::views::transform(std::forward<E2>(e2).linear_terms(),
                                 [c1 = e1.constant()](auto && t2) {
                                     return std::make_pair(
                                         std::get<0>(t2), c1 * std::get<1>(t2));
                                 }),
        std::forward<E1>(e1).constant() * std::forward<E2>(e2).constant());
}

template <bilinear_expression E1, bilinear_expression E2>
constexpr auto bilinear_expression_add(E1 && e1, E2 && e2) {
    return bilinear_expression_view(
        ranges::views::concat(std::forward<E1>(e1).bilinear_terms(),
                              std::forward<E2>(e2).bilinear_terms()),
        ranges::views::concat(std::forward<E1>(e1).left_linear_terms(),
                              std::forward<E2>(e2).left_linear_terms()),
        ranges::views::concat(std::forward<E1>(e1).right_linear_terms(),
                              std::forward<E2>(e2).right_linear_terms()),
        std::forward<E1>(e1).constant() + std::forward<E2>(e2).constant());
}

template <bilinear_expression E>
constexpr auto bilinear_expression_negate(E && e) {
    return bilinear_expression_view(
        ranges::views::transform(std::forward<E>(e).bilinear_terms(),
                                 [](auto && t) {
                                     return std::make_tuple(std::get<0>(t),
                                                            std::get<1>(t),
                                                            -std::get<2>(t));
                                 }),
        ranges::views::transform(std::forward<E>(e).left_linear_terms(),
                                 [](auto && t) {
                                     return std::make_pair(std::get<0>(t),
                                                           -std::get<1>(t));
                                 }),
        ranges::views::transform(std::forward<E>(e).right_linear_terms(),
                                 [](auto && t) {
                                     return std::make_pair(std::get<0>(t),
                                                           -std::get<1>(t));
                                 }),
        -std::forward<E>(e).constant());
}

template <bilinear_expression E, typename S>
constexpr auto bilinear_expression_scalar_add(E && e, const S c) {
    using scalar_t = bilinear_expression_scalar_t<E>;
    return bilinear_expression_view(
        std::forward<E>(e).bilinear_terms(),
        std::forward<E>(e).left_linear_terms(),
        std::forward<E>(e).right_linear_terms(),
        std::forward<E>(e).constant() + static_cast<scalar_t>(c));
}

template <bilinear_expression E, typename S>
constexpr auto bilinear_expression_scalar_mul(E && e, const S c_) {
    using scalar_t = bilinear_expression_scalar_t<E>;
    scalar_t c = static_cast<scalar_t>(c_);
    return bilinear_expression_view(
        ranges::views::transform(std::forward<E>(e).bilinear_terms(),
                                 [c](auto && t) {
                                     return std::make_tuple(std::get<0>(t),
                                                            std::get<1>(t),
                                                            c * std::get<2>(t));
                                 }),
        ranges::views::transform(std::forward<E>(e).left_linear_terms(),
                                 [c](auto && t) {
                                     return std::make_pair(std::get<0>(t),
                                                           c * std::get<1>(t));
                                 }),
        ranges::views::transform(std::forward<E>(e).right_linear_terms(),
                                 [c](auto && t) {
                                     return std::make_pair(std::get<0>(t),
                                                           c * std::get<1>(t));
                                 }),
        c * std::forward<E>(e).constant());
}

template <bilinear_expression E1, linear_expression E2>
    requires(std::same_as<bilinear_term_left_variable_t<bilinear_term_t<E1>>,
                          linear_term_variable_t<left_linear_term_t<E2>>> &&
             !std::same_as<bilinear_term_right_variable_t<bilinear_term_t<E1>>,
                           linear_term_variable_t<left_linear_term_t<E2>>>)
constexpr auto bilinear_expression_lexpr_add(E1 && e1, E2 && e2) {
    return bilinear_expression_view(
        std::forward<E1>(e1).bilinear_terms(),
        ranges::views::concat(std::forward<E1>(e1).left_linear_terms(),
                              std::forward<E2>(e2).linear_terms()),
        std::forward<E1>(e1).right_linear_terms(),
        std::forward<E1>(e1).constant() + std::forward<E2>(e2).constant());
}

template <bilinear_expression E1, linear_expression E2>
    requires(!std::same_as<bilinear_term_left_variable_t<bilinear_term_t<E1>>,
                           linear_term_variable_t<left_linear_term_t<E2>>> &&
             std::same_as<bilinear_term_right_variable_t<bilinear_term_t<E1>>,
                          linear_term_variable_t<left_linear_term_t<E2>>>)
constexpr auto bilinear_expression_lexpr_add(E1 && e1, E2 && e2) {
    return bilinear_expression_view(
        std::forward<E1>(e1).bilinear_terms(),
        std::forward<E1>(e1).left_linear_terms(),
        ranges::views::concat(std::forward<E1>(e1).right_linear_terms(),
                              std::forward<E2>(e2).linear_terms()),
        std::forward<E1>(e1).constant() + std::forward<E2>(e2).constant());
}

////////////////////////////////// OPERATORS //////////////////////////////////

namespace operators {

template <linear_expression E1, linear_expression E2>
[[nodiscard]] constexpr auto operator*(E1 && e1, E2 && e2) {
    return linear_expression_mult(std::forward<E1>(e1), std::forward<E2>(e2));
};

template <bilinear_expression E1, bilinear_expression E2>
[[nodiscard]] constexpr auto operator+(E1 && e1, E2 && e2) {
    return bilinear_expression_add(std::forward<E1>(e1), std::forward<E2>(e2));
};
template <bilinear_expression E1, bilinear_expression E2>
[[nodiscard]] constexpr auto operator-(E1 && e1, E2 && e2) {
    return bilinear_expression_add(
        std::forward<E1>(e1), bilinear_expression_negate(std::forward<E2>(e2)));
};

template <bilinear_expression E>
[[nodiscard]] constexpr auto operator-(E && e) {
    return bilinear_expression_negate(std::forward<E>(e));
};

template <bilinear_expression E>
[[nodiscard]] constexpr auto operator+(E && e,
                                       bilinear_expression_scalar_t<E> c) {
    return bilinear_expression_scalar_add(std::forward<E>(e), c);
};
template <bilinear_expression E>
[[nodiscard]] constexpr auto operator+(bilinear_expression_scalar_t<E> c,
                                       E && e) {
    return bilinear_expression_scalar_add(std::forward<E>(e), c);
};
template <bilinear_expression E>
[[nodiscard]] constexpr auto operator-(E && e,
                                       bilinear_expression_scalar_t<E> c) {
    return bilinear_expression_scalar_add(std::forward<E>(e), -c);
};
template <bilinear_expression E>
[[nodiscard]] constexpr auto operator-(bilinear_expression_scalar_t<E> c,
                                       E && e) {
    return bilinear_expression_scalar_add(
        bilinear_expression_negate(std::forward<E>(e)), c);
};

template <bilinear_expression E>
[[nodiscard]] constexpr auto operator*(E && e,
                                       bilinear_expression_scalar_t<E> c) {
    return bilinear_expression_scalar_mul(std::forward<E>(e), c);
};
template <bilinear_expression E>
[[nodiscard]] constexpr auto operator*(bilinear_expression_scalar_t<E> c,
                                       E && e) {
    return bilinear_expression_scalar_mul(std::forward<E>(e), c);
};
template <bilinear_expression E>
[[nodiscard]] constexpr auto operator/(E && e,
                                       bilinear_expression_scalar_t<E> c) {
    return bilinear_expression_scalar_mul(
        std::forward<E>(e), bilinear_expression_scalar_t<E>{1} / c);
};

}  // namespace operators

// (w[u] + xsum(options(), [](auto i) { return w(u, i) * Y(i); })) * Pi(u) +
//     M(t, a) * Y(i) * C(a, i)

}  // namespace fhamonic::mippp

#endif  // MIPPP_BIPARTITE_BILINEAR_EXPRESSION_HPP