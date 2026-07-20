#pragma once

#include <concepts>
#include <ranges>
#include <tuple>
#include <type_traits>

#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/utility/zero.hpp"

namespace mippp {

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Concepts ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename T>
concept quadratic_term = requires { typename std::tuple_size<T>::type; } &&
                         (std::tuple_size_v<T> == 3);

template <typename QT>
using quadratic_term_variable_t = std::decay_t<std::tuple_element_t<0, QT>>;

template <typename QT>
using quadratic_term_scalar_t = std::decay_t<std::tuple_element_t<2, QT>>;

template <typename QE>
using quadratic_terms_range_t =
    decltype(std::declval<QE &>().quadratic_terms());

template <typename QE>
using quadratic_term_t =
    std::ranges::range_value_t<quadratic_terms_range_t<QE>>;

template <typename QE>
using quadratic_expression_scalar_t =
    quadratic_term_scalar_t<quadratic_term_t<QE>>;

template <typename QE>
using quadratic_expression_variable_t =
    quadratic_term_variable_t<quadratic_term_t<QE>>;

template <typename QE>
using quadratic_exprresion_linear_part_t =
    decltype(std::declval<QE &>().linear_expression());

template <typename QE>
using quadratic_expression_linear_term_t =
    linear_term_t<quadratic_exprresion_linear_part_t<QE>>;

template <typename QE>
concept quadratic_expression =
    quadratic_term<quadratic_term_t<QE>> &&
    linear_expression<quadratic_exprresion_linear_part_t<QE>>;

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// Views ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename QTerms, typename LExpr>
class quadratic_expression_view {
private:
    QTerms _quadratic_terms;
    LExpr _linear_expression;

public:
    template <std::ranges::range QT, linear_expression LE>
    [[nodiscard]] constexpr quadratic_expression_view(QT && quadratic_terms,
                                                      LE && linear_expression)
        : _quadratic_terms(std::views::all(std::forward<QT>(quadratic_terms)))
        , _linear_expression(std::forward<LE>(linear_expression)) {}

    [[nodiscard]] constexpr decltype(auto) quadratic_terms() const & noexcept {
        return _quadratic_terms;
    }
    [[nodiscard]] constexpr QTerms && quadratic_terms() && noexcept {
        return std::move(_quadratic_terms);
    }
    [[nodiscard]] constexpr decltype(auto) linear_expression()
        const & noexcept {
        return _linear_expression;
    }
    [[nodiscard]] constexpr LExpr && linear_expression() && noexcept {
        return std::move(_linear_expression);
    }
};

template <typename QT, typename LE>
quadratic_expression_view(QT &&, LE &&)
    -> quadratic_expression_view<std::views::all_t<QT>, std::decay_t<LE>>;

template <typename LExpr>
class linear_expression_square {
private:
    LExpr _linear_expression;

public:
    template <linear_expression E>
    [[nodiscard]] constexpr linear_expression_square(E && linear_expression)
        : _linear_expression(std::forward<E>(linear_expression)) {}

    [[nodiscard]] constexpr decltype(auto) quadratic_terms() const & noexcept {
        return std::views::transform(
            std::views::cartesian_product(_linear_expression.linear_terms(),
                                          _linear_expression.linear_terms()),
            [](auto && p) {
                auto && [t1, t2] = p;
                auto && [v1, c1] = t1;
                auto && [v2, c2] = t2;
                return std::make_tuple(v1, v2, c1 * c2);
            });
    }
    [[nodiscard]] constexpr decltype(auto) linear_expression()
        const & noexcept {
        if constexpr(statically_zero<linear_expression_constant_t<LExpr>>) {
            return empty_linear_expression<linear_expression_variable_t<LExpr>,
                                           linear_expression_scalar_t<LExpr>>;
        } else {
            return linear_expression_view(
                std::views::transform(
                    _linear_expression.linear_terms(),
                    [c = _linear_expression.constant()](auto && t) {
                        return std::make_pair(std::get<0>(t),
                                              2 * c * std::get<1>(t));
                    }),
                _linear_expression.constant() * _linear_expression.constant());
        }
    }
};

template <typename LE>
linear_expression_square(LE &&) -> linear_expression_square<std::decay_t<LE>>;

template <typename LExpr1, typename LExpr2>
class linear_expression_mul_view {
private:
    LExpr1 _linear_expression_1;
    LExpr2 _linear_expression_2;

public:
    template <linear_expression E1, linear_expression E2>
    [[nodiscard]] constexpr linear_expression_mul_view(
        E1 && linear_expression_1, E2 && linear_expression_2)
        : _linear_expression_1(std::forward<E1>(linear_expression_1))
        , _linear_expression_2(std::forward<E2>(linear_expression_2)) {}

    [[nodiscard]] constexpr decltype(auto) quadratic_terms() const & noexcept {
        return std::views::transform(
            std::views::cartesian_product(_linear_expression_1.linear_terms(),
                                          _linear_expression_2.linear_terms()),
            [](auto && p) {
                auto && [t1, t2] = p;
                auto && [v1, c1] = t1;
                auto && [v2, c2] = t2;
                return std::make_tuple(v1, v2, c1 * c2);
            });
    }
    [[nodiscard]] constexpr auto linear_expression() const & noexcept {
        using constant_1 = linear_expression_constant_t<LExpr1>;
        using constant_2 = linear_expression_constant_t<LExpr2>;
        if constexpr(statically_zero<constant_1> &&
                     statically_zero<constant_2>) {
            return empty_linear_expression<linear_expression_variable_t<LExpr1>,
                                           linear_expression_scalar_t<LExpr1>>;
        } else if constexpr(statically_zero<constant_1>) {
            return linear_expression_scalar_mul(
                _linear_expression_1, _linear_expression_2.constant());
        } else if constexpr(statically_zero<constant_2>) {
            return linear_expression_scalar_mul(
                _linear_expression_2, _linear_expression_1.constant());
        } else {
            return linear_expression_view(
                detail::unordered_concat(
                    std::views::transform(
                        _linear_expression_1.linear_terms(),
                        [c = _linear_expression_2.constant()](auto && t) {
                            return std::make_pair(std::get<0>(t),
                                                  c * std::get<1>(t));
                        }),
                    std::views::transform(
                        _linear_expression_2.linear_terms(),
                        [c = _linear_expression_1.constant()](auto && t) {
                            return std::make_pair(std::get<0>(t),
                                                  c * std::get<1>(t));
                        })),
                _linear_expression_1.constant() *
                    _linear_expression_2.constant());
        }
    }
};

template <typename E1, typename E2>
linear_expression_mul_view(E1 &&, E2 &&)
    -> linear_expression_mul_view<std::decay_t<E1>, std::decay_t<E2>>;

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////// Operations //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <quadratic_expression E1, quadratic_expression E2>
constexpr auto quadratic_expression_add(E1 && e1, E2 && e2) {
    auto linear_part =
        linear_expression_add(e1.linear_expression(), e2.linear_expression());
    return quadratic_expression_view(
        detail::unordered_concat(std::forward<E1>(e1).quadratic_terms(),
                                 std::forward<E2>(e2).quadratic_terms()),
        std::move(linear_part));
}

template <quadratic_expression E>
constexpr auto quadratic_expression_negate(E && e) {
    auto linear_part = linear_expression_negate(e.linear_expression());
    return quadratic_expression_view(
        std::views::transform(std::forward<E>(e).quadratic_terms(),
                              [](auto && t) {
                                  return std::make_tuple(std::get<0>(t),
                                                         std::get<1>(t),
                                                         -std::get<2>(t));
                              }),
        std::move(linear_part));
}

template <quadratic_expression E, typename S>
constexpr auto quadratic_expression_scalar_add(E && e, const S c) {
    auto linear_part = linear_expression_scalar_add(e.linear_expression(), c);
    return quadratic_expression_view(std::forward<E>(e).quadratic_terms(),
                                     std::move(linear_part));
}

template <quadratic_expression E, typename S>
constexpr auto quadratic_expression_scalar_mul(E && e, const S c_) {
    const auto c = static_cast<quadratic_expression_scalar_t<E>>(c_);
    auto linear_part = linear_expression_scalar_mul(e.linear_expression(), c);
    return quadratic_expression_view(
        std::views::transform(std::forward<E>(e).quadratic_terms(),
                              [c](auto && t) {
                                  return std::make_tuple(std::get<0>(t),
                                                         std::get<1>(t),
                                                         c * std::get<2>(t));
                              }),
        std::move(linear_part));
}

template <quadratic_expression E, typename S>
constexpr auto quadratic_expression_scalar_div(E && e, const S c_) {
    const auto c = static_cast<quadratic_expression_scalar_t<E>>(c_);
    auto linear_part = linear_expression_scalar_div(e.linear_expression(), c);
    return quadratic_expression_view(
        std::views::transform(std::forward<E>(e).quadratic_terms(),
                              [c](auto && t) {
                                  return std::make_tuple(std::get<0>(t),
                                                         std::get<1>(t),
                                                         std::get<2>(t) / c);
                              }),
        std::move(linear_part));
}

template <quadratic_expression E1, linear_expression E2>
    requires std::same_as<quadratic_term_variable_t<quadratic_term_t<E1>>,
                          linear_term_variable_t<linear_term_t<E2>>>
constexpr auto quadratic_expression_lexpr_add(E1 && e1, E2 && e2) {
    return quadratic_expression_view(
        std::forward<E1>(e1).quadratic_terms(),
        linear_expression_add(std::forward<E1>(e1).linear_expression(),
                              std::forward<E2>(e2)));
}

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Operators //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

namespace operators {

template <linear_expression E>
[[nodiscard]] constexpr auto square(E && e) {
    return linear_expression_square(std::forward<E>(e));
};

template <linear_expression E1, linear_expression E2>
[[nodiscard]] constexpr auto operator*(E1 && e1, E2 && e2) {
    return linear_expression_mul_view(std::forward<E1>(e1),
                                      std::forward<E2>(e2));
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
    return quadratic_expression_scalar_div(std::forward<E>(e), c);
};

template <quadratic_expression QE, linear_expression LE>
[[nodiscard]] constexpr auto operator+(QE && qe, LE && le) {
    return quadratic_expression_lexpr_add(std::forward<QE>(qe),
                                          std::forward<LE>(le));
};
template <quadratic_expression QE, linear_expression LE>
[[nodiscard]] constexpr auto operator+(LE && le, QE && qe) {
    return quadratic_expression_lexpr_add(std::forward<QE>(qe),
                                          std::forward<LE>(le));
};
template <quadratic_expression QE, linear_expression LE>
[[nodiscard]] constexpr auto operator-(QE && qe, LE && le) {
    return quadratic_expression_lexpr_add(
        std::forward<QE>(qe), linear_expression_negate(std::forward<LE>(le)));
};
template <quadratic_expression QE, linear_expression LE>
[[nodiscard]] constexpr auto operator-(LE && le, QE && qe) {
    return quadratic_expression_lexpr_add(
        quadratic_expression_negate(std::forward<QE>(qe)),
        std::forward<LE>(le));
};

}  // namespace operators
}  // namespace mippp
