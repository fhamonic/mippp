#pragma once

#include <concepts>
#include <ranges>
#include <tuple>
#include <type_traits>
#include <utility>

#include "mippp/linear_expression.hpp"
#include "mippp/utility/zero.hpp"

namespace mippp {

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Concepts ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename T>
concept quadratic_term = requires { typename std::tuple_size<T>::type; } &&
                         (std::tuple_size_v<T> == 3) &&
                         std::same_as<std::decay_t<std::tuple_element_t<0, T>>,
                                      std::decay_t<std::tuple_element_t<1, T>>>;

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
using quadratic_expression_linear_part_t =
    decltype(std::declval<QE &>().linear_part());

template <typename QE>
using quadratic_expression_linear_term_t =
    linear_term_t<quadratic_expression_linear_part_t<QE>>;

template <typename QE>
using quadratic_expression_variable_t =
    quadratic_term_variable_t<quadratic_term_t<QE>>;

template <typename QE>
using quadratic_expression_scalar_t =
    quadratic_term_scalar_t<quadratic_term_t<QE>>;

template <typename QE>
using quadratic_expression_constant_t =
    linear_expression_constant_t<quadratic_expression_linear_part_t<QE>>;

// Requirement on types modeling `quadratic_expression`:
// the `&&` accessors must partition the object's state — if one may move from a
// subobject, no other accessor may read it. MIP++ forwards the same expression
// to both `quadratic_terms()` and `linear_part()` in one call, which is
// well-defined only because those consume disjoint state. Providing only `const
// &` accessors (see `linear_expression_square`) satisfies this trivially.

template <typename QE>
concept quadratic_expression =
    quadratic_term<quadratic_term_t<QE>> &&
    linear_expression<quadratic_expression_linear_part_t<QE>> &&
    std::same_as<
        linear_expression_variable_t<quadratic_expression_linear_part_t<QE>>,
        quadratic_expression_variable_t<QE>> &&
    std::convertible_to<
        linear_expression_scalar_t<quadratic_expression_linear_part_t<QE>>,
        quadratic_expression_scalar_t<QE>>;

template <typename E1, typename E2>
concept compatible_quadratic_expressions =
    quadratic_expression<E1> && quadratic_expression<E2> &&
    std::same_as<quadratic_expression_variable_t<E1>,
                 quadratic_expression_variable_t<E2>> &&
    std::same_as<quadratic_expression_scalar_t<E1>,
                 quadratic_expression_scalar_t<E2>>;

// Reading a quadratic expression does not consume it: quadratic terms and
// linear part both accessors are are reachable through a `const &`, which by
// construction cannot move out of it. An expression that owns a move-only term
// range fails this.
template <typename E>
concept const_readable_quadratic_expression =
    requires(const std::remove_cvref_t<E> & e) {
        e.quadratic_terms();
        e.linear_part();
    };

namespace detail {
template <typename E1, typename E2>
consteval void assert_compatible_quadratic_expressions() {
    static_assert(std::same_as<quadratic_expression_variable_t<E1>,
                               quadratic_expression_variable_t<E2>>,
                  "MIP++: these expressions use different variable types.");
    static_assert(std::same_as<quadratic_expression_scalar_t<E1>,
                               quadratic_expression_scalar_t<E2>>,
                  "MIP++: these expressions use different scalar types; "
                  "cast one side explicitly.");
}

template <typename QE, typename LE>
consteval void assert_compatible_qexpr_lexpr() {
    static_assert(std::same_as<quadratic_expression_variable_t<QE>,
                               linear_expression_variable_t<LE>>,
                  "MIP++: these expressions use different variable types.");
    static_assert(std::same_as<quadratic_expression_scalar_t<QE>,
                               linear_expression_scalar_t<LE>>,
                  "MIP++: these expressions use different scalar types; "
                  "cast one side explicitly.");
}

// Mirror of forwardable_linear_expression: an operand may be handed to a
// consuming operation either because it is an rvalue we are allowed to gut, or
// because reading it doesn't consume it. Value-category dependent -- it
// constrains the argument, not the expression type.
template <typename E>
concept forwardable_quadratic_expression =
    (!std::is_lvalue_reference_v<E> &&
     !std::is_const_v<std::remove_reference_t<E>>) ||
    const_readable_quadratic_expression<E>;

template <typename... Es>
consteval void assert_forwardable_quadratic_expressions() {
    static_assert(
        (forwardable_quadratic_expression<Es> && ...),
        "MIP++: this quadratic expression owns one of its term ranges (the "
        "quadratic terms or the linear part were built from an rvalue range) "
        "and is single-use. Either pass it as an rvalue (std::move) if this is "
        "its last use, or build it over named ranges so it only references the "
        "terms.");
}

// Products need `multipass_linear_terms` on both operands: the quadratic views
// expose only `const &` accessors (see the note above), and a cartesian product
// walks one operand once per term of the other.
template <typename... Es>
consteval void assert_multipliable_linear_expressions() {
    static_assert(
        (const_readable_linear_terms<Es> && ...),
        "MIP++: this expression owns a move-only term range, so a product "
        "cannot read it (products read their operands through `const &`). "
        "Materialize it into a runtime_linear_expression first.");
    static_assert(
        (multipass_linear_terms<Es> && ...),
        "MIP++: this expression's term range is single-pass, but a product "
        "traverses each operand once per term of the other and so needs a "
        "forward_range. This usually means the expression comes from xsum(), "
        "whose terms are a join of temporary ranges. Materialize it first: "
        "square(materialize(e)), or materialize(e1) * materialize(e2).");
}
}  // namespace detail

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// Views ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <std::ranges::view QTerms, linear_expression LExpr>
    requires quadratic_term<std::ranges::range_value_t<QTerms>>
class quadratic_expression_view {
private:
    QTerms _quadratic_terms;
    LExpr _linear_expression;

public:
    template <std::ranges::viewable_range QT, typename LE>
        requires std::constructible_from<QTerms, std::views::all_t<QT &&>> &&
                     std::constructible_from<LExpr, LE &&>
    constexpr quadratic_expression_view(QT && quadratic_terms,
                                        LE && linear_expression)
        : _quadratic_terms(std::views::all(std::forward<QT>(quadratic_terms)))
        , _linear_expression(std::forward<LE>(linear_expression)) {}

    [[nodiscard]] constexpr QTerms quadratic_terms() const & noexcept(
        std::is_nothrow_copy_constructible_v<QTerms>)
        requires std::copy_constructible<QTerms>
    {
        return _quadratic_terms;
    }
    // lvalue access must not copy: QTerms may be move-only
    [[nodiscard]] constexpr QTerms & quadratic_terms() & noexcept {
        return _quadratic_terms;
    }
    [[nodiscard]] constexpr QTerms && quadratic_terms() && noexcept {
        return std::move(_quadratic_terms);
    }

    [[nodiscard]] constexpr LExpr linear_part() const & noexcept(
        std::is_nothrow_copy_constructible_v<LExpr>)
        requires std::copy_constructible<LExpr>
    {
        return _linear_expression;
    }
    // lvalue access must not copy: Terms may be move-only
    [[nodiscard]] constexpr LExpr & linear_part() & noexcept {
        return _linear_expression;
    }
    [[nodiscard]] constexpr LExpr && linear_part() && noexcept {
        return std::move(_linear_expression);
    }
};

template <std::ranges::viewable_range QT, typename LE>
quadratic_expression_view(QT &&, LE &&)
    -> quadratic_expression_view<std::views::all_t<QT>, std::decay_t<LE>>;

template <linear_expression LExpr>
    requires multipass_linear_terms<LExpr>
class linear_expression_square {
private:
    LExpr _linear_expression;

public:
    template <typename E>
        requires std::constructible_from<LExpr, E &&>
    constexpr explicit linear_expression_square(E && linear_expression)
        : _linear_expression(std::forward<E>(linear_expression)) {}

    [[nodiscard]] constexpr auto quadratic_terms() const & noexcept {
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
    [[nodiscard]] constexpr auto linear_part() const & noexcept {
        if constexpr(statically_zero<linear_expression_constant_t<LExpr>>) {
            return empty_linear_expression<linear_expression_variable_t<LExpr>,
                                           linear_expression_scalar_t<LExpr>>;
        } else {
            const auto c = _linear_expression.constant();
            return linear_expression_view(
                std::views::transform(_linear_expression.linear_terms(),
                                      [c](auto && t) {
                                          return std::make_pair(
                                              std::get<0>(t),
                                              2 * c * std::get<1>(t));
                                      }),
                c * c);
        }
    }
};

// `expression_all_t` mirrors the `views::all_t` guides of the range adaptors:
// an rvalue operand is moved into the view, a named operand is referenced.
template <typename LE>
linear_expression_square(LE &&)
    -> linear_expression_square<detail::expression_all_t<LE>>;

template <linear_expression LExpr1, linear_expression LExpr2>
    requires compatible_linear_expressions<LExpr1, LExpr2> &&
             multipass_linear_terms<LExpr1> && multipass_linear_terms<LExpr2>
class linear_expression_mul_view {
private:
    LExpr1 _linear_expression_1;
    LExpr2 _linear_expression_2;

public:
    template <typename E1, typename E2>
        requires std::constructible_from<LExpr1, E1 &&> &&
                     std::constructible_from<LExpr2, E2 &&>
    constexpr linear_expression_mul_view(E1 && linear_expression_1,
                                         E2 && linear_expression_2)
        : _linear_expression_1(std::forward<E1>(linear_expression_1))
        , _linear_expression_2(std::forward<E2>(linear_expression_2)) {}

    [[nodiscard]] constexpr auto quadratic_terms() const & noexcept {
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
    [[nodiscard]] constexpr auto linear_part() const & noexcept {
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
    -> linear_expression_mul_view<detail::expression_all_t<E1>,
                                  detail::expression_all_t<E2>>;

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////// Operations //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <quadratic_expression E1, quadratic_expression E2>
constexpr auto quadratic_expression_add(E1 && e1, E2 && e2) {
    detail::assert_compatible_quadratic_expressions<E1, E2>();
    detail::assert_forwardable_quadratic_expressions<E1, E2>();
    return quadratic_expression_view(
        detail::unordered_concat(std::forward<E1>(e1).quadratic_terms(),
                                 std::forward<E2>(e2).quadratic_terms()),
        linear_expression_add(std::forward<E1>(e1).linear_part(),
                              std::forward<E2>(e2).linear_part()));
}

template <quadratic_expression E>
constexpr auto quadratic_expression_negate(E && e) {
    detail::assert_forwardable_quadratic_expressions<E>();
    return quadratic_expression_view(
        std::views::transform(std::forward<E>(e).quadratic_terms(),
                              [](auto && t) {
                                  return std::make_tuple(std::get<0>(t),
                                                         std::get<1>(t),
                                                         -std::get<2>(t));
                              }),
        linear_expression_negate(std::forward<E>(e).linear_part()));
}

template <quadratic_expression E, typename S>
constexpr auto quadratic_expression_scalar_add(E && e, const S c) {
    detail::assert_forwardable_quadratic_expressions<E>();
    return quadratic_expression_view(
        std::forward<E>(e).quadratic_terms(),
        linear_expression_scalar_add(std::forward<E>(e).linear_part(), c));
}

template <quadratic_expression E, typename S>
constexpr auto quadratic_expression_scalar_mul(E && e, const S c_) {
    detail::assert_forwardable_quadratic_expressions<E>();
    const auto c = static_cast<quadratic_expression_scalar_t<E>>(c_);
    return quadratic_expression_view(
        std::views::transform(std::forward<E>(e).quadratic_terms(),
                              [c](auto && t) {
                                  return std::make_tuple(std::get<0>(t),
                                                         std::get<1>(t),
                                                         c * std::get<2>(t));
                              }),
        linear_expression_scalar_mul(std::forward<E>(e).linear_part(), c));
}

template <quadratic_expression E, typename S>
constexpr auto quadratic_expression_scalar_div(E && e, const S c_) {
    detail::assert_forwardable_quadratic_expressions<E>();
    const auto c = static_cast<quadratic_expression_scalar_t<E>>(c_);
    return quadratic_expression_view(
        std::views::transform(std::forward<E>(e).quadratic_terms(),
                              [c](auto && t) {
                                  return std::make_tuple(std::get<0>(t),
                                                         std::get<1>(t),
                                                         std::get<2>(t) / c);
                              }),
        linear_expression_scalar_div(std::forward<E>(e).linear_part(), c));
}

template <quadratic_expression E1, linear_expression E2>
constexpr auto quadratic_expression_lexpr_add(E1 && e1, E2 && e2) {
    detail::assert_compatible_qexpr_lexpr<E1, E2>();
    detail::assert_forwardable_quadratic_expressions<E1>();
    return quadratic_expression_view(
        std::forward<E1>(e1).quadratic_terms(),
        linear_expression_add(std::forward<E1>(e1).linear_part(),
                              std::forward<E2>(e2)));
}

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Operators //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

namespace operators {

// Products store their operands `views::all`-style (`expression_all_t`): an
// rvalue is moved into the view, a named operand is only referenced -- reading
// it is non-consuming (`multipass` implies const-readable) and referencing
// avoids deep-copying e.g. a named runtime_linear_expression's terms. The
// operand must then outlive the view, as with any range referenced by a view.
template <linear_expression E>
[[nodiscard]] constexpr auto square(E && e) {
    detail::assert_multipliable_linear_expressions<E>();
    return linear_expression_square<detail::expression_all_t<E>>(
        std::forward<E>(e));
}

template <linear_expression E1, linear_expression E2>
[[nodiscard]] constexpr auto operator*(E1 && e1, E2 && e2) {
    detail::assert_compatible_linear_expressions<E1, E2>();
    detail::assert_multipliable_linear_expressions<E1, E2>();
    return linear_expression_mul_view<detail::expression_all_t<E1>,
                                      detail::expression_all_t<E2>>(
        std::forward<E1>(e1), std::forward<E2>(e2));
}

template <quadratic_expression E1, quadratic_expression E2>
[[nodiscard]] constexpr auto operator+(E1 && e1, E2 && e2) {
    detail::assert_compatible_quadratic_expressions<E1, E2>();
    return quadratic_expression_add(std::forward<E1>(e1), std::forward<E2>(e2));
}
template <quadratic_expression E1, quadratic_expression E2>
[[nodiscard]] constexpr auto operator-(E1 && e1, E2 && e2) {
    detail::assert_compatible_quadratic_expressions<E1, E2>();
    return quadratic_expression_add(
        std::forward<E1>(e1),
        quadratic_expression_negate(std::forward<E2>(e2)));
}

template <quadratic_expression E>
[[nodiscard]] constexpr auto operator-(E && e) {
    return quadratic_expression_negate(std::forward<E>(e));
}

template <quadratic_expression E>
[[nodiscard]] constexpr auto operator+(E && e,
                                       quadratic_expression_scalar_t<E> c) {
    return quadratic_expression_scalar_add(std::forward<E>(e), c);
}
template <quadratic_expression E>
[[nodiscard]] constexpr auto operator+(quadratic_expression_scalar_t<E> c,
                                       E && e) {
    return quadratic_expression_scalar_add(std::forward<E>(e), c);
}
template <quadratic_expression E>
[[nodiscard]] constexpr auto operator-(E && e,
                                       quadratic_expression_scalar_t<E> c) {
    return quadratic_expression_scalar_add(std::forward<E>(e), -c);
}
template <quadratic_expression E>
[[nodiscard]] constexpr auto operator-(quadratic_expression_scalar_t<E> c,
                                       E && e) {
    return quadratic_expression_scalar_add(
        quadratic_expression_negate(std::forward<E>(e)), c);
}

template <quadratic_expression E>
[[nodiscard]] constexpr auto operator*(E && e,
                                       quadratic_expression_scalar_t<E> c) {
    return quadratic_expression_scalar_mul(std::forward<E>(e), c);
}
template <quadratic_expression E>
[[nodiscard]] constexpr auto operator*(quadratic_expression_scalar_t<E> c,
                                       E && e) {
    return quadratic_expression_scalar_mul(std::forward<E>(e), c);
}
template <quadratic_expression E>
[[nodiscard]] constexpr auto operator/(E && e,
                                       quadratic_expression_scalar_t<E> c) {
    return quadratic_expression_scalar_div(std::forward<E>(e), c);
}

template <quadratic_expression QE, linear_expression LE>
[[nodiscard]] constexpr auto operator+(QE && qe, LE && le) {
    return quadratic_expression_lexpr_add(std::forward<QE>(qe),
                                          std::forward<LE>(le));
}
template <quadratic_expression QE, linear_expression LE>
[[nodiscard]] constexpr auto operator+(LE && le, QE && qe) {
    return quadratic_expression_lexpr_add(std::forward<QE>(qe),
                                          std::forward<LE>(le));
}
template <quadratic_expression QE, linear_expression LE>
[[nodiscard]] constexpr auto operator-(QE && qe, LE && le) {
    return quadratic_expression_lexpr_add(
        std::forward<QE>(qe), linear_expression_negate(std::forward<LE>(le)));
}
template <quadratic_expression QE, linear_expression LE>
[[nodiscard]] constexpr auto operator-(LE && le, QE && qe) {
    return quadratic_expression_lexpr_add(
        quadratic_expression_negate(std::forward<QE>(qe)),
        std::forward<LE>(le));
}

}  // namespace operators

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Evaluate ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// `QE &&` (not `const QE &`): `quadratic_expression_view::quadratic_terms()
// const &` returns a copy of the view, and doesn't exist at all when QTerms is
// move-only.
template <quadratic_expression QE, typename VM>
constexpr auto evaluate(QE && e, const VM & values_map) {
    static_assert(
        input_mapping<const VM, quadratic_expression_variable_t<QE>>,
        "MIP++: evaluate needs a values map readable by the expression's "
        "variables; adapt raw storage or a callable with views::mapping_all "
        "or an entity_mapping.");
    using scalar = quadratic_expression_scalar_t<QE>;
    scalar acc = static_cast<scalar>(evaluate(e.linear_part(), values_map));
    for(auto && [var1, var2, coef] : e.quadratic_terms())
        acc += values_map[var1] * values_map[var2] * coef;
    return acc;
}

}  // namespace mippp
