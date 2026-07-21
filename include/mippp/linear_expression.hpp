#pragma once

#include <concepts>
#include <ranges>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "mippp/detail/concat_view.hpp"
#include "mippp/utility/zero.hpp"

namespace mippp {

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Concepts ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename T>
concept linear_term = requires { typename std::tuple_size<T>::type; } &&
                      (std::tuple_size_v<T> == 2);

template <linear_term LT>
using linear_term_variable_t = std::decay_t<std::tuple_element_t<0, LT>>;

template <linear_term LT>
using linear_term_scalar_t = std::decay_t<std::tuple_element_t<1, LT>>;

template <typename LE>
using linear_terms_range_t = decltype(std::declval<LE &>().linear_terms());

template <typename LE>
using linear_term_t = std::ranges::range_value_t<linear_terms_range_t<LE>>;

template <typename LE>
using linear_expression_variable_t = linear_term_variable_t<linear_term_t<LE>>;

template <typename LE>
using linear_expression_scalar_t = linear_term_scalar_t<linear_term_t<LE>>;

template <typename LE>
using linear_expression_constant_t =
    std::decay_t<decltype(std::declval<LE &>().constant())>;

template <typename LE>
concept linear_expression =
    linear_term<linear_term_t<LE>> &&
    std::convertible_to<linear_expression_constant_t<LE>,
                        linear_expression_scalar_t<LE>>;

template <typename E1, typename E2>
concept compatible_linear_expressions =
    linear_expression<E1> && linear_expression<E2> &&
    std::same_as<linear_expression_variable_t<E1>,
                 linear_expression_variable_t<E2>> &&
    std::same_as<linear_expression_scalar_t<E1>,
                 linear_expression_scalar_t<E2>>;

// Reading the terms does not consume the expression: they are reachable through
// a `const &`, which by construction cannot move out of it. An expression that
// owns a move-only term range fails this.
template <typename E>
concept const_readable_linear_terms =
    requires(const std::remove_cvref_t<E> & e) { e.linear_terms(); };

// Const-readable *and* restartable. Everything that walks the terms more than
// once needs this (e.g., lazily joined ranges built by `xsum()` fails this).
template <typename E>
concept multipass_linear_terms =
    const_readable_linear_terms<E> &&
    std::ranges::forward_range<
        linear_terms_range_t<const std::remove_cvref_t<E>>>;

namespace detail {
template <typename E1, typename E2>
consteval void assert_compatible_linear_expressions() {
    static_assert(std::same_as<linear_expression_variable_t<E1>,
                               linear_expression_variable_t<E2>>,
                  "MIP++: these expressions use different variable types.");
    static_assert(std::same_as<linear_expression_scalar_t<E1>,
                               linear_expression_scalar_t<E2>>,
                  "MIP++: these expressions use different scalar types; "
                  "cast one side explicitly.");
}

// An lvalue expression can only be consumed if its term range is copyable.
// Owning expressions (rvalue-range-backed) must be forwarded as rvalues.
template <typename E>
concept forwardable_linear_terms =
    (!std::is_lvalue_reference_v<E> &&
     !std::is_const_v<std::remove_reference_t<E>>) ||
    std::copy_constructible<
        std::remove_cvref_t<linear_terms_range_t<std::remove_reference_t<E>>>>;

template <typename... Es>
consteval void assert_forwardable_linear_terms() {
    static_assert(
        (forwardable_linear_terms<Es> && ...),
        "MIP++: this expression owns its term range (it was built from an "
        "rvalue range) and is single-use. Either pass it as an rvalue "
        "(std::move) if this is its last use, build it over a named range "
        "so it only references the terms, or materialize it into a "
        "runtime_linear_expression for reuse.");
}

// A constraint is read through a `const &` (see the `linear_constraint`
// concept), so `linear_constraint_view` constrains its template parameter on
// const-readability. Without this assert the comparison operators would fail
// with a class-template-argument-deduction error naming no cause.
template <typename... Es>
consteval void assert_constrainable_linear_expressions() {
    static_assert(
        (const_readable_linear_terms<Es> && ...),
        "MIP++: this expression owns a move-only term range (it was built from "
        "an rvalue range), so it cannot back a constraint: reading a "
        "constraint must not consume it. Build the expression over a named "
        "range so it only references the terms, or materialize it into a "
        "runtime_linear_expression.");
}
}  // namespace detail

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// Views ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <
    std::ranges::view Terms,
    std::convertible_to<linear_term_scalar_t<std::ranges::range_value_t<Terms>>>
        Constant = linear_term_scalar_t<std::ranges::range_value_t<Terms>>>
class linear_expression_view {
private:
    Terms _terms;
    [[no_unique_address]] Constant _constant;

public:
    template <std::ranges::viewable_range R>
        requires std::constructible_from<Terms, std::views::all_t<R &&>>
    constexpr explicit linear_expression_view(R && terms)
        : _terms(std::views::all(std::forward<R>(terms))), _constant{} {}

    template <std::ranges::viewable_range R, std::convertible_to<Constant> C>
        requires std::constructible_from<Terms, std::views::all_t<R &&>>
    constexpr linear_expression_view(R && terms, C constant)
        : _terms(std::views::all(std::forward<R>(terms)))
        , _constant(static_cast<Constant>(constant)) {}

    [[nodiscard]] constexpr Terms linear_terms() const & noexcept(
        std::is_nothrow_copy_constructible_v<Terms>)
        requires std::copy_constructible<Terms>
    {
        return _terms;
    }
    // lvalue access must not copy: Terms may be move-only
    [[nodiscard]] constexpr Terms & linear_terms() & noexcept { return _terms; }
    [[nodiscard]] constexpr Terms && linear_terms() && noexcept {
        return std::move(_terms);
    }
    [[nodiscard]] constexpr Constant constant() const & noexcept {
        return _constant;
    }
    [[nodiscard]] constexpr Constant && constant() && noexcept {
        return std::move(_constant);
    }
};

template <std::ranges::viewable_range Terms>
linear_expression_view(Terms &&)
    -> linear_expression_view<std::views::all_t<Terms>, zero_t>;

template <std::ranges::viewable_range Terms, typename Constant>
linear_expression_view(Terms &&, Constant) -> linear_expression_view<
    std::views::all_t<Terms>,
    std::conditional_t<
        statically_zero<Constant>, zero_t,
        linear_term_scalar_t<std::ranges::range_value_t<Terms>>>>;

template <typename V, typename S>
constexpr auto empty_linear_expression =
    linear_expression_view(std::views::empty<std::pair<V, S>>, zero);

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////// Operations //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <linear_expression E1, linear_expression E2>
constexpr auto linear_expression_add(E1 && e1, E2 && e2) {
    detail::assert_compatible_linear_expressions<E1, E2>();
    detail::assert_forwardable_linear_terms<E1, E2>();
    auto constant = e1.constant() + e2.constant();
    return linear_expression_view(
        detail::unordered_concat(std::forward<E1>(e1).linear_terms(),
                                 std::forward<E2>(e2).linear_terms()),
        std::move(constant));
}

template <linear_expression E>
constexpr auto linear_expression_negate(E && e) {
    detail::assert_forwardable_linear_terms<E>();
    auto constant = -e.constant();
    return linear_expression_view(
        std::views::transform(std::forward<E>(e).linear_terms(),
                              [](auto && t) {
                                  return std::make_pair(std::get<0>(t),
                                                        -std::get<1>(t));
                              }),
        std::move(constant));
}

template <linear_expression E, typename S>
constexpr auto linear_expression_scalar_add(E && e, const S c) {
    detail::assert_forwardable_linear_terms<E>();
    auto constant = e.constant() + c;
    return linear_expression_view(std::forward<E>(e).linear_terms(),
                                  std::move(constant));
}

template <linear_expression E, typename S>
constexpr auto linear_expression_scalar_mul(E && e, const S c) {
    detail::assert_forwardable_linear_terms<E>();
    auto constant = c * e.constant();
    return linear_expression_view(
        std::views::transform(std::forward<E>(e).linear_terms(),
                              [c](auto && t) {
                                  return std::make_pair(std::get<0>(t),
                                                        c * std::get<1>(t));
                              }),
        std::move(constant));
}

template <linear_expression E, typename S>
constexpr auto linear_expression_scalar_div(E && e, const S c) {
    detail::assert_forwardable_linear_terms<E>();
    auto constant = e.constant() / c;
    return linear_expression_view(
        std::views::transform(std::forward<E>(e).linear_terms(),
                              [c](auto && t) {
                                  return std::make_pair(std::get<0>(t),
                                                        std::get<1>(t) / c);
                              }),
        std::move(constant));
}

template <std::ranges::input_range R>
    requires linear_expression<std::ranges::range_reference_t<R>>
constexpr auto linear_expressions_sum(R && r) {
    using expression_t = std::ranges::range_reference_t<R>;
    using constant_t = linear_expression_constant_t<expression_t>;
    detail::assert_forwardable_linear_terms<expression_t>();
    auto terms_of = [](auto && e) {
        return std::views::all(std::forward<decltype(e)>(e).linear_terms());
    };
    if constexpr(statically_zero<constant_t>) {
        return linear_expression_view(std::views::join(std::views::transform(
                                          std::forward<R>(r), terms_of)),
                                      zero);
    } else {
        static_assert(std::ranges::forward_range<R>,
                      "summing expressions with runtime constants traverses "
                      "the range twice and requires a forward_range");
        constant_t constant{0};
        for(auto && e : r) constant += e.constant();
        return linear_expression_view(std::views::join(std::views::transform(
                                          std::forward<R>(r), terms_of)),
                                      std::move(constant));
    }
}

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Operators //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

namespace operators {

template <linear_expression E1, linear_expression E2>
    requires compatible_linear_expressions<E1, E2>
[[nodiscard]] constexpr auto operator+(E1 && e1, E2 && e2) {
    return linear_expression_add(std::forward<E1>(e1), std::forward<E2>(e2));
}

template <linear_expression E>
[[nodiscard]] constexpr auto operator-(E && e) {
    return linear_expression_negate(std::forward<E>(e));
}

template <linear_expression E1, linear_expression E2>
    requires compatible_linear_expressions<E1, E2>
[[nodiscard]] constexpr auto operator-(E1 && e1, E2 && e2) {
    return linear_expression_add(
        std::forward<E1>(e1), linear_expression_negate(std::forward<E2>(e2)));
}

template <linear_expression E>
[[nodiscard]] constexpr auto operator+(E && e,
                                       linear_expression_scalar_t<E> c) {
    return linear_expression_scalar_add(std::forward<E>(e), c);
}

template <linear_expression E>
[[nodiscard]] constexpr auto operator+(linear_expression_scalar_t<E> c,
                                       E && e) {
    return linear_expression_scalar_add(std::forward<E>(e), c);
}

template <linear_expression E>
[[nodiscard]] constexpr auto operator-(E && e,
                                       linear_expression_scalar_t<E> c) {
    return linear_expression_scalar_add(std::forward<E>(e), -c);
}

template <linear_expression E>
[[nodiscard]] constexpr auto operator-(linear_expression_scalar_t<E> c,
                                       E && e) {
    return linear_expression_scalar_add(
        linear_expression_negate(std::forward<E>(e)), c);
}

template <linear_expression E>
[[nodiscard]] constexpr auto operator*(E && e,
                                       linear_expression_scalar_t<E> c) {
    return linear_expression_scalar_mul(std::forward<E>(e), c);
}

template <linear_expression E>
[[nodiscard]] constexpr auto operator*(linear_expression_scalar_t<E> c,
                                       E && e) {
    return linear_expression_scalar_mul(std::forward<E>(e), c);
}

template <linear_expression E>
[[nodiscard]] constexpr auto operator/(E && e,
                                       linear_expression_scalar_t<E> c) {
    return linear_expression_scalar_div(std::forward<E>(e), c);
}

template <std::ranges::input_range R>
    requires linear_expression<std::ranges::range_reference_t<R>>
[[nodiscard]] constexpr auto xsum(R && r) {
    return linear_expressions_sum(std::forward<R>(r));
}

template <std::ranges::input_range R, typename F>
    requires linear_expression<
        std::invoke_result_t<F &, std::ranges::range_reference_t<R>>>
[[nodiscard]] constexpr auto xsum(R && r, F && f) {
    return linear_expressions_sum(
        std::views::transform(std::forward<R>(r), std::forward<F>(f)));
}

}  // namespace operators

///////////////////////////////////////////////////////////////////////////////
///////////////////////////// Runtime expression //////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename Variable, typename Scalar,
          std::convertible_to<Scalar> Constant = Scalar,
          typename Container = std::vector<std::pair<Variable, Scalar>>>
    requires std::same_as<
        linear_term_scalar_t<std::ranges::range_value_t<Container>>, Scalar>
class runtime_linear_expression {
private:
    Container _terms;
    [[no_unique_address]] Constant _constant;

public:
    constexpr runtime_linear_expression() : _terms(), _constant() {}

    [[nodiscard]] constexpr const Container & linear_terms() const & noexcept {
        return _terms;
    }
    [[nodiscard]] constexpr Container && linear_terms() && noexcept {
        return std::move(_terms);
    }
    [[nodiscard]] constexpr const Constant & constant() const & noexcept {
        return _constant;
    }
    [[nodiscard]] constexpr Constant && constant() && noexcept {
        return std::move(_constant);
    }

    template <linear_expression E>
        requires std::same_as<linear_expression_variable_t<E>, Variable> &&
                 std::constructible_from<Scalar,
                                         linear_expression_scalar_t<E>> &&
                 std::convertible_to<linear_expression_constant_t<E>, Constant>
    constexpr runtime_linear_expression & operator+=(E && e) {
        _constant += e.constant();
        auto terms = std::forward<E>(e).linear_terms();
        if constexpr(std::ranges::sized_range<linear_terms_range_t<E>>) {
            _terms.reserve(_terms.size() + terms.size());
        }
        for(auto && [var, coef] : terms) {
            _terms.emplace_back(var, coef);
        }
        return *this;
    }

    template <std::convertible_to<Constant> C>
    constexpr runtime_linear_expression & operator+=(C && c) {
        _constant += std::forward<C>(c);
        return *this;
    }
};

template <linear_expression E>
[[nodiscard]] constexpr auto materialize(E && e) {
    runtime_linear_expression<linear_expression_variable_t<E>,
                              linear_expression_scalar_t<E>,
                              linear_expression_constant_t<E>>
        r;
    r += std::forward<E>(e);
    return r;
}

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Evaluate ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// `LE &&` (not `const LE &`): `linear_expression_view::linear_terms() const &`
// returns a copy of the view, and doesn't exist at all when Terms is move-only.
template <linear_expression LE, typename VM>
constexpr auto evaluate(LE && e, const VM & values_map) {
    using scalar = linear_expression_scalar_t<LE>;
    scalar acc = static_cast<scalar>(e.constant());
    for(auto && [var, coef] : e.linear_terms()) acc += values_map[var] * coef;
    return acc;
}

}  // namespace mippp
