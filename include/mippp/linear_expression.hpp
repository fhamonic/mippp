#pragma once

#include <algorithm>
#include <concepts>
#include <functional>
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
using linear_expression_constant_t = decltype(std::declval<LE &>().constant());

template <typename LE>
concept linear_expression =
    linear_term<linear_term_t<LE>> &&
    std::convertible_to<linear_expression_constant_t<LE>,
                        linear_expression_scalar_t<LE>>;

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// Views ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <std::ranges::view Terms,
          typename Constant =
              linear_term_scalar_t<std::ranges::range_value_t<Terms>>>
class linear_expression_view {
private:
    Terms _terms;
    [[no_unique_address]] Constant _constant;

public:
    template <std::ranges::range T>
    constexpr linear_expression_view(T && terms)
        : _terms(std::views::all(std::forward<T>(terms))), _constant{} {}

    template <std::ranges::range T, typename S>
    constexpr linear_expression_view(T && terms, S constant)
        : _terms(std::views::all(std::forward<T>(terms)))
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

template <typename Terms>
linear_expression_view(Terms &&)
    -> linear_expression_view<std::views::all_t<Terms>, zero_t>;

template <typename Terms, typename Constant>
linear_expression_view(Terms &&, Constant) -> linear_expression_view<
    std::views::all_t<Terms>,
    std::conditional_t<statically_zero<Constant>, zero_t,
                       linear_term_scalar_t<std::ranges::range_value_t<
                           std::views::all_t<Terms>>>>>;

template <typename V, typename S>
constexpr auto empty_linear_expression =
    linear_expression_view(std::views::empty<std::pair<V, S>>, zero_t{});

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////// Operations //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

namespace detail {
// An lvalue expression can only be consumed if its term range is copyable;
// owning (rvalue-range-backed) expressions must be forwarded as rvalues.
template <typename E>  // E as deduced by the E&& parameter
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
}  // namespace detail

template <linear_expression E1, linear_expression E2>
constexpr auto linear_expression_add(E1 && e1, E2 && e2) {
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
    requires linear_expression<std::ranges::range_value_t<R>>
constexpr auto linear_expressions_sum(R && r) {
    using expression = std::ranges::range_value_t<R>;
    using constant_t = linear_expression_constant_t<expression>;
    auto terms_of = [](auto && e) {
        return std::forward<decltype(e)>(e).linear_terms();
    };
    if constexpr(statically_zero<constant_t>) {
        return linear_expression_view(std::views::join(std::views::transform(
                                          std::forward<R>(r), terms_of)),
                                      zero_t{});
    } else {
        static_assert(std::ranges::forward_range<R>,
                      "summing expressions with runtime constants traverses "
                      "the range twice and requires a forward_range");
        constant_t constant{0};
        for(const expression & e : r) constant += e.constant();
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
    requires std::same_as<linear_expression_variable_t<E1>,
                          linear_expression_variable_t<E2>> &&
             std::same_as<linear_expression_scalar_t<E1>,
                          linear_expression_scalar_t<E2>>
[[nodiscard]] constexpr auto operator+(E1 && e1, E2 && e2) {
    return linear_expression_add(std::forward<E1>(e1), std::forward<E2>(e2));
}

template <linear_expression E>
[[nodiscard]] constexpr auto operator-(E && e) {
    return linear_expression_negate(std::forward<E>(e));
}

template <linear_expression E1, linear_expression E2>
    requires std::same_as<linear_expression_variable_t<E1>,
                          linear_expression_variable_t<E2>> &&
             std::same_as<linear_expression_scalar_t<E1>,
                          linear_expression_scalar_t<E2>>
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
    requires linear_expression<std::ranges::range_value_t<R>>
[[nodiscard]] constexpr auto xsum(R && r) {
    return linear_expressions_sum(std::forward<R>(r));
}

template <std::ranges::input_range R, typename F>
    requires linear_expression<
        std::invoke_result_t<F, std::ranges::range_reference_t<R>>>
[[nodiscard]] constexpr auto xsum(R && r, F && f) {
    return linear_expressions_sum(
        std::views::transform(std::forward<R>(r), std::forward<F>(f)));
}

}  // namespace operators

///////////////////////////////////////////////////////////////////////////////
///////////////////////////// Runtime expression //////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename Variable, typename Scalar,
          typename Container = std::vector<std::pair<Variable, Scalar>>>
class runtime_linear_expression {
private:
    Container _terms;
    Scalar _constant;

public:
    constexpr runtime_linear_expression() : _terms(), _constant(0) {}

    [[nodiscard]] constexpr const Container & linear_terms() const & noexcept {
        return _terms;
    }
    [[nodiscard]] constexpr Container && linear_terms() && noexcept {
        return std::move(_terms);
    }
    [[nodiscard]] constexpr const Scalar & constant() const & noexcept {
        return _constant;
    }
    [[nodiscard]] constexpr Scalar && constant() && noexcept {
        return std::move(_constant);
    }

    template <linear_expression E>
        requires std::same_as<linear_expression_variable_t<E>, Variable> &&
                 std::same_as<linear_expression_scalar_t<E>, Scalar>
    constexpr runtime_linear_expression & operator+=(E && e) {
        _constant += e.constant();
        if constexpr(std::ranges::sized_range<linear_terms_range_t<E>>) {
            _terms.reserve(_terms.size() + e.linear_terms().size());
        }
        for(auto && [var, coef] : e.linear_terms()) {
            _terms.emplace_back(var, coef);
        }
        return *this;
    }
    constexpr runtime_linear_expression & operator+=(Scalar c) {
        _constant += c;
        return *this;
    }
};

template <linear_expression E>
[[nodiscard]] constexpr auto materialize(E && e) {
    runtime_linear_expression<linear_expression_variable_t<E>,
                              linear_expression_scalar_t<E>>
        r;
    r += std::forward<E>(e);
    return r;
}

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Evaluate ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename LE, typename VM>
    requires linear_expression<std::remove_cvref_t<LE>>
constexpr auto evaluate(LE && e, const VM & values_map) {
    using scalar = linear_expression_scalar_t<std::remove_cvref_t<LE>>;
    scalar acc = static_cast<scalar>(e.constant());
    for(auto && [var, coef] : std::forward<LE>(e).linear_terms())
        acc += values_map[var] * coef;
    return acc;
}

}  // namespace mippp
