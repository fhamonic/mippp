#ifndef MIPPP_LINEAR_EXPRESSION_HPP
#define MIPPP_LINEAR_EXPRESSION_HPP

#include <algorithm>
#include <concepts>
#include <functional>
#include <ranges>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

namespace fhamonic::mippp {

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
    { __t.linear_terms() } -> std::ranges::range;
    { __t.constant() } -> std::convertible_to<linear_expression_scalar_t<_Tp>>;
} && linear_term<linear_term_t<_Tp>>;

//////////////////////////////////// CLASS ////////////////////////////////////

template <std::ranges::view _Terms>
class linear_expression_view {
private:
    using scalar_t = linear_term_scalar_t<std::ranges::range_value_t<_Terms>>;
    _Terms _terms;
    scalar_t _constant;

public:
    template <std::ranges::range T>
    [[nodiscard]] constexpr linear_expression_view(T && terms)
        : _terms(std::views::all(std::forward<T>(terms))), _constant(0) {}

    template <std::ranges::range T, typename S>
    [[nodiscard]] constexpr linear_expression_view(T && terms, S constant)
        : _terms(std::views::all(std::forward<T>(terms)))
        , _constant(static_cast<scalar_t>(constant)) {}

    [[nodiscard]] constexpr _Terms linear_terms() const & noexcept {
        return _terms;
    }
    [[nodiscard]] constexpr _Terms && linear_terms() && noexcept {
        return std::move(_terms);
    }
    [[nodiscard]] constexpr scalar_t constant() const & noexcept {
        return _constant;
    }
    [[nodiscard]] constexpr scalar_t && constant() && noexcept {
        return std::move(_constant);
    }
};

template <typename T>
linear_expression_view(T &&) -> linear_expression_view<std::views::all_t<T>>;

template <typename T, typename S>
// requires std::convertible_to<
//     S, linear_term_scalar_t<std::ranges::range_value_t<T>>>
linear_expression_view(T &&, S) -> linear_expression_view<std::views::all_t<T>>;

///////////////////////////////// OPERATIONS //////////////////////////////////

// probably required until GCC 15.2
template <std::ranges::range R1, std::ranges::range R2>
constexpr auto unordered_concat(R1 && r1, R2 && r2) {
    if constexpr(std::ranges::range<decltype(std::views::concat(
                     std::forward<R1>(r1), std::forward<R2>(r2)))>) {
        return std::views::concat(std::forward<R1>(r1), std::forward<R2>(r2));
    } else {
        return std::views::concat(std::forward<R2>(r2), std::forward<R1>(r1));
    }
}

template <linear_expression E1, linear_expression E2>
constexpr auto linear_expression_add(E1 && e1, E2 && e2) {
    return linear_expression_view(
        unordered_concat(std::forward<E1>(e1).linear_terms(),
                         std::forward<E2>(e2).linear_terms()),
        std::forward<E1>(e1).constant() + std::forward<E2>(e2).constant());
}

template <linear_expression E>
constexpr auto linear_expression_negate(E && e) {
    return linear_expression_view(
        std::views::transform(std::forward<E>(e).linear_terms(),
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
        std::views::transform(std::forward<E>(e).linear_terms(),
                              [c](auto && t) {
                                  return std::make_pair(std::get<0>(t),
                                                        c * std::get<1>(t));
                              }),
        c * std::forward<E>(e).constant());
}

template <typename R>
    requires linear_expression<std::ranges::range_value_t<R>>
constexpr auto linear_expressions_sum(R && r) {
    using expression_t = std::ranges::range_value_t<R>;
    using scalar_t = linear_expression_scalar_t<expression_t>;
    return linear_expression_view(
        std::views::join(std::views::transform(
            std::forward<R>(r),
            [](const expression_t & e) { return e.linear_terms(); })),
        std::ranges::fold_left(
            std::views::transform(
                r, [](const expression_t & e) { return e.constant(); }),
            scalar_t{0}, std::plus<scalar_t>()));
}

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

template <typename R>
    requires linear_expression<std::ranges::range_value_t<R>>
[[nodiscard]] constexpr auto xsum(R && r) {
    return linear_expressions_sum(std::forward<R>(r));
};

template <typename R, typename F>
    requires linear_expression<
        std::invoke_result_t<F, std::ranges::range_value_t<R>>>
[[nodiscard]] constexpr auto xsum(R && r, F && f) {
    return linear_expressions_sum(
        std::views::transform(std::forward<R>(r), std::forward<F>(f)));
};

}  // namespace operators

///

template <typename _Var, typename _Scalar,
          typename _Container = std::vector<std::pair<_Var, _Scalar>>>
class runtime_linear_expression {
private:
    _Container _terms;
    _Scalar _constant;

public:
    [[nodiscard]] constexpr runtime_linear_expression()
        : _terms(), _constant(0) {}

    [[nodiscard]] constexpr const _Container & linear_terms() const & noexcept {
        return _terms;
    }
    [[nodiscard]] constexpr _Container && linear_terms() && noexcept {
        return std::move(_terms);
    }
    [[nodiscard]] constexpr const _Scalar & constant() const & noexcept {
        return _constant;
    }
    [[nodiscard]] constexpr _Scalar && constant() && noexcept {
        return std::move(_constant);
    }

    template <linear_expression E>
        requires std::same_as<linear_expression_variable_t<E>, _Var> &&
                 std::same_as<linear_expression_scalar_t<E>, _Scalar>
    constexpr runtime_linear_expression & operator+=(E && e) {
        _constant += std::forward<E>(e).constant();
        for(auto && [var, coef] : std::forward<E>(e).linear_terms()) {
            _terms.emplace_back(var, coef);
        }
        return *this;
    }
    constexpr runtime_linear_expression & operator+=(_Scalar c) {
        _constant += c;
        return *this;
    }
};

}  // namespace fhamonic::mippp

#endif  // MIPPP_LINEAR_EXPRESSION_HPP