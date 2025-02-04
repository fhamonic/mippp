#ifndef MIPPP_LINEAR_EXPRESSION_HPP
#define MIPPP_LINEAR_EXPRESSION_HPP

#include <cassert>
#include <concepts>
#include <ranges>
#include <type_traits>

#include <range/v3/view/all.hpp>
#include <range/v3/view/concat.hpp>
#include <range/v3/view/transform.hpp>

namespace fhamonic {
namespace mippp {

/////////////////////////////////// CONCEPT ///////////////////////////////////

template <typename _Tp>
using variables_range_t = decltype(std::declval<_Tp &>().variables());

template <typename _Tp>
using coefficients_range_t = decltype(std::declval<_Tp &>().coefficients());

template <typename _Tp>
using expression_variable_id_t =
    std::ranges::range_value_t<variables_range_t<_Tp>>;

template <typename _Tp>
using expression_scalar_t =
    std::ranges::range_value_t<coefficients_range_t<_Tp>>;

template <typename _Tp>
concept linear_expression_c = requires(const _Tp & __t) {
    { __t.variables() } -> std::ranges::range;
    { __t.coefficients() } -> std::ranges::range;
    { __t.constant() } -> std::convertible_to<expression_scalar_t<_Tp>>;
};

//////////////////////////////////// CLASS ////////////////////////////////////

template <typename _Vars, typename _Coefs>
class linear_expression {
public:
    using variable_id_t = std::ranges::range_value_t<_Vars>;
    using scalar_t = std::ranges::range_value_t<_Coefs>;

private:
    _Vars _variables;
    _Coefs _coefficients;
    scalar_t _constant;

public:
    template <std::ranges::range V, std::ranges::range C>
    [[nodiscard]] constexpr linear_expression(V && variables, C && coefficients)
        : _variables(ranges::views::all(std::forward<V>(variables)))
        , _coefficients(ranges::views::all(std::forward<C>(coefficients)))
        , _constant(static_cast<scalar_t>(0)) {}

    template <std::ranges::range V, std::ranges::range C, typename S>
    [[nodiscard]] constexpr linear_expression(V && variables, C && coefficients,
                                              S constant)
        : _variables(ranges::views::all(std::forward<V>(variables)))
        , _coefficients(ranges::views::all(std::forward<C>(coefficients)))
        , _constant(static_cast<scalar_t>(constant)) {}

    [[nodiscard]] constexpr const _Vars & variables() const & noexcept {
        return _variables;
    }
    [[nodiscard]] constexpr _Vars && variables() && noexcept {
        return std::move(_variables);
    }
    [[nodiscard]] constexpr const _Coefs & coefficients() const & noexcept {
        return _coefficients;
    }
    [[nodiscard]] constexpr _Coefs && coefficients() && noexcept {
        return std::move(_coefficients);
    }
    [[nodiscard]] constexpr const scalar_t & constant() const & noexcept {
        return _constant;
    }
    [[nodiscard]] constexpr scalar_t && constant() && noexcept {
        return std::move(_constant);
    }
};

template <typename V, typename C>
linear_expression(V &&, C &&)
    -> linear_expression<ranges::views::all_t<V>, ranges::views::all_t<C>>;

template <typename V, typename C, typename S>
    requires std::convertible_to<S, std::ranges::range_value_t<C>>
linear_expression(V &&, C &&, S)
    -> linear_expression<ranges::views::all_t<V>, ranges::views::all_t<C>>;

///////////////////////////////// OPERATIONS //////////////////////////////////

template <linear_expression_c E1, linear_expression_c E2>
constexpr auto linear_expression_add(E1 && e1, E2 && e2) {
    return linear_expression(
        ranges::views::concat(std::forward<E1>(e1).variables(),
                              std::forward<E2>(e2).variables()),
        ranges::views::concat(std::forward<E1>(e1).coefficients(),
                              std::forward<E2>(e2).coefficients()),
        std::forward<E1>(e1).constant() + std::forward<E2>(e2).constant());
}

template <linear_expression_c E>
constexpr auto linear_expression_negate(E && e) {
    using scalar_t = expression_scalar_t<E>;
    return linear_expression(
        std::forward<E>(e).variables(),
        ranges::views::transform(std::forward<E>(e).coefficients(),
                                 std::negate<scalar_t>()),
        -std::forward<E>(e).constant());
}

template <linear_expression_c E, typename S>
constexpr auto linear_expression_scalar_add(E && e, const S c) {
    using scalar_t = expression_scalar_t<E>;
    return linear_expression(
        std::forward<E>(e).variables(), std::forward<E>(e).coefficients(),
        std::forward<E>(e).constant() + static_cast<scalar_t>(c));
}

template <linear_expression_c E, typename S>
constexpr auto linear_expression_scalar_mul(E && e, const S c) {
    using scalar_t = expression_scalar_t<E>;
    return linear_expression(
        std::forward<E>(e).variables(),
        ranges::views::transform(
            std::forward<E>(e).coefficients(),
            [c](auto && coef) -> scalar_t { return c * coef; }),
        std::forward<E>(e).constant() * c);
}

////////////////////////////////// OPERATORS //////////////////////////////////

namespace operators {

template <linear_expression_c E1, linear_expression_c E2>
    requires std::same_as<expression_variable_id_t<E1>, expression_variable_id_t<E2>> &&
             std::same_as<expression_scalar_t<E1>, expression_scalar_t<E2>>
[[nodiscard]] constexpr auto operator+(E1 && e1, E2 && e2) {
    return linear_expression_add(std::forward<E1>(e1), std::forward<E2>(e2));
};

template <linear_expression_c E>
[[nodiscard]] constexpr auto operator-(E && e) {
    return linear_expression_negate(std::forward<E>(e));
};

template <linear_expression_c E1, linear_expression_c E2>
    requires std::same_as<expression_variable_id_t<E1>, expression_variable_id_t<E2>> &&
             std::same_as<expression_scalar_t<E1>, expression_scalar_t<E2>>
[[nodiscard]] constexpr auto operator-(E1 && e1, E2 && e2) {
    return linear_expression_add(
        std::forward<E1>(e1), linear_expression_negate(std::forward<E2>(e2)));
};

template <linear_expression_c E>
[[nodiscard]] constexpr auto operator+(E && e, expression_scalar_t<E> c) {
    return linear_expression_scalar_add(std::forward<E>(e), c);
};

template <linear_expression_c E>
[[nodiscard]] constexpr auto operator+(expression_scalar_t<E> c, E && e) {
    return linear_expression_scalar_add(std::forward<E>(e), c);
};

template <linear_expression_c E>
[[nodiscard]] constexpr auto operator-(E && e, expression_scalar_t<E> c) {
    return linear_expression_scalar_add(std::forward<E>(e), -c);
};

template <linear_expression_c E>
[[nodiscard]] constexpr auto operator-(expression_scalar_t<E> c, E && e) {
    return linear_expression_scalar_add(
        linear_expression_negate(std::forward<E>(e)), c);
};

template <linear_expression_c E>
[[nodiscard]] constexpr auto operator*(E && e, expression_scalar_t<E> c) {
    return linear_expression_scalar_mul(std::forward<E>(e), c);
};

template <linear_expression_c E>
[[nodiscard]] constexpr auto operator*(expression_scalar_t<E> c, E && e) {
    return linear_expression_scalar_mul(std::forward<E>(e), c);
};

template <linear_expression_c E>
[[nodiscard]] constexpr auto operator/(E && e, expression_scalar_t<E> c) {
    return linear_expression_scalar_mul(std::forward<E>(e),
                                        expression_scalar_t<E>{1} / c);
};

}  // namespace operators

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_LINEAR_EXPRESSION_HPP