#ifndef MIPPP_LINEAR_CONSTRAINT_HPP
#define MIPPP_LINEAR_CONSTRAINT_HPP

#include <cassert>
#include <ranges>
#include <type_traits>

#include <range/v3/view/all.hpp>
#include <range/v3/view/concat.hpp>
#include <range/v3/view/single.hpp>
#include <range/v3/view/transform.hpp>

#include "mippp/linear_expression.hpp"

namespace fhamonic {
namespace mippp {

/////////////////////////////////// CONCEPT ///////////////////////////////////

template <typename _Tp>
using variables_range_t = decltype(std::declval<_Tp &>().variables());

template <typename _Tp>
using coefficients_range_t = decltype(std::declval<_Tp &>().coefficients());

template <typename _Tp>
using constraint_variable_id_t =
    std::ranges::range_value_t<variables_range_t<_Tp>>;

template <typename _Tp>
using constraint_scalar_t =
    std::ranges::range_value_t<coefficients_range_t<_Tp>>;

template <typename _Tp>
concept linear_constraint_c = requires(const _Tp & __t) {
    { __t.variables() } -> std::ranges::range;
    { __t.coefficients() } -> std::ranges::range;
    { __t.lower_bound() } -> std::convertible_to<constraint_scalar_t<_Tp>>;
    { __t.upper_bound() } -> std::convertible_to<constraint_scalar_t<_Tp>>;
};

//////////////////////////////////// CLASS ////////////////////////////////////

template <typename _Vars, typename _Coefs>
class linear_constraint {
public:
    using variable_id_t = std::ranges::range_value_t<_Vars>;
    using scalar_t = std::ranges::range_value_t<_Coefs>;

private:
    _Vars _variables;
    _Coefs _coefficients;
    scalar_t _lb;
    scalar_t _ub;

public:
    template <std::ranges::range V, std::ranges::range C, typename S>
    constexpr linear_constraint(V && variables, C && coefficients, S lb = 0,
                                S ub = 0)
        : _variables(ranges::views::all(std::forward<V>(variables)))
        , _coefficients(ranges::views::all(std::forward<C>(coefficients)))
        , _lb(static_cast<scalar_t>(lb))
        , _ub(static_cast<scalar_t>(ub)) {}

    constexpr const _Vars & variables() const noexcept { return _variables; }
    constexpr const _Coefs & coefficients() const noexcept {
        return _coefficients;
    }
    constexpr scalar_t lower_bound() const noexcept { return _lb; }
    constexpr scalar_t upper_bound() const noexcept { return _ub; }
};

template <typename V, typename C, typename S>
    requires std::convertible_to<S, std::ranges::range_value_t<C>>
linear_constraint(V &&, C &&, S, S)
    -> linear_constraint<ranges::views::all_t<V>, ranges::views::all_t<C>>;

///////////////////////////////// OPERATIONS //////////////////////////////////

template <linear_expression_c E1, linear_expression_c E2>
constexpr auto linear_constraint_less_equal(E1 && e1, E2 && e2) {
    using scalar_t = expression_scalar_t<E1>;
    return linear_constraint(
        ranges::views::concat(e1.variables(), e2.variables()),
        ranges::views::concat(e1.coefficients(),
                              ranges::views::transform(
                                  e2.coefficients(), std::negate<scalar_t>())),
        std::numeric_limits<scalar_t>::lowest(), e2.constant() - e1.constant());
}

template <linear_expression_c E1, linear_expression_c E2>
constexpr auto linear_constraint_equal(E1 && e1, E2 && e2) {
    using scalar_t = expression_scalar_t<E1>;
    return linear_constraint(
        ranges::views::concat(e1.variables(), e2.variables()),
        ranges::views::concat(e1.coefficients(),
                              ranges::views::transform(
                                  e2.coefficients(), std::negate<scalar_t>())),
        e2.constant() - e1.constant(), e2.constant() - e1.constant());
}

template <linear_expression_c E>
class linear_constraint_bounds;

template <linear_expression_c E>
class linear_constraint_lower_bound {
    friend class linear_constraint_bounds<E>;

public:
    using variable_id_t = expression_variable_id_t<E>;
    using scalar_t = expression_scalar_t<E>;

private:
    E _expr;
    const scalar_t _scalar;

public:
    constexpr linear_constraint_lower_bound(E && e, const scalar_t c)
        : _expr(std::forward<E>(e)), _scalar(c) {};

    constexpr auto variables() const noexcept { return _expr.variables(); }
    constexpr auto coefficients() const noexcept {
        return _expr.coefficients();
    }
    constexpr scalar_t lower_bound() const noexcept {
        return _scalar - _expr.constant();
    }
    constexpr scalar_t upper_bound() const noexcept {
        return std::numeric_limits<scalar_t>::max();
    }
};

template <linear_expression_c E>
class linear_constraint_upper_bound {
    friend class linear_constraint_bounds<E>;

public:
    using variable_id_t = expression_variable_id_t<E>;
    using scalar_t = expression_scalar_t<E>;

private:
    E _expr;
    const scalar_t _scalar;

public:
    constexpr linear_constraint_upper_bound(E && e, const scalar_t c)
        : _expr(std::forward<E>(e)), _scalar(c) {};

    constexpr auto variables() const noexcept { return _expr.variables(); }
    constexpr auto coefficients() const noexcept {
        return _expr.coefficients();
    }
    constexpr scalar_t lower_bound() const noexcept {
        return std::numeric_limits<scalar_t>::lowest();
    }
    constexpr scalar_t upper_bound() const noexcept {
        return _scalar - _expr.constant();
    }
};

template <linear_expression_c E>
class linear_constraint_bounds {
public:
    using variable_id_t = expression_variable_id_t<E>;
    using scalar_t = expression_scalar_t<E>;

private:
    E _expr;
    const scalar_t _lb;
    const scalar_t _ub;

public:
    constexpr linear_constraint_bounds(E && e, const scalar_t lb,
                                       const scalar_t ub)
        : _expr(std::forward<E>(e)), _lb(lb), _ub(ub) {};

    constexpr linear_constraint_bounds(linear_constraint_lower_bound<E> && e,
                                       const scalar_t ub)
        : _expr(std::forward<E>(e._expr)), _lb(e._scalar), _ub(ub) {};

    constexpr linear_constraint_bounds(linear_constraint_upper_bound<E> && e,
                                       const scalar_t lb)
        : _expr(std::forward<E>(e._expr)), _lb(lb), _ub(e._scalar) {};

    constexpr auto variables() const noexcept { return _expr.variables(); }
    constexpr auto coefficients() const noexcept {
        return _expr.coefficients();
    }
    constexpr scalar_t lower_bound() const noexcept {
        return _lb - _expr.constant();
    }
    constexpr scalar_t upper_bound() const noexcept {
        return _ub - _expr.constant();
    }
};

////////////////////////////////// OPERATORS //////////////////////////////////

namespace operators {

template <linear_expression_c E1, linear_expression_c E2>
    requires std::same_as<expression_variable_id_t<E1>,
                          expression_variable_id_t<E2>> &&
             std::same_as<expression_scalar_t<E1>, expression_scalar_t<E2>>
constexpr auto operator<=(E1 && e1, E2 && e2) {
    return linear_constraint_less_equal<E1 &&, E2 &&>(std::forward<E1>(e1),
                                                      std::forward<E2>(e2));
};
template <linear_expression_c E1, linear_expression_c E2>
    requires std::same_as<expression_variable_id_t<E1>,
                          expression_variable_id_t<E2>> &&
             std::same_as<expression_scalar_t<E1>, expression_scalar_t<E2>>
constexpr auto operator>=(E1 && e1, E2 && e2) {
    return linear_constraint_less_equal<E2 &&, E1 &&>(std::forward<E2>(e2),
                                                      std::forward<E1>(e1));
};

template <linear_expression_c E1, linear_expression_c E2>
    requires std::same_as<expression_variable_id_t<E1>,
                          expression_variable_id_t<E2>> &&
             std::same_as<expression_scalar_t<E1>, expression_scalar_t<E2>>
constexpr auto operator==(E1 && e1, E2 && e2) {
    return linear_constraint_equal<E1 &&, E2 &&>(std::forward<E1>(e1),
                                                 std::forward<E2>(e2));
};

template <linear_expression_c E>
constexpr auto operator<=(expression_scalar_t<E> c, E && e) {
    return linear_constraint_lower_bound<E &&>(std::forward<E>(e), c);
};
template <linear_expression_c E>
constexpr auto operator>=(E && e, expression_scalar_t<E> c) {
    return linear_constraint_lower_bound<E &&>(std::forward<E>(e), c);
};

template <linear_expression_c E>
constexpr auto operator<=(E && e, expression_scalar_t<E> c) {
    return linear_constraint_upper_bound<E &&>(std::forward<E>(e), c);
};
template <linear_expression_c E>
constexpr auto operator>=(expression_scalar_t<E> c, E && e) {
    return linear_constraint_upper_bound<E &&>(std::forward<E>(e), c);
};

template <linear_expression_c E>
constexpr auto operator==(E && e, expression_scalar_t<E> c) {
    return linear_constraint_bounds<E &&>(std::forward<E>(e), c, c);
};
template <linear_expression_c E>
constexpr auto operator==(expression_scalar_t<E> c, E && e) {
    return linear_constraint_bounds<E &&>(std::forward<E>(e), c, c);
};

template <linear_expression_c E>
constexpr auto operator<=(linear_constraint_lower_bound<E> && c,
                          expression_scalar_t<E> ub) {
    return linear_constraint_bounds<E &&>(
        std::forward<linear_constraint_lower_bound<E>>(c), ub);
};
template <linear_expression_c E>
constexpr auto operator>=(expression_scalar_t<E> ub,
                          linear_constraint_lower_bound<E> && c) {
    return linear_constraint_bounds<E &&>(
        std::forward<linear_constraint_lower_bound<E>>(c), ub);
};

template <linear_expression_c E>
constexpr auto operator<=(expression_scalar_t<E> lb,
                          linear_constraint_upper_bound<E> && c) {
    return linear_constraint_bounds<E &&>(
        std::forward<linear_constraint_upper_bound<E>>(c), lb);
};
template <linear_expression_c E>
constexpr auto operator>=(linear_constraint_upper_bound<E> && c,
                          expression_scalar_t<E> lb) {
    return linear_constraint_bounds<E &&>(
        std::forward<linear_constraint_upper_bound<E>>(c), lb);
};

}  // namespace operators

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_LINEAR_CONSTRAINT_HPP