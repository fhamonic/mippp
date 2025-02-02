#ifndef MIPPP_LINEAR_CONSTRAINT_OPERATIONS_HPP
#define MIPPP_LINEAR_CONSTRAINT_OPERATIONS_HPP

#include <functional>

#include <iostream>

#include <range/v3/view/concat.hpp>
#include <range/v3/view/transform.hpp>

#include "mippp/concepts/linear_expression.hpp"
#include "mippp/constraints/linear_constraint.hpp"

namespace fhamonic {
namespace mippp {

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
        ranges::views::concat(
            e1.coefficients(),
            ranges::views::transform(e2.coefficients(),
                                     std::negate<scalar_t>())),
        e2.constant() - e1.constant(), e2.constant() - e1.constant());
}

template <linear_expression_c E>
class linear_constraint_bounds;

template <linear_expression_c E>
class linear_constraint_lower_bound {
    friend class linear_constraint_bounds<E>;

public:
    using var_id_t = expression_var_id_t<E>;
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
    using var_id_t = expression_var_id_t<E>;
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
    using var_id_t = expression_var_id_t<E>;
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

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_LINEAR_CONSTRAINT_OPERATIONS_HPP