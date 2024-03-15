#ifndef MIPPP_LINEAR_EXPRESSION_OPERATIONS_HPP
#define MIPPP_LINEAR_EXPRESSION_OPERATIONS_HPP

#include <functional>

#include <iostream>

#include <range/v3/view/concat.hpp>
#include <range/v3/view/transform.hpp>

#include "mippp/concepts/linear_expression.hpp"

namespace fhamonic {
namespace mippp {

template <linear_expression_c E1, linear_expression_c E2>
requires std::same_as<expression_var_id_t<E1>, expression_var_id_t<E2>> &&
    std::same_as<expression_scalar_t<E1>, expression_scalar_t<E2>>
class linear_expression_add {
public:
    using var_id_t = expression_var_id_t<E1>;
    using scalar_t = expression_scalar_t<E1>;

private:
    E1 _lhs;
    E2 _rhs;

public:
    [[nodiscard]] constexpr linear_expression_add(E1 && e1, E2 && e2)
        : _lhs(std::forward<E1>(e1)), _rhs(std::forward<E2>(e2)){};

    [[nodiscard]] constexpr auto variables() const noexcept {
        return ranges::views::concat(_lhs.variables(), _rhs.variables());
    }
    [[nodiscard]] constexpr auto coefficients() const noexcept {
        return ranges::views::concat(_lhs.coefficients(), _rhs.coefficients());
    }
    [[nodiscard]] constexpr scalar_t constant() const noexcept {
        return _lhs.constant() + _rhs.constant();
    }
};

template <linear_expression_c E>
class linear_expression_negate {
public:
    using var_id_t = expression_var_id_t<E>;
    using scalar_t = expression_scalar_t<E>;

private:
    E _expr;

public:
    [[nodiscard]] constexpr explicit linear_expression_negate(E && e)
        : _expr(std::forward<E>(e)){};

    [[nodiscard]] constexpr auto variables() const noexcept {
        return _expr.variables();
    }
    [[nodiscard]] constexpr auto coefficients() const noexcept {
        return ranges::views::transform(_expr.coefficients(),
                                        std::negate<scalar_t>());
    }
    [[nodiscard]] constexpr scalar_t constant() const noexcept {
        return -_expr.constant();
    }
};

template <linear_expression_c E>
class linear_expression_scalar_add {
public:
    using var_id_t = expression_var_id_t<E>;
    using scalar_t = expression_scalar_t<E>;

private:
    E _expr;
    const scalar_t _scalar;

public:
    [[nodiscard]] constexpr linear_expression_scalar_add(E && e,
                                                         const scalar_t c)
        : _expr(std::forward<E>(e)), _scalar(c){};

    [[nodiscard]] constexpr auto variables() const noexcept {
        return _expr.variables();
    }
    [[nodiscard]] constexpr auto coefficients() const noexcept {
        return _expr.coefficients();
    }
    [[nodiscard]] constexpr scalar_t constant() const noexcept {
        return _expr.constant() + _scalar;
    }
};

template <linear_expression_c E>
class linear_expression_scalar_mul {
public:
    using var_id_t = expression_var_id_t<E>;
    using scalar_t = expression_scalar_t<E>;

private:
    E _expr;
    const scalar_t _scalar;

public:
    [[nodiscard]] constexpr linear_expression_scalar_mul(E && e,
                                                         const scalar_t c)
        : _expr(std::forward<E>(e)), _scalar(c){};

    [[nodiscard]] constexpr auto variables() const noexcept {
        return _expr.variables();
    }
    [[nodiscard]] constexpr auto coefficients() const noexcept {
        return ranges::views::transform(
            _expr.coefficients(), [scalar = _scalar](auto && coef) -> scalar_t {
                return scalar * coef;
            });
    }
    [[nodiscard]] constexpr scalar_t constant() const noexcept {
        return _scalar * _expr.constant();
    }
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_LINEAR_EXPRESSION_OPERATIONS_HPP