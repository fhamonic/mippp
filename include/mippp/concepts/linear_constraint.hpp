#ifndef MIPPP_CONCEPTS_LINEAR_CONSTRAINT_HPP
#define MIPPP_CONCEPTS_LINEAR_CONSTRAINT_HPP

#include <concepts>
#include <ranges>

#include "mippp/concepts/linear_expression.hpp"
#include "mippp/detail/range_of.hpp"

namespace fhamonic {
namespace mippp {

// clang-format off
template <typename C>
using constraint_expression_t = typename std::remove_reference<C>::type::expression_t;

template <typename C>
using constraint_var_id_t = expression_var_id_t<constraint_expression_t<C>>;

template <typename C>
using constraint_scalar_t = expression_scalar_t<constraint_expression_t<C>>;

template <typename C>
concept linear_constraint_c =
    requires(typename std::remove_reference<C>::type c, constraint_expression_t<C>) {
    { c.expression() } -> linear_expression_c;
    { c.lower_bound() } -> std::same_as<constraint_var_id_t<C>>;
    { c.upper_bound() } -> std::same_as<constraint_scalar_t<C>>;
};
// clang-format on

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_CONCEPTS_LINEAR_CONSTRAINT_HPP