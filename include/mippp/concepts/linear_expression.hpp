#ifndef MIPPP_CONCEPTS_LINEAR_EXPRESSION_HPP
#define MIPPP_CONCEPTS_LINEAR_EXPRESSION_HPP

#include <concepts>
#include <ranges>

#include "mippp/detail/range_of.hpp"

namespace fhamonic {
namespace mippp {

// clang-format off
template <typename E>
using expression_var_id_t = typename std::remove_reference<E>::type::var_id_t;

template <typename E>
using expression_scalar_t = typename std::remove_reference<E>::type::scalar_t;

template <typename E>
concept linear_expression_c =
    requires(typename std::remove_reference<E>::type e, expression_var_id_t<E>,
             expression_scalar_t<E>) {
    { e.variables() } -> detail::range_of<expression_var_id_t<E>>;
    { e.coefficients() } -> detail::range_of<expression_scalar_t<E>>;
    { e.constant() } -> std::same_as<expression_scalar_t<E>>;
};
// clang-format on

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_CONCEPTS_LINEAR_EXPRESSION_HPP