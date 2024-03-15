#ifndef MIPPP_CONCEPTS_LINEAR_CONSTRAINT_HPP
#define MIPPP_CONCEPTS_LINEAR_CONSTRAINT_HPP

#include <concepts>
#include <ranges>

#include "mippp/detail/range_of.hpp"

namespace fhamonic {
namespace mippp {

// clang-format off
template <typename C>
using constraint_var_id_t = typename std::remove_reference<C>::type::var_id_t;

template <typename C>
using constraint_scalar_t = typename std::remove_reference<C>::type::scalar_t;

template <typename C>
concept linear_constraint_c =
    requires(typename std::remove_reference<C>::type c, constraint_var_id_t<C>,
             constraint_scalar_t<C>) {
    { c.variables() } -> detail::range_of<constraint_var_id_t<C>>;
    { c.coefficients() } -> detail::range_of<constraint_scalar_t<C>>;
    { c.lower_bound() } -> std::same_as<constraint_scalar_t<C>>;
    { c.upper_bound() } -> std::same_as<constraint_scalar_t<C>>;
};
// clang-format on

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_CONCEPTS_LINEAR_CONSTRAINT_HPP