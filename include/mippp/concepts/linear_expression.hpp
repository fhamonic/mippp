#ifndef MIPPP_CONCEPTS_LINEAR_EXPRESSION_HPP
#define MIPPP_CONCEPTS_LINEAR_EXPRESSION_HPP

#include <concepts>
#include <ranges>

#include "mippp/detail/range_of.hpp"

namespace fhamonic {
namespace mippp {

// clang-format off
template <typename E>
concept linear_expression_c = requires(E e, typename E::var_id_t,
                                            typename E::scalar_t) {
    { e.variables() } -> detail::range_of<typename E::var_id_t>;
    { e.coeficients() } -> detail::range_of<typename E::scalar_t>;
    { e.constant() } -> std::same_as<typename E::scalar_t>;
};
// clang-format on

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_CONCEPTS_LINEAR_EXPRESSION_HPP