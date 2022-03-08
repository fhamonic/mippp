#ifndef MIPPP_CONCEPTS_ID_VALUE_MAP_HPP
#define MIPPP_CONCEPTS_ID_VALUE_MAP_HPP

#include <concepts>

namespace fhamonic {
namespace mippp {

// clang-format off
template <typename M, typename I, typename V>
concept id_value_map = requires(M m, I id) {
    { m[id] } -> std::same_as<V>;
};
// clang-format on

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_CONCEPTS_ID_VALUE_MAP_HPP