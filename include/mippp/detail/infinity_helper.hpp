#ifndef MIPPP_INFINITY_HELPER_HPP
#define MIPPP_INFINITY_HELPER_HPP

#include <limits>

namespace fhamonic {
namespace mippp {
namespace detail {

template <typename T>
constexpr auto minus_infinity_or_lowest() {
    if constexpr(std::numeric_limits<T>::has_infinity) {
        return -std::numeric_limits<T>::infinity();
    } else {
        return std::numeric_limits<T>::lowest();
    }
}

template <typename T>
constexpr auto infinity_or_max() {
    if constexpr(std::numeric_limits<T>::has_infinity) {
        return std::numeric_limits<T>::infinity();
    } else {
        return std::numeric_limits<T>::max();
    }
}

}  // namespace detail
}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_INFINITY_HELPER_HPP