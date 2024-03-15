#ifndef MIPPP_DETAIL_RANGE_OF_HPP
#define MIPPP_DETAIL_RANGE_OF_HPP

#include <concepts>
#include <ranges>

namespace fhamonic {
namespace mippp {
namespace detail {

template <typename T, typename V>
concept range_of =
    std::ranges::range<T> && std::same_as<std::ranges::range_value_t<T>, V>;

template <typename T, typename V>
concept random_access_range_of = std::ranges::random_access_range<T> &&
    std::same_as<std::ranges::range_value_t<T>, V>;

template <typename T, typename V>
concept contiguous_range_of = std::ranges::contiguous_range<T> &&
    std::same_as<std::ranges::range_value_t<T>, V>;

}  // namespace detail
}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_DETAIL_RANGE_OF_HPP