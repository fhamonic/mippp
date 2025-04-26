#ifndef MIPPP_OPTIONAL_HELPER_HPP
#define MIPPP_OPTIONAL_HELPER_HPP

#include <optional>

#define OPT(cond, ...) ((cond) ? std::make_optional(__VA_ARGS__) : std::nullopt)

namespace fhamonic {
namespace mippp {
namespace detail {

template <typename T>
struct is_optional_type : std::false_type {};

template <typename U>
struct is_optional_type<std::optional<U>> : std::true_type {};

template <typename T>
concept optional_type = is_optional_type<std::remove_cvref_t<T>>::value;

template <optional_type T>
using optional_type_value_t = typename T::value_type;

}  // namespace detail
}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_OPTIONAL_HELPER_HPP
