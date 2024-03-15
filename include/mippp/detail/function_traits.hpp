#ifndef MIPPP_DETAIL_FUCTION_TRAITS_HPP
#define MIPPP_DETAIL_FUCTION_TRAITS_HPP

namespace fhamonic {
namespace mippp {
namespace detail {

template <typename... Args>
struct pack {};

template <typename T>
struct function_traits : public function_traits<decltype(&T::operator())> {};

template <typename ClassType, typename ReturnType, typename... Args>
struct function_traits<ReturnType (ClassType::*)(Args...) const> {
    using result_type = ReturnType;
    using arg_types = pack<Args...>;
};

}  // namespace detail
}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_DETAIL_FUCTION_TRAITS_HPP