#ifndef FUCTION_TRAITS_HPP
#define FUCTION_TRAITS_HPP

template<typename... Args>
struct pack {};

template <typename T>
struct function_traits
    : public function_traits<decltype(&T::operator())>
{};
template <typename ClassType, typename ReturnType, typename... Args>
struct function_traits<ReturnType(ClassType::*)(Args...) const> {
    using result_type = ReturnType;
    using arg_types = pack<Args...>;
};

#endif //FUCTION_TRAITS_HPP