#ifndef MIPPP_VARS_RANGE_HPP
#define MIPPP_VARS_RANGE_HPP

#include <range/v3/view/iota.hpp>
#include <range/v3/view/repeat_n.hpp>

namespace fhamonic {
namespace mippp {

template <typename V, typename F, typename... Args>
class vars_range {
public:
    using var_id_t = typename V::var_id_t;
    using scalar_t = typename V::scalar_t;
    using Var = V;

private:
    const std::size_t _offset;
    const std::size_t _count;
    const F _id_lambda;

public:
    constexpr vars_range(std::size_t offset, std::size_t count,
                         F && id_lambda) noexcept
        : _offset(offset)
        , _count(count)
        , _id_lambda(std::forward<F>(id_lambda)) {}

    constexpr Var operator()(Args... args) const noexcept {
        const var_id_t id = static_cast<var_id_t>(_id_lambda(args...));
        assert(static_cast<std::size_t>(id) < _count);
        return Var(static_cast<var_id_t>(_offset) + id);
    }

    constexpr auto variables() const noexcept {
        return ranges::iota_view<var_id_t, var_id_t>(
            static_cast<var_id_t>(_offset),
            static_cast<var_id_t>(_offset + _count));
    }
    constexpr auto coefficients() const noexcept {
        return ranges::views::repeat_n(scalar_t{1}, _count);
    }
    constexpr scalar_t constant() const noexcept { return scalar_t{0}; }
};


    // template <typename R, typename V, typename C>
    // constexpr auto xsum(R && values, V && vars_map, C && coefs_map) const noexcept { 
    //     return linear_expression(
    //         ranges::views::transform(values, vars_map), 
    //         ranges::views::transform(values, coefs_map));
    // }

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_VARS_RANGE_HPP