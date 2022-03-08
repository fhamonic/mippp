#ifndef MIPPP_LINEAR_TERM_HPP
#define MIPPP_LINEAR_TERM_HPP

#include <range/v3/view/single.hpp>

namespace fhamonic {
namespace mippp {

template <typename V, typename C>
class linear_term {
public:
    using var_id_t = V;
    using scalar_t = C;

private:
    var_id_t _var;
    scalar_t _coef;

public:
    constexpr linear_term(var_id_t v, scalar_t c) : _var(v), _coef(c){};

    constexpr auto variables() const noexcept {
        return ranges::views::single(_var);
    }
    constexpr auto coefficients() const noexcept {
        return ranges::views::single(_coef);
    }
    constexpr scalar_t constant() const noexcept { return scalar_t{0}; }
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_LINEAR_TERM_HPP