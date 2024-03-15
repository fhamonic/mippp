#ifndef MIPPP_VARIABLE_HPP
#define MIPPP_VARIABLE_HPP

#include <iostream>

#include <range/v3/view/single.hpp>

namespace fhamonic {
namespace mippp {

template <typename V, typename C>
class variable {
public:
    using var_id_t = V;
    using scalar_t = C;

private:
    var_id_t _id;

public:
    constexpr explicit variable(var_id_t id) : _id(id){};

    constexpr var_id_t id() const noexcept { return _id; }

    constexpr variable(const variable & v) : _id(v._id){};

    constexpr auto variables() const noexcept {
        return ranges::views::single(_id);
    }
    constexpr auto coefficients() const noexcept {
        return ranges::views::single(scalar_t{1});
    }
    constexpr scalar_t constant() const noexcept { return scalar_t{0}; }
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_VARIABLE_HPP