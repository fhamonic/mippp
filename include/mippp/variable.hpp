#ifndef MIPPP_VARIABLE_HPP
#define MIPPP_VARIABLE_HPP

// #include <ranges>

#include <range/v3/view/single.hpp>

namespace fhamonic {
namespace mippp {

template <typename V, typename C>
class variable {
public:
    using variable_id_t = V;
    using scalar_t = C;

private:
    variable_id_t _id;

public:
    constexpr variable(const variable & v) : _id(v._id) {};
    constexpr explicit variable(variable_id_t id) : _id(id) {};

    constexpr variable_id_t id() const noexcept { return _id; }

    constexpr auto terms() const noexcept {
        return ranges::views::single(std::make_pair(scalar_t{1}, _id));
    }
    constexpr scalar_t constant() const noexcept { return scalar_t{0}; }
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_VARIABLE_HPP