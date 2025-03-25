#ifndef MIPPP_MODEL_VARIABLE_HPP
#define MIPPP_MODEL_VARIABLE_HPP

// #include <ranges>

#include <range/v3/view/single.hpp>

namespace fhamonic {
namespace mippp {

template <typename V, typename C>
class model_variable {
public:
    using variable_id_t = V;
    using scalar_t = C;

private:
    variable_id_t _id;

public:
    constexpr model_variable(const model_variable & v) : _id(v._id) {};
    constexpr explicit model_variable(variable_id_t id) : _id(id) {};

    constexpr variable_id_t id() const noexcept { return _id; }

    constexpr auto linear_terms() const noexcept {
        return ranges::views::single(std::make_pair(_id, scalar_t{1}));
    }
    constexpr scalar_t constant() const noexcept { return scalar_t{0}; }
};

template <typename Arr>
class variable_mapping {
private:
    Arr arr;

public:
    variable_mapping(Arr && t) : arr(std::move(t)) {}

    double operator[](int i) const { return arr[static_cast<std::size_t>(i)]; }
    double operator[](model_variable<int, double> x) const {
        return arr[static_cast<std::size_t>(x.id())];
    }
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_MODEL_VARIABLE_HPP