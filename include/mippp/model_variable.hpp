#ifndef MIPPP_MODEL_VARIABLE_HPP
#define MIPPP_MODEL_VARIABLE_HPP

// #include <ranges>

#include <range/v3/view/iota.hpp>
#include <range/v3/view/single.hpp>

#include "mippp/detail/function_traits.hpp"

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

template <typename Vars, typename IdLambda>
class variables_range_base {
private:
    using variable = ranges::range_value_t<Vars>;
    using scalar_t = typename variable::scalar_t;

protected:
    const Vars _variables;
    const IdLambda _id_lambda;

public:
    template <typename VR, typename IL>
    constexpr variables_range_base(VR && variables, IL && id_lambda) noexcept
        : _variables(std::forward<VR>(variables))
        , _id_lambda(std::forward<IL>(id_lambda)) {}

    constexpr auto size() const noexcept { return ranges::size(_variables); }
    constexpr auto begin() const noexcept { return ranges::begin(_variables); }
    constexpr auto end() const noexcept { return ranges::end(_variables); }

    constexpr auto linear_terms() const noexcept {
        return ranges::views::transform(_variables, [](auto && i) {
            return std::make_pair(i, scalar_t{1});
        });
    }
    constexpr scalar_t constant() const noexcept { return scalar_t{0}; }
};

template <ranges::random_access_range Vars, typename IdLambda, typename... Args>
class variables_range : public variables_range_base<Vars, IdLambda> {
public:
    template <typename VR, typename IL>
    constexpr variables_range(VR && variables, IL && id_lambda) noexcept
        : variables_range_base<Vars, IdLambda>(std::forward<VR>(variables),
                                               std::forward<IL>(id_lambda)) {}

    constexpr auto operator()(Args... args) const noexcept {
        const auto index = static_cast<std::size_t>(_id_lambda(args...));
        assert(index < this.size());
        return this.begin()[index];
    }
};

template <typename VR, typename IL, typename... Args>
auto make_variables_range(detail::pack<Args...>, VR && variables,
                          IL && id_lambda) {
    return variables_range<ranges::view::all_t<VR>, std::decay_t<IL>, Args...>(
        std::forward<VR>(variables), std::forward<IL>(id_lambda));
}

template <typename Vars, typename IdLambda, typename NamingLambda,
          typename... Args>
class lazily_named_variables_range
    : public variables_range_base<Vars, IdLambda> {
private:
    mutable NamingLambda _naming_lambda;

public:
    template <typename VR, typename IL, typename NL>
    constexpr lazily_named_variables_range(VR && variables, IL && id_lambda,
                                           NL && naming_lambda) noexcept
        : variables_range_base<Vars, IdLambda>(std::forward<VR>(variables),
                                               std::forward<IL>(id_lambda))
        , _naming_lambda(std::forward<NL>(naming_lambda)) {}

    constexpr auto operator()(Args... args) const noexcept {
        const auto index = static_cast<std::size_t>(_id_lambda(args...));
        assert(index < this.size());
        auto && var = this.begin()[index];
        _naming_lambda(var, args...);
        return var;
    }
};

template <typename VR, typename IL, typename NL, typename... Args>
auto make_lazily_named_variables_range(detail::pack<Args...>, VR && variables,
                                       IL && id_lambda, NL && naming_lambda) {
    return lazily_named_variables_range<
        ranges::view::all_t<VR>, std::decay_t<IL>, std::decay_t<NL>, Args...>(
        std::forward<VR>(variables), std::forward<IL>(id_lambda),
        std::forward<NL>(naming_lambda));
}

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_MODEL_VARIABLE_HPP