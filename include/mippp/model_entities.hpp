#ifndef MIPPP_MODEL_ENTITIES_HPP
#define MIPPP_MODEL_ENTITIES_HPP

// #include <ranges>

#include <range/v3/view/iota.hpp>
#include <range/v3/view/single.hpp>

#include "mippp/detail/function_traits.hpp"

namespace fhamonic {
namespace mippp {

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// Strong types /////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename I, typename C>
class model_entity_base {
public:
    using id_t = I;
    using scalar_t = C;

private:
    id_t _id;

public:
    constexpr model_entity_base(const model_entity_base & v) : _id(v._id) {};

    template <typename T>
        requires std::constructible_from<I, T>
    constexpr explicit model_entity_base(T t) : _id(t) {}

    constexpr id_t id() const noexcept { return _id; }

    constexpr std::size_t uid() const noexcept
        requires std::integral<id_t>
    {
        return static_cast<std::size_t>(_id);
    }

    friend constexpr auto operator==(const model_entity_base & a,
                                     const model_entity_base & b) noexcept {
        return a._id == b._id;
    }
};

template <typename I, typename C>
class model_variable : public model_entity_base<I, C> {
public:
    using id_t = I;
    using scalar_t = C;

public:
    constexpr model_variable(const model_variable & v)
        : model_entity_base<I, C>(v) {}

    template <typename T>
    constexpr explicit model_variable(T t) : model_entity_base<I, C>(t) {}

    constexpr auto linear_terms() const noexcept {
        return ranges::views::single(std::make_pair(*this, scalar_t{1}));
    }
    constexpr scalar_t constant() const noexcept { return scalar_t{0}; }
};

template <typename I, typename C>
class model_constraint : public model_entity_base<I, C> {
public:
    using id_t = I;
    using scalar_t = C;

public:
    constexpr model_constraint(const model_constraint & v)
        : model_entity_base<I, C>(v) {}

    template <typename T>
    constexpr explicit model_constraint(T t) : model_entity_base<I, C>(t) {}
};

///////////////////////////////////////////////////////////////////////////////
//////////////////////////// Strong types mappings ////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename Map>
class variable_mapping {
private:
    Map _map;

public:
    variable_mapping(Map && t) : _map(std::move(t)) {}

    template <typename I, typename C>
    double operator[](const model_variable<I, C> & x) const {
        return _map[static_cast<std::size_t>(x.id())];
    }
};

template <typename Map>
class constraint_mapping {
private:
    Map _map;

public:
    constraint_mapping(Map && t) : _map(std::move(t)) {}

    template <typename I, typename C>
    double operator[](const model_constraint<I, C> & x) const {
        return _map[static_cast<std::size_t>(x.id())];
    }
};

///////////////////////////////////////////////////////////////////////////////
/////////////////////////////// Entities range ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename Vars>
class variables_range {
private:
    using variable = ranges::range_value_t<Vars>;
    using scalar_t = typename variable::scalar_t;

protected:
    const Vars _variables;

public:
    template <typename VR>
    constexpr variables_range(VR && variables) noexcept
        : _variables(std::forward<VR>(variables)) {}

    constexpr auto size() const noexcept { return ranges::size(_variables); }
    constexpr auto begin() const noexcept { return ranges::begin(_variables); }
    constexpr auto end() const noexcept { return ranges::end(_variables); }

    template <std::integral T>
    constexpr auto operator[](T i) const {
        if(static_cast<std::size_t>(i) >= this->size())
            throw std::out_of_range("variable's index out of range.");
        return begin()[static_cast<std::ranges::range_difference_t<Vars>>(i)];
    }

    constexpr auto linear_terms() const noexcept {
        return ranges::views::transform(_variables, [](auto && i) {
            return std::make_pair(i, scalar_t{1});
        });
    }
    constexpr scalar_t constant() const noexcept { return scalar_t{0}; }
};

template <ranges::viewable_range VR>
auto make_variables_range(VR && variables) {
    return variables_range<ranges::view::all_t<VR>>(
        ranges::view::all(variables));
}

template <ranges::random_access_range Vars, typename IdLambda, typename... Args>
    requires std::integral<std::invoke_result_t<IdLambda, Args...>>
class indexed_variables_range : public variables_range<Vars> {
protected:
    const IdLambda _id_lambda;

public:
    template <typename VR, typename IL>
    constexpr indexed_variables_range(VR && variables, IL && id_lambda) noexcept
        : variables_range<Vars>(std::forward<VR>(variables))
        , _id_lambda(std::forward<IL>(id_lambda)) {}

    constexpr auto operator()(Args... args) const {
        const auto index = static_cast<std::ranges::range_difference_t<Vars>>(
            this->_id_lambda(args...));
        if(static_cast<std::size_t>(index) >= this->size())
            throw std::out_of_range("variable's index out of range.");
        return this->begin()[index];
    }
};

template <ranges::viewable_range VR, typename IL, typename... Args>
auto make_indexed_variables_range(detail::pack<Args...>, VR && variables,
                                  IL && id_lambda) {
    return indexed_variables_range<ranges::view::all_t<VR>, std::decay_t<IL>,
                                   Args...>(ranges::view::all(variables),
                                            std::forward<IL>(id_lambda));
}

template <ranges::random_access_range Vars, typename IdLambda,
          typename NamingLambda, typename... Args>
    requires std::integral<std::invoke_result_t<IdLambda, Args...>> &&
             std::convertible_to<std::invoke_result_t<NamingLambda, Args...>,
                                 std::string>
class lazily_named_variables_range
    : public indexed_variables_range<Vars, IdLambda> {
private:
    mutable NamingLambda _naming_lambda;

public:
    template <typename VR, typename IL, typename NL>
    constexpr lazily_named_variables_range(VR && variables, IL && id_lambda,
                                           NL && naming_lambda) noexcept
        : indexed_variables_range<Vars, IdLambda>(std::forward<VR>(variables),
                                                  std::forward<IL>(id_lambda))
        , _naming_lambda(std::forward<NL>(naming_lambda)) {}

    constexpr auto operator()(Args... args) const noexcept {
        const auto index = static_cast<std::ranges::range_difference_t<Vars>>(
            this->_id_lambda(args...));
        assert(static_cast<std::size_t>(index) < this->size());
        auto && var = this->begin()[index];
        _naming_lambda(var, args...);
        return var;
    }
};

template <ranges::viewable_range VR, typename IL, typename NL, typename... Args>
auto make_lazily_named_variables_range(detail::pack<Args...>, VR && variables,
                                       IL && id_lambda, NL && naming_lambda) {
    return lazily_named_variables_range<
        ranges::view::all_t<VR>, std::decay_t<IL>, std::decay_t<NL>, Args...>(
        ranges::view::all(variables), std::forward<IL>(id_lambda),
        std::forward<NL>(naming_lambda));
}

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_MODEL_ENTITIES_HPP