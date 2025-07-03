#ifndef MIPPP_MODEL_ENTITIES_HPP
#define MIPPP_MODEL_ENTITIES_HPP

// #include <flat_map>
#include <map>
#include <optional>
#include <ranges>

#include "mippp/model_concepts.hpp"

namespace fhamonic::mippp {

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// Strong types /////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename Id>
class model_entity_base {
private:
    Id _id;

public:
    template <typename T>
        requires std::constructible_from<Id, T>
    constexpr explicit model_entity_base(T t) : _id(t) {}

    constexpr Id id() const noexcept { return _id; }

    constexpr std::size_t uid() const noexcept
        requires std::integral<Id>
    {
        return static_cast<std::size_t>(_id);
    }

    friend constexpr auto operator==(const model_entity_base & a,
                                     const model_entity_base & b) noexcept {
        return a._id == b._id;
    }
    friend constexpr auto operator<(const model_entity_base & a,
                                    const model_entity_base & b) noexcept {
        return a._id < b._id;
    }
};

template <typename Id, typename Scalar>
class model_variable : public model_entity_base<Id> {
public:
    constexpr model_variable() = default;
    constexpr model_variable(model_variable && v) = default;
    constexpr model_variable(const model_variable & v) = default;

    constexpr model_variable & operator=(const model_variable &) = default;
    constexpr model_variable & operator=(model_variable &&) = default;

    template <typename T>
    constexpr explicit model_variable(T t) : model_entity_base<Id>(t) {}

    constexpr auto linear_terms() const noexcept {
        // return std::views::single(std::make_pair(*this, Scalar{1}));
        return std::views::single(
            std::pair<model_variable<Id, Scalar>, Scalar>(*this, Scalar{1}));
    }
    constexpr Scalar constant() const noexcept { return Scalar{0}; }
};

template <typename Id>
class model_constraint : public model_entity_base<Id> {
public:
    constexpr model_constraint() = default;
    constexpr model_constraint(model_constraint && v) = default;
    constexpr model_constraint(const model_constraint & v) = default;

    constexpr model_constraint & operator=(const model_constraint &) = default;
    constexpr model_constraint & operator=(model_constraint &&) = default;

    template <typename T>
    constexpr explicit model_constraint(T t) : model_entity_base<Id>(t) {}
};

///////////////////////////////////////////////////////////////////////////////
//////////////////////////// Strong types mappings ////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename Entity, typename Map>
class entity_mapping {
private:
    Map _map;

public:
    entity_mapping(Map && t) : _map(std::move(t)) {}

    double operator[](const Entity & x) const {
        return _map[static_cast<std::size_t>(x.id())];
    }
};

///////////////////////////////////////////////////////////////////////////////
/////////////////////////// Function traits detail ////////////////////////////
///////////////////////////////////////////////////////////////////////////////

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

///////////////////////////////////////////////////////////////////////////////
/////////////////////////////// Variables range ///////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <std::ranges::random_access_range Vars, typename IdLambda,
          typename... Args>
    requires std::integral<
        std::decay_t<std::invoke_result_t<IdLambda, Args...>>>
class variables_range {
protected:
    using variable = std::ranges::range_value_t<Vars>;
    using scalar = linear_expression_scalar_t<variable>;

    const Vars _variables;
    [[no_unique_address]] const IdLambda _id_lambda;

public:
    template <typename VR>
    constexpr variables_range(VR && variables) noexcept
        : _variables(std::views::all(std::forward<VR>(variables)))
        , _id_lambda() {}

    template <typename AP, typename VR, typename IL>
    constexpr variables_range(AP, VR && variables, IL && id_lambda) noexcept
        : _variables(std::views::all(std::forward<VR>(variables)))
        , _id_lambda(std::forward<IL>(id_lambda)) {}

    constexpr auto size() const noexcept {
        return std::ranges::size(_variables);
    }
    constexpr auto begin() const noexcept {
        return std::ranges::begin(_variables);
    }
    constexpr auto end() const noexcept { return std::ranges::end(_variables); }

    template <std::integral T>
    constexpr auto operator[](T i) const {
        if(static_cast<std::size_t>(i) >= this->size())
            throw std::out_of_range("variable's index out of range.");
        return begin()[static_cast<std::ranges::range_difference_t<Vars>>(i)];
    }

    constexpr auto operator()(Args... args) const {
        const auto index = static_cast<std::ranges::range_difference_t<Vars>>(
            this->_id_lambda(args...));
        if(static_cast<std::size_t>(index) >= this->size())
            throw std::out_of_range("variable's index out of range.");
        return this->begin()[index];
    }

    constexpr auto linear_terms() const noexcept {
        return std::views::transform(
            _variables, [](auto && i) { return std::make_pair(i, scalar{1}); });
    }
    constexpr scalar constant() const noexcept { return scalar{0}; }
};

template <std::ranges::random_access_range Vars, typename IdLambda,
          typename NameLambda, typename Model, typename... Args>
    requires std::integral<
                 std::decay_t<std::invoke_result_t<IdLambda, Args...>>> &&
             std::convertible_to<std::invoke_result_t<NameLambda, Args...>,
                                 std::string>
class lazily_named_variables_range
    : public variables_range<Vars, IdLambda, Args...> {
private:
    [[no_unique_address]] mutable NameLambda _name_lambda;
    std::unique_ptr<bool[]> _name_set_map;
    Model * _model;

public:
    template <typename VR, typename NL, typename M>
    constexpr lazily_named_variables_range(VR && variables, NL && name_lambda,
                                           M * model) noexcept
        : variables_range<Vars, IdLambda, Args...>(std::forward<VR>(variables))
        , _name_lambda(std::forward<NL>(name_lambda))
        , _name_set_map(std::make_unique<bool[]>(this->size()))
        , _model(model) {}

    template <typename AP, typename VR, typename IL, typename NL, typename M>
    constexpr lazily_named_variables_range(AP p, VR && variables,
                                           IL && id_lambda, NL && name_lambda,
                                           M * model) noexcept
        : variables_range<Vars, IdLambda, Args...>(
              p, std::forward<VR>(variables), std::forward<IL>(id_lambda))
        , _name_lambda(std::forward<NL>(name_lambda))
        , _name_set_map(std::make_unique<bool[]>(this->size()))
        , _model(model) {}

    constexpr auto operator()(Args... args) const {
        const auto index = static_cast<std::size_t>(this->_id_lambda(args...));
        if(index >= this->size())
            throw std::out_of_range("variable's index out of range.");
        auto && var =
            this->begin()[static_cast<std::ranges::range_difference_t<Vars>>(
                index)];
        if(!_name_set_map[index]) {
            _name_set_map[index] = true;
            _model->set_variable_name(var, _name_lambda(args...));
        }
        return var;
    }
};

template <std::ranges::viewable_range VR>
variables_range(VR &&)
    -> variables_range<std::views::all_t<VR>, std::identity, std::size_t>;

template <std::ranges::viewable_range VR, typename IL, typename... Args>
variables_range(detail::pack<Args...>, VR &&, IL &&)
    -> variables_range<std::views::all_t<VR>, IL, Args...>;

template <std::ranges::viewable_range VR, typename NL, typename M>
lazily_named_variables_range(VR &&, NL &&, M *)
    -> lazily_named_variables_range<std::views::all_t<VR>, std::identity, NL, M,
                                    std::size_t>;

template <std::ranges::viewable_range VR, typename IL, typename NL, typename M,
          typename... Args>
lazily_named_variables_range(detail::pack<Args...>, VR &&, IL &&, NL &&, M *)
    -> lazily_named_variables_range<std::views::all_t<VR>, IL, NL, M, Args...>;

///////////////////////////////////////////////////////////////////////////////
/////////////////////////////// Optional helper ///////////////////////////////
///////////////////////////////////////////////////////////////////////////////

namespace detail {

template <typename T>
struct is_optional_type : std::false_type {};

template <typename U>
struct is_optional_type<std::optional<U>> : std::true_type {};

template <typename T>
concept optional_type = is_optional_type<std::remove_cvref_t<T>>::value;

template <optional_type T>
using optional_type_value_t = typename T::value_type;

#define OPT(cond, ...) ((cond) ? std::make_optional(__VA_ARGS__) : std::nullopt)

}  // namespace detail

///////////////////////////////////////////////////////////////////////////////
////////////////////////////// Constraints range
/////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename Key, typename Constraint>
class constraints_range {
protected:
    // flat_map will be supported in GCC 15
    // const std::flat_map<Key, Constraint> _constraints_map;
    std::map<Key, Constraint> _constraints_map;

public:
    // template <typename KR, typename CR>
    // constexpr constraints_range(KR && keys, CR && constraints) noexcept
    //     : _constraints_map(std::from_range_t{},
    //                        std::views::zip(keys, constraints)) {}
    template <typename KR, typename CR>
    constexpr constraints_range(KR && keys, CR && constraints) noexcept {
        // if constexpr(std::ranges::sized_range<CR>) {
        //     _constraints_map.reserve(std::ranges::size(constraints));
        // }
        for(auto && keyval_pair : std::views::zip(keys, constraints)) {
            _constraints_map.insert(keyval_pair);
        }
    }

    constexpr auto size() const noexcept { return _constraints_map.size(); }
    constexpr auto begin() const noexcept {
        return _constraints_map.values().begin();
    }
    constexpr auto end() const noexcept {
        return _constraints_map.values().end();
    }

    constexpr auto operator()(const Key & k) const {
        return _constraints_map.at(k);
    }
};

template <typename KR, typename CR>
constraints_range(KR &&, CR &&)
    -> constraints_range<std::ranges::range_value_t<KR>,
                         std::ranges::range_value_t<CR>>;

}  // namespace fhamonic::mippp

#endif  // MIPPP_MODEL_ENTITIES_HPP