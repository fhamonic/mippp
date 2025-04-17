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

template <typename Var, typename IL, typename... Args>
class variables_range_base {
public:
    using variable_id_t = typename Var::variable_id_t;
    using scalar_t = typename Var::scalar_t;

protected:
    const std::size_t _offset;
    const std::size_t _count;
    const IL _id_lambda;

public:
    constexpr variables_range_base(std::size_t offset, std::size_t count,
                                   IL && id_lambda) noexcept
        : _offset(offset)
        , _count(count)
        , _id_lambda(std::forward<IL>(id_lambda)) {}

    constexpr auto ids() const noexcept {
        return ranges::views::iota(
            static_cast<variable_id_t>(_offset),
            static_cast<variable_id_t>(_offset + _count));
    }
    constexpr auto linear_terms() const noexcept {
        return ranges::views::transform(
            ids(), [](auto && i) { return std::make_pair(i, scalar_t{1}); });
    }
    constexpr scalar_t constant() const noexcept { return scalar_t{0}; }
};

template <typename Var, typename IL, typename... Args>
class variables_range : public variables_range_base<Var, IL, Args...> {
public:
    using variable_id_t = typename Var::variable_id_t;
    using scalar_t = typename Var::scalar_t;

public:
    constexpr variables_range(std::size_t offset, std::size_t count,
                              IL && id_lambda) noexcept
        : variables_range_base<Var, IL, Args...>(offset, count,
                                                 std::forward<IL>(id_lambda)) {}

    constexpr Var operator()(Args... args) const noexcept {
        const variable_id_t id =
            static_cast<variable_id_t>(_id_lambda(args...));
        assert(static_cast<std::size_t>(id) < this._count);
        return Var(static_cast<variable_id_t>(this._offset) + id);
    }
};

template <typename Var, typename IL, typename NL, typename... Args>
class lazily_named_variables_range
    : public variables_range_base<Var, IL, Args...> {
public:
    using variable_id_t = typename Var::variable_id_t;
    using scalar_t = typename Var::scalar_t;

private:
    mutable NL _naming_lambda;

public:
    constexpr lazily_named_variables_range(std::size_t offset,
                                           std::size_t count, IL && id_lambda,
                                           NL && naming_lambda) noexcept
        : variables_range_base<Var, IL, Args...>(offset, count,
                                                 std::forward<IL>(id_lambda))
        , _naming_lambda(std::forward<NL>(naming_lambda)) {}

    constexpr Var operator()(Args... args) const noexcept {
        const variable_id_t id =
            static_cast<variable_id_t>(_id_lambda(args...));
        assert(static_cast<std::size_t>(id) < this._count);
        const Var var(static_cast<variable_id_t>(this._offset) + id);
        _naming_lambda(var, args...);
        return var;
    }
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_MODEL_VARIABLE_HPP