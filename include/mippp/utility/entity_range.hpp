#pragma once

#include <algorithm>
#include <concepts>
#include <cstddef>
#include <functional>
#include <iterator>
#include <ranges>
#include <stdexcept>
#include <string>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "mippp/detail/cartesian_product_view.hpp"
#include "mippp/detail/invoke_key.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/utility/keys_view.hpp"
#include "mippp/utility/zero.hpp"

namespace mippp {

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////// Key index ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// An index resolves a key to the position of its entity, or npos. The
// key_index customization point object picks one from the type of a key
// range: a `key_index(keys)` found by argument-dependent lookup first, then
// the built-in strategies in order. An index may instead take the entity's
// coordinates directly (lambda_index), which no key range describes.

inline constexpr std::size_t npos = static_cast<std::size_t>(-1);

template <typename I, typename Key>
concept key_index_for = requires(const I & index, const Key & key) {
    { index.position(key) } -> std::same_as<std::size_t>;
};

namespace detail {

// --- affine: iota ranges and cartesian products of them. Position is
// arithmetic on the key; only the bounds are stored.

template <typename R>
inline constexpr bool affine_keys_v = false;
template <std::integral W, std::integral B>
inline constexpr bool affine_keys_v<std::ranges::iota_view<W, B>> = true;
// the standard view where it exists, the two-operand fallback elsewhere
template <typename... Vs>
inline constexpr bool affine_keys_v<cartesian_product_view<Vs...>> =
    (affine_keys_v<Vs> && ...);

template <typename Key>
struct affine_index;

template <std::integral K>
struct affine_index<K> {
    using key_type = K;
    K front{};
    std::size_t extent = 0;

    static constexpr affine_index from_bounds(K front, K back) noexcept {
        return {front, static_cast<std::size_t>(back - front) + 1};
    }

    // any integral, so that a signed lookup on unsigned keys compares right
    template <std::integral I>
    constexpr std::size_t position(I key) const noexcept {
        if(std::cmp_less(key, front)) return npos;
        const auto d =
            static_cast<std::size_t>(key) - static_cast<std::size_t>(front);
        return d < extent ? d : npos;
    }
};

template <typename... Ks>
struct affine_index<std::tuple<Ks...>> {
    using key_type = std::tuple<Ks...>;
    std::tuple<affine_index<Ks>...> subs{};
    std::size_t extent = 0;

    template <typename T>
    static constexpr affine_index from_bounds(const T & front,
                                              const T & back) noexcept {
        return [&]<std::size_t... I>(std::index_sequence<I...>) {
            affine_index index{{affine_index<Ks>::from_bounds(
                std::get<I>(front), std::get<I>(back))...}};
            index.extent =
                (std::size_t{1} * ... * std::get<I>(index.subs).extent);
            return index;
        }(std::index_sequence_for<Ks...>{});
    }

    // row-major, the order cartesian products iterate in
    template <typename T>
        requires(std::tuple_size_v<std::remove_cvref_t<T>> == sizeof...(Ks))
    constexpr std::size_t position(const T & key) const noexcept {
        std::size_t pos = 0;
        const bool found = [&]<std::size_t... I>(std::index_sequence<I...>) {
            return (... && step<I>(pos, std::get<I>(key)));
        }(std::index_sequence_for<Ks...>{});
        return found ? pos : npos;
    }

private:
    template <std::size_t I, typename T>
    constexpr bool step(std::size_t & pos, const T & key) const noexcept {
        const auto & sub = std::get<I>(subs);
        const std::size_t p = sub.position(key);
        if(p == npos) return false;
        pos = pos * sub.extent + p;
        return true;
    }
};

// The bounds are read off the first and last elements: the standard product
// view keeps its operands private, and every affine range is sized. Reaching
// the last element is constant time on a random-access range and a walk on
// the forward-only fallback product, no more than registering the keys costs.
template <typename R>
constexpr auto make_affine_index(const R & keys) {
    using Key = std::ranges::range_value_t<R>;
    const auto n = std::ranges::size(keys);
    if(n == 0) return affine_index<Key>{};
    const auto front = std::ranges::begin(keys);
    const auto back = std::ranges::next(
        front, static_cast<std::ranges::range_difference_t<R>>(n - 1));
    return affine_index<Key>::from_bounds(*front, *back);
}

// --- table: indexed(keys, id), one position per id up to the largest.

template <typename Key, typename IdFn>
class table_index {
private:
    [[no_unique_address]] IdFn _id;
    std::vector<std::size_t> _table;

public:
    using key_type = Key;

    template <typename R>
    explicit table_index(R & keys) : _id(keys.id_fn()) {
        std::size_t pos = 0;
        for(auto && key : keys) {
            const auto id = invoke_key(_id, key);
            if(!std::in_range<std::size_t>(id))
                throw std::invalid_argument(
                    "indexed keys: id must be non-negative.");
            const auto i = static_cast<std::size_t>(id);
            if(i >= _table.size()) _table.resize(i + 1, npos);
            if(_table[i] == npos) _table[i] = pos;
            ++pos;
        }
    }

    template <typename K>
        requires key_invocable<const IdFn &, const K &>
    constexpr std::size_t position(const K & key) const {
        const auto id = invoke_key(_id, key);
        if(!std::in_range<std::size_t>(id)) return npos;
        const auto i = static_cast<std::size_t>(id);
        return i < _table.size() ? _table[i] : npos;
    }
};

// --- hash and sorted: the keys copied into a std::unordered_map, or into a
// sorted vector when they are not hashable. Duplicate keys resolve to their
// first position.

template <typename K>
concept hashable_key = std::equality_comparable<K> && requires(const K & k) {
    { std::hash<K>{}(k) } -> std::convertible_to<std::size_t>;
};

template <typename Key>
class hash_index {
private:
    std::unordered_map<Key, std::size_t> _positions;

public:
    using key_type = Key;

    template <typename R>
    explicit hash_index(R & keys) {
        if constexpr(std::ranges::sized_range<R>)
            _positions.reserve(std::ranges::size(keys));
        std::size_t pos = 0;
        for(auto && key : keys) _positions.try_emplace(key, pos++);
    }

    std::size_t position(const Key & key) const {
        const auto it = _positions.find(key);
        return it == _positions.end() ? npos : it->second;
    }
};

template <std::totally_ordered Key>
class sorted_index {
private:
    using entry = std::pair<Key, std::size_t>;
    std::vector<entry> _positions;

public:
    using key_type = Key;

    template <typename R>
    explicit sorted_index(R & keys) {
        if constexpr(std::ranges::sized_range<R>)
            _positions.reserve(std::ranges::size(keys));
        std::size_t pos = 0;
        for(auto && key : keys) _positions.emplace_back(key, pos++);
        // stable: equal keys stay in registration order, unique keeps the
        // first one
        std::ranges::stable_sort(_positions, {}, &entry::first);
        const auto duplicates =
            std::ranges::unique(_positions, {}, &entry::first);
        _positions.erase(duplicates.begin(), duplicates.end());
    }

    std::size_t position(const Key & key) const {
        const auto it =
            std::ranges::lower_bound(_positions, key, {}, &entry::first);
        return it != _positions.end() && it->first == key ? it->second : npos;
    }
};

// --- none: the range stays iterable and positionally indexable.

struct no_index {
    constexpr std::size_t position(const auto &...) const noexcept {
        return npos;
    }
};

// --- lambda: the coordinates are mapped to the position by the user's
// function, as add_variables(count, id_lambda) takes it.

template <typename IdFn>
struct lambda_index {
    [[no_unique_address]] IdFn id;

    template <typename... Args>
        requires std::invocable<const IdFn &, Args...>
    constexpr std::size_t position(Args &&... args) const {
        const auto i = std::invoke(id, std::forward<Args>(args)...);
        return std::in_range<std::size_t>(i) ? static_cast<std::size_t>(i)
                                             : npos;
    }
};

// add_variables(count): X(i) is the i-th variable
using positional_index = lambda_index<std::identity>;

// Names the variable on its first access, since coordinates cannot be
// enumerated ahead of time.
template <typename IdFn, typename NameFn, typename Model, typename Entity>
class lazily_named_index {
private:
    [[no_unique_address]] lambda_index<IdFn> _ids;
    [[no_unique_address]] NameFn _name;
    Model * _model;
    Entity _front;
    mutable std::vector<bool> _named;

public:
    lazily_named_index(IdFn id, NameFn name, Model * model, Entity front,
                       std::size_t count)
        : _ids{std::move(id)}
        , _name(std::move(name))
        , _model(model)
        , _front(front)
        , _named(count, false) {}

    template <typename... Args>
        requires std::invocable<const IdFn &, const Args &...> &&
                 std::invocable<const NameFn &, const Args &...>
    std::size_t position(const Args &... args) const {
        const std::size_t pos = _ids.position(args...);
        if(pos < _named.size() && !_named[pos]) {
            _named[pos] = true;
            using id_t = decltype(_front.id());
            _model->set_variable_name(
                Entity{static_cast<id_t>(_front.id() + static_cast<id_t>(pos))},
                std::string(std::invoke(_name, args...)));
        }
        return pos;
    }
};

// --- the customization point object

namespace key_index_cpo {

// Poison pill in the form the standard library uses for ranges::begin: a
// customization is a non-template `key_index(const range &)` found by
// argument-dependent lookup, which then beats this template on a tie.
void key_index(const auto &) = delete;

template <typename R>
concept has_adl_key_index = requires(const R & keys) {
    { key_index(keys) } -> key_index_for<std::ranges::range_value_t<R>>;
};

struct fn {
    template <typename R>
    [[nodiscard]] constexpr auto operator()(R && keys) const {
        using K = std::remove_cvref_t<R>;
        using Key = std::ranges::range_value_t<R>;
        if constexpr(has_adl_key_index<R>)
            return key_index(std::as_const(keys));
        // a name never changes how keys are found, an id does
        else if constexpr(unindexed_keys<K>)
            return (*this)(keys.base());
        else if constexpr(affine_keys_v<K>)
            return make_affine_index(keys);
        // single-pass keys cannot be walked a second time
        else if constexpr(!std::ranges::forward_range<R>)
            return no_index{};
        else if constexpr(indexed_keys<K>)
            return table_index<Key, typename K::id_fn_type>(keys);
        else if constexpr(hashable_key<Key>)
            return hash_index<Key>(keys);
        else if constexpr(std::totally_ordered<Key>)
            return sorted_index<Key>(keys);
        else
            return no_index{};
    }
};

}  // namespace key_index_cpo
}  // namespace detail

// key_index(keys): the index of a key range. A range type supplies its own
// by defining a non-template `key_index(const range &)` in its namespace,
// returning any object satisfying key_index_for<its value type>.
inline constexpr detail::key_index_cpo::fn key_index{};

///////////////////////////////////////////////////////////////////////////////
/////////////////////////////// Entity range //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

namespace detail {

template <typename I, typename... Args>
concept positionable = requires(const I & index, Args &&... args) {
    {
        index.position(std::forward<Args>(args)...)
    } -> std::same_as<std::size_t>;
};

// rows(i, j) for a tuple-keyed index
template <typename I, typename... Args>
concept keyed_by = requires { typename I::key_type; } &&
                   std::constructible_from<typename I::key_type, Args...>;

// the key an index resolves, or a type no argument converts to
struct no_key {
    no_key() = delete;
};
template <typename I>
struct index_key {
    using type = no_key;
};
template <typename I>
    requires requires { typename I::key_type; }
struct index_key<I> {
    using type = typename I::key_type;
};
template <typename I>
using index_key_t = typename index_key<I>::type;

}  // namespace detail

namespace detail {

template <typename Entity>
struct entity_from_id {
    template <typename Id>
    constexpr Entity operator()(Id id) const {
        return Entity{id};
    }
};

// Out of line so that a lookup stays small enough to be inlined into the
// loops filling constraints: with the throw inline, GCC keeps it a call.
[[noreturn]] inline void throw_index_out_of_range() {
    throw std::out_of_range("entity's index out of range.");
}
[[noreturn]] inline void throw_no_entity_for_key() {
    throw std::out_of_range("no entity for this key.");
}

}  // namespace detail

// The variables or constraints of one bulk addition: `count` entities with
// contiguous ids from `first`. Coordinates or a key resolve to an entity
// through the index; a range of variables is also a linear expression.
template <typename Entity, typename Index = detail::positional_index>
class entity_range {
private:
    using id_t = decltype(std::declval<const Entity &>().id());
    using ids_view =
        std::ranges::transform_view<std::ranges::iota_view<id_t, id_t>,
                                    detail::entity_from_id<Entity>>;

    Entity _front;
    ids_view _entities;
    [[no_unique_address]] Index _index;

    template <typename, typename>
    friend class entity_range;

public:
    constexpr entity_range(Entity front, std::size_t count, Index index = {})
        : _front(front)
        , _entities(std::ranges::iota_view<id_t, id_t>(
                        front.id(), static_cast<id_t>(
                                        front.id() + static_cast<id_t>(count))),
                    detail::entity_from_id<Entity>{})
        , _index(std::move(index)) {}
    // the same entities under another index
    template <typename I>
    constexpr entity_range(const entity_range<Entity, I> & entities,
                           Index index)
        : _front(entities._front)
        , _entities(entities._entities)
        , _index(std::move(index)) {}

    constexpr Entity front() const noexcept { return _front; }
    constexpr std::size_t size() const {
        return static_cast<std::size_t>(std::ranges::size(_entities));
    }
    constexpr auto begin() const { return std::ranges::begin(_entities); }
    constexpr auto end() const { return std::ranges::end(_entities); }

    constexpr Entity operator[](std::size_t i) const {
        if(i >= size()) detail::throw_index_out_of_range();
        return nth(i);
    }

    // rows({i, j}): a braced key cannot reach the template below
    constexpr Entity operator()(const detail::index_key_t<Index> & key) const {
        return checked(locate(key));
    }
    template <typename... Args>
    constexpr Entity operator()(Args &&... args) const {
        return checked(locate(std::forward<Args>(args)...));
    }

    constexpr auto linear_terms() const
        requires linear_expression<Entity>
    {
        using scalar = linear_expression_scalar_t<Entity>;
        return std::views::transform(_entities, [](const Entity & v) {
            return std::make_pair(v, scalar{1});
        });
    }
    constexpr zero_t constant() const noexcept
        requires linear_expression<Entity>
    {
        return {};
    }

private:
    template <typename... Args>
    constexpr std::size_t locate(Args &&... args) const {
        static_assert(
            !std::same_as<Index, detail::no_index>,
            "these entities cannot be retrieved by key: the keys are neither "
            "hashable (std::hash) nor totally ordered, or the key range is "
            "single-pass. Iterate the range, index it positionally with "
            "operator[], or build it from mippp::indexed(keys, id) with a "
            "function mapping each key to a dense non-negative integer.");
        static_assert(detail::positionable<Index, Args...> ||
                          detail::keyed_by<Index, Args...>,
                      "these arguments are neither the coordinates this index "
                      "takes nor the parts of one of its keys.");
        if constexpr(detail::positionable<Index, Args...>)
            return _index.position(std::forward<Args>(args)...);
        else
            return _index.position(
                typename Index::key_type(std::forward<Args>(args)...));
    }
    // one comparison: npos is never below size
    constexpr Entity checked(std::size_t pos) const {
        if(pos >= size()) detail::throw_no_entity_for_key();
        return nth(pos);
    }
    constexpr Entity nth(std::size_t i) const {
        return begin()[static_cast<std::ranges::range_difference_t<ids_view>>(
            i)];
    }
};

///////////////////////////////////////////////////////////////////////////////
////////////////////////////// Keyed additions ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

namespace detail {

// the model's naming member, when it has one
struct set_variable_name_fn {
    template <typename M, typename E>
    constexpr auto operator()(M & model, E entity, std::string name) const
        -> decltype(model.set_variable_name(entity, std::move(name))) {
        return model.set_variable_name(entity, std::move(name));
    }
};
struct set_constraint_name_fn {
    template <typename M, typename E>
    constexpr auto operator()(M & model, E entity, std::string name) const
        -> decltype(model.set_constraint_name(entity, std::move(name))) {
        return model.set_constraint_name(entity, std::move(name));
    }
};
inline constexpr set_variable_name_fn set_variable_name{};
inline constexpr set_constraint_name_fn set_constraint_name{};

// Called by add_variables(keys) and add_constraints(keys, ...) once the
// entities exist as a positional range: names them after the keys' name
// function when there is one, and attaches the keys' index.
template <typename Model, typename KR, typename SetName, typename Entity,
          typename Index>
auto keyed_entities(Model & model, KR & keys, SetName set_name,
                    entity_range<Entity, Index> positional) {
    if constexpr(named_keys<std::remove_cvref_t<KR>>) {
        static_assert(
            std::invocable<SetName, Model &, Entity, std::string>,
            "named(keys, ...) needs names for this kind of model entity, which "
            "this backend does not provide (see the has_named_variables and "
            "has_named_constraints concepts).");
        if constexpr(std::invocable<SetName, Model &, Entity, std::string>) {
            std::size_t pos = 0;
            for(auto && key : keys)
                set_name(model, positional[pos++],
                         std::string(invoke_key(keys.name_fn(), key)));
        }
    }
    return entity_range(positional, mippp::key_index(keys));
}

}  // namespace detail

}  // namespace mippp
