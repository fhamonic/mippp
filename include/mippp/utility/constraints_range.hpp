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
#include "mippp/utility/keys_view.hpp"

namespace mippp {

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////// Key index ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// An index resolves a key to its position in the key range, or npos. The
// key_index customization point object picks one from the type of the key
// range: a `key_index(keys)` found by argument-dependent lookup first, then
// the built-in strategies in order.

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
    K first{};
    std::size_t extent = 0;

    static constexpr affine_index from_bounds(K front, K last) noexcept {
        return {front, static_cast<std::size_t>(last - front) + 1};
    }

    constexpr std::size_t position(K key) const noexcept {
        if(key < first) return npos;
        const auto d = static_cast<std::size_t>(key - first);
        return d < extent ? d : npos;
    }
};

template <typename... Ks>
struct affine_index<std::tuple<Ks...>> {
    std::tuple<affine_index<Ks>...> subs{};
    std::size_t extent = 0;

    template <typename T>
    static constexpr affine_index from_bounds(const T & front,
                                              const T & last) noexcept {
        return [&]<std::size_t... I>(std::index_sequence<I...>) {
            affine_index index{{affine_index<Ks>::from_bounds(
                std::get<I>(front), std::get<I>(last))...}};
            index.extent =
                (std::size_t{1} * ... * std::get<I>(index.subs).extent);
            return index;
        }(std::index_sequence_for<Ks...>{});
    }

    // row-major, the order cartesian products iterate in
    template <typename T>
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
    const auto first = std::ranges::begin(keys);
    const auto last = std::ranges::next(
        first, static_cast<std::ranges::range_difference_t<R>>(n - 1));
    return affine_index<Key>::from_bounds(*first, *last);
}

// --- table: indexed(keys, id), one position per id up to the largest.

template <typename IdFn>
class table_key_index {
private:
    [[no_unique_address]] IdFn _id;
    std::vector<std::size_t> _table;

public:
    template <typename R>
    explicit table_key_index(R & keys) : _id(keys.id_fn()) {
        std::size_t pos = 0;
        for(auto && key : keys) {
            const auto id = std::invoke(_id, key);
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
    constexpr std::size_t position(const K & key) const {
        const auto id = std::invoke(_id, key);
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
class hash_key_index {
private:
    std::unordered_map<Key, std::size_t> _positions;

public:
    template <typename R>
    explicit hash_key_index(R & keys) {
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
class sorted_key_index {
private:
    using entry = std::pair<Key, std::size_t>;
    std::vector<entry> _positions;

public:
    template <typename R>
    explicit sorted_key_index(R & keys) {
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

struct no_key_index {
    template <typename K>
    constexpr std::size_t position(const K &) const noexcept {
        return npos;
    }
};

// --- the customization point object

namespace key_index_cpo {

void key_index() = delete;  // argument-dependent lookup only

template <typename R>
concept has_adl_key_index = requires(R & keys) {
    { key_index(keys) } -> key_index_for<std::ranges::range_value_t<R>>;
};

struct fn {
    template <typename R>
    [[nodiscard]] constexpr auto operator()(R & keys) const {
        using K = std::remove_cv_t<R>;
        using Key = std::ranges::range_value_t<R>;
        if constexpr(has_adl_key_index<R>) return key_index(keys);
        // a name never changes how keys are found, an id does
        else if constexpr(unindexed_keys<K>)
            return (*this)(keys.base());
        else if constexpr(affine_keys_v<K>)
            return make_affine_index(keys);
        // single-pass keys cannot be walked a second time
        else if constexpr(!std::ranges::forward_range<R>)
            return no_key_index{};
        else if constexpr(indexed_keys<K>)
            return table_key_index<typename K::id_fn_type>(keys);
        else if constexpr(hashable_key<Key>)
            return hash_key_index<Key>(keys);
        else if constexpr(std::totally_ordered<Key>)
            return sorted_key_index<Key>(keys);
        else
            return no_key_index{};
    }
};

}  // namespace key_index_cpo

template <typename R>
using key_index_t =
    decltype(key_index_cpo::fn{}(std::declval<std::remove_reference_t<R> &>()));

///////////////////////////////////////////////////////////////////////////////
////////////////////////////// Constraint names ///////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename Model, typename Constraint>
concept can_name_constraints = requires(
    Model & m, Constraint c, std::string s) { m.set_constraint_name(c, s); };

// Called by add_constraints once the rows exist: names them after the keys'
// name function, when there is one.
template <typename Model, typename KR, typename Constraint>
constexpr void name_constraints(Model & model, KR & keys, Constraint first) {
    if constexpr(named_keys<std::remove_cvref_t<KR>>) {
        static_assert(
            can_name_constraints<Model, Constraint>,
            "named(keys, ...) needs constraint names, which this backend "
            "does not support (see the has_named_constraints concept).");
        if constexpr(can_name_constraints<Model, Constraint>) {
            auto id = first.id();
            for(auto && key : keys)
                model.set_constraint_name(
                    Constraint{id++},
                    std::string(std::invoke(keys.name_fn(), key)));
        }
    }
}

}  // namespace detail

// key_index(keys): the index of a key range. A range type supplies its own
// by defining a `key_index(range &)` in its namespace, returning any object
// satisfying key_index_for<its value type>.
inline constexpr detail::key_index_cpo::fn key_index{};

///////////////////////////////////////////////////////////////////////////////
////////////////////////////// Constraints range //////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// The constraints of add_constraints(keys, ...): one per key, in key order,
// with contiguous ids from the first one. A key resolves to its constraint
// through the index chosen for the key range.
template <typename Key, typename Constraint, typename Index>
class constraints_range {
private:
    using id_t = decltype(std::declval<const Constraint &>().id());
    struct to_constraint {
        constexpr Constraint operator()(id_t id) const {
            return Constraint{id};
        }
    };
    using ids_view =
        std::ranges::transform_view<std::ranges::iota_view<id_t, id_t>,
                                    to_constraint>;

    ids_view _constraints;
    [[no_unique_address]] Index _index;

public:
    template <typename KR>
    constexpr constraints_range(KR && keys, Constraint first, std::size_t count)
        : _constraints(
              std::ranges::iota_view<id_t, id_t>(
                  first.id(),
                  static_cast<id_t>(first.id() + static_cast<id_t>(count))),
              to_constraint{})
        , _index(mippp::key_index(keys)) {}

    constexpr std::size_t size() const {
        return static_cast<std::size_t>(std::ranges::size(_constraints));
    }
    constexpr auto begin() const { return std::ranges::begin(_constraints); }
    constexpr auto end() const { return std::ranges::end(_constraints); }

    constexpr Constraint operator[](std::size_t i) const {
        if(i >= size())
            throw std::out_of_range("constraint's index out of range.");
        return begin()[static_cast<std::ranges::range_difference_t<ids_view>>(
            i)];
    }

    constexpr Constraint operator()(const Key & key) const {
        static_assert(
            !std::same_as<Index, detail::no_key_index>,
            "these constraints cannot be retrieved by key: the keys are "
            "neither hashable (std::hash) nor totally ordered, or the "
            "key range is single-pass. Iterate the range, index it "
            "positionally with operator[], or build it from "
            "mippp::indexed(keys, id) with a function mapping each key to a "
            "dense non-negative integer.");
        const std::size_t pos = _index.position(key);
        if(pos == npos) throw std::out_of_range("no constraint for this key.");
        return (*this)[pos];
    }
    // rows(i, j) for tuple keys
    template <typename... Args>
        requires(sizeof...(Args) >= 2) &&
                std::constructible_from<Key, Args &&...>
    constexpr Constraint operator()(Args &&... args) const {
        return (*this)(Key(std::forward<Args>(args)...));
    }
};

template <typename KR, typename Constraint>
constraints_range(KR &&, Constraint, std::size_t)
    -> constraints_range<std::ranges::range_value_t<KR>, Constraint,
                         detail::key_index_t<KR>>;

}  // namespace mippp
