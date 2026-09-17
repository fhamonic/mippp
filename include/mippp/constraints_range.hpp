#pragma once

#include <algorithm>
#include <concepts>
#include <cstddef>
#include <functional>
#include <ranges>
#include <stdexcept>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>
#include <version>

#include "mippp/detail/cartesian_product_view.hpp"

namespace mippp {

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Indexed keys ///////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// Keys paired with a function mapping each of them to a dense non-negative
// integer id: constraints_range then resolves a key through a table sized to
// the largest id, instead of hashing or sorting the keys.
template <std::ranges::view V, typename IdFn>
class indexed_keys {
private:
    V _keys;
    [[no_unique_address]] IdFn _id;

public:
    using id_fn_type = IdFn;

    constexpr indexed_keys(V keys, IdFn id)
        : _keys(std::move(keys)), _id(std::move(id)) {}

    constexpr auto begin() { return std::ranges::begin(_keys); }
    constexpr auto end() { return std::ranges::end(_keys); }
    constexpr auto begin() const
        requires std::ranges::range<const V>
    {
        return std::ranges::begin(_keys);
    }
    constexpr auto end() const
        requires std::ranges::range<const V>
    {
        return std::ranges::end(_keys);
    }

    constexpr const IdFn & id_fn() const noexcept { return _id; }
};

// forward_range: the keys are walked once to register the constraints and
// once more to fill the table.
template <std::ranges::viewable_range R, typename F>
    requires std::ranges::forward_range<R> &&
             std::integral<std::remove_cvref_t<std::invoke_result_t<
                 const std::decay_t<F> &, std::ranges::range_reference_t<R>>>>
constexpr auto indexed(R && keys, F && id) {
    return indexed_keys<std::views::all_t<R>, std::decay_t<F>>(
        std::views::all(std::forward<R>(keys)), std::forward<F>(id));
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////// Key index strategies ////////////////////////////
///////////////////////////////////////////////////////////////////////////////

namespace detail {

inline constexpr std::size_t npos = static_cast<std::size_t>(-1);

// Every strategy exposes position(key): the rank of the key in the range the
// constraints were built from, or npos.

// --- affine: iota ranges and cartesian products of them. Position is
// arithmetic on the key, nothing is stored but the bounds.

template <typename R>
inline constexpr bool affine_keys_v = false;
template <std::integral W, std::integral B>
inline constexpr bool affine_keys_v<std::ranges::iota_view<W, B>> = true;
#ifdef __cpp_lib_ranges_cartesian_product
template <typename... Vs>
inline constexpr bool
    affine_keys_v<std::ranges::cartesian_product_view<Vs...>> =
        (affine_keys_v<Vs> && ...);
#endif
#if !(defined(__cpp_lib_ranges_cartesian_product) && \
      !defined(MIPPP_PORTABLE_RANGE_SHAPES))
template <typename V1, typename V2>
inline constexpr bool affine_keys_v<cartesian_product_view<V1, V2>> =
    affine_keys_v<V1> && affine_keys_v<V2>;
#endif

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

    static constexpr affine_index from_subs(affine_index<Ks>... s) noexcept {
        return {std::tuple(s...), (std::size_t{1} * ... * s.extent)};
    }
    template <typename T>
    static constexpr affine_index from_bounds(const T & front,
                                              const T & last) noexcept {
        return [&]<std::size_t... I>(std::index_sequence<I...>) {
            return from_subs(affine_index<Ks>::from_bounds(
                std::get<I>(front), std::get<I>(last))...);
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

template <std::integral W, std::integral B>
constexpr auto make_affine_index(const std::ranges::iota_view<W, B> & keys) {
    return affine_index<W>{*keys.begin(),
                           static_cast<std::size_t>(keys.size())};
}
#ifdef __cpp_lib_ranges_cartesian_product
// The standard view keeps its operands private: the bounds are read off its
// first and last elements instead.
template <typename... Vs>
constexpr auto make_affine_index(
    const std::ranges::cartesian_product_view<Vs...> & keys) {
    using V = std::ranges::cartesian_product_view<Vs...>;
    using Key = std::ranges::range_value_t<V>;
    const auto n = std::ranges::size(keys);
    if(n == 0) return affine_index<Key>{};
    return affine_index<Key>::from_bounds(
        *keys.begin(),
        keys.begin()[static_cast<std::ranges::range_difference_t<V>>(n - 1)]);
}
#endif
#if !(defined(__cpp_lib_ranges_cartesian_product) && \
      !defined(MIPPP_PORTABLE_RANGE_SHAPES))
template <typename V1, typename V2>
constexpr auto make_affine_index(const cartesian_product_view<V1, V2> & keys) {
    using Key = std::tuple<std::ranges::range_value_t<V1>,
                           std::ranges::range_value_t<V2>>;
    return affine_index<Key>::from_subs(make_affine_index(keys.first_base()),
                                        make_affine_index(keys.second_base()));
}
#endif

// --- table: indexed(keys, id), one std::size_t per id up to the largest

template <typename IdFn>
class table_key_index {
private:
    [[no_unique_address]] IdFn _id;
    std::vector<std::size_t> _table;

public:
    template <typename IK>
    explicit table_key_index(IK & keys) : _id(keys.id_fn()) {
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

// --- hash and sorted: for every other key type, the first strategy the key
// supports. Duplicate keys resolve to their first constraint.

template <typename K>
concept hashable_key = std::equality_comparable<K> && requires(const K & k) {
    { std::hash<K>{}(k) } -> std::convertible_to<std::size_t>;
};

template <typename K>
concept ordered_key = requires(const K & a, const K & b) {
    { a < b } -> std::convertible_to<bool>;
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

template <typename Key>
class sorted_key_index {
private:
    std::vector<std::pair<Key, std::size_t>> _positions;

    static constexpr bool key_less(const std::pair<Key, std::size_t> & a,
                                   const std::pair<Key, std::size_t> & b) {
        return a.first < b.first;
    }

public:
    template <typename R>
    explicit sorted_key_index(R & keys) {
        if constexpr(std::ranges::sized_range<R>)
            _positions.reserve(std::ranges::size(keys));
        std::size_t pos = 0;
        for(auto && key : keys) _positions.emplace_back(key, pos++);
        // stable: equivalent keys stay in registration order, so unique
        // keeps the first one
        std::stable_sort(_positions.begin(), _positions.end(), key_less);
        _positions.erase(std::unique(_positions.begin(), _positions.end(),
                                     [](const auto & a, const auto & b) {
                                         return !key_less(a, b) &&
                                                !key_less(b, a);
                                     }),
                         _positions.end());
    }

    std::size_t position(const Key & key) const {
        const auto it = std::lower_bound(
            _positions.begin(), _positions.end(), key,
            [](const auto & entry, const Key & k) { return entry.first < k; });
        if(it == _positions.end() || key < it->first) return npos;
        return it->second;
    }
};

// --- none: the range stays iterable and positionally indexable

struct no_key_index {
    template <typename R>
    constexpr explicit no_key_index(R &) noexcept {}

    template <typename K>
    constexpr std::size_t position(const K &) const noexcept {
        return npos;
    }
};

template <typename R>
inline constexpr bool is_indexed_keys_v = false;
template <typename V, typename F>
inline constexpr bool is_indexed_keys_v<indexed_keys<V, F>> = true;

template <typename R>
constexpr auto key_index_choice() {
    using Key = std::ranges::range_value_t<R>;
    if constexpr(affine_keys_v<R>)
        return std::type_identity<affine_index<Key>>{};
    // single-pass keys cannot be walked a second time
    else if constexpr(!std::ranges::forward_range<R>)
        return std::type_identity<no_key_index>{};
    else if constexpr(is_indexed_keys_v<R>)
        return std::type_identity<table_key_index<typename R::id_fn_type>>{};
    else if constexpr(hashable_key<Key>)
        return std::type_identity<hash_key_index<Key>>{};
    else if constexpr(ordered_key<Key>)
        return std::type_identity<sorted_key_index<Key>>{};
    else
        return std::type_identity<no_key_index>{};
}

template <typename R>
using key_index_t =
    typename decltype(key_index_choice<std::remove_cvref_t<R>>())::type;

template <typename Index, typename R>
constexpr Index make_key_index(R & keys) {
    if constexpr(affine_keys_v<std::remove_cvref_t<R>>)
        return make_affine_index(keys);
    else
        return Index(keys);
}

}  // namespace detail

///////////////////////////////////////////////////////////////////////////////
////////////////////////////// Constraints range //////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// The constraints of add_constraints(keys, ...): one per key, in key order,
// with contiguous ids from the first one. A key resolves to its constraint
// through the index strategy chosen from the type of the key range.
template <typename Key, typename Constraint, typename Index>
class constraints_range {
private:
    using id_t = decltype(std::declval<const Constraint &>().id());

    struct to_constraint {
        id_t offset;
        constexpr Constraint operator()(id_t i) const {
            return Constraint{static_cast<id_t>(offset + i)};
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
        : _constraints(std::ranges::iota_view<id_t, id_t>(
                           id_t{0}, static_cast<id_t>(count)),
                       to_constraint{first.id()})
        , _index(detail::make_key_index<Index>(keys)) {}

    constexpr constraints_range(const constraints_range &) = default;
    constexpr constraints_range(constraints_range &&) = default;
    constexpr constraints_range & operator=(const constraints_range &) =
        default;
    constexpr constraints_range & operator=(constraints_range &&) = default;

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
            "neither hashable (std::hash) nor ordered (operator<), or the key "
            "range is single-pass. Iterate the range, index it positionally "
            "with operator[], or build it from mippp::indexed(keys, id) with "
            "a function mapping each key to a dense non-negative integer.");
        const std::size_t pos = _index.position(key);
        if(pos >= size())
            throw std::out_of_range("no constraint for this key.");
        return begin()[static_cast<std::ranges::range_difference_t<ids_view>>(
            pos)];
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
