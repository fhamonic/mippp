#pragma once

#include <concepts>
#include <memory>
#include <tuple>
#include <type_traits>
#include <utility>

namespace mippp {

///////////////////////////////////////////////////////////////////////////////
///////////////////////////// Mapping concepts ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename Map, typename Key>
concept mapping = requires(Map map, Key key) { map[key]; };

template <typename Map, typename Key>
    requires mapping<Map, Key>
using mapped_reference_t =
    decltype(std::declval<Map>()[std::declval<Key>()]);

template <typename Map, typename Key>
    requires mapping<Map, Key>
using mapped_const_reference_t =
    decltype(std::declval<std::add_const_t<Map>>()[std::declval<Key>()]);

template <typename Map, typename Key>
    requires mapping<Map, Key>
using mapped_value_t = std::decay_t<mapped_const_reference_t<Map, Key>>;

template <typename Map, typename Key>
concept input_mapping =
    mapping<Map, Key> && !std::same_as<mapped_value_t<Map, Key>, void>;

template <typename Map, typename Key, typename Value>
concept input_mapping_of =
    mapping<Map, Key> && std::same_as<mapped_value_t<Map, Key>, Value>;

template <typename Map, typename Key>
concept output_mapping =
    input_mapping<Map, Key> &&
    requires(Map map, Key key, mapped_value_t<Map, Key> value) {
        {
            map[key] = value
        } -> std::same_as<
            std::add_lvalue_reference_t<mapped_reference_t<Map, Key>>>;
    };

template <typename Map, typename Key, typename Value>
concept output_mapping_of = output_mapping<Map, Key> &&
                            std::same_as<mapped_value_t<Map, Key>, Value>;

template <typename Map, typename Key>
concept contiguous_mapping =
    input_mapping<Map, Key> && std::integral<Key> && requires(Map & map) {
        {
            map.data()
        } -> std::same_as<std::add_pointer_t<mapped_value_t<Map, Key>>>;
    };

template <typename Map, typename Key, typename Value>
concept contiguous_mapping_of =
    contiguous_mapping<Map, Key> &&
    std::same_as<mapped_value_t<Map, Key>, Value>;

///////////////////////////////////////////////////////////////////////////////
///////////////////////////// Mapping views ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

struct mapping_view_base {};

template <typename T>
inline constexpr bool enable_mapping_view =
    std::derived_from<T, mapping_view_base>;

template <typename Map, typename Key>
concept mapping_view =
    mapping<Map, Key> && std::movable<Map> && enable_mapping_view<Map>;

namespace detail {

// The three storage protocols a mapping can expose. Constness of `Map` is
// the probe's: test `const M` to know what a deep-const access would do.
template <typename Map, typename Key>
concept mapping_subscriptable =
    requires(Map & map, Key && key) { map[std::forward<Key>(key)]; } ||
    requires(Map & map, Key && key) { map(std::forward<Key>(key)); } ||
    requires(Map & map, Key && key) { map.at(std::forward<Key>(key)); };

// Single point of dispatch between the three storage protocols a mapping
// can expose. Constness and value category of `map` are the caller's: each
// requires-clause tests exactly the expression the branch evaluates.
template <typename Map, typename Key>
constexpr decltype(auto) mapping_subscript(Map & map, Key && key) {
    if constexpr(requires { map[std::forward<Key>(key)]; })
        return map[std::forward<Key>(key)];
    else if constexpr(requires { map(std::forward<Key>(key)); })
        return map(std::forward<Key>(key));
    else if constexpr(requires { map.at(std::forward<Key>(key)); })
        return map.at(std::forward<Key>(key));
    else
        static_assert(false,
                      "MIP++: this type cannot be used as a mapping with "
                      "this key; it must provide operator[](key), "
                      "operator()(key) or at(key).");
}

// std::ranges-style movable-box: makes a non-assignable Map (typically a
// capturing lambda) assignable through destroy + reconstruct, so that the
// views owning one stay std::movable. The reconstruct path is only enabled
// when the move cannot throw, so a failed assignment can never leave the
// box destroyed.
template <typename T>
    requires std::move_constructible<T> && std::is_object_v<T>
class movable_box {
private:
    [[no_unique_address]] T _value;

public:
    constexpr movable_box()
        requires std::default_initializable<T>
    = default;
    constexpr movable_box(T && t) noexcept(
        std::is_nothrow_move_constructible_v<T>)
        : _value(std::move(t)) {}
    constexpr movable_box(const T & t)
        requires std::copy_constructible<T>
        : _value(t) {}

    constexpr movable_box(const movable_box &) = default;
    constexpr movable_box(movable_box &&) = default;

    constexpr movable_box & operator=(movable_box && other) noexcept(
        std::movable<T> ? std::is_nothrow_move_assignable_v<T> : true)
        requires std::movable<T> || std::is_nothrow_move_constructible_v<T>
    {
        if constexpr(std::movable<T>) {
            _value = std::move(other._value);
        } else {
            if(this != std::addressof(other)) {
                std::destroy_at(std::addressof(_value));
                std::construct_at(std::addressof(_value),
                                  std::move(other._value));
            }
        }
        return *this;
    }
    constexpr movable_box & operator=(const movable_box & other) noexcept(
        std::copyable<T> ? std::is_nothrow_copy_assignable_v<T> : false)
        requires std::copyable<T> ||
                 (std::copy_constructible<T> &&
                  std::is_nothrow_move_constructible_v<T>)
    {
        if constexpr(std::copyable<T>) {
            _value = other._value;
        } else {
            if(this != std::addressof(other)) {
                T tmp(other._value);
                std::destroy_at(std::addressof(_value));
                std::construct_at(std::addressof(_value), std::move(tmp));
            }
        }
        return *this;
    }

    constexpr T & operator*() & noexcept { return _value; }
    constexpr const T & operator*() const & noexcept { return _value; }
    constexpr T && operator*() && noexcept { return std::move(_value); }
};

}  // namespace detail

// Reference view: shallow const, like std::ranges::ref_view — constness of
// the access is carried by Map itself (mapping_ref_view<const M> reads
// const, mapping_ref_view<M> reads mutable, through a const view object).
template <typename Map>
    requires std::is_object_v<Map>
class mapping_ref_view : public mapping_view_base {
private:
    Map * _map;

    static void bindable_test(Map &);
    static void bindable_test(Map &&) = delete;

public:
    template <typename T>
        requires(!std::same_as<std::remove_cvref_t<T>, mapping_ref_view>) &&
                std::convertible_to<T, Map &> &&
                requires { bindable_test(std::declval<T>()); }
    constexpr mapping_ref_view(T && t) noexcept(
        noexcept(static_cast<Map &>(std::declval<T>())))
        : _map(std::addressof(static_cast<Map &>(std::forward<T>(t)))) {}

    constexpr Map & base() const noexcept { return *_map; }

    template <typename Key>
    [[nodiscard]] constexpr decltype(auto) operator[](Key && key) const {
        return detail::mapping_subscript(*_map, std::forward<Key>(key));
    }

    [[nodiscard]] constexpr auto data() const
        requires std::is_pointer_v<decltype(std::declval<Map &>().data())>
    {
        return _map->data();
    }
};

template <typename Map>
mapping_ref_view(Map &) -> mapping_ref_view<Map>;

// Owning view: deep const, like std::ranges::owning_view. Constructible
// from rvalues only; storage goes through movable_box so a view owning a
// capturing lambda remains std::movable.
template <typename Map>
    requires std::move_constructible<Map> && std::is_object_v<Map>
class mapping_owning_view : public mapping_view_base {
private:
    [[no_unique_address]] detail::movable_box<Map> _map;

public:
    constexpr mapping_owning_view()
        requires std::default_initializable<Map>
    = default;
    constexpr mapping_owning_view(Map && map) noexcept(
        std::is_nothrow_move_constructible_v<Map>)
        : _map(std::move(map)) {}

    constexpr mapping_owning_view(const mapping_owning_view &) = default;
    constexpr mapping_owning_view(mapping_owning_view &&) = default;
    constexpr mapping_owning_view & operator=(const mapping_owning_view &) =
        default;
    constexpr mapping_owning_view & operator=(mapping_owning_view &&) =
        default;

    constexpr Map & base() & noexcept { return *_map; }
    constexpr const Map & base() const & noexcept { return *_map; }
    constexpr Map && base() && noexcept { return *std::move(_map); }

    template <typename Key>
    [[nodiscard]] constexpr decltype(auto) operator[](Key && key) {
        return detail::mapping_subscript(*_map, std::forward<Key>(key));
    }
    template <typename Key>
    [[nodiscard]] constexpr decltype(auto) operator[](Key && key) const {
        return detail::mapping_subscript(*_map, std::forward<Key>(key));
    }

    [[nodiscard]] constexpr auto data()
        requires std::is_pointer_v<decltype(std::declval<Map &>().data())>
    {
        return (*_map).data();
    }
    [[nodiscard]] constexpr auto data() const
        requires std::is_pointer_v<
            decltype(std::declval<const Map &>().data())>
    {
        return (*_map).data();
    }
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////// views::mapping_all //////////////////////////////
///////////////////////////////////////////////////////////////////////////////

namespace views {

namespace detail {

template <typename Map>
concept can_mapping_ref_view =
    requires { mapping_ref_view{std::declval<Map>()}; };

}  // namespace detail

struct mapping_all_fn {
private:
    template <typename Map>
    static constexpr bool pass_through =
        std::movable<std::decay_t<Map>> &&
        enable_mapping_view<std::decay_t<Map>> &&
        std::constructible_from<std::decay_t<Map>, Map>;

    template <typename Map>
    static consteval bool is_noexcept() {
        if constexpr(pass_through<Map>)
            return std::is_nothrow_constructible_v<std::decay_t<Map>, Map>;
        else if constexpr(detail::can_mapping_ref_view<Map>)
            return noexcept(mapping_ref_view{std::declval<Map>()});
        else
            return noexcept(mapping_owning_view{std::declval<Map>()});
    }

public:
    template <typename Map>
    [[nodiscard]] constexpr auto operator()(Map && map) const
        noexcept(is_noexcept<Map>()) {
        if constexpr(pass_through<Map>)
            return std::decay_t<Map>(std::forward<Map>(map));
        else if constexpr(detail::can_mapping_ref_view<Map>)
            return mapping_ref_view{std::forward<Map>(map)};
        else
            return mapping_owning_view{std::forward<Map>(map)};
    }
};

inline constexpr mapping_all_fn mapping_all{};

template <typename Map>
using mapping_all_t = decltype(mapping_all(std::declval<Map>()));

///////////////////////////////////////////////////////////////////////////////
///////////////////////////// Utility mappings ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename F>
[[nodiscard]] constexpr auto map(F && f) {
    return mapping_owning_view<std::decay_t<F>>(
        std::decay_t<F>(std::forward<F>(f)));
}

struct true_map : public mapping_view_base {
    [[nodiscard]] constexpr bool operator[](const auto &) const noexcept {
        return true;
    }
};

struct false_map : public mapping_view_base {
    [[nodiscard]] constexpr bool operator[](const auto &) const noexcept {
        return false;
    }
};

struct identity_map : public mapping_view_base {
    template <typename T>
    [[nodiscard]] constexpr auto operator[](T && e) const
        noexcept(std::is_nothrow_constructible_v<std::decay_t<T>, T>) {
        return std::forward<T>(e);
    }
};

template <std::size_t... I>
struct element_map : public mapping_view_base {
private:
    template <std::size_t First, std::size_t... Rest, typename T>
    static constexpr decltype(auto) get_chain(T && e) noexcept {
        if constexpr(sizeof...(Rest) == 0) {
            return std::get<First>(std::forward<T>(e));
        } else {
            return get_chain<Rest...>(std::get<First>(std::forward<T>(e)));
        }
    }

public:
    template <typename T>
    [[nodiscard]] constexpr decltype(auto) operator[](T && e) const noexcept {
        return get_chain<I...>(std::forward<T>(e));
    }
};

}  // namespace views

}  // namespace mippp
