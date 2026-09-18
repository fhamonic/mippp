#pragma once

#include <concepts>
#include <ranges>
#include <string>
#include <type_traits>
#include <utility>

#include "mippp/detail/invoke_key.hpp"

namespace mippp {

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Keys view //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

namespace detail {
struct absent {};
}  // namespace detail

// A key range carrying up to two functions of the key: an id, mapping each
// key to a dense non-negative integer the lookup then goes through, and a
// name, given to each constraint as it is added.
template <std::ranges::view V, typename IdFn = detail::absent,
          typename NameFn = detail::absent>
class keys_view {
private:
    V _keys;
    [[no_unique_address]] IdFn _id;
    [[no_unique_address]] NameFn _name;

public:
    using base_type = V;
    using id_fn_type = IdFn;
    using name_fn_type = NameFn;
    static constexpr bool has_id = !std::same_as<IdFn, detail::absent>;
    static constexpr bool has_name = !std::same_as<NameFn, detail::absent>;

    constexpr keys_view(V keys, IdFn id, NameFn name)
        : _keys(std::move(keys)), _id(std::move(id)), _name(std::move(name)) {}

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

    constexpr const V & base() const noexcept { return _keys; }
    constexpr const IdFn & id_fn() const noexcept { return _id; }
    constexpr const NameFn & name_fn() const noexcept { return _name; }
};

namespace detail {

template <typename R>
inline constexpr bool is_keys_view_v = false;
template <typename V, typename I, typename N>
inline constexpr bool is_keys_view_v<keys_view<V, I, N>> = true;

template <typename R>
concept indexed_keys = is_keys_view_v<R> && R::has_id;
template <typename R>
concept unindexed_keys = is_keys_view_v<R> && !R::has_id;
template <typename R>
concept named_keys = is_keys_view_v<R> && R::has_name;

template <typename R, typename F>
concept key_id_fn = std::integral<std::remove_cvref_t<key_invoke_result_t<
    const std::decay_t<F> &, std::ranges::range_reference_t<R>>>>;
template <typename R, typename F>
concept key_name_fn =
    std::convertible_to<key_invoke_result_t<const std::decay_t<F> &,
                                            std::ranges::range_reference_t<R>>,
                        std::string>;

template <typename R, typename Id, typename Name>
constexpr auto make_keys_view(R && keys, Id && id, Name && name) {
    return keys_view<std::views::all_t<R>, std::decay_t<Id>,
                     std::decay_t<Name>>(std::views::all(std::forward<R>(keys)),
                                         std::forward<Id>(id),
                                         std::forward<Name>(name));
}

}  // namespace detail

// Wrappers do not nest: indexed_named gives both functions at once.
// forward_range: the keys are walked once to register the constraints and
// once more to fill the id table or to name the constraints.
template <typename R>
concept wrappable_keys =
    std::ranges::viewable_range<R> && std::ranges::forward_range<R> &&
    !detail::is_keys_view_v<std::remove_cvref_t<R>>;

template <wrappable_keys R, typename F>
    requires detail::key_id_fn<R, F>
constexpr auto indexed(R && keys, F && id) {
    return detail::make_keys_view(std::forward<R>(keys), std::forward<F>(id),
                                  detail::absent{});
}
template <wrappable_keys R, typename F>
    requires detail::key_name_fn<R, F>
constexpr auto named(R && keys, F && name) {
    return detail::make_keys_view(std::forward<R>(keys), detail::absent{},
                                  std::forward<F>(name));
}
template <wrappable_keys R, typename IdFn, typename NameFn>
    requires detail::key_id_fn<R, IdFn> && detail::key_name_fn<R, NameFn>
constexpr auto indexed_named(R && keys, IdFn && id, NameFn && name) {
    return detail::make_keys_view(std::forward<R>(keys), std::forward<IdFn>(id),
                                  std::forward<NameFn>(name));
}

}  // namespace mippp
