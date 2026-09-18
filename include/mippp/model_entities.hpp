#pragma once

#include <cstddef>
#include <functional>
#include <optional>
#include <ranges>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "mippp/mapping.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/utility/entity_range.hpp"
#include "mippp/utility/zero.hpp"

namespace mippp {

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// Strong types /////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// CRTP: the comparisons take the *derived* type, so a variable never compares
// against a constraint that happens to share its id type. Deriving from the
// base with a different Derived is what would later give each model its own
// entity types.
template <typename Derived, typename Id>
class model_entity_base {
private:
    Id _id;

public:
    constexpr model_entity_base() = default;
    template <typename T>
        requires std::constructible_from<Id, T>
    constexpr explicit model_entity_base(T t) : _id(t) {}

    constexpr Id id() const noexcept { return _id; }

    constexpr std::size_t uid() const noexcept
        requires std::integral<Id>
    {
        return static_cast<std::size_t>(_id);
    }

    friend constexpr auto operator==(const Derived & a,
                                     const Derived & b) noexcept {
        return a._id == b._id;
    }
    friend constexpr auto operator<(const Derived & a,
                                    const Derived & b) noexcept {
        return a._id < b._id;
    }
};

template <typename Id, typename Scalar>
class model_variable
    : public model_entity_base<model_variable<Id, Scalar>, Id> {
private:
    using base = model_entity_base<model_variable<Id, Scalar>, Id>;

public:
    constexpr model_variable() = default;
    constexpr model_variable(model_variable && v) = default;
    constexpr model_variable(const model_variable & v) = default;

    constexpr model_variable & operator=(const model_variable &) = default;
    constexpr model_variable & operator=(model_variable &&) = default;

    // same clause as the base: without it std::constructible_from lies
    template <typename T>
        requires std::constructible_from<Id, T>
    constexpr explicit model_variable(T t) : base(t) {}

    constexpr auto linear_terms() const noexcept {
        return std::views::single(
            std::pair<model_variable<Id, Scalar>, Scalar>(*this, Scalar{1}));
    }
    constexpr zero_t constant() const noexcept { return {}; }
};

template <typename Id>
class model_constraint : public model_entity_base<model_constraint<Id>, Id> {
private:
    using base = model_entity_base<model_constraint<Id>, Id>;

public:
    constexpr model_constraint() = default;
    constexpr model_constraint(model_constraint && v) = default;
    constexpr model_constraint(const model_constraint & v) = default;

    constexpr model_constraint & operator=(const model_constraint &) = default;
    constexpr model_constraint & operator=(model_constraint &&) = default;

    template <typename T>
        requires std::constructible_from<Id, T>
    constexpr explicit model_constraint(T t) : base(t) {}
};

///////////////////////////////////////////////////////////////////////////////
//////////////////////////// Strong types mappings ////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// Adapts any mapping storage into one keyed by a model entity. The storage
// is lifted through maps::mapping_all (reference semantics for lvalues,
// ownership for rvalues); a lookup passes the entity itself when the storage
// understands it (callables, associative maps keyed by the entity) and falls
// back to the entity's uid() (arrays, vectors).
template <typename Entity, typename Map>
class entity_mapping : public mapping_view_base {
private:
    [[no_unique_address]] maps::mapping_all_t<Map> _map;

    // what a const access reaches: ref views are shallow-const (constness
    // carried by Map itself), owning views are deep-const
    using const_probe =
        std::conditional_t<std::is_reference_v<Map>,
                           std::remove_reference_t<Map>, const Map>;

public:
    constexpr entity_mapping(Map && map)
        : _map(maps::mapping_all(std::forward<Map>(map))) {}

    [[nodiscard]] constexpr decltype(auto) operator[](const Entity & e) {
        if constexpr(detail::mapping_subscriptable<std::remove_reference_t<Map>,
                                                   const Entity &>)
            return _map[e];
        else
            return _map[e.uid()];
    }
    [[nodiscard]] constexpr decltype(auto) operator[](const Entity & e) const {
        if constexpr(detail::mapping_subscriptable<const_probe, const Entity &>)
            return _map[e];
        else
            return _map[e.uid()];
    }
};

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

}  // namespace detail

}  // namespace mippp
