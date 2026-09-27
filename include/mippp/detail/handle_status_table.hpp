#pragma once

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "mippp/utility/variant.hpp"

namespace mippp::detail {

template <typename V>
inline constexpr bool is_tag_variant_v = false;

template <typename... Tags>
inline constexpr bool is_tag_variant_v<std::variant<Tags...>> =
    sizeof...(Tags) <= 256 &&
    ((std::is_empty_v<Tags> && std::default_initializable<Tags>) && ...);

// One status per handle id, stored as the index of its alternative. Ids past
// the bound read as alternative 0, so a producer only writes the others.
template <typename Status>
    requires is_tag_variant_v<Status>
class handle_status_table {
private:
    static constexpr std::size_t _num_alternatives =
        std::variant_size_v<Status>;

    template <std::size_t... I>
    static constexpr Status _make_status(std::size_t code,
                                         std::index_sequence<I...>) noexcept {
        Status status;
        (void)((code == I && (status.template emplace<I>(), true)) || ...);
        return status;
    }

    template <typename Tag, std::size_t... I>
    static constexpr std::array<bool, sizeof...(I)> _make_derived_mask(
        std::index_sequence<I...>) {
        return {
            std::derived_from<std::variant_alternative_t<I, Status>, Tag>...};
    }

    std::vector<std::uint8_t> _codes;

public:
    constexpr handle_status_table() = default;
    constexpr explicit handle_status_table(std::size_t id_bound)
        : _codes(id_bound, std::uint8_t{0}) {}

    [[nodiscard]] constexpr std::size_t id_bound() const noexcept {
        return _codes.size();
    }

    // The bound is fixed at construction: an id past it throws
    // std::out_of_range rather than growing the table.
    constexpr void set(std::size_t id, const Status & status) {
        _codes.at(id) = static_cast<std::uint8_t>(status.index());
    }
    // A tag the variant does not list would convert to the base alternative
    // it derives from, silently dropping its refinement.
    template <typename Tag>
        requires(!std::same_as<Tag, Status> &&
                 !variant_with_alternative<Status, Tag>)
    void set(std::size_t, Tag) = delete;

    [[nodiscard]] constexpr Status get(std::size_t id) const noexcept {
        const std::size_t code = id < _codes.size() ? _codes[id] : 0u;
        return _make_status(code,
                            std::make_index_sequence<_num_alternatives>{});
    }

    // Counts the ids below the bound only, even when alternative 0 derives
    // from Tag.
    template <typename Tag>
    [[nodiscard]] constexpr std::size_t count_a() const noexcept {
        constexpr std::array<bool, _num_alternatives> derived =
            _make_derived_mask<Tag>(
                std::make_index_sequence<_num_alternatives>{});
        std::size_t count = 0;
        for(const std::uint8_t code : _codes)
            if(derived[code]) ++count;
        return count;
    }
};

}  // namespace mippp::detail
