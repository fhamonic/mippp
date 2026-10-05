#pragma once

#include <algorithm>
#include <concepts>
#include <cstddef>
#include <type_traits>
#include <vector>

#include "mippp/utility/variant.hpp"

namespace mippp::detail {

// One status per handle id. Ids past the bound read as alternative 0, so a
// producer only writes the others.
template <typename Status>
class handle_status_table {
private:
    std::vector<Status> _statuses;

public:
    // kept apart from the next one: a defaulted id_bound there would make the
    // default constructor explicit, and clang rejects {} as an empty table
    constexpr handle_status_table() = default;
    constexpr explicit handle_status_table(std::size_t id_bound)
        : _statuses(id_bound) {}

    [[nodiscard]] constexpr std::size_t id_bound() const noexcept {
        return _statuses.size();
    }

    // The bound is fixed at construction: an id past it throws
    // std::out_of_range rather than growing the table.
    constexpr void set(std::size_t id, const Status & status) {
        _statuses.at(id) = status;
    }
    // A tag the variant does not list would convert to the base alternative
    // it derives from, silently dropping its refinement.
    template <typename Tag>
        requires(!std::same_as<Tag, Status> &&
                 !variant_with_alternative<Status, Tag>)
    void set(std::size_t, Tag) = delete;

    [[nodiscard]] constexpr Status get(std::size_t id) const
        noexcept(std::is_nothrow_copy_constructible_v<Status>) {
        return id < _statuses.size() ? _statuses[id] : Status{};
    }

    // Counts the ids below the bound only, even when alternative 0 derives
    // from Tag.
    template <typename Tag>
        requires variant_containing_a<Status, Tag>
    [[nodiscard]] constexpr std::size_t count_a() const noexcept {
        return static_cast<std::size_t>(std::ranges::count_if(
            _statuses, [](const Status & s) { return is_a<Tag>(s); }));
    }
};

}  // namespace mippp::detail
