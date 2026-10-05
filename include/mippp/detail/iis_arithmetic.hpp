#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <ranges>
#include <span>
#include <utility>

#include "mippp/model_concepts.hpp"

// What an IIS answer can be decided from by arithmetic on the model's data,
// without a solve, on the models where a solve or a native routine fails.
namespace mippp::detail {

// The side of a row without terms that its activity, 0, violates: true for
// the lower side, which wins when 0 violates both, false for the upper one,
// and empty when 0 satisfies the row. The comparison is exact.
template <typename Scalar>
[[nodiscard]] constexpr std::optional<bool> iis_side_violated_by_zero(
    Scalar lower, Scalar upper) noexcept {
    if(lower > Scalar{0}) return true;
    if(upper < Scalar{0}) return false;
    return std::nullopt;
}

// Solvers report unknown on a model without columns, so the rows are read
// instead: every left-hand side is 0, and the first side that 0 violates is
// the whole explanation. The rows come from the caller, which may hold them
// already: constraints() may build a fresh snapshot on every call.
template <typename M, std::ranges::forward_range R>
    requires has_readable_constraint_bounds<M>
[[nodiscard]] std::optional<std::pair<model_constraint_t<M>, bool>>
iis_column_less_precheck(M & model, R && rows) {
    for(auto c : rows) {
        if(const auto lower =
               iis_side_violated_by_zero(model.get_constraint_lower_bound(c),
                                         model.get_constraint_upper_bound(c)))
            return std::pair{c, *lower};
    }
    return std::nullopt;
}

// other stands for the kinds whose admissible values this arithmetic does not
// decide, such as semi-continuous columns, which may also take 0.
enum class iis_column_kind { continuous, integer, binary, other };

struct iis_column_sides {
    bool lower;
    bool upper;
};

// A column whose own bounds admit no value is an IIS by itself: crossed
// bounds, an integer column whose interval holds no integer, or a binary
// column whose interval holds neither 0 nor 1, its bounds being kept even
// beyond [0, 1]. Freeing one bound of an integer or continuous column
// readmits a value, so both bounds are members; a binary bound beyond the
// domain excludes it alone. Empty when the bounds admit a value or the kind
// is other. The comparison is exact.
template <typename Scalar>
[[nodiscard]] std::optional<iis_column_sides> iis_self_infeasible_column(
    Scalar lower, Scalar upper, iis_column_kind kind) noexcept {
    if(kind == iis_column_kind::other) return std::nullopt;
    if(kind != iis_column_kind::continuous) {
        lower = std::ceil(lower);
        upper = std::floor(upper);
    }
    if(kind == iis_column_kind::binary) {
        if(lower > Scalar{1}) return iis_column_sides{true, false};
        if(upper < Scalar{0}) return iis_column_sides{false, true};
        lower = std::max(lower, Scalar{0});
        upper = std::min(upper, Scalar{1});
    }
    if(lower > upper) return iis_column_sides{true, true};
    return std::nullopt;
}

// The kind of a column from the type codes a backend spells for the three
// kinds; any other code is other.
template <char Continuous, char Integer, char Binary>
[[nodiscard]] constexpr iis_column_kind iis_column_kind_of(char type) noexcept {
    if(type == Continuous) return iis_column_kind::continuous;
    if(type == Integer) return iis_column_kind::integer;
    if(type == Binary) return iis_column_kind::binary;
    return iis_column_kind::other;
}

struct iis_self_infeasible_column_at {
    std::size_t index;
    iis_column_sides sides;
};

// The first column whose own bounds admit no value, over the bounds and type
// codes a backend reads in bulk: lower and upper hold an entry per type code.
template <char Continuous, char Integer, char Binary>
[[nodiscard]] std::optional<iis_self_infeasible_column_at>
iis_first_self_infeasible_column(std::span<const double> lower,
                                 std::span<const double> upper,
                                 std::span<const char> types) noexcept {
    for(std::size_t j = 0; j < types.size(); ++j) {
        if(const auto sides = iis_self_infeasible_column(
               lower[j], upper[j],
               iis_column_kind_of<Continuous, Integer, Binary>(types[j])))
            return iis_self_infeasible_column_at{j, *sides};
    }
    return std::nullopt;
}

}  // namespace mippp::detail
