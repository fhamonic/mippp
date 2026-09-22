#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <span>
#include <type_traits>
#include <vector>
#include "mippp/algorithm/ray_support.hpp"
#include "mippp/utility/prepared_linear_system.hpp"

namespace mippp::iis::detail {

template <bool OrderByWeight>
struct native_seed_data {
    seed_outcome outcome = seed_outcome::unavailable;
    std::optional<std::vector<std::size_t>> members;
    // Seeding alone needs no weight buffer, not even its vector bookkeeping.
    [[no_unique_address]] std::conditional_t<
        OrderByWeight, std::vector<long double>, disabled_feature> weights;
};

// Read and copy a certificate from an already solved model; no extra solve is
// performed here. Its selected candidates are only a proposal until verified.
template <bool PruneBounds = false, bool OrderByWeight = false, typename Model,
          std::floating_point Scalar>
native_seed_data<OrderByWeight> collect_native_seed(
    Model & model, std::span<const std::size_t> active,
    const prepared_linear_system<Scalar> & prepared,
    certificate_parameters native) {
    native_seed_data<OrderByWeight> answer;
    auto & native_seed = answer.members;
    auto & certificate_weights = answer.weights;
    const auto & system = prepared.system;
    const auto & candidates = prepared.candidates;
    // Validate native dimensions against the formulation, not just against
    // the certificate arrays. Malformed provider output must not turn reserve
    // subtraction or candidate-to-column indexing into undefined behavior.
    std::size_t expected_rows = 0;
    for(auto id : active) {
        if(id >= candidates.size()) return answer;
        const auto kind = candidates[id].kind;
        expected_rows +=
            kind == member_kind::row_lower || kind == member_kind::row_upper;
    }
    const auto column_count = model.num_variables();
    if(model.num_constraints() != expected_rows ||
       column_count != std::max(system.variables.size(), std::size_t{1})) {
        answer.outcome = seed_outcome::invalid_certificate;
        return answer;
    }
    if constexpr(has_infeasibility_certificate<Model>) {
        if(auto certificate = model.get_infeasibility_certificate()) {
            answer.outcome = seed_outcome::invalid_certificate;
            const auto rows = expected_rows;
            const auto columns = column_count;
            const auto & c = *certificate;
            const auto finite = [](const auto & values) {
                return std::all_of(values.begin(), values.end(),
                                   [](auto v) { return std::isfinite(v); });
            };
            if(c.row_lower.size() == rows && c.row_upper.size() == rows &&
               c.variable_lower.size() == columns &&
               c.variable_upper.size() == columns && finite(c.row_lower) &&
               finite(c.row_upper) && finite(c.variable_lower) &&
               finite(c.variable_upper)) {
                // Each adapter row contains exactly one original side.
                // Column order is unchanged, including unused columns.
                // Select both row and bound contributions; unlike the
                // row-ray path there is no need to retain every bound.
                std::vector<Scalar> weights;
                weights.reserve(active.size());
                std::size_t row = 0;
                for(auto id : active) {
                    const auto [kind, index] = candidates[id];
                    switch(kind) {
                        case member_kind::row_lower:
                            weights.push_back(c.row_lower[row++]);
                            break;
                        case member_kind::row_upper:
                            weights.push_back(c.row_upper[row++]);
                            break;
                        case member_kind::variable_lower:
                            weights.push_back(c.variable_lower[index]);
                            break;
                        case member_kind::variable_upper:
                            weights.push_back(c.variable_upper[index]);
                            break;
                    }
                }
                if(auto support = ray_support<Scalar>(
                       weights, native.relative_tolerance)) {
                    if constexpr(OrderByWeight) {
                        certificate_weights.assign(candidates.size(), 0);
                        for(std::size_t i = 0; i < active.size(); ++i)
                            certificate_weights[active[i]] =
                                std::abs(static_cast<long double>(weights[i]));
                    }
                    native_seed.emplace();
                    native_seed->reserve(support->size());
                    for(auto id : *support) native_seed->push_back(active[id]);
                }
            }
        }

    } else if constexpr(has_infeasibility_ray<Model>) {
        if(auto ray = model.get_infeasibility_ray()) {
            answer.outcome = seed_outcome::invalid_certificate;
            // Every row side was inserted separately, in active order.
            // Default: keep every bound. Optional A^T y screening uses
            // full signed multipliers and still revalidates the seed.
            std::vector<std::size_t> rows, bounds;
            rows.reserve(expected_rows);
            bounds.reserve(active.size() - expected_rows);
            for(auto id : active) {
                const auto kind = candidates[id].kind;
                if(kind == member_kind::row_lower ||
                   kind == member_kind::row_upper)
                    rows.push_back(id);
                else
                    bounds.push_back(id);
            }
            if(ray->size() == rows.size()) {
                if(auto support =
                       ray_support<Scalar>(*ray, native.relative_tolerance)) {
                    if constexpr(OrderByWeight || PruneBounds) {
                        auto columns = ray_column_magnitudes<Scalar>(
                            system.variables.size(), *ray,
                            [&](std::size_t row) -> const auto & {
                                return system.rows[candidates[rows[row]].index]
                                    .terms;
                            });
                        if constexpr(OrderByWeight) {
                            long double scale = 0;
                            for(auto value : *ray)
                                scale = std::max(
                                    scale,
                                    std::abs(static_cast<long double>(value)));
                            // Unknown bound contributions sort last, not
                            // as zero evidence. All values remain fixed
                            // during sorting and deletion.
                            certificate_weights.assign(
                                candidates.size(),
                                std::numeric_limits<long double>::max());
                            for(std::size_t i = 0; i < rows.size(); ++i)
                                certificate_weights[rows[i]] =
                                    std::abs(
                                        static_cast<long double>((*ray)[i])) /
                                    scale;
                            if(columns)
                                for(auto id : bounds)
                                    certificate_weights[id] =
                                        (*columns)[candidates[id].index];
                        }
                        if constexpr(PruneBounds) {
                            if(columns) {
                                const auto largest =
                                    columns->empty()
                                        ? 0.L
                                        : *std::max_element(columns->begin(),
                                                            columns->end());
                                std::erase_if(bounds, [&](auto id) {
                                    return largest == 0 ||
                                           (*columns)[candidates[id].index] /
                                                   largest <=
                                               native.relative_tolerance;
                                });
                            }
                        }
                    }
                    native_seed = std::move(bounds);
                    native_seed->reserve(native_seed->size() + support->size());
                    for(auto row : *support) native_seed->push_back(rows[row]);
                }
            }
        }
    }

    if(native_seed)
        answer.outcome = native_seed->size() < candidates.size()
                             ? seed_outcome::verification_skipped
                             : seed_outcome::not_smaller;
    return answer;
}
}  // namespace mippp::iis::detail
