#pragma once
#include <concepts>
#include <cstddef>
#include <optional>
#include <vector>

#include "mippp/algorithm/deletion_filter.hpp"
#include "mippp/infeasibility_certificate.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/utility/iis_statistics.hpp"

namespace mippp::iis {

// Integrality is fixed background in original mode, not a removable candidate.
// A feasible LP relaxation says nothing about feasibility of the original MIP.
enum class domain { original, lp_relaxation };
enum class member_kind { row_lower, row_upper, variable_lower, variable_upper };
struct member {
    member_kind kind;
    // Index into system.rows or system.variables, selected by kind. Never a
    // backend handle: rebuilding trials changes their solver-side numbering.
    std::size_t index;
    friend bool operator==(const member &, const member &) = default;
};

// Explicit input avoids losing integrality or hidden constraints during model
// introspection. Bounds are absent when infinite; binary variables are integer
// variables with explicit [0,1] bounds, so each bound can be tested separately.
template <std::floating_point Scalar = double>
struct linear_system {
    struct variable {
        std::optional<Scalar> lower;
        std::optional<Scalar> upper;
        bool integer = false;
    };
    struct row {
        // Coefficients refer to variables by input index. Move any constant
        // term to the bounds before supplying the row; duplicate terms are OK.
        std::vector<std::pair<std::size_t, Scalar>> terms;
        std::optional<Scalar> lower;
        std::optional<Scalar> upper;
    };
    std::vector<variable> variables;
    std::vector<row> rows;
};

// Row multipliers in original insertion order. Bounds need not be included in
// this ray: by default the adapter keeps ALL finite variable bounds.
template <typename Model>
concept has_infeasibility_ray = requires(Model & model) {
    {
        model.get_infeasibility_ray()
    } -> std::same_as<std::optional<std::vector<model_scalar_t<Model>>>>;
};

template <typename Model>
concept has_infeasibility_certificate = requires(Model & model) {
    {
        model.get_infeasibility_certificate()
    }
    -> std::same_as<
        std::optional<linear_infeasibility_certificate<model_scalar_t<Model>>>>;
};

// Adapter comparators see original row/bound identities, never temporary solver
// handles or seed-local indices. Return true when a should be tried before b.
template <typename Order>
concept linear_candidate_order =
    std::same_as<Order, input_order> ||
    std::strict_weak_order<Order &, member, member>;

struct rows_first_order {
    bool operator()(member a, member b) const noexcept {
        const auto row = [](member m) {
            return m.kind == member_kind::row_lower ||
                   m.kind == member_kind::row_upper;
        };
        return row(a) && !row(b);
    }
};

struct bounds_first_order {
    bool operator()(member a, member b) const noexcept {
        return rows_first_order{}(b, a);
    }
};

enum class elasticity_strategy { off, rebuild, reuse };
enum class deletion_strategy { rebuild, reuse };

// Pass as a constexpr template argument to select algorithms and workspaces.
// Per-run limits and tolerances belong in linear_options below.
struct linear_policy {
    domain analyzed_domain = domain::original;
    elasticity_strategy elasticity = elasticity_strategy::off;
    deletion_strategy deletion = deletion_strategy::rebuild;
    bool native_seed = false;
    bool prune_bounds = false;
    bool order_by_weight = false;
    bool measure_time = false;
};

template <linear_policy Policy>
concept valid_linear_policy =
    (Policy.analyzed_domain == domain::original ||
     Policy.analyzed_domain == domain::lp_relaxation) &&
    (Policy.elasticity == elasticity_strategy::off ||
     Policy.elasticity == elasticity_strategy::rebuild ||
     Policy.elasticity == elasticity_strategy::reuse) &&
    (Policy.deletion == deletion_strategy::rebuild ||
     Policy.deletion == deletion_strategy::reuse) &&
    (Policy.native_seed || (!Policy.prune_bounds && !Policy.order_by_weight));

struct elasticity_parameters {
    std::size_t max_solves = 16;
    double violation_tolerance = 1e-7;
};
struct certificate_parameters {
    double relative_tolerance = 1e-9;
};

// Per-extraction settings. The comparator can hold runtime scores or move-only
// state; its concrete type lets calls be inlined.
template <linear_candidate_order Order = input_order>
struct linear_options {
    options limits{};
    elasticity_parameters elastic{};
    certificate_parameters native{};
    [[no_unique_address]] Order order{};
};

struct linear_diagnostics {
    // Value snapshot: formatting later never depends on a live options object,
    // comparator, factory, or cancellation source.
    linear_policy policy{};
    elasticity_parameters elastic{};
    certificate_parameters native{};
    std::size_t max_solves = std::numeric_limits<std::size_t>::max();
    double time_limit_seconds = std::numeric_limits<double>::infinity();
    std::chrono::steady_clock::time_point deadline =
        std::chrono::steady_clock::time_point::max();
    bool cancellation_requested = false;
    bool deletion_reuse_supported = false;
    bool elastic_reuse_supported = false;
    bool solver_time_limit_supported = false;
    seed_outcome native_seed = seed_outcome::disabled;
    seed_outcome elastic_seed = seed_outcome::disabled;
    bool full_set_fallback = false;
};

struct linear_result {
    result reduction;
    // Populated only for a proven infeasible subsystem. An empty list alone
    // does not distinguish feasibility, an unknown result, or an empty IIS;
    // consult reduction.initial_status and reduction.irreducible as well.
    std::vector<member> members;
    domain analyzed_domain = domain::original;
    std::size_t elasticity_calls = 0;
    bool elasticity_seed_used = false;
    std::size_t elasticity_reoptimizations = 0;
    bool native_seed_used = false;
    std::size_t native_seed_size = 0;
    bool deletion_model_reused = false;
    std::size_t deletion_reoptimizations = 0;
    linear_statistics statistics;
    linear_diagnostics diagnostics;
};

namespace detail {
struct disabled_feature {};
}  // namespace detail

}  // namespace mippp::iis
