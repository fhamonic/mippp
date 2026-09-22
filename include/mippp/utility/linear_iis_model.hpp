#pragma once

#include <concepts>
#include <cstddef>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "mippp/algorithm/deletion_filter.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/utility/iis_limits.hpp"
#include "mippp/utility/iis_statistics.hpp"

namespace mippp::iis::detail {

// Preserve a portable explanation, never a solver-specific numeric code.
// Called on a status already fetched for correctness; no extra solver queries.
template <typename StatusVariant>
void record_solver_issue(const StatusVariant & value, work_statistics & stats) {
    const auto issue = std::visit(
        [](const auto & s) {
            using S = std::decay_t<decltype(s)>;
            if constexpr(std::derived_from<S, status::time_limit>)
                return solve_issue::time_limit;
            else if constexpr(std::derived_from<S, status::iteration_limit>)
                return solve_issue::iteration_limit;
            else if constexpr(std::derived_from<S, status::node_limit>)
                return solve_issue::node_limit;
            else if constexpr(std::derived_from<S, status::solution_limit>)
                return solve_issue::solution_limit;
            else if constexpr(std::derived_from<S, status::memory_limit> ||
                              std::derived_from<S, status::out_of_memory>)
                return solve_issue::memory_limit;
            else if constexpr(std::derived_from<S, status::limit_reached>)
                return solve_issue::other_limit;
            else if constexpr(std::derived_from<S, status::interrupted>)
                return solve_issue::interrupted;
            else if constexpr(std::derived_from<S, status::numerical_failure> ||
                              std::same_as<S,
                                           status::optimal_infeasible_unscaled>)
                return solve_issue::numerical;
            else if constexpr(std::derived_from<S, status::failed>)
                return solve_issue::failed;
            else if constexpr(std::same_as<
                                  S, status::primal_and_dual_infeasible> ||
                              std::same_as<S, status::infeasible_or_unbounded>)
                return solve_issue::ambiguous;
            else if constexpr(std::derived_from<S, status::infeasible>)
                return solve_issue::none;
            else
                return s.solution_available ? solve_issue::none
                                            : solve_issue::no_solution;
        },
        value);
    if(issue != solve_issue::none) stats.last_solver_issue = issue;
}

inline std::string nonempty_model_message(std::size_t variables,
                                          std::size_t constraints) {
    return "The model factory must create an empty model for each conflict "
           "check. "
           "It returned " +
           std::to_string(variables) + " variables and " +
           std::to_string(constraints) +
           " constraints. Supply the problem through linear_system; "
           "use the factory only to select and configure a fresh solver model.";
}

// Shared by ordinary feasibility trials and elastic LPs. Ambiguous or
// numerically suspect statuses must never authorize dropping assumptions.
template <typename StatusVariant>
feasibility classify_feasibility(const StatusVariant & status_value) {
    return std::visit(
        [](const auto & value) {
            using Status = std::decay_t<decltype(value)>;
            if constexpr(std::derived_from<
                             Status, status::primal_and_dual_infeasible> ||
                         std::derived_from<
                             Status, status::optimal_infeasible_unscaled> ||
                         std::derived_from<Status, status::numerical_failure>)
                return feasibility::unknown;
            else if constexpr(std::derived_from<Status, status::infeasible>)
                return feasibility::infeasible;
            else if(value.solution_available)
                return feasibility::feasible;
            else
                return feasibility::unknown;
        },
        status_value);
}

// The adapter supplies a linear inequality for each candidate: terms >= rhs
// for a lower side, terms <= rhs for an upper side. This avoids depending on
// the linear_system/member representation and lets bounds use the same path.
template <typename Scalar>
struct linear_inequality {
    std::vector<std::pair<std::size_t, Scalar>> terms;
    Scalar rhs;
    bool lower;
};

}  // namespace mippp::iis::detail
