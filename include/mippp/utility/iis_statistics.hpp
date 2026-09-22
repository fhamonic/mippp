#pragma once
#include <algorithm>
#include <chrono>
#include <cstddef>
#include <optional>
#include "mippp/algorithm/deletion_filter.hpp"

namespace mippp::iis {

// A proposal is only a shortcut. 'used' means it passed a separate check;
// none of these states by itself asserts that the final conflict is minimal.
enum class seed_outcome {
    disabled,
    not_reached,
    unsupported,
    integer_model,
    unavailable,
    invalid_certificate,
    not_smaller,
    no_progress,
    inconclusive,
    limit_reached,
    verification_skipped,
    verification_feasible,
    verification_unknown,
    used
};

enum class solve_issue {
    none,
    time_limit,
    iteration_limit,
    node_limit,
    solution_limit,
    memory_limit,
    other_limit,
    interrupted,
    numerical,
    ambiguous,
    no_solution,
    failed
};

struct work_statistics {
    std::size_t models_built = 0;
    std::size_t solver_runs =
        0;  // Model::solve calls, not hidden internal optimizer passes
    std::size_t bound_updates = 0;
    std::size_t bound_only_proofs = 0;
    std::size_t solves_skipped = 0;  // stopped after construction/update
    std::size_t peak_variables = 0;
    std::size_t peak_constraints = 0;
    solve_issue last_solver_issue = solve_issue::none;
    // Captured only if a solver time limit is already being queried. Absent
    // means not observed, NOT unlimited. No extra native queries for
    // statistics.
    std::optional<double> observed_solver_time_limit;

    void model_built(std::size_t variables, std::size_t constraints) noexcept {
        ++models_built;
        peak_variables = std::max(peak_variables, variables);
        peak_constraints = std::max(peak_constraints, constraints);
    }
};

struct linear_statistics {
    std::size_t input_variables = 0;
    std::size_t input_rows = 0;
    std::size_t input_candidates = 0;
    std::size_t certificate_queries = 0;
    std::size_t native_verification_calls = 0;
    std::size_t elastic_verification_calls = 0;
    std::size_t elastic_seed_size = 0;
    check_statistics feasibility_checks, elastic_checks;
    work_statistics rebuild, deletion, elastic;
    // Two clock reads only when linear_policy::measure_time is true. Includes
    // input validation and cleanup; never claims to be pure optimizer time.
    std::optional<std::chrono::duration<double>> elapsed;
    [[nodiscard]] std::size_t solver_runs() const noexcept {
        return rebuild.solver_runs + deletion.solver_runs + elastic.solver_runs;
    }
    [[nodiscard]] std::size_t models_built() const noexcept {
        return rebuild.models_built + deletion.models_built +
               elastic.models_built;
    }
};

}  // namespace mippp::iis
