#pragma once

#include <concepts>
#include <cstddef>
#include <functional>
#include <span>
#include <stdexcept>
#include <vector>

#include "mippp/algorithm/deletion_filter.hpp"

namespace mippp::iis {

// The checker permits violations of every candidate outside the enforced
// (hard) set. A feasible reply lists candidates with positive violations.
// Infeasible means the enforced candidates alone conflict; all others must
// be freely relaxable.
struct elastic_trial {
    feasibility status = feasibility::unknown;
    std::vector<std::size_t> violated;
};

struct elasticity_result {
    std::vector<std::size_t> members;
    bool proven_infeasible = false;
    termination reason = termination::completed;
    std::size_t solve_count = 0;
    check_statistics outcomes;
    bool no_progress = false;
};

template <typename Oracle>
concept elastic_oracle =
    requires(Oracle & oracle, std::span<const std::size_t> hard) {
        { std::invoke(oracle, hard) } -> std::same_as<elastic_trial>;
    };

// The checker builds and solves the elastic problem. The first violated set
// may not conflict: another solution could satisfy it while violating other
// candidates. Keep enforcing new violations until the checker proves a
// conflict.
template <elastic_oracle Oracle>
[[nodiscard]] elasticity_result elasticity_filter(std::size_t candidate_count,
                                                  Oracle && oracle,
                                                  options opts = {}) {
    opts = detail::normalize_limits(opts);
    elasticity_result answer;
    std::vector<bool> hard(candidate_count, false);
    for(;;) {
        if(auto reason = detail::stop_reason(opts, answer.solve_count))
            answer.reason = *reason;
        else {
            ++answer.solve_count;
            auto trial = std::invoke(
                oracle, std::span<const std::size_t>(answer.members));
            answer.outcomes.record(trial.status);
            if(trial.status == feasibility::infeasible) {
                answer.proven_infeasible = true;
                return answer;
            }
            if(trial.status == feasibility::unknown || trial.violated.empty()) {
                answer.no_progress = trial.status == feasibility::feasible &&
                                     trial.violated.empty();
                // No progress is not evidence for an infeasible seed. The
                // caller may fall back to ordinary deletion on the full model.
                answer.reason = detail::stop_reason(opts, answer.solve_count)
                                    .value_or(termination::indeterminate);
                return answer;
            }
            for(auto id : trial.violated) {
                if(id >= candidate_count || hard[id])
                    throw std::invalid_argument(
                        "The soft-constraint check returned candidate " +
                        std::to_string(id) +
                        " which is out of range or was already enforced. "
                        "Return only new candidate IDs from 0 through "
                        "candidate_count - 1; "
                        "candidate_count=" +
                        std::to_string(candidate_count) + ".");
                hard[id] = true;
                answer.members.push_back(id);
            }
            // At least one new candidate per successful iteration implies at
            // most N+1 calls, even if the oracle never finds a small seed.
            continue;
        }
        return answer;
    }
}

}  // namespace mippp::iis
