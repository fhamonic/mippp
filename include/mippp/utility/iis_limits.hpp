#pragma once

#include <algorithm>
#include <chrono>

#include "mippp/algorithm/iis_limits.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/utility/iis_statistics.hpp"

namespace mippp::iis::detail {

// Check cancellation and time after building/updating the model, just before
// each solve. Pass remaining time to the solver when supported; otherwise
// stopping can only be checked between solver calls.
template <typename Model>
bool prepare_iis_solve(Model & model, const options & limits,
                       work_statistics & stats,
                       budget_clock::time_point now = budget_clock::now()) {
    if(limits.stop.stop_requested() || now >= limits.deadline) {
        ++stats.solves_skipped;
        return false;
    }
    if constexpr(has_time_limit<Model> &&
                 requires(std::chrono::duration<double> t) {
                     model.set_time_limit(t);
                 }) {
        if(limits.deadline != budget_clock::time_point::max()) {
            const auto remaining =
                std::chrono::duration<double>(limits.deadline - now);
            // Never loosen a factory's per-solve limit. Recompute for retained
            // models too; their original allowance must not restart each solve.
            const auto current =
                std::chrono::duration<double>(model.get_time_limit());
            stats.observed_solver_time_limit = current.count();
            model.set_time_limit(std::min(remaining, current));
        }
    }
    return true;
}
}  // namespace mippp::iis::detail
