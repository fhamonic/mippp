#pragma once

#include <chrono>
#include <cstddef>
#include <limits>
#include <stop_token>

namespace mippp {

struct iis_limits {
    std::size_t max_solves = std::numeric_limits<std::size_t>::max();
    // one budget for the whole call, not per trial: it becomes a single
    // deadline when the call starts. NaN or negative throws
    // std::invalid_argument, and infinity means no deadline.
    std::chrono::duration<double> time_limit{
        std::numeric_limits<double>::infinity()};
    std::stop_token stop_token = {};
};

enum class iis_outcome {
    irreducible,
    not_proven_minimal,
    feasible,
    undetermined
};

// Reported as a std::optional, empty on a complete answer. A native
// not_proven_minimal answer is also empty when its routine did not prove
// minimality and no limit stopped it: there is no limit to raise.
enum class iis_reason {
    solve_limit,
    time_limit,
    cancelled,
    inconclusive_trial
};

}  // namespace mippp
