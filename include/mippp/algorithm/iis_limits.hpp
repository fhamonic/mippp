#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <stdexcept>
#include <stop_token>
#include "mippp/algorithm/iis_messages.hpp"

namespace mippp::iis {

enum class termination {
    completed,
    indeterminate,
    solve_limit,
    time_limit,
    cancelled
};

struct options {
    // Counts oracle calls, including verification and bound-only certificates.
    std::size_t max_solves = std::numeric_limits<std::size_t>::max();
    std::chrono::steady_clock::time_point deadline =
        std::chrono::steady_clock::time_point::max();
    std::stop_token stop = {};
    // Zero and one disable grouping; the final singleton pass is unchanged.
    std::size_t initial_batch_size = 1;
    // Overall elapsed budget, not a fresh allowance for each oracle call.
    std::chrono::duration<double> time_limit{
        std::numeric_limits<double>::infinity()};
};

namespace detail {
using budget_clock = std::chrono::steady_clock;

inline options normalize_limits(
    options opts, budget_clock::time_point now = budget_clock::now()) {
    const auto seconds = opts.time_limit.count();
    if(std::isnan(seconds) || seconds < 0)
        throw std::invalid_argument(
            "The conflict search time limit is invalid. " +
            setting_help("limits.time_limit (seconds)", setting_value(seconds),
                         "0 to stop immediately, a positive duration, or "
                         "infinity for no relative limit"));
    // Saturate before converting a floating duration to integral clock ticks.
    const auto available = budget_clock::time_point::max() - now;
    if(opts.time_limit < std::chrono::duration<double>(available))
        opts.deadline =
            std::min(opts.deadline,
                     now + std::chrono::duration_cast<budget_clock::duration>(
                               opts.time_limit));
    // Nested phases share the resulting deadline; never restart the stopwatch.
    opts.time_limit = std::chrono::duration<double>::max();
    return opts;
}

inline std::optional<termination> stop_reason(
    const options & opts, std::size_t calls,
    budget_clock::time_point now = budget_clock::now()) {
    if(opts.stop.stop_requested()) return termination::cancelled;
    if(now >= opts.deadline) return termination::time_limit;
    if(calls >= opts.max_solves) return termination::solve_limit;
    return std::nullopt;
}

// Phase accounting for composed algorithms. Deadlines are normalized once;
// remaining() only narrows solve counts and never restarts relative time.
// Counts reported by a completed phase are charged exactly once. Reject a
// broken phase contract rather than wrapping unsigned arithmetic into a huge
// apparent allowance. This helper owns no oracle and makes no solver calls.
class phase_budget {
    options limits_;
    std::size_t used_ = 0;

public:
    explicit phase_budget(options limits) : limits_(normalize_limits(limits)) {}
    [[nodiscard]] const options & limits() const noexcept { return limits_; }
    [[nodiscard]] std::size_t used() const noexcept { return used_; }
    [[nodiscard]] options remaining(
        std::size_t local_cap = std::numeric_limits<std::size_t>::max()) const {
        auto next = limits_;
        next.max_solves = std::min(local_cap, limits_.max_solves - used_);
        return next;
    }
    void consume(std::size_t calls) {
        if(calls > limits_.max_solves - used_)
            throw std::logic_error(
                "Internal conflict-search accounting error: reported " +
                setting_value(calls) + " checks with only " +
                setting_value(limits_.max_solves - used_) +
                " remaining. Please report this as a library bug; increasing "
                "the limit is not a fix.");
        used_ += calls;
    }
};
}  // namespace detail
}  // namespace mippp::iis
