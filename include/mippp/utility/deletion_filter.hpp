#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <functional>
#include <numeric>
#include <optional>
#include <span>
#include <stdexcept>
#include <stop_token>
#include <utility>
#include <variant>
#include <vector>

#include "mippp/utility/iis_outcome.hpp"

namespace mippp {

enum class deletion_verdict { feasible, infeasible, inconclusive };

// Invoked as an lvalue with the active candidate indices, borrowed for the
// call and in no particular order. The answer is only correct for a monotone
// oracle: every subset of a feasible set of candidates must be feasible.
template <typename O>
concept deletion_oracle =
    requires(O & oracle, std::span<const std::size_t> active) {
        { std::invoke(oracle, active) } -> std::same_as<deletion_verdict>;
    };

// incomplete is never returned: it is the value of a result no run wrote
using deletion_filter_outcome =
    std::variant<iis_outcome::incomplete, iis_outcome::irreducible,
                 iis_outcome::feasible, iis_outcome::inconclusive_trial,
                 iis_outcome::interrupted, iis_outcome::time_limit,
                 iis_outcome::solve_limit>;

struct deletion_filter_result {
    // Ascending, and empty unless a trial proved infeasibility. A stop keeps
    // the last proven subset, untested candidates included. Empty members with
    // irreducible mean that the background alone is infeasible.
    std::vector<std::size_t> members;
    deletion_filter_outcome outcome;
};

namespace detail {

template <typename Clock>
struct deletion_budget {
    std::size_t max_solves;
    typename Clock::time_point deadline;
    std::stop_token stop_token;
};

template <typename Clock = std::chrono::steady_clock>
[[nodiscard]] deletion_budget<Clock> make_deletion_budget(
    const iis_limits & limits, typename Clock::time_point now = Clock::now()) {
    const double seconds = limits.time_limit.count();
    if(std::isnan(seconds) || seconds < 0)
        throw std::invalid_argument(
            "iis_limits::time_limit must be a non-negative duration");
    // Saturate in floating point: converting a huge duration to the clock's
    // integer ticks first would overflow.
    const auto room = Clock::time_point::max() - now;
    const double ticks =
        std::chrono::duration<double, typename Clock::period>(limits.time_limit)
            .count();
    auto deadline = Clock::time_point::max();
    if(ticks < static_cast<double>(room.count())) {
        const typename Clock::duration offset(
            static_cast<typename Clock::rep>(ticks));
        if(offset < room) deadline = now + offset;
    }
    return {limits.max_solves, deadline, limits.stop_token};
}

template <typename Clock>
[[nodiscard]] std::optional<deletion_filter_outcome> deletion_stop(
    const deletion_budget<Clock> & budget, std::size_t solves, bool proven) {
    if(budget.stop_token.stop_requested())
        return iis_outcome::interrupted(proven);
    if(Clock::now() >= budget.deadline) return iis_outcome::time_limit(proven);
    if(solves >= budget.max_solves) return iis_outcome::solve_limit(proven);
    return std::nullopt;
}

// proven = true asserts that the same oracle already found members
// infeasible. The initial trial is then skipped, so a wrong claim gives a wrong
// answer.
struct deletion_state {
    std::vector<std::size_t> members;
    bool proven = false;
};

template <typename Clock, deletion_oracle O>
[[nodiscard]] deletion_filter_result run_deletion_filter(
    deletion_state state, O && oracle, const deletion_budget<Clock> & budget,
    std::size_t batch_size = 1) {
    std::size_t solves = 0;
    auto trial = [&](std::span<const std::size_t> active) {
        ++solves;
        return std::invoke(oracle, active);
    };
    auto stop = [&] { return deletion_stop(budget, solves, state.proven); };
    auto inconclusive = [&]() -> deletion_filter_outcome {
        if(Clock::now() >= budget.deadline)
            return iis_outcome::time_limit(state.proven);
        return iis_outcome::inconclusive_trial(state.proven);
    };
    auto finish = [&](deletion_filter_outcome outcome) {
        deletion_filter_result result{{}, outcome};
        if(state.proven) {
            result.members = std::move(state.members);
            std::ranges::sort(result.members);
        }
        return result;
    };

    if(!state.proven) {
        if(auto s = stop()) return finish(*s);
        const auto verdict = trial(state.members);
        if(verdict == deletion_verdict::feasible)
            return finish(iis_outcome::feasible{});
        if(verdict != deletion_verdict::infeasible)
            return finish(inconclusive());
        state.proven = true;
    }

    // Group trials never prove a member necessary, so an inconclusive group
    // leaves no gap: the single pass below decides every survivor.
    if(auto size = std::min(batch_size, state.members.size()); size > 1) {
        std::vector<std::size_t> remainder;
        remainder.reserve(state.members.size());
        for(; size > 1; size /= 2) {
            std::size_t begin = 0;
            while(begin < state.members.size()) {
                const auto count = std::min(size, state.members.size() - begin);
                if(count == 1) break;
                if(auto s = stop()) return finish(*s);
                const auto first =
                    state.members.begin() + static_cast<std::ptrdiff_t>(begin);
                const auto last = first + static_cast<std::ptrdiff_t>(count);
                remainder.assign(state.members.begin(), first);
                remainder.insert(remainder.end(), last, state.members.end());
                // on a deletion the next group shifts into begin: no advance
                if(trial(remainder) == deletion_verdict::infeasible)
                    state.members.swap(remainder);
                else
                    begin += count;
            }
        }
    }

    // One pass suffices: a member kept before index stays necessary, since
    // removing more candidates keeps its feasible witness feasible.
    std::optional<deletion_filter_outcome> gap;
    std::size_t index = 0;
    while(index < state.members.size()) {
        if(auto s = stop()) return finish(*s);
        const auto candidate = state.members[index];
        state.members[index] = state.members.back();
        state.members.pop_back();
        const auto verdict = trial(state.members);
        // the untested last member now sits at index: advancing would skip it
        if(verdict == deletion_verdict::infeasible) continue;
        state.members.push_back(candidate);
        std::swap(state.members[index], state.members.back());
        ++index;
        if(verdict != deletion_verdict::feasible) gap = inconclusive();
    }
    return finish(gap.value_or(iis_outcome::irreducible{}));
}

}  // namespace detail

// Limits act between trials only: a running oracle call is never interrupted,
// so an oracle that can overrun the deadline must enforce it itself.
template <deletion_oracle O>
[[nodiscard]] deletion_filter_result deletion_filter(
    std::size_t candidate_count, O && oracle, const iis_limits & limits = {}) {
    const auto budget = detail::make_deletion_budget(limits);
    detail::deletion_state state;
    // not resize(): on a fresh vector, gcc 14 reports a false
    // -Wnull-dereference in the caller's translation unit
    state.members.assign(candidate_count, std::size_t{0});
    std::iota(state.members.begin(), state.members.end(), std::size_t{0});
    return detail::run_deletion_filter(std::move(state), oracle, budget);
}

}  // namespace mippp
