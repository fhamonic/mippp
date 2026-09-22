#pragma once

#include <algorithm>
#include <chrono>
#include <concepts>
#include <cstddef>
#include <functional>
#include <limits>
#include <numeric>
#include <span>
#include <stop_token>
#include <utility>
#include <vector>

#include "mippp/algorithm/iis_limits.hpp"

// A standalone utility using only the standard library. The supplied checker
// (oracle) tests candidate IDs together with fixed background requirements.
// Removing candidates must never turn a feasible subset into an infeasible one.
namespace mippp::iis {

// Unknown includes limits, numerical trouble and ambiguous solver statuses.
// It is a separate outcome because neither retaining nor deleting a candidate
// on that evidence establishes that the candidate is necessary.
enum class feasibility { feasible, infeasible, unknown };
struct check_statistics {
    std::size_t feasible = 0, infeasible = 0, unknown = 0;
    void record(feasibility state) noexcept {
        if(state == feasibility::feasible)
            ++feasible;
        else if(state == feasibility::infeasible)
            ++infeasible;
        else
            ++unknown;
    }
};
struct reduction_statistics {
    check_statistics outcomes;
    std::size_t initial_checks = 0, batch_checks = 0, singleton_checks = 0;
    std::size_t removed_by_batches = 0, removed_by_singletons = 0;
    // These describe completed individual tests in the returned subsystem,
    // not discarded seed attempts or candidates still awaiting their test.
    std::size_t necessary_members = 0, unresolved_members = 0;
};
struct result {
    // IDs index the original candidate list, even after reduction.
    // Until initial_status is infeasible, these are only unchecked candidates.
    std::vector<std::size_t> members;
    feasibility initial_status = feasibility::unknown;
    termination reason = termination::completed;
    std::size_t solve_count = 0;
    // Inclusion-minimal relative to the oracle's fixed background assumptions;
    // this does not promise the smallest conflict among all possible conflicts.
    bool irreducible = false;
    reduction_statistics statistics;

    [[nodiscard]] bool proven_infeasible() const noexcept {
        // Every accepted deletion preserves infeasibility, so the initial
        // proof remains sufficient even when reduction stops prematurely.
        return initial_status == feasibility::infeasible;
    }
};

// Invoke through a reference: stateful and move-only oracles need no copying.
// The span is borrowed for one call and its order is not part of the contract.
template <typename Oracle>
concept feasibility_oracle =
    requires(Oracle & oracle, std::span<const std::size_t> subset) {
        { std::invoke(oracle, subset) } -> std::same_as<feasibility>;
    };

// Default traversal requires neither sorting nor comparator calls.
struct input_order {};

template <typename Order>
concept candidate_order =
    std::same_as<Order, input_order> ||
    std::strict_weak_order<Order &, std::size_t, std::size_t>;

// Without batching, at most candidate_count + 1 oracle calls. Batching can
// reduce calls on redundant systems but adds work when most members are needed.
// Limits are checked between calls;
// the oracle is responsible for enforcing a deadline during its own work.
// Unknown trials retain their candidates, preserving a proven infeasible
// subsystem. Unresolved individual trials prevent a claim of irreducibility.
// Oracle exceptions propagate unchanged.
namespace detail {
// Internal continuation state must come from this extraction, with the same
// oracle, candidates and fixed background. Counts and limits are cumulative.
// Public entry points always start with an unchecked result; only the adapter
// may reuse its own unchanged full-model proof after an unsuccessful prefilter.
template <feasibility_oracle Oracle, candidate_order Order>
[[nodiscard]] result deletion_filter_impl(result answer, Oracle && oracle,
                                          options opts, Order order) {
    opts = detail::normalize_limits(opts);
    const auto candidate_count = answer.members.size();
    answer.reason = termination::completed;
    auto stopped = [&] {
        if(auto reason = detail::stop_reason(opts, answer.solve_count)) {
            answer.reason = *reason;
            return true;
        }
        return false;
    };
    auto check = [&](std::span<const std::size_t> subset) {
        ++answer.solve_count;
        const auto state = std::invoke(oracle, subset);
        answer.statistics.outcomes.record(state);
        return state;
    };
    // Prove that the full set conflicts before removing anything. A caller's
    // earlier solve may have used different candidates, fixed requirements
    // or tolerances, so it cannot substitute for this check.
    if(!answer.proven_infeasible()) {
        if(stopped()) return answer;
        ++answer.statistics.initial_checks;
        answer.initial_status = check(answer.members);
        if(answer.initial_status != feasibility::infeasible) {
            if(answer.initial_status == feasibility::unknown && !stopped())
                answer.reason = termination::indeterminate;
            return answer;
        }
    }
    if constexpr(!std::same_as<Order, input_order>) {
        if(!answer.members.empty() && stopped()) return answer;
        // Smaller priority comes first. Resolve equivalent priorities by ID
        // for deterministic trials. Capture by reference so move-only scorers
        // work without type erasure or copies by the sorting implementation.
        std::sort(answer.members.begin(), answer.members.end(),
                  [&](auto a, auto b) {
                      if(std::invoke(order, a, b)) return true;
                      if(std::invoke(order, b, a)) return false;
                      return a < b;
                  });
    }
    auto batch_size = std::min(opts.initial_batch_size, candidate_count);
    if(batch_size > 1) {
        // Allocate scratch space only for callers opting into batching. Keep
        // members intact while checking a trial so an inconclusive group never
        // discards the last proven infeasible subsystem.
        std::vector<std::size_t> trial;
        trial.reserve(candidate_count);
        for(; batch_size > 1; batch_size /= 2) {
            std::size_t begin = 0;
            while(begin < answer.members.size()) {
                const auto count =
                    std::min(batch_size, answer.members.size() - begin);
                if(count == 1)
                    break;  // leave a singleton tail to the final pass
                if(stopped()) return answer;
                const auto first =
                    answer.members.begin() + static_cast<std::ptrdiff_t>(begin);
                const auto last = first + static_cast<std::ptrdiff_t>(count);
                trial.clear();
                trial.insert(trial.end(), answer.members.begin(), first);
                trial.insert(trial.end(), last, answer.members.end());
                ++answer.statistics.batch_checks;
                if(check(trial) == feasibility::infeasible) {
                    answer.statistics.removed_by_batches += count;
                    answer.members.swap(trial);
                    // The next group shifted into begin; don't skip it.
                } else {
                    // Feasibility proves only that something in this group
                    // is necessary. Unknown proves even less. Restore the
                    // whole group and let smaller trials decide its members.
                    begin += count;
                }
            }
        }
    }
    // Group trials never certify necessity of individual members. Even unknown
    // group results are harmless if this final pass resolves every survivor.
    bool all_decided = true;
    if constexpr(!std::same_as<Order, input_order>) {
        // Batch removal preserved relative order. Reverse once so the next
        // priority is at the end of the untested prefix. Swap/pop then costs
        // O(1) per trial without allowing a successful deletion to promote an
        // arbitrary tail candidate ahead of the requested priority.
        std::reverse(answer.members.begin(), answer.members.end());
        for(auto pending = answer.members.size(); pending > 0; --pending) {
            if(stopped()) return answer;
            const auto index = pending - 1;
            std::swap(answer.members[index], answer.members.back());
            const auto candidate = answer.members.back();
            answer.members.pop_back();
            ++answer.statistics.singleton_checks;
            const auto status = check(answer.members);
            if(status == feasibility::infeasible) {
                ++answer.statistics.removed_by_singletons;
                continue;
            }
            if(status == feasibility::feasible)
                ++answer.statistics.necessary_members;
            else
                ++answer.statistics.unresolved_members;
            answer.members.push_back(candidate);
            std::swap(answer.members[index], answer.members.back());
            if(status == feasibility::unknown) all_decided = false;
        }
        answer.irreducible = all_decided;
        answer.reason =
            all_decided ? termination::completed : termination::indeterminate;
        if(!all_decided) stopped();
        return answer;
    }
    std::size_t index = 0;
    while(index < answer.members.size()) {
        if(stopped()) return answer;
        // Swap/pop avoids quadratic vector shifting. Previously retained
        // candidates stay before index; their feasible witnesses remain valid
        // because removing more constraints cannot invalidate a solution.
        const auto candidate = answer.members[index];
        answer.members[index] = answer.members.back();
        answer.members.pop_back();
        ++answer.statistics.singleton_checks;
        const auto status = check(answer.members);
        // The last, still-untested member now occupies index. Test it next;
        // advancing here would accidentally skip a candidate after deletion.
        if(status == feasibility::infeasible) {
            ++answer.statistics.removed_by_singletons;
            continue;
        }
        if(status == feasibility::feasible)
            ++answer.statistics.necessary_members;
        else
            ++answer.statistics.unresolved_members;
        // Restore both the candidate and the traversal partition. A feasible
        // trial proves necessity; an unknown trial merely forbids deletion.
        answer.members.push_back(candidate);
        std::swap(answer.members[index], answer.members.back());
        ++index;
        // No retries in this pass: an unresolved candidate stays in the output
        // and conservatively prevents certification of irreducibility.
        if(status == feasibility::unknown) all_decided = false;
    }
    // An empty result can be an IIS relative to the fixed background: it means
    // the background alone is infeasible and there is no candidate to remove.
    answer.irreducible = all_decided;
    answer.reason =
        all_decided ? termination::completed : termination::indeterminate;
    if(!all_decided) stopped();
    return answer;
}

}  // namespace detail

template <feasibility_oracle Oracle, candidate_order Order = input_order>
[[nodiscard]] result deletion_filter(std::size_t candidate_count,
                                     Oracle && oracle, options opts = {},
                                     Order order = {}) {
    result answer;
    answer.members.resize(candidate_count);
    std::iota(answer.members.begin(), answer.members.end(), std::size_t{0});
    return detail::deletion_filter_impl(std::move(answer),
                                        std::forward<Oracle>(oracle), opts,
                                        std::move(order));
}

}  // namespace mippp::iis
