#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <numeric>
#include <ratio>
#include <set>
#include <span>
#include <stdexcept>
#include <stop_token>
#include <string>
#include <utility>
#include <vector>

#include "mippp/utility/deletion_filter.hpp"
#include "mippp/utility/iis_outcome.hpp"
#include "mippp/utility/variant.hpp"

#include "iis_outcome_assert.hpp"

using namespace mippp;
using namespace std::chrono_literals;

namespace {

using ids = std::vector<std::size_t>;
using seconds = std::chrono::duration<double>;

// Moved by hand, so that no deadline test sleeps or races a loaded machine.
struct fake_clock {
    using rep = std::int64_t;
    using period = std::nano;
    using duration = std::chrono::duration<rep, period>;
    using time_point = std::chrono::time_point<fake_clock>;
    static inline time_point current{};
    static time_point now() noexcept { return current; }
};

bool contains(std::span<const std::size_t> active, std::size_t id) {
    return std::ranges::find(active, id) != active.end();
}

template <typename O>
deletion_filter_result run_batched(std::size_t candidate_count, O & oracle,
                                   std::size_t batch_size,
                                   const iis_limits & limits = {}) {
    mippp::detail::deletion_state state;
    state.members.resize(candidate_count);
    std::iota(state.members.begin(), state.members.end(), std::size_t{0});
    return mippp::detail::run_deletion_filter(
        std::move(state), oracle, mippp::detail::make_deletion_budget(limits),
        batch_size);
}

enum class stop_event { none, stop_request, deadline, trial_limit };

// Call i gets verdict i of the script, and feasible past its end. The event
// fires during call at, or before the first call when at is 0.
deletion_filter_result run_scripted(
    const std::vector<deletion_verdict> & script, stop_event kind,
    std::size_t at, bool proven, std::size_t batch_size) {
    fake_clock::current = {};
    std::stop_source source;
    iis_limits limits{.time_limit = 1s, .stop_token = source.get_token()};
    if(kind == stop_event::trial_limit) limits.max_trials = at;
    if(kind == stop_event::stop_request && at == 0) source.request_stop();
    if(kind == stop_event::deadline && at == 0)
        limits.time_limit = seconds::zero();
    const auto budget = mippp::detail::make_deletion_budget<fake_clock>(limits);
    std::size_t calls = 0;
    auto oracle = [&](std::span<const std::size_t>) {
        const auto call = ++calls;
        if(kind == stop_event::stop_request && call == at)
            source.request_stop();
        if(kind == stop_event::deadline && call == at)
            fake_clock::current = budget.deadline;
        return call <= script.size() ? script[call - 1]
                                     : deletion_verdict::feasible;
    };
    return mippp::detail::run_deletion_filter({ids{0, 1, 2}, proven}, oracle,
                                              budget, batch_size);
}

struct lvalue_oracle {
    deletion_verdict operator()(std::span<const std::size_t>) & {
        return deletion_verdict::feasible;
    }
};
struct rvalue_oracle {
    deletion_verdict operator()(std::span<const std::size_t>) && {
        return deletion_verdict::feasible;
    }
};
struct boolean_oracle {
    bool operator()(std::span<const std::size_t>) { return true; }
};

}  // namespace

static_assert(deletion_oracle<lvalue_oracle>);
static_assert(!deletion_oracle<rvalue_oracle>);
static_assert(!deletion_oracle<boolean_oracle>);

///////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// Limits ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

TEST(iis_limits, default_is_unlimited) {
    const iis_limits limits;
    EXPECT_EQ(limits.max_trials, std::numeric_limits<std::size_t>::max());
    EXPECT_TRUE(std::isinf(limits.time_limit.count()));
    EXPECT_FALSE(limits.stop_token.stop_possible());
    EXPECT_EQ(mippp::detail::make_deletion_budget<fake_clock>(limits).deadline,
              fake_clock::time_point::max());
}

TEST(iis_limits, nan_or_negative_time_limit_throws_before_any_trial) {
    std::size_t calls = 0;
    auto oracle = [&calls](std::span<const std::size_t>) {
        ++calls;
        return deletion_verdict::infeasible;
    };
    for(const double limit : {std::numeric_limits<double>::quiet_NaN(), -1.0,
                              -std::numeric_limits<double>::infinity()})
        EXPECT_THROW(
            (void)deletion_filter(1, oracle, {.time_limit = seconds(limit)}),
            std::invalid_argument);
    EXPECT_EQ(calls, 0u);
}

TEST(iis_limits, time_limit_becomes_one_deadline) {
    const auto now = std::chrono::steady_clock::now();
    EXPECT_EQ(mippp::detail::make_deletion_budget({.time_limit = 250ms}, now)
                  .deadline,
              now + 250ms);
    const fake_clock::time_point start{1h};
    EXPECT_EQ(mippp::detail::make_deletion_budget<fake_clock>(
                  {.time_limit = seconds(1.5)}, start)
                  .deadline,
              start + 1500ms);
}

TEST(iis_limits, unbounded_time_limit_saturates_to_no_deadline) {
    const auto never = fake_clock::time_point::max();
    const fake_clock::time_point start{1h};
    for(const double limit : {std::numeric_limits<double>::infinity(),
                              std::numeric_limits<double>::max(), 1e300})
        EXPECT_EQ(mippp::detail::make_deletion_budget<fake_clock>(
                      {.time_limit = seconds(limit)}, start)
                      .deadline,
                  never);
    // finite in seconds, past the clock's range from this start
    const auto late = never - 1s;
    EXPECT_EQ(mippp::detail::make_deletion_budget<fake_clock>(
                  {.time_limit = 2s}, late)
                  .deadline,
              never);
    EXPECT_EQ(mippp::detail::make_deletion_budget<fake_clock>(
                  {.time_limit = 500ms}, late)
                  .deadline,
              late + 500ms);
}

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Outcomes ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

TEST(deletion_filter, feasible_initial_trial) {
    std::size_t calls = 0;
    const auto answer =
        deletion_filter(3, [&calls](std::span<const std::size_t> active) {
            ++calls;
            EXPECT_EQ(active.size(), 3u);
            return deletion_verdict::feasible;
        });
    EXPECT_TRUE(outcome_is<iis_outcome::feasible>(answer.outcome));
    EXPECT_TRUE(answer.members.empty());
    EXPECT_EQ(calls, 1u);
}

TEST(deletion_filter, infeasible_background_alone_is_an_empty_iis) {
    std::size_t calls = 0;
    const auto answer =
        deletion_filter(0, [&calls](std::span<const std::size_t>) {
            ++calls;
            return deletion_verdict::infeasible;
        });
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(answer.outcome));
    EXPECT_TRUE(answer.members.empty());
    EXPECT_EQ(calls, 1u);
}

TEST(deletion_filter, members_are_listed_in_ascending_order) {
    // swap/pop leaves this conflict as {0, 3, 2}
    auto oracle = [](std::span<const std::size_t> active) {
        return contains(active, 0) && contains(active, 2) && contains(active, 3)
                   ? deletion_verdict::infeasible
                   : deletion_verdict::feasible;
    };
    const auto answer = deletion_filter(5, oracle);
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(answer.outcome));
    EXPECT_EQ(answer.members, (ids{0, 2, 3}));
}

TEST(deletion_filter, inconclusive_initial_trial_proves_nothing) {
    std::size_t calls = 0;
    const auto answer =
        deletion_filter(2, [&calls](std::span<const std::size_t>) {
            ++calls;
            return deletion_verdict::inconclusive;
        });
    EXPECT_TRUE(
        outcome_is<iis_outcome::inconclusive_trial>(answer.outcome, false));
    EXPECT_TRUE(answer.members.empty());
    EXPECT_EQ(calls, 1u);
}

TEST(deletion_filter, inconclusive_singletons_keep_the_proven_set) {
    auto oracle = [](std::span<const std::size_t> active) {
        return active.size() == 3 ? deletion_verdict::infeasible
                                  : deletion_verdict::inconclusive;
    };
    const auto answer = deletion_filter(3, oracle);
    EXPECT_TRUE(
        outcome_is<iis_outcome::inconclusive_trial>(answer.outcome, true));
    EXPECT_EQ(answer.members, (ids{0, 1, 2}));
}

TEST(deletion_filter, move_only_oracle_and_exceptions) {
    auto oracle = [state = std::make_unique<int>(0)](
                      std::span<const std::size_t> active) {
        ++*state;
        return active.empty() ? deletion_verdict::feasible
                              : deletion_verdict::infeasible;
    };
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(
        deletion_filter(1, std::move(oracle)).outcome));
    struct oracle_error {};
    auto throws = [](std::span<const std::size_t>) -> deletion_verdict {
        throw oracle_error{};
    };
    EXPECT_THROW((void)deletion_filter(1, throws), oracle_error);
}

TEST(deletion_filter, exhaustive_monotone_oracles) {
    // Every family of conflicts over four candidates, empty and overlapping
    // conflicts included. A set is infeasible when it contains a conflict of
    // the family.
    const std::size_t batch_sizes[] = {1, 0, 2, 3, 4, 32};
    for(unsigned family = 0; family < (1u << 16); ++family) {
        std::size_t calls = 0;
        auto oracle = [family, &calls](std::span<const std::size_t> active) {
            ++calls;
            unsigned mask = 0;
            for(auto id : active) mask |= 1u << id;
            for(unsigned conflict = 0; conflict < 16; ++conflict)
                if((family & (1u << conflict)) && (mask & conflict) == conflict)
                    return deletion_verdict::infeasible;
            return deletion_verdict::feasible;
        };
        for(const auto batch : batch_sizes) {
            calls = 0;
            const auto answer = batch == 1 ? deletion_filter(4, oracle)
                                           : run_batched(4, oracle, batch);
            if(batch <= 1) {
                ASSERT_LE(calls, 5u)
                    << "family " << family << " batch " << batch;
            }
            if(family == 0) {
                ASSERT_TRUE(outcome_is<iis_outcome::feasible>(answer.outcome))
                    << "family " << family << " batch " << batch;
                ASSERT_TRUE(answer.members.empty())
                    << "family " << family << " batch " << batch;
                continue;
            }
            ASSERT_TRUE(outcome_is<iis_outcome::irreducible>(answer.outcome))
                << "family " << family << " batch " << batch;
            ASSERT_TRUE(std::ranges::is_sorted(answer.members))
                << "family " << family << " batch " << batch;
            ASSERT_EQ(oracle(answer.members), deletion_verdict::infeasible)
                << "family " << family << " batch " << batch;
            for(std::size_t i = 0; i < answer.members.size(); ++i) {
                auto subset = answer.members;
                subset.erase(subset.begin() + static_cast<std::ptrdiff_t>(i));
                ASSERT_EQ(oracle(subset), deletion_verdict::feasible)
                    << "family " << family << " batch " << batch << " member "
                    << answer.members[i];
            }
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// Stops ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

TEST(deletion_filter, limits_stop_between_trials) {
    std::size_t calls = 0;
    auto oracle = [&calls](std::span<const std::size_t>) {
        ++calls;
        return deletion_verdict::infeasible;
    };
    auto answer = deletion_filter(3, oracle, {.max_trials = 2});
    EXPECT_TRUE(outcome_is<iis_outcome::trial_limit>(answer.outcome, true));
    EXPECT_EQ(answer.members, (ids{1, 2}));
    EXPECT_EQ(calls, 2u);

    calls = 0;
    answer = deletion_filter(3, oracle, {.max_trials = 0});
    EXPECT_TRUE(outcome_is<iis_outcome::trial_limit>(answer.outcome, false));
    EXPECT_TRUE(answer.members.empty());
    answer = deletion_filter(3, oracle, {.time_limit = seconds::zero()});
    EXPECT_TRUE(outcome_is<iis_outcome::time_limit>(answer.outcome, false));
    std::stop_source source;
    source.request_stop();
    answer = deletion_filter(3, oracle, {.stop_token = source.get_token()});
    EXPECT_TRUE(outcome_is<iis_outcome::interrupted>(answer.outcome, false));
    EXPECT_EQ(calls, 0u);
}

TEST(deletion_filter, stop_request_beats_deadline_beats_trial_limit) {
    std::size_t calls = 0;
    auto oracle = [&calls](std::span<const std::size_t>) {
        ++calls;
        return deletion_verdict::infeasible;
    };
    std::stop_source source;
    const iis_limits limits{.max_trials = 0,
                            .time_limit = seconds::zero(),
                            .stop_token = source.get_token()};
    EXPECT_TRUE(outcome_is<iis_outcome::time_limit>(
        deletion_filter(1, oracle, limits).outcome, false));
    source.request_stop();
    EXPECT_TRUE(outcome_is<iis_outcome::interrupted>(
        deletion_filter(1, oracle, limits).outcome, false));
    EXPECT_EQ(calls, 0u);
}

TEST(deletion_filter, proof_on_the_last_permitted_trial_is_complete) {
    std::size_t calls = 0;
    auto oracle = [&calls](std::span<const std::size_t> active) {
        ++calls;
        return active.empty() ? deletion_verdict::feasible
                              : deletion_verdict::infeasible;
    };
    const auto answer = deletion_filter(1, oracle, {.max_trials = 2});
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(answer.outcome));
    EXPECT_EQ(answer.members, (ids{0}));
    EXPECT_EQ(calls, 2u);
}

TEST(deletion_filter, inconclusive_last_trial_is_not_a_trial_limit) {
    auto singleton_inconclusive = [](std::span<const std::size_t> active) {
        return active.empty() ? deletion_verdict::inconclusive
                              : deletion_verdict::infeasible;
    };
    auto answer = deletion_filter(1, singleton_inconclusive, {.max_trials = 2});
    EXPECT_TRUE(
        outcome_is<iis_outcome::inconclusive_trial>(answer.outcome, true));
    EXPECT_EQ(answer.members, (ids{0}));

    auto initial_inconclusive = [](std::span<const std::size_t>) {
        return deletion_verdict::inconclusive;
    };
    answer = deletion_filter(1, initial_inconclusive, {.max_trials = 1});
    EXPECT_TRUE(
        outcome_is<iis_outcome::inconclusive_trial>(answer.outcome, false));
    EXPECT_TRUE(answer.members.empty());
}

TEST(deletion_filter, a_later_stop_replaces_an_inconclusive_trial) {
    // the first singleton trial is inconclusive, the budget stops the second
    auto oracle = [](std::span<const std::size_t> active) {
        return active.size() == 2 && !contains(active, 0)
                   ? deletion_verdict::inconclusive
                   : deletion_verdict::infeasible;
    };
    const auto answer = deletion_filter(3, oracle, {.max_trials = 2});
    EXPECT_TRUE(outcome_is<iis_outcome::trial_limit>(answer.outcome, true));
    EXPECT_EQ(answer.members, (ids{0, 1, 2}));
}

TEST(deletion_filter, stop_requested_during_a_trial_keeps_the_proven_set) {
    std::stop_source source;
    std::size_t calls = 0;
    auto oracle = [&](std::span<const std::size_t>) {
        ++calls;
        source.request_stop();
        return deletion_verdict::infeasible;
    };
    const auto answer =
        deletion_filter(3, oracle, {.stop_token = source.get_token()});
    EXPECT_TRUE(outcome_is<iis_outcome::interrupted>(answer.outcome, true));
    EXPECT_EQ(answer.members, (ids{0, 1, 2}));
    EXPECT_EQ(calls, 1u);
}

TEST(deletion_filter, deadline_passing_inside_the_last_trial) {
    for(const bool final_proof : {false, true}) {
        fake_clock::current = {};
        const auto budget =
            mippp::detail::make_deletion_budget<fake_clock>({.time_limit = 1s});
        auto oracle = [&](std::span<const std::size_t> active) {
            if(!active.empty()) return deletion_verdict::infeasible;
            fake_clock::current = budget.deadline;
            return final_proof ? deletion_verdict::feasible
                               : deletion_verdict::inconclusive;
        };
        const auto answer =
            mippp::detail::run_deletion_filter({ids{0}}, oracle, budget);
        EXPECT_EQ(answer.members, (ids{0}));
        if(final_proof) {
            EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(answer.outcome));
        } else {
            EXPECT_TRUE(
                outcome_is<iis_outcome::time_limit>(answer.outcome, true));
        }
    }
}

TEST(deletion_filter, deadline_passing_inside_the_initial_trial) {
    for(const auto verdict :
        {deletion_verdict::infeasible, deletion_verdict::inconclusive}) {
        fake_clock::current = {};
        const auto budget =
            mippp::detail::make_deletion_budget<fake_clock>({.time_limit = 1s});
        std::size_t calls = 0;
        auto oracle = [&](std::span<const std::size_t>) {
            ++calls;
            fake_clock::current = budget.deadline;
            return verdict;
        };
        const auto answer =
            mippp::detail::run_deletion_filter({ids{0, 1, 2}}, oracle, budget);
        EXPECT_EQ(calls, 1u);
        if(verdict == deletion_verdict::infeasible) {
            EXPECT_TRUE(
                outcome_is<iis_outcome::time_limit>(answer.outcome, true));
            EXPECT_EQ(answer.members, (ids{0, 1, 2}));
        } else {
            EXPECT_TRUE(
                outcome_is<iis_outcome::time_limit>(answer.outcome, false));
            EXPECT_TRUE(answer.members.empty());
        }
    }
}

TEST(deletion_filter, the_flag_is_set_exactly_when_members_are_returned) {
    // Every script of five verdicts, monotone or not, against a stop request,
    // the deadline or the solve limit at every call reaches every return.
    constexpr std::size_t max_calls = 5;  // three candidates, batches of two
    std::vector<std::pair<stop_event, std::size_t>> events{
        {stop_event::none, 0}};
    for(const auto kind : {stop_event::stop_request, stop_event::deadline,
                           stop_event::trial_limit})
        for(std::size_t at = 0; at <= max_calls; ++at)
            events.emplace_back(kind, at);
    std::vector<std::vector<deletion_verdict>> scripts{{}};
    for(std::size_t call = 0; call < max_calls; ++call) {
        std::vector<std::vector<deletion_verdict>> longer;
        for(const auto & script : scripts)
            for(const auto verdict :
                {deletion_verdict::feasible, deletion_verdict::infeasible,
                 deletion_verdict::inconclusive}) {
                longer.push_back(script);
                longer.back().push_back(verdict);
            }
        scripts = std::move(longer);
    }
    std::set<std::string> seen;
    for(const bool proven : {false, true})
        for(const std::size_t batch : {std::size_t{1}, std::size_t{2}})
            for(std::size_t index = 0; index < scripts.size(); ++index)
                for(const auto & [kind, at] : events) {
                    const auto answer =
                        run_scripted(scripts[index], kind, at, proven, batch);
                    const auto text =
                        iis_outcome_assert_detail::describe(answer.outcome);
                    ASSERT_EQ(iis_outcome::conflict_available(answer.outcome),
                              !answer.members.empty() ||
                                  is<iis_outcome::irreducible>(answer.outcome))
                        << text << " with " << answer.members.size()
                        << " members, proven " << proven << " batch " << batch
                        << " script " << index << " event "
                        << static_cast<int>(kind) << " at " << at;
                    seen.insert(text);
                }
    // every return, and never the unwritten incomplete
    EXPECT_EQ(seen,
              (std::set<std::string>{
                  "feasible", "irreducible", "inconclusive_trial (no conflict)",
                  "inconclusive_trial (conflict held)",
                  "interrupted (no conflict)", "interrupted (conflict held)",
                  "time_limit (no conflict)", "time_limit (conflict held)",
                  "trial_limit (no conflict)", "trial_limit (conflict held)"}));
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////// Continuation from a proof ///////////////////////////
///////////////////////////////////////////////////////////////////////////////

TEST(deletion_filter, continuation_skips_the_known_proof) {
    auto infeasible = [](std::span<const std::size_t>) {
        return deletion_verdict::infeasible;
    };
    const auto known = deletion_filter(3, infeasible, {.max_trials = 1});
    ASSERT_TRUE(outcome_is<iis_outcome::trial_limit>(known.outcome, true));
    ASSERT_EQ(known.members, (ids{0, 1, 2}));

    auto resume_stopped = [&known](const iis_limits & limits) {
        std::size_t calls = 0;
        auto counted = [&calls](std::span<const std::size_t>) {
            ++calls;
            return deletion_verdict::inconclusive;
        };
        const auto answer = mippp::detail::run_deletion_filter(
            {known.members, true}, counted,
            mippp::detail::make_deletion_budget(limits));
        EXPECT_EQ(calls, 0u);
        EXPECT_EQ(answer.members, known.members);
        return answer.outcome;
    };
    EXPECT_TRUE(outcome_is<iis_outcome::trial_limit>(
        resume_stopped({.max_trials = 0}), true));
    EXPECT_TRUE(outcome_is<iis_outcome::time_limit>(
        resume_stopped({.time_limit = seconds::zero()}), true));
    std::stop_source source;
    source.request_stop();
    EXPECT_TRUE(outcome_is<iis_outcome::interrupted>(
        resume_stopped({.stop_token = source.get_token()}), true));

    std::size_t calls = 0;
    auto inconclusive = [&calls](std::span<const std::size_t> active) {
        ++calls;
        EXPECT_EQ(active.size(), 2u);
        return deletion_verdict::inconclusive;
    };
    const auto answer = mippp::detail::run_deletion_filter(
        {known.members, true}, inconclusive,
        mippp::detail::make_deletion_budget({}));
    EXPECT_EQ(calls, 3u);
    EXPECT_TRUE(
        outcome_is<iis_outcome::inconclusive_trial>(answer.outcome, true));
    EXPECT_EQ(answer.members, known.members);
}

///////////////////////////////////////////////////////////////////////////////
/////////////////////////////// Dormant batching //////////////////////////////
///////////////////////////////////////////////////////////////////////////////

TEST(deletion_filter, batching_reduces_trials_on_a_sparse_conflict) {
    std::size_t calls = 0;
    auto oracle = [&calls](std::span<const std::size_t> active) {
        ++calls;
        return contains(active, 17) && contains(active, 900)
                   ? deletion_verdict::infeasible
                   : deletion_verdict::feasible;
    };
    const auto single = deletion_filter(1024, oracle);
    const auto single_calls = std::exchange(calls, 0);
    const auto batched = run_batched(1024, oracle, 64);
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(single.outcome));
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(batched.outcome));
    EXPECT_EQ(single.members, (ids{17, 900}));
    EXPECT_EQ(batched.members, single.members);
    EXPECT_LT(calls, single_calls / 4);
}

TEST(deletion_filter, inconclusive_batches_still_give_an_irreducible_answer) {
    auto oracle = [](std::span<const std::size_t> active) {
        if(active.size() == 4) return deletion_verdict::infeasible;
        if(active.size() == 2) return deletion_verdict::inconclusive;
        return deletion_verdict::feasible;
    };
    const auto answer = run_batched(4, oracle, 2);
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(answer.outcome));
    EXPECT_EQ(answer.members, (ids{0, 1, 2, 3}));
}

TEST(deletion_filter, batch_budget_keeps_only_proven_deletions) {
    std::size_t calls = 0;
    auto oracle = [&calls](std::span<const std::size_t> active) {
        ++calls;
        return contains(active, 7) ? deletion_verdict::infeasible
                                   : deletion_verdict::feasible;
    };
    for(std::size_t budget = 0; budget < 16; ++budget) {
        calls = 0;
        const auto answer = run_batched(8, oracle, 4, {.max_trials = budget});
        EXPECT_LE(calls, budget);
        if(iis_outcome::conflict_available(answer.outcome)) {
            EXPECT_TRUE(contains(answer.members, 7));
        }
        if(is<iis_outcome::irreducible>(answer.outcome)) {
            EXPECT_EQ(answer.members, (ids{7}));
        } else {
            // only a stop before the initial trial holds no conflict
            EXPECT_TRUE(outcome_is<iis_outcome::trial_limit>(answer.outcome,
                                                             budget > 0));
        }
    }
}

TEST(deletion_filter, stop_after_an_inconclusive_batch_keeps_the_full_set) {
    std::stop_source source;
    std::size_t calls = 0;
    auto oracle = [&](std::span<const std::size_t>) {
        if(++calls == 1) return deletion_verdict::infeasible;
        source.request_stop();
        return deletion_verdict::inconclusive;
    };
    const auto answer =
        run_batched(8, oracle, 4, {.stop_token = source.get_token()});
    EXPECT_TRUE(outcome_is<iis_outcome::interrupted>(answer.outcome, true));
    EXPECT_EQ(answer.members, (ids{0, 1, 2, 3, 4, 5, 6, 7}));
    EXPECT_EQ(calls, 2u);
}
