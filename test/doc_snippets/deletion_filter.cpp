// The code of docs/algorithms/deletion-filter.md. The page shows the sections
// between the --8<-- markers, so they stay plain user code; the tests below
// them check what the page says about their output and outcome.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <fstream>
#include <optional>
#include <ostream>
#include <span>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "iis_outcome_assert.hpp"
#include "mippp/solvers/highs/all.hpp"
#include "mippp/utility/deletion_filter.hpp"
#include "mippp/utility/iis_by_deletion.hpp"

using namespace mippp;

#include "test_suites/all.hpp"

namespace deletion_filter_page {

// --8<-- [start:timing-rules]
enum event : std::size_t { nine_am, talk_start, talk_end, lunch, num_events };

// `to` comes at least `at_least` and at most `at_most` minutes after `from`
struct timing_rule {
    std::string_view text;
    event from;
    event to;
    std::optional<int> at_least;
    std::optional<int> at_most;
};
// --8<-- [end:timing-rules]

std::vector<timing_rule> morning_rules() {
    // --8<-- [start:timing-data]
    const std::vector<timing_rule> rules = {
        {"the talk starts at 9:00 or later", nine_am, talk_start, 0, {}},
        {"the talk starts by 9:30", nine_am, talk_start, {}, 30},
        {"the talk lasts 50 to 60 minutes", talk_start, talk_end, 50, 60},
        {"lunch is 20 minutes or more after the talk", talk_end, lunch, 20, {}},
        {"lunch starts at 9:45 or later", nine_am, lunch, 45, {}},
        {"lunch starts by 10:00", nine_am, lunch, {}, 60},
    };
    // --8<-- [end:timing-data]
    return rules;
}

auto schedule_oracle(const std::vector<timing_rule> & rules) {
    // --8<-- [start:timing-oracle]
    auto schedulable = [&rules](std::span<const std::size_t> active) {
        std::vector<int> time(num_events, 0);
        for(std::size_t round = 0; round < num_events; ++round) {
            bool lowered = false;
            for(const std::size_t k : active) {
                const timing_rule & rule = rules[k];
                if(rule.at_most &&
                   time[rule.from] + *rule.at_most < time[rule.to]) {
                    time[rule.to] = time[rule.from] + *rule.at_most;
                    lowered = true;
                }
                if(rule.at_least &&
                   time[rule.to] - *rule.at_least < time[rule.from]) {
                    time[rule.from] = time[rule.to] - *rule.at_least;
                    lowered = true;
                }
            }
            // nothing left to lower: time is a schedule of the active rules
            if(!lowered) return deletion_verdict::feasible;
        }
        // still lowering after one round per event: a cycle of rules has a
        // negative length, and no schedule exists
        return deletion_verdict::infeasible;
    };
    // --8<-- [end:timing-oracle]
    return schedulable;
}

deletion_filter_result print_morning_conflict(std::ostream & out) {
    const std::vector<timing_rule> rules = morning_rules();
    auto schedulable = schedule_oracle(rules);
    // --8<-- [start:timing-filter]
    const auto answer = deletion_filter(rules.size(), schedulable);
    for(const std::size_t k : answer.members) out << rules[k].text << '\n';
    // --8<-- [end:timing-filter]
    return answer;
}

// Search stands for the reader's own check: given the active candidates and
// the time left, it answers inconclusive when that time runs out.
template <typename Search>
deletion_filter_result filter_within_budget(std::size_t candidate_count,
                                            Search && search) {
    // --8<-- [start:own-deadline]
    using std::chrono::steady_clock;
    const auto budget = std::chrono::seconds(30);
    const auto deadline = steady_clock::now() + budget;
    auto bounded = [&](std::span<const std::size_t> active) {
        const auto left = deadline - steady_clock::now();
        if(left <= steady_clock::duration::zero())
            return deletion_verdict::inconclusive;
        return search(active, left);
    };
    const auto answer =
        deletion_filter(candidate_count, bounded, {.time_limit = budget});
    // --8<-- [end:own-deadline]
    return answer;
}

template <typename Model>
auto analyze_within_a_minute(Model & model) {
    // --8<-- [start:forwarded-limit]
    using namespace std::chrono_literals;
    model.set_time_limit(5s);
    const auto iis = compute_iis_by_deletion(model, {.time_limit = 1min});
    // --8<-- [end:forwarded-limit]
    return iis;
}

}  // namespace deletion_filter_page

namespace {

using namespace deletion_filter_page;

// The page includes the same file under the example's code.
std::string page_output(const char * name) {
    std::ifstream file(std::string(MIPPP_DOC_SNIPPETS_DIR "/") + name);
    // std::string(istreambuf_iterator...) draws a false -Wnull-dereference
    // from gcc 15
    std::ostringstream text;
    text << file.rdbuf();
    return text.str();
}

bool contains(std::span<const std::size_t> active, std::size_t k) {
    return std::ranges::find(active, k) != active.end();
}

struct deletion_filter_page_highs_lp : model_test<highs_api, highs_lp> {
    static void SetUpTestSuite() { construct_api("HIGHS"); }
};

// x and y in [0, 1] with x + y >= 3: five finite sides, three in the IIS.
template <typename Model>
void build_overfull_model(Model & model) {
    using namespace mippp::operators;
    auto x = model.add_variable({.lower_bound = 0, .upper_bound = 1});
    auto y = model.add_variable({.lower_bound = 0, .upper_bound = 1});
    model.add_constraint(x + y >= 3);
}

}  // namespace

TEST(deletion_filter_page, morning_prints_the_page_output) {
    std::ostringstream out;
    const auto answer = print_morning_conflict(out);
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(answer.outcome));
    EXPECT_EQ(answer.members, (std::vector<std::size_t>{0, 2, 3, 5}));
    EXPECT_EQ(out.str(), page_output("deletion_filter_morning.txt"));
}

TEST(deletion_filter_page,
     morning_run_calls_the_oracle_once_per_rule_plus_one) {
    const auto rules = morning_rules();
    auto schedulable = schedule_oracle(rules);
    std::size_t calls = 0;
    auto counted = [&](std::span<const std::size_t> active) {
        ++calls;
        return schedulable(active);
    };
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(
        deletion_filter(rules.size(), counted).outcome));
    EXPECT_EQ(rules.size(), 6u);
    EXPECT_EQ(calls, 7u);
}

// Every subset of the six rules, so the conflict the page explains by hand is
// the only one: its four rules conflict, and no other subset is an IIS.
TEST(deletion_filter_page, morning_has_a_single_iis) {
    const auto rules = morning_rules();
    auto schedulable = schedule_oracle(rules);
    const auto subset = [](unsigned mask) {
        std::vector<std::size_t> active;
        for(std::size_t k = 0; k < 6; ++k)
            if(mask & (1u << k)) active.push_back(k);
        return active;
    };
    std::vector<unsigned> iiss;
    for(unsigned mask = 0; mask < (1u << 6); ++mask) {
        if(schedulable(subset(mask)) != deletion_verdict::infeasible) continue;
        bool irreducible = true;
        for(std::size_t k = 0; k < 6; ++k)
            if((mask & (1u << k)) && schedulable(subset(mask & ~(1u << k))) !=
                                         deletion_verdict::feasible)
                irreducible = false;
        if(irreducible) iiss.push_back(mask);
    }
    EXPECT_EQ(iiss, (std::vector<unsigned>{0b101101u}));
}

// The page's warning: without monotonicity the members still conflict, but
// the irreducible claim can be false. Here {1, 2} is feasible while {2}
// alone is not, and the run keeps 0 because {1, 2} was feasible when 0 was
// tested.
TEST(deletion_filter_page, non_monotone_oracle_can_claim_a_false_iis) {
    auto helped = [](std::span<const std::size_t> active) {
        const bool all = active.size() == 3;
        return all || (contains(active, 2) && !contains(active, 1))
                   ? deletion_verdict::infeasible
                   : deletion_verdict::feasible;
    };
    const auto answer = deletion_filter(3, helped);
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(answer.outcome));
    EXPECT_EQ(answer.members, (std::vector<std::size_t>{0, 2}));
    const std::vector<std::size_t> smaller{2};
    EXPECT_EQ(helped(smaller), deletion_verdict::infeasible);
}

TEST(deletion_filter_page, own_deadline_finds_the_morning_conflict) {
    const auto rules = morning_rules();
    auto schedulable = schedule_oracle(rules);
    auto search = [&](std::span<const std::size_t> active,
                      std::chrono::steady_clock::duration) {
        return schedulable(active);
    };
    const auto answer = filter_within_budget(rules.size(), search);
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(answer.outcome));
    EXPECT_EQ(answer.members, (std::vector<std::size_t>{0, 2, 3, 5}));
}

// A search that always gives up proves nothing, and leaves no member.
TEST(deletion_filter_page, own_deadline_inconclusive_search_proves_nothing) {
    std::size_t calls = 0;
    auto give_up = [&](std::span<const std::size_t>,
                       std::chrono::steady_clock::duration left) {
        ++calls;
        EXPECT_GT(left, std::chrono::steady_clock::duration::zero());
        return deletion_verdict::inconclusive;
    };
    const auto answer = filter_within_budget(6, give_up);
    EXPECT_TRUE(
        outcome_is<iis_outcome::inconclusive_trial>(answer.outcome, false));
    EXPECT_TRUE(answer.members.empty());
    EXPECT_EQ(calls, 1u);
}

// One solve per finite side plus one, each under at most the model's own
// 5 s, which reads back exactly afterwards.
TEST_F(deletion_filter_page_highs_lp, forwarded_limit_is_capped_and_restored) {
    iis_trial_probe<highs_lp> model(*api);
    build_overfull_model(model);
    const auto iis = analyze_within_a_minute(model);
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
    EXPECT_EQ(iis.num_variable_members() + iis.num_constraint_members(), 3u);
    EXPECT_EQ(model.solves, 6u);
    ASSERT_EQ(model.trial_time_limits.size(), 6u);
    for(const double limit : model.trial_time_limits) {
        EXPECT_GT(limit, 0.);
        EXPECT_LE(limit, 5.);
    }
    EXPECT_EQ(model.get_time_limit(), std::chrono::duration<double>(5));
    EXPECT_TRUE(is<status::unknown>(model.get_status()));
}
