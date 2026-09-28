#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstddef>
#include <limits>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "mippp/detail/handle_status_table.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"
#include "mippp/utility/iis_outcome.hpp"
#include "mippp/utility/iis_snapshot.hpp"

#include "test_suites/iis_oracle.hpp"
#include "test_suites/iis_vectors.hpp"

using namespace mippp;
using namespace mippp::iis_oracle;

namespace {

constexpr auto none = std::nullopt;

// Its IISs: variable 2 crossed, row 0 crossed, and row 0's lower side against
// the upper sides of variables 1 and 2.
linear_system incompatible_bounds() {
    return published_vectors().front().system;
}

// x >= 0 against x == -1, where only the upper side of the row is needed
const linear_system equality_row{{{0., none}}, {{{{0, 1.}}, -1., -1.}}};

using enum membership;

std::string failure_of(const ::testing::AssertionResult & result) {
    return result ? std::string() : std::string(result.message());
}

std::string domain_error_of(const linear_system & system) {
    try {
        (void)is_feasible(system, all_sides(system));
    } catch(const std::domain_error & e) {
        return e.what();
    }
    return {};
}

}  // namespace

///////////////////////////////////////////////////////////////////////////////
/////////////////////////////// Elimination ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

TEST(iis_oracle, published_vectors_have_exactly_the_listed_conflicts) {
    for(const auto & input : published_vectors()) {
        SCOPED_TRACE(input.name);
        const auto candidates = all_sides(input.system);
        EXPECT_FALSE(is_feasible(input.system, candidates));
        EXPECT_TRUE(is_feasible(input.system, {}));
        std::set<std::size_t> sizes;
        for(std::size_t mask = 0; mask < (std::size_t{1} << candidates.size());
            ++mask) {
            std::vector<side> subset;
            for(std::size_t i = 0; i < candidates.size(); ++i)
                if(mask & (std::size_t{1} << i))
                    subset.push_back(candidates[i]);
            if(is_feasible(input.system, subset)) continue;
            bool minimal = true;
            for(std::size_t i = 0; i < subset.size(); ++i) {
                auto rest = subset;
                rest.erase(rest.begin() + static_cast<std::ptrdiff_t>(i));
                minimal = minimal && is_feasible(input.system, rest);
            }
            if(minimal) sizes.insert(subset.size());
        }
        EXPECT_EQ(sizes, std::set<std::size_t>(input.iis_sizes.begin(),
                                               input.iis_sizes.end()));
    }
}

TEST(iis_oracle, decides_a_system_without_variables) {
    const linear_system empty_rows{{}, {{{}, 0., 0.}, {{}, 1., none}}};
    const std::vector<side> row_0{{side_kind::row_lower, 0},
                                  {side_kind::row_upper, 0}};
    EXPECT_TRUE(is_feasible(empty_rows, row_0));
    EXPECT_FALSE(is_feasible(empty_rows, all_sides(empty_rows)));
}

TEST(iis_oracle, a_rounded_sum_throws) {
    // 1 * 3 and -1 * 0.1 are exact, 3 - 0.1 is not
    const linear_system tenths{{{none, none}},
                               {{{{0, 0.1}}, none, 1.}, {{{0, 3.}}, 1., none}}};
    EXPECT_THAT(domain_error_of(tenths), ::testing::HasSubstr("a sum rounds"));
}

TEST(iis_oracle, a_rounded_product_throws) {
    // -0.1 * 0.1 rounds, adding it to 0 * 3 does not
    const linear_system tenths{
        {{none, none}}, {{{{0, 0.1}}, none, 0.}, {{{0, 3.}}, 0.1, none}}};
    EXPECT_THAT(domain_error_of(tenths),
                ::testing::HasSubstr("a product rounds"));
}

TEST(iis_oracle, rejects_infinite_data) {
    const linear_system infinite{
        {{0., std::numeric_limits<double>::infinity()}}, {}};
    EXPECT_THROW((void)is_feasible(infinite, all_sides(infinite)),
                 std::invalid_argument);
}

TEST(iis_oracle, rejects_a_term_of_a_missing_variable) {
    const linear_system one_variable{{{0., 1.}}, {{{{1, 1.}}, none, -1.}}};
    EXPECT_THROW((void)is_feasible(one_variable, all_sides(one_variable)),
                 std::out_of_range);
}

TEST(iis_oracle, rejects_more_than_six_variables) {
    linear_system system;
    system.variables.assign(max_variables, {0., none});
    EXPECT_TRUE(is_feasible(system, all_sides(system)));
    system.variables.push_back({0., none});
    EXPECT_THROW((void)is_feasible(system, all_sides(system)),
                 std::length_error);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////// Answers /////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

TEST(iis_oracle, accepts_every_iis_of_a_case) {
    const auto system = incompatible_bounds();
    EXPECT_TRUE(is_iis(system, {{absent, absent, both}, {absent, absent}}));
    EXPECT_TRUE(is_iis(system, {{absent, absent, absent}, {both, absent}}));
    EXPECT_TRUE(is_iis(system, {{absent, absent, absent}, {whole, absent}}));
    EXPECT_TRUE(is_iis(system, {{absent, upper, upper}, {lower, absent}}));
}

TEST(iis_oracle, rejects_a_feasible_answer) {
    EXPECT_THAT(failure_of(is_iis(incompatible_bounds(),
                                  {{absent, absent, lower}, {absent, absent}})),
                ::testing::HasSubstr("the reported sides are feasible"));
    EXPECT_THAT(
        failure_of(is_iis(incompatible_bounds(),
                          {{absent, absent, absent}, {absent, absent}})),
        ::testing::HasSubstr("the reported sides are feasible"));
}

TEST(iis_oracle, rejects_a_reducible_answer) {
    EXPECT_THAT(
        failure_of(is_iis(incompatible_bounds(),
                          {{lower, absent, both}, {absent, absent}})),
        ::testing::HasSubstr("still infeasible without variable 0 lower"));
}

TEST(iis_oracle, a_member_row_is_kept_whole) {
    EXPECT_THAT(
        failure_of(is_iis(incompatible_bounds(),
                          {{absent, upper, upper}, {whole, absent}})),
        ::testing::HasSubstr("still infeasible without variable 1 upper"));
}

TEST(iis_oracle, member_both_drops_each_side_in_turn) {
    EXPECT_TRUE(is_iis(equality_row, {{lower}, {whole}}));
    EXPECT_TRUE(is_iis(equality_row, {{lower}, {upper}}));
    EXPECT_THAT(failure_of(is_iis(equality_row, {{lower}, {both}})),
                ::testing::HasSubstr("still infeasible without row 0 lower"));
}

TEST(iis_oracle, rejects_a_side_the_case_does_not_have) {
    const auto lp_get_iis = published_vectors().back().system;
    EXPECT_TRUE(is_iis(lp_get_iis, {{lower, lower}, {absent, absent, upper}}));
    EXPECT_THAT(failure_of(is_iis(lp_get_iis,
                                  {{lower, lower}, {absent, absent, both}})),
                ::testing::HasSubstr("does not have: row 2"));
    EXPECT_THAT(failure_of(is_iis(lp_get_iis,
                                  {{both, lower}, {absent, absent, upper}})),
                ::testing::HasSubstr("does not have: variable 0"));
    const linear_system free_variable{{{none, none}, {0., -1.}}, {}};
    EXPECT_THAT(failure_of(is_iis(free_variable, {{whole, both}, {}})),
                ::testing::HasSubstr("does not have: variable 0"));
}

TEST(iis_oracle, rejects_an_answer_of_another_shape) {
    EXPECT_THAT(failure_of(is_iis(incompatible_bounds(),
                                  {{absent, absent, both}, {absent}})),
                ::testing::HasSubstr("the case has 3 and 2"));
}

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// Snapshots ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

namespace {

using variable = model_variable<int, double>;
using constraint = model_constraint<int>;
using sided_status =
    std::variant<iis_status::absent, iis_status::member_lower,
                 iis_status::member_upper, iis_status::member_both>;
using row_status = std::variant<iis_status::absent, iis_status::member_lower,
                                iis_status::member_upper, iis_status::member>;
using snapshot = iis_snapshot<variable, constraint, sided_status, row_status>;

}  // namespace

TEST(iis_oracle, reads_each_tag_by_its_rule) {
    EXPECT_EQ(membership_of(sided_status{iis_status::absent{}}), absent);
    EXPECT_EQ(membership_of(sided_status{iis_status::member_lower{}}), lower);
    EXPECT_EQ(membership_of(sided_status{iis_status::member_upper{}}), upper);
    EXPECT_EQ(membership_of(sided_status{iis_status::member_both{}}), both);
    EXPECT_EQ(membership_of(row_status{iis_status::member{}}), whole);
}

TEST(iis_oracle, reads_a_snapshot_by_case_local_position) {
    mippp::detail::handle_status_table<sided_status> variables(3);
    variables.set(0, iis_status::member_upper{});
    variables.set(1, iis_status::member_upper{});
    mippp::detail::handle_status_table<row_status> rows(2);
    rows.set(1, iis_status::member_lower{});
    const snapshot iis(std::move(variables), std::move(rows),
                       iis_outcome::irreducible);
    // built in reverse: case variable i is handle 2 - i, row j is handle 1 - j
    const std::vector<variable> case_variables{variable(2), variable(1),
                                               variable(0)};
    const std::vector<constraint> case_rows{constraint(1), constraint(0)};
    const auto answer = read_answer(iis, case_variables, case_rows);
    EXPECT_EQ(answer.variables, (std::vector{absent, upper, upper}));
    EXPECT_EQ(answer.rows, (std::vector{lower, absent}));
    EXPECT_TRUE(is_iis(incompatible_bounds(), answer));
}
