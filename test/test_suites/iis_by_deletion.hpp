#pragma once

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <stop_token>
#include <string>
#include <utility>
#include <vector>

#include "mippp/model_concepts.hpp"
#include "mippp/utility/iis_by_deletion.hpp"
#include "mippp/utility/iis_outcome.hpp"
#include "mippp/utility/variant.hpp"

#include "iis_cases.hpp"

namespace mippp {

struct iis_deletion_path {
    template <typename M>
    static auto compute(M & model) {
        return compute_iis_by_deletion(model);
    }
};

// Sees every trial: the free function deduces this type, so its solve() is
// the one the trials call. It records rather than asserts, so that the test
// that built the model reports the failure with its own context.
template <iis_by_deletion_model M>
struct iis_trial_probe : M {
    using M::M;
    std::size_t solves = 0;
    bool saw_nonzero_objective = false;
    bool saw_crossed_variable = false;
    bool saw_row_side_moved_to_a_finite_value = false;
    // the sides a trial may only leave in place or at infinity
    std::vector<std::pair<double, double>> original_row_sides;
    // the time limit each trial ran under, on a model that has one
    std::vector<double> trial_time_limits;

    void record_row_sides() {
        original_row_sides.clear();
        for(auto c : this->constraints())
            original_row_sides.emplace_back(
                this->get_constraint_lower_bound(c),
                this->get_constraint_upper_bound(c));
    }

    void solve() {
        ++solves;
        if constexpr(has_time_limit<M>)
            trial_time_limits.push_back(
                std::chrono::duration<double>(this->get_time_limit()).count());
        for(auto v : this->variables()) {
            if(this->get_objective_coefficient(v) != 0)
                saw_nonzero_objective = true;
            if(this->get_variable_lower_bound(v) >
               this->get_variable_upper_bound(v))
                saw_crossed_variable = true;
        }
        std::size_t i = 0;
        for(auto c : this->constraints()) {
            if(i == original_row_sides.size()) break;
            const auto [lower, upper] = original_row_sides[i++];
            const auto lb = this->get_constraint_lower_bound(c);
            const auto ub = this->get_constraint_upper_bound(c);
            if((lb != lower && !this->is_infinite(lb)) ||
               (ub != upper && !this->is_infinite(ub)))
                saw_row_side_moved_to_a_finite_value = true;
        }
        M::solve();
    }
};

template <typename T>
struct IisByDeletionTest : public iis_cases::fixture<T, iis_deletion_path> {
    using typename T::model_type;
    static_assert(iis_by_deletion_model<model_type>);
    static_assert(iis_by_deletion_model<iis_trial_probe<model_type>>);
    using probe = iis_trial_probe<model_type>;
    using iis_cases::fixture<T, iis_deletion_path>::save;
    using iis_cases::fixture<T, iis_deletion_path>::expect_unchanged;
    using iis_cases::fixture<T, iis_deletion_path>::answer_text;
    using membership = iis_cases::membership;
    using case_answer = iis_cases::case_answer;
    using linear_system = iis_cases::linear_system;
    template <typename M>
    using built_case = iis_cases::built_case<M>;
    static constexpr auto none = iis_cases::none;

    static constexpr std::chrono::duration<double> zero_seconds{0};

    // The folding is_iis does internally, which cannot be reused since it also
    // demands minimality.
    static std::vector<iis_oracle::side> reported_sides(
        const linear_system & system, const case_answer & answer) {
        using iis_oracle::side_kind;
        std::vector<iis_oracle::side> sides;
        const auto add = [&](membership m, bool has_lower, bool has_upper,
                             iis_oracle::side lower, iis_oracle::side upper) {
            const bool whole = m == membership::whole;
            if(has_lower &&
               (whole || m == membership::lower || m == membership::both))
                sides.push_back(lower);
            if(has_upper &&
               (whole || m == membership::upper || m == membership::both))
                sides.push_back(upper);
        };
        for(std::size_t i = 0; i < answer.variables.size(); ++i)
            add(answer.variables[i], system.variables[i].lower.has_value(),
                system.variables[i].upper.has_value(),
                {side_kind::variable_lower, i}, {side_kind::variable_upper, i});
        for(std::size_t i = 0; i < answer.rows.size(); ++i)
            add(answer.rows[i], system.rows[i].lower.has_value(),
                system.rows[i].upper.has_value(), {side_kind::row_lower, i},
                {side_kind::row_upper, i});
        return sides;
    }

    static void expect_probe_saw_nothing_wrong(const probe & model) {
        EXPECT_FALSE(model.saw_nonzero_objective);
        EXPECT_FALSE(model.saw_crossed_variable);
        EXPECT_FALSE(model.saw_row_side_moved_to_a_finite_value);
    }

    template <typename Iis>
    static void expect_stopped(const Iis & iis, iis_outcome outcome,
                               iis_reason reason) {
        EXPECT_EQ(iis.get_outcome(), outcome);
        EXPECT_EQ(iis.get_reason(), std::optional(reason));
    }

    void check_budget_sweep() {
        using namespace operators;
        // named through its namespace: MSVC does not bring in the enumerators
        // of the member alias inside a class template
        using enum iis_oracle::membership;
        probe model(*this->api);
        auto x0 = model.add_variable({.lower_bound = 0., .upper_bound = 1.});
        auto x1 = model.add_variable({.lower_bound = 0., .upper_bound = 1.});
        auto x2 = model.add_variable({.lower_bound = 0., .upper_bound = 1.});
        auto r0 = model.add_constraint(x0 + x1 >= 3.);
        auto r1 = model.add_constraint(x2 >= 2.);
        model.record_row_sides();
        const linear_system system{
            {{0., 1.}, {0., 1.}, {0., 1.}},
            {{{{0, 1.}, {1, 1.}}, 3., none}, {{{2, 1.}}, 2., none}}};
        const built_case<probe> built{{x0, x1, x2}, {r0, r1}};
        const iis_cases::saved_model_data before = save(model, built);

        const auto full = compute_iis_by_deletion(model);
        ASSERT_EQ(full.get_outcome(), iis_outcome::irreducible);
        EXPECT_EQ(full.get_reason(), std::nullopt);
        // the initial trial and one trial per side
        EXPECT_EQ(model.solves, 9u);
        const case_answer full_answer =
            iis_oracle::read_answer(full, built.variables, built.constraints);
        EXPECT_TRUE(iis_oracle::is_iis(system, full_answer))
            << answer_text(full_answer);
        expect_probe_saw_nothing_wrong(model);
        expect_unchanged(model, built, before);
        const std::vector<iis_oracle::side> full_sides =
            reported_sides(system, full_answer);

        for(std::size_t k = 0; k <= 10; ++k) {
            SCOPED_TRACE("max_solves = " + std::to_string(k));
            model.solves = 0;
            const auto iis =
                compute_iis_by_deletion(model, iis_limits{.max_solves = k});
            EXPECT_LE(model.solves, k);
            const case_answer answer = iis_oracle::read_answer(
                iis, built.variables, built.constraints);
            if(k == 0) {
                expect_stopped(iis, iis_outcome::undetermined,
                               iis_reason::solve_limit);
                EXPECT_EQ(answer, (case_answer{{absent, absent, absent},
                                               {absent, absent}}));
                EXPECT_EQ(iis.num_variable_members(), 0u);
                EXPECT_EQ(iis.num_constraint_members(), 0u);
            } else if(k == 1) {
                // the initial trial proved infeasibility and the stop kept
                // every candidate
                expect_stopped(iis, iis_outcome::not_proven_minimal,
                               iis_reason::solve_limit);
                EXPECT_EQ(answer,
                          (case_answer{{both, both, both}, {lower, lower}}));
            } else if(k < 9) {
                expect_stopped(iis, iis_outcome::not_proven_minimal,
                               iis_reason::solve_limit);
                const std::vector<iis_oracle::side> sides =
                    reported_sides(system, answer);
                EXPECT_FALSE(iis_oracle::is_feasible(system, sides))
                    << answer_text(answer);
                // a stop keeps the last proven subset, which the same
                // deterministic pass only shrinks towards the full answer
                for(const iis_oracle::side & s : full_sides)
                    EXPECT_TRUE(std::ranges::find(sides, s) != sides.end())
                        << answer_text(answer);
            } else {
                EXPECT_EQ(iis.get_outcome(), iis_outcome::irreducible);
                EXPECT_EQ(iis.get_reason(), std::nullopt);
                EXPECT_EQ(answer, full_answer);
                EXPECT_EQ(model.solves, 9u);
            }
            expect_probe_saw_nothing_wrong(model);
            expect_unchanged(model, built, before);
        }
    }

    void check_stop_requested_beforehand() {
        probe model(*this->api);
        const built_case<probe> built =
            build(model, iis_cases::bounds_against_a_row_case());
        model.solve();
        ASSERT_TRUE(is_a<status::infeasible>(model.get_status()));
        model.solves = 0;
        const iis_cases::saved_model_data before = save(model, built);
        std::stop_source source;
        source.request_stop();
        {
            const auto iis = compute_iis_by_deletion(
                model, iis_limits{.stop_token = source.get_token()});
            expect_stopped(iis, iis_outcome::undetermined,
                           iis_reason::cancelled);
            EXPECT_EQ(iis.num_variable_members(), 0u);
            EXPECT_EQ(iis.num_constraint_members(), 0u);
            EXPECT_EQ(model.solves, 0u);
            // a run that solved nothing leaves the status
            EXPECT_TRUE(is_a<status::infeasible>(model.get_status()));
            expect_unchanged(model, built, before);
        }
        {
            // a stop request beats the deadline, which beats the solve count
            const auto iis = compute_iis_by_deletion(
                model, iis_limits{.max_solves = 0,
                                  .time_limit = zero_seconds,
                                  .stop_token = source.get_token()});
            expect_stopped(iis, iis_outcome::undetermined,
                           iis_reason::cancelled);
            EXPECT_EQ(model.solves, 0u);
            EXPECT_TRUE(is_a<status::infeasible>(model.get_status()));
            expect_unchanged(model, built, before);
        }
    }

    void check_zero_second_budget() {
        probe model(*this->api);
        const built_case<probe> built =
            build(model, iis_cases::bounds_against_a_row_case());
        model.solve();
        ASSERT_TRUE(is_a<status::infeasible>(model.get_status()));
        model.solves = 0;
        const iis_cases::saved_model_data before = save(model, built);
        {
            const auto iis = compute_iis_by_deletion(
                model, iis_limits{.time_limit = zero_seconds});
            expect_stopped(iis, iis_outcome::undetermined,
                           iis_reason::time_limit);
            EXPECT_EQ(iis.num_variable_members(), 0u);
            EXPECT_EQ(iis.num_constraint_members(), 0u);
            EXPECT_EQ(model.solves, 0u);
            EXPECT_TRUE(is_a<status::infeasible>(model.get_status()));
            expect_unchanged(model, built, before);
        }
        {
            const auto iis = compute_iis_by_deletion(
                model, iis_limits{.max_solves = 0, .time_limit = zero_seconds});
            expect_stopped(iis, iis_outcome::undetermined,
                           iis_reason::time_limit);
            EXPECT_EQ(model.solves, 0u);
            EXPECT_TRUE(is_a<status::infeasible>(model.get_status()));
            expect_unchanged(model, built, before);
        }
    }

    void check_status_survives_a_run_without_a_solve() {
        using namespace operators;
        using M = model_type;
        {
            // the precheck answers alone
            auto model = this->new_model();
            constexpr auto & no_terms =
                empty_linear_expression<model_variable_t<M>, model_scalar_t<M>>;
            auto r0 = model.add_constraint(no_terms >= 1.);
            model.add_constraint(no_terms <= 2.);
            model.solve();
            const std::size_t before = model.get_status().index();
            const auto iis = compute_iis_by_deletion(model);
            EXPECT_EQ(iis.get_outcome(), iis_outcome::irreducible);
            EXPECT_EQ(iis_oracle::membership_of(iis.get_status(r0)),
                      membership::lower);
            EXPECT_EQ(model.get_status().index(), before);
        }
        {
            probe model(*this->api);
            build(model, iis_cases::bounds_against_a_row_case());
            model.solve();
            ASSERT_TRUE(is_a<status::infeasible>(model.get_status()));
            model.solves = 0;
            const auto iis =
                compute_iis_by_deletion(model, iis_limits{.max_solves = 0});
            expect_stopped(iis, iis_outcome::undetermined,
                           iis_reason::solve_limit);
            EXPECT_EQ(model.solves, 0u);
            EXPECT_TRUE(is_a<status::infeasible>(model.get_status()));
        }
        {
            // a crossed pair's continuation stopped before its trials solves
            // nothing either
            probe model(*this->api);
            build(model, iis_cases::crossed_variable_bounds_case());
            ASSERT_TRUE(is<status::unknown>(model.get_status()));
            const auto iis =
                compute_iis_by_deletion(model, iis_limits{.max_solves = 0});
            expect_stopped(iis, iis_outcome::not_proven_minimal,
                           iis_reason::solve_limit);
            EXPECT_EQ(model.solves, 0u);
            EXPECT_TRUE(is<status::unknown>(model.get_status()));
        }
    }

    void check_column_less_precheck_ignores_the_limits() {
        using namespace operators;
        using M = model_type;
        constexpr auto & no_terms =
            empty_linear_expression<model_variable_t<M>, model_scalar_t<M>>;
        std::stop_source source;
        source.request_stop();
        const iis_limits exhausted{.max_solves = 0,
                                   .time_limit = zero_seconds,
                                   .stop_token = source.get_token()};
        {
            auto model = this->new_model();
            auto r0 = model.add_constraint(no_terms >= 1.);
            auto r1 = model.add_constraint(no_terms <= 2.);
            const auto iis = compute_iis_by_deletion(model, exhausted);
            EXPECT_EQ(iis.get_outcome(), iis_outcome::irreducible);
            EXPECT_EQ(iis.get_reason(), std::nullopt);
            EXPECT_EQ(iis_oracle::membership_of(iis.get_status(r0)),
                      membership::lower);
            EXPECT_TRUE(is<iis_status::absent>(iis.get_status(r1)));
            EXPECT_EQ(iis.num_constraint_members(), 1u);
            EXPECT_EQ(iis.num_variable_members(), 0u);
        }
        {
            auto model = this->new_model();
            auto r0 = model.add_constraint(no_terms <= 2.);
            for(const iis_limits & limits : {exhausted, iis_limits{}}) {
                const auto iis = compute_iis_by_deletion(model, limits);
                EXPECT_EQ(iis.get_outcome(), iis_outcome::feasible);
                EXPECT_EQ(iis.get_reason(), std::nullopt);
                EXPECT_TRUE(is<iis_status::absent>(iis.get_status(r0)));
                EXPECT_EQ(iis.num_constraint_members(), 0u);
                EXPECT_EQ(iis.num_variable_members(), 0u);
            }
        }
    }

    void check_crossed_pair_under_a_budget() {
        using enum iis_oracle::membership;
        {
            probe model(*this->api);
            const built_case<probe> built =
                build(model, iis_cases::crossed_variable_bounds_case());
            model.record_row_sides();
            const iis_cases::saved_model_data before = save(model, built);
            const auto x0 = built.variables[0];
            const auto x1 = built.variables[1];
            const auto r0 = built.constraints[0];
            {
                const auto iis =
                    compute_iis_by_deletion(model, iis_limits{.max_solves = 0});
                expect_stopped(iis, iis_outcome::not_proven_minimal,
                               iis_reason::solve_limit);
                EXPECT_EQ(iis_oracle::membership_of(iis.get_status(x0)), both);
                EXPECT_EQ(iis_oracle::membership_of(iis.get_status(x1)),
                          absent);
                EXPECT_EQ(iis_oracle::membership_of(iis.get_status(r0)),
                          absent);
                EXPECT_EQ(iis.num_variable_members(), 1u);
                EXPECT_EQ(iis.num_constraint_members(), 0u);
                EXPECT_EQ(model.solves, 0u);
                expect_unchanged(model, built, before);
            }
            {
                // one side tested and kept, the other untested and kept
                model.solves = 0;
                const auto iis =
                    compute_iis_by_deletion(model, iis_limits{.max_solves = 1});
                expect_stopped(iis, iis_outcome::not_proven_minimal,
                               iis_reason::solve_limit);
                EXPECT_EQ(iis_oracle::membership_of(iis.get_status(x0)), both);
                EXPECT_EQ(model.solves, 1u);
                expect_unchanged(model, built, before);
            }
            {
                model.solves = 0;
                const auto iis = compute_iis_by_deletion(model);
                EXPECT_EQ(iis.get_outcome(), iis_outcome::irreducible);
                EXPECT_EQ(iis.get_reason(), std::nullopt);
                EXPECT_EQ(iis_oracle::membership_of(iis.get_status(x0)), both);
                EXPECT_EQ(iis_oracle::membership_of(iis.get_status(x1)),
                          absent);
                EXPECT_EQ(iis_oracle::membership_of(iis.get_status(r0)),
                          absent);
                EXPECT_EQ(model.solves, 2u);
                // each trial relaxes the other side
                expect_probe_saw_nothing_wrong(model);
                expect_unchanged(model, built, before);
            }
        }
        const iis_cases::iis_case crossed_row =
            iis_cases::crossed_term_less_row_case();
        if(!iis_cases::can_build<probe>(crossed_row))
            GTEST_SKIP() << "the crossed term-less row needs ranged rows";
        if(const auto reason = this->skip_reason(crossed_row))
            GTEST_SKIP() << *reason;
        probe model(*this->api);
        const built_case<probe> built = build(model, crossed_row);
        model.record_row_sides();
        const iis_cases::saved_model_data before = save(model, built);
        const auto r0 = built.constraints[0];
        {
            const auto iis = compute_iis_by_deletion(model);
            EXPECT_EQ(iis.get_outcome(), iis_outcome::irreducible);
            EXPECT_EQ(iis.get_reason(), std::nullopt);
            EXPECT_EQ(iis_oracle::membership_of(iis.get_status(r0)), lower);
            EXPECT_EQ(model.solves, 2u);
            expect_probe_saw_nothing_wrong(model);
            expect_unchanged(model, built, before);
        }
        {
            model.solves = 0;
            const auto iis =
                compute_iis_by_deletion(model, iis_limits{.max_solves = 0});
            expect_stopped(iis, iis_outcome::not_proven_minimal,
                           iis_reason::solve_limit);
            EXPECT_EQ(iis_oracle::membership_of(iis.get_status(r0)), both);
            EXPECT_EQ(model.solves, 0u);
            expect_unchanged(model, built, before);
        }
    }

    // The sense is not readable through a concept: the maximum of 69 after
    // the call pins it, since a zeroed objective would give 7 and the
    // minimum of the same model is another value.
    void check_restores_everything_it_saved() {
        using namespace operators;
        using M = model_type;
        auto model = this->new_model();
        auto x0 = model.add_variable({.lower_bound = 0., .upper_bound = 4.});
        auto x1 = model.add_variable({.lower_bound = -1.});
        auto x2 = add_integer_where_possible(model, {.upper_bound = 2.});
        model.set_maximization();
        model.set_objective(3 * x0 - x1 + 0.5 * x2 + 7);
        auto r0 = model.add_constraint(x0 - x1 >= 10.);
        auto r1 = add_ranged_where_possible(model, x1 + x2, -3., 3.);
        const built_case<M> built{{x0, x1, x2}, {r0, r1}};
        const linear_system system{
            {{0., 4.}, {-1., none}, {none, 2.}},
            {{{{0, 1.}, {1, -1.}}, 10., none},
             has_ranged_constraints<M>
                 ? linear_system::row{{{1, 1.}, {2, 1.}}, -3., 3.}
                 : linear_system::row{{{1, 1.}, {2, 1.}}, none, 3.}}};
        const iis_cases::saved_model_data before = save(model, built);
        ASSERT_EQ(before.objective_coefficients,
                  (std::vector<double>{3., -1., 0.5}));
        ASSERT_EQ(before.objective_offset, 7.);
        const auto iis = compute_iis_by_deletion(model);
        ASSERT_EQ(iis.get_outcome(), iis_outcome::irreducible);
        const case_answer answer =
            iis_oracle::read_answer(iis, built.variables, built.constraints);
        // two IISs exist here ({x0 upper, x1 lower, r0 lower} and one through
        // x2 and r1), so the answer is validated, not pinned
        EXPECT_TRUE(iis_oracle::is_iis(system, answer)) << answer_text(answer);
        expect_unchanged(model, built, before);
        if constexpr(has_verbosity<M>) {
            EXPECT_FALSE(model.is_verbose());
        }
        model.set_variable_upper_bound(x0, 20.);
        model.solve();
        ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
        EXPECT_NEAR(model.get_solution_value(), 69., TEST_EPSILON);
    }

    // A caller's limit shorter than the budget caps every trial, a longer one
    // gives way to the time that remains, and either reads back exactly after
    // the call.
    void check_forwarded_time_limit_is_restored() {
        if constexpr(!has_time_limit<model_type>) {
            GTEST_SKIP() << "no time limit";
        } else {
            const iis_limits budget{.time_limit = std::chrono::seconds(3600)};
            for(const double caller : {7., 7200.}) {
                SCOPED_TRACE("caller's limit " + std::to_string(caller));
                probe model(*this->api);
                build(model, iis_cases::bounds_against_a_row_case());
                model.set_time_limit(std::chrono::duration<double>(caller));
                const auto before = model.get_time_limit();
                const auto iis = compute_iis_by_deletion(model, budget);
                EXPECT_EQ(iis.get_outcome(), iis_outcome::irreducible);
                ASSERT_EQ(model.trial_time_limits.size(), model.solves);
                ASSERT_GT(model.solves, 0u);
                for(const double seen : model.trial_time_limits) {
                    if(caller < 3600.)
                        EXPECT_EQ(seen, caller);
                    else
                        EXPECT_TRUE(seen > 3500. && seen <= 3600.) << seen;
                }
                EXPECT_EQ(model.get_time_limit(), before);
            }
        }
    }

private:
    template <typename M>
    static auto add_integer_where_possible(
        M & model, const model_variable_params_t<M> & params) {
        if constexpr(milp_model<M>)
            return model.add_integer_variable(params);
        else
            return model.add_variable(params);
    }
    template <typename M, linear_expression LE>
    static auto add_ranged_where_possible(M & model, LE && lhs, double lower,
                                          double upper) {
        using namespace operators;
        if constexpr(has_ranged_constraints<M>)
            return model.add_ranged_constraint(std::forward<LE>(lhs), lower,
                                               upper);
        else
            return model.add_constraint(std::forward<LE>(lhs) <= upper);
    }
};
TYPED_TEST_SUITE_P(IisByDeletionTest);
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(IisByDeletionTest);

TYPED_TEST_P(IisByDeletionTest, bounds_against_a_row) {
    this->SkipOnLicenseError(
        [this]() { this->check_case(iis_cases::bounds_against_a_row_case()); });
}
TYPED_TEST_P(IisByDeletionTest, one_side_of_an_equality_row) {
    this->SkipOnLicenseError([this]() {
        this->check_case(iis_cases::one_side_of_an_equality_row_case());
    });
}
TYPED_TEST_P(IisByDeletionTest, integer_equal_to_one_half) {
    this->SkipOnLicenseError([this]() {
        this->check_case(iis_cases::integer_equal_to_one_half_case());
    });
}
TYPED_TEST_P(IisByDeletionTest, integers_summing_to_one_half) {
    this->SkipOnLicenseError([this]() {
        this->check_case(iis_cases::integers_summing_to_one_half_case());
    });
}
TYPED_TEST_P(IisByDeletionTest, integer_in_a_fractional_interval) {
    this->SkipOnLicenseError([this]() {
        this->check_case(iis_cases::integer_in_a_fractional_interval_case());
    });
}
TYPED_TEST_P(IisByDeletionTest, ranged_row_lower_side) {
    this->SkipOnLicenseError([this]() {
        this->check_case(iis_cases::ranged_row_lower_side_case());
    });
}
TYPED_TEST_P(IisByDeletionTest, ranged_row_upper_side) {
    this->SkipOnLicenseError([this]() {
        this->check_case(iis_cases::ranged_row_upper_side_case());
    });
}
TYPED_TEST_P(IisByDeletionTest, ranged_row_holding_no_integer) {
    this->SkipOnLicenseError([this]() {
        this->check_case(iis_cases::ranged_row_holding_no_integer_case());
    });
}
TYPED_TEST_P(IisByDeletionTest, redundant_rows) {
    this->SkipOnLicenseError(
        [this]() { this->check_case(iis_cases::redundant_rows_case()); });
}
TYPED_TEST_P(IisByDeletionTest, two_disjoint_conflicts) {
    this->SkipOnLicenseError([this]() {
        this->check_case(iis_cases::two_disjoint_conflicts_case());
    });
}
TYPED_TEST_P(IisByDeletionTest, chain_where_every_row_is_needed) {
    this->SkipOnLicenseError([this]() {
        this->check_case(iis_cases::chain_where_every_row_is_needed_case());
    });
}
TYPED_TEST_P(IisByDeletionTest, crossed_variable_bounds) {
    this->SkipOnLicenseError([this]() {
        this->check_case(iis_cases::crossed_variable_bounds_case());
    });
}
TYPED_TEST_P(IisByDeletionTest, crossed_row_sides) {
    this->SkipOnLicenseError(
        [this]() { this->check_case(iis_cases::crossed_row_sides_case()); });
}
TYPED_TEST_P(IisByDeletionTest, crossed_term_less_row) {
    this->SkipOnLicenseError([this]() {
        this->check_case(iis_cases::crossed_term_less_row_case());
    });
}
TYPED_TEST_P(IisByDeletionTest, feasible_model) {
    this->SkipOnLicenseError(
        [this]() { this->check_case(iis_cases::feasible_model_case()); });
}
TYPED_TEST_P(IisByDeletionTest, removed_variable_is_skipped) {
    this->SkipOnLicenseError(
        [this]() { this->check_removed_variable_is_skipped(); });
}
TYPED_TEST_P(IisByDeletionTest, answer_survives_a_later_removal) {
    this->SkipOnLicenseError(
        [this]() { this->check_answer_survives_a_later_removal(); });
}
TYPED_TEST_P(IisByDeletionTest, published_vectors_under_transforms) {
    this->SkipOnLicenseError([this]() { this->check_published_vectors(); });
}
TYPED_TEST_P(IisByDeletionTest, model_modified_after_an_infeasible_solve) {
    this->SkipOnLicenseError(
        [this]() { this->check_model_modified_after_an_infeasible_solve(); });
}
TYPED_TEST_P(IisByDeletionTest, model_data_and_result_survive_the_call) {
    this->SkipOnLicenseError(
        [this]() { this->check_model_data_and_result_survive_the_call(); });
}
TYPED_TEST_P(IisByDeletionTest, column_less_model_names_the_violated_row) {
    this->SkipOnLicenseError([this]() { this->check_column_less_model(); });
}

TYPED_TEST_P(IisByDeletionTest, budget_sweep_keeps_a_valid_answer) {
    this->SkipOnLicenseError([this]() { this->check_budget_sweep(); });
}
TYPED_TEST_P(IisByDeletionTest, stop_requested_beforehand_solves_nothing) {
    this->SkipOnLicenseError(
        [this]() { this->check_stop_requested_beforehand(); });
}
TYPED_TEST_P(IisByDeletionTest, zero_second_budget_solves_nothing) {
    this->SkipOnLicenseError([this]() { this->check_zero_second_budget(); });
}
TYPED_TEST_P(IisByDeletionTest, status_is_unknown_after_a_run_that_solved) {
    this->SkipOnLicenseError(
        [this]() { this->check_status_is_unknown_after_the_call(); });
}
TYPED_TEST_P(IisByDeletionTest, status_survives_a_run_without_a_solve) {
    this->SkipOnLicenseError(
        [this]() { this->check_status_survives_a_run_without_a_solve(); });
}
TYPED_TEST_P(IisByDeletionTest, column_less_precheck_ignores_the_limits) {
    this->SkipOnLicenseError(
        [this]() { this->check_column_less_precheck_ignores_the_limits(); });
}
TYPED_TEST_P(IisByDeletionTest, crossed_pair_is_the_proven_set_under_a_budget) {
    this->SkipOnLicenseError(
        [this]() { this->check_crossed_pair_under_a_budget(); });
}
TYPED_TEST_P(IisByDeletionTest, restores_everything_it_saved) {
    this->SkipOnLicenseError(
        [this]() { this->check_restores_everything_it_saved(); });
}
TYPED_TEST_P(IisByDeletionTest, time_limit_reads_back_unchanged) {
    this->SkipOnLicenseError(
        [this]() { this->check_time_limit_reads_back_unchanged(); });
}
TYPED_TEST_P(IisByDeletionTest, indicator_constraints_are_background) {
    this->SkipOnLicenseError(
        [this]() { this->check_indicator_constraints_are_background(); });
}
TYPED_TEST_P(IisByDeletionTest, forwarded_time_limit_is_restored) {
    this->SkipOnLicenseError(
        [this]() { this->check_forwarded_time_limit_is_restored(); });
}

REGISTER_TYPED_TEST_SUITE_P(
    IisByDeletionTest, bounds_against_a_row, one_side_of_an_equality_row,
    integer_equal_to_one_half, integers_summing_to_one_half,
    integer_in_a_fractional_interval, ranged_row_lower_side,
    ranged_row_upper_side, ranged_row_holding_no_integer, redundant_rows,
    two_disjoint_conflicts, chain_where_every_row_is_needed,
    crossed_variable_bounds, crossed_row_sides, crossed_term_less_row,
    feasible_model, removed_variable_is_skipped,
    answer_survives_a_later_removal, published_vectors_under_transforms,
    model_modified_after_an_infeasible_solve,
    model_data_and_result_survive_the_call,
    column_less_model_names_the_violated_row, budget_sweep_keeps_a_valid_answer,
    stop_requested_beforehand_solves_nothing, zero_second_budget_solves_nothing,
    status_is_unknown_after_a_run_that_solved,
    status_survives_a_run_without_a_solve,
    column_less_precheck_ignores_the_limits,
    crossed_pair_is_the_proven_set_under_a_budget, restores_everything_it_saved,
    time_limit_reads_back_unchanged, forwarded_time_limit_is_restored,
    indicator_constraints_are_background);

}  // namespace mippp
