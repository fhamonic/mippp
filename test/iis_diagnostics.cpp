#include <gtest/gtest.h>
#include <algorithm>
#include <exception>
#include <limits>
#include <string>
#include "mippp/solvers/clp/all.hpp"
#include "mippp/solvers/mosek/all.hpp"
#include "mippp/utility/linear_iis.hpp"

using namespace mippp::iis;
namespace {
std::size_t total(const check_statistics & s) {
    return s.feasible + s.infeasible + s.unknown;
}
linear_system<> conflict() {
    linear_system<> s;
    s.variables = {{std::nullopt, 1., false}, {-10., 10., false}};
    s.rows = {{{{0, 1.}}, 2., std::nullopt}, {{{1, 1.}}, -20., 20.}};
    return s;
}
template <typename F>
std::string invalid_message(F && call) {
    try {
        call();
    } catch(const std::invalid_argument & e) {
        return e.what();
    }
    ADD_FAILURE() << "Expected an invalid_argument";
    return {};
}
}  // namespace

TEST(DeletionFilter, StatisticsCountChecksAndNecessity) {
    auto oracle = [](std::span<const std::size_t> ids) {
        const auto has = [&](auto id) {
            return std::find(ids.begin(), ids.end(), id) != ids.end();
        };
        return has(0u) && has(1u) ? feasibility::infeasible
                                  : feasibility::feasible;
    };
    for(auto batch : {1u, 2u, 4u}) {
        const auto a =
            deletion_filter(4, oracle, {.initial_batch_size = batch});
        EXPECT_TRUE(a.irreducible);
        EXPECT_EQ(a.statistics.necessary_members, 2u);
        EXPECT_EQ(a.statistics.unresolved_members, 0u);
        EXPECT_EQ(a.statistics.removed_by_batches +
                      a.statistics.removed_by_singletons,
                  2u);
        EXPECT_EQ(total(a.statistics.outcomes), a.solve_count);
        EXPECT_EQ(a.statistics.initial_checks + a.statistics.batch_checks +
                      a.statistics.singleton_checks,
                  a.solve_count);
    }
    const auto a = deletion_filter(2, [](auto ids) {
        return ids.size() == 2 ? feasibility::infeasible : feasibility::unknown;
    });
    EXPECT_FALSE(a.irreducible);
    EXPECT_EQ(a.statistics.unresolved_members, 2u);
    EXPECT_EQ(a.statistics.necessary_members, 0u);
    EXPECT_EQ(a.statistics.outcomes.unknown, 2u);
}

TEST(DeletionFilter, DiagnosticSettingsAreSnapshots) {
    linear_options options{.limits = {.max_solves = 0}};
    unsigned calls = 0;
    auto factory = [&] {
        ++calls;
        return mippp::clp_lp{};
    };
    const auto a = compute_linear_iis(conflict(), factory, options);
    options.limits.max_solves = 50;
    const auto text = describe_result(a);
    EXPECT_EQ(calls, 0u);
    EXPECT_NE(text.find("No verified conflict is available"),
              std::string::npos);
    EXPECT_NE(text.find("limits.max_solves: current=0; available="),
              std::string::npos);
    EXPECT_EQ(a.statistics.models_built(), 0u);
    const auto timed =
        compute_linear_iis(conflict(), factory,
                           {.limits = {.time_limit = std::chrono::seconds(0)}});
    EXPECT_NE(describe_result(timed).find(
                  "limits.time_limit (seconds): current=0; available="),
              std::string::npos);
    std::stop_source stop;
    stop.request_stop();
    const auto cancelled = compute_linear_iis(
        conflict(), factory, {.limits = {.stop = stop.get_token()}});
    EXPECT_NE(describe_result(cancelled).find(
                  "limits.stop: current=requested; available="),
              std::string::npos);
    EXPECT_EQ(calls, 0u);
}

TEST(DeletionFilter, ValidationNamesValuesAndOriginalIndices) {
    auto factory = [] {
        ADD_FAILURE();
        return mippp::clp_lp{};
    };
    auto s = conflict();
    s.variables[1].upper = std::numeric_limits<double>::infinity();
    auto text = invalid_message([&] { (void)compute_linear_iis(s, factory); });
    EXPECT_NE(text.find("variable 1"), std::string::npos);
    EXPECT_NE(text.find("upper: current=inf; available="), std::string::npos);
    s = conflict();
    s.rows[1].terms[0].first = 99;
    text = invalid_message([&] { (void)compute_linear_iis(s, factory); });
    EXPECT_NE(text.find("Row 1 refers to variable 99"), std::string::npos);
    EXPECT_NE(text.find("0 through 1"), std::string::npos);
    s = conflict();
    s.rows[1].terms[0].second = std::numeric_limits<double>::quiet_NaN();
    text = invalid_message([&] { (void)compute_linear_iis(s, factory); });
    EXPECT_NE(text.find("coefficient: current=nan; available="),
              std::string::npos);
    s = conflict();
    s.variables[0].integer = true;
    text = invalid_message([&] { (void)compute_linear_iis(s, factory); });
    EXPECT_NE(text.find("policy.analyzed_domain: current=original; "
                        "available=original, lp_relaxation"),
              std::string::npos);
    EXPECT_NE(text.find("cannot explain"), std::string::npos);
    constexpr linear_policy native{.native_seed = true};
    text = invalid_message([&] {
        (void)compute_linear_iis<native>(
            conflict(), factory, {.native = {.relative_tolerance = -0.25}});
    });
    EXPECT_NE(text.find("native.relative_tolerance: current=-0.25; available="),
              std::string::npos);
}

TEST(DeletionFilter, PortableSolverIssuesRetainObservedFacts) {
    work_statistics s;
    detail::record_solver_issue(std::variant<mippp::status::time_limit>{}, s);
    EXPECT_EQ(s.last_solver_issue, solve_issue::time_limit);
    detail::record_solver_issue(std::variant<mippp::status::optimal>{}, s);
    EXPECT_EQ(s.last_solver_issue,
              solve_issue::time_limit);  // history is not erased
    detail::record_solver_issue(
        std::variant<mippp::status::optimal_infeasible_unscaled>{}, s);
    EXPECT_EQ(s.last_solver_issue, solve_issue::numerical);
    detail::record_solver_issue(
        std::variant<mippp::status::primal_and_dual_infeasible>{}, s);
    EXPECT_EQ(s.last_solver_issue, solve_issue::ambiguous);
    detail::record_solver_issue(std::variant<mippp::status::limit_reached>{},
                                s);
    EXPECT_EQ(s.last_solver_issue, solve_issue::other_limit);
    EXPECT_FALSE(s.observed_solver_time_limit);
}

TEST(DeletionFilter, NumericalFailureCannotProveFeasibility) {
    // A solution flag does not make a numerically unreliable result a proof.
    const std::variant<mippp::status::numerical_failure> numerical{
        mippp::status::numerical_failure{true}};
    EXPECT_EQ(detail::classify_feasibility(numerical), feasibility::unknown);

    // A reliable incumbent remains useful when a solve stops at a limit.
    const std::variant<mippp::status::time_limit> limited{
        mippp::status::time_limit{true}};
    EXPECT_EQ(detail::classify_feasibility(limited), feasibility::feasible);
}

TEST(DeletionFilter, MosekPrimalStatesAndStoppedSolveEvidence) {
    using namespace mippp::mosek::impl::v1;
    // Expose only the pure shared predicate; never construct a MOSEK model.
    struct probe : mosek_base {
        using mosek_base::_has_primal_solution;
    };
    for(auto state : {MSK_SOL_STA_OPTIMAL, MSK_SOL_STA_INTEGER_OPTIMAL,
                      MSK_SOL_STA_PRIM_FEAS, MSK_SOL_STA_PRIM_AND_DUAL_FEAS})
        EXPECT_TRUE(probe::_has_primal_solution(state));
    for(auto state :
        {MSK_SOL_STA_UNKNOWN, MSK_SOL_STA_DUAL_FEAS,
         MSK_SOL_STA_PRIM_INFEAS_CER, MSK_SOL_STA_DUAL_INFEAS_CER,
         MSK_SOL_STA_PRIM_ILLPOSED_CER, MSK_SOL_STA_DUAL_ILLPOSED_CER})
        EXPECT_FALSE(probe::_has_primal_solution(state));

    using namespace mippp::status;
    for(bool incumbent : {false, true}) {
        const auto expected =
            incumbent ? feasibility::feasible : feasibility::unknown;
        auto check = [&](auto status) {
            EXPECT_EQ(detail::classify_feasibility(
                          std::variant<decltype(status)>{status}),
                      expected);
        };
        check(time_limit{incumbent});
        check(iteration_limit{incumbent});
        check(node_limit{incumbent});
        check(solution_limit{incumbent});
        check(limit_reached{incumbent});
        check(interrupted{incumbent});
        check(failed{incumbent});
        EXPECT_EQ(detail::classify_feasibility(std::variant<numerical_failure>{
                      numerical_failure{incumbent}}),
                  feasibility::unknown);
    }
}

TEST(DeletionFilter, LicenseFailuresRemainCatchableAndExplainExpansion) {
    constexpr linear_policy policy{.elasticity = elasticity_strategy::reuse,
                                   .deletion = deletion_strategy::reuse};
    try {
        (void)compute_linear_iis<policy>(conflict(), []() -> mippp::clp_lp {
            throw mippp::license_error("test license refused");
        });
        FAIL();
    } catch(const mippp::license_error & e) {
        const std::string text = e.what();
        EXPECT_NE(
            text.find(
                "policy.deletion: current=reuse; available=rebuild, reuse"),
            std::string::npos);
        EXPECT_NE(text.find("policy.elasticity: current=reuse; available=off, "
                            "rebuild, reuse"),
                  std::string::npos);
        EXPECT_NE(text.find("2 variables, 2 rows, 6 separate"),
                  std::string::npos);
        EXPECT_NE(text.find("Solver details: test license refused"),
                  std::string::npos);
        try {
            std::rethrow_if_nested(e);
            FAIL();
        } catch(const mippp::license_error & original) {
            EXPECT_STREQ(original.what(), "test license refused");
        }
    }
    try {
        (void)compute_linear_iis(conflict(), []() -> mippp::clp_lp {
            throw std::runtime_error("caller failure");
        });
        FAIL();
    } catch(const std::runtime_error & e) {
        EXPECT_STREQ(e.what(), "caller failure");
    }
}

TEST(ClpRay, DiagnosticAccountingAcrossStrategiesAndLimits) {
    auto run = [&]<linear_policy Policy>() {
        for(std::size_t limit = 0; limit < 12; ++limit) {
            unsigned builds = 0;
            const auto a = compute_linear_iis<Policy>(
                conflict(),
                [&] {
                    ++builds;
                    return mippp::clp_lp{};
                },
                {.limits = {.max_solves = limit, .initial_batch_size = 2}});
            const auto & s = a.statistics;
            EXPECT_EQ(s.input_candidates, 6u);
            EXPECT_EQ(s.models_built(), builds);
            EXPECT_EQ(total(s.feasibility_checks) + total(s.elastic_checks),
                      a.reduction.solve_count);
            EXPECT_EQ(s.solver_runs() + s.rebuild.bound_only_proofs +
                          s.rebuild.solves_skipped + s.deletion.solves_skipped +
                          s.elastic.solves_skipped,
                      a.reduction.solve_count);
            EXPECT_LE(s.native_verification_calls, 1u);
            EXPECT_LE(s.elastic_verification_calls, 1u);
            EXPECT_LE(s.certificate_queries, 1u);
            EXPECT_LE(a.reduction.solve_count, limit);
            EXPECT_FALSE(s.elapsed);
        }
    };
    run.template operator()<linear_policy{}>();
    run.template
    operator()<linear_policy{.deletion = deletion_strategy::reuse}>();
    run.template
    operator()<linear_policy{.elasticity = elasticity_strategy::reuse,
                             .deletion = deletion_strategy::reuse,
                             .native_seed = true,
                             .prune_bounds = true}>();
}

TEST(ClpRay, DiagnosticBoundShortcutsAndCancellation) {
    auto s = conflict();
    s.variables[0].lower = 2.;
    auto a = compute_linear_iis(s, [] { return mippp::clp_lp{}; });
    EXPECT_GT(a.statistics.rebuild.bound_only_proofs, 0u);
    EXPECT_EQ(
        a.statistics.solver_runs() + a.statistics.rebuild.bound_only_proofs,
        a.reduction.solve_count);
    std::stop_source stop;
    a = compute_linear_iis(conflict(),
                           [&] {
                               stop.request_stop();
                               return mippp::clp_lp{};
                           },
                           {.limits = {.stop = stop.get_token()}});
    EXPECT_EQ(a.statistics.models_built(), 1u);
    EXPECT_EQ(a.statistics.solver_runs(), 0u);
    EXPECT_EQ(a.statistics.rebuild.solves_skipped, 1u);
    EXPECT_EQ(a.statistics.feasibility_checks.unknown, 1u);
}

TEST(ClpRay, DiagnosticSeedOutcomesAndOptionalTiming) {
    constexpr linear_policy policy{
        .native_seed = true, .prune_bounds = true, .measure_time = true};
    const auto a =
        compute_linear_iis<policy>(conflict(), [] { return mippp::clp_lp{}; });
    EXPECT_TRUE(a.reduction.irreducible);
    EXPECT_EQ(a.diagnostics.native_seed, seed_outcome::used);
    EXPECT_EQ(a.statistics.native_verification_calls, 1u);
    const auto report = describe_result(a, {.include_members = true});
    EXPECT_NE(report.find("Variable 0 upper bound (original zero-based index)"),
              std::string::npos);
    EXPECT_NE(report.find("Row 0 lower bound (original zero-based index)"),
              std::string::npos);
    ASSERT_TRUE(a.statistics.elapsed);
    EXPECT_GE(a.statistics.elapsed->count(), 0.);
    const auto rejected =
        compute_linear_iis<policy>(conflict(), [] { return mippp::clp_lp{}; },
                                   {.native = {.relative_tolerance = 1.}});
    EXPECT_EQ(rejected.diagnostics.native_seed,
              seed_outcome::verification_feasible);
    EXPECT_TRUE(rejected.diagnostics.full_set_fallback);
    EXPECT_NE(describe_result(rejected).find(
                  "native.relative_tolerance: current=1; available="),
              std::string::npos);
    const auto stopped =
        compute_linear_iis<policy>(conflict(), [] { return mippp::clp_lp{}; },
                                   {.limits = {.max_solves = 1}});
    EXPECT_EQ(stopped.diagnostics.native_seed,
              seed_outcome::verification_skipped);
    EXPECT_EQ(stopped.statistics.native_verification_calls, 0u);
}

TEST(ClpRay, DiagnosticElasticNoProgress) {
    constexpr linear_policy policy{.elasticity = elasticity_strategy::reuse};
    const auto a =
        compute_linear_iis<policy>(conflict(), [] { return mippp::clp_lp{}; },
                                   {.elastic = {.violation_tolerance = 1e6}});
    EXPECT_TRUE(a.reduction.irreducible);
    EXPECT_EQ(a.diagnostics.elastic_seed, seed_outcome::no_progress);
    EXPECT_TRUE(a.diagnostics.full_set_fallback);
    EXPECT_EQ(a.statistics.elastic_checks.feasible, 1u);
    EXPECT_NE(describe_result(a).find(
                  "elastic.violation_tolerance: current=1000000; available="),
              std::string::npos);
}

TEST(DeletionFilter, DiagnosticSummariesRespectProofState) {
    linear_result a;
    EXPECT_NE(describe_result(a).find("No verified conflict"),
              std::string::npos);
    a.reduction.initial_status = feasibility::feasible;
    EXPECT_NE(describe_result(a).find("can be satisfied"), std::string::npos);
    a.analyzed_domain = domain::lp_relaxation;
    EXPECT_NE(
        describe_result(a).find(
            "does not establish feasibility of the original integer problem"),
        std::string::npos);
    a.reduction.initial_status = feasibility::infeasible;
    a.reduction.irreducible = true;
    EXPECT_NE(describe_result(a).find("fixed requirements conflict"),
              std::string::npos);
}
