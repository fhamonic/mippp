#include <gtest/gtest.h>

#include <concepts>
#include <ranges>
#include <utility>
#include <variant>

#include "mippp/solvers/gurobi/all.hpp"

using namespace mippp;

#include "test_suites/all.hpp"

MIPPP_API_VERSION_TEST(Gurobi_api, gurobi_api, "GUROBI")

struct gurobi_lp_test : public model_test<gurobi_api, gurobi_lp> {
    static void SetUpTestSuite() { construct_api("GUROBI"); }

    // Three depots of 8 t for three stores ordering 9 t each, the six rows
    // being the only IIS. The route capacities keep presolve from finding
    // the conflict, so a solve needs simplex iterations to prove it.
    static void add_depot_store_conflict(gurobi_lp & model) {
        using namespace operators;
        const auto sites = std::views::iota(0, 3);
        auto ship =
            model.add_variables(9, [](int i, int j) { return 3 * i + j; });
        for(int i : sites)
            for(int j : sites)
                model.set_variable_upper_bound(ship(i, j), 3 + (i + 2 * j) % 4);
        for(int i : sites)
            model.add_constraint(
                xsum(sites, [&, i](int j) { return ship(i, j); }) <= 8.);
        for(int j : sites)
            model.add_constraint(
                xsum(sites, [&, j](int i) { return ship(i, j); }) >= 9.);
    }
};
INSTANTIATE_TEST(Gurobi_lp, LpModelTest, gurobi_lp_test);
INSTANTIATE_TEST(Gurobi_lp, EnumerableEntitiesTest, gurobi_lp_test);
INSTANTIATE_TEST(Gurobi_lp, ReadableObjectiveTest, gurobi_lp_test);
INSTANTIATE_TEST(Gurobi_lp, ModifiableObjectiveTest, gurobi_lp_test);
INSTANTIATE_TEST(Gurobi_lp, ReadableVariablesBoundsTest, gurobi_lp_test);
INSTANTIATE_TEST(Gurobi_lp, ModifiableVariablesBoundsTest, gurobi_lp_test);
INSTANTIATE_TEST(Gurobi_lp, NamedVariablesTest, gurobi_lp_test);
INSTANTIATE_TEST(Gurobi_lp, NamedConstraintsTest, gurobi_lp_test);
INSTANTIATE_TEST(Gurobi_lp, AddColumnTest, gurobi_lp_test);
INSTANTIATE_TEST(Gurobi_lp, RemoveVariableTest, gurobi_lp_test);
INSTANTIATE_TEST(Gurobi_lp, ReadableConstraintsTest, gurobi_lp_test);
INSTANTIATE_TEST(Gurobi_lp, ReadableConstraintBoundsTest, gurobi_lp_test);
INSTANTIATE_TEST(Gurobi_lp, IisTest, gurobi_lp_test);
INSTANTIATE_TEST(Gurobi_lp, IisByDeletionTest, gurobi_lp_test);
INSTANTIATE_TEST(Gurobi_lp, LpStatusTest, gurobi_lp_test);
INSTANTIATE_TEST(Gurobi_lp, DualSolutionTest, gurobi_lp_test);
INSTANTIATE_TEST(Gurobi_lp, ReducedCostsTest, gurobi_lp_test);
INSTANTIATE_TEST(Gurobi_lp, CuttingStockTest, gurobi_lp_test);
INSTANTIATE_TEST(Gurobi_lp, ColumnManagerTest, gurobi_lp_test);
INSTANTIATE_TEST(Gurobi_lp, LpFuzzyTest, gurobi_lp_test);
INSTANTIATE_TEST(Gurobi_lp, TimeLimitTest, gurobi_lp_test);
INSTANTIATE_TEST(Gurobi_lp, IterationLimitTest, gurobi_lp_test);
INSTANTIATE_TEST(Gurobi_lp, LpBasisTest, gurobi_lp_test);
INSTANTIATE_TEST(Gurobi_lp, LpBasisWarmStartTest, gurobi_lp_test);
INSTANTIATE_TEST(Gurobi_lp, VerbosityTest, gurobi_lp_test);
INSTANTIATE_TEST(Gurobi_lp, ThreadLimitTest, gurobi_lp_test);
INSTANTIATE_TEST(Gurobi_lp, LpAlgorithmTest, gurobi_lp_test);

struct gurobi_milp_test : public model_test<gurobi_api, gurobi_milp> {
    static void SetUpTestSuite() { construct_api("GUROBI"); }

    // z is held at 1 by a row, and the indicator then asks x >= 5 of x in
    // [0, 1]: without the indicator the model is feasible
    struct indicator_conflict {
        model_variable_t<gurobi_milp> z, x;
        model_constraint_t<gurobi_milp> row;
    };
    static indicator_conflict add_indicator_conflict(gurobi_milp & model) {
        using namespace operators;
        auto z = model.add_binary_variable();
        auto x = model.add_variable({.lower_bound = 0., .upper_bound = 1.});
        auto row = model.add_constraint(z >= 1.);
        model.add_indicator_constraint(z, true, x >= 5.);
        return {z, x, row};
    }
    // attribute writes are queued in Gurobi until an update, which the
    // model's own getters run but a native read does not
    static int read_int_attribute(const gurobi_milp & model, const char * name,
                                  int index) {
        const auto [env, native] = model.native_model();
        int value = 0;
        model.native_api()._check(env, model.native_api().updatemodel(native));
        model.native_api()._check(env, model.native_api().getintattrelement(
                                           native, name, index, &value));
        return value;
    }
    static void write_int_attribute(const gurobi_milp & model,
                                    const char * name, int index, int value) {
        const auto [env, native] = model.native_model();
        model.native_api()._check(env, model.native_api().updatemodel(native));
        model.native_api()._check(env, model.native_api().setintattrelement(
                                           native, name, index, value));
    }
};
INSTANTIATE_TEST(Gurobi_milp, LpModelTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, MilpModelTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, EnumerableEntitiesTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, ReadableObjectiveTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, ModifiableObjectiveTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, ReadableVariablesBoundsTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, ModifiableVariablesBoundsTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, NamedVariablesTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, NamedConstraintsTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, AddColumnTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, RemoveVariableTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, ReadableConstraintsTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, ReadableConstraintBoundsTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, IisTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, IisByDeletionTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, SudokuTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, CandidateSolutionCallbackTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, LazyConstraintsTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, SolutionInjectionTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, NodeRelaxationCallbackTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, UserCutsTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, TravellingSalesmanTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, TimeLimitTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, TimeLimitIncumbentTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, MipStartTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, OptimalityToleranceTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, MipGapTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, DualBoundTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, NodeCountTest, gurobi_milp_test);
// INSTANTIATE_TEST(Gurobi_milp, IntegralityToleranceTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, VerbosityTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, ThreadLimitTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, ProgressCallbackTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, ProgressCallbackNodeCountTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, ProgressCallbackSharingTest, gurobi_milp_test);

static_assert(has_iis<gurobi_lp>);
static_assert(has_iis<gurobi_milp>);
static_assert(has_lp_basis_warm_start<gurobi_lp>);
static_assert(!has_lp_basis<gurobi_milp>);
static_assert(
    std::same_as<
        native_iis_outcome_t<gurobi_lp>,
        std::variant<iis_outcome::incomplete, iis_outcome::irreducible,
                     iis_outcome::feasible, iis_outcome::stopped,
                     iis_outcome::interrupted, iis_outcome::limit_reached,
                     iis_outcome::time_limit, iis_outcome::iteration_limit,
                     iis_outcome::memory_limit>>);
static_assert(std::same_as<native_iis_outcome_t<gurobi_milp>,
                           native_iis_outcome_t<gurobi_lp>>);
// A Gurobi range is a slack column, not a second side on the row, so the
// models have no row-bound setters and the deletion filter writes their rows
// through the sense and the rhs.
static_assert(!has_modifiable_constraint_bounds<gurobi_lp>);
static_assert(iis_by_deletion_model<gurobi_lp>);
static_assert(iis_by_deletion_model<gurobi_milp>);
static_assert(detail::iis_rows_as_sense_and_rhs<gurobi_lp>);
static_assert(detail::iis_rows_as_sense_and_rhs<gurobi_milp>);

// The cause a stop before any subsystem gets from Status, pinned without a
// licence since no test run reaches 11, 15 or 17. A stopped MIP reads 1,
// which leaves the clock to decide.
struct gurobi_iis_mapping : gurobi_lp {
    using gurobi_base::_iis_stop_outcome;
};
TEST(gurobi_iis_stop_outcome, names_the_cause_status_reports) {
    using namespace gurobi::impl::v1;
    const auto map = &gurobi_iis_mapping::_iis_stop_outcome;
    EXPECT_TRUE(
        outcome_is<iis_outcome::time_limit>(map(GRB_TIME_LIMIT, false), false));
    EXPECT_TRUE(outcome_is<iis_outcome::iteration_limit>(
        map(GRB_ITERATION_LIMIT, false), false));
    EXPECT_TRUE(outcome_is<iis_outcome::memory_limit>(map(GRB_MEM_LIMIT, false),
                                                      false));
    EXPECT_TRUE(outcome_is<iis_outcome::interrupted>(
        map(GRB_INTERRUPTED, false), false));
    EXPECT_TRUE(outcome_is<iis_outcome::limit_reached>(
        map(GRB_USER_OBJ_LIMIT, false), false));
    EXPECT_TRUE(outcome_is<iis_outcome::limit_reached>(
        map(GRB_WORK_LIMIT, false), false));
    EXPECT_TRUE(outcome_is<iis_outcome::time_limit>(map(1, true), false));
    EXPECT_TRUE(outcome_is<iis_outcome::stopped>(map(1, false), false));
}

// Gurobi's IterationLimit also stops the routine's solves: a stop before any
// subsystem leaves Status at 7, read as an iteration limit with no conflict,
// and the limit stays where the user set it.
TEST_F(gurobi_lp_test, iteration_limit_stops_compute_iis) {
    auto model = new_model();
    add_depot_store_conflict(model);
    model.set_iteration_limit(0);
    model.solve();
    ASSERT_TRUE(is_a<status::iteration_limit>(model.get_status()));
    const auto iis = model.compute_iis();
    EXPECT_TRUE(
        outcome_is<iis_outcome::iteration_limit>(iis.get_outcome(), false));
    EXPECT_EQ(iis.num_variable_members(), 0u);
    EXPECT_EQ(iis.num_constraint_members(), 0u);
    EXPECT_EQ(model.get_iteration_limit(), 0u);
    EXPECT_TRUE(is<status::unknown>(model.get_status()));
    // lifted, the same model answers in full
    model.set_iteration_limit(1000);
    const auto answer = model.compute_iis();
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(answer.get_outcome()));
    EXPECT_EQ(answer.num_constraint_members(), 6u);
    EXPECT_EQ(answer.num_variable_members(), 0u);
}

// A tag Gurobi has no value for, or that it would run as another, must not
// compile: Method has no "either simplex", and the selector is an LP member.
static_assert(has_lp_algorithm<gurobi_lp>);
static_assert(has_lp_algorithm<gurobi_lp, lp_algorithm::primal_simplex>);
static_assert(has_lp_algorithm<gurobi_lp, lp_algorithm::dual_simplex>);
static_assert(has_lp_algorithm<gurobi_lp, lp_algorithm::interior_point>);
static_assert(has_lp_algorithm<gurobi_lp,
                               lp_algorithm::interior_point_without_crossover>);
static_assert(!has_lp_algorithm<gurobi_lp, lp_algorithm::simplex>);
static_assert(!has_lp_algorithm<gurobi_lp, lp_algorithm::other>);
static_assert(variant_with_alternative<model_lp_algorithm_t<gurobi_lp>,
                                       lp_algorithm::other>);
static_assert(!has_lp_algorithm<gurobi_milp>);

namespace {
struct gurobi_iterations {
    double simplex;
    int barrier;
};
gurobi_iterations iterations_of(gurobi_lp & model) {
    const auto [env, native] = model.native_model();
    const gurobi_api & grb = model.native_api();
    gurobi_iterations count{};
    grb._check(env, grb.getdblattr(native, "IterCount", &count.simplex));
    grb._check(env, grb.getintattr(native, "BarIterCount", &count.barrier));
    return count;
}
std::pair<int, int> method_and_crossover(gurobi_lp & model) {
    const auto [env, native] = model.native_model();
    const gurobi_api & grb = model.native_api();
    std::pair<int, int> knobs;
    grb._check(env, grb.getintparam(env, "Method", &knobs.first));
    grb._check(env, grb.getintparam(env, "Crossover", &knobs.second));
    return knobs;
}
}  // namespace

// The shared suite cannot see which algorithm ran: Gurobi's counters can.
// A crossover shows as simplex iterations after the barrier's.
TEST_F(gurobi_lp_test, each_lp_algorithm_runs_the_one_it_names) {
    SkipOnLicenseError([this]() {
        const auto solved_under = [this](auto algorithm) {
            auto model = new_model();
            lp_basis_cases::build_dense(model);
            model.set_lp_algorithm(algorithm);
            model.solve();
            EXPECT_TRUE(is_a<status::optimal>(model.get_status()));
            return std::pair(method_and_crossover(model), iterations_of(model));
        };
        {
            const auto [knobs, count] =
                solved_under(lp_algorithm::primal_simplex{});
            EXPECT_EQ(knobs, std::pair(0, -1));
            EXPECT_GT(count.simplex, 0.);
            EXPECT_EQ(count.barrier, 0);
        }
        {
            const auto [knobs, count] =
                solved_under(lp_algorithm::dual_simplex{});
            EXPECT_EQ(knobs, std::pair(1, -1));
            EXPECT_GT(count.simplex, 0.);
            EXPECT_EQ(count.barrier, 0);
        }
        {
            const auto [knobs, count] =
                solved_under(lp_algorithm::interior_point{});
            EXPECT_EQ(knobs, std::pair(2, -1));
            EXPECT_GT(count.barrier, 0);
        }
        {
            const auto [knobs, count] =
                solved_under(lp_algorithm::interior_point_without_crossover{});
            EXPECT_EQ(knobs, std::pair(2, 0));
            EXPECT_GT(count.barrier, 0);
            EXPECT_EQ(count.simplex, 0.);
        }
        {
            const auto [knobs, count] = solved_under(lp_algorithm::automatic{});
            EXPECT_EQ(knobs, std::pair(-1, -1));
        }
    });
}

// A setting no tag writes reads as other, and stays: the concurrent method,
// and a crossover strategy that interior_point would reset.
TEST_F(gurobi_lp_test, a_native_lp_method_reads_as_other) {
    SkipOnLicenseError([this]() {
        auto model = new_model();
        const auto [env, native] = model.native_model();
        const gurobi_api & grb = model.native_api();
        grb._check(env, grb.setintparam(env, "Method", 3));
        EXPECT_TRUE(is<lp_algorithm::other>(model.get_lp_algorithm()));
        grb._check(env, grb.setintparam(env, "Method", 2));
        EXPECT_TRUE(is<lp_algorithm::interior_point>(model.get_lp_algorithm()));
        grb._check(env, grb.setintparam(env, "Crossover", 3));
        EXPECT_TRUE(is<lp_algorithm::other>(model.get_lp_algorithm()));
        EXPECT_EQ(method_and_crossover(model), std::pair(2, 3));
    });
}

// Under a barrier without crossover the refinement would run that barrier
// again and stay undecided: it runs the dual simplex, and the model's
// algorithm reads as before.
TEST_F(gurobi_lp_test,
       refine_lp_status_decides_under_a_crossover_less_barrier) {
    SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = new_model();
        auto x = model.add_variable();
        model.add_constraint(x >= 1.);
        model.set_maximization();
        model.set_objective(1. * x);
        model.set_lp_algorithm(
            lp_algorithm::interior_point_without_crossover{});
        model.solve();
        ASSERT_TRUE(is<status::infeasible_or_unbounded>(model.get_status()));
        model.refine_lp_status();
        EXPECT_TRUE(is<status::unbounded>(model.get_status()));
        EXPECT_TRUE(is<lp_algorithm::interior_point_without_crossover>(
            model.get_lp_algorithm()));
    });
}

// The re-solve under DualReductions 0 can throw: a callback that returns
// nonzero aborts GRBoptimize with an error. The parameters must read the
// user's values again afterwards, not the ones the call set.
TEST_F(gurobi_lp_test, refine_lp_status_restores_dual_reductions_on_a_throw) {
    using namespace operators;
    using gurobi::impl::v1::GRBmodel;
    auto model = new_model();
    // presolve's dual fixing proves x unbounded without telling it from an
    // infeasible rest, which is what INF_OR_UNBD reports
    auto x = model.add_variable();
    model.add_constraint(x >= 1.);
    model.set_maximization();
    model.set_objective(1. * x);
    model.solve();
    ASSERT_TRUE(is<status::infeasible_or_unbounded>(model.get_status()));
    const auto [env, native] = model.native_model();
    const gurobi_api & grb = model.native_api();
    grb._check(env,
               grb.setcallbackfunc(
                   native, +[](GRBmodel *, void *, int, void *) { return 1; },
                   nullptr));
    grb._check(env, grb.setintparam(env, "Method", 0));
    EXPECT_ANY_THROW(model.refine_lp_status());
    int dual_reductions = 0, method = -1;
    grb._check(env, grb.getintparam(env, "DualReductions", &dual_reductions));
    grb._check(env, grb.getintparam(env, "Method", &method));
    EXPECT_EQ(dual_reductions, 1);
    EXPECT_EQ(method, 0);
}

// A solve that throws reports unknown, not the status of the solve before it:
// a callback that returns nonzero aborts GRBoptimize with an error. A trivial
// model is solved without a callback call, so the re-solve runs on the model
// of the refine_lp_status test, under DualReductions 0.
TEST_F(gurobi_lp_test, a_throwing_solve_reports_unknown) {
    using namespace operators;
    using gurobi::impl::v1::GRBmodel;
    auto model = new_model();
    auto x = model.add_variable();
    model.add_constraint(x >= 1.);
    model.set_maximization();
    model.set_objective(1. * x);
    model.solve();
    ASSERT_TRUE(is<status::infeasible_or_unbounded>(model.get_status()));
    const auto [env, native] = model.native_model();
    const gurobi_api & grb = model.native_api();
    grb._check(env, grb.setintparam(env, "DualReductions", 0));
    grb._check(env,
               grb.setcallbackfunc(
                   native, +[](GRBmodel *, void *, int, void *) { return 1; },
                   nullptr));
    EXPECT_ANY_THROW(model.solve());
    EXPECT_TRUE(is<status::unknown>(model.get_status()));
}

TEST_F(gurobi_lp_test, get_basis_throws_after_presolve_proves_infeasibility) {
    auto model = new_model();
    lp_basis_cases::build_infeasible(model);
    model.solve();
    ASSERT_TRUE(is_a<status::infeasible_or_unbounded>(model.get_status()));
    ASSERT_THROW(model.get_basis(), solver_error);
}
// A row of Gurobi is never at no bound: is_a<superbasic> compiles on a
// column status only.
static_assert(variant_containing_a<model_variable_basis_status_t<gurobi_lp>,
                                   basis_status::superbasic>);
static_assert(!variant_containing_a<model_constraint_basis_status_t<gurobi_lp>,
                                    basis_status::superbasic>);

// CBasis names no side on a row: an '=' row at its right-hand side reads
// nonbasic_fixed, from the sense.
TEST_F(gurobi_lp_test, get_basis_reads_an_equality_row_fixed) {
    auto model = new_model();
    const auto v = lp_basis_cases::build_vertex(model);
    model.solve();
    ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
    const auto basis = model.get_basis();
    ASSERT_TRUE(is<basis_status::nonbasic_fixed>(basis.get_status(v.eq)));
}

// Without rows, the row arrays reach Gurobi with length 0 and a null buffer.
TEST_F(gurobi_lp_test, basis_of_a_model_without_rows) {
    using namespace operators;
    auto model = new_model();
    const auto x =
        model.add_variable(lp_basis_cases::bounds<gurobi_lp>(0.0, 1.0));
    const auto y =
        model.add_variable(lp_basis_cases::bounds<gurobi_lp>(-1.0, 3.0));
    model.set_maximization();
    model.set_objective(x - y);
    model.solve();
    ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
    const auto basis = model.get_basis();
    ASSERT_FALSE(is<basis_status::basic>(basis.get_status(x)));
    ASSERT_FALSE(is<basis_status::basic>(basis.get_status(y)));
    model.set_basis(basis);
    model.solve();
    ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
    ASSERT_NEAR(model.get_solution_value(), 2.0, TEST_EPSILON);
}
// The code set_basis writes for each tag the statuses list, pinned without a
// library: a tag that loses its line in the table writes as its nearest
// base, which still compiles.
struct gurobi_basis_codes : gurobi_lp {
    using column_status = model_variable_basis_status_t<gurobi_lp>;
    using row_status = model_constraint_basis_status_t<gurobi_lp>;
    template <typename Tag>
    static constexpr auto column() {
        return _basis_variable_code(column_status(std::in_place_type<Tag>));
    }
    template <typename Tag>
    static constexpr auto row() {
        return _basis_constraint_code(row_status(std::in_place_type<Tag>));
    }
};
TEST(gurobi_lp_basis_codes, one_per_listed_tag) {
    using namespace gurobi::impl::v1;
    using namespace basis_status;
    using codes = gurobi_basis_codes;
    // clang-format off
    static_assert(std::variant_size_v<codes::column_status> == 5);
    static_assert(std::variant_size_v<codes::row_status>    == 5);
    static_assert(codes::column<basic>()                   == GRB_BASIC);
    static_assert(codes::column<nonbasic_at_lower_bound>() == GRB_NONBASIC_LOWER);
    static_assert(codes::column<nonbasic_at_upper_bound>() == GRB_NONBASIC_UPPER);
    static_assert(codes::column<superbasic>()              == GRB_SUPERBASIC);
    static_assert(codes::column<nonbasic>()                == GRB_NONBASIC_LOWER);
    static_assert(codes::row<basic>()                   == GRB_BASIC);
    static_assert(codes::row<nonbasic_at_lower_bound>() == GRB_NONBASIC_LOWER);
    static_assert(codes::row<nonbasic_at_upper_bound>() == GRB_NONBASIC_LOWER);
    static_assert(codes::row<nonbasic_fixed>()          == GRB_NONBASIC_LOWER);
    static_assert(codes::row<nonbasic>()                == GRB_NONBASIC_LOWER);
    // clang-format on
}

// The forcing attributes are the user's: values set to 0 beforehand, which
// the call overrides with 1, must read back as 0 afterwards, on every kind
// of special constraint the guard covers.
TEST_F(gurobi_milp_test, compute_iis_restores_the_forcing_attributes) {
    using namespace operators;
    auto model = new_model();
    const auto conflict = add_indicator_conflict(model);
    auto y = model.add_variable({.lower_bound = 0., .upper_bound = 1.});
    const auto [env, native] = model.native_model();
    const gurobi_api & grb = model.native_api();
    {
        // an SOS1 on {x, y} and y * y <= 100, both feasible with the answer
        int types[1] = {1}, beg[1] = {0};
        int ind[2] = {model.native_id(conflict.x), model.native_id(y)};
        double weight[2] = {1., 2.};
        grb._check(env, grb.addsos(native, 1, 2, types, beg, ind, weight));
        int qrow[1] = {model.native_id(y)}, qcol[1] = {model.native_id(y)};
        double qval[1] = {1.};
        grb._check(env, grb.addqconstr(native, 0, nullptr, nullptr, 1, qrow,
                                       qcol, qval, '<', 100., nullptr));
    }
    write_int_attribute(model, "IISSOSForce", 0, 0);
    write_int_attribute(model, "IISQConstrForce", 0, 0);
    write_int_attribute(model, "IISGenConstrForce", 0, 0);
    const auto iis = model.compute_iis();
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
    EXPECT_TRUE(is<iis_status::member_upper>(iis.get_status(conflict.x)));
    EXPECT_TRUE(is<iis_status::member_lower>(iis.get_status(conflict.row)));
    EXPECT_EQ(read_int_attribute(model, "IISSOSForce", 0), 0);
    EXPECT_EQ(read_int_attribute(model, "IISQConstrForce", 0), 0);
    EXPECT_EQ(read_int_attribute(model, "IISGenConstrForce", 0), 0);
}

// A forcing attribute left at its default reads it again after the call.
TEST_F(gurobi_milp_test, compute_iis_keeps_a_default_forcing_attribute) {
    auto model = new_model();
    add_indicator_conflict(model);
    const auto iis = model.compute_iis();
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
    EXPECT_EQ(read_int_attribute(model, "IISGenConstrForce", 0), -1);
}

// The removal shifts the native ids the indicator and the answer are
// written with: both must still name the handles.
TEST_F(gurobi_milp_test, removed_variable_before_an_indicator_conflict) {
    auto model = new_model();
    auto gone = model.add_variable({.lower_bound = 0., .upper_bound = 1.});
    const auto conflict = add_indicator_conflict(model);
    model.remove_variable(gone);
    const auto iis = model.compute_iis();
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
    EXPECT_TRUE(is<iis_status::absent>(iis.get_status(gone)));
    EXPECT_TRUE(is<iis_status::absent>(iis.get_status(conflict.z)));
    EXPECT_TRUE(is<iis_status::member_upper>(iis.get_status(conflict.x)));
    EXPECT_TRUE(is<iis_status::member_lower>(iis.get_status(conflict.row)));
    EXPECT_EQ(iis.num_variable_members(), 1u);
    EXPECT_EQ(iis.num_constraint_members(), 1u);
}

// Gurobi documents that a binary's bounds are implicit in its type and never
// members: the row alone is the answer, an IIS relative to the integrality
// kept as background, whereas an integer column in [0, 1] has a real bound.
TEST_F(gurobi_milp_test, binary_bounds_are_implicit_background) {
    using namespace operators;
    {
        auto model = new_model();
        auto z = model.add_binary_variable();
        auto row = model.add_constraint(z >= 2.);
        const auto iis = model.compute_iis();
        EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
        EXPECT_TRUE(is<iis_status::absent>(iis.get_status(z)));
        EXPECT_TRUE(is<iis_status::member_lower>(iis.get_status(row)));
        EXPECT_EQ(iis.num_variable_members(), 0u);
        EXPECT_EQ(iis.num_constraint_members(), 1u);
    }
    {
        auto model = new_model();
        auto z =
            model.add_integer_variable({.lower_bound = 0., .upper_bound = 1.});
        auto row = model.add_constraint(z >= 2.);
        const auto iis = model.compute_iis();
        EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
        EXPECT_TRUE(is<iis_status::member_upper>(iis.get_status(z)));
        EXPECT_TRUE(is<iis_status::member_lower>(iis.get_status(row)));
        EXPECT_EQ(iis.num_variable_members(), 1u);
        EXPECT_EQ(iis.num_constraint_members(), 1u);
    }
}
