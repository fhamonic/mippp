#include <gtest/gtest.h>

#include <concepts>
#include <ranges>
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
INSTANTIATE_TEST(Gurobi_lp, VerbosityTest, gurobi_lp_test);

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
INSTANTIATE_TEST(Gurobi_milp, TravellingSalesmanTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, TimeLimitTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, TimeLimitIncumbentTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, MipStartTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, OptimalityToleranceTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, MipGapTest, gurobi_milp_test);
// INSTANTIATE_TEST(Gurobi_milp, IntegralityToleranceTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, VerbosityTest, gurobi_milp_test);

static_assert(has_iis<gurobi_lp>);
static_assert(has_iis<gurobi_milp>);
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

// The re-solve under DualReductions 0 can throw: a callback that returns
// nonzero aborts GRBoptimize with an error. The parameter must read the
// user's value again afterwards, not the 0 the call set.
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
    EXPECT_ANY_THROW(model.refine_lp_status());
    int dual_reductions = 0;
    grb._check(env, grb.getintparam(env, "DualReductions", &dual_reductions));
    EXPECT_EQ(dual_reductions, 1);
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
