#include <gtest/gtest.h>

#include <optional>
#include <ranges>

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
// A Gurobi range is a slack column, not a second side on the row, so the
// models have no row-bound setters and the deletion filter writes their rows
// through the sense and the rhs.
static_assert(!has_modifiable_constraint_bounds<gurobi_lp>);
static_assert(iis_by_deletion_model<gurobi_lp>);
static_assert(iis_by_deletion_model<gurobi_milp>);
static_assert(detail::iis_rows_as_sense_and_rhs<gurobi_lp>);
static_assert(detail::iis_rows_as_sense_and_rhs<gurobi_milp>);

// Gurobi's IterationLimit also stops the routine's solves, and a stop that
// is not the time limit's carries no reason: the answer is only that there
// is none, and the limit stays where the user set it.
TEST_F(gurobi_lp_test, iteration_limit_stops_compute_iis_without_a_reason) {
    auto model = new_model();
    add_depot_store_conflict(model);
    model.set_iteration_limit(0);
    model.solve();
    ASSERT_TRUE(is_a<status::iteration_limit>(model.get_status()));
    const auto iis = model.compute_iis();
    EXPECT_EQ(iis.get_outcome(), iis_outcome::undetermined);
    EXPECT_EQ(iis.get_reason(), std::nullopt);
    EXPECT_EQ(iis.num_variable_members(), 0u);
    EXPECT_EQ(iis.num_constraint_members(), 0u);
    EXPECT_EQ(model.get_iteration_limit(), 0u);
    EXPECT_TRUE(is<status::unknown>(model.get_status()));
    // lifted, the same model answers in full
    model.set_iteration_limit(1000);
    const auto answer = model.compute_iis();
    EXPECT_EQ(answer.get_outcome(), iis_outcome::irreducible);
    EXPECT_EQ(answer.num_constraint_members(), 6u);
    EXPECT_EQ(answer.num_variable_members(), 0u);
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
    EXPECT_EQ(iis.get_outcome(), iis_outcome::irreducible);
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
    EXPECT_EQ(iis.get_outcome(), iis_outcome::irreducible);
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
    EXPECT_EQ(iis.get_outcome(), iis_outcome::irreducible);
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
        EXPECT_EQ(iis.get_outcome(), iis_outcome::irreducible);
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
        EXPECT_EQ(iis.get_outcome(), iis_outcome::irreducible);
        EXPECT_TRUE(is<iis_status::member_upper>(iis.get_status(z)));
        EXPECT_TRUE(is<iis_status::member_lower>(iis.get_status(row)));
        EXPECT_EQ(iis.num_variable_members(), 1u);
        EXPECT_EQ(iis.num_constraint_members(), 1u);
    }
}
