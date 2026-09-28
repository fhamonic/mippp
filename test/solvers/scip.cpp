#include "mippp/solvers/scip/all.hpp"

#include <limits>

using namespace mippp;

#include "test_suites/all.hpp"

MIPPP_API_VERSION_TEST(SCIP_api, scip_api, "SCIP")

struct scip_milp_test : public model_test<scip_api, scip_milp> {
    static void SetUpTestSuite() { construct_api("SCIP"); }
};

// presolve fixes both columns here, which SCIP's global bounds follow
TEST_F(scip_milp_test, variable_bounds_after_a_solve_are_the_model_ones) {
    using namespace operators;
    auto model = new_model();
    auto x = model.add_variable({.lower_bound = 0., .upper_bound = 10.});
    auto y = model.add_variable({.lower_bound = 0., .upper_bound = 10.});
    model.add_constraint(x <= 4.);
    model.add_constraint(x + y <= 7.);
    model.set_objective(x + y);
    model.set_maximization();
    model.solve();
    ASSERT_TRUE(is<status::optimal>(model.get_status()));
    EXPECT_EQ(model.get_variable_lower_bound(x), 0.);
    EXPECT_EQ(model.get_variable_upper_bound(x), 10.);
    EXPECT_EQ(model.get_variable_lower_bound(y), 0.);
    EXPECT_EQ(model.get_variable_upper_bound(y), 10.);
}

TEST_F(scip_milp_test, variable_bounds_read_as_scip_solves_them) {
    auto model = new_model();
    auto x =
        model.add_integer_variable({.lower_bound = 0.25, .upper_bound = 2.75});
    EXPECT_EQ(model.get_variable_lower_bound(x), 1.);
    EXPECT_EQ(model.get_variable_upper_bound(x), 2.);
    model.set_variable_upper_bound(x, 3.5);
    EXPECT_EQ(model.get_variable_upper_bound(x), 3.);
    auto y = model.add_variable(
        {.lower_bound = -std::numeric_limits<double>::infinity(),
         .upper_bound = 1e30});
    EXPECT_EQ(model.get_variable_lower_bound(y), -model.infinity());
    EXPECT_EQ(model.get_variable_upper_bound(y), model.infinity());
}

INSTANTIATE_TEST(SCIP, LpModelTest, scip_milp_test);
INSTANTIATE_TEST(SCIP, MilpModelTest, scip_milp_test);
INSTANTIATE_TEST(SCIP, EnumerableEntitiesTest, scip_milp_test);
INSTANTIATE_TEST(SCIP, ReadableObjectiveTest, scip_milp_test);
INSTANTIATE_TEST(SCIP, ModifiableObjectiveTest, scip_milp_test);
INSTANTIATE_TEST(SCIP, ReadableVariablesBoundsTest, scip_milp_test);
INSTANTIATE_TEST(SCIP, ModifiableVariablesBoundsTest, scip_milp_test);
INSTANTIATE_TEST(SCIP, NamedVariablesTest, scip_milp_test);
INSTANTIATE_TEST(SCIP, ReadableConstraintBoundsTest, scip_milp_test);
// INSTANTIATE_TEST(SCIP, CandidateSolutionCallbackTest, scip_milp_test);
INSTANTIATE_TEST(SCIP, SudokuTest, scip_milp_test);
INSTANTIATE_TEST(SCIP, TimeLimitTest, scip_milp_test);
INSTANTIATE_TEST(SCIP, TimeLimitIncumbentTest, scip_milp_test);
INSTANTIATE_TEST(SCIP, OptimalityToleranceTest, scip_milp_test);
INSTANTIATE_TEST(SCIP, MipGapTest, scip_milp_test);
INSTANTIATE_TEST(SCIP, VerbosityTest, scip_milp_test);
