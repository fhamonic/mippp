#include "mippp/solvers/scip/all.hpp"

#include <limits>
#include <stdexcept>
#include <string_view>
#include <utility>

using namespace mippp;

#include "test_suites/all.hpp"

MIPPP_API_VERSION_TEST(SCIP_api, scip_api, "SCIP")

struct scip_milp_test : public model_test<scip_api, scip_milp> {
    static void SetUpTestSuite() { construct_api("SCIP"); }
};
static_assert(!has_iis<scip_milp>);

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

// SCIP alone would ignore both moves, being below its numerics/epsilon
TEST_F(scip_milp_test, row_side_moved_below_epsilon_reads_back_as_set) {
    using namespace operators;
    auto model = new_model();
    auto x = model.add_variable();
    auto c = model.add_constraint(x >= 2.);
    model.set_constraint_upper_bound(c, 7.);
    model.set_constraint_lower_bound(c, 2. + 1e-10);
    model.set_constraint_upper_bound(c, 7. - 1e-10);
    EXPECT_EQ(model.get_constraint_lower_bound(c), 2. + 1e-10);
    EXPECT_EQ(model.get_constraint_upper_bound(c), 7. - 1e-10);
}

// the moved sides start finite: an assert-enabled SCIP aborts on a side
// moved from one infinity to the other
TEST_F(scip_milp_test, row_side_beyond_infinity_reads_the_infinity) {
    using namespace operators;
    auto model = new_model();
    auto x = model.add_variable();
    auto c = model.add_constraint(x == 3.);
    model.set_constraint_lower_bound(c, 1e30);
    EXPECT_EQ(model.get_constraint_lower_bound(c), model.infinity());
    model.set_constraint_upper_bound(c,
                                     -std::numeric_limits<double>::infinity());
    EXPECT_EQ(model.get_constraint_upper_bound(c), -model.infinity());
}

// SCIP types a [0, 1] integer column BINARY and accepts any bound on it, but
// the next solve rejects a BINARY column whose domain left [0, 1]
TEST_F(scip_milp_test, binary_column_with_a_relaxed_bound_fails_to_solve) {
    using namespace operators;
    auto model = new_model();
    auto x = model.add_binary_variable();
    auto y = model.add_integer_variable({.lower_bound = 0., .upper_bound = 1.});
    model.add_constraint(x + y <= 5.);
    for(auto v : {x, y}) {
        model.set_variable_lower_bound(v, -model.infinity());
        EXPECT_EQ(model.get_variable_lower_bound(v), -model.infinity());
        EXPECT_THROW(model.solve(), std::runtime_error);
        model.set_variable_lower_bound(v, 0.);
        model.set_variable_upper_bound(v, model.infinity());
        EXPECT_EQ(model.get_variable_upper_bound(v), model.infinity());
        EXPECT_THROW(model.solve(), std::runtime_error);
        model.set_variable_upper_bound(v, 1.);
        model.solve();
        EXPECT_TRUE(is<status::optimal>(model.get_status()));
    }
}

// On an infeasible model the deletion filter relaxes each bound of a BINARY
// column in a trial, so it throws there, after restoring the model.
TEST_F(scip_milp_test, deletion_filter_throws_on_a_binary_column) {
    using namespace operators;
    auto model = new_model();
    auto x = model.add_binary_variable();
    auto r = model.add_constraint(x >= 2.);
    model.solve();
    ASSERT_TRUE(is_a<status::infeasible>(model.get_status()));
    EXPECT_THROW((void)compute_iis_by_deletion(model), std::runtime_error);
    EXPECT_EQ(model.get_variable_lower_bound(x), 0.);
    EXPECT_EQ(model.get_variable_upper_bound(x), 1.);
    EXPECT_EQ(model.get_constraint_lower_bound(r), 2.);
    EXPECT_EQ(model.get_constraint_upper_bound(r), model.infinity());
    model.solve();
    EXPECT_TRUE(is_a<status::infeasible>(model.get_status()));
}

// SCIP rounds the bounds of integer_in_a_fractional_interval into [1, 0]
// and so types that column BINARY: its trials hit the gap pinned above. Any
// other case failing the same way is a regression, not this gap.
struct scip_milp_iis_test : public scip_milp_test {
    template <typename F>
    void SkipOnLicenseError(F && f) {
        try {
            scip_milp_test::SkipOnLicenseError(std::forward<F>(f));
        } catch(const std::runtime_error & e) {
            if(std::string_view(e.what()) != "scip_milp: error in input data" ||
               std::string_view(::testing::UnitTest::GetInstance()
                                    ->current_test_info()
                                    ->name()) !=
                   "integer_in_a_fractional_interval")
                throw;
            GTEST_SKIP() << "SCIP rejects a relaxed bound on a BINARY column: "
                         << e.what();
        }
    }
};

INSTANTIATE_TEST(SCIP, LpModelTest, scip_milp_test);
INSTANTIATE_TEST(SCIP, MilpModelTest, scip_milp_test);
INSTANTIATE_TEST(SCIP, EnumerableEntitiesTest, scip_milp_test);
INSTANTIATE_TEST(SCIP, ReadableObjectiveTest, scip_milp_test);
INSTANTIATE_TEST(SCIP, ModifiableObjectiveTest, scip_milp_test);
INSTANTIATE_TEST(SCIP, ReadableVariablesBoundsTest, scip_milp_test);
INSTANTIATE_TEST(SCIP, ModifiableVariablesBoundsTest, scip_milp_test);
INSTANTIATE_TEST(SCIP, NamedVariablesTest, scip_milp_test);
INSTANTIATE_TEST(SCIP, ReadableConstraintBoundsTest, scip_milp_test);
INSTANTIATE_TEST(SCIP, ModifiableConstraintBoundsTest, scip_milp_test);
INSTANTIATE_TEST(SCIP, IisByDeletionTest, scip_milp_iis_test);
// INSTANTIATE_TEST(SCIP, CandidateSolutionCallbackTest, scip_milp_test);
INSTANTIATE_TEST(SCIP, SudokuTest, scip_milp_test);
INSTANTIATE_TEST(SCIP, TimeLimitTest, scip_milp_test);
INSTANTIATE_TEST(SCIP, TimeLimitIncumbentTest, scip_milp_test);
INSTANTIATE_TEST(SCIP, OptimalityToleranceTest, scip_milp_test);
INSTANTIATE_TEST(SCIP, MipGapTest, scip_milp_test);
INSTANTIATE_TEST(SCIP, VerbosityTest, scip_milp_test);
