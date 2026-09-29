#include "mippp/solvers/scip/all.hpp"

#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>

using namespace mippp;

#include "test_suites/all.hpp"

MIPPP_API_VERSION_TEST(SCIP_api, scip_api, "SCIP")

namespace {
struct scip_status_probe : scip::impl::v1::scip_milp {
    using scip_milp::_status_code;
};
}  // namespace

TEST(SCIP_status, codes_of_scip_10_are_renumbered) {
    using namespace scip::impl::v1;
    EXPECT_EQ(scip_status_probe::_status_code(10, 1), SCIP_STATUS_OPTIMAL);
    EXPECT_EQ(scip_status_probe::_status_code(10, 2), SCIP_STATUS_INFEASIBLE);
    EXPECT_EQ(scip_status_probe::_status_code(10, 3), SCIP_STATUS_UNBOUNDED);
    EXPECT_EQ(scip_status_probe::_status_code(10, 4), SCIP_STATUS_INFORUNBD);
    EXPECT_EQ(scip_status_probe::_status_code(10, 10),
              SCIP_STATUS_USERINTERRUPT);
    EXPECT_EQ(scip_status_probe::_status_code(10, 11), SCIP_STATUS_TERMINATE);
    EXPECT_EQ(scip_status_probe::_status_code(10, 20), SCIP_STATUS_NODELIMIT);
    EXPECT_EQ(scip_status_probe::_status_code(10, 23), SCIP_STATUS_TIMELIMIT);
    EXPECT_EQ(scip_status_probe::_status_code(10, 25), SCIP_STATUS_GAPLIMIT);
    EXPECT_EQ(scip_status_probe::_status_code(10, 28), SCIP_STATUS_SOLLIMIT);
    EXPECT_EQ(scip_status_probe::_status_code(10, 30),
              SCIP_STATUS_RESTARTLIMIT);
    EXPECT_EQ(scip_status_probe::_status_code(10, 0), SCIP_STATUS_UNKNOWN);
}
TEST(SCIP_status, codes_below_scip_10_are_read_as_declared) {
    using namespace scip::impl::v1;
    EXPECT_EQ(scip_status_probe::_status_code(9, 11), SCIP_STATUS_OPTIMAL);
    EXPECT_EQ(scip_status_probe::_status_code(8, 1), SCIP_STATUS_USERINTERRUPT);
    EXPECT_EQ(scip_status_probe::_status_code(9, 16), SCIP_STATUS_PRIMALLIMIT);
}

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

// SCIP types an integer column BINARY when each of its bounds, rounded inward
// to an integer, is 0 or 1, crossed or not, and every deletion trial that
// relaxes a bound of such a column hits the gap pinned above: a case holding
// one is skipped from its data, before anything is built.
struct scip_milp_iis_test : public scip_milp_test {
    static std::optional<std::string> iis_case_skip_reason(
        const iis_cases::iis_case & c) {
        for(const std::size_t i : c.integer_columns) {
            const auto & bounds = c.system.variables[i];
            const double lower = std::ceil(bounds.lower.value_or(
                -std::numeric_limits<double>::infinity()));
            const double upper = std::floor(
                bounds.upper.value_or(std::numeric_limits<double>::infinity()));
            if((lower == 0. || lower == 1.) && (upper == 0. || upper == 1.))
                return "SCIP types integer column " + std::to_string(i) +
                       " BINARY and rejects a solve once a bound of it leaves "
                       "[0, 1]";
        }
        return std::nullopt;
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
