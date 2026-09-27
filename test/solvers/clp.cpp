#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/solvers/clp/all.hpp"

using namespace mippp;

#include "test_suites/all.hpp"

MIPPP_API_VERSION_TEST(Clp_api, clp_api, "CLP")

namespace {
struct clp_lp_status_probe : clp::impl::v1::clp_lp {
    using clp_lp::_simplex_status;
};

auto simplex_status(int problem_status, int secondary_status) {
    return clp_lp_status_probe::_simplex_status(problem_status,
                                                secondary_status);
}
}  // namespace

TEST(Clp_simplex_status, unscaled_primal_infeasibilities_are_flagged) {
    for(int secondary_status : {2, 4})
        EXPECT_TRUE(is<status::optimal_infeasible_unscaled>(
            simplex_status(0, secondary_status)))
            << secondary_status;
}
TEST(Clp_simplex_status, unscaled_dual_infeasibilities_stay_optimal) {
    EXPECT_TRUE(is<status::optimal>(simplex_status(0, 0)));
    EXPECT_TRUE(is<status::optimal>(simplex_status(0, 3)));
}
TEST(Clp_simplex_status, other_problem_statuses_ignore_the_secondary) {
    for(int secondary_status : {0, 1, 2, 4}) {
        EXPECT_TRUE(is<status::infeasible>(simplex_status(1, secondary_status)))
            << secondary_status;
        EXPECT_TRUE(is<status::unbounded>(simplex_status(2, secondary_status)))
            << secondary_status;
        for(int problem_status : {-1, 3, 4, 5})
            EXPECT_TRUE(is<status::unknown>(
                simplex_status(problem_status, secondary_status)))
                << problem_status << ' ' << secondary_status;
    }
}

struct clp_lp_test : public model_test<clp_api, clp_lp> {
    static void SetUpTestSuite() { construct_api("CLP"); }
};
// infeasible, but scaling shrinks the violation at x = 0 below Clp's
// tolerance, and its primary status says optimal
TEST_F(clp_lp_test, infeasible_once_unscaled_is_not_plain_optimal) {
    using namespace operators;
    auto model = new_model();
    auto x = model.add_variable();
    model.add_constraint(1e8 * x <= -1e-3);
    model.solve();
    EXPECT_TRUE(is<status::optimal_infeasible_unscaled>(model.get_status()));
}
INSTANTIATE_TEST(Clp, LpModelTest, clp_lp_test);
INSTANTIATE_TEST(Clp, EnumerableEntitiesTest, clp_lp_test);
INSTANTIATE_TEST(Clp, ReadableObjectiveTest, clp_lp_test);
INSTANTIATE_TEST(Clp, ModifiableObjectiveTest, clp_lp_test);
INSTANTIATE_TEST(Clp, ReadableVariablesBoundsTest, clp_lp_test);
INSTANTIATE_TEST(Clp, ModifiableVariablesBoundsTest, clp_lp_test);
INSTANTIATE_TEST(Clp, NamedVariablesTest, clp_lp_test);
INSTANTIATE_TEST(Clp, NamedConstraintsTest, clp_lp_test);
INSTANTIATE_TEST(Clp, AddColumnTest, clp_lp_test);
INSTANTIATE_TEST(Clp, RangedConstraintsTest, clp_lp_test);
INSTANTIATE_TEST(Clp, ReadableConstraintBoundsTest, clp_lp_test);
INSTANTIATE_TEST(Clp, ModifiableConstraintBoundsTest, clp_lp_test);
INSTANTIATE_TEST(Clp, RemoveVariableTest, clp_lp_test);
INSTANTIATE_TEST(Clp, DualSolutionTest, clp_lp_test);
INSTANTIATE_TEST(Clp, ReducedCostsTest, clp_lp_test);
INSTANTIATE_TEST(Clp, LpStatusTest, clp_lp_test);
INSTANTIATE_TEST(Clp, CuttingStockTest, clp_lp_test);
INSTANTIATE_TEST(Clp, ColumnManagerTest, clp_lp_test);
INSTANTIATE_TEST(Clp, LpFuzzyTest, clp_lp_test);
INSTANTIATE_TEST(Clp, VerbosityTest, clp_lp_test);
