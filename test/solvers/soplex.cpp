#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/solvers/soplex/all.hpp"

using namespace mippp;

#include "test_suites/all.hpp"

MIPPP_API_VERSION_TEST(SoPlex_api, soplex_api, "SOPLEX")

struct soplex_lp_test : public model_test<soplex_api, soplex_lp> {
    static void SetUpTestSuite() { construct_api("SOPLEX"); }
};
// The column's single entry sits on row 1, at its nonzero count: a length
// taken from that count would drop it.
TEST_F(soplex_lp_test, column_entry_past_its_nonzero_count_is_kept) {
    using namespace operators;
    auto model = new_model();
    auto x = model.add_variable();
    model.add_constraint(x <= 4.0);
    auto c1 = model.add_constraint(x <= 4.0);
    auto y = model.add_column({{c1, 2.0}});
    model.set_maximization();
    model.set_objective(y);
    model.solve();
    ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
    EXPECT_NEAR(model.get_solution_value(), 2.0, TEST_EPSILON);
}
INSTANTIATE_TEST(SoPlex, LpModelTest, soplex_lp_test);
INSTANTIATE_TEST(SoPlex, EnumerableEntitiesTest, soplex_lp_test);
INSTANTIATE_TEST(SoPlex, AddColumnTest, soplex_lp_test);
INSTANTIATE_TEST(SoPlex, ReadableConstraintBoundsTest, soplex_lp_test);
INSTANTIATE_TEST(SoPlex, DualSolutionTest, soplex_lp_test);
INSTANTIATE_TEST(SoPlex, CuttingStockTest, soplex_lp_test);
INSTANTIATE_TEST(SoPlex, TimeLimitTest, soplex_lp_test);
INSTANTIATE_TEST(SoPlex, VerbosityTest, soplex_lp_test);
