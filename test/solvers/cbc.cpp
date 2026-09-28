#include "mippp/solvers/cbc/all.hpp"

#include <optional>

using namespace mippp;

#include "test_suites/all.hpp"

MIPPP_API_VERSION_TEST(Cbc_api, cbc_api, "CBC")

struct cbc_milp_test : public model_test<cbc_api, cbc_milp> {
    static void SetUpTestSuite() { construct_api("CBC"); }
};

// Cbc_solve stops before branch and bound, as for an LP
TEST_F(cbc_milp_test, mip_with_an_infeasible_relaxation_is_infeasible) {
    using namespace operators;
    auto model = new_model();
    auto x = model.add_integer_variable({.lower_bound = 0., .upper_bound = 1.});
    model.add_constraint(x >= 2.);
    model.solve();
    EXPECT_TRUE(is<status::infeasible>(model.get_status()));
}

INSTANTIATE_TEST(Cbc, LpModelTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, MilpModelTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, EnumerableEntitiesTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, ReadableObjectiveTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, ModifiableObjectiveTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, ReadableVariablesBoundsTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, MipStartTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, ModifiableVariablesBoundsTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, NamedVariablesTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, NamedConstraintsTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, AddColumnTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, ReadableConstraintsTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, ReadableConstraintBoundsTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, RangedConstraintsTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, SudokuTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, TimeLimitTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, TimeLimitIncumbentTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, OptimalityToleranceTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, MipGapTest, cbc_milp_test);
// INSTANTIATE_TEST(Cbc, IntegralityToleranceTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, VerbosityTest, cbc_milp_test);
