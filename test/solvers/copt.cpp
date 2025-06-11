#include "mippp/solvers/copt/all.hpp"

using namespace fhamonic::mippp;

#include "test_suites/all.hpp"

struct copt_lp_test : public model_test<copt_api, copt_lp> {
    static void SetUpTestSuite() { construct_api(); }
};
INSTANTIATE_TEST(COPT_lp, LpModelTest, copt_lp_test);
INSTANTIATE_TEST(COPT_lp, ReadableObjectiveTest, copt_lp_test);
INSTANTIATE_TEST(COPT_lp, ModifiableObjectiveTest, copt_lp_test);
INSTANTIATE_TEST(COPT_lp, ReadableVariablesBoundsTest, copt_lp_test);
INSTANTIATE_TEST(COPT_lp, ModifiableVariablesBoundsTest, copt_lp_test);
INSTANTIATE_TEST(COPT_lp, NamedVariablesTest, copt_lp_test);
INSTANTIATE_TEST(COPT_lp, ColumGenerationTest, copt_lp_test);
INSTANTIATE_TEST(COPT_lp, DualSolutionTest, copt_lp_test);
INSTANTIATE_TEST(COPT_lp, LpStatusTest, copt_lp_test);

struct copt_milp_test : public model_test<copt_api, copt_milp> {
    static void SetUpTestSuite() { construct_api(); }
};
INSTANTIATE_TEST(COPT_milp, LpModelTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, MilpModelTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, ReadableObjectiveTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, ModifiableObjectiveTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, ReadableVariablesBoundsTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, ModifiableVariablesBoundsTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, NamedVariablesTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, ColumGenerationTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, SudokuTest, copt_milp_test);
