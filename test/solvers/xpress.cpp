#include "mippp/solvers/xpress/all.hpp"

using namespace fhamonic::mippp;

#include "test_suites/all.hpp"

struct xpress_lp_test : public model_test<xpress_api, xpress_lp> {
    static void SetUpTestSuite() { construct_api(); }
};
INSTANTIATE_TEST(Xpress_lp, LpModelTest, xpress_lp_test);
INSTANTIATE_TEST(Xpress_lp, ReadableObjectiveTest, xpress_lp_test);
INSTANTIATE_TEST(Xpress_lp, ModifiableObjectiveTest, xpress_lp_test);
INSTANTIATE_TEST(Xpress_lp, ReadableVariablesBoundsTest, xpress_lp_test);
INSTANTIATE_TEST(Xpress_lp, ModifiableVariablesBoundsTest, xpress_lp_test);
INSTANTIATE_TEST(Xpress_lp, NamedVariablesTest, xpress_lp_test);
INSTANTIATE_TEST(Xpress_lp, AddColumnTest, xpress_lp_test);
INSTANTIATE_TEST(Xpress_lp, DualSolutionTest, xpress_lp_test);
INSTANTIATE_TEST(Xpress_lp, LpStatusTest, xpress_lp_test);
INSTANTIATE_TEST(Xpress_lp, CuttingStockTest, xpress_lp_test);

struct xpress_milp_test : public model_test<xpress_api, xpress_milp> {
    static void SetUpTestSuite() { construct_api(); }
};
INSTANTIATE_TEST(Xpress_milp, LpModelTest, xpress_milp_test);
INSTANTIATE_TEST(Xpress_milp, MilpModelTest, xpress_milp_test);
INSTANTIATE_TEST(Xpress_milp, ReadableObjectiveTest, xpress_milp_test);
INSTANTIATE_TEST(Xpress_milp, ModifiableObjectiveTest, xpress_milp_test);
INSTANTIATE_TEST(Xpress_milp, ReadableVariablesBoundsTest, xpress_milp_test);
INSTANTIATE_TEST(Xpress_milp, ModifiableVariablesBoundsTest, xpress_milp_test);
INSTANTIATE_TEST(Xpress_milp, NamedVariablesTest, xpress_milp_test);
INSTANTIATE_TEST(Xpress_milp, AddColumnTest, xpress_milp_test);
// INSTANTIATE_TEST(Xpress_milp, CandidateSolutionCallbackTest, xpress_milp_test);
INSTANTIATE_TEST(Xpress_milp, SudokuTest, xpress_milp_test);
