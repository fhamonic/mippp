#include "mippp/solvers/highs/all.hpp"

using namespace fhamonic::mippp;

#include "test_suites/all.hpp"

struct highs_lp_test : public model_test<highs_api, highs_lp> {
    static void SetUpTestSuite() { construct_api(); }
};
INSTANTIATE_TEST(HiGHS_lp, LpModelTest, highs_lp_test);
INSTANTIATE_TEST(HiGHS_lp, ReadableObjectiveTest, highs_lp_test);
INSTANTIATE_TEST(HiGHS_lp, ModifiableObjectiveTest, highs_lp_test);
INSTANTIATE_TEST(HiGHS_lp, ReadableVariablesBoundsTest, highs_lp_test);
INSTANTIATE_TEST(HiGHS_lp, ModifiableVariablesBoundsTest, highs_lp_test);
INSTANTIATE_TEST(HiGHS_lp, NamedVariablesTest, highs_lp_test);
INSTANTIATE_TEST(HiGHS_lp, AddColumnTest, highs_lp_test);
INSTANTIATE_TEST(HiGHS_lp, DualSolutionTest, highs_lp_test);
INSTANTIATE_TEST(HiGHS_lp, LpStatusTest, highs_lp_test);
INSTANTIATE_TEST(HiGHS_lp, CuttingStockTest, highs_lp_test);

struct highs_milp_test : public model_test<highs_api, highs_milp> {
    static void SetUpTestSuite() { construct_api(); }
};
INSTANTIATE_TEST(HiGHS_milp, LpModelTest, highs_milp_test);
INSTANTIATE_TEST(HiGHS_milp, MilpModelTest, highs_milp_test);
INSTANTIATE_TEST(HiGHS_milp, ReadableObjectiveTest, highs_milp_test);
INSTANTIATE_TEST(HiGHS_milp, ModifiableObjectiveTest, highs_milp_test);
INSTANTIATE_TEST(HiGHS_milp, ReadableVariablesBoundsTest, highs_milp_test);
INSTANTIATE_TEST(HiGHS_milp, ModifiableVariablesBoundsTest, highs_milp_test);
INSTANTIATE_TEST(HiGHS_milp, NamedVariablesTest, highs_milp_test);
INSTANTIATE_TEST(HiGHS_milp, AddColumnTest, highs_milp_test);
INSTANTIATE_TEST(HiGHS_milp, SudokuTest, highs_milp_test);

struct highs_qp_test : public model_test<highs_api, highs_qp> {
    static void SetUpTestSuite() { construct_api(); }
};
INSTANTIATE_TEST(HiGHS_qp, LpModelTest, highs_qp_test);
INSTANTIATE_TEST(HiGHS_qp, QpModelTest, highs_qp_test);
// INSTANTIATE_TEST(HiGHS_qp, ReadableObjectiveTest, highs_qp_test);
// INSTANTIATE_TEST(HiGHS_qp, ModifiableObjectiveTest, highs_qp_test);
// INSTANTIATE_TEST(HiGHS_qp, ReadableVariablesBoundsTest, highs_qp_test);
// INSTANTIATE_TEST(HiGHS_qp, ModifiableVariablesBoundsTest, highs_qp_test);
// INSTANTIATE_TEST(HiGHS_qp, NamedVariablesTest, highs_qp_test);
// INSTANTIATE_TEST(HiGHS_qp, AddColumnTest, highs_qp_test);
// INSTANTIATE_TEST(HiGHS_qp, DualSolutionTest, highs_qp_test);
// INSTANTIATE_TEST(HiGHS_qp, LpStatusTest, highs_qp_test);
// INSTANTIATE_TEST(HiGHS_qp, CuttingStockTest, highs_qp_test);
