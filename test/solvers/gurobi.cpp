#include "mippp/solvers/gurobi/all.hpp"

using namespace fhamonic::mippp;

#include "test_suites/all.hpp"

struct gurobi_lp_test : public model_test<gurobi_api, gurobi_lp> {
    static void SetUpTestSuite() { construct_api(); }
};
INSTANTIATE_TEST(Gurobi_lp, LpModelTest, gurobi_lp_test);
INSTANTIATE_TEST(Gurobi_lp, ReadableObjectiveTest, gurobi_lp_test);
INSTANTIATE_TEST(Gurobi_lp, ModifiableObjectiveTest, gurobi_lp_test);
INSTANTIATE_TEST(Gurobi_lp, ReadableVariablesBoundsTest, gurobi_lp_test);
INSTANTIATE_TEST(Gurobi_lp, ModifiableVariablesBoundsTest, gurobi_lp_test);
INSTANTIATE_TEST(Gurobi_lp, NamedVariablesTest, gurobi_lp_test);
INSTANTIATE_TEST(Gurobi_lp, AddColumnTest, gurobi_lp_test);
INSTANTIATE_TEST(Gurobi_lp, LpStatusTest, gurobi_lp_test);
INSTANTIATE_TEST(Gurobi_lp, DualSolutionTest, gurobi_lp_test);
INSTANTIATE_TEST(Gurobi_lp, CuttingStockTest, gurobi_lp_test);

struct gurobi_milp_test : public model_test<gurobi_api, gurobi_milp> {
    static void SetUpTestSuite() { construct_api(); }
};
INSTANTIATE_TEST(Gurobi_milp, LpModelTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, MilpModelTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, ReadableObjectiveTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, ModifiableObjectiveTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, ReadableVariablesBoundsTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, ModifiableVariablesBoundsTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, NamedVariablesTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, AddColumnTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, CandidateSolutionCallbackTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, SudokuTest, gurobi_milp_test);
