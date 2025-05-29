#include "mippp/solvers/cplex/all.hpp"

using namespace fhamonic::mippp;

#include "test_suites/all.hpp"

struct cplex_lp_test : public model_test<cplex_api, cplex_lp> {
    static void SetUpTestSuite() { construct_api(); }
};
INSTANTIATE_TEST(CPLEX_lp, LpModelTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, ReadableObjectiveTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, ModifiableObjectiveTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, ReadableVariablesBoundsTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, ModifiableVariablesBoundsTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, ColumGenerationTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, DualSolutionTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, LpStatusTest, cplex_lp_test);

struct cplex_milp_test : public model_test<cplex_api, cplex_milp> {
    static void SetUpTestSuite() { construct_api(); }
};
INSTANTIATE_TEST(CPLEX_milp, LpModelTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, MilpModelTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, ReadableObjectiveTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, ModifiableObjectiveTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, ReadableVariablesBoundsTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, ModifiableVariablesBoundsTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, ColumGenerationTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, CandidateSolutionCallbackTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, SudokuTest, cplex_milp_test);
