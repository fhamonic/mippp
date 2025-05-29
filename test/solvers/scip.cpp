#include "mippp/solvers/scip/all.hpp"

using namespace fhamonic::mippp;

#include "test_suites/all.hpp"

struct scip_milp_test : public model_test<scip_api, scip_milp> {
    static void SetUpTestSuite() { construct_api(); }
};
INSTANTIATE_TEST(SCIP, LpModelTest, scip_milp_test);
INSTANTIATE_TEST(SCIP, MilpModelTest, scip_milp_test);
INSTANTIATE_TEST(SCIP, ReadableObjectiveTest, scip_milp_test);
INSTANTIATE_TEST(SCIP, ModifiableObjectiveTest, scip_milp_test);
INSTANTIATE_TEST(SCIP, ReadableVariablesBoundsTest, scip_milp_test);
INSTANTIATE_TEST(SCIP, ModifiableVariablesBoundsTest, scip_milp_test);
INSTANTIATE_TEST(SCIP, SudokuTest, scip_milp_test);
