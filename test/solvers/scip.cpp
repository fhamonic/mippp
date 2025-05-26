#include "mippp/solvers/scip/all.hpp"

using namespace fhamonic::mippp;

#include "../test_suites/all.hpp"

struct scip_milp_test {
    using model_type = scip_milp;
    scip_api api;
    auto construct_model() const { return model_type(api); }
};
INSTANTIATE_TEST(SCIP, LpModelTest, scip_milp_test);
INSTANTIATE_TEST(SCIP, MilpModelTest, scip_milp_test);
INSTANTIATE_TEST(SCIP, ReadableObjectiveTest, scip_milp_test);
INSTANTIATE_TEST(SCIP, ReadableVariablesBoundsTest, scip_milp_test);
INSTANTIATE_TEST(SCIP, SudokuTest, scip_milp_test);