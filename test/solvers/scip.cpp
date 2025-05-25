#include "mippp/solvers/scip/all.hpp"

using namespace fhamonic::mippp;

#include "../test_suites/lp_model.hpp"
#include "../test_suites/milp_model.hpp"
#include "../test_suites/sudoku.hpp"

struct scip_milp_test {
    using model_type = scip_milp;
    scip_api api;
    auto construct_model() const { return model_type(api); }
};
INSTANTIATE_TYPED_TEST_SUITE_P(SCIP, LpModelTest,
                               ::testing::Types<scip_milp_test>);
INSTANTIATE_TYPED_TEST_SUITE_P(SCIP, MilpModelTest,
                               ::testing::Types<scip_milp_test>);
INSTANTIATE_TYPED_TEST_SUITE_P(SCIP, SudokuTest,
                               ::testing::Types<scip_milp_test>);