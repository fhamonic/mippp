#include "mippp/solvers/cbc/all.hpp"

using namespace fhamonic::mippp;

#include "../test_suites/lp_model.hpp"
#include "../test_suites/milp_model.hpp"
#include "../test_suites/sudoku.hpp"

struct cbc_milp_test {
    using model_type = cbc_milp;
    cbc_api api;
    auto construct_model() const { return model_type(api); }
};
INSTANTIATE_TYPED_TEST_SUITE_P(Cbc, LpModelTest,
                               ::testing::Types<cbc_milp_test>);
INSTANTIATE_TYPED_TEST_SUITE_P(Cbc, MilpModelTest,
                               ::testing::Types<cbc_milp_test>);
INSTANTIATE_TYPED_TEST_SUITE_P(Cbc, SudokuTest,
                               ::testing::Types<cbc_milp_test>);