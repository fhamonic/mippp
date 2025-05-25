#include "mippp/solvers/highs/all.hpp"

using namespace fhamonic::mippp;

#include "../test_suites/lp_model.hpp"
#include "../test_suites/milp_model.hpp"
#include "../test_suites/sudoku.hpp"

struct highs_lp_test {
    using model_type = highs_lp;
    highs_api api;
    auto construct_model() const { return model_type(api); }
};
INSTANTIATE_TYPED_TEST_SUITE_P(HiGHS_lp, LpModelTest,
                               ::testing::Types<highs_lp_test>);

struct highs_milp_test {
    using model_type = highs_milp;
    highs_api api;
    auto construct_model() const { return model_type(api); }
};
INSTANTIATE_TYPED_TEST_SUITE_P(HiGHS_milp, LpModelTest,
                               ::testing::Types<highs_milp_test>);
INSTANTIATE_TYPED_TEST_SUITE_P(HiGHS_milp, MilpModelTest,
                               ::testing::Types<highs_milp_test>);
INSTANTIATE_TYPED_TEST_SUITE_P(HiGHS_milp, SudokuTest,
                               ::testing::Types<highs_milp_test>);