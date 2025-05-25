#include "mippp/solvers/copt/all.hpp"

using namespace fhamonic::mippp;

#include "../test_suites/lp_model.hpp"
#include "../test_suites/milp_model.hpp"
#include "../test_suites/sudoku.hpp"

struct copt_lp_test {
    using model_type = copt_lp;
    copt_api api;
    auto construct_model() const { return model_type(api); }
};
INSTANTIATE_TYPED_TEST_SUITE_P(COPT_lp, LpModelTest,
                               ::testing::Types<copt_lp_test>);

struct copt_milp_test {
    using model_type = copt_milp;
    copt_api api;
    auto construct_model() const { return model_type(api); }
};
INSTANTIATE_TYPED_TEST_SUITE_P(COPT_milp, LpModelTest,
                               ::testing::Types<copt_milp_test>);
INSTANTIATE_TYPED_TEST_SUITE_P(COPT_milp, MilpModelTest,
                               ::testing::Types<copt_milp_test>);
INSTANTIATE_TYPED_TEST_SUITE_P(COPT_milp, SudokuTest,
                               ::testing::Types<copt_milp_test>);