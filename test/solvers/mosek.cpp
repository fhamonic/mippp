#include "mippp/solvers/mosek/all.hpp"

using namespace fhamonic::mippp;

#include "../test_suites/lp_model.hpp"
#include "../test_suites/milp_model.hpp"
#include "../test_suites/sudoku.hpp"

struct mosek_lp_test {
    using model_type = mosek_lp;
    mosek_api api;
    auto construct_model() const { return model_type(api); }
};
INSTANTIATE_TYPED_TEST_SUITE_P(MOSEK_lp, LpModelTest,
                               ::testing::Types<mosek_lp_test>);

struct mosek_milp_test {
    using model_type = mosek_milp;
    mosek_api api;
    auto construct_model() const { return model_type(api); }
};
INSTANTIATE_TYPED_TEST_SUITE_P(MOSEK_milp, LpModelTest,
                               ::testing::Types<mosek_milp_test>);
INSTANTIATE_TYPED_TEST_SUITE_P(MOSEK_milp, MilpModelTest,
                               ::testing::Types<mosek_milp_test>);
INSTANTIATE_TYPED_TEST_SUITE_P(MOSEK_milp, SudokuTest,
                               ::testing::Types<mosek_milp_test>);