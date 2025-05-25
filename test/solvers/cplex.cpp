#include "mippp/solvers/cplex/all.hpp"

using namespace fhamonic::mippp;

#include "../test_suites/lp_model.hpp"
#include "../test_suites/milp_model.hpp"
#include "../test_suites/sudoku.hpp"

struct cplex_lp_test {
    using model_type = cplex_lp;
    cplex_api api;
    auto construct_model() const { return model_type(api); }
};
INSTANTIATE_TYPED_TEST_SUITE_P(CPLEX_lp, LpModelTest,
                               ::testing::Types<cplex_lp_test>);

struct cplex_milp_test {
    using model_type = cplex_milp;
    cplex_api api;
    auto construct_model() const { return model_type(api); }
};
INSTANTIATE_TYPED_TEST_SUITE_P(CPLEX_milp, LpModelTest,
                               ::testing::Types<cplex_milp_test>);
INSTANTIATE_TYPED_TEST_SUITE_P(CPLEX_milp, MilpModelTest,
                               ::testing::Types<cplex_milp_test>);
INSTANTIATE_TYPED_TEST_SUITE_P(CPLEX_milp, SudokuTest,
                               ::testing::Types<cplex_milp_test>);