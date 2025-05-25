#include "mippp/solvers/gurobi/all.hpp"

using namespace fhamonic::mippp;

#include "../test_suites/lp_model.hpp"
#include "../test_suites/milp_model.hpp"
#include "../test_suites/sudoku.hpp"

struct gurobi_lp_test {
    using model_type = gurobi_lp;
    gurobi_api api;
    auto construct_model() const { return model_type(api); }
};
INSTANTIATE_TYPED_TEST_SUITE_P(Gurobi_lp, LpModelTest,
                               ::testing::Types<gurobi_lp_test>);

struct gurobi_milp_test {
    using model_type = gurobi_milp;
    gurobi_api api;
    auto construct_model() const { return model_type(api); }
};
INSTANTIATE_TYPED_TEST_SUITE_P(Gurobi_milp, LpModelTest,
                               ::testing::Types<gurobi_milp_test>);
INSTANTIATE_TYPED_TEST_SUITE_P(Gurobi_milp, MilpModelTest,
                               ::testing::Types<gurobi_milp_test>);
INSTANTIATE_TYPED_TEST_SUITE_P(Gurobi_milp, SudokuTest,
                               ::testing::Types<gurobi_milp_test>);