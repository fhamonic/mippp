#include "mippp/solvers/gurobi/all.hpp"

using namespace fhamonic::mippp;

#include "../test_suites/all.hpp"

struct gurobi_lp_test {
    using model_type = gurobi_lp;
    gurobi_api api;
    auto construct_model() const { return model_type(api); }
};
INSTANTIATE_TEST(Gurobi_lp, LpModelTest, gurobi_lp_test);
INSTANTIATE_TEST(Gurobi_lp, ReadableObjectiveTest, gurobi_lp_test);
INSTANTIATE_TEST(Gurobi_lp, ReadableVariablesBoundsTest, gurobi_lp_test);

struct gurobi_milp_test {
    using model_type = gurobi_milp;
    gurobi_api api;
    auto construct_model() const { return model_type(api); }
};
INSTANTIATE_TEST(Gurobi_milp, LpModelTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, MilpModelTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, ReadableObjectiveTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, ReadableVariablesBoundsTest, gurobi_milp_test);
INSTANTIATE_TEST(Gurobi_milp, SudokuTest, gurobi_milp_test);