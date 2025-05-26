#include "mippp/solvers/cplex/all.hpp"

using namespace fhamonic::mippp;

#include "../test_suites/all.hpp"

struct cplex_lp_test {
    using model_type = cplex_lp;
    cplex_api api;
    auto construct_model() const { return model_type(api); }
};
INSTANTIATE_TEST(CPLEX_lp, LpModelTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, ReadableObjectiveTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, ReadableVariablesBoundsTest, cplex_lp_test);

struct cplex_milp_test {
    using model_type = cplex_milp;
    cplex_api api;
    auto construct_model() const { return model_type(api); }
};
INSTANTIATE_TEST(CPLEX_milp, LpModelTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, MilpModelTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, ReadableObjectiveTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, ReadableVariablesBoundsTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, SudokuTest, cplex_milp_test);