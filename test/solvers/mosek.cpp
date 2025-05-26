#include "mippp/solvers/mosek/all.hpp"

using namespace fhamonic::mippp;

#include "../test_suites/all.hpp"

struct mosek_lp_test {
    using model_type = mosek_lp;
    mosek_api api;
    auto construct_model() const { return model_type(api); }
};
INSTANTIATE_TEST(MOSEK_lp, LpModelTest, mosek_lp_test);
INSTANTIATE_TEST(MOSEK_lp, ReadableObjectiveTest, mosek_lp_test);
INSTANTIATE_TEST(MOSEK_lp, ReadableVariablesBoundsTest, mosek_lp_test);

struct mosek_milp_test {
    using model_type = mosek_milp;
    mosek_api api;
    auto construct_model() const { return model_type(api); }
};
INSTANTIATE_TEST(MOSEK_milp, LpModelTest, mosek_milp_test);
INSTANTIATE_TEST(MOSEK_milp, MilpModelTest, mosek_milp_test);
INSTANTIATE_TEST(MOSEK_milp, ReadableObjectiveTest, mosek_milp_test);
INSTANTIATE_TEST(MOSEK_milp, ReadableVariablesBoundsTest, mosek_milp_test);
INSTANTIATE_TEST(MOSEK_milp, SudokuTest, mosek_milp_test);