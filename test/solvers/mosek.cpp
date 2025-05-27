#include "mippp/solvers/mosek/all.hpp"

using namespace fhamonic::mippp;

#include "../test_suites/all.hpp"

struct mosek_lp_test : public model_test<mosek_api, mosek_lp> {
    static void SetUpTestSuite() { construct_api(); }
};
INSTANTIATE_TEST(MOSEK_lp, LpModelTest, mosek_lp_test);
INSTANTIATE_TEST(MOSEK_lp, ReadableObjectiveTest, mosek_lp_test);
INSTANTIATE_TEST(MOSEK_lp, ReadableVariablesBoundsTest, mosek_lp_test);
INSTANTIATE_TEST(MOSEK_lp, ModifiableVariablesBoundsTest, mosek_lp_test);

struct mosek_milp_test : public model_test<mosek_api, mosek_milp> {
    static void SetUpTestSuite() { construct_api(); }
};
INSTANTIATE_TEST(MOSEK_milp, LpModelTest, mosek_milp_test);
INSTANTIATE_TEST(MOSEK_milp, MilpModelTest, mosek_milp_test);
INSTANTIATE_TEST(MOSEK_milp, ReadableObjectiveTest, mosek_milp_test);
INSTANTIATE_TEST(MOSEK_milp, ReadableVariablesBoundsTest, mosek_milp_test);
INSTANTIATE_TEST(MOSEK_milp, ModifiableVariablesBoundsTest, mosek_milp_test);
INSTANTIATE_TEST(MOSEK_milp, SudokuTest, mosek_milp_test);
