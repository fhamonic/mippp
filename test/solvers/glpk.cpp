#include "mippp/solvers/glpk/all.hpp"

using namespace fhamonic::mippp;

#include "../test_suites/all.hpp"

struct glpk_lp_test : public model_test<glpk_api, glpk_lp> {
    static void SetUpTestSuite() { construct_api(); }
};
INSTANTIATE_TEST(GLPK_lp, LpModelTest, glpk_lp_test);
INSTANTIATE_TEST(GLPK_lp, ReadableObjectiveTest, glpk_lp_test);
INSTANTIATE_TEST(GLPK_lp, ModifiableObjectiveTest, glpk_lp_test);
INSTANTIATE_TEST(GLPK_lp, ReadableVariablesBoundsTest, glpk_lp_test);
INSTANTIATE_TEST(GLPK_lp, ModifiableVariablesBoundsTest, glpk_lp_test);
INSTANTIATE_TEST(GLPK_lp, DualSolutionTest, glpk_lp_test);
INSTANTIATE_TEST(GLPK_lp, LpStatusTest, glpk_lp_test);

struct glpk_milp_test : public model_test<glpk_api, glpk_milp> {
    static void SetUpTestSuite() { construct_api(); }
};
INSTANTIATE_TEST(GLPK_milp, LpModelTest, glpk_milp_test);
INSTANTIATE_TEST(GLPK_milp, MilpModelTest, glpk_milp_test);
INSTANTIATE_TEST(GLPK_milp, ReadableObjectiveTest, glpk_milp_test);
INSTANTIATE_TEST(GLPK_milp, ModifiableObjectiveTest, glpk_milp_test);
INSTANTIATE_TEST(GLPK_milp, ReadableVariablesBoundsTest, glpk_milp_test);
INSTANTIATE_TEST(GLPK_milp, ModifiableVariablesBoundsTest, glpk_milp_test);
