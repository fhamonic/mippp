#include "mippp/solvers/glpk/all.hpp"

using namespace fhamonic::mippp;

#include "../test_suites/all.hpp"

struct glpk_lp_test {
    using model_type = glpk_lp;
    glpk_api api;
    auto construct_model() const { return model_type(api); }
};
INSTANTIATE_TEST(GLPK_lp, LpModelTest, glpk_lp_test);
INSTANTIATE_TEST(GLPK_lp, ReadableObjectiveTest, glpk_lp_test);
INSTANTIATE_TEST(GLPK_lp, ReadableVariablesBoundsTest, glpk_lp_test);

struct glpk_milp_test {
    using model_type = glpk_milp;
    glpk_api api;
    auto construct_model() const { return model_type(api); }
};
INSTANTIATE_TEST(GLPK_milp, LpModelTest, glpk_milp_test);
INSTANTIATE_TEST(GLPK_milp, MilpModelTest, glpk_milp_test);
INSTANTIATE_TEST(GLPK_milp, ReadableObjectiveTest, glpk_milp_test);
INSTANTIATE_TEST(GLPK_milp, ReadableVariablesBoundsTest, glpk_milp_test);