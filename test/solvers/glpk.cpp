#include "mippp/solvers/glpk/all.hpp"

using namespace fhamonic::mippp;

#include "../test_suites/lp_model.hpp"
#include "../test_suites/milp_model.hpp"

struct glpk_lp_test {
    using model_type = glpk_lp;
    glpk_api api;
    auto construct_model() const { return model_type(api); }
};
INSTANTIATE_TYPED_TEST_SUITE_P(GLPK_lp, LpModelTest,
                               ::testing::Types<glpk_lp_test>);

struct glpk_milp_test {
    using model_type = glpk_milp;
    glpk_api api;
    auto construct_model() const { return model_type(api); }
};
INSTANTIATE_TYPED_TEST_SUITE_P(GLPK_milp, LpModelTest,
                               ::testing::Types<glpk_milp_test>);
INSTANTIATE_TYPED_TEST_SUITE_P(GLPK_milp, MilpModelTest,
                               ::testing::Types<glpk_milp_test>);