#include "mippp/solvers/copt/all.hpp"

using namespace fhamonic::mippp;

#include "../test_suites/all.hpp"

struct copt_lp_test {
    using model_type = copt_lp;
    copt_api api;
    auto construct_model() const { return model_type(api); }
};
INSTANTIATE_TEST(COPT_lp, LpModelTest, copt_lp_test);
INSTANTIATE_TEST(COPT_lp, ReadableObjectiveTest, copt_lp_test);
INSTANTIATE_TEST(COPT_lp, ReadableVariablesBoundsTest, copt_lp_test);

struct copt_milp_test {
    using model_type = copt_milp;
    copt_api api;
    auto construct_model() const { return model_type(api); }
};
INSTANTIATE_TEST(COPT_milp, LpModelTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, MilpModelTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, ReadableObjectiveTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, ReadableVariablesBoundsTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, SudokuTest, copt_milp_test);