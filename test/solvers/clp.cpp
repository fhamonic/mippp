#include "mippp/solvers/clp/all.hpp"

using namespace fhamonic::mippp;

#include "../test_suites/all.hpp"

struct clp_lp_test : public model_test<clp_api, clp_lp> {
    static void SetUpTestSuite() { construct_api(); }
};
INSTANTIATE_TEST(Clp, LpModelTest, clp_lp_test);
INSTANTIATE_TEST(Clp, ReadableObjectiveTest, clp_lp_test);
INSTANTIATE_TEST(Clp, ModifiableObjectiveTest, clp_lp_test);
INSTANTIATE_TEST(Clp, ReadableVariablesBoundsTest, clp_lp_test);
INSTANTIATE_TEST(Clp, ModifiableVariablesBoundsTest, clp_lp_test);
INSTANTIATE_TEST(Clp, ColumGenerationTest, clp_lp_test);
INSTANTIATE_TEST(Clp, DualSolutionTest, clp_lp_test);
INSTANTIATE_TEST(Clp, LpStatusTest, clp_lp_test);
