#include "dumb_lp.hpp"

using namespace mippp;

#include "test_suites/all.hpp"

struct dumb_lp_test : public model_test<clp_api, dumb_lp> {
    static void SetUpTestSuite() { construct_api(); }
};
INSTANTIATE_TEST(Dumb, LpModelTest, dumb_lp_test);
INSTANTIATE_TEST(Dumb, ReadableObjectiveTest, dumb_lp_test);
INSTANTIATE_TEST(Dumb, ModifiableObjectiveTest, dumb_lp_test);
INSTANTIATE_TEST(Dumb, ReadableVariablesBoundsTest, dumb_lp_test);
INSTANTIATE_TEST(Dumb, ModifiableVariablesBoundsTest, dumb_lp_test);
INSTANTIATE_TEST(Dumb, NamedVariablesTest, dumb_lp_test);
INSTANTIATE_TEST(Dumb, ReadableConstraintsTest, dumb_lp_test);
INSTANTIATE_TEST(Dumb, AddColumnTest, dumb_lp_test);
INSTANTIATE_TEST(Dumb, RemoveVariableTest, dumb_lp_test);
INSTANTIATE_TEST(Dumb, DualSolutionTest, dumb_lp_test);
INSTANTIATE_TEST(Dumb, LpStatusTest, dumb_lp_test);
INSTANTIATE_TEST(Dumb, CuttingStockTest, dumb_lp_test);
INSTANTIATE_TEST(Dumb, LpFuzzyTest, dumb_lp_test);
