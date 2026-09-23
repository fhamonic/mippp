#include "mippp/solvers/soplex/all.hpp"

using namespace mippp;

#include "test_suites/all.hpp"

MIPPP_API_VERSION_TEST(SoPlex_api, soplex_api, "SOPLEX")

struct soplex_lp_test : public model_test<soplex_api, soplex_lp> {
    static void SetUpTestSuite() { construct_api("SOPLEX"); }
};
INSTANTIATE_TEST(SoPlex, LpModelTest, soplex_lp_test);
INSTANTIATE_TEST(SoPlex, AddColumnTest, soplex_lp_test);
INSTANTIATE_TEST(SoPlex, DualSolutionTest, soplex_lp_test);
// INSTANTIATE_TEST(SoPlex, CuttingStockTest, soplex_lp_test);
INSTANTIATE_TEST(SoPlex, TimeLimitTest, soplex_lp_test);
INSTANTIATE_TEST(SoPlex, VerbosityTest, soplex_lp_test);
