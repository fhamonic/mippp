#include "mippp/solvers/soplex/all.hpp"

using namespace fhamonic::mippp;

#include "test_suites/all.hpp"

struct soplex_lp_test : public model_test<soplex_api, soplex_lp> {
    static void SetUpTestSuite() { construct_api(); }
};
INSTANTIATE_TEST(SoPlex, LpModelTest, soplex_lp_test);
INSTANTIATE_TEST(SoPlex, AddColumnTest, soplex_lp_test);
INSTANTIATE_TEST(SoPlex, DualSolutionTest, soplex_lp_test);
// INSTANTIATE_TEST(SoPlex, CuttingStockTest, soplex_lp_test);
