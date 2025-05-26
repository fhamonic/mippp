#include "mippp/solvers/soplex/all.hpp"

using namespace fhamonic::mippp;

#include "../test_suites/all.hpp"

struct soplex_lp_test {
    using model_type = soplex_lp;
    soplex_api api;
    auto construct_model() const { return model_type(api); }
};
INSTANTIATE_TEST(SoPlex, LpModelTest, soplex_lp_test);