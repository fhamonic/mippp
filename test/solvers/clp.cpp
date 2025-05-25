#include "mippp/solvers/clp/all.hpp"

using namespace fhamonic::mippp;

#include "../test_suites/lp_model.hpp"

struct clp_lp_test {
    using model_type = clp_lp;
    clp_api api;
    auto construct_model() const { return model_type(api); }
};
INSTANTIATE_TYPED_TEST_SUITE_P(Clp, LpModelTest, ::testing::Types<clp_lp_test>);