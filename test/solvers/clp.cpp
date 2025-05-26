#include "mippp/solvers/clp/all.hpp"

using namespace fhamonic::mippp;

#include "../test_suites/all.hpp"

struct clp_lp_test {
    using model_type = clp_lp;
    clp_api api;
    auto construct_model() const { return model_type(api); }
};
INSTANTIATE_TEST(Clp, LpModelTest, clp_lp_test);
INSTANTIATE_TEST(Clp, ReadableObjectiveTest, clp_lp_test);
INSTANTIATE_TEST(Clp, ReadableVariablesBoundsTest, clp_lp_test);