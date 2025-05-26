#include "mippp/solvers/cbc/all.hpp"

using namespace fhamonic::mippp;

#include "../test_suites/all.hpp"

struct cbc_milp_test {
    using model_type = cbc_milp;
    cbc_api api;
    auto construct_model() const { return model_type(api); }
};
INSTANTIATE_TEST(Cbc, LpModelTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, MilpModelTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, ReadableObjectiveTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, ReadableVariablesBoundsTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, SudokuTest, cbc_milp_test);