#include "mippp/solvers/cbc/all.hpp"

#include <optional>

using namespace fhamonic::mippp;

#include "test_suites/all.hpp"

struct cbc_milp_test : public model_test<cbc_api, cbc_milp> {
    static void SetUpTestSuite() { construct_api(); }
};
INSTANTIATE_TEST(Cbc, LpModelTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, MilpModelTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, ReadableObjectiveTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, ModifiableObjectiveTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, ReadableVariablesBoundsTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, ModifiableVariablesBoundsTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, NamedVariablesTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, ColumGenerationTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, SudokuTest, cbc_milp_test);
