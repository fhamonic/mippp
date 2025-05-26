#include "mippp/solvers/highs/all.hpp"

using namespace fhamonic::mippp;

#include "../test_suites/all.hpp"

struct highs_lp_test {
    using model_type = highs_lp;
    highs_api api;
    auto construct_model() const { return model_type(api); }
};
INSTANTIATE_TEST(HiGHS_lp, LpModelTest, highs_lp_test);
INSTANTIATE_TEST(HiGHS_lp, ReadableObjectiveTest, highs_lp_test);
INSTANTIATE_TEST(HiGHS_lp, ReadableVariablesBoundsTest, highs_lp_test);

struct highs_milp_test {
    using model_type = highs_milp;
    highs_api api;
    auto construct_model() const { return model_type(api); }
};
INSTANTIATE_TEST(HiGHS_milp, LpModelTest, highs_milp_test);
INSTANTIATE_TEST(HiGHS_milp, MilpModelTest, highs_milp_test);
INSTANTIATE_TEST(HiGHS_milp, ReadableObjectiveTest, highs_milp_test);
INSTANTIATE_TEST(HiGHS_milp, ReadableVariablesBoundsTest, highs_milp_test);
INSTANTIATE_TEST(HiGHS_milp, SudokuTest, highs_milp_test);