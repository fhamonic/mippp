#include "mippp/solvers/cbc/all.hpp"

#include <optional>
#include <random>
#include <ranges>
#include <vector>

using namespace mippp;

#include "test_suites/all.hpp"

MIPPP_API_VERSION_TEST(Cbc_api, cbc_api, "CBC")

struct cbc_milp_test : public model_test<cbc_api, cbc_milp> {
    static void SetUpTestSuite() { construct_api("CBC"); }

    // Cornuejols-Dawande market split rows without slacks: the relaxation is
    // feasible and branch and bound finds no integer point in a few nodes.
    static void add_market_split_rows(
        cbc_milp & model, const std::vector<model_variable_t<cbc_milp>> & x) {
        using namespace operators;
        std::mt19937 rng(7);
        std::uniform_int_distribution<int> coef(0, 99);
        std::vector<int> row(x.size());
        for(std::size_t i = 0; i < 4; ++i) {
            int total = 0;
            for(int & a : row) total += (a = coef(rng));
            model.add_constraint(
                xsum(std::views::iota(std::size_t{0}, x.size()),
                     [&](std::size_t j) { return row[j] * x[j]; }) ==
                total / 2);
        }
    }
};

// Cbc_solve stops before branch and bound, as for an LP
TEST_F(cbc_milp_test, mip_with_an_infeasible_relaxation_is_infeasible) {
    using namespace operators;
    auto model = new_model();
    auto x = model.add_integer_variable({.lower_bound = 0., .upper_bound = 1.});
    model.add_constraint(x >= 2.);
    model.solve();
    EXPECT_TRUE(is<status::infeasible>(model.get_status()));
}

TEST_F(cbc_milp_test, limit_stop_does_not_claim_an_earlier_solution) {
    auto model = new_model();
    std::vector<model_variable_t<cbc_milp>> x;
    for(int j = 0; j < 30; ++j) x.push_back(model.add_binary_variable());
    model.solve();
    ASSERT_TRUE(is<status::optimal>(model.get_status()));
    add_market_split_rows(model, x);
    model.set_node_limit(3);
    model.solve();
    ASSERT_TRUE(is<status::node_limit>(model.get_status()));
    EXPECT_FALSE(std::visit([](auto s) { return s.solution_available; },
                            model.get_status()));
}

INSTANTIATE_TEST(Cbc, LpModelTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, MilpModelTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, EnumerableEntitiesTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, ReadableObjectiveTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, ModifiableObjectiveTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, ReadableVariablesBoundsTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, MipStartTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, ModifiableVariablesBoundsTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, NamedVariablesTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, NamedConstraintsTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, AddColumnTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, ReadableConstraintsTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, ReadableConstraintBoundsTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, ModifiableConstraintBoundsTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, RangedConstraintsTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, SudokuTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, TimeLimitTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, TimeLimitIncumbentTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, OptimalityToleranceTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, MipGapTest, cbc_milp_test);
// INSTANTIATE_TEST(Cbc, IntegralityToleranceTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, VerbosityTest, cbc_milp_test);
