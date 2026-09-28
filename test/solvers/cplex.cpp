#include <stdexcept>
#include <vector>

#include "mippp/solvers/cplex/all.hpp"

using namespace mippp;

#include "test_suites/all.hpp"

MIPPP_API_VERSION_TEST(CPLEX_api, cplex_api, "CPLEX")

namespace {
struct fake_cplex_release_api {
    std::vector<char> & released;
    std::vector<cplex::impl::v1::CPXENVptr> & freeprob_envs;
    int freeprob(cplex::impl::v1::CPXENVptr env,
                 cplex::impl::v1::CPXLPptr *) const {
        released.push_back('l');
        freeprob_envs.push_back(env);
        return -1;
    }
    int closeCPLEX(cplex::impl::v1::CPXENVptr *) const {
        released.push_back('e');
        return -1;
    }
};
}  // namespace

TEST(CPLEX_handle_guard, releases_partial_allocations_without_throwing) {
    using namespace cplex::impl::v1;
    int storage = 0;
    const auto handle = reinterpret_cast<CPXENVptr>(&storage);
    for(unsigned allocated : {0u, 1u, 2u}) {
        std::vector<char> released;
        std::vector<CPXENVptr> freeprob_envs;
        const fake_cplex_release_api api{released, freeprob_envs};
        CPXENVptr env = nullptr;
        CPXLPptr lp = nullptr;
        try {
            mippp::detail::handle_guard<fake_cplex_release_api, CPXENVptr,
                                        CPXLPptr, cplex_handle_release>
                guard(api, env, lp);
            if(allocated >= 1) env = handle;
            if(allocated >= 2) lp = reinterpret_cast<CPXLPptr>(&storage);
            throw std::runtime_error("construction failure");
        } catch(const std::runtime_error & e) {
            EXPECT_STREQ(e.what(), "construction failure");
        }
        EXPECT_EQ(env, nullptr);
        EXPECT_EQ(lp, nullptr);
        const std::vector<char> expected =
            allocated == 2   ? std::vector<char>{'l', 'e'}
            : allocated == 1 ? std::vector<char>{'e'}
                             : std::vector<char>{};
        EXPECT_EQ(released, expected);
        EXPECT_EQ(freeprob_envs, allocated == 2 ? std::vector<CPXENVptr>{handle}
                                                : std::vector<CPXENVptr>{});
    }
}

struct cplex_lp_test : public model_test<cplex_api, cplex_lp> {
    static void SetUpTestSuite() { construct_api("CPLEX"); }
};
INSTANTIATE_TEST(CPLEX_lp, LpModelTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, ReadableObjectiveTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, ModifiableObjectiveTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, ReadableVariablesBoundsTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, ModifiableVariablesBoundsTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, NamedVariablesTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, AddColumnTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, RemoveVariableTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, ReadableConstraintsTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, DualSolutionTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, ReducedCostsTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, LpStatusTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, CuttingStockTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, ColumnManagerTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, LpFuzzyTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, TimeLimitTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, IterationLimitTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, OptimalityToleranceTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, VerbosityTest, cplex_lp_test);

struct cplex_milp_test : public model_test<cplex_api, cplex_milp> {
    static void SetUpTestSuite() { construct_api("CPLEX"); }
};
INSTANTIATE_TEST(CPLEX_milp, LpModelTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, MilpModelTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, ReadableObjectiveTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, ModifiableObjectiveTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, ReadableVariablesBoundsTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, ModifiableVariablesBoundsTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, NamedVariablesTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, AddColumnTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, RemoveVariableTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, ReadableConstraintsTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, SudokuTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, CandidateSolutionCallbackTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, LazyConstraintsTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, TravellingSalesmanTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, TimeLimitTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, TimeLimitIncumbentTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, MipStartTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, OptimalityToleranceTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, MipGapTest, cplex_milp_test);
// INSTANTIATE_TEST(CPLEX_milp, IntegralityToleranceTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, VerbosityTest, cplex_milp_test);
