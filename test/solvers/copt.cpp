#include <stdexcept>
#include <vector>

#include "mippp/solvers/copt/all.hpp"

using namespace mippp;

#include "test_suites/all.hpp"

MIPPP_API_VERSION_TEST(COPT_api, copt_api, "COPT")

namespace {
struct fake_copt_release_api {
    std::vector<char> & released;
    int DeleteProb(copt::impl::v1::copt_prob **) const {
        released.push_back('p');
        return -1;
    }
    int DeleteEnv(copt::impl::v1::copt_env **) const {
        released.push_back('e');
        return -1;
    }
};
}  // namespace

TEST(COPT_handle_guard, releases_partial_allocations_without_throwing) {
    using namespace copt::impl::v1;
    int storage = 0;
    for(unsigned allocated : {0u, 1u, 2u}) {
        std::vector<char> released;
        const fake_copt_release_api api{released};
        copt_env * env = nullptr;
        copt_prob * prob = nullptr;
        try {
            mippp::detail::handle_guard<fake_copt_release_api, copt_env *,
                                        copt_prob *, copt_handle_release>
                guard(api, env, prob);
            if(allocated >= 1) env = reinterpret_cast<copt_env *>(&storage);
            if(allocated >= 2) prob = reinterpret_cast<copt_prob *>(&storage);
            throw std::runtime_error("construction failure");
        } catch(const std::runtime_error & e) {
            EXPECT_STREQ(e.what(), "construction failure");
        }
        EXPECT_EQ(env, nullptr);
        EXPECT_EQ(prob, nullptr);
        const std::vector<char> expected =
            allocated == 2   ? std::vector<char>{'p', 'e'}
            : allocated == 1 ? std::vector<char>{'e'}
                             : std::vector<char>{};
        EXPECT_EQ(released, expected);
    }
}

struct copt_lp_test : public model_test<copt_api, copt_lp> {
    static void SetUpTestSuite() { construct_api("COPT"); }
};
INSTANTIATE_TEST(COPT_lp, LpModelTest, copt_lp_test);
INSTANTIATE_TEST(COPT_lp, EnumerableEntitiesTest, copt_lp_test);
INSTANTIATE_TEST(COPT_lp, ReadableObjectiveTest, copt_lp_test);
INSTANTIATE_TEST(COPT_lp, ModifiableObjectiveTest, copt_lp_test);
INSTANTIATE_TEST(COPT_lp, ReadableVariablesBoundsTest, copt_lp_test);
INSTANTIATE_TEST(COPT_lp, ModifiableVariablesBoundsTest, copt_lp_test);
INSTANTIATE_TEST(COPT_lp, NamedVariablesTest, copt_lp_test);
INSTANTIATE_TEST(COPT_lp, AddColumnTest, copt_lp_test);
INSTANTIATE_TEST(COPT_lp, DualSolutionTest, copt_lp_test);
INSTANTIATE_TEST(COPT_lp, ReducedCostsTest, copt_lp_test);
INSTANTIATE_TEST(COPT_lp, LpStatusTest, copt_lp_test);
INSTANTIATE_TEST(COPT_lp, CuttingStockTest, copt_lp_test);
INSTANTIATE_TEST(COPT_lp, TimeLimitTest, copt_lp_test);
INSTANTIATE_TEST(COPT_lp, VerbosityTest, copt_lp_test);

struct copt_milp_test : public model_test<copt_api, copt_milp> {
    static void SetUpTestSuite() { construct_api("COPT"); }
};
INSTANTIATE_TEST(COPT_milp, LpModelTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, MilpModelTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, EnumerableEntitiesTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, ReadableObjectiveTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, ModifiableObjectiveTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, ReadableVariablesBoundsTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, ModifiableVariablesBoundsTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, NamedVariablesTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, AddColumnTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, SudokuTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, CandidateSolutionCallbackTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, LazyConstraintsTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, TravellingSalesmanTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, TimeLimitTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, TimeLimitIncumbentTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, MipStartTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, OptimalityToleranceTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, MipGapTest, copt_milp_test);
// INSTANTIATE_TEST(COPT_milp, IntegralityToleranceTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, VerbosityTest, copt_milp_test);