#include "mippp/solvers/cbc/all.hpp"

#include <chrono>
#include <optional>
#include <random>
#include <ranges>
#include <string_view>
#include <vector>

using namespace mippp;

#include "test_suites/all.hpp"

MIPPP_API_VERSION_TEST(Cbc_api, cbc_api, "CBC")

struct cbc_milp_test : public model_test<cbc_api, cbc_milp> {
    static void SetUpTestSuite() { construct_api("CBC"); }
    // Measured on x0 + x1 = 1.5 over unbounded integers: branch and bound of
    // Cbc's master branch never ends, and the presolve of Cbc 2.10 answers
    // optimal with x0 = 1.5, which the deletion filter reads as feasible.
    void SetUp() override {
        model_test::SetUp();
        if(HasFatalFailure() || IsSkipped()) return;
        const std::string_view test_name =
            ::testing::UnitTest::GetInstance()->current_test_info()->name();
        if(test_name != "integers_summing_to_one_half") return;
        if(api->library_version() && api->library_version()->major < 3)
            GTEST_SKIP() << "wrong answer: Cbc 2.10 reports x0 + x1 = 1.5 "
                            "over integers optimal with x0 = 1.5";
        GTEST_SKIP() << "Cbc proves no integer infeasibility of "
                        "x0 + x1 = 1.5 over unbounded integers";
    }

    static constexpr std::string_view rows_without_terms_refused =
        "cbc_milp: this Cbc drops rows without terms";
    // The shared tests that build a row without terms skip where cbc_milp
    // refuses it, and only there.
    template <typename F>
    void SkipOnLicenseError(F && f) {
        model_test::SkipOnLicenseError([&f]() {
            try {
                f();
            } catch(const solver_error & e) {
                if(e.what() != rows_without_terms_refused) throw;
                GTEST_SKIP() << e.what();
            }
        });
    }

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
static_assert(!has_iis<cbc_milp>);

// Cbc_solve stops before branch and bound, as for an LP
TEST_F(cbc_milp_test, mip_with_an_infeasible_relaxation_is_infeasible) {
    using namespace operators;
    auto model = new_model();
    auto x = model.add_integer_variable({.lower_bound = 0., .upper_bound = 1.});
    model.add_constraint(x >= 2.);
    model.solve();
    EXPECT_TRUE(is<status::infeasible>(model.get_status()));
}

// Cbc branches on 2 x0 + 2 x1 == 1 until its time limit stops it, as in a full
// run.
TEST_F(cbc_milp_test, narrowing_returns_on_an_endless_integer_row) {
    check_narrowing_returns_on_an_endless_integer_row(new_model());
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

TEST_F(cbc_milp_test, row_without_terms_is_kept_or_refused) {
    using namespace operators;
    auto model = new_model();
    auto x = model.add_variable({.lower_bound = 0., .upper_bound = 1.});
    constexpr auto & no_terms =
        empty_linear_expression<model_variable_t<cbc_milp>, double>;
    try {
        auto r0 = model.add_constraint(no_terms >= 1.);
        EXPECT_EQ(model.num_constraints(), 1u);
        EXPECT_EQ(model.get_constraint_lower_bound(r0), 1.);
        model.set_constraint_upper_bound(r0, 3.);
        EXPECT_EQ(model.get_constraint_upper_bound(r0), 3.);
    } catch(const solver_error & e) {
        EXPECT_EQ(e.what(), rows_without_terms_refused);
        EXPECT_EQ(model.num_constraints(), 0u);
    }
    auto r1 = model.add_constraint(x <= 0.5);
    EXPECT_EQ(model.get_constraint_upper_bound(r1), 0.5);
    model.set_objective(x);
    model.set_maximization();
    model.set_constraint_upper_bound(r1, 0.25);
    model.solve();
    if(model.num_constraints() == 1u) {
        ASSERT_TRUE(is<status::optimal>(model.get_status()));
        EXPECT_NEAR(model.get_solution_value(), 0.25, TEST_EPSILON);
    } else {
        EXPECT_TRUE(is<status::infeasible>(model.get_status()));
    }
}

struct cbc_time_limit_probe : cbc_milp {
    using cbc_milp::cbc_milp;
    std::vector<std::chrono::duration<double>> trial_limits;
    void solve() {
        trial_limits.push_back(get_time_limit());
        cbc_milp::solve();
    }
};
struct cbc_time_limit_test : cbc_milp_test {
    // the IIS of x + y >= 3 over [0, 1]^2 under a caller's limit and a budget
    cbc_time_limit_probe run_deletion_filter(std::chrono::seconds caller_limit,
                                             std::chrono::seconds budget) {
        using namespace operators;
        cbc_time_limit_probe model(*api);
        auto x = model.add_variable({.lower_bound = 0., .upper_bound = 1.});
        auto y = model.add_variable({.lower_bound = 0., .upper_bound = 1.});
        model.add_constraint(x + y >= 3.);
        model.set_time_limit(caller_limit);
        const auto iis =
            compute_iis_by_deletion(model, iis_limits{.time_limit = budget});
        EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
        EXPECT_EQ(model.get_time_limit(), caller_limit);
        EXPECT_FALSE(model.trial_limits.empty());
        return model;
    }
};
TEST_F(cbc_time_limit_test, deletion_filter_keeps_a_shorter_caller_limit) {
    const auto model = run_deletion_filter(std::chrono::seconds(7),
                                           std::chrono::seconds(3600));
    for(const auto limit : model.trial_limits)
        EXPECT_EQ(limit, std::chrono::seconds(7));
}
TEST_F(cbc_time_limit_test, deletion_filter_forwards_a_shorter_budget) {
    const auto model = run_deletion_filter(std::chrono::seconds(7200),
                                           std::chrono::seconds(3600));
    for(const auto limit : model.trial_limits) {
        EXPECT_LE(limit, std::chrono::seconds(3600));
        EXPECT_GT(limit, std::chrono::seconds(3500));
    }
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
INSTANTIATE_TEST(Cbc, IisByDeletionTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, RangedConstraintsTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, SudokuTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, TimeLimitTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, TimeLimitIncumbentTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, OptimalityToleranceTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, MipGapTest, cbc_milp_test);
// INSTANTIATE_TEST(Cbc, IntegralityToleranceTest, cbc_milp_test);
INSTANTIATE_TEST(Cbc, VerbosityTest, cbc_milp_test);
