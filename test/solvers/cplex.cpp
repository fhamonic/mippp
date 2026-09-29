#include <array>
#include <chrono>
#include <cstddef>
#include <limits>
#include <numeric>
#include <optional>
#include <ranges>
#include <stdexcept>
#include <string>
#include <utility>
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
INSTANTIATE_TEST(CPLEX_lp, EnumerableEntitiesTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, ReadableObjectiveTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, ModifiableObjectiveTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, ReadableVariablesBoundsTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, ModifiableVariablesBoundsTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, NamedVariablesTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, AddColumnTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, RemoveVariableTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, ReadableConstraintsTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, ReadableConstraintBoundsTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, IisTest, cplex_lp_test);
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
INSTANTIATE_TEST(CPLEX_milp, EnumerableEntitiesTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, ReadableObjectiveTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, ModifiableObjectiveTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, ReadableVariablesBoundsTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, ModifiableVariablesBoundsTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, NamedVariablesTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, AddColumnTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, RemoveVariableTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, ReadableConstraintsTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, ReadableConstraintBoundsTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, IisTest, cplex_milp_test);
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

static_assert(has_iis<cplex_lp>);
static_assert(has_iis<cplex_milp>);

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// IIS /////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

namespace {
// A conflict that needs a solve to be found: a stop before the refiner's
// first iteration leaves every group excluded, which is no answer.
template <typename Model>
auto add_row_conflict(Model & model) {
    using namespace operators;
    auto x = model.add_variable();
    auto y = model.add_variable();
    return std::array{
        model.add_constraint(x + y >= 3.), model.add_constraint(x <= 1.),
        model.add_constraint(y <= 1.), model.add_constraint(x - y <= 10.)};
}

template <typename Model>
void check_compute_iis_zero_budget_is_a_time_limit_stop(Model model) {
    add_row_conflict(model);
    model.set_time_limit(std::chrono::seconds(0));
    const auto iis = model.compute_iis();
    EXPECT_EQ(iis.get_outcome(), iis_outcome::undetermined);
    EXPECT_EQ(iis.get_reason(), iis_reason::time_limit);
    EXPECT_EQ(iis.num_variable_members(), 0u);
    EXPECT_EQ(iis.num_constraint_members(), 0u);
    EXPECT_EQ(model.get_time_limit().count(), 0.);
    EXPECT_TRUE(is<status::unknown>(model.get_status()));
}

// The refiner resumes a stopped refinement on an unchanged problem and, when
// the stop came before its first iteration, ends "feasible" (measured on
// 22.1.1 and 22.1.2): the per-call preference makes the next call refine
// afresh.
template <typename Model>
void check_compute_iis_after_a_stop_refines_afresh(Model model) {
    const auto rows = add_row_conflict(model);
    model.set_time_limit(std::chrono::seconds(0));
    ASSERT_EQ(model.compute_iis().get_outcome(), iis_outcome::undetermined);
    model.set_time_limit(std::chrono::hours(1));
    const auto iis = model.compute_iis();
    EXPECT_EQ(iis.get_outcome(), iis_outcome::irreducible);
    EXPECT_EQ(iis.get_reason(), std::nullopt);
    EXPECT_TRUE(is<iis_status::member_lower>(iis.get_status(rows[0])));
    EXPECT_TRUE(is<iis_status::member_upper>(iis.get_status(rows[1])));
    EXPECT_TRUE(is<iis_status::member_upper>(iis.get_status(rows[2])));
    EXPECT_TRUE(is<iis_status::absent>(iis.get_status(rows[3])));
    EXPECT_EQ(iis.num_constraint_members(), 3u);
    EXPECT_EQ(iis.num_variable_members(), 0u);
    // and a third call reads the same answer
    EXPECT_EQ(model.compute_iis().num_constraint_members(), 3u);
}

// The stopped refinement lives in the problem, which moves with the model:
// a moved model that restarted its preferences would resume it.
template <typename Model>
void check_compute_iis_after_a_stop_refines_afresh_once_moved(Model model) {
    const auto rows = add_row_conflict(model);
    model.set_time_limit(std::chrono::seconds(0));
    ASSERT_EQ(model.compute_iis().get_outcome(), iis_outcome::undetermined);
    Model moved = std::move(model);
    moved.set_time_limit(std::chrono::hours(1));
    const auto iis = moved.compute_iis();
    EXPECT_EQ(iis.get_outcome(), iis_outcome::irreducible);
    EXPECT_EQ(iis.get_reason(), std::nullopt);
    EXPECT_TRUE(is<iis_status::member_lower>(iis.get_status(rows[0])));
    EXPECT_TRUE(is<iis_status::member_upper>(iis.get_status(rows[1])));
    EXPECT_TRUE(is<iis_status::member_upper>(iis.get_status(rows[2])));
    EXPECT_TRUE(is<iis_status::absent>(iis.get_status(rows[3])));
    EXPECT_EQ(iis.num_constraint_members(), 3u);
    EXPECT_EQ(iis.num_variable_members(), 0u);
}

// The groups are arguments of the native call, and nothing is set on the
// environment or the problem for it: the parameters the refiner reads and the
// problem's sizes are what they were.
template <typename Model>
void check_compute_iis_sets_nothing(Model model) {
    const auto & api = model.native_api();
    const auto [env, lp] = model.native_model();
    const auto read = [&api, env, lp] {
        std::array<double, 5> values{};
        api._check(env,
                   api.getdblparam(env, cplex::impl::v1::CPXPARAM_TimeLimit,
                                   &values[0]));
        int algorithm = 0, display = 0;
        api._check(env, api.getintparam(
                            env, cplex::impl::v1::CPXPARAM_Conflict_Algorithm,
                            &algorithm));
        api._check(env, api.getintparam(
                            env, cplex::impl::v1::CPXPARAM_Conflict_Display,
                            &display));
        values[1] = algorithm;
        values[2] = display;
        values[3] = api.getnumcols(env, lp);
        values[4] = api.getnumrows(env, lp);
        return values;
    };
    add_row_conflict(model);
    model.set_time_limit(std::chrono::seconds(3));
    const auto before = read();
    const auto iis = model.compute_iis();
    EXPECT_EQ(iis.get_outcome(), iis_outcome::irreducible);
    EXPECT_EQ(read(), before);
    EXPECT_EQ(model.get_time_limit().count(), 3.);
}
}  // namespace

TEST_F(cplex_lp_test, compute_iis_zero_budget_is_a_time_limit_stop) {
    check_compute_iis_zero_budget_is_a_time_limit_stop(new_model());
}
TEST_F(cplex_milp_test, compute_iis_zero_budget_is_a_time_limit_stop) {
    check_compute_iis_zero_budget_is_a_time_limit_stop(new_model());
}
TEST_F(cplex_lp_test, compute_iis_after_a_stop_refines_afresh) {
    check_compute_iis_after_a_stop_refines_afresh(new_model());
}
TEST_F(cplex_milp_test, compute_iis_after_a_stop_refines_afresh) {
    check_compute_iis_after_a_stop_refines_afresh(new_model());
}
TEST_F(cplex_lp_test, compute_iis_after_a_stop_refines_afresh_once_moved) {
    check_compute_iis_after_a_stop_refines_afresh_once_moved(new_model());
}
TEST_F(cplex_milp_test, compute_iis_after_a_stop_refines_afresh_once_moved) {
    check_compute_iis_after_a_stop_refines_afresh_once_moved(new_model());
}
TEST_F(cplex_lp_test, compute_iis_sets_nothing_on_the_problem) {
    check_compute_iis_sets_nothing(new_model());
}
TEST_F(cplex_milp_test, compute_iis_sets_nothing_on_the_problem) {
    check_compute_iis_sets_nothing(new_model());
}

// The refiner runs under the simplex iteration limit too, and a stop by it
// is no answer, with no reason since only the time limit has one. Unlike a
// time-limit stop, CPLEX keeps this one on the unchanged problem, the next
// calls ending on a contradiction whatever their preference, until a bound
// or a side is written, even to its current value (measured on 22.1.1 and
// 22.1.2).
TEST_F(cplex_lp_test, iteration_limit_stops_compute_iis_without_an_answer) {
    using namespace operators;
    auto model = new_model();
    auto x = model.add_variable();
    auto y = model.add_variable();
    model.add_constraint(x + y >= 3.);
    model.add_constraint(x <= 1.);
    model.add_constraint(y <= 1.);
    model.add_constraint(x - y <= 10.);
    model.set_iteration_limit(0);
    const auto iis = model.compute_iis();
    EXPECT_EQ(iis.get_outcome(), iis_outcome::undetermined);
    EXPECT_EQ(iis.get_reason(), std::nullopt);
    EXPECT_EQ(iis.num_constraint_members(), 0u);
    EXPECT_EQ(model.get_iteration_limit(), 0u);
    model.set_iteration_limit(std::numeric_limits<std::size_t>::max());
    EXPECT_EQ(model.compute_iis().get_outcome(), iis_outcome::undetermined);
    model.set_variable_upper_bound(x, model.get_variable_upper_bound(x));
    const auto afterwards = model.compute_iis();
    EXPECT_EQ(afterwards.get_outcome(), iis_outcome::irreducible);
    EXPECT_EQ(afterwards.num_constraint_members(), 3u);
}

// The refiner refuses to run while a generic callback is registered, on a
// feasible model too: the callback is detached for the call and back for the
// next solve.
TEST_F(cplex_milp_test, registered_callback_does_not_run_during_compute_iis) {
    using namespace operators;
    auto model = new_model();
    auto x = model.add_integer_variable({.lower_bound = 0., .upper_bound = 5.});
    auto y = model.add_integer_variable({.lower_bound = 0., .upper_bound = 5.});
    auto r = model.add_constraint(x + y <= 8.);
    model.set_maximization();
    model.set_objective(x + y);
    int fired = 0;
    model.set_candidate_solution_callback([&](auto & handle) {
        ++fired;
        handle.add_lazy_constraint(x + y <= -1.);
    });
    const auto iis = model.compute_iis();
    EXPECT_EQ(iis.get_outcome(), iis_outcome::feasible);
    EXPECT_EQ(fired, 0);
    EXPECT_TRUE(is<iis_status::absent>(iis.get_status(x)));
    EXPECT_TRUE(is<iis_status::absent>(iis.get_status(r)));
    model.solve();
    EXPECT_GE(fired, 1);
    EXPECT_TRUE(is_a<status::infeasible>(model.get_status()));
}

// The refiner is told nothing of the indicators, so they stay in every
// subproblem: the members are an IIS relative to them, and a conflict that
// lives in the indicators alone has no member.
TEST_F(cplex_milp_test, indicator_constraints_are_background) {
    using namespace operators;
    {
        auto model = new_model();
        auto z = model.add_binary_variable();
        auto x = model.add_variable({.lower_bound = 0., .upper_bound = 1.});
        auto y = model.add_variable({.lower_bound = 0., .upper_bound = 10.});
        auto r0 = model.add_constraint(x >= 2.);
        model.add_indicator_constraint(z, true, x + y <= 5.);
        const auto iis = model.compute_iis();
        EXPECT_EQ(iis.get_outcome(), iis_outcome::irreducible);
        EXPECT_TRUE(is<iis_status::absent>(iis.get_status(z)));
        EXPECT_TRUE(is<iis_status::member_upper>(iis.get_status(x)));
        EXPECT_TRUE(is<iis_status::absent>(iis.get_status(y)));
        EXPECT_TRUE(is<iis_status::member_lower>(iis.get_status(r0)));
        EXPECT_EQ(iis.num_variable_members(), 1u);
        EXPECT_EQ(iis.num_constraint_members(), 1u);
    }
    {
        auto model = new_model();
        auto z = model.add_binary_variable();
        auto x = model.add_variable();
        model.add_indicator_constraint(z, true, x >= 5.);
        model.add_indicator_constraint(z, false, x >= 5.);
        model.add_indicator_constraint(z, true, x <= 1.);
        model.add_indicator_constraint(z, false, x <= 1.);
        const auto iis = model.compute_iis();
        EXPECT_EQ(iis.get_outcome(), iis_outcome::irreducible);
        EXPECT_EQ(iis.num_variable_members(), 0u);
        EXPECT_EQ(iis.num_constraint_members(), 0u);
        model.solve();
        EXPECT_TRUE(is_a<status::infeasible>(model.get_status()));
    }
}

// Cornuejols-Dawande market split without slacks, 3 dense equality rows over
// 20 binaries, each row summing to half its coefficients rounded down. The
// rows are spelled out rather than drawn, since the outcome below is a
// property of this instance: under a node limit of 0 the refiner excludes
// groups whose subproblem only hit the limit, and the rows it leaves flagged
// solve to an optimum (measured on 22.1.1 and 22.1.2), so the flags of that
// stop are no answer.
TEST_F(cplex_milp_test, node_limit_stops_compute_iis_without_an_answer) {
    using namespace operators;
    auto model = new_model();
    constexpr std::size_t n = 20;
    constexpr std::array<std::array<int, n>, 3> rows{
        {{41, 99, 72, 93, 0,  12, 30, 99, 14, 23,
          9,  39, 18, 38, 34, 66, 39, 93, 53, 84},
         {41, 31, 68, 52, 20, 44, 87, 22, 2,  53,
          67, 91, 41, 45, 55, 43, 14, 93, 19, 77},
         {80, 71, 96, 80, 31, 9,  69, 51, 87, 86,
          89, 82, 8,  82, 3,  27, 16, 5,  87, 67}}};
    auto x = model.add_binary_variables(n);
    for(const auto & row : rows) {
        const int total = std::accumulate(row.begin(), row.end(), 0);
        model.add_constraint(xsum(std::views::iota(std::size_t{0}, n),
                                  [&](auto j) { return row[j] * x(j); }) ==
                             total / 2);
    }
    model.solve();
    ASSERT_TRUE(is_a<status::infeasible>(model.get_status()));
    model.set_node_limit(0);
    const auto iis = model.compute_iis();
    EXPECT_EQ(iis.get_outcome(), iis_outcome::undetermined);
    EXPECT_EQ(iis.get_reason(), std::nullopt);
    EXPECT_EQ(iis.num_variable_members(), 0u);
    EXPECT_EQ(iis.num_constraint_members(), 0u);
    EXPECT_EQ(model.get_node_limit(), 0u);
    EXPECT_TRUE(is<status::unknown>(model.get_status()));
}
