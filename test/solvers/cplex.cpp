#include <algorithm>
#include <array>
#include <chrono>
#include <concepts>
#include <cstddef>
#include <limits>
#include <numeric>
#include <optional>
#include <ranges>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
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

// CPLEX stores a ranged row as a right-hand side and a range width, so a row
// whose sides cross has no representation and the setters reject it.
struct cplex_lp_test : public model_test<cplex_api, cplex_lp> {
    static void SetUpTestSuite() { construct_api("CPLEX"); }
    static std::optional<std::string> iis_case_skip_reason(
        const iis_cases::iis_case & c) {
        return iis_cases::crossed_row_skip_reason(c, "CPLEX");
    }
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
INSTANTIATE_TEST(CPLEX_lp, ModifiableConstraintBoundsTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, IisTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, IisByDeletionTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, DualSolutionTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, ReducedCostsTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, LpStatusTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, CuttingStockTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, ColumnManagerTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, LpFuzzyTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, TimeLimitTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, IterationLimitTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, LpBasisTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, LpBasisWarmStartTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, OptimalityToleranceTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, VerbosityTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, ThreadLimitTest, cplex_lp_test);
INSTANTIATE_TEST(CPLEX_lp, LpAlgorithmTest, cplex_lp_test);

struct cplex_milp_test : public model_test<cplex_api, cplex_milp> {
    static void SetUpTestSuite() { construct_api("CPLEX"); }
    static std::optional<std::string> iis_case_skip_reason(
        const iis_cases::iis_case & c) {
        return iis_cases::crossed_row_skip_reason(c, "CPLEX");
    }
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
INSTANTIATE_TEST(CPLEX_milp, ModifiableConstraintBoundsTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, IisTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, IisByDeletionTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, SudokuTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, CandidateSolutionCallbackTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, LazyConstraintsTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, SolutionInjectionTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, NodeRelaxationCallbackTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, UserCutsTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, TravellingSalesmanTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, TimeLimitTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, TimeLimitIncumbentTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, MipStartTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, OptimalityToleranceTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, MipGapTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, DualBoundTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, NodeCountTest, cplex_milp_test);
// INSTANTIATE_TEST(CPLEX_milp, IntegralityToleranceTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, VerbosityTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, ThreadLimitTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, ProgressCallbackTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, ProgressCallbackNodeCountTest, cplex_milp_test);
INSTANTIATE_TEST(CPLEX_milp, ProgressCallbackSharingTest, cplex_milp_test);

static_assert(has_iis<cplex_lp>);
static_assert(has_iis<cplex_milp>);
static_assert(
    std::same_as<
        native_iis_outcome_t<cplex_lp>,
        std::variant<iis_outcome::incomplete, iis_outcome::irreducible,
                     iis_outcome::feasible, iis_outcome::interrupted,
                     iis_outcome::limit_reached, iis_outcome::time_limit,
                     iis_outcome::iteration_limit, iis_outcome::node_limit,
                     iis_outcome::memory_limit>>);
static_assert(std::same_as<native_iis_outcome_t<cplex_milp>,
                           native_iis_outcome_t<cplex_lp>>);
static_assert(iis_by_deletion_model<cplex_lp>);
static_assert(iis_by_deletion_model<cplex_milp>);
// CPLEX ranges a row through its row-bound setters, so the filter writes the
// sides, not the sense and the rhs
static_assert(!detail::iis_rows_as_sense_and_rhs<cplex_lp>);
static_assert(!detail::iis_rows_as_sense_and_rhs<cplex_milp>);
static_assert(has_lp_basis_warm_start<cplex_lp>);
static_assert(!has_lp_basis<cplex_milp>);

// The cause of each conflict-abort status, pinned without a licence since no
// test run reaches 36 to 38.
struct cplex_iis_mapping : cplex_lp {
    using cplex_base::_conflict_abort_outcome;
};
TEST(cplex_conflict_abort_outcome, names_the_cause_of_each_abort) {
    using namespace cplex::impl::v1;
    const auto map = &cplex_iis_mapping::_conflict_abort_outcome;
    EXPECT_TRUE(outcome_is<iis_outcome::incomplete>(
        map(CPX_STAT_CONFLICT_ABORT_CONTRADICTION), false));
    EXPECT_TRUE(outcome_is<iis_outcome::time_limit>(
        map(CPX_STAT_CONFLICT_ABORT_TIME_LIM), false));
    EXPECT_TRUE(outcome_is<iis_outcome::iteration_limit>(
        map(CPX_STAT_CONFLICT_ABORT_IT_LIM), false));
    EXPECT_TRUE(outcome_is<iis_outcome::node_limit>(
        map(CPX_STAT_CONFLICT_ABORT_NODE_LIM), false));
    EXPECT_TRUE(outcome_is<iis_outcome::limit_reached>(
        map(CPX_STAT_CONFLICT_ABORT_OBJ_LIM), false));
    EXPECT_TRUE(outcome_is<iis_outcome::memory_limit>(
        map(CPX_STAT_CONFLICT_ABORT_MEM_LIM), false));
    EXPECT_TRUE(outcome_is<iis_outcome::interrupted>(
        map(CPX_STAT_CONFLICT_ABORT_USER), false));
    EXPECT_TRUE(outcome_is<iis_outcome::limit_reached>(
        map(CPX_STAT_CONFLICT_ABORT_DETTIME_LIM), false));
}

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Row bounds /////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// A <= row given a finite lower side becomes a native 'R' row, whose width is
// the range value: its sides still read back, its sense and rhs no longer
// exist.
TEST_F(cplex_lp_test, ranged_row_has_no_sense_or_rhs) {
    using namespace operators;
    auto model = new_model();
    auto x = model.add_variable();
    auto c = model.add_constraint(x <= 3.);
    model.set_constraint_lower_bound(c, 1.);
    EXPECT_EQ(model.get_constraint_lower_bound(c), 1.);
    EXPECT_EQ(model.get_constraint_upper_bound(c), 3.);
    EXPECT_THROW((void)model.get_constraint_sense(c), std::runtime_error);
    EXPECT_THROW((void)model.get_constraint_rhs(c), std::runtime_error);
    EXPECT_THROW((void)model.get_constraint(c), std::runtime_error);
    EXPECT_THROW(model.set_constraint_rhs(c, 2.), std::runtime_error);
    EXPECT_THROW(model.set_constraint_sense(c, constraint_sense::less_equal),
                 std::runtime_error);
    const auto [env, lp] = model.native_model();
    char sense = '?';
    double range = 0.;
    model.native_api()._check(
        env, model.native_api().getsense(env, lp, &sense, model.native_id(c),
                                         model.native_id(c)));
    model.native_api()._check(
        env, model.native_api().getrngval(env, lp, &range, model.native_id(c),
                                          model.native_id(c)));
    EXPECT_EQ(sense, 'R');
    EXPECT_EQ(range, 2.);
    model.set_constraint_lower_bound(c, -model.infinity());
    EXPECT_EQ(model.get_constraint_sense(c), constraint_sense::less_equal);
    EXPECT_EQ(model.get_constraint_rhs(c), 3.);
}

// The width of an 'R' row is |rngval| and its sign only picks the side of the
// rhs, so sides that cross cannot be stored; written as they are, CPLEX would
// solve the swapped range.
TEST_F(cplex_lp_test, crossed_row_sides_are_rejected) {
    using namespace operators;
    auto model = new_model();
    auto x = model.add_variable({.lower_bound = 0., .upper_bound = 5.});
    auto c = model.add_constraint(x <= 0.);
    EXPECT_THROW(model.set_constraint_lower_bound(c, 1.),
                 std::invalid_argument);
    EXPECT_EQ(model.get_constraint_lower_bound(c), -model.infinity());
    EXPECT_EQ(model.get_constraint_upper_bound(c), 0.);
    model.set_constraint_lower_bound(c, -1.);
    EXPECT_THROW(model.set_constraint_upper_bound(c, -2.),
                 std::invalid_argument);
    EXPECT_EQ(model.get_constraint_lower_bound(c), -1.);
    EXPECT_EQ(model.get_constraint_upper_bound(c), 0.);
    model.set_objective(x);
    model.set_maximization();
    model.solve();
    ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
    EXPECT_NEAR(model.get_solution_value(), 0., TEST_EPSILON);
}

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// LP status //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// The two rows conflict by 0.05. The feasibility tolerance is what CPLEX lets
// a row be violated by, on a MIP as on an LP: the MIP tolerance group has a
// Linearization parameter that reads like one and changes nothing here.
TEST_F(cplex_milp_test, feasibility_tolerance_bounds_row_violations) {
    using namespace operators;
    for(const double tolerance : {0.01, 0.09}) {
        auto model = new_model();
        auto x =
            model.add_integer_variable({.lower_bound = 0, .upper_bound = 10});
        auto y = model.add_variable();
        auto z = model.add_variable();
        model.set_maximization();
        model.set_objective(x);
        model.add_constraint(y + z >= 2);
        model.add_constraint(y + z <= 1.95);
        model.add_constraint(2 * x <= 14.6);
        model.set_feasibility_tolerance(tolerance);
        ASSERT_DOUBLE_EQ(model.get_feasibility_tolerance(), tolerance);
        model.solve();
        if(tolerance < 0.05) {
            ASSERT_TRUE(is_a<status::infeasible>(model.get_status()));
        } else {
            ASSERT_TRUE(is<status::optimal>(model.get_status()));
            ASSERT_NEAR(model.get_solution_value(), 7, TEST_EPSILON);
        }
    }
}

// The refinement re-solves without advanced start and presolve reductions,
// and writes both parameters back. Under the default reductions, presolve
// reports this unbounded LP as "infeasible or unbounded" (measured on 22.1.2,
// which reports a bound conflict as plain infeasible).
TEST_F(cplex_lp_test, refine_lp_status_writes_its_parameters_back) {
    using namespace operators;
    using namespace cplex::impl::v1;
    auto model = new_model();
    const auto & native = model.native_api();
    const auto [env, lp] = model.native_model();
    const auto read = [&native, env] {
        std::array<int, 2> values{};
        native._check(env,
                      native.getintparam(env, CPXPARAM_Advance, &values[0]));
        native._check(env, native.getintparam(
                               env, CPXPARAM_Preprocessing_Reduce, &values[1]));
        return values;
    };
    native._check(env, native.setintparam(env, CPXPARAM_Advance, 2));
    const auto before = read();
    auto x = model.add_variable();
    model.set_maximization();
    model.set_objective(x);
    model.add_constraint(x >= 1.);
    model.solve();
    ASSERT_TRUE(is<status::infeasible_or_unbounded>(model.get_status()));
    model.refine_lp_status();
    EXPECT_TRUE(is<status::unbounded>(model.get_status()));
    EXPECT_EQ(read(), before);
    EXPECT_EQ(before[0], 2);
}

// A solve that throws reports unknown, not the status of the solve before it:
// CPXlpopt refuses a problem turned MIP through the native handle.
TEST_F(cplex_lp_test, a_throwing_solve_reports_unknown) {
    using namespace operators;
    using namespace cplex::impl::v1;
    auto model = new_model();
    auto x = model.add_variable({.upper_bound = 1.});
    model.set_maximization();
    model.set_objective(x);
    model.solve();
    ASSERT_TRUE(is<status::optimal>(model.get_status()));
    const auto & native = model.native_api();
    const auto [env, lp] = model.native_model();
    native._check(env, native.chgprobtype(env, lp, CPXPROB_MILP));
    EXPECT_ANY_THROW(model.solve());
    EXPECT_TRUE(is<status::unknown>(model.get_status()));
}

// The refinement's CPXprimopt refuses a problem turned MIP through the native
// handle (error 1017, measured on 22.1.2): both parameters are written back
// on that throw.
TEST_F(cplex_lp_test, refine_lp_status_writes_its_parameters_back_on_a_throw) {
    using namespace operators;
    using namespace cplex::impl::v1;
    auto model = new_model();
    const auto & native = model.native_api();
    const auto [env, lp] = model.native_model();
    const auto read = [&native, env] {
        std::array<int, 2> values{};
        native._check(env,
                      native.getintparam(env, CPXPARAM_Advance, &values[0]));
        native._check(env, native.getintparam(
                               env, CPXPARAM_Preprocessing_Reduce, &values[1]));
        return values;
    };
    native._check(env, native.setintparam(env, CPXPARAM_Advance, 2));
    const auto before = read();
    auto x = model.add_variable();
    model.set_maximization();
    model.set_objective(x);
    model.add_constraint(x >= 1.);
    model.solve();
    ASSERT_TRUE(is<status::infeasible_or_unbounded>(model.get_status()));
    native._check(env, native.chgprobtype(env, lp, CPXPROB_MILP));
    EXPECT_ANY_THROW(model.refine_lp_status());
    EXPECT_EQ(read(), before);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// IIS /////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

namespace {
// A conflict that needs a solve to be found: a stop before the refiner's
// first iteration leaves every group excluded, which is no answer.
template <typename Model>
auto add_row_conflict(Model & model) {
    return iis_cases::build(model, iis_cases::four_row_conflict_case())
        .constraints;
}

// The refiner resumes a stopped refinement on an unchanged problem and, when
// the stop came before its first iteration, ends "feasible" (measured on
// 22.1.1 and 22.1.2): the per-call preference makes the next call refine
// afresh.
template <typename Model>
void check_compute_iis_after_a_stop_refines_afresh(Model model) {
    const auto rows = add_row_conflict(model);
    model.set_time_limit(std::chrono::seconds(0));
    ASSERT_TRUE(outcome_is<iis_outcome::time_limit>(
        model.compute_iis().get_outcome(), false));
    model.set_time_limit(std::chrono::hours(1));
    const auto iis = model.compute_iis();
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
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
    ASSERT_TRUE(outcome_is<iis_outcome::time_limit>(
        model.compute_iis().get_outcome(), false));
    Model moved = std::move(model);
    moved.set_time_limit(std::chrono::hours(1));
    const auto iis = moved.compute_iis();
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
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
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
    EXPECT_EQ(read(), before);
}
}  // namespace

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
// is no answer. Unlike a time-limit stop, CPLEX keeps this one on the unchanged
// problem, the next calls ending on a contradiction whatever their preference,
// until a bound or a side is written, even to its current value (measured
// on 22.1.1 and 22.1.2).
TEST_F(cplex_lp_test, iteration_limit_stops_compute_iis_without_an_answer) {
    auto model = new_model();
    const auto x = iis_cases::build(model, iis_cases::four_row_conflict_case())
                       .variables[0];
    model.set_iteration_limit(0);
    const auto iis = model.compute_iis();
    EXPECT_TRUE(
        outcome_is<iis_outcome::iteration_limit>(iis.get_outcome(), false));
    EXPECT_EQ(iis.num_constraint_members(), 0u);
    EXPECT_EQ(model.get_iteration_limit(), 0u);
    model.set_iteration_limit(std::numeric_limits<std::size_t>::max());
    EXPECT_TRUE(outcome_is<iis_outcome::incomplete>(
        model.compute_iis().get_outcome(), false));
    model.set_variable_upper_bound(x, model.get_variable_upper_bound(x));
    const auto afterwards = model.compute_iis();
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(afterwards.get_outcome()));
    EXPECT_EQ(afterwards.num_constraint_members(), 3u);
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
    EXPECT_TRUE(outcome_is<iis_outcome::node_limit>(iis.get_outcome(), false));
    EXPECT_EQ(iis.num_variable_members(), 0u);
    EXPECT_EQ(iis.num_constraint_members(), 0u);
    EXPECT_EQ(model.get_node_limit(), 0u);
    EXPECT_TRUE(is<status::unknown>(model.get_status()));
}

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// Basis ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// LPMethod has no "either simplex", and the selector is an LP member.
static_assert(has_lp_algorithm<cplex_lp>);
static_assert(has_lp_algorithm<cplex_lp, lp_algorithm::primal_simplex>);
static_assert(has_lp_algorithm<cplex_lp, lp_algorithm::dual_simplex>);
static_assert(has_lp_algorithm<cplex_lp, lp_algorithm::interior_point>);
static_assert(
    has_lp_algorithm<cplex_lp, lp_algorithm::interior_point_without_crossover>);
static_assert(!has_lp_algorithm<cplex_lp, lp_algorithm::simplex>);
static_assert(!has_lp_algorithm<cplex_lp, lp_algorithm::other>);
static_assert(variant_with_alternative<model_lp_algorithm_t<cplex_lp>,
                                       lp_algorithm::other>);
static_assert(!has_lp_algorithm<cplex_milp>);

namespace {
struct cplex_lp_run {
    int lp_method = 0, solution_type = 0;  // the parameters
    int method = 0;                        // CPXgetmethod: what ended the solve
    int simplex_iterations = 0, barrier_iterations = 0;
    int solution_kind = 0;  // CPXsolninfo: basic or nonbasic
};
// The counters are no members of cplex_api: asking the library for them
// here keeps a release without one of them loadable.
cplex_lp_run run_of(cplex_lp & model) {
    using namespace cplex::impl::v1;
    using counter = int(CPXCENVptr, CPXCLPptr);
    const auto & native = model.native_api();
    const auto [env, lp] = model.native_model();
    const detail::dynamic_library library(native.library_path());
    cplex_lp_run run;
    native._check(env,
                  native.getintparam(env, CPXPARAM_LPMethod, &run.lp_method));
    native._check(env, native.getintparam(env, CPXPARAM_SolutionType,
                                          &run.solution_type));
    run.method = library.get_function<counter>("CPXgetmethod")(env, lp);
    run.simplex_iterations =
        library.get_function<counter>("CPXgetitcnt")(env, lp);
    run.barrier_iterations =
        library.get_function<counter>("CPXgetbaritcnt")(env, lp);
    int feasible = 0;
    native._check(env, native.solninfo(env, lp, nullptr, &run.solution_kind,
                                       &feasible, &feasible));
    return run;
}
}  // namespace

// The shared suite cannot see which algorithm ran: CPLEX's counters can.
// After a crossover CPLEX names the simplex that finished, so the barrier
// shows in its iteration count alone.
TEST_F(cplex_lp_test, each_lp_algorithm_runs_the_one_it_names) {
    using namespace cplex::impl::v1;
    SkipOnLicenseError([this]() {
        const auto solved_under = [this](auto algorithm) {
            auto model = new_model();
            lp_basis_cases::build_dense(model);
            model.set_lp_algorithm(algorithm);
            model.solve();
            EXPECT_TRUE(is_a<status::optimal>(model.get_status()));
            return run_of(model);
        };
        {
            const auto run = solved_under(lp_algorithm::primal_simplex{});
            EXPECT_EQ(run.lp_method, CPX_ALG_PRIMAL);
            EXPECT_EQ(run.method, CPX_ALG_PRIMAL);
            EXPECT_GT(run.simplex_iterations, 0);
            EXPECT_EQ(run.barrier_iterations, 0);
        }
        {
            const auto run = solved_under(lp_algorithm::dual_simplex{});
            EXPECT_EQ(run.lp_method, CPX_ALG_DUAL);
            EXPECT_EQ(run.method, CPX_ALG_DUAL);
            EXPECT_GT(run.simplex_iterations, 0);
            EXPECT_EQ(run.barrier_iterations, 0);
        }
        {
            const auto run = solved_under(lp_algorithm::interior_point{});
            EXPECT_EQ(run.lp_method, CPX_ALG_BARRIER);
            EXPECT_EQ(run.solution_type, CPX_AUTO_SOLN);
            EXPECT_GT(run.barrier_iterations, 0);
            EXPECT_EQ(run.solution_kind, CPX_BASIC_SOLN);
        }
        {
            const auto run =
                solved_under(lp_algorithm::interior_point_without_crossover{});
            EXPECT_EQ(run.lp_method, CPX_ALG_BARRIER);
            EXPECT_EQ(run.solution_type, CPX_NONBASIC_SOLN);
            EXPECT_EQ(run.method, CPX_ALG_BARRIER);
            EXPECT_GT(run.barrier_iterations, 0);
            EXPECT_EQ(run.solution_kind, CPX_NONBASIC_SOLN);
        }
        {
            const auto run = solved_under(lp_algorithm::automatic{});
            EXPECT_EQ(run.lp_method, CPX_ALG_AUTOMATIC);
            EXPECT_EQ(run.solution_type, CPX_AUTO_SOLN);
        }
    });
}

// solve() runs CPXlpopt, which reads LPMethod: a value written through the
// native handle is the algorithm that runs, and reads back as its tag, or as
// other when no tag writes it.
TEST_F(cplex_lp_test, a_native_lp_method_is_honoured) {
    using namespace cplex::impl::v1;
    SkipOnLicenseError([this]() {
        auto model = new_model();
        lp_basis_cases::build_dense(model);
        const auto & native = model.native_api();
        const auto [env, lp] = model.native_model();
        native._check(
            env, native.setintparam(env, CPXPARAM_LPMethod, CPX_ALG_BARRIER));
        EXPECT_TRUE(is<lp_algorithm::interior_point>(model.get_lp_algorithm()));
        model.solve();
        ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
        EXPECT_GT(run_of(model).barrier_iterations, 0);
        native._check(env, native.setintparam(env, CPXPARAM_LPMethod, 6));
        EXPECT_TRUE(is<lp_algorithm::other>(model.get_lp_algorithm()));
        native._check(env,
                      native.setintparam(env, CPXPARAM_LPMethod, CPX_ALG_DUAL));
        native._check(env, native.setintparam(env, CPXPARAM_SolutionType,
                                              CPX_BASIC_SOLN));
        EXPECT_TRUE(is<lp_algorithm::other>(model.get_lp_algorithm()));
    });
}

TEST_F(cplex_lp_test, get_basis_throws_after_presolve_proves_infeasibility) {
    auto model = new_model();
    lp_basis_cases::build_infeasible(model);
    model.solve();
    ASSERT_TRUE(is_a<status::infeasible_or_unbounded>(model.get_status()));
    ASSERT_THROW(model.get_basis(), solver_error);
}
// A row of CPLEX is never at no bound: is_a<superbasic> compiles on a
// column status only.
static_assert(variant_containing_a<model_variable_basis_status_t<cplex_lp>,
                                   basis_status::superbasic>);
static_assert(!variant_containing_a<model_constraint_basis_status_t<cplex_lp>,
                                    basis_status::superbasic>);
// CPXgetbase returns what CPXcopybase stored until the next solve, which
// checks the codes set_basis() writes without the solve get_basis() needs.
TEST_F(cplex_lp_test, a_written_basis_reaches_cplex_before_a_solve) {
    using namespace operators;
    using namespace cplex::impl::v1;
    auto model = new_model();
    const auto x = model.add_variable({.lower_bound = 0., .upper_bound = 4.});
    const auto y = model.add_variable({.lower_bound = 0., .upper_bound = 4.});
    const auto z = model.add_variable({.lower_bound = 0., .upper_bound = 4.});
    const auto f = model.add_variable({.lower_bound = std::nullopt});
    const auto w = model.add_variable({.lower_bound = 0., .upper_bound = 4.});
    const auto le = model.add_constraint(x + y <= 4.);
    const auto ge = model.add_constraint(x + z >= 1.);
    const auto eq = model.add_constraint(y + f == 2.);
    const auto lower = model.add_constraint(y + z >= 1.);
    model.set_constraint_upper_bound(lower, 5.);
    const auto upper = model.add_constraint(x + f >= 1.);
    model.set_constraint_upper_bound(upper, 5.);
    const auto loose = model.add_constraint(x + y + z <= 100.);
    model_basis_t<cplex_lp> written;
    written.set_status(x, basis_status::basic{});
    written.set_status(y, basis_status::nonbasic_at_lower_bound{});
    written.set_status(z, basis_status::nonbasic_at_upper_bound{});
    written.set_status(f, basis_status::superbasic{});
    written.set_status(w, basis_status::nonbasic{});
    written.set_status(le, basis_status::nonbasic_at_upper_bound{});
    written.set_status(ge, basis_status::nonbasic_at_lower_bound{});
    written.set_status(eq, basis_status::nonbasic_fixed{});
    written.set_status(lower, basis_status::nonbasic_at_lower_bound{});
    written.set_status(upper, basis_status::nonbasic_at_upper_bound{});
    written.set_status(loose, basis_status::basic{});
    model.set_basis(written);
    const auto & native = model.native_api();
    const auto [env, lp] = model.native_model();
    std::array<int, 5> cstat{};
    std::array<int, 6> rstat{};
    native._check(env, native.getbase(env, lp, cstat.data(), rstat.data()));
    EXPECT_EQ(cstat, (std::array<int, 5>{CPX_BASIC, CPX_AT_LOWER, CPX_AT_UPPER,
                                         CPX_FREE_SUPER, CPX_AT_LOWER}));
    EXPECT_EQ(rstat,
              (std::array<int, 6>{CPX_AT_LOWER, CPX_AT_LOWER, CPX_AT_LOWER,
                                  CPX_AT_LOWER, CPX_AT_UPPER, CPX_BASIC}));
}

// CPXcopybase refuses a null array, which an empty std::vector may hold.
TEST_F(cplex_lp_test, set_basis_writes_an_empty_dimension) {
    using namespace operators;
    auto no_rows = new_model();
    const auto x = no_rows.add_variable({.lower_bound = 0., .upper_bound = 2.});
    no_rows.set_maximization();
    no_rows.set_objective(x);
    model_basis_t<cplex_lp> basis;
    basis.set_status(x, basis_status::nonbasic_at_upper_bound{});
    no_rows.set_basis(basis);
    no_rows.solve();
    ASSERT_TRUE(is<status::optimal>(no_rows.get_status()));
    EXPECT_NEAR(no_rows.get_solution_value(), 2., TEST_EPSILON);
    EXPECT_TRUE(is<basis_status::nonbasic_at_upper_bound>(
        no_rows.get_basis().get_status(x)));

    auto no_columns = new_model();
    constexpr auto & no_terms =
        empty_linear_expression<model_variable_t<cplex_lp>, double>;
    no_columns.add_constraint(no_terms <= 1.);
    no_columns.set_basis(model_basis_t<cplex_lp>{});
    no_columns.solve();
    EXPECT_TRUE(is<status::optimal>(no_columns.get_status()));
}
// The code set_basis writes for each tag the statuses list, pinned without a
// library: a tag that loses its line in the table writes as its nearest
// base, which still compiles.
struct cplex_basis_codes : cplex_lp {
    using column_status = model_variable_basis_status_t<cplex_lp>;
    using row_status = model_constraint_basis_status_t<cplex_lp>;
    template <typename Tag>
    static constexpr auto column() {
        return _column_basis_code(column_status(std::in_place_type<Tag>));
    }
    template <typename Tag>
    static constexpr auto row(char sense) {
        return _row_basis_code(row_status(std::in_place_type<Tag>), sense);
    }
};
TEST(cplex_lp_basis_codes, one_per_listed_tag) {
    using namespace cplex::impl::v1;
    using namespace basis_status;
    using codes = cplex_basis_codes;
    // clang-format off
    static_assert(std::variant_size_v<codes::column_status> == 5);
    static_assert(std::variant_size_v<codes::row_status>    == 5);
    static_assert(codes::column<basic>()                   == CPX_BASIC);
    static_assert(codes::column<nonbasic_at_lower_bound>() == CPX_AT_LOWER);
    static_assert(codes::column<nonbasic_at_upper_bound>() == CPX_AT_UPPER);
    static_assert(codes::column<superbasic>()              == CPX_FREE_SUPER);
    static_assert(codes::column<nonbasic>()                == CPX_AT_LOWER);
    // only the slack of an 'R' row has two distinct bounds
    static_assert(codes::row<basic>('R')                   == CPX_BASIC);
    static_assert(codes::row<nonbasic_at_lower_bound>('R') == CPX_AT_LOWER);
    static_assert(codes::row<nonbasic_at_upper_bound>('R') == CPX_AT_UPPER);
    static_assert(codes::row<nonbasic_fixed>('R')          == CPX_AT_LOWER);
    static_assert(codes::row<nonbasic>('R')                == CPX_AT_LOWER);
    static_assert(std::ranges::all_of(std::string_view("LGE"), [](char sense) {
        return codes::row<basic>(sense)                   == CPX_BASIC
            && codes::row<nonbasic_at_lower_bound>(sense) == CPX_AT_LOWER
            && codes::row<nonbasic_at_upper_bound>(sense) == CPX_AT_LOWER
            && codes::row<nonbasic_fixed>(sense)          == CPX_AT_LOWER
            && codes::row<nonbasic>(sense)                == CPX_AT_LOWER;
    }));
    // clang-format on
}
