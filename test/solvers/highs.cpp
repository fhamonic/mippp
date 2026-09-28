#include <gmock/gmock.h>

#include <chrono>
#include <stdexcept>
#include <string>

#include "mippp/solvers/highs/all.hpp"

using namespace mippp;

#include "test_suites/all.hpp"

MIPPP_API_VERSION_TEST(HiGHS_api, highs_api, "HIGHS")

struct highs_lp_test : public model_test<highs_api, highs_lp> {
    static void SetUpTestSuite() { construct_api("HIGHS"); }
};
INSTANTIATE_TEST(HiGHS_lp, LpModelTest, highs_lp_test);
INSTANTIATE_TEST(HiGHS_lp, EnumerableEntitiesTest, highs_lp_test);
INSTANTIATE_TEST(HiGHS_lp, ReadableObjectiveTest, highs_lp_test);
INSTANTIATE_TEST(HiGHS_lp, ModifiableObjectiveTest, highs_lp_test);
INSTANTIATE_TEST(HiGHS_lp, ReadableVariablesBoundsTest, highs_lp_test);
INSTANTIATE_TEST(HiGHS_lp, ModifiableVariablesBoundsTest, highs_lp_test);
INSTANTIATE_TEST(HiGHS_lp, NamedVariablesTest, highs_lp_test);
INSTANTIATE_TEST(HiGHS_lp, NamedConstraintsTest, highs_lp_test);
INSTANTIATE_TEST(HiGHS_lp, AddColumnTest, highs_lp_test);
INSTANTIATE_TEST(HiGHS_lp, RemoveVariableTest, highs_lp_test);
INSTANTIATE_TEST(HiGHS_lp, ReadableConstraintsTest, highs_lp_test);
INSTANTIATE_TEST(HiGHS_lp, ReadableConstraintBoundsTest, highs_lp_test);
INSTANTIATE_TEST(HiGHS_lp, ModifiableConstraintBoundsTest, highs_lp_test);
INSTANTIATE_TEST(HiGHS_lp, IisByDeletionTest, highs_lp_test);
INSTANTIATE_TEST(HiGHS_lp, DualSolutionTest, highs_lp_test);
INSTANTIATE_TEST(HiGHS_lp, ReducedCostsTest, highs_lp_test);
INSTANTIATE_TEST(HiGHS_lp, LpStatusTest, highs_lp_test);
INSTANTIATE_TEST(HiGHS_lp, CuttingStockTest, highs_lp_test);
INSTANTIATE_TEST(HiGHS_lp, ColumnManagerTest, highs_lp_test);
INSTANTIATE_TEST(HiGHS_lp, LpFuzzyTest, highs_lp_test);
INSTANTIATE_TEST(HiGHS_lp, TimeLimitTest, highs_lp_test);
INSTANTIATE_TEST(HiGHS_lp, IterationLimitTest, highs_lp_test);
INSTANTIATE_TEST(HiGHS_lp, VerbosityTest, highs_lp_test);

// A <= row given a finite lower side is ranged: its sides still read back,
// its sense and rhs no longer exist.
TEST_F(highs_lp_test, ranged_row_has_no_sense_or_rhs) {
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
    model.set_constraint_lower_bound(c, -model.infinity());
    EXPECT_EQ(model.get_constraint_sense(c), constraint_sense::less_equal);
    EXPECT_EQ(model.get_constraint_rhs(c), 3.);
}

struct highs_milp_test : public model_test<highs_api, highs_milp> {
    static void SetUpTestSuite() { construct_api("HIGHS"); }
};
INSTANTIATE_TEST(HiGHS_milp, LpModelTest, highs_milp_test);
INSTANTIATE_TEST(HiGHS_milp, MilpModelTest, highs_milp_test);
INSTANTIATE_TEST(HiGHS_milp, EnumerableEntitiesTest, highs_milp_test);
INSTANTIATE_TEST(HiGHS_milp, ReadableObjectiveTest, highs_milp_test);
INSTANTIATE_TEST(HiGHS_milp, ModifiableObjectiveTest, highs_milp_test);
INSTANTIATE_TEST(HiGHS_milp, ReadableVariablesBoundsTest, highs_milp_test);
INSTANTIATE_TEST(HiGHS_milp, ModifiableVariablesBoundsTest, highs_milp_test);
INSTANTIATE_TEST(HiGHS_milp, NamedVariablesTest, highs_milp_test);
INSTANTIATE_TEST(HiGHS_milp, NamedConstraintsTest, highs_milp_test);
INSTANTIATE_TEST(HiGHS_milp, AddColumnTest, highs_milp_test);
INSTANTIATE_TEST(HiGHS_milp, RemoveVariableTest, highs_milp_test);
INSTANTIATE_TEST(HiGHS_milp, ReadableConstraintsTest, highs_milp_test);
INSTANTIATE_TEST(HiGHS_milp, ReadableConstraintBoundsTest, highs_milp_test);
INSTANTIATE_TEST(HiGHS_milp, ModifiableConstraintBoundsTest, highs_milp_test);
INSTANTIATE_TEST(HiGHS_milp, IisByDeletionTest, highs_milp_test);
INSTANTIATE_TEST(HiGHS_milp, SudokuTest, highs_milp_test);
// INSTANTIATE_TEST(HiGHS_milp, MipStartTest, highs_milp_test);
INSTANTIATE_TEST(HiGHS_milp, TimeLimitTest, highs_milp_test);
INSTANTIATE_TEST(HiGHS_milp, TimeLimitIncumbentTest, highs_milp_test);
INSTANTIATE_TEST(HiGHS_milp, OptimalityToleranceTest, highs_milp_test);
INSTANTIATE_TEST(HiGHS_milp, MipGapTest, highs_milp_test);
INSTANTIATE_TEST(HiGHS_milp, IntegralityToleranceTest, highs_milp_test);
INSTANTIATE_TEST(HiGHS_milp, VerbosityTest, highs_milp_test);

struct highs_qp_test : public model_test<highs_api, highs_qp> {
    static void SetUpTestSuite() { construct_api("HIGHS"); }
};
INSTANTIATE_TEST(HiGHS_qp, LpModelTest, highs_qp_test);
INSTANTIATE_TEST(HiGHS_qp, QpModelTest, highs_qp_test);
INSTANTIATE_TEST(HiGHS_qp, EnumerableEntitiesTest, highs_qp_test);
INSTANTIATE_TEST(HiGHS_qp, ReadableObjectiveTest, highs_qp_test);
INSTANTIATE_TEST(HiGHS_qp, ReadableQuadraticObjectiveTest, highs_qp_test);
INSTANTIATE_TEST(HiGHS_qp, ModifiableObjectiveTest, highs_qp_test);
INSTANTIATE_TEST(HiGHS_qp, ReadableVariablesBoundsTest, highs_qp_test);
INSTANTIATE_TEST(HiGHS_qp, ModifiableVariablesBoundsTest, highs_qp_test);
INSTANTIATE_TEST(HiGHS_qp, NamedVariablesTest, highs_qp_test);
INSTANTIATE_TEST(HiGHS_qp, NamedConstraintsTest, highs_qp_test);
INSTANTIATE_TEST(HiGHS_qp, AddColumnTest, highs_qp_test);
INSTANTIATE_TEST(HiGHS_qp, RemoveVariableTest, highs_qp_test);
INSTANTIATE_TEST(HiGHS_qp, ReadableConstraintsTest, highs_qp_test);
INSTANTIATE_TEST(HiGHS_qp, ReadableConstraintBoundsTest, highs_qp_test);
INSTANTIATE_TEST(HiGHS_qp, ModifiableConstraintBoundsTest, highs_qp_test);
INSTANTIATE_TEST(HiGHS_qp, IisByDeletionTest, highs_qp_test);
INSTANTIATE_TEST(HiGHS_qp, DualSolutionTest, highs_qp_test);
INSTANTIATE_TEST(HiGHS_qp, LpStatusTest, highs_qp_test);
INSTANTIATE_TEST(HiGHS_qp, CuttingStockTest, highs_qp_test);
INSTANTIATE_TEST(HiGHS_qp, ColumnManagerTest, highs_qp_test);
INSTANTIATE_TEST(HiGHS_qp, TimeLimitTest, highs_qp_test);
INSTANTIATE_TEST(HiGHS_qp, IterationLimitTest, highs_qp_test);
INSTANTIATE_TEST(HiGHS_qp, VerbosityTest, highs_qp_test);

namespace {
// Cost swings of many pivots each make the time spent in HiGHS dwarf the
// final re-solve, which the halved value turns into a few dual simplex
// pivots: where the limit bounds the model's cumulative solve time, that
// re-solve stops before its first pivot.
template <typename Fixture>
void check_time_limit_bounds_each_solve(typename Fixture::model_type model) {
    using seconds = std::chrono::duration<double>;
    TimeLimitTest<Fixture>::build_dense_lp(model, 300);
    const auto x0 = model.variables().front();
    const double cost = model.get_objective_coefficient(x0);
    seconds spent{0};
    for(const double c : {cost, 1e4, cost, 1e4, cost}) {
        model.set_objective_coefficient(x0, c);
        const auto start = std::chrono::steady_clock::now();
        model.solve();
        spent += std::chrono::steady_clock::now() - start;
        ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
    }
    model.set_time_limit(spent / 2);
    const auto solution = model.get_solution();
    auto largest = x0;
    for(auto v : model.variables())
        if(solution[v] > solution[largest]) largest = v;
    model.set_variable_upper_bound(largest, solution[largest] / 2);
    model.solve();
    EXPECT_TRUE(is_a<status::optimal>(model.get_status()))
        << "under a limit of " << (spent / 2).count() << " s";
}
}  // namespace
TEST_F(highs_lp_test, time_limit_bounds_each_solve) {
    check_time_limit_bounds_each_solve<highs_lp_test>(new_model());
}
TEST_F(highs_milp_test, time_limit_bounds_each_solve) {
    check_time_limit_bounds_each_solve<highs_milp_test>(new_model());
}
TEST_F(highs_qp_test, time_limit_bounds_each_solve) {
    check_time_limit_bounds_each_solve<highs_qp_test>(new_model());
}

// The removal shifts the native ids the Hessian is written with, and the
// re-solve meets the quadratic optimum, which a lost Hessian would leave
// unbounded.
TEST_F(highs_qp_test, deletion_filter_keeps_the_quadratic_objective) {
    using namespace operators;
    auto model = new_model();
    auto removed = model.add_variable();
    auto x = model.add_variable({.lower_bound = 0., .upper_bound = 1.});
    auto v = model.add_variable();
    model.remove_variable(removed);
    model.add_constraint(x >= 2.);
    model.set_quadratic_objective(v * v - 2 * v + x);
    const auto iis = compute_iis_by_deletion(model);
    EXPECT_EQ(iis.get_outcome(), iis_outcome::irreducible);
    ASSERT_QUAD_EXPR(model.get_quadratic_objective(), {{v, v, 1.}},
                     {{x, 1.}, {v, -2.}}, 0.);
    model.set_variable_upper_bound(x, 3.);
    model.solve();
    ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
    EXPECT_NEAR(model.get_solution_value(), 1., TEST_EPSILON);
}

static_assert(has_iis<highs_lp>);
static_assert(has_iis<highs_qp>);
// the routine explains the relaxation only
static_assert(!has_iis<highs_milp>);
static_assert(iis_by_deletion_model<highs_lp>);
static_assert(iis_by_deletion_model<highs_milp>);
static_assert(iis_by_deletion_model<highs_qp>);

namespace {
// Highs_getIis is decodable from this release on; older libraries throw
constexpr solver_version highs_native_iis_floor{1, 14, 0};
}  // namespace

// A library reporting no version is taken to be a development build, newer
// than any release, so it runs the suite.
template <typename Model>
struct highs_iis_test : public model_test<highs_api, Model> {
    static void SetUpTestSuite() {
        model_test<highs_api, Model>::construct_api("HIGHS");
    }
    void SetUp() override {
        model_test<highs_api, Model>::SetUp();
        if(::testing::Test::HasFatalFailure() || ::testing::Test::IsSkipped() ||
           this->api == nullptr)
            return;
        const auto loaded = this->api->library_version();
        if(loaded && *loaded < highs_native_iis_floor)
            GTEST_SKIP() << "Highs_getIis needs HiGHS "
                         << to_string(highs_native_iis_floor) << ", "
                         << this->api->library_path() << " is "
                         << to_string(*loaded);
    }

    // x in [0, 1] against x >= 2: the row's lower side and the upper bound
    static auto add_bound_row_conflict(Model & model) {
        using namespace operators;
        auto x = model.add_variable({.lower_bound = 0., .upper_bound = 1.});
        model.add_constraint(x >= 2.);
        return x;
    }
    // A conflict that needs a solve to be found: the cheap checks that run
    // before the time budget applies answer a singleton-row conflict at once
    static void add_row_conflict(Model & model) {
        using namespace operators;
        auto x = model.add_variable();
        auto y = model.add_variable();
        model.add_constraint(x + y >= 3.);
        model.add_constraint(x <= 1.);
        model.add_constraint(y <= 1.);
        model.add_constraint(x - y <= 10.);
    }
    static int read_iis_strategy(const Model & model) {
        int value = 0;
        model.native_api()._check(model.native_api().getIntOptionValue(
            model.native_model(), "iis_strategy", &value));
        return value;
    }
    static double read_iis_time_limit(const Model & model) {
        double value = 0.;
        model.native_api()._check(model.native_api().getDoubleOptionValue(
            model.native_model(), "iis_time_limit", &value));
        return value;
    }
};
using highs_lp_iis_test = highs_iis_test<highs_lp>;
using highs_qp_iis_test = highs_iis_test<highs_qp>;
INSTANTIATE_TEST(HiGHS_lp, IisTest, highs_lp_iis_test);
INSTANTIATE_TEST(HiGHS_qp, IisTest, highs_qp_iis_test);

TEST(HiGHS_lp, compute_iis_below_native_floor_throws) {
    // inline, as MIPPP_API_VERSION_TEST does: GTEST_SKIP returns void, so no
    // helper returning the api can skip
    const highs_api * api = nullptr;
    try {
        api = &highs_api::load();
    } catch(const std::exception & e) {
        if(is_required_solver("HIGHS")) FAIL() << e.what();
        GTEST_SKIP() << e.what();
    }
    const auto loaded = api->library_version();
    if(!loaded || *loaded >= highs_native_iis_floor)
        GTEST_SKIP() << "the loaded HiGHS has the native routine";
    using namespace operators;
    highs_lp model(*api);
    auto x = model.add_variable({.lower_bound = 0., .upper_bound = 1.});
    model.add_constraint(x >= 2.);
    model.solve();
    // the message names the floor and the library, so a generic failure of
    // the same exception type is told apart
    try {
        [[maybe_unused]] const auto iis = model.compute_iis();
        ADD_FAILURE() << "compute_iis() returned below the native floor";
    } catch(const solver_error & e) {
        EXPECT_THAT(e.what(), ::testing::HasSubstr("1.14"));
        EXPECT_THAT(e.what(),
                    ::testing::HasSubstr(api->library_path().string()));
    }
    // the status is reset before the version check, so a throw leaves it too
    EXPECT_TRUE(is<status::unknown>(model.get_status()));
}

// The routine reads its own options, filled from the model's limit for the
// call only: nothing the wrapper set for the call may outlive it.
TEST_F(highs_lp_iis_test, compute_iis_restores_its_options) {
    auto model = this->new_model();
    const int strategy_before = read_iis_strategy(model);
    const double iis_time_limit_before = read_iis_time_limit(model);
    model.set_time_limit(std::chrono::seconds(3));
    add_bound_row_conflict(model);
    const auto iis = model.compute_iis();
    EXPECT_EQ(iis.get_outcome(), iis_outcome::irreducible);
    EXPECT_EQ(read_iis_strategy(model), strategy_before);
    EXPECT_EQ(read_iis_time_limit(model), iis_time_limit_before);
    EXPECT_EQ(model.get_time_limit().count(), 3.);
}

TEST_F(highs_lp_iis_test, compute_iis_zero_budget_is_a_time_limit_stop) {
    auto model = this->new_model();
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

TEST_F(highs_lp_iis_test, compute_iis_status_is_unknown_after_the_call) {
    using namespace operators;
    auto model = this->new_model();
    auto x = model.add_variable();
    model.add_constraint(x <= 1.);
    model.set_maximization();
    model.set_objective(x);
    model.solve();
    ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
    const auto iis = model.compute_iis();
    EXPECT_EQ(iis.get_outcome(), iis_outcome::feasible);
    EXPECT_TRUE(is<status::unknown>(model.get_status()));
    model.solve();
    ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
    EXPECT_NEAR(model.get_solution()[x], 1., TEST_EPSILON);
}

TEST_F(highs_qp_iis_test, compute_iis_keeps_the_quadratic_objective) {
    using namespace operators;
    auto model = this->new_model();
    auto x = add_bound_row_conflict(model);
    auto v = model.add_variable();
    model.set_quadratic_objective(v * v + x);
    const auto iis = model.compute_iis();
    EXPECT_EQ(iis.get_outcome(), iis_outcome::irreducible);
    ASSERT_QUAD_EXPR(model.get_quadratic_objective(), {{v, v, 1.}},
                     {{x, 1.}, {v, 0.}}, 0.);
}

// The row brings the first nonzero, so HiGHS holds the matrix row-wise, and
// its answer is that row alone. The sides are written through the native
// call, so that the case does not depend on the row-bound setters. Without
// the column-wise switch, a gcc build usually still passes: the read past the
// array shows only under valgrind, in a clang build (segfault) or on a HiGHS
// built with assertions.
TEST_F(highs_lp_iis_test, one_row_answer_on_a_row_wise_matrix) {
    using namespace operators;
    auto model = this->new_model();
    auto x = model.add_variable();
    auto y = model.add_variable({.lower_bound = 0., .upper_bound = 1.});
    auto c = model.add_constraint(x >= 1.);
    model.add_constraint(y <= 3.);
    model.native_api()._check(model.native_api().changeRowBounds(
        model.native_model(), model.native_id(c), 1., 0.));
    const auto iis = model.compute_iis();
    EXPECT_EQ(iis.get_outcome(), iis_outcome::irreducible);
    EXPECT_TRUE(is<iis_status::member_both>(iis.get_status(c)));
    EXPECT_EQ(iis.num_constraint_members(), 1u);
    EXPECT_EQ(iis.num_variable_members(), 0u);
}

// HiGHS reports any crossed row as boxed, while 0, the activity of a row
// without terms, violates only one of the two sides.
TEST_F(highs_lp_iis_test, crossed_row_without_terms_keeps_one_side) {
    using namespace operators;
    constexpr auto & no_terms =
        empty_linear_expression<model_variable_t<highs_lp>, double>;
    struct crossed_row {
        double lower;
        double upper;
        bool lower_side_expected;
    };
    for(const crossed_row & r :
        {crossed_row{1., 0., true}, crossed_row{0., -1., false},
         crossed_row{2., -1., true}}) {
        SCOPED_TRACE(std::to_string(r.lower) + " > " + std::to_string(r.upper));
        auto model = this->new_model();
        auto x = model.add_variable({.lower_bound = 0., .upper_bound = 1.});
        auto c = model.add_constraint(no_terms >= r.lower);
        model.add_constraint(x <= 3.);
        model.native_api()._check(model.native_api().changeRowBounds(
            model.native_model(), model.native_id(c), r.lower, r.upper));
        const auto iis = model.compute_iis();
        EXPECT_EQ(iis.get_outcome(), iis_outcome::irreducible);
        if(r.lower_side_expected)
            EXPECT_TRUE(is<iis_status::member_lower>(iis.get_status(c)));
        else
            EXPECT_TRUE(is<iis_status::member_upper>(iis.get_status(c)));
        EXPECT_EQ(iis.num_constraint_members(), 1u);
        EXPECT_EQ(iis.num_variable_members(), 0u);
    }
}
