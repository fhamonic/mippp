#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <random>
#include <ranges>
#include <stdexcept>
#include <string>
#include <vector>

#include "mippp/solvers/xpress/all.hpp"

using namespace mippp;

#include "test_suites/all.hpp"

MIPPP_API_VERSION_TEST(Xpress_api, xpress_api, "XPRESS")

struct xpress_lp_test : public model_test<xpress_api, xpress_lp> {
    static void SetUpTestSuite() { construct_api("XPRESS"); }
};
INSTANTIATE_TEST(Xpress_lp, LpModelTest, xpress_lp_test);
INSTANTIATE_TEST(Xpress_lp, EnumerableEntitiesTest, xpress_lp_test);
INSTANTIATE_TEST(Xpress_lp, ReadableObjectiveTest, xpress_lp_test);
INSTANTIATE_TEST(Xpress_lp, ModifiableObjectiveTest, xpress_lp_test);
INSTANTIATE_TEST(Xpress_lp, ReadableVariablesBoundsTest, xpress_lp_test);
INSTANTIATE_TEST(Xpress_lp, ModifiableVariablesBoundsTest, xpress_lp_test);
INSTANTIATE_TEST(Xpress_lp, NamedVariablesTest, xpress_lp_test);
INSTANTIATE_TEST(Xpress_lp, AddColumnTest, xpress_lp_test);
INSTANTIATE_TEST(Xpress_lp, ReadableConstraintBoundsTest, xpress_lp_test);
INSTANTIATE_TEST(Xpress_lp, DualSolutionTest, xpress_lp_test);
INSTANTIATE_TEST(Xpress_lp, ReducedCostsTest, xpress_lp_test);
INSTANTIATE_TEST(Xpress_lp, LpStatusTest, xpress_lp_test);
INSTANTIATE_TEST(Xpress_lp, CuttingStockTest, xpress_lp_test);
INSTANTIATE_TEST(Xpress_lp, TimeLimitTest, xpress_lp_test);
INSTANTIATE_TEST(Xpress_lp, VerbosityTest, xpress_lp_test);

struct xpress_milp_test : public model_test<xpress_api, xpress_milp> {
    static void SetUpTestSuite() { construct_api("XPRESS"); }
};
INSTANTIATE_TEST(Xpress_milp, LpModelTest, xpress_milp_test);
INSTANTIATE_TEST(Xpress_milp, MilpModelTest, xpress_milp_test);
INSTANTIATE_TEST(Xpress_milp, EnumerableEntitiesTest, xpress_milp_test);
INSTANTIATE_TEST(Xpress_milp, ReadableObjectiveTest, xpress_milp_test);
INSTANTIATE_TEST(Xpress_milp, ModifiableObjectiveTest, xpress_milp_test);
INSTANTIATE_TEST(Xpress_milp, ReadableVariablesBoundsTest, xpress_milp_test);
INSTANTIATE_TEST(Xpress_milp, ModifiableVariablesBoundsTest, xpress_milp_test);
INSTANTIATE_TEST(Xpress_milp, NamedVariablesTest, xpress_milp_test);
INSTANTIATE_TEST(Xpress_milp, AddColumnTest, xpress_milp_test);
INSTANTIATE_TEST(Xpress_milp, ReadableConstraintBoundsTest, xpress_milp_test);
INSTANTIATE_TEST(Xpress_milp, CandidateSolutionCallbackTest, xpress_milp_test);
static_assert(mippp::has_candidate_solution_callback<mippp::xpress_milp>);
static_assert(!mippp::has_lazy_constraints<
              mippp::candidate_solution_callback_handle_t<mippp::xpress_milp>,
              mippp::xpress_milp>);
static_assert(mippp::has_candidate_solution_rejection<
              mippp::candidate_solution_callback_handle_t<mippp::xpress_milp>>);
INSTANTIATE_TEST(Xpress_milp, SudokuTest, xpress_milp_test);
INSTANTIATE_TEST(Xpress_milp, TimeLimitTest, xpress_milp_test);
INSTANTIATE_TEST(Xpress_milp, TimeLimitIncumbentTest, xpress_milp_test);
INSTANTIATE_TEST(Xpress_milp, MipStartTest, xpress_milp_test);
INSTANTIATE_TEST(Xpress_milp, OptimalityToleranceTest, xpress_milp_test);
INSTANTIATE_TEST(Xpress_milp, MipGapTest, xpress_milp_test);
INSTANTIATE_TEST(Xpress_milp, IntegralityToleranceTest, xpress_milp_test);
INSTANTIATE_TEST(Xpress_milp, VerbosityTest, xpress_milp_test);

// Xpress stores a ranged row as its upper side and a non-negative width, so
// a row whose sides cross has no encoding and the setters refuse it: a case
// holding one is skipped from its data. 45.01 also solves a model with a
// column whose bounds hold no value (crossed, or an integer column with no
// integer between them) beside a row to optimal, where 47.01 reports it
// infeasible, so the closing solve check of such a case skips below 47.1; no
// 46 was measured.
template <typename Model>
struct xpress_deletion_test : public model_test<xpress_api, Model> {
    static void SetUpTestSuite() {
        model_test<xpress_api, Model>::construct_api("XPRESS");
    }
    static std::optional<std::string> crossed_row_reason(
        const iis_cases::iis_case & c) {
        for(std::size_t i = 0; i < c.system.rows.size(); ++i) {
            const auto & row = c.system.rows[i];
            if(row.lower && row.upper && *row.lower > *row.upper)
                return "Xpress cannot hold row " + std::to_string(i) +
                       ", whose sides cross";
        }
        return std::nullopt;
    }
    static std::optional<std::size_t> column_whose_bounds_hold_no_value(
        const iis_cases::iis_case & c) {
        constexpr double inf = std::numeric_limits<double>::infinity();
        for(std::size_t i = 0; i < c.system.variables.size(); ++i) {
            const auto & bounds = c.system.variables[i];
            double lower = bounds.lower.value_or(-inf);
            double upper = bounds.upper.value_or(inf);
            if(std::ranges::find(c.integer_columns, i) !=
               c.integer_columns.end()) {
                lower = std::ceil(lower);
                upper = std::floor(upper);
            }
            if(lower > upper) return i;
        }
        return std::nullopt;
    }
    static std::optional<std::string> iis_case_skip_reason(
        const iis_cases::iis_case & c) {
        if(const auto reason = crossed_row_reason(c)) return reason;
        const auto * api = model_test<xpress_api, Model>::api;
        const auto loaded = api ? api->library_version() : std::nullopt;
        if(!loaded || *loaded >= solver_version{47, 1}) return std::nullopt;
        if(const auto i = column_whose_bounds_hold_no_value(c))
            return "Xpress " + to_string(*loaded) +
                   " solves a model whose column " + std::to_string(*i) +
                   " has bounds holding no value to optimal (measured on "
                   "45.01; 47.1 reports infeasible), so the closing solve "
                   "check cannot run here";
        return std::nullopt;
    }
};
using xpress_lp_deletion_test = xpress_deletion_test<xpress_lp>;
using xpress_milp_deletion_test = xpress_deletion_test<xpress_milp>;
static_assert(has_modifiable_constraint_bounds<xpress_lp>);
static_assert(has_modifiable_constraint_bounds<xpress_milp>);
static_assert(iis_by_deletion_model<xpress_lp>);
static_assert(iis_by_deletion_model<xpress_milp>);
INSTANTIATE_TEST(Xpress_lp, ModifiableConstraintBoundsTest, xpress_lp_test);
INSTANTIATE_TEST(Xpress_milp, ModifiableConstraintBoundsTest, xpress_milp_test);
INSTANTIATE_TEST(Xpress_lp, IisByDeletionTest, xpress_lp_deletion_test);
INSTANTIATE_TEST(Xpress_milp, IisByDeletionTest, xpress_milp_deletion_test);

// Both sides read back exactly as set, through every row type a row can
// pass through: one-sided, ranged, equal and free.
TEST_F(xpress_lp_test, row_sides_read_back_through_every_row_type) {
    using namespace operators;
    auto model = this->new_model();
    auto x = model.add_variable({.lower_bound = -10., .upper_bound = 10.});
    auto c = model.add_constraint(x >= 1.);
    model.set_constraint_upper_bound(c, 4.);
    EXPECT_EQ(model.get_constraint_lower_bound(c), 1.);
    EXPECT_EQ(model.get_constraint_upper_bound(c), 4.);
    model.set_constraint_lower_bound(c, -2.);
    EXPECT_EQ(model.get_constraint_lower_bound(c), -2.);
    EXPECT_EQ(model.get_constraint_upper_bound(c), 4.);
    model.set_objective(x);
    model.set_maximization();
    model.solve();
    EXPECT_NEAR(model.get_solution_value(), 4., TEST_EPSILON);
    model.set_minimization();
    model.solve();
    EXPECT_NEAR(model.get_solution_value(), -2., TEST_EPSILON);
    model.set_constraint_upper_bound(c, model.infinity());
    model.set_constraint_lower_bound(c, -model.infinity());
    EXPECT_TRUE(model.is_infinite(model.get_constraint_lower_bound(c)));
    EXPECT_TRUE(model.is_infinite(model.get_constraint_upper_bound(c)));
    model.set_constraint_lower_bound(c, 3.);
    model.set_constraint_upper_bound(c, 3.);
    EXPECT_EQ(model.get_constraint_lower_bound(c), 3.);
    EXPECT_EQ(model.get_constraint_upper_bound(c), 3.);
    model.solve();
    EXPECT_NEAR(model.get_solution_value(), 3., TEST_EPSILON);
}

TEST_F(xpress_lp_test, crossed_row_sides_are_refused) {
    using namespace operators;
    auto model = this->new_model();
    auto x = model.add_variable({.lower_bound = -10., .upper_bound = 10.});
    auto c = model.add_constraint(x >= 1.);
    EXPECT_THROW(model.set_constraint_upper_bound(c, 0.),
                 std::invalid_argument);
    EXPECT_EQ(model.get_constraint_lower_bound(c), 1.);
    EXPECT_TRUE(model.is_infinite(model.get_constraint_upper_bound(c)));
    model.set_constraint_upper_bound(c, 5.);
    EXPECT_THROW(model.set_constraint_lower_bound(c, 6.),
                 std::invalid_argument);
    EXPECT_EQ(model.get_constraint_lower_bound(c), 1.);
    EXPECT_EQ(model.get_constraint_upper_bound(c), 5.);
}

// Xpress 45.01 leaves an LP stopped by its time limit presolved, with the
// fixed columns and the redundant rows removed: the rows built must still
// be the ones addressed afterwards. Dense rows keep the solve past the 10 ms
// step of the solver's clock.
TEST_F(xpress_lp_test, rows_stay_addressable_after_a_stopped_solve) {
    using namespace operators;
    auto model = this->new_model();
    constexpr std::size_t n = 400;
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> coef(0.1, 1.);
    auto x = model.add_variables(n, {.lower_bound = 0., .upper_bound = 10.});
    (void)model.add_variables(n / 5, {.lower_bound = 3., .upper_bound = 3.});
    model.set_objective(xsum(std::views::iota(std::size_t{0}, n),
                             [&](auto j) { return x(j); }));
    model.set_minimization();
    std::vector<double> row(n);
    for(std::size_t i = 0; i < n; ++i) {
        for(auto & a : row) a = coef(rng);
        model.add_constraint(xsum(std::views::iota(std::size_t{0}, n),
                                  [&](auto j) { return row[j] * x(j); }) >= 1.);
    }
    for(std::size_t i = 0; i < n / 4; ++i) model.add_constraint(x(i) <= 1e6);
    auto last = model.add_constraint(xsum(std::views::iota(std::size_t{0}, n),
                                          [&](auto j) { return x(j); }) <= 1e6);
    const auto num_rows = model.num_constraints();
    model.set_time_limit(std::chrono::milliseconds(1));
    model.solve();
    ASSERT_TRUE(is<status::time_limit>(model.get_status()));
    EXPECT_EQ(model.num_constraints(), num_rows);
    EXPECT_EQ(model.get_constraint_upper_bound(last), 1e6);
    model.set_constraint_upper_bound(last, 5.);
    EXPECT_EQ(model.get_constraint_upper_bound(last), 5.);
    auto added = model.add_constraint(x(0) <= 8.);
    EXPECT_EQ(model.get_constraint_upper_bound(added), 8.);
}

// XPRSiisfirst refuses the search on a column whose bounds hold no value,
// crossed or an integer column with no integer between them: stated from the
// case data, so that the suite skips the case instead of catching the throw.
template <typename Model>
struct xpress_iis_test : public xpress_deletion_test<Model> {
    using base = xpress_deletion_test<Model>;
    static std::optional<std::string> iis_case_skip_reason(
        const iis_cases::iis_case & c) {
        if(const auto reason = base::crossed_row_reason(c)) return reason;
        if(const auto i = base::column_whose_bounds_hold_no_value(c))
            return "Xpress refuses the IIS search on column " +
                   std::to_string(*i) + ", whose bounds hold no value";
        return std::nullopt;
    }

    // x in [0, 1] against x >= 2: the row's lower side and the upper bound
    static auto add_bound_row_conflict(Model & model) {
        using namespace operators;
        auto x = model.add_variable({.lower_bound = 0., .upper_bound = 1.});
        model.add_constraint(x >= 2.);
        return x;
    }
    // A conflict that needs a solve to be found: a zero time limit stops the
    // routine before it, and with no limit the three rows are the IIS.
    static void add_row_conflict(Model & model) {
        using namespace operators;
        auto x = model.add_variable();
        auto y = model.add_variable();
        model.add_constraint(x + y >= 3.);
        model.add_constraint(x <= 1.);
        model.add_constraint(y <= 1.);
        model.add_constraint(x - y <= 10.);
    }
    // Four dense equality rows over 26 binaries with no integer point: the
    // IIS is the whole model, found by a MIP deletion filter of several
    // seconds, so a time limit stops it. Under the default IISOPS, Xpress
    // 45.01 and 47.01 abort the process on it.
    static void add_market_split_conflict(Model & model) {
        using namespace operators;
        constexpr std::size_t n = 26;
        std::mt19937 rng(42);
        std::uniform_int_distribution<int> coef(0, 99);
        auto x = model.add_binary_variables(n);
        std::vector<int> row(n);
        for(int i = 0; i < 4; ++i) {
            int total = 0;
            for(int & a : row) total += (a = coef(rng));
            model.add_constraint(xsum(std::views::iota(std::size_t{0}, n),
                                      [&](auto j) { return row[j] * x(j); }) ==
                                 total / 2);
        }
    }
    static constexpr int iisops_control = xpress::impl::v1::XPRS_IISOPS;
    // Xpress reports an iteration limit hit inside the search as the same
    // stop status as its time limit.
    static void limit_lp_iterations_to_one(const Model & model) {
        model.native_api()._check(
            model.native_model(),
            model.native_api().setintcontrol(
                model.native_model(), xpress::impl::v1::XPRS_LPITERLIMIT, 1));
    }
    static int read_iisops(const Model & model) {
        int value = 0;
        model.native_api()._check(
            model.native_model(),
            model.native_api().getintcontrol(model.native_model(),
                                             iisops_control, &value));
        return value;
    }
    static void write_iisops(const Model & model, int value) {
        model.native_api()._check(
            model.native_model(),
            model.native_api().setintcontrol(model.native_model(),
                                             iisops_control, value));
    }
};
using xpress_lp_iis_test = xpress_iis_test<xpress_lp>;
using xpress_milp_iis_test = xpress_iis_test<xpress_milp>;
static_assert(has_iis<xpress_lp>);
static_assert(has_iis<xpress_milp>);
INSTANTIATE_TEST(Xpress_lp, IisTest, xpress_lp_iis_test);
INSTANTIATE_TEST(Xpress_milp, IisTest, xpress_milp_iis_test);

// The routine protects integrality and the special constraints through
// IISOPS for the call only: a value the user set through the native handle
// reads back afterwards, whether the call returns or throws.
TEST_F(xpress_lp_iis_test, compute_iis_restores_iisops) {
    auto model = this->new_model();
    constexpr int keep_all_variable_bounds = 8;
    write_iisops(model, keep_all_variable_bounds);
    model.set_time_limit(std::chrono::seconds(3));
    add_bound_row_conflict(model);
    const auto iis = model.compute_iis();
    EXPECT_EQ(iis.get_outcome(), iis_outcome::irreducible);
    EXPECT_EQ(read_iisops(model), keep_all_variable_bounds);
    EXPECT_EQ(model.get_time_limit().count(), 3.);
}

TEST_F(xpress_lp_iis_test, bound_conflict_on_a_column_is_refused) {
    using namespace operators;
    auto model = this->new_model();
    constexpr int keep_all_variable_bounds = 8;
    write_iisops(model, keep_all_variable_bounds);
    auto x = model.add_variable({.lower_bound = 1., .upper_bound = 0.});
    auto y = model.add_variable({.lower_bound = 0., .upper_bound = 5.});
    model.add_constraint(y <= 3.);
    EXPECT_THROW((void)model.compute_iis(), solver_error);
    EXPECT_EQ(read_iisops(model), keep_all_variable_bounds);
    EXPECT_TRUE(is<status::unknown>(model.get_status()));
    EXPECT_EQ(model.get_variable_lower_bound(x), 1.);
    EXPECT_EQ(model.get_variable_upper_bound(x), 0.);
}

TEST_F(xpress_lp_iis_test, compute_iis_zero_budget_is_a_time_limit_stop) {
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

TEST_F(xpress_milp_iis_test, compute_iis_zero_budget_is_a_time_limit_stop) {
    auto model = this->new_model();
    add_row_conflict(model);
    model.set_time_limit(std::chrono::seconds(0));
    const auto iis = model.compute_iis();
    EXPECT_EQ(iis.get_outcome(), iis_outcome::undetermined);
    EXPECT_EQ(iis.get_reason(), iis_reason::time_limit);
    EXPECT_EQ(iis.num_constraint_members(), 0u);
    EXPECT_EQ(model.get_time_limit().count(), 0.);
    EXPECT_TRUE(is<status::unknown>(model.get_status()));
}

// A stop of another origin than the model's time limit carries no reason.
TEST_F(xpress_lp_iis_test, iteration_limit_stop_carries_no_reason) {
    auto model = this->new_model();
    add_row_conflict(model);
    limit_lp_iterations_to_one(model);
    const auto iis = model.compute_iis();
    EXPECT_EQ(iis.get_outcome(), iis_outcome::undetermined);
    EXPECT_EQ(iis.get_reason(), std::nullopt);
    EXPECT_EQ(iis.num_constraint_members(), 0u);
    EXPECT_TRUE(is<status::unknown>(model.get_status()));
}

TEST_F(xpress_milp_iis_test, iteration_limit_stop_carries_no_reason) {
    auto model = this->new_model();
    add_market_split_conflict(model);
    limit_lp_iterations_to_one(model);
    const auto iis = model.compute_iis();
    EXPECT_EQ(iis.get_outcome(), iis_outcome::undetermined);
    EXPECT_EQ(iis.get_reason(), std::nullopt);
    EXPECT_EQ(iis.num_constraint_members(), 0u);
    EXPECT_TRUE(is<status::unknown>(model.get_status()));
}

// Xpress lists a column once per bound it needs, 'U' then 'L' (measured on
// 45.01 and 47.01): a continuous x in [0.5, 1.5] with an integer y under
// x + 2y = 4 needs both, since x = 0 lets y = 2 and x = 2 lets y = 1.
TEST_F(xpress_milp_iis_test, column_listed_with_both_bounds_merges_the_sides) {
    using namespace operators;
    auto model = this->new_model();
    auto x = model.add_variable({.lower_bound = 0.5, .upper_bound = 1.5});
    auto y =
        model.add_integer_variable({.lower_bound = 0., .upper_bound = 10.});
    auto r = model.add_constraint(x + 2 * y == 4.);
    const auto iis = model.compute_iis();
    EXPECT_EQ(iis.get_outcome(), iis_outcome::irreducible);
    EXPECT_TRUE(is<iis_status::member_both>(iis.get_status(x)));
    EXPECT_TRUE(is<iis_status::absent>(iis.get_status(y)));
    EXPECT_TRUE(is<iis_status::member_both>(iis.get_status(r)));
    EXPECT_EQ(iis.num_variable_members(), 1u);
    EXPECT_EQ(iis.num_constraint_members(), 1u);
}

// The routine's MIP solves would run a registered callback, whose rejection
// of every candidate makes a feasible model read infeasible and the answer
// name the whole model.
TEST_F(xpress_milp_iis_test,
       registered_callback_does_not_run_during_compute_iis) {
    using namespace operators;
    auto model = this->new_model();
    auto x = model.add_integer_variable({.lower_bound = 0., .upper_bound = 5.});
    auto y = model.add_integer_variable({.lower_bound = 0., .upper_bound = 5.});
    auto r = model.add_constraint(x + y <= 8.);
    model.set_maximization();
    model.set_objective(x + y);
    int fired = 0;
    model.set_candidate_solution_callback([&](auto & handle) {
        ++fired;
        handle.reject_solution();
    });
    const auto iis = model.compute_iis();
    EXPECT_EQ(iis.get_outcome(), iis_outcome::feasible);
    EXPECT_EQ(fired, 0);
    EXPECT_TRUE(is<iis_status::absent>(iis.get_status(x)));
    EXPECT_TRUE(is<iis_status::absent>(iis.get_status(r)));
    // the callback is back for the next solve
    model.solve();
    EXPECT_GE(fired, 1);
    EXPECT_TRUE(is_a<status::infeasible>(model.get_status()));
}

TEST_F(xpress_milp_iis_test, market_split_conflict_completes) {
    auto model = this->new_model();
    add_market_split_conflict(model);
    const auto iis = model.compute_iis();
    EXPECT_EQ(iis.get_outcome(), iis_outcome::irreducible);
    EXPECT_EQ(iis.get_reason(), std::nullopt);
    EXPECT_EQ(iis.num_constraint_members(), 4u);
    EXPECT_EQ(iis.num_variable_members(), 0u);
    model.solve();
    EXPECT_TRUE(is_a<status::infeasible>(model.get_status()));
}

// A stop in the MIP deletion filter returns the set it holds, not proven
// minimal, or nothing at all when it comes before the first subsystem. The
// budget is a small fraction of the seconds the search needs, so a faster
// release or machine still stops.
TEST_F(xpress_milp_iis_test, market_split_stop_is_a_time_limit_stop) {
    auto model = this->new_model();
    add_market_split_conflict(model);
    model.set_time_limit(std::chrono::milliseconds(10));
    const auto iis = model.compute_iis();
    EXPECT_EQ(iis.get_reason(), iis_reason::time_limit);
    EXPECT_TRUE(iis.get_outcome() == iis_outcome::undetermined ||
                iis.get_outcome() == iis_outcome::not_proven_minimal);
    EXPECT_LE(iis.num_constraint_members(), 4u);
    EXPECT_EQ(iis.num_variable_members(), 0u);
    EXPECT_EQ(model.get_time_limit().count(), 0.01);
    EXPECT_TRUE(is<status::unknown>(model.get_status()));
}
