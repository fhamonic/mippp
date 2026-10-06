#include <algorithm>
#include <chrono>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <limits>
#include <optional>
#include <random>
#include <ranges>
#include <stdexcept>
#include <string>
#include <variant>
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

// A column whose bounds hold no value makes the model infeasible, beyond the
// tolerances: FEASTOL for bounds that cross, MIPTOL around an integer.
// Measured on 46.1.3 and 47.1.1; below 46.1.3 solve() answers it itself.
template <typename Model>
static bool solves_infeasible(Model model, double lower, double upper,
                              bool integer) {
    using namespace operators;
    auto x = model.add_variable({.lower_bound = lower, .upper_bound = upper});
    if constexpr(milp_model<Model>)
        if(integer)
            x = model.add_integer_variable(
                {.lower_bound = lower, .upper_bound = upper});
    model.add_constraint(x <= 5.);
    model.solve();
    return is<status::infeasible>(model.get_status());
}
TEST_F(xpress_lp_test, column_whose_bounds_cross_is_infeasible) {
    EXPECT_TRUE(solves_infeasible(new_model(), 1., 0., false));
    EXPECT_TRUE(solves_infeasible(new_model(), 1., 1. - 1e-5, false));
    EXPECT_FALSE(solves_infeasible(new_model(), 1., 1. - 1e-7, false));
}
TEST_F(xpress_milp_test, integer_column_holding_no_integer_is_infeasible) {
    EXPECT_TRUE(solves_infeasible(new_model(), 1., 0., false));
    EXPECT_TRUE(solves_infeasible(new_model(), 0.2, 0.8, true));
    EXPECT_TRUE(solves_infeasible(new_model(), 1. + 1e-5, 1.5, true));
    EXPECT_FALSE(solves_infeasible(new_model(), 1. + 1e-7, 1.5, true));
    // crossing beyond FEASTOL, though 1 lies within MIPTOL of both bounds
    EXPECT_TRUE(solves_infeasible(new_model(), 1., 1. - 2e-6, true));
}

// Xpress stores a ranged row as its upper side and a non-negative width, so
// a row whose sides cross has no encoding and the setters refuse it.
template <typename Model>
struct xpress_deletion_test : public model_test<xpress_api, Model> {
    static void SetUpTestSuite() {
        model_test<xpress_api, Model>::construct_api("XPRESS");
    }
    static std::optional<std::string> iis_case_skip_reason(
        const iis_cases::iis_case & c) {
        return iis_cases::crossed_row_skip_reason(c, "Xpress");
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

template <typename Model>
struct xpress_iis_test : public xpress_deletion_test<Model> {
    // x in [0, 1] against x >= 2: the row's lower side and the upper bound
    static auto add_bound_row_conflict(Model & model) {
        using namespace operators;
        auto x = model.add_variable({.lower_bound = 0., .upper_bound = 1.});
        model.add_constraint(x >= 2.);
        return x;
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
    // a kind no MIP++ setter creates
    static void make_semi_continuous(const Model & model,
                                     model_variable_t<Model> x) {
        const int column = model.native_id(x);
        const char type = 'S';
        model.native_api()._check(model.native_model(),
                                  model.native_api().chgcoltype(
                                      model.native_model(), 1, &column, &type));
    }
    // the column alone, next to a row on another column, under a row on itself
    template <typename Side>
    void expect_binary_column_is_the_iis(double lower, double upper) {
        using namespace operators;
        for(int placement = 0; placement < 3; ++placement) {
            auto model = this->new_model();
            auto x = model.add_binary_variable();
            if(placement == 1) {
                auto y =
                    model.add_variable({.lower_bound = 0., .upper_bound = 1.});
                model.add_constraint(y <= 0.5);
            }
            if(placement == 2) model.add_constraint(x <= 4.);
            model.set_variable_lower_bound(x, lower);
            model.set_variable_upper_bound(x, upper);
            const auto iis = model.compute_iis();
            EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()))
                << placement;
            EXPECT_TRUE(is<Side>(iis.get_status(x))) << placement;
            EXPECT_EQ(iis.num_variable_members(), 1u) << placement;
            EXPECT_EQ(iis.num_constraint_members(), 0u) << placement;
        }
    }
};
using xpress_lp_iis_test = xpress_iis_test<xpress_lp>;
using xpress_milp_iis_test = xpress_iis_test<xpress_milp>;
static_assert(has_iis<xpress_lp>);
static_assert(has_iis<xpress_milp>);
static_assert(
    std::same_as<native_iis_outcome_t<xpress_lp>,
                 std::variant<iis_outcome::incomplete, iis_outcome::irreducible,
                              iis_outcome::feasible, iis_outcome::stopped,
                              iis_outcome::time_limit>>);
static_assert(std::same_as<native_iis_outcome_t<xpress_milp>,
                           native_iis_outcome_t<xpress_lp>>);
INSTANTIATE_TEST(Xpress_lp, IisTest, xpress_lp_iis_test);
INSTANTIATE_TEST(Xpress_milp, IisTest, xpress_milp_iis_test);

// The routine protects integrality and the special constraints through
// IISOPS for the call only: a value the user set through the native handle
// reads back afterwards, whether the call returns or throws.
TEST_F(xpress_lp_iis_test, compute_iis_restores_iisops) {
    auto model = this->new_model();
    constexpr int keep_all_variable_bounds = 8;
    write_iisops(model, keep_all_variable_bounds);
    add_bound_row_conflict(model);
    const auto iis = model.compute_iis();
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
    EXPECT_EQ(read_iisops(model), keep_all_variable_bounds);
}

// The routine's IIS stays readable through the native handle after the call,
// with its Farkas multipliers: the IISOPS write-back changes no problem data.
// Each member is needed, so each multiplier is nonzero.
TEST_F(xpress_lp_iis_test, routine_data_stays_readable_after_the_call) {
    using namespace operators;
    auto model = this->new_model();
    auto x = model.add_variable({.lower_bound = 0., .upper_bound = 1.});
    auto r = model.add_constraint(x >= 2.);
    const auto iis = model.compute_iis();
    ASSERT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
    const auto & native = model.native_api();
    int num_rows = 0, num_cols = 0;
    native._check(
        model.native_model(),
        native.getiisdata(model.native_model(), 1, &num_rows, &num_cols,
                          nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
                          nullptr, nullptr));
    ASSERT_EQ(static_cast<std::size_t>(num_rows), iis.num_constraint_members());
    ASSERT_EQ(static_cast<std::size_t>(num_cols), iis.num_variable_members());
    int row = -1, col = -1;
    char row_sense = 0, bound_side = 0;
    double dual = 0., dj = 0.;
    native._check(model.native_model(),
                  native.getiisdata(model.native_model(), 1, &num_rows,
                                    &num_cols, &row, &col, &row_sense,
                                    &bound_side, &dual, &dj, nullptr, nullptr));
    EXPECT_EQ(row, model.native_id(r));
    EXPECT_EQ(col, model.native_id(x));
    EXPECT_EQ(row_sense, 'G');
    EXPECT_EQ(bound_side, 'U');
    EXPECT_NE(dual, 0.);
    EXPECT_NE(dj, 0.);
}

// XPRSiisfirst refuses the whole search on a column whose bounds admit no
// value, even beside an unrelated row, so the column is answered from its
// bounds.
TEST_F(xpress_lp_iis_test, crossed_column_is_answered_from_its_bounds) {
    using namespace operators;
    auto model = this->new_model();
    constexpr int keep_all_variable_bounds = 8;
    write_iisops(model, keep_all_variable_bounds);
    auto x = model.add_variable({.lower_bound = 1., .upper_bound = 0.});
    auto y = model.add_variable({.lower_bound = 0., .upper_bound = 5.});
    auto r = model.add_constraint(y <= 3.);
    const auto iis = model.compute_iis();
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
    EXPECT_TRUE(is<iis_status::member_both>(iis.get_status(x)));
    EXPECT_TRUE(is<iis_status::absent>(iis.get_status(y)));
    EXPECT_TRUE(is<iis_status::absent>(iis.get_status(r)));
    EXPECT_EQ(iis.num_variable_members(), 1u);
    EXPECT_EQ(iis.num_constraint_members(), 0u);
    EXPECT_EQ(read_iisops(model), keep_all_variable_bounds);
    EXPECT_TRUE(is<status::unknown>(model.get_status()));
    EXPECT_EQ(model.get_variable_lower_bound(x), 1.);
    EXPECT_EQ(model.get_variable_upper_bound(x), 0.);
}

// Without rows the columns hold any conflict, so the model is answered from
// their bounds, under a zero limit too: 45.01 stops on the gap of the
// routine's internal MIP on this feasible model.
TEST_F(xpress_milp_iis_test, model_without_rows_is_answered_from_its_columns) {
    auto model = this->new_model();
    model.add_integer_variable({.lower_bound = 0., .upper_bound = 1.});
    model.set_time_limit(std::chrono::seconds(0));
    const auto iis = model.compute_iis();
    EXPECT_TRUE(outcome_is<iis_outcome::feasible>(iis.get_outcome()));
    EXPECT_EQ(iis.num_variable_members(), 0u);
    EXPECT_EQ(iis.num_constraint_members(), 0u);
}

// Xpress keeps the bounds of a binary column that exclude [0, 1] (lower bound
// 2, upper bound -2) and its routine refuses the search on them: the column is
// the IIS wherever the rows are, by the bound that excludes the domain alone,
// or by both when the interval lies inside it. A bound leaving room for a
// value outside [0, 1] turns the column integer instead.
TEST_F(xpress_milp_iis_test, binary_column_outside_its_domain_is_the_iis) {
    this->expect_binary_column_is_the_iis<iis_status::member_lower>(2., 1.);
    this->expect_binary_column_is_the_iis<iis_status::member_upper>(0., -2.);
    this->expect_binary_column_is_the_iis<iis_status::member_both>(0.25, 0.75);
}

// The refusal comes on a semi-continuous column with crossed bounds too, a
// kind whose admissible values the wrapper does not decide: it still throws,
// and IISOPS reads back.
TEST_F(xpress_milp_iis_test, refusal_without_a_decidable_column_throws) {
    auto model = this->new_model();
    constexpr int keep_all_variable_bounds = 8;
    write_iisops(model, keep_all_variable_bounds);
    auto x = model.add_variable({.lower_bound = 0., .upper_bound = 4.});
    make_semi_continuous(model, x);
    model.set_variable_lower_bound(x, 2.);
    model.set_variable_upper_bound(x, 1.);
    EXPECT_THROW((void)model.compute_iis(), solver_error);
    EXPECT_EQ(read_iisops(model), keep_all_variable_bounds);
    EXPECT_TRUE(is<status::unknown>(model.get_status()));
}

// A stop well before the model's time limit names no cause: STOPSTATUS does
// not tell an iteration limit from an interrupt.
TEST_F(xpress_lp_iis_test, iteration_limit_stop_names_no_cause) {
    auto model = this->new_model();
    iis_cases::build(model, iis_cases::four_row_conflict_case());
    limit_lp_iterations_to_one(model);
    const auto iis = model.compute_iis();
    EXPECT_TRUE(outcome_is<iis_outcome::stopped>(iis.get_outcome(), false));
    EXPECT_EQ(iis.num_constraint_members(), 0u);
    EXPECT_TRUE(is<status::unknown>(model.get_status()));
}

TEST_F(xpress_milp_iis_test, iteration_limit_stop_names_no_cause) {
    auto model = this->new_model();
    add_market_split_conflict(model);
    limit_lp_iterations_to_one(model);
    const auto iis = model.compute_iis();
    EXPECT_TRUE(outcome_is<iis_outcome::stopped>(iis.get_outcome(), false));
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
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
    EXPECT_TRUE(is<iis_status::member_both>(iis.get_status(x)));
    EXPECT_TRUE(is<iis_status::absent>(iis.get_status(y)));
    EXPECT_TRUE(is<iis_status::member_both>(iis.get_status(r)));
    EXPECT_EQ(iis.num_variable_members(), 1u);
    EXPECT_EQ(iis.num_constraint_members(), 1u);
}

// Without integer columns the routine names the side of a ranged row, and
// xpress_milp keeps it, as xpress_lp does.
TEST_F(xpress_milp_iis_test, ranged_row_keeps_its_side_without_integers) {
    using namespace operators;
    auto model = this->new_model();
    auto x = model.add_variable({.lower_bound = 0., .upper_bound = 10.});
    auto y = model.add_variable({.lower_bound = 0., .upper_bound = 10.});
    auto ranged = model.add_constraint(x + y >= 0.);
    model.set_constraint_upper_bound(ranged, 12.);
    auto at_least = model.add_constraint(x + y >= 15.);
    const auto iis = model.compute_iis();
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
    EXPECT_TRUE(is<iis_status::member_upper>(iis.get_status(ranged)));
    EXPECT_TRUE(is<iis_status::member_lower>(iis.get_status(at_least)));
    EXPECT_EQ(iis.num_variable_members(), 0u);
}

TEST_F(xpress_milp_iis_test, market_split_conflict_completes) {
    auto model = this->new_model();
    add_market_split_conflict(model);
    const auto iis = model.compute_iis();
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
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
    EXPECT_TRUE(outcome_is<iis_outcome::time_limit>(iis.get_outcome()));
    EXPECT_EQ(iis_outcome::conflict_available(iis.get_outcome()),
              iis.num_constraint_members() > 0);
    EXPECT_LE(iis.num_constraint_members(), 4u);
    EXPECT_EQ(iis.num_variable_members(), 0u);
    EXPECT_EQ(model.get_time_limit().count(), 0.01);
    EXPECT_TRUE(is<status::unknown>(model.get_status()));
}
