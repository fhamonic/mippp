#include <algorithm>
#include <array>
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
#include <utility>
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
INSTANTIATE_TEST(Xpress_lp, LpBasisTest, xpress_lp_test);
INSTANTIATE_TEST(Xpress_lp, LpBasisWarmStartTest, xpress_lp_test);
INSTANTIATE_TEST(Xpress_lp, VerbosityTest, xpress_lp_test);
INSTANTIATE_TEST(Xpress_lp, ThreadLimitTest, xpress_lp_test);
INSTANTIATE_TEST(Xpress_lp, LpAlgorithmTest, xpress_lp_test);
static_assert(has_lp_basis_warm_start<xpress_lp>);

// xpress_lp has no iteration limit: Xpress's own count proves the warm start
static int simplex_iterations(const xpress_lp & model) {
    int count = -1;
    model.native_api()._check(
        model.native_model(),
        model.native_api().getintattrib(
            model.native_model(), xpress::impl::v1::XPRS_SIMPLEXITER, &count));
    return count;
}
TEST_F(xpress_lp_test, set_basis_starts_at_the_optimum) {
    auto a = new_model();
    lp_basis_cases::build_dense(a);
    a.solve();
    ASSERT_TRUE(is_a<status::optimal>(a.get_status()));
    ASSERT_GT(simplex_iterations(a), 0);
    auto b = new_model();
    lp_basis_cases::build_dense(b);
    b.set_basis(a.get_basis());
    b.solve();
    ASSERT_TRUE(is_a<status::optimal>(b.get_status()));
    ASSERT_EQ(simplex_iterations(b), 0);
    ASSERT_NEAR(b.get_solution_value(), a.get_solution_value(),
                lp_basis_cases::tolerance(a.get_solution_value()));
}
// Xpress repairs at load time a code naming a side the bounds lack, so only a
// boxed entity reads back either side as written.
TEST_F(xpress_lp_test, set_basis_writes_the_side_of_boxed_entities) {
    using namespace operators;
    auto model = new_model();
    const auto x = model.add_variable({.lower_bound = 0.0, .upper_bound = 4.0});
    const auto y = model.add_variable();
    const auto z = model.add_variable();
    const auto r1 = lp_basis_cases::add_ranged_row(model, x + y, 1.0, 3.0);
    const auto r2 = lp_basis_cases::add_ranged_row(model, y + z, 2.0, 8.0);
    model_basis_t<xpress_lp> basis;
    basis.set_status(x, basis_status::nonbasic_at_upper_bound{});
    basis.set_status(y, basis_status::basic{});
    basis.set_status(z, basis_status::basic{});
    basis.set_status(r1, basis_status::nonbasic_at_upper_bound{});
    basis.set_status(r2, basis_status::nonbasic_at_lower_bound{});
    model.set_basis(basis);
    int rows[2] = {}, cols[3] = {};
    model.native_api()._check(
        model.native_model(),
        model.native_api().getbasis(model.native_model(), rows, cols));
    ASSERT_EQ(cols[model.native_id(x)],
              xpress::impl::v1::XPRS_BASISSTATUS_NONBASIC_UPPER);
    ASSERT_EQ(rows[model.native_id(r1)],
              xpress::impl::v1::XPRS_BASISSTATUS_NONBASIC_LOWER);
    ASSERT_EQ(rows[model.native_id(r2)],
              xpress::impl::v1::XPRS_BASISSTATUS_NONBASIC_UPPER);
}

// Xpress returns a basis loaded over a solved model as it was written, which
// checks the codes set_basis() writes without the solve get_basis() needs.
// Xpress codes a row by its slack, so a row at the upper side of its
// activity reads NONBASIC_LOWER.
TEST_F(xpress_lp_test, set_basis_writes_the_codes_over_a_solved_model) {
    using namespace operators;
    using namespace xpress::impl::v1;
    auto model = new_model();
    const auto x = model.add_variable({.lower_bound = 0.0, .upper_bound = 4.0});
    const auto y = model.add_variable();
    const auto z = model.add_variable();
    model.set_minimization();
    model.set_objective(-x + y + z);
    const auto r1 = lp_basis_cases::add_ranged_row(model, x + y, 1.0, 3.0);
    const auto r2 = lp_basis_cases::add_ranged_row(model, y + z, 2.0, 8.0);
    model_basis_t<xpress_lp> basis;
    basis.set_status(x, basis_status::nonbasic_at_upper_bound{});
    basis.set_status(y, basis_status::basic{});
    basis.set_status(z, basis_status::basic{});
    basis.set_status(r1, basis_status::nonbasic_at_upper_bound{});
    basis.set_status(r2, basis_status::nonbasic_at_lower_bound{});
    model.solve();
    ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
    model.set_basis(basis);
    std::array<int, 3> cstat{-1, -1, -1};
    std::array<int, 2> rstat{-1, -1};
    model.native_api()._check(
        model.native_model(),
        model.native_api().getbasis(model.native_model(), rstat.data(),
                                    cstat.data()));
    EXPECT_EQ(cstat, (std::array<int, 3>{XPRS_BASISSTATUS_NONBASIC_UPPER,
                                         XPRS_BASISSTATUS_BASIC,
                                         XPRS_BASISSTATUS_BASIC}));
    EXPECT_EQ(rstat, (std::array<int, 2>{XPRS_BASISSTATUS_NONBASIC_LOWER,
                                         XPRS_BASISSTATUS_NONBASIC_UPPER}));
}
// LPFLAGS names the two simplex sides and the barrier, not "either simplex",
// and the selector is an LP member.
static_assert(has_lp_algorithm<xpress_lp>);
static_assert(has_lp_algorithm<xpress_lp, lp_algorithm::primal_simplex>);
static_assert(has_lp_algorithm<xpress_lp, lp_algorithm::dual_simplex>);
static_assert(has_lp_algorithm<xpress_lp, lp_algorithm::interior_point>);
static_assert(has_lp_algorithm<xpress_lp,
                               lp_algorithm::interior_point_without_crossover>);
static_assert(!has_lp_algorithm<xpress_lp, lp_algorithm::simplex>);
static_assert(!has_lp_algorithm<xpress_lp, lp_algorithm::other>);
static_assert(variant_with_alternative<model_lp_algorithm_t<xpress_lp>,
                                       lp_algorithm::other>);
static_assert(!has_lp_algorithm<xpress_milp>);

namespace {
int xpress_attribute(const xpress_lp & model, int attribute) {
    int value = -1;
    model.native_api()._check(model.native_model(),
                              model.native_api().getintattrib(
                                  model.native_model(), attribute, &value));
    return value;
}
int xpress_control(const xpress_lp & model, int control) {
    int value = -1;
    model.native_api()._check(model.native_model(),
                              model.native_api().getintcontrol(
                                  model.native_model(), control, &value));
    return value;
}
// ALGORITHM: the algorithm Xpress last ran, 2 dual, 3 primal, 4 barrier
constexpr int xprs_algorithm = 1049;
}  // namespace

// The shared suite cannot see which algorithm ran: Xpress's attributes can.
TEST_F(xpress_lp_test, each_lp_algorithm_runs_the_one_it_names) {
    using namespace xpress::impl::v1;
    SkipOnLicenseError([this]() {
        struct run {
            int lp_flags, crossover, algorithm, simplex, barrier;
        };
        const auto solved_under = [this](auto algorithm) {
            auto model = new_model();
            lp_basis_cases::build_dense(model);
            model.set_lp_algorithm(algorithm);
            model.solve();
            EXPECT_TRUE(is_a<status::optimal>(model.get_status()));
            return run{xpress_control(model, XPRS_LPFLAGS),
                       xpress_control(model, XPRS_CROSSOVER),
                       xpress_attribute(model, xprs_algorithm),
                       xpress_attribute(model, XPRS_SIMPLEXITER),
                       xpress_attribute(model, XPRS_BARITER)};
        };
        {
            const run r = solved_under(lp_algorithm::primal_simplex{});
            EXPECT_EQ(r.lp_flags, XPRS_LPFLAGS_PRIMAL);
            EXPECT_EQ(r.algorithm, 3);
            EXPECT_GT(r.simplex, 0);
            EXPECT_EQ(r.barrier, 0);
        }
        {
            const run r = solved_under(lp_algorithm::dual_simplex{});
            EXPECT_EQ(r.lp_flags, XPRS_LPFLAGS_DUAL);
            EXPECT_EQ(r.algorithm, 2);
            EXPECT_GT(r.simplex, 0);
            EXPECT_EQ(r.barrier, 0);
        }
        {
            const run r = solved_under(lp_algorithm::interior_point{});
            EXPECT_EQ(r.lp_flags, XPRS_LPFLAGS_BARRIER);
            EXPECT_EQ(r.crossover, -1);
            EXPECT_EQ(r.algorithm, 4);
            EXPECT_GT(r.barrier, 0);
        }
        {
            const run r =
                solved_under(lp_algorithm::interior_point_without_crossover{});
            EXPECT_EQ(r.lp_flags, XPRS_LPFLAGS_BARRIER);
            EXPECT_EQ(r.crossover, 0);
            EXPECT_EQ(r.algorithm, 4);
            EXPECT_GT(r.barrier, 0);
            EXPECT_EQ(r.simplex, 0);
        }
        {
            const run r = solved_under(lp_algorithm::automatic{});
            EXPECT_EQ(r.lp_flags, 0);
            EXPECT_EQ(r.crossover, -1);
        }
    });
}

// LPFLAGS is a bit set: two algorithms at once run concurrently, which no
// tag names, and a crossover order is not what interior_point writes.
TEST_F(xpress_lp_test, a_native_lp_flags_value_reads_as_other) {
    using namespace xpress::impl::v1;
    SkipOnLicenseError([this]() {
        auto model = new_model();
        const auto & native = model.native_api();
        const auto prob = model.native_model();
        native._check(prob, native.setintcontrol(
                                prob, XPRS_LPFLAGS,
                                XPRS_LPFLAGS_DUAL | XPRS_LPFLAGS_BARRIER));
        EXPECT_TRUE(is<lp_algorithm::other>(model.get_lp_algorithm()));
        native._check(prob, native.setintcontrol(prob, XPRS_LPFLAGS,
                                                 XPRS_LPFLAGS_BARRIER));
        EXPECT_TRUE(is<lp_algorithm::interior_point>(model.get_lp_algorithm()));
        native._check(prob, native.setintcontrol(prob, XPRS_CROSSOVER, 2));
        EXPECT_TRUE(is<lp_algorithm::other>(model.get_lp_algorithm()));
    });
}

// Presolve proves the infeasibility without a simplex run, and Xpress would
// hand back the slack basis, or after an optimal solve that basis, stale.
TEST_F(xpress_lp_test, get_basis_throws_after_presolve_proves_infeasibility) {
    using namespace operators;
    auto fresh = new_model();
    lp_basis_cases::build_infeasible(fresh);
    fresh.solve();
    ASSERT_TRUE(is_a<status::infeasible_or_unbounded>(fresh.get_status()));
    ASSERT_THROW(fresh.get_basis(), solver_error);

    auto solved = new_model();
    const auto x = solved.add_variable();
    const auto y = solved.add_variable();
    solved.set_maximization();
    solved.set_objective(x + y);
    solved.add_constraint(x + 2 * y <= 3);
    solved.solve();
    ASSERT_TRUE(is_a<status::optimal>(solved.get_status()));
    solved.add_constraint(x + y >= 5);
    solved.solve();
    ASSERT_TRUE(is_a<status::infeasible_or_unbounded>(solved.get_status()));
    ASSERT_THROW(solved.get_basis(), solver_error);
}

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
// the suite repairs candidates that a lazy constraint cut off
// INSTANTIATE_TEST(Xpress_milp, SolutionInjectionTest, xpress_milp_test);
// not wired yet: the cut-round callback with managed cuts, which do nothing
// before Xpress 45.01.04
// INSTANTIATE_TEST(Xpress_milp, NodeRelaxationCallbackTest, xpress_milp_test);
// INSTANTIATE_TEST(Xpress_milp, UserCutsTest, xpress_milp_test);
INSTANTIATE_TEST(Xpress_milp, SudokuTest, xpress_milp_test);
INSTANTIATE_TEST(Xpress_milp, TimeLimitTest, xpress_milp_test);
INSTANTIATE_TEST(Xpress_milp, TimeLimitIncumbentTest, xpress_milp_test);
INSTANTIATE_TEST(Xpress_milp, MipStartTest, xpress_milp_test);
INSTANTIATE_TEST(Xpress_milp, OptimalityToleranceTest, xpress_milp_test);
INSTANTIATE_TEST(Xpress_milp, MipGapTest, xpress_milp_test);
INSTANTIATE_TEST(Xpress_milp, DualBoundTest, xpress_milp_test);
INSTANTIATE_TEST(Xpress_milp, NodeCountTest, xpress_milp_test);
INSTANTIATE_TEST(Xpress_milp, IntegralityToleranceTest, xpress_milp_test);
INSTANTIATE_TEST(Xpress_milp, VerbosityTest, xpress_milp_test);
INSTANTIATE_TEST(Xpress_milp, ThreadLimitTest, xpress_milp_test);
INSTANTIATE_TEST(Xpress_milp, ProgressCallbackTest, xpress_milp_test);
INSTANTIATE_TEST(Xpress_milp, ProgressCallbackNodeCountTest, xpress_milp_test);
INSTANTIATE_TEST(Xpress_milp, ProgressCallbackSharingTest, xpress_milp_test);
static_assert(!has_lp_basis<xpress_milp>);
// The code set_basis writes for each tag the statuses list, pinned without a
// library: a tag that loses its line in the table writes as its nearest
// base, which still compiles.
struct xpress_basis_codes : xpress_lp {
    using column_status = model_variable_basis_status_t<xpress_lp>;
    using row_status = model_constraint_basis_status_t<xpress_lp>;
    template <typename Tag>
    static constexpr auto column() {
        return _basis_code<_column_at_lower, _column_at_upper>(
            column_status(std::in_place_type<Tag>));
    }
    template <typename Tag>
    static constexpr auto row() {
        return _basis_code<_row_at_lower, _row_at_upper>(
            row_status(std::in_place_type<Tag>));
    }
};
TEST(xpress_lp_basis_codes, one_per_listed_tag) {
    using namespace xpress::impl::v1;
    using namespace basis_status;
    using codes = xpress_basis_codes;
    // clang-format off
    static_assert(std::variant_size_v<codes::column_status> == 5);
    static_assert(std::variant_size_v<codes::row_status>    == 5);
    static_assert(codes::column<basic>()                   == XPRS_BASISSTATUS_BASIC);
    static_assert(codes::column<nonbasic_at_lower_bound>() == XPRS_BASISSTATUS_NONBASIC_LOWER);
    static_assert(codes::column<nonbasic_at_upper_bound>() == XPRS_BASISSTATUS_NONBASIC_UPPER);
    static_assert(codes::column<superbasic>()              == XPRS_BASISSTATUS_SUPERBASIC);
    static_assert(codes::column<nonbasic>()                == XPRS_BASISSTATUS_NONBASIC_LOWER);
    // a row's code names the side of its slack, opposite its activity's
    static_assert(codes::row<basic>()                   == XPRS_BASISSTATUS_BASIC);
    static_assert(codes::row<nonbasic_at_lower_bound>() == XPRS_BASISSTATUS_NONBASIC_UPPER);
    static_assert(codes::row<nonbasic_at_upper_bound>() == XPRS_BASISSTATUS_NONBASIC_LOWER);
    static_assert(codes::row<superbasic>()              == XPRS_BASISSTATUS_SUPERBASIC);
    static_assert(codes::row<nonbasic>()                == XPRS_BASISSTATUS_NONBASIC_LOWER);
    // clang-format on
}

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
