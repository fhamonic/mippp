#include <chrono>
#include <cmath>
#include <limits>
#include <tuple>
#include <utility>
#include <variant>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/solvers/glpk/all.hpp"

using namespace mippp;

#include "test_suites/all.hpp"

MIPPP_API_VERSION_TEST(GLPK_api, glpk_api, "GLPK")

namespace {
struct glpk_lp_status_probe : glpk::impl::v1::glpk_lp {
    using glpk_lp::_simplex_status;
};
struct glpk_milp_status_probe : glpk::impl::v1::glpk_milp {
    using glpk_milp::_intopt_status;
};

auto simplex_status(int ret, int generic_status, int primal_status,
                    double objective = 1.0,
                    int dual_status = glpk::impl::v1::GLP_INFEAS) {
    return glpk_lp_status_probe::_simplex_status(
        ret, generic_status, primal_status, dual_status, objective);
}
auto intopt_status(int ret, int mip_status) {
    return glpk_milp_status_probe::_intopt_status(ret, mip_status);
}

// An IEEE infinity on the side it cannot free is a real bound that no point
// satisfies.
template <typename NewModel>
void check_sides_at_the_wrong_infinity_are_infeasible(NewModel new_model) {
    using namespace operators;
    constexpr double inf = std::numeric_limits<double>::infinity();
    {
        auto model = new_model();
        auto x = model.add_variable({.lower_bound = 0, .upper_bound = 4});
        auto c = model.add_constraint(x <= 3);
        model.set_constraint_upper_bound(c, -inf);
        model.set_objective(x);
        model.solve();
        EXPECT_TRUE(is<status::infeasible>(model.get_status()));
    }
    for(const bool minimize : {true, false}) {
        auto model = new_model();
        auto x = model.add_variable({.lower_bound = 0, .upper_bound = 4});
        auto c = model.add_constraint(x >= 1);
        model.set_constraint_lower_bound(c, inf);
        if(minimize)
            model.set_minimization();
        else
            model.set_maximization();
        model.set_objective(x);
        model.solve();
        EXPECT_TRUE(is<status::infeasible>(model.get_status())) << minimize;
    }
    {
        auto model = new_model();
        auto x = model.add_variable({.lower_bound = 0, .upper_bound = 4});
        model.add_constraint(x <= 3);
        model.set_variable_upper_bound(x, -inf);
        model.set_objective(x);
        model.solve();
        EXPECT_TRUE(is<status::infeasible>(model.get_status()));
    }
}
// Typed double-bounded, a column added with bounds at -/+infinity() is
// bounded at -/+DBL_MAX, where glp_simplex stops at an optimum of -1.8e308.
template <typename NewModel>
void check_infinite_column_bounds_are_free(NewModel new_model) {
    using namespace operators;
    auto model = new_model();
    const double inf = model.infinity();
    auto x = model.add_variable({.lower_bound = 1, .upper_bound = inf});
    auto y = model.add_variable({.lower_bound = -5, .upper_bound = 5});
    auto z = model.add_variable({.lower_bound = -inf, .upper_bound = inf});
    auto c = model.add_constraint(x + 3 * y + 3 * z <= 3);
    model.set_constraint_lower_bound(c, 2);
    model.set_objective(3 * x + y + z);
    model.solve();
    ASSERT_TRUE(is<status::optimal>(model.get_status()));
    EXPECT_NEAR(model.get_solution_value(), 10.0 / 3.0, TEST_EPSILON);
}
}  // namespace

TEST(GLPK_simplex_status, completed_search_reads_the_basic_solution) {
    using namespace glpk::impl::v1;
    EXPECT_TRUE(is<status::optimal>(simplex_status(0, GLP_OPT, GLP_FEAS)));
    EXPECT_TRUE(
        is<status::infeasible>(simplex_status(0, GLP_NOFEAS, GLP_NOFEAS)));
    EXPECT_TRUE(is<status::unbounded>(simplex_status(0, GLP_UNBND, GLP_FEAS)));
    EXPECT_TRUE(is<status::unbounded>(simplex_status(
        0, GLP_OPT, GLP_FEAS, std::numeric_limits<double>::infinity())));
    const auto feasible = simplex_status(0, GLP_FEAS, GLP_FEAS);
    EXPECT_TRUE(is<status::unknown>(feasible));
    EXPECT_TRUE(status::solution_available(feasible));
}
// The dual simplex stops on a primal infeasible basis when the dual has no
// feasible point: the LP has no optimum, and which of the two it is stays
// open. The presolver's own "no dual feasible solution" says the same.
TEST(GLPK_simplex_status, a_dual_without_a_point_leaves_the_lp_undecided) {
    using namespace glpk::impl::v1;
    EXPECT_TRUE(is<status::infeasible_or_unbounded>(
        simplex_status(0, GLP_INFEAS, GLP_INFEAS, 1.0, GLP_NOFEAS)));
    EXPECT_TRUE(is<status::infeasible_or_unbounded>(
        simplex_status(0, GLP_UNDEF, GLP_UNDEF, 1.0, GLP_NOFEAS)));
    EXPECT_TRUE(is<status::infeasible_or_unbounded>(
        simplex_status(GLP_ENODFS, GLP_UNDEF, GLP_UNDEF)));
    // with a feasible point in hand GLPK itself says unbounded
    EXPECT_TRUE(is<status::unbounded>(
        simplex_status(0, GLP_UNBND, GLP_FEAS, 1.0, GLP_NOFEAS)));
    // a stop is no verdict, whatever the dual status reads
    EXPECT_FALSE(is_a<status::completed>(
        simplex_status(GLP_EITLIM, GLP_INFEAS, GLP_INFEAS, 1.0, GLP_NOFEAS)));
}
TEST(GLPK_simplex_status, infeasible_basic_solution_proves_nothing) {
    using namespace glpk::impl::v1;
    for(int ret : {0, GLP_EITLIM, GLP_ETMLIM, GLP_EOBJLL, GLP_EOBJUL}) {
        const auto r = simplex_status(ret, GLP_INFEAS, GLP_INFEAS);
        EXPECT_FALSE(is_a<status::completed>(r)) << ret;
        EXPECT_FALSE(status::solution_available(r)) << ret;
    }
}
TEST(GLPK_simplex_status, limits_stop_even_on_an_optimal_basis) {
    using namespace glpk::impl::v1;
    for(int ret : {GLP_EITLIM, GLP_EOBJLL, GLP_EOBJUL}) {
        const auto r = simplex_status(ret, GLP_OPT, GLP_FEAS);
        EXPECT_TRUE(is<status::limit_reached>(r)) << ret;
        EXPECT_TRUE(status::solution_available(r)) << ret;
    }
    const auto timed_out = simplex_status(GLP_ETMLIM, GLP_OPT, GLP_FEAS);
    EXPECT_TRUE(is<status::time_limit>(timed_out));
    EXPECT_TRUE(status::solution_available(timed_out));
}
TEST(GLPK_simplex_status, errors_are_failures) {
    using namespace glpk::impl::v1;
    for(int ret : {GLP_EBADB, GLP_EBOUND, GLP_EFAIL})
        EXPECT_TRUE(
            is<status::failed>(simplex_status(ret, GLP_UNDEF, GLP_UNDEF)))
            << ret;
    for(int ret : {GLP_ESING, GLP_ECOND})
        EXPECT_TRUE(is<status::numerical_failure>(
            simplex_status(ret, GLP_UNDEF, GLP_UNDEF)))
            << ret;
}
TEST(GLPK_simplex_status, presolver_infeasibility_is_kept) {
    using namespace glpk::impl::v1;
    EXPECT_TRUE(is<status::infeasible>(
        simplex_status(GLP_ENOPFS, GLP_UNDEF, GLP_UNDEF)));
}

TEST(GLPK_intopt_status, dual_infeasible_relaxation_leaves_the_mip_undecided) {
    using namespace glpk::impl::v1;
    EXPECT_TRUE(is<status::infeasible_or_unbounded>(
        intopt_status(GLP_ENODFS, GLP_UNDEF)));
}
TEST(GLPK_intopt_status, stops_carry_the_incumbent) {
    using namespace glpk::impl::v1;
    for(int mip_status : {GLP_UNDEF, GLP_FEAS}) {
        const bool has_sol = (mip_status == GLP_FEAS);
        for(int ret : {GLP_EBOUND, GLP_EROOT, GLP_EFAIL}) {
            const auto r = intopt_status(ret, mip_status);
            EXPECT_TRUE(is<status::failed>(r)) << ret;
            EXPECT_EQ(status::solution_available(r), has_sol) << ret;
        }
        const auto timed_out = intopt_status(GLP_ETMLIM, mip_status);
        EXPECT_TRUE(is<status::time_limit>(timed_out));
        EXPECT_EQ(status::solution_available(timed_out), has_sol);
        const auto stopped = intopt_status(GLP_ESTOP, mip_status);
        EXPECT_TRUE(is<status::interrupted>(stopped));
        EXPECT_EQ(status::solution_available(stopped), has_sol);
    }
}
TEST(GLPK_intopt_status, completed_search_reads_the_mip_status) {
    using namespace glpk::impl::v1;
    EXPECT_TRUE(is<status::optimal>(intopt_status(0, GLP_OPT)));
    EXPECT_TRUE(is<status::infeasible>(intopt_status(0, GLP_NOFEAS)));
    EXPECT_TRUE(is<status::optimal>(intopt_status(GLP_EMIPGAP, GLP_FEAS)));
    EXPECT_TRUE(is<status::infeasible>(intopt_status(GLP_ENOPFS, GLP_NOFEAS)));
}

struct glpk_lp_test : public model_test<glpk_api, glpk_lp> {
    static void SetUpTestSuite() { construct_api("GLPK"); }
};
static_assert(!has_iis<glpk_lp>);
static_assert(has_lp_basis_warm_start<glpk_lp>);
// glp_simplex has the two simplex sides; glp_interior is another routine,
// with a solution store of its own and no basis.
static_assert(has_lp_algorithm<glpk_lp>);
static_assert(has_lp_algorithm<glpk_lp, lp_algorithm::primal_simplex>);
static_assert(has_lp_algorithm<glpk_lp, lp_algorithm::dual_simplex>);
static_assert(!has_lp_algorithm<glpk_lp, lp_algorithm::simplex>);
static_assert(!has_lp_algorithm<glpk_lp, lp_algorithm::interior_point>);
static_assert(
    !has_lp_algorithm<glpk_lp, lp_algorithm::interior_point_without_crossover>);
// the model holds the choice itself, so no native setting can read as other
static_assert(!variant_with_alternative<model_lp_algorithm_t<glpk_lp>,
                                        lp_algorithm::other>);
static_assert(has_refinable_lp_status<glpk_lp>);

namespace {
// max x under x - y >= 1 over x, y >= 0: unbounded along x, from a slack
// basis that is neither primal nor dual feasible
void build_unbounded_from_an_infeasible_basis(glpk_lp & model) {
    using namespace operators;
    const auto x = model.add_variable();
    const auto y = model.add_variable();
    model.set_maximization();
    model.set_objective(x);
    model.add_constraint(x - y >= 1);
}
}  // namespace

// The shared suite cannot see which algorithm ran. On this LP GLPK's two
// sides end apart: the primal simplex reaches a feasible point and proves
// the LP unbounded, the dual one stops where the dual has no point, and
// refine_lp_status() then decides by the primal simplex without changing
// the model's algorithm.
TEST_F(glpk_lp_test, each_lp_algorithm_runs_the_one_it_names) {
    for(const bool automatic : {true, false}) {
        auto model = new_model();
        build_unbounded_from_an_infeasible_basis(model);
        if(!automatic) model.set_lp_algorithm(lp_algorithm::primal_simplex{});
        model.solve();
        EXPECT_TRUE(is<status::unbounded>(model.get_status())) << automatic;
    }
    auto model = new_model();
    build_unbounded_from_an_infeasible_basis(model);
    model.set_lp_algorithm(lp_algorithm::dual_simplex{});
    model.solve();
    ASSERT_TRUE(is<status::infeasible_or_unbounded>(model.get_status()));
    model.refine_lp_status();
    EXPECT_TRUE(is<status::unbounded>(model.get_status()));
    EXPECT_TRUE(is<lp_algorithm::dual_simplex>(model.get_lp_algorithm()));
}

// On an LP with an optimum both sides agree, by different paths.
TEST_F(glpk_lp_test, the_two_simplex_sides_take_different_paths) {
    const auto iterations_under = [this](auto algorithm) {
        auto model = new_model();
        lp_algorithm_cases::build_dense_covering(model);
        model.set_lp_algorithm(algorithm);
        model.solve();
        EXPECT_TRUE(is_a<status::optimal>(model.get_status()));
        return api->get_it_cnt(model.native_model());
    };
    const int primal = iterations_under(lp_algorithm::primal_simplex{});
    EXPECT_EQ(iterations_under(lp_algorithm::automatic{}), primal);
    // from a dual feasible basis the dual simplex needs no first phase
    EXPECT_LT(iterations_under(lp_algorithm::dual_simplex{}), primal);
}

// GLPK refuses to start on crossed bounds with GLP_EBOUND, and the crossing
// alone proves infeasibility
TEST_F(glpk_lp_test, crossing_variable_bounds_are_infeasible) {
    using namespace operators;
    auto model = new_model();
    auto x = model.add_variable({.lower_bound = 2, .upper_bound = 1});
    model.add_constraint(x <= 5);
    model.set_objective(x);
    model.solve();
    EXPECT_TRUE(is<status::infeasible>(model.get_status()));
}
TEST_F(glpk_lp_test, crossing_row_sides_are_infeasible) {
    using namespace operators;
    auto model = new_model();
    auto x = model.add_variable({.lower_bound = 0, .upper_bound = 4});
    auto c = model.add_constraint(x >= 2);
    model.set_constraint_upper_bound(c, 1);
    model.set_objective(x);
    model.solve();
    EXPECT_TRUE(is<status::infeasible>(model.get_status()));
}
// GLP_EBOUND also refuses a double-bounded column with lb == ub, a fixed
// column that proves nothing; only the native handle can write one
TEST_F(glpk_lp_test, equal_double_bounds_written_natively_fail) {
    using namespace operators;
    using namespace glpk::impl::v1;
    auto model = new_model();
    auto x = model.add_variable({.lower_bound = 0, .upper_bound = 4});
    model.add_constraint(x <= 5);
    model.set_objective(x);
    model.native_api().set_col_bnds(model.native_model(), model.native_id(x),
                                    GLP_DB, 1, 1);
    model.solve();
    EXPECT_TRUE(is<status::failed>(model.get_status()));
}
TEST_F(glpk_lp_test, sides_at_the_wrong_infinity_are_infeasible) {
    check_sides_at_the_wrong_infinity_are_infeasible(
        [this] { return new_model(); });
}
TEST_F(glpk_lp_test, infinite_column_bounds_are_free) {
    check_infinite_column_bounds_are_free([this] { return new_model(); });
}
// glpk_lp has no iteration limit: GLPK's own count proves the warm start
TEST_F(glpk_lp_test, set_basis_starts_at_the_optimum) {
    auto a = new_model();
    lp_basis_cases::build_dense(a);
    a.solve();
    ASSERT_TRUE(is_a<status::optimal>(a.get_status()));
    ASSERT_GT(api->get_it_cnt(a.native_model()), 0);
    auto b = new_model();
    lp_basis_cases::build_dense(b);
    b.set_basis(a.get_basis());
    b.solve();
    ASSERT_TRUE(is_a<status::optimal>(b.get_status()));
    ASSERT_EQ(api->get_it_cnt(b.native_model()), 0);
    ASSERT_NEAR(b.get_solution_value(), a.get_solution_value(),
                lp_basis_cases::tolerance(a.get_solution_value()));
}
namespace {
// GLPK's code for an entity, read past get_basis(), which reads the basis of
// a solve only: the tests below check what set_basis() wrote.
int glpk_code(glpk_lp & model, model_variable_t<glpk_lp> v) {
    return glpk_lp_test::api->get_col_stat(model.native_model(),
                                           model.native_id(v));
}
int glpk_code(glpk_lp & model, model_constraint_t<glpk_lp> c) {
    return glpk_lp_test::api->get_row_stat(model.native_model(),
                                           model.native_id(c));
}
}  // namespace
// GLPK rewrites a nonbasic code to the one the entity's type implies, so NL
// and NU stay apart only on a double-bounded entity, x1 here
TEST_F(glpk_lp_test, set_basis_writes_the_codes_before_any_solve) {
    using namespace glpk::impl::v1;
    auto a = new_model();
    lp_basis_cases::build_vertex(a);
    a.solve();
    ASSERT_TRUE(is_a<status::optimal>(a.get_status()));
    auto b = new_model();
    const auto v = lp_basis_cases::build_vertex(b);
    b.set_basis(a.get_basis());
    EXPECT_EQ(glpk_code(b, v.x1), GLP_NU);
    EXPECT_EQ(glpk_code(b, v.x2), GLP_NS);
    EXPECT_EQ(glpk_code(b, v.x3), GLP_BS);
    EXPECT_EQ(glpk_code(b, v.x4), GLP_BS);
    EXPECT_EQ(glpk_code(b, v.x5), GLP_BS);
    EXPECT_EQ(glpk_code(b, v.w), GLP_NL);
    EXPECT_EQ(glpk_code(b, v.f), GLP_NF);
    EXPECT_EQ(glpk_code(b, v.le), GLP_NU);
    EXPECT_EQ(glpk_code(b, v.ge), GLP_NL);
    EXPECT_EQ(glpk_code(b, v.eq), GLP_NS);
    EXPECT_EQ(glpk_code(b, v.slack), GLP_BS);
    auto source = a.get_basis();
    source.set_status(v.x1, basis_status::nonbasic_at_lower_bound{});
    b.set_basis(source);
    EXPECT_EQ(glpk_code(b, v.x1), GLP_NL);
    source.set_status(v.x1, basis_status::nonbasic{});
    b.set_basis(source);
    EXPECT_EQ(glpk_code(b, v.x1), GLP_NL);
    // superbasic, which glpk_lp's snapshot does not list, writes as bare
    // nonbasic: NL, NF on the free f
    basis_snapshot<model_variable_t<glpk_lp>, model_constraint_t<glpk_lp>,
                   std::variant<basis_status::nonbasic, basis_status::basic,
                                basis_status::superbasic>,
                   std::variant<basis_status::basic, basis_status::nonbasic>>
        foreign;
    foreign.set_status(v.x1, basis_status::superbasic{});
    foreign.set_status(v.f, basis_status::superbasic{});
    b.set_basis(foreign);
    EXPECT_EQ(glpk_code(b, v.x1), GLP_NL);
    EXPECT_EQ(glpk_code(b, v.f), GLP_NF);
}
namespace {
// LpBasisTest's ranged rows: r1 is tight at its upper side, r2 at its lower
auto build_ranged(glpk_lp & model) {
    using namespace operators;
    const auto x =
        model.add_variable(lp_basis_cases::bounds<glpk_lp>(0.0, 10.0));
    const auto y = model.add_variable();
    const auto z = model.add_variable();
    model.set_minimization();
    model.set_objective(-x + y + z);
    const auto r1 = lp_basis_cases::add_ranged_row(model, x + y, 1.0, 3.0);
    const auto r2 = lp_basis_cases::add_ranged_row(model, z + y, 2.0, 8.0);
    return std::tuple{x, y, z, r1, r2};
}
}  // namespace
TEST_F(glpk_lp_test, set_basis_writes_a_ranged_row_before_any_solve) {
    using namespace glpk::impl::v1;
    auto a = new_model();
    build_ranged(a);
    a.solve();
    ASSERT_TRUE(is_a<status::optimal>(a.get_status()));
    auto b = new_model();
    const auto [x, y, z, r1, r2] = build_ranged(b);
    b.set_basis(a.get_basis());
    EXPECT_EQ(glpk_code(b, x), GLP_BS);
    EXPECT_EQ(glpk_code(b, y), GLP_NL);
    EXPECT_EQ(glpk_code(b, z), GLP_BS);
    EXPECT_EQ(glpk_code(b, r1), GLP_NU);
    EXPECT_EQ(glpk_code(b, r2), GLP_NL);
    b.solve();
    ASSERT_TRUE(is_a<status::optimal>(b.get_status()));
    ASSERT_EQ(api->get_it_cnt(b.native_model()), 0);
}
// f is in no row. Basic beside the auxiliaries of le, ge and eq, it makes
// GLPK's factorizer abort the process (bflib/sgf.c:185) instead of reporting
// GLP_ESING; not every basis holding f does.
TEST_F(glpk_lp_test, set_basis_never_writes_a_basic_column_in_no_row) {
    auto model = new_model();
    const auto v = lp_basis_cases::build_vertex(model);
    model.solve();
    model_basis_t<glpk_lp> basis;
    basis.set_status(v.f, basis_status::basic{});
    basis.set_status(v.slack, basis_status::nonbasic_at_upper_bound{});
    model.set_basis(basis);
    ASSERT_NE(glpk_code(model, v.f), glpk::impl::v1::GLP_BS);
    model.solve();
    ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
    ASSERT_NEAR(model.get_solution_value(), -8.0, TEST_EPSILON);
}
TEST_F(glpk_lp_test, set_basis_replaces_a_basis_glpk_cannot_factorize) {
    const auto check = [this](auto make_basis) {
        auto model = new_model();
        const auto v = lp_basis_cases::build_vertex(model);
        model.solve();
        model.set_basis(make_basis(v));
        for(const auto x : {v.x1, v.x2, v.x3, v.x4, v.x5, v.w, v.f})
            ASSERT_NE(glpk_code(model, x), glpk::impl::v1::GLP_BS);
        for(const auto c : {v.le, v.ge, v.eq, v.slack})
            ASSERT_EQ(glpk_code(model, c), glpk::impl::v1::GLP_BS);
        model.solve();
        ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
        ASSERT_NEAR(model.get_solution_value(), -8.0, TEST_EPSILON);
    };
    // every row is basic by default: w makes one basic entity too many
    check([](const auto & v) {
        model_basis_t<glpk_lp> basis;
        basis.set_status(v.w, basis_status::basic{});
        return basis;
    });
    // the right count, but singular: x2 is in row eq only, as eq's auxiliary
    // variable is
    check([](const auto & v) {
        model_basis_t<glpk_lp> basis;
        basis.set_status(v.x2, basis_status::basic{});
        basis.set_status(v.slack, basis_status::nonbasic_at_upper_bound{});
        return basis;
    });
}
INSTANTIATE_TEST(GLPK_lp, LpModelTest, glpk_lp_test);
INSTANTIATE_TEST(GLPK_lp, EnumerableEntitiesTest, glpk_lp_test);
INSTANTIATE_TEST(GLPK_lp, ReadableObjectiveTest, glpk_lp_test);
INSTANTIATE_TEST(GLPK_lp, ModifiableObjectiveTest, glpk_lp_test);
INSTANTIATE_TEST(GLPK_lp, ReadableVariablesBoundsTest, glpk_lp_test);
INSTANTIATE_TEST(GLPK_lp, ModifiableVariablesBoundsTest, glpk_lp_test);
INSTANTIATE_TEST(GLPK_lp, NamedVariablesTest, glpk_lp_test);
INSTANTIATE_TEST(GLPK_lp, AddColumnTest, glpk_lp_test);
INSTANTIATE_TEST(GLPK_lp, ReadableConstraintBoundsTest, glpk_lp_test);
INSTANTIATE_TEST(GLPK_lp, ModifiableConstraintBoundsTest, glpk_lp_test);
INSTANTIATE_TEST(GLPK_lp, IisByDeletionTest, glpk_lp_test);
INSTANTIATE_TEST(GLPK_lp, DualSolutionTest, glpk_lp_test);
INSTANTIATE_TEST(GLPK_lp, ReducedCostsTest, glpk_lp_test);
INSTANTIATE_TEST(GLPK_lp, LpStatusTest, glpk_lp_test);
INSTANTIATE_TEST(GLPK_lp, CuttingStockTest, glpk_lp_test);
INSTANTIATE_TEST(GLPK_lp, VerbosityTest, glpk_lp_test);
INSTANTIATE_TEST(GLPK_lp, TimeLimitTest, glpk_lp_test);
INSTANTIATE_TEST(GLPK_lp, LpBasisTest, glpk_lp_test);
INSTANTIATE_TEST(GLPK_lp, LpBasisWarmStartTest, glpk_lp_test);
INSTANTIATE_TEST(GLPK_lp, LpAlgorithmTest, glpk_lp_test);

struct glpk_milp_test : public model_test<glpk_api, glpk_milp> {
    static void SetUpTestSuite() { construct_api("GLPK"); }
};
static_assert(!has_iis<glpk_milp>);
static_assert(!has_lp_basis<glpk_milp>);
static_assert(!has_dual_bound<glpk_milp>);
static_assert(!has_node_count<glpk_milp>);
// The code set_basis writes for each tag the statuses list, pinned without a
// library: a tag that loses its line in the table writes as its nearest
// base, which still compiles.
struct glpk_basis_codes : glpk_lp {
    using column_status = model_variable_basis_status_t<glpk_lp>;
    using row_status = model_constraint_basis_status_t<glpk_lp>;
    template <typename Tag>
    static constexpr auto column() {
        return _basis_code(column_status(std::in_place_type<Tag>));
    }
    template <typename Tag>
    static constexpr auto row() {
        return _basis_code(row_status(std::in_place_type<Tag>));
    }
};
TEST(glpk_lp_basis_codes, one_per_listed_tag) {
    using namespace glpk::impl::v1;
    using namespace basis_status;
    using codes = glpk_basis_codes;
    // clang-format off
    static_assert(std::variant_size_v<codes::column_status> == 6);
    static_assert(std::variant_size_v<codes::row_status>    == 6);
    static_assert(codes::column<basic>()                   == GLP_BS);
    static_assert(codes::column<nonbasic_at_lower_bound>() == GLP_NL);
    static_assert(codes::column<nonbasic_at_upper_bound>() == GLP_NU);
    static_assert(codes::column<nonbasic_fixed>()          == GLP_NS);
    static_assert(codes::column<nonbasic_free>()           == GLP_NF);
    static_assert(codes::column<nonbasic>()                == GLP_NL);
    static_assert(codes::row<basic>()                   == GLP_BS);
    static_assert(codes::row<nonbasic_at_lower_bound>() == GLP_NL);
    static_assert(codes::row<nonbasic_at_upper_bound>() == GLP_NU);
    static_assert(codes::row<nonbasic_fixed>()          == GLP_NS);
    static_assert(codes::row<nonbasic_free>()           == GLP_NF);
    static_assert(codes::row<nonbasic>()                == GLP_NL);
    // clang-format on
}
TEST_F(glpk_milp_test, crossing_variable_bounds_are_infeasible) {
    using namespace operators;
    auto model = new_model();
    auto x = model.add_variable({.lower_bound = 2, .upper_bound = 1});
    auto y = model.add_integer_variable({.lower_bound = 0, .upper_bound = 3});
    model.add_constraint(x + y <= 5);
    model.set_objective(x + y);
    model.solve();
    EXPECT_TRUE(is<status::infeasible>(model.get_status()));
}
TEST_F(glpk_milp_test, crossing_row_sides_are_infeasible) {
    using namespace operators;
    auto model = new_model();
    auto x = model.add_integer_variable({.lower_bound = 0, .upper_bound = 4});
    auto c = model.add_constraint(x >= 2);
    model.set_constraint_upper_bound(c, 1);
    model.set_objective(x);
    model.solve();
    EXPECT_TRUE(is<status::infeasible>(model.get_status()));
}
TEST_F(glpk_milp_test, equal_double_bounds_written_natively_fail) {
    using namespace operators;
    using namespace glpk::impl::v1;
    auto model = new_model();
    auto x = model.add_integer_variable({.lower_bound = 0, .upper_bound = 4});
    model.add_constraint(x <= 5);
    model.set_objective(x);
    model.native_api().set_col_bnds(model.native_model(), model.native_id(x),
                                    GLP_DB, 1, 1);
    model.solve();
    EXPECT_TRUE(is<status::failed>(model.get_status()));
}
// glp_intopt refuses fractional bounds on an integer column with GLP_EBOUND
TEST_F(glpk_milp_test, fractional_bounds_of_integer_columns_are_rounded) {
    using namespace operators;
    auto model = new_model();
    auto x =
        model.add_integer_variable({.lower_bound = 0.5, .upper_bound = 2.5});
    auto y = model.add_integer_variable({.upper_bound = 1.5});
    auto z = model.add_integer_variable({.lower_bound = 1.0 + 1e-9});
    model.add_constraint(x + y + z <= 10);
    model.set_maximization();
    model.set_objective(x + y - z);
    model.solve();
    ASSERT_TRUE(is<status::optimal>(model.get_status()));
    auto solution = model.get_solution();
    EXPECT_NEAR(solution[x], 2.0, TEST_EPSILON);
    EXPECT_NEAR(solution[y], 1.0, TEST_EPSILON);
    // within the integrality tolerance, the noise does not cut off 1
    EXPECT_NEAR(solution[z], 1.0, TEST_EPSILON);
    EXPECT_NEAR(model.get_solution_value(), 2.0, TEST_EPSILON);
    model.set_minimization();
    model.set_objective(x);
    model.solve();
    ASSERT_TRUE(is<status::optimal>(model.get_status()));
    EXPECT_NEAR(model.get_solution()[x], 1.0, TEST_EPSILON);
    EXPECT_EQ(model.get_variable_lower_bound(x), 0.5);
    EXPECT_EQ(model.get_variable_upper_bound(x), 2.5);
    EXPECT_TRUE(model.is_infinite(model.get_variable_lower_bound(y)));
    EXPECT_EQ(model.get_variable_upper_bound(y), 1.5);
    EXPECT_EQ(model.get_variable_lower_bound(z), 1.0 + 1e-9);
}
TEST_F(glpk_milp_test, integer_column_without_integer_in_its_bounds) {
    using namespace operators;
    for(auto [lb, ub] : {std::pair{0.25, 0.75}, std::pair{0.5, 0.5}}) {
        auto model = new_model();
        auto x =
            model.add_integer_variable({.lower_bound = lb, .upper_bound = ub});
        model.add_constraint(x <= 5);
        model.set_objective(x);
        model.solve();
        EXPECT_TRUE(is<status::infeasible>(model.get_status())) << lb;
        EXPECT_EQ(model.get_variable_lower_bound(x), lb);
        EXPECT_EQ(model.get_variable_upper_bound(x), ub);
    }
}
// Unrounded, x + y == 1.5 over free integers keeps glp_intopt branching
// without end: every branch leaves the other column fractional
TEST_F(glpk_milp_test, integral_rows_are_rounded) {
    using namespace operators;
    {
        auto model = new_model();
        auto x = model.add_integer_variable({});
        auto y = model.add_integer_variable({});
        auto c = model.add_constraint(x + y == 1.5);
        model.solve();
        EXPECT_TRUE(is<status::infeasible>(model.get_status()));
        EXPECT_EQ(model.get_constraint_lower_bound(c), 1.5);
        EXPECT_EQ(model.get_constraint_upper_bound(c), 1.5);
    }
    {
        auto model = new_model();
        auto x = model.add_integer_variable();
        auto y = model.add_integer_variable();
        auto z = model.add_variable({.upper_bound = 0.5});
        auto c = model.add_constraint(x + 2 * y <= 3.5);
        model.add_constraint(x + y + z <= 3.5);
        model.add_constraint(0.5 * x <= 1.75);
        model.set_maximization();
        model.set_objective(x + y + z);
        model.solve();
        ASSERT_TRUE(is<status::optimal>(model.get_status()));
        // a continuous column or a fractional coefficient leaves a row alone
        EXPECT_NEAR(model.get_solution_value(), 3.5, TEST_EPSILON);
        EXPECT_NEAR(model.get_solution()[x], 3.0, TEST_EPSILON);
        EXPECT_EQ(model.get_constraint_upper_bound(c), 3.5);
        EXPECT_TRUE(model.is_infinite(model.get_constraint_lower_bound(c)));
    }
}
TEST_F(glpk_milp_test,
       unbounded_relaxation_without_integer_point_is_not_unbounded) {
    using namespace operators;
    auto model = new_model();
    auto x = model.add_variable();
    auto a = model.add_integer_variable({.lower_bound = 0, .upper_bound = 10});
    auto b = model.add_integer_variable({.lower_bound = 0, .upper_bound = 10});
    model.add_constraint(2 * a + 2 * b == 1);
    model.set_maximization();
    model.set_objective(x);
    model.solve();
    EXPECT_TRUE(is_a<status::infeasible_or_unbounded>(model.get_status()));
    EXPECT_FALSE(is_a<status::unbounded>(model.get_status()));
}
// 2 x0 + 2 x1 == 1 has integral sides, so no rounding proves it infeasible,
// and glp_intopt branches on it until its time limit stops it. The limit
// forwarded to the trial ends it at the deadline, so the run reports the
// time limit.
TEST_F(glpk_milp_test, deletion_filter_returns_on_an_endless_integer_row) {
    using namespace operators;
    using seconds = std::chrono::duration<double>;
    auto model = new_model();
    auto x0 = model.add_integer_variable({});
    auto x1 = model.add_integer_variable({});
    model.add_constraint(2 * x0 + 2 * x1 == 1);
    constexpr seconds budget{0.5};
    const auto start = std::chrono::steady_clock::now();
    const auto iis =
        compute_iis_by_deletion(model, iis_limits{.time_limit = budget});
    const seconds elapsed = std::chrono::steady_clock::now() - start;
    EXPECT_LT(elapsed.count(), budget.count() + 1.0);
    EXPECT_TRUE(outcome_is<iis_outcome::time_limit>(iis.get_outcome(), false));
    EXPECT_TRUE(std::isinf(model.get_time_limit().count()));
}
// glp_intopt branches on 2 x0 + 2 x1 == 1 until its time limit stops it, as in
// a full run.
TEST_F(glpk_milp_test, narrowing_returns_on_an_endless_integer_row) {
    check_narrowing_returns_on_an_endless_integer_row(new_model());
}
TEST_F(glpk_milp_test, sides_at_the_wrong_infinity_are_infeasible) {
    check_sides_at_the_wrong_infinity_are_infeasible(
        [this] { return new_model(); });
}
TEST_F(glpk_milp_test, infinite_column_bounds_are_free) {
    check_infinite_column_bounds_are_free([this] { return new_model(); });
}
INSTANTIATE_TEST(GLPK_milp, LpModelTest, glpk_milp_test);
INSTANTIATE_TEST(GLPK_milp, MilpModelTest, glpk_milp_test);
INSTANTIATE_TEST(GLPK_milp, EnumerableEntitiesTest, glpk_milp_test);
INSTANTIATE_TEST(GLPK_milp, ReadableObjectiveTest, glpk_milp_test);
INSTANTIATE_TEST(GLPK_milp, ModifiableObjectiveTest, glpk_milp_test);
INSTANTIATE_TEST(GLPK_milp, ReadableVariablesBoundsTest, glpk_milp_test);
INSTANTIATE_TEST(GLPK_milp, ModifiableVariablesBoundsTest, glpk_milp_test);
INSTANTIATE_TEST(GLPK_milp, NamedVariablesTest, glpk_milp_test);
INSTANTIATE_TEST(GLPK_milp, AddColumnTest, glpk_milp_test);
INSTANTIATE_TEST(GLPK_milp, ReadableConstraintBoundsTest, glpk_milp_test);
INSTANTIATE_TEST(GLPK_milp, ModifiableConstraintBoundsTest, glpk_milp_test);
INSTANTIATE_TEST(GLPK_milp, IisByDeletionTest, glpk_milp_test);
// INSTANTIATE_TEST(GLPK_milp, OptimalityToleranceTest, glpk_milp_test);
// INSTANTIATE_TEST(GLPK_milp, MipGapTest, glpk_milp_test);
// GLPK reports neither after glp_intopt, only to its callbacks
// INSTANTIATE_TEST(GLPK_milp, DualBoundTest, glpk_milp_test);
// INSTANTIATE_TEST(GLPK_milp, NodeCountTest, glpk_milp_test);
INSTANTIATE_TEST(GLPK_milp, IntegralityToleranceTest, glpk_milp_test);
INSTANTIATE_TEST(GLPK_milp, VerbosityTest, glpk_milp_test);
// not wired yet: GLP_ICUTGEN and the cut pool, with the presolver off
// INSTANTIATE_TEST(GLPK_milp, NodeRelaxationCallbackTest, glpk_milp_test);
// INSTANTIATE_TEST(GLPK_milp, UserCutsTest, glpk_milp_test);
INSTANTIATE_TEST(GLPK_milp, ProgressCallbackTest, glpk_milp_test);
INSTANTIATE_TEST(GLPK_milp, ProgressCallbackNodeCountTest, glpk_milp_test);
// no candidate solution callback
// INSTANTIATE_TEST(GLPK_milp, ProgressCallbackSharingTest, glpk_milp_test);
INSTANTIATE_TEST(GLPK_milp, TimeLimitTest, glpk_milp_test);
INSTANTIATE_TEST(GLPK_milp, TimeLimitIncumbentTest, glpk_milp_test);
