#include <chrono>
#include <utility>
#include <variant>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/solvers/clp/all.hpp"

using namespace mippp;

#include "test_suites/all.hpp"

MIPPP_API_VERSION_TEST(Clp_api, clp_api, "CLP")

namespace {
struct clp_lp_status_probe : clp::impl::v1::clp_lp {
    using clp_lp::_simplex_status;
};

auto simplex_status(int problem_status, int secondary_status,
                    bool primal_feasible = false) {
    return clp_lp_status_probe::_simplex_status(
        problem_status, secondary_status, primal_feasible);
}
}  // namespace

TEST(Clp_simplex_status, unscaled_primal_infeasibilities_are_flagged) {
    for(int secondary_status : {2, 4})
        EXPECT_TRUE(is<status::optimal_infeasible_unscaled>(
            simplex_status(0, secondary_status)))
            << secondary_status;
}
TEST(Clp_simplex_status, unscaled_dual_infeasibilities_stay_optimal) {
    EXPECT_TRUE(is<status::optimal>(simplex_status(0, 0)));
    EXPECT_TRUE(is<status::optimal>(simplex_status(0, 3)));
}
TEST(Clp_simplex_status, other_problem_statuses_ignore_the_secondary) {
    for(int secondary_status : {0, 1, 2, 4}) {
        EXPECT_TRUE(is<status::infeasible>(simplex_status(1, secondary_status)))
            << secondary_status;
        EXPECT_TRUE(is<status::unbounded>(simplex_status(2, secondary_status)))
            << secondary_status;
        for(int problem_status : {-1, 4, 5})
            EXPECT_TRUE(is<status::unknown>(
                simplex_status(problem_status, secondary_status)))
                << problem_status << ' ' << secondary_status;
    }
}
TEST(Clp_simplex_status, only_the_time_stop_is_a_time_limit) {
    for(const bool primal_feasible : {false, true}) {
        const auto timed_out = simplex_status(3, 9, primal_feasible);
        EXPECT_TRUE(is<status::time_limit>(timed_out));
        EXPECT_EQ(status::solution_available(timed_out), primal_feasible);
        for(int secondary_status : {0, 10}) {
            const auto stopped =
                simplex_status(3, secondary_status, primal_feasible);
            EXPECT_TRUE(is<status::limit_reached>(stopped)) << secondary_status;
            EXPECT_EQ(status::solution_available(stopped), primal_feasible)
                << secondary_status;
        }
    }
}

struct clp_lp_test : public model_test<clp_api, clp_lp> {
    static void SetUpTestSuite() { construct_api("CLP"); }
};
static_assert(!has_iis<clp_lp>);
static_assert(has_lp_basis_warm_start<clp_lp>);

// clp_lp has no iteration limit: Clp's own count proves the warm start
TEST_F(clp_lp_test, set_basis_starts_at_the_optimum) {
    auto a = new_model();
    lp_basis_cases::build_dense(a);
    a.solve();
    ASSERT_TRUE(is_a<status::optimal>(a.get_status()));
    ASSERT_GT(api->numberIterations(a.native_model()), 0);
    auto b = new_model();
    lp_basis_cases::build_dense(b);
    b.set_basis(a.get_basis());
    b.solve();
    ASSERT_TRUE(is_a<status::optimal>(b.get_status()));
    ASSERT_EQ(api->numberIterations(b.native_model()), 0);
    ASSERT_NEAR(b.get_solution_value(), a.get_solution_value(),
                lp_basis_cases::tolerance(a.get_solution_value()));
}

// Clp codes a fixed column and an equality row 5, where the shared suite also
// accepts a side.
TEST_F(clp_lp_test, get_basis_reads_fixed_entities_as_nonbasic_fixed) {
    for(const bool maximize : {false, true}) {
        auto model = new_model();
        const auto v = lp_basis_cases::build_vertex(model, maximize);
        model.solve();
        ASSERT_TRUE(is_a<status::optimal>(model.get_status())) << maximize;
        const auto basis = model.get_basis();
        ASSERT_TRUE(is<basis_status::nonbasic_fixed>(basis.get_status(v.x2)))
            << maximize;
        ASSERT_TRUE(is<basis_status::nonbasic_fixed>(basis.get_status(v.eq)))
            << maximize;
    }
}

// A removed column stays in Clp, empty and fixed at [0, 0]: written basic, as
// its entry in a snapshot taken before the removal says, it would make the
// basis singular.
TEST_F(clp_lp_test, set_basis_writes_a_removed_column_fixed) {
    auto model = new_model();
    const auto v = lp_basis_cases::build_vertex(model);
    model.solve();
    const auto basis = model.get_basis();
    ASSERT_TRUE(is<basis_status::basic>(basis.get_status(v.x3)));
    model.remove_variable(v.x3);
    model.set_basis(basis);
    const unsigned char * status = api->statusArray(model.native_model());
    ASSERT_NE(status, nullptr);
    ASSERT_EQ(status[model.native_id(v.x3)] & 7, 5);
    ASSERT_EQ(status[model.native_id(v.x4)] & 7, 1);
}
// The code set_basis writes for each tag the statuses list, pinned without a
// library: a tag that loses its line in the table writes as its nearest
// base, which still compiles.
struct clp_basis_codes : clp_lp {
    using column_status = model_variable_basis_status_t<clp_lp>;
    using row_status = model_constraint_basis_status_t<clp_lp>;
    template <typename Tag>
    static constexpr auto column() {
        return _basis_code(column_status(std::in_place_type<Tag>));
    }
    template <typename Tag>
    static constexpr auto row() {
        return _basis_code(row_status(std::in_place_type<Tag>));
    }
};
TEST(clp_lp_basis_codes, one_per_listed_tag) {
    using namespace clp::impl::v1;
    using namespace basis_status;
    using codes = clp_basis_codes;
    // clang-format off
    static_assert(std::variant_size_v<codes::column_status> == 7);
    static_assert(std::variant_size_v<codes::row_status>    == 7);
    // Clp's codes: 0 free, 1 basic, 2 at upper, 3 at lower, 4 superbasic,
    // 5 fixed.
    static_assert(codes::column<basic>()                   == 1);
    static_assert(codes::column<nonbasic_at_lower_bound>() == 3);
    static_assert(codes::column<nonbasic_at_upper_bound>() == 2);
    static_assert(codes::column<nonbasic_fixed>()          == 5);
    static_assert(codes::column<superbasic>()              == 4);
    static_assert(codes::column<nonbasic_free>()           == 0);
    static_assert(codes::column<nonbasic>()                == 3);
    static_assert(codes::row<basic>()                   == 1);
    static_assert(codes::row<nonbasic_at_lower_bound>() == 3);
    static_assert(codes::row<nonbasic_at_upper_bound>() == 2);
    static_assert(codes::row<nonbasic_fixed>()          == 5);
    static_assert(codes::row<superbasic>()              == 4);
    static_assert(codes::row<nonbasic_free>()           == 0);
    static_assert(codes::row<nonbasic>()                == 3);
    // clang-format on
}
// infeasible, but scaling shrinks the violation at x = 0 below Clp's
// tolerance, and its primary status says optimal
TEST_F(clp_lp_test, infeasible_once_unscaled_is_not_plain_optimal) {
    using namespace operators;
    auto model = new_model();
    auto x = model.add_variable();
    model.add_constraint(1e8 * x <= -1e-3);
    model.solve();
    EXPECT_TRUE(is<status::optimal_infeasible_unscaled>(model.get_status()));
}
// Clp_setMaximumSeconds sets a deadline on the CPU clock, counted from the
// call: a limit armed by set_time_limit would have run out before this solve.
TEST_F(clp_lp_test, time_limit_counts_from_the_solve) {
    using seconds = std::chrono::duration<double>;
    auto model = new_model();
    TimeLimitTest<clp_lp_test>::build_dense_lp(model, 30);
    model.set_time_limit(seconds(0.05));
    const auto start = std::chrono::steady_clock::now();
    volatile double spin = 0.0;
    while(std::chrono::steady_clock::now() - start < seconds(0.2))
        spin = spin + 1.0;
    model.solve();
    EXPECT_TRUE(is<status::optimal>(model.get_status()));
}
INSTANTIATE_TEST(Clp, LpModelTest, clp_lp_test);
INSTANTIATE_TEST(Clp, EnumerableEntitiesTest, clp_lp_test);
INSTANTIATE_TEST(Clp, ReadableObjectiveTest, clp_lp_test);
INSTANTIATE_TEST(Clp, ModifiableObjectiveTest, clp_lp_test);
INSTANTIATE_TEST(Clp, ReadableVariablesBoundsTest, clp_lp_test);
INSTANTIATE_TEST(Clp, ModifiableVariablesBoundsTest, clp_lp_test);
INSTANTIATE_TEST(Clp, NamedVariablesTest, clp_lp_test);
INSTANTIATE_TEST(Clp, NamedConstraintsTest, clp_lp_test);
INSTANTIATE_TEST(Clp, AddColumnTest, clp_lp_test);
INSTANTIATE_TEST(Clp, RangedConstraintsTest, clp_lp_test);
INSTANTIATE_TEST(Clp, ReadableConstraintBoundsTest, clp_lp_test);
INSTANTIATE_TEST(Clp, ModifiableConstraintBoundsTest, clp_lp_test);
INSTANTIATE_TEST(Clp, IisByDeletionTest, clp_lp_test);
INSTANTIATE_TEST(Clp, RemoveVariableTest, clp_lp_test);
INSTANTIATE_TEST(Clp, DualSolutionTest, clp_lp_test);
INSTANTIATE_TEST(Clp, ReducedCostsTest, clp_lp_test);
INSTANTIATE_TEST(Clp, LpStatusTest, clp_lp_test);
INSTANTIATE_TEST(Clp, CuttingStockTest, clp_lp_test);
INSTANTIATE_TEST(Clp, ColumnManagerTest, clp_lp_test);
INSTANTIATE_TEST(Clp, LpFuzzyTest, clp_lp_test);
INSTANTIATE_TEST(Clp, VerbosityTest, clp_lp_test);
INSTANTIATE_TEST(Clp, TimeLimitTest, clp_lp_test);
INSTANTIATE_TEST(Clp, LpBasisTest, clp_lp_test);
INSTANTIATE_TEST(Clp, LpBasisWarmStartTest, clp_lp_test);
