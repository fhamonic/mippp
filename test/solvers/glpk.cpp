#include <limits>

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
                    double objective = 1.0) {
    return glpk_lp_status_probe::_simplex_status(ret, generic_status,
                                                 primal_status, objective);
}
auto intopt_status(int ret, int mip_status) {
    return glpk_milp_status_probe::_intopt_status(ret, mip_status);
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
    for(int ret : {GLP_EITLIM, GLP_ETMLIM, GLP_EOBJLL, GLP_EOBJUL}) {
        const auto r = simplex_status(ret, GLP_OPT, GLP_FEAS);
        EXPECT_TRUE(is<status::limit_reached>(r)) << ret;
        EXPECT_TRUE(status::solution_available(r)) << ret;
    }
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
// GLPK refuses to start rather than report the model infeasible
TEST_F(glpk_lp_test, crossing_variable_bounds_fail) {
    using namespace operators;
    auto model = new_model();
    auto x = model.add_variable({.lower_bound = 2, .upper_bound = 1});
    model.add_constraint(x <= 5);
    model.set_objective(x);
    model.solve();
    EXPECT_TRUE(is<status::failed>(model.get_status()));
}
INSTANTIATE_TEST(GLPK_lp, LpModelTest, glpk_lp_test);
INSTANTIATE_TEST(GLPK_lp, ReadableObjectiveTest, glpk_lp_test);
INSTANTIATE_TEST(GLPK_lp, ModifiableObjectiveTest, glpk_lp_test);
INSTANTIATE_TEST(GLPK_lp, ReadableVariablesBoundsTest, glpk_lp_test);
INSTANTIATE_TEST(GLPK_lp, ModifiableVariablesBoundsTest, glpk_lp_test);
INSTANTIATE_TEST(GLPK_lp, NamedVariablesTest, glpk_lp_test);
INSTANTIATE_TEST(GLPK_lp, AddColumnTest, glpk_lp_test);
INSTANTIATE_TEST(GLPK_lp, DualSolutionTest, glpk_lp_test);
INSTANTIATE_TEST(GLPK_lp, ReducedCostsTest, glpk_lp_test);
INSTANTIATE_TEST(GLPK_lp, LpStatusTest, glpk_lp_test);
INSTANTIATE_TEST(GLPK_lp, CuttingStockTest, glpk_lp_test);
INSTANTIATE_TEST(GLPK_lp, VerbosityTest, glpk_lp_test);

struct glpk_milp_test : public model_test<glpk_api, glpk_milp> {
    static void SetUpTestSuite() { construct_api("GLPK"); }
};
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
INSTANTIATE_TEST(GLPK_milp, LpModelTest, glpk_milp_test);
INSTANTIATE_TEST(GLPK_milp, MilpModelTest, glpk_milp_test);
INSTANTIATE_TEST(GLPK_milp, ReadableObjectiveTest, glpk_milp_test);
INSTANTIATE_TEST(GLPK_milp, ModifiableObjectiveTest, glpk_milp_test);
INSTANTIATE_TEST(GLPK_milp, ReadableVariablesBoundsTest, glpk_milp_test);
INSTANTIATE_TEST(GLPK_milp, ModifiableVariablesBoundsTest, glpk_milp_test);
INSTANTIATE_TEST(GLPK_milp, NamedVariablesTest, glpk_milp_test);
INSTANTIATE_TEST(GLPK_milp, AddColumnTest, glpk_milp_test);
// INSTANTIATE_TEST(GLPK_milp, OptimalityToleranceTest, glpk_milp_test);
// INSTANTIATE_TEST(GLPK_milp, MipGapTest, glpk_milp_test);
INSTANTIATE_TEST(GLPK_milp, IntegralityToleranceTest, glpk_milp_test);
INSTANTIATE_TEST(GLPK_milp, VerbosityTest, glpk_milp_test);
