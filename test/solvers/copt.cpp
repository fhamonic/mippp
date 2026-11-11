#include <chrono>
#include <concepts>
#include <cstddef>
#include <random>
#include <ranges>
#include <stdexcept>
#include <utility>
#include <variant>
#include <vector>

#include "mippp/solvers/copt/all.hpp"

using namespace mippp;

#include "test_suites/all.hpp"

MIPPP_API_VERSION_TEST(COPT_api, copt_api, "COPT")

namespace {
struct fake_copt_release_api {
    std::vector<char> & released;
    int DeleteProb(copt::impl::v1::copt_prob **) const {
        released.push_back('p');
        return -1;
    }
    int DeleteEnv(copt::impl::v1::copt_env **) const {
        released.push_back('e');
        return -1;
    }
};
}  // namespace

TEST(COPT_handle_guard, releases_partial_allocations_without_throwing) {
    using namespace copt::impl::v1;
    int storage = 0;
    for(unsigned allocated : {0u, 1u, 2u}) {
        std::vector<char> released;
        const fake_copt_release_api api{released};
        copt_env * env = nullptr;
        copt_prob * prob = nullptr;
        try {
            mippp::detail::handle_guard<fake_copt_release_api, copt_env *,
                                        copt_prob *, copt_handle_release>
                guard(api, env, prob);
            if(allocated >= 1) env = reinterpret_cast<copt_env *>(&storage);
            if(allocated >= 2) prob = reinterpret_cast<copt_prob *>(&storage);
            throw std::runtime_error("construction failure");
        } catch(const std::runtime_error & e) {
            EXPECT_STREQ(e.what(), "construction failure");
        }
        EXPECT_EQ(env, nullptr);
        EXPECT_EQ(prob, nullptr);
        const std::vector<char> expected =
            allocated == 2   ? std::vector<char>{'p', 'e'}
            : allocated == 1 ? std::vector<char>{'e'}
                             : std::vector<char>{};
        EXPECT_EQ(released, expected);
    }
}

struct copt_lp_test : public model_test<copt_api, copt_lp> {
    static void SetUpTestSuite() { construct_api("COPT"); }
};
INSTANTIATE_TEST(COPT_lp, LpModelTest, copt_lp_test);
INSTANTIATE_TEST(COPT_lp, EnumerableEntitiesTest, copt_lp_test);
INSTANTIATE_TEST(COPT_lp, ReadableObjectiveTest, copt_lp_test);
INSTANTIATE_TEST(COPT_lp, ModifiableObjectiveTest, copt_lp_test);
INSTANTIATE_TEST(COPT_lp, ReadableVariablesBoundsTest, copt_lp_test);
INSTANTIATE_TEST(COPT_lp, ModifiableVariablesBoundsTest, copt_lp_test);
INSTANTIATE_TEST(COPT_lp, NamedVariablesTest, copt_lp_test);
INSTANTIATE_TEST(COPT_lp, AddColumnTest, copt_lp_test);
INSTANTIATE_TEST(COPT_lp, ReadableConstraintBoundsTest, copt_lp_test);
INSTANTIATE_TEST(COPT_lp, DualSolutionTest, copt_lp_test);
INSTANTIATE_TEST(COPT_lp, ReducedCostsTest, copt_lp_test);
INSTANTIATE_TEST(COPT_lp, LpStatusTest, copt_lp_test);
INSTANTIATE_TEST(COPT_lp, CuttingStockTest, copt_lp_test);
INSTANTIATE_TEST(COPT_lp, TimeLimitTest, copt_lp_test);
INSTANTIATE_TEST(COPT_lp, LpBasisTest, copt_lp_test);
INSTANTIATE_TEST(COPT_lp, LpBasisWarmStartTest, copt_lp_test);
INSTANTIATE_TEST(COPT_lp, VerbosityTest, copt_lp_test);
INSTANTIATE_TEST(COPT_lp, ThreadLimitTest, copt_lp_test);
INSTANTIATE_TEST(COPT_lp, LpAlgorithmTest, copt_lp_test);
static_assert(has_lp_basis_warm_start<copt_lp>);

// COPT's LpMethod has no primal simplex, so neither side-less "simplex" nor
// primal_simplex is listed, and the selector is an LP member. Its barrier
// without crossover is left to the native handle: COPT 8.0 then reports LPs
// that have no optimum as optimal, with an objective value.
static_assert(has_lp_algorithm<copt_lp>);
static_assert(has_lp_algorithm<copt_lp, lp_algorithm::dual_simplex>);
static_assert(has_lp_algorithm<copt_lp, lp_algorithm::interior_point>);
static_assert(
    !has_lp_algorithm<copt_lp, lp_algorithm::interior_point_without_crossover>);
static_assert(!has_lp_algorithm<copt_lp, lp_algorithm::primal_simplex>);
static_assert(!has_lp_algorithm<copt_lp, lp_algorithm::simplex>);
static_assert(!has_lp_algorithm<copt_lp, lp_algorithm::other>);
static_assert(variant_with_alternative<model_lp_algorithm_t<copt_lp>,
                                       lp_algorithm::other>);
static_assert(!has_lp_algorithm<copt_milp>);

namespace {
int simplex_iterations(const copt_lp & model) {
    using namespace copt::impl::v1;
    int count = -1;
    model.native_api()._check(
        model.native_model().first,
        model.native_api().GetIntAttr(model.native_model().second,
                                      COPT_INTATTR_SIMPLEXITER, &count));
    return count;
}
}  // namespace

// copt_lp has no iteration limit: COPT's own count proves the warm start
namespace {
int copt_int_attribute(const copt_lp & model, const char * name) {
    int value = -1;
    model.native_api()._check(model.native_model().first,
                              model.native_api().GetIntAttr(
                                  model.native_model().second, name, &value));
    return value;
}
int copt_int_parameter(const copt_lp & model, const char * name) {
    int value = -99;
    model.native_api()._check(model.native_model().first,
                              model.native_api().GetIntParam(
                                  model.native_model().second, name, &value));
    return value;
}
}  // namespace

// The shared suite cannot see which algorithm ran: COPT's counters can.
TEST_F(copt_lp_test, each_lp_algorithm_runs_the_one_it_names) {
    using namespace copt::impl::v1;
    SkipOnLicenseError([this]() {
        struct run {
            int method, crossover, simplex, barrier;
        };
        const auto solved_under = [this](auto algorithm) {
            auto model = new_model();
            lp_basis_cases::build_dense(model);
            model.set_lp_algorithm(algorithm);
            model.solve();
            EXPECT_TRUE(is_a<status::optimal>(model.get_status()));
            return run{copt_int_parameter(model, COPT_INTPARAM_LPMETHOD),
                       copt_int_parameter(model, COPT_INTPARAM_CROSSOVER),
                       copt_int_attribute(model, COPT_INTATTR_SIMPLEXITER),
                       copt_int_attribute(model, COPT_INTATTR_BARRIERITER)};
        };
        {
            const run r = solved_under(lp_algorithm::dual_simplex{});
            EXPECT_EQ(r.method, 1);
            EXPECT_GT(r.simplex, 0);
            EXPECT_EQ(r.barrier, 0);
        }
        {
            const run r = solved_under(lp_algorithm::interior_point{});
            EXPECT_EQ(r.method, 2);
            EXPECT_EQ(r.crossover, 1);
            EXPECT_GT(r.barrier, 0);
        }
        {
            const run r = solved_under(lp_algorithm::automatic{});
            EXPECT_EQ(r.method, -1);
            EXPECT_EQ(r.crossover, 1);
        }
    });
}

// The concurrent method is no tag, and neither is a barrier whose crossover
// is off or left to COPT: they read as other, and interior_point puts the
// crossover back.
TEST_F(copt_lp_test, a_native_lp_method_reads_as_other) {
    using namespace copt::impl::v1;
    SkipOnLicenseError([this]() {
        auto model = new_model();
        const auto & native = model.native_api();
        const auto [env, prob] = model.native_model();
        native._check(env, native.SetIntParam(prob, COPT_INTPARAM_LPMETHOD, 4));
        EXPECT_TRUE(is<lp_algorithm::other>(model.get_lp_algorithm()));
        native._check(env, native.SetIntParam(prob, COPT_INTPARAM_LPMETHOD, 2));
        EXPECT_TRUE(is<lp_algorithm::interior_point>(model.get_lp_algorithm()));
        for(const int crossover : {-1, 0}) {
            native._check(env, native.SetIntParam(prob, COPT_INTPARAM_CROSSOVER,
                                                  crossover));
            EXPECT_TRUE(is<lp_algorithm::other>(model.get_lp_algorithm()));
        }
        model.set_lp_algorithm(lp_algorithm::interior_point{});
        EXPECT_EQ(copt_int_parameter(model, COPT_INTPARAM_CROSSOVER), 1);
    });
}

TEST_F(copt_lp_test, set_basis_starts_at_the_optimum) {
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

// Before a solve, COPT returns a written basis as it is, which checks the
// codes set_basis() writes without the solve get_basis() needs; a bare
// nonbasic entity is written at its lower bound.
TEST_F(copt_lp_test, set_basis_writes_the_codes_before_a_solve) {
    using namespace copt::impl::v1;
    auto model = new_model();
    const auto v = lp_basis_cases::build_vertex(model);
    model_basis_t<copt_lp> basis;
    basis.set_status(v.x1, basis_status::nonbasic_at_upper_bound{});
    basis.set_status(v.x2, basis_status::nonbasic_fixed{});
    basis.set_status(v.x3, basis_status::basic{});
    basis.set_status(v.x4, basis_status::basic{});
    basis.set_status(v.x5, basis_status::basic{});
    basis.set_status(v.f, basis_status::superbasic{});
    basis.set_status(v.le, basis_status::nonbasic_at_upper_bound{});
    basis.set_status(v.ge, basis_status::nonbasic_at_lower_bound{});
    basis.set_status(v.eq, basis_status::nonbasic_fixed{});
    basis.set_status(v.slack, basis_status::basic{});
    model.set_basis(basis);
    const auto native_status = [&](auto entity) {
        std::vector<int> col_status(model.num_variables());
        std::vector<int> row_status(model.num_constraints());
        EXPECT_EQ(api->GetBasis(model.native_model().second, col_status.data(),
                                row_status.data()),
                  0);
        if constexpr(std::same_as<decltype(entity), model_variable_t<copt_lp>>)
            return col_status[static_cast<std::size_t>(
                model.native_id(entity))];
        else
            return row_status[static_cast<std::size_t>(
                model.native_id(entity))];
    };
    EXPECT_EQ(native_status(v.x1), COPT_BASIS_UPPER);
    EXPECT_EQ(native_status(v.x2), COPT_BASIS_FIXED);
    EXPECT_EQ(native_status(v.x3), COPT_BASIS_BASIC);
    EXPECT_EQ(native_status(v.x4), COPT_BASIS_BASIC);
    EXPECT_EQ(native_status(v.x5), COPT_BASIS_BASIC);
    EXPECT_EQ(native_status(v.w), COPT_BASIS_LOWER);
    EXPECT_EQ(native_status(v.f), COPT_BASIS_SUPERBASIC);
    EXPECT_EQ(native_status(v.le), COPT_BASIS_UPPER);
    EXPECT_EQ(native_status(v.ge), COPT_BASIS_LOWER);
    EXPECT_EQ(native_status(v.eq), COPT_BASIS_FIXED);
    EXPECT_EQ(native_status(v.slack), COPT_BASIS_BASIC);
}

// an empty vector's data() is null, which COPT_SetBasis rejects
TEST_F(copt_lp_test, set_basis_accepts_a_model_with_an_empty_dimension) {
    using namespace operators;
    auto columns_only = new_model();
    columns_only.add_variable();
    columns_only.add_variable();
    auto rows_only = new_model();
    constexpr auto & no_terms =
        empty_linear_expression<model_variable_t<copt_lp>, double>;
    rows_only.add_constraint(no_terms <= 1.0);
    for(copt_lp * model : {&columns_only, &rows_only}) {
        ASSERT_NO_THROW(model->set_basis(model_basis_t<copt_lp>{}));
        model->solve();
        ASSERT_TRUE(is_a<status::optimal>(model->get_status()));
    }
}

// COPT_Reset, which compute_iis() runs, leaves a basis of zeros that
// COPT_GetBasis returns without an error: compute_iis() resets the status,
// so get_basis() refuses to read them.
TEST_F(copt_lp_test, get_basis_after_compute_iis_throws) {
    using namespace operators;
    auto model = new_model();
    const auto x = model.add_variable();
    const auto y = model.add_variable();
    model.add_constraint(x + y >= 5);
    model.add_constraint(x + 2 * y <= 3);
    const auto iis = model.compute_iis();
    ASSERT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
    int col_status[2] = {-1, -1}, row_status[2] = {-1, -1};
    ASSERT_EQ(
        api->GetBasis(model.native_model().second, col_status, row_status), 0);
    ASSERT_THROW(model.get_basis(), solve_required_error);
}

// COPT holds no basis for a model without rows until it is solved, and says
// so by COPT_RETCODE_INVALID rather than by zeros: get_basis() needs that
// solve anyway.
TEST_F(copt_lp_test, get_basis_on_a_model_without_rows_throws) {
    auto model = new_model();
    model.add_variable();
    model.add_variable();
    int col_status[2] = {-1, -1}, row_status[1] = {-1};
    ASSERT_EQ(
        api->GetBasis(model.native_model().second, col_status, row_status),
        copt::impl::v1::COPT_RETCODE_INVALID);
    ASSERT_THROW(model.get_basis(), solve_required_error);
}
TEST_F(copt_lp_test, get_basis_throws_after_presolve_proves_infeasibility) {
    auto model = new_model();
    lp_basis_cases::build_infeasible(model);
    model.solve();
    ASSERT_TRUE(is_a<status::infeasible_or_unbounded>(model.get_status()));
    ASSERT_THROW(model.get_basis(), solver_error);
}

struct copt_milp_test : public model_test<copt_api, copt_milp> {
    static void SetUpTestSuite() { construct_api("COPT"); }
};
INSTANTIATE_TEST(COPT_milp, LpModelTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, MilpModelTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, EnumerableEntitiesTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, ReadableObjectiveTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, ModifiableObjectiveTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, ReadableVariablesBoundsTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, ModifiableVariablesBoundsTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, NamedVariablesTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, AddColumnTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, ReadableConstraintBoundsTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, SudokuTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, CandidateSolutionCallbackTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, LazyConstraintsTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, SolutionInjectionTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, NodeRelaxationCallbackTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, UserCutsTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, TravellingSalesmanTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, TimeLimitTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, TimeLimitIncumbentTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, MipStartTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, OptimalityToleranceTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, MipGapTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, DualBoundTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, NodeCountTest, copt_milp_test);
// INSTANTIATE_TEST(COPT_milp, IntegralityToleranceTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, VerbosityTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, ThreadLimitTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, ProgressCallbackTest, copt_milp_test);
// NodeCnt reads 0 inside a callback
// INSTANTIATE_TEST(COPT_milp, ProgressCallbackNodeCountTest, copt_milp_test);
INSTANTIATE_TEST(COPT_milp, ProgressCallbackSharingTest, copt_milp_test);

static_assert(!has_lp_basis<copt_milp>);
// The code set_basis writes for each tag the statuses list, pinned without a
// library: a tag that loses its line in the table writes as its nearest
// base, which still compiles.
struct copt_basis_codes : copt_lp {
    using column_status = model_variable_basis_status_t<copt_lp>;
    using row_status = model_constraint_basis_status_t<copt_lp>;
    template <typename Tag>
    static constexpr auto column() {
        return _basis_code(column_status(std::in_place_type<Tag>));
    }
    template <typename Tag>
    static constexpr auto row() {
        return _basis_code(row_status(std::in_place_type<Tag>));
    }
};
TEST(copt_lp_basis_codes, one_per_listed_tag) {
    using namespace copt::impl::v1;
    using namespace basis_status;
    using codes = copt_basis_codes;
    // clang-format off
    static_assert(std::variant_size_v<codes::column_status> == 6);
    static_assert(std::variant_size_v<codes::row_status>    == 6);
    static_assert(codes::column<basic>()                   == COPT_BASIS_BASIC);
    static_assert(codes::column<nonbasic_at_lower_bound>() == COPT_BASIS_LOWER);
    static_assert(codes::column<nonbasic_at_upper_bound>() == COPT_BASIS_UPPER);
    static_assert(codes::column<nonbasic_fixed>()          == COPT_BASIS_FIXED);
    static_assert(codes::column<superbasic>()              == COPT_BASIS_SUPERBASIC);
    static_assert(codes::column<nonbasic>()                == COPT_BASIS_LOWER);
    static_assert(codes::row<basic>()                   == COPT_BASIS_BASIC);
    static_assert(codes::row<nonbasic_at_lower_bound>() == COPT_BASIS_LOWER);
    static_assert(codes::row<nonbasic_at_upper_bound>() == COPT_BASIS_UPPER);
    static_assert(codes::row<nonbasic_fixed>()          == COPT_BASIS_FIXED);
    static_assert(codes::row<superbasic>()              == COPT_BASIS_SUPERBASIC);
    static_assert(codes::row<nonbasic>()                == COPT_BASIS_LOWER);
    // clang-format on
}
static_assert(has_iis<copt_lp>);
static_assert(has_iis<copt_milp>);
static_assert(std::same_as<
              native_iis_outcome_t<copt_lp>,
              std::variant<iis_outcome::incomplete, iis_outcome::irreducible,
                           iis_outcome::feasible, iis_outcome::interrupted,
                           iis_outcome::time_limit, iis_outcome::node_limit>>);
static_assert(std::same_as<native_iis_outcome_t<copt_milp>,
                           native_iis_outcome_t<copt_lp>>);
static_assert(iis_by_deletion_model<copt_lp>);
static_assert(iis_by_deletion_model<copt_milp>);
INSTANTIATE_TEST(COPT_lp, IisTest, copt_lp_test);
INSTANTIATE_TEST(COPT_milp, IisTest, copt_milp_test);
INSTANTIATE_TEST(COPT_lp, ModifiableConstraintBoundsTest, copt_lp_test);
INSTANTIATE_TEST(COPT_milp, ModifiableConstraintBoundsTest, copt_milp_test);
INSTANTIATE_TEST(COPT_lp, IisByDeletionTest, copt_lp_test);
INSTANTIATE_TEST(COPT_milp, IisByDeletionTest, copt_milp_test);

template <typename Model>
struct copt_iis_test : public model_test<copt_api, Model> {
    static void SetUpTestSuite() {
        model_test<copt_api, Model>::construct_api("COPT");
    }

    static int read_iis_method(const Model & model) {
        int value = 0;
        model.native_api()._check(
            model.native_model().first,
            model.native_api().GetIntParam(model.native_model().second,
                                           "IISMethod", &value));
        return value;
    }
    static int read_int_attr(const Model & model, const char * name) {
        int value = 0;
        model.native_api()._check(
            model.native_model().first,
            model.native_api().GetIntAttr(model.native_model().second, name,
                                          &value));
        return value;
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
    // z = value -> x >= rhs, or x <= rhs under COPT_LESS_EQUAL
    static void add_indicator(Model & model, model_variable_t<Model> z,
                              int value, model_variable_t<Model> x, double rhs,
                              char sense = copt::impl::v1::COPT_GREATER_EQUAL) {
        const int column = model.native_id(x);
        const double one = 1.;
        model.native_api()._check(
            model.native_model().first,
            model.native_api().AddIndicator(model.native_model().second,
                                            model.native_id(z), value, 1,
                                            &column, &one, sense, rhs));
    }
    static void add_sos1(Model & model, model_variable_t<Model> x,
                         model_variable_t<Model> y) {
        const int type = 1, begin = 0, count = 2;
        const int columns[2] = {model.native_id(x), model.native_id(y)};
        const double weights[2] = {1., 2.};
        model.native_api()._check(
            model.native_model().first,
            model.native_api().AddSOSs(model.native_model().second, 1, &type,
                                       &begin, &count, columns, weights));
    }
};
using copt_lp_iis_test = copt_iis_test<copt_lp>;
using copt_milp_iis_test = copt_iis_test<copt_milp>;

// Neither path writes IISMethod, whether the LP path decodes an answer or
// confirms feasibility by a solve, or the MIP path solves first.
TEST_F(copt_lp_iis_test, compute_iis_leaves_iis_method_alone) {
    using namespace operators;
    auto model = this->new_model();
    const int method_before = read_iis_method(model);
    auto x = model.add_variable({.lower_bound = 0., .upper_bound = 1.});
    auto r = model.add_constraint(x >= 2.);
    const auto iis = model.compute_iis();
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
    EXPECT_TRUE(is<iis_status::member_upper>(iis.get_status(x)));
    EXPECT_TRUE(is<iis_status::member_lower>(iis.get_status(r)));
    EXPECT_EQ(read_iis_method(model), method_before);
    model.set_variable_upper_bound(x, 3.);
    const auto iis2 = model.compute_iis();
    EXPECT_TRUE(outcome_is<iis_outcome::feasible>(iis2.get_outcome()));
    EXPECT_EQ(read_iis_method(model), method_before);
}

TEST_F(copt_milp_iis_test, compute_iis_leaves_iis_method_alone) {
    using namespace operators;
    auto model = this->new_model();
    const int method_before = read_iis_method(model);
    auto x = model.add_integer_variable({.lower_bound = 0., .upper_bound = 1.});
    model.add_constraint(x >= 2.);
    const auto iis = model.compute_iis();
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
    EXPECT_EQ(read_iis_method(model), method_before);
}

// COPT_ComputeIIS reads past its arrays on a model without rows whose
// bounds are feasible as an LP: the wrapper answers such a model itself.
TEST_F(copt_lp_iis_test, row_less_model_is_answered_from_its_columns) {
    {
        auto model = this->new_model();
        const auto iis = model.compute_iis();
        EXPECT_TRUE(outcome_is<iis_outcome::feasible>(iis.get_outcome()));
    }
    {
        auto model = this->new_model();
        auto x = model.add_variable({.lower_bound = 0., .upper_bound = 1.});
        const auto iis = model.compute_iis();
        EXPECT_TRUE(outcome_is<iis_outcome::feasible>(iis.get_outcome()));
        EXPECT_TRUE(is<iis_status::absent>(iis.get_status(x)));
    }
    {
        auto model = this->new_model();
        auto x = model.add_variable({.lower_bound = 0., .upper_bound = 1.});
        auto y = model.add_variable({.lower_bound = 1., .upper_bound = 0.});
        const auto iis = model.compute_iis();
        EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
        EXPECT_TRUE(is<iis_status::absent>(iis.get_status(x)));
        EXPECT_TRUE(is<iis_status::member_both>(iis.get_status(y)));
        EXPECT_EQ(iis.num_variable_members(), 1u);
    }
}

TEST_F(copt_milp_iis_test, row_less_model_is_answered_from_its_columns) {
    {
        auto model = this->new_model();
        auto x =
            model.add_integer_variable({.lower_bound = 0., .upper_bound = 1.});
        const auto iis = model.compute_iis();
        EXPECT_TRUE(outcome_is<iis_outcome::feasible>(iis.get_outcome()));
        EXPECT_TRUE(is<iis_status::absent>(iis.get_status(x)));
    }
    {
        auto model = this->new_model();
        auto x =
            model.add_integer_variable({.lower_bound = 0., .upper_bound = 1.});
        auto y = model.add_integer_variable(
            {.lower_bound = 0.25, .upper_bound = 0.75});
        const auto iis = model.compute_iis();
        EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
        EXPECT_TRUE(is<iis_status::absent>(iis.get_status(x)));
        EXPECT_TRUE(is<iis_status::member_both>(iis.get_status(y)));
        EXPECT_EQ(iis.num_variable_members(), 1u);
    }
}

// On a MIP the routine flags the whole model as an IIS unless a solve has
// just found it infeasible, and a status survives every modification: the
// wrapper solves first.
TEST_F(copt_milp_iis_test, feasible_mip_is_feasible_unsolved_or_stale) {
    using namespace operators;
    {
        auto model = this->new_model();
        auto x =
            model.add_integer_variable({.lower_bound = 0., .upper_bound = 4.});
        auto y = model.add_variable({.lower_bound = 0., .upper_bound = 4.});
        model.set_maximization();
        model.set_objective(x + 2 * y + 1);
        model.add_constraint(x + y <= 6.5);
        const auto iis = model.compute_iis();
        EXPECT_TRUE(outcome_is<iis_outcome::feasible>(iis.get_outcome()));
        EXPECT_EQ(iis.num_variable_members(), 0u);
        EXPECT_EQ(iis.num_constraint_members(), 0u);
        EXPECT_TRUE(is<status::unknown>(model.get_status()));
        model.solve();
        ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
        EXPECT_NEAR(model.get_solution_value(), 11., TEST_EPSILON);
    }
    {
        auto model = this->new_model();
        auto x =
            model.add_integer_variable({.lower_bound = 0., .upper_bound = 1.});
        auto r = model.add_constraint(x >= 2.);
        model.solve();
        ASSERT_TRUE(is_a<status::infeasible>(model.get_status()));
        model.set_variable_upper_bound(x, 3.);
        const auto iis = model.compute_iis();
        EXPECT_TRUE(outcome_is<iis_outcome::feasible>(iis.get_outcome()));
        model.set_variable_upper_bound(x, 1.);
        const auto iis2 = model.compute_iis();
        EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis2.get_outcome()));
        EXPECT_TRUE(is_a<iis_status::member>(iis2.get_status(x)));
        EXPECT_TRUE(is<iis_status::member_lower>(iis2.get_status(r)));
    }
}

// One flag where both sides are needed: x needs both bounds (x = 0 lets
// y = 2, x = 2 lets y = 1) and the row both sides, yet COPT flags one each.
TEST_F(copt_milp_iis_test, mip_members_needing_both_sides_are_whole) {
    using namespace operators;
    auto model = this->new_model();
    auto x =
        model.add_integer_variable({.lower_bound = 0.5, .upper_bound = 1.5});
    auto y =
        model.add_integer_variable({.lower_bound = 0., .upper_bound = 10.});
    auto r = model.add_constraint(x + 2 * y == 4.);
    const auto iis = model.compute_iis();
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
    EXPECT_TRUE(is<iis_status::member>(iis.get_status(x)));
    EXPECT_TRUE(is<iis_status::absent>(iis.get_status(y)));
    EXPECT_TRUE(is<iis_status::member>(iis.get_status(r)));
    EXPECT_EQ(iis.num_variable_members(), 1u);
    EXPECT_EQ(iis.num_constraint_members(), 1u);
}

// A one-sided row and a one-bounded column keep their side on a MIP; a
// two-bounded column is whole whether it is integer or continuous.
TEST_F(copt_milp_iis_test, one_sided_members_keep_their_side_on_a_mip) {
    using namespace operators;
    auto model = this->new_model();
    auto x = model.add_integer_variable({.lower_bound = 0., .upper_bound = 1.});
    auto y = model.add_variable({.lower_bound = 0., .upper_bound = 1.});
    auto w = model.add_variable(
        {.lower_bound = -model.infinity(), .upper_bound = 1.});
    auto r = model.add_constraint(x + y + w >= 4.);
    const auto iis = model.compute_iis();
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
    EXPECT_TRUE(is<iis_status::member>(iis.get_status(x)));
    EXPECT_TRUE(is<iis_status::member>(iis.get_status(y)));
    EXPECT_TRUE(is<iis_status::member_upper>(iis.get_status(w)));
    EXPECT_TRUE(is<iis_status::member_lower>(iis.get_status(r)));
}

// COPT's routine leaks memory on a model without columns (measured on 8.0.5
// under LeakSanitizer), so such a model is answered from its constant rows:
// a side an infinite value reads as 1e30 on COPT, and 0 satisfies it.
template <typename Model>
void check_column_less_rows_that_admit_zero_are_feasible(Model model) {
    using namespace operators;
    constexpr auto & no_terms =
        empty_linear_expression<model_variable_t<Model>, model_scalar_t<Model>>;
    auto r0 = model.add_constraint(no_terms <= 2.);
    auto r1 = model.add_constraint(no_terms >= -1.);
    auto r2 = model.add_constraint(no_terms == 0.);
    const auto iis = model.compute_iis();
    EXPECT_TRUE(outcome_is<iis_outcome::feasible>(iis.get_outcome()));
    EXPECT_TRUE(is<iis_status::absent>(iis.get_status(r0)));
    EXPECT_TRUE(is<iis_status::absent>(iis.get_status(r1)));
    EXPECT_TRUE(is<iis_status::absent>(iis.get_status(r2)));
    EXPECT_EQ(iis.num_constraint_members(), 0u);
}
TEST_F(copt_lp_iis_test, column_less_rows_that_admit_zero_are_feasible) {
    check_column_less_rows_that_admit_zero_are_feasible(this->new_model());
}
TEST_F(copt_milp_iis_test, column_less_rows_that_admit_zero_are_feasible) {
    check_column_less_rows_that_admit_zero_are_feasible(this->new_model());
}

// COPT flags one bound of a two-bounded continuous column on a MIP where both
// are needed: x = 0 lets y = 2 and x = 2 lets y = 1, so a sided x would name
// a feasible subsystem.
TEST_F(copt_milp_iis_test, two_bounded_continuous_column_is_whole_on_a_mip) {
    using namespace operators;
    auto model = this->new_model();
    auto x = model.add_variable({.lower_bound = 0.5, .upper_bound = 1.5});
    auto y =
        model.add_integer_variable({.lower_bound = 0., .upper_bound = 10.});
    auto r = model.add_constraint(x + 2 * y == 4.);
    const auto iis = model.compute_iis();
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
    EXPECT_TRUE(is<iis_status::member>(iis.get_status(x)));
    EXPECT_TRUE(is<iis_status::absent>(iis.get_status(y)));
    EXPECT_TRUE(is<iis_status::member>(iis.get_status(r)));
    EXPECT_EQ(iis.num_variable_members(), 1u);
    EXPECT_EQ(iis.num_constraint_members(), 1u);
}

// The indicator is a member for COPT; the snapshot lists the linear members
// only, an IIS relative to it.
TEST_F(copt_milp_iis_test, indicator_is_background) {
    using namespace operators;
    auto model = this->new_model();
    auto z = model.add_binary_variable();
    auto x = model.add_variable({.lower_bound = 0., .upper_bound = 10.});
    model.set_variable_lower_bound(z, 1.);
    auto r = model.add_constraint(x <= 3.);
    add_indicator(model, z, 1, x, 5.);
    ASSERT_EQ(read_int_attr(model, "Indicators"), 1);
    const auto iis = model.compute_iis();
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
    EXPECT_TRUE(is_a<iis_status::member>(iis.get_status(z)));
    EXPECT_TRUE(is<iis_status::absent>(iis.get_status(x)));
    EXPECT_TRUE(is<iis_status::member_upper>(iis.get_status(r)));
    EXPECT_EQ(iis.num_variable_members(), 1u);
    EXPECT_EQ(iis.num_constraint_members(), 1u);
    EXPECT_EQ(read_int_attr(model, "IISIndicators"), 1);
}

// The routine takes indicators for candidates, and an IIS that leaves one
// out is minimal against the ones it names only: the counts cannot tell an
// unrelated indicator, as here, from one whose conflict would make r
// redundant, so the answer holds a conflict without claiming minimality.
// The unrelated indicator acts on x: on a column of its own, 8.0.5 flags both
// bounds of that column outside IISCols, and the answer holds no conflict.
TEST_F(copt_milp_iis_test, iis_naming_some_indicators_is_not_irreducible) {
    using namespace operators;
    auto model = this->new_model();
    auto z = model.add_binary_variable();
    auto z2 = model.add_binary_variable();
    auto x = model.add_variable({.lower_bound = 0., .upper_bound = 10.});
    model.set_variable_lower_bound(z, 1.);
    auto r = model.add_constraint(x <= 3.);
    add_indicator(model, z, 1, x, 5.);
    add_indicator(model, z2, 1, x, 0.);
    ASSERT_EQ(read_int_attr(model, "Indicators"), 2);
    const auto iis = model.compute_iis();
    EXPECT_TRUE(outcome_is<iis_outcome::incomplete>(iis.get_outcome(), true));
    EXPECT_TRUE(is_a<iis_status::member>(iis.get_status(z)));
    EXPECT_TRUE(is<iis_status::absent>(iis.get_status(z2)));
    EXPECT_TRUE(is<iis_status::absent>(iis.get_status(x)));
    EXPECT_TRUE(is<iis_status::member_upper>(iis.get_status(r)));
    EXPECT_EQ(read_int_attr(model, "IISIndicators"), 1);
}

// Two IISs: {z, r, z -> x >= 5} and {z, z -> x >= 5, z -> x <= 2}. Against
// the indicators as background z alone is the IIS, so an answer naming r
// must not be irreducible, whichever IIS the routine returns.
TEST_F(copt_milp_iis_test, irreducible_answer_never_names_a_redundant_row) {
    using namespace operators;
    auto model = this->new_model();
    auto z = model.add_binary_variable();
    auto x = model.add_variable({.lower_bound = 0., .upper_bound = 10.});
    model.set_variable_lower_bound(z, 1.);
    auto r = model.add_constraint(x <= 3.);
    add_indicator(model, z, 1, x, 5.);
    add_indicator(model, z, 1, x, 2., copt::impl::v1::COPT_LESS_EQUAL);
    const auto iis = model.compute_iis();
    EXPECT_TRUE(is_a<iis_status::member>(iis.get_status(z)));
    if(read_int_attr(model, "IISIndicators") == 2) {
        EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
        EXPECT_TRUE(is<iis_status::absent>(iis.get_status(r)));
    } else {
        EXPECT_TRUE(
            outcome_is<iis_outcome::incomplete>(iis.get_outcome(), true));
        EXPECT_TRUE(is<iis_status::member_upper>(iis.get_status(r)));
    }
}

TEST_F(copt_milp_iis_test, sos_is_background) {
    using namespace operators;
    auto model = this->new_model();
    // one-bounded columns keep their side on a MIP
    auto x = model.add_variable({.lower_bound = 1.});
    auto y = model.add_variable({.lower_bound = 1.});
    auto r = model.add_constraint(x + y <= 20.);
    add_sos1(model, x, y);
    ASSERT_EQ(read_int_attr(model, "Soss"), 1);
    const auto iis = model.compute_iis();
    EXPECT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
    EXPECT_TRUE(is<iis_status::member_lower>(iis.get_status(x)));
    EXPECT_TRUE(is<iis_status::member_lower>(iis.get_status(y)));
    EXPECT_TRUE(is<iis_status::absent>(iis.get_status(r)));
    EXPECT_EQ(iis.num_variable_members(), 2u);
    EXPECT_EQ(iis.num_constraint_members(), 0u);
    EXPECT_EQ(read_int_attr(model, "IISSOSs"), 1);
}

// The confirming solve cannot tell infeasible from unbounded, and the
// routine would then flag the whole model.
TEST_F(copt_milp_iis_test, infeasible_or_unbounded_mip_is_incomplete) {
    using namespace operators;
    auto model = this->new_model();
    auto x = model.add_integer_variable();
    model.add_constraint(x >= 1.);
    model.set_maximization();
    model.set_objective(x);
    const auto iis = model.compute_iis();
    EXPECT_TRUE(outcome_is<iis_outcome::incomplete>(iis.get_outcome(), false));
    EXPECT_EQ(iis.num_variable_members(), 0u);
    EXPECT_EQ(iis.num_constraint_members(), 0u);
}

// A node limit stops the confirming solve, not the routine itself.
TEST_F(copt_milp_iis_test, node_limit_stops_the_confirming_solve) {
    using namespace operators;
    auto model = this->new_model();
    // market split: 30 binaries, 4 equality rows, infeasible only by branching
    std::mt19937 rng(42);
    std::uniform_int_distribution<int> coef(0, 99);
    auto x = model.add_binary_variables(30);
    for(int i = 0; i < 4; ++i) {
        std::vector<int> row(30);
        int total = 0;
        for(int & a : row) total += (a = coef(rng));
        model.add_constraint(xsum(std::views::iota(0, 30), [&](int j) {
                                 return row[static_cast<std::size_t>(j)] *
                                        x(static_cast<std::size_t>(j));
                             }) == total / 2);
    }
    model.native_api()._check(model.native_model().first,
                              model.native_api().SetIntParam(
                                  model.native_model().second, "NodeLimit", 0));
    const auto iis = model.compute_iis();
    EXPECT_TRUE(outcome_is<iis_outcome::node_limit>(iis.get_outcome(), false));
    EXPECT_EQ(read_int_attr(model, "MipStatus"),
              copt::impl::v1::COPT_MIPSTATUS_NODELIMIT);
}

// A feasible MIP is answered at the first incumbent of the confirming solve,
// which a market split instance otherwise runs for seconds: the solve ends
// interrupted, not optimal nor at the generous limit, and the interrupt does
// not outlive the call.
TEST_F(copt_milp_iis_test, feasible_mip_is_answered_at_its_first_incumbent) {
    auto model = this->new_model();
    TimeLimitTest<copt_milp_test>::build_market_split(model, 5);
    model.set_time_limit(std::chrono::minutes(1));
    const auto iis = model.compute_iis();
    EXPECT_TRUE(outcome_is<iis_outcome::feasible>(iis.get_outcome()));
    EXPECT_EQ(read_int_attr(model, "MipStatus"),
              copt::impl::v1::COPT_MIPSTATUS_INTERRUPTED);
    EXPECT_EQ(read_int_attr(model, "HasMipSol"), 1);
    model.solve();
    EXPECT_TRUE(is<status::optimal>(model.get_status()));
}

// COPT keeps the bounds of a binary column moved outside [0, 1] and calls
// the model infeasible, but its routine names nothing: the column is the IIS
// wherever the rows are, by the bound that excludes the domain alone, or by
// both when the interval lies inside it.
TEST_F(copt_milp_iis_test, binary_column_outside_its_domain_is_the_iis) {
    this->expect_binary_column_is_the_iis<iis_status::member_lower>(2., 3.);
    this->expect_binary_column_is_the_iis<iis_status::member_upper>(-3., -2.);
    this->expect_binary_column_is_the_iis<iis_status::member_both>(0.25, 0.75);
}
