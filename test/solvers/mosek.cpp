#include <optional>
#include <stdexcept>
#include <utility>
#include <variant>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/solvers/mosek/all.hpp"

using namespace mippp;

#include "test_suites/all.hpp"

MIPPP_API_VERSION_TEST(MOSEK_api, mosek_api, "MOSEK")

namespace {
struct fake_mosek_release_api {
    std::vector<char> & released;
    int deletetask(mosek::impl::v1::MSKtask_t *) const {
        released.push_back('t');
        return -1;
    }
    int deleteenv(mosek::impl::v1::MSKenv_t *) const {
        released.push_back('e');
        return -1;
    }
};

struct mosek_slot_probe : mosek::impl::v1::mosek_base {
    using mosek_base::_has_primal_solution;
    using mosek_base::_pick_continuous_slot;
};

template <typename Model>
void expect_column_less_status_follows_rows(Model empty, Model satisfied,
                                            Model violated) {
    using namespace operators;
    constexpr auto & no_terms =
        empty_linear_expression<model_variable_t<Model>, double>;
    satisfied.add_constraint(no_terms <= 1.0);
    violated.add_constraint(no_terms >= 1.0);
    empty.solve();
    satisfied.solve();
    violated.solve();
    EXPECT_TRUE(is_a<status::optimal>(empty.get_status()));
    EXPECT_TRUE(is_a<status::optimal>(satisfied.get_status()));
    EXPECT_FALSE(is_a<status::optimal>(violated.get_status()));
}

int iteration_count(const mosek_api & api, mosek_lp & model,
                    mosek::impl::v1::MSKiinfiteme item) {
    mosek::impl::v1::MSKint32t count;
    api._check(api.getintinf(model.native_model().second, item, &count));
    return count;
}
int optimizer(const mosek_api & api, mosek_lp & model) {
    mosek::impl::v1::MSKint32t value;
    api._check(api.getintparam(model.native_model().second,
                               mosek::impl::v1::MSK_IPAR_OPTIMIZER, &value));
    return value;
}

template <typename Model>
void expect_reset_status_drops_the_solution(Model model) {
    using namespace operators;
    auto x = model.add_variable({.upper_bound = 3});
    auto y = model.add_variable();
    model.set_maximization();
    model.set_objective(x + y);
    model.add_constraint(x + 2 * y <= 4);
    model.solve();
    ASSERT_NEAR(model.get_solution_value(), 3.5, TEST_EPSILON);
    model.reset_status();
    EXPECT_THROW(model.get_solution_value(), solve_required_error);
    EXPECT_THROW(model.get_solution(), solve_required_error);
    model.solve();
    ASSERT_NEAR(model.get_solution_value(), 3.5, TEST_EPSILON);
    auto solution = model.get_solution();
    EXPECT_NEAR(solution[x], 3.0, TEST_EPSILON);
    EXPECT_NEAR(solution[y], 0.5, TEST_EPSILON);
}
}  // namespace

TEST(MOSEK_handle_guard, releases_partial_allocations_without_throwing) {
    using namespace mosek::impl::v1;
    int storage = 0;
    for(unsigned allocated : {0u, 1u, 2u}) {
        std::vector<char> released;
        const fake_mosek_release_api api{released};
        MSKenv_t env = nullptr;
        MSKtask_t task = nullptr;
        try {
            mippp::detail::handle_guard<fake_mosek_release_api, MSKenv_t,
                                        MSKtask_t, mosek_handle_release>
                guard(api, env, task);
            if(allocated >= 1) env = reinterpret_cast<MSKenv_t>(&storage);
            if(allocated >= 2) task = reinterpret_cast<MSKtask_t>(&storage);
            throw std::runtime_error("construction failure");
        } catch(const std::runtime_error & e) {
            EXPECT_STREQ(e.what(), "construction failure");
        }
        EXPECT_EQ(env, nullptr);
        EXPECT_EQ(task, nullptr);
        const std::vector<char> expected =
            allocated == 2   ? std::vector<char>{'t', 'e'}
            : allocated == 1 ? std::vector<char>{'e'}
                             : std::vector<char>{};
        EXPECT_EQ(released, expected);
    }
}

TEST(MOSEK_slots, primal_states_carry_a_solution) {
    using namespace mosek::impl::v1;
    for(auto state : {MSK_SOL_STA_OPTIMAL, MSK_SOL_STA_INTEGER_OPTIMAL,
                      MSK_SOL_STA_PRIM_FEAS, MSK_SOL_STA_PRIM_AND_DUAL_FEAS})
        EXPECT_TRUE(mosek_slot_probe::_has_primal_solution(state));
    for(auto state :
        {MSK_SOL_STA_UNKNOWN, MSK_SOL_STA_DUAL_FEAS,
         MSK_SOL_STA_PRIM_INFEAS_CER, MSK_SOL_STA_DUAL_INFEAS_CER,
         MSK_SOL_STA_PRIM_ILLPOSED_CER, MSK_SOL_STA_DUAL_ILLPOSED_CER})
        EXPECT_FALSE(mosek_slot_probe::_has_primal_solution(state));
}

TEST(MOSEK_slots, primal_feasible_slot_ranks_above_dual_only) {
    using namespace mosek::impl::v1;
    const auto pick = [](MSKsolstae basic, MSKsolstae interior) {
        return mosek_slot_probe::_pick_continuous_slot(basic, interior);
    };
    EXPECT_EQ(pick(MSK_SOL_STA_DUAL_FEAS, MSK_SOL_STA_PRIM_FEAS), MSK_SOL_ITR);
    EXPECT_EQ(pick(MSK_SOL_STA_PRIM_FEAS, MSK_SOL_STA_DUAL_FEAS), MSK_SOL_BAS);
    EXPECT_EQ(pick(MSK_SOL_STA_DUAL_FEAS, MSK_SOL_STA_PRIM_AND_DUAL_FEAS),
              MSK_SOL_ITR);
    EXPECT_EQ(pick(MSK_SOL_STA_PRIM_FEAS, MSK_SOL_STA_OPTIMAL), MSK_SOL_ITR);
    EXPECT_EQ(pick(MSK_SOL_STA_PRIM_INFEAS_CER, MSK_SOL_STA_PRIM_FEAS),
              MSK_SOL_BAS);
    EXPECT_EQ(pick(MSK_SOL_STA_UNKNOWN, MSK_SOL_STA_DUAL_FEAS), MSK_SOL_ITR);
    EXPECT_EQ(pick(MSK_SOL_STA_OPTIMAL, MSK_SOL_STA_OPTIMAL), MSK_SOL_BAS);
    EXPECT_EQ(pick(MSK_SOL_STA_PRIM_FEAS, MSK_SOL_STA_PRIM_AND_DUAL_FEAS),
              MSK_SOL_BAS);
}

TEST(MOSEK_slots, a_missing_slot_is_never_picked) {
    using namespace mosek::impl::v1;
    EXPECT_FALSE(
        mosek_slot_probe::_pick_continuous_slot(std::nullopt, std::nullopt));
    EXPECT_EQ(mosek_slot_probe::_pick_continuous_slot(MSK_SOL_STA_UNKNOWN,
                                                      std::nullopt),
              MSK_SOL_BAS);
    EXPECT_EQ(mosek_slot_probe::_pick_continuous_slot(std::nullopt,
                                                      MSK_SOL_STA_UNKNOWN),
              MSK_SOL_ITR);
}

struct mosek_lp_test : public model_test<mosek_api, mosek_lp> {
    static void SetUpTestSuite() { construct_api("MOSEK"); }
};
static_assert(!has_iis<mosek_lp>);
static_assert(has_lp_basis_warm_start<mosek_lp>);
TEST_F(mosek_lp_test, license_codes_throw_license_error) {
    for(int code = 1000; code <= 1028; ++code)
        EXPECT_THROW(api->_check(code), license_error) << code;
    for(int code : {999, 1029, 1050}) {
        try {
            api->_check(code);
            ADD_FAILURE() << code << " did not throw";
        } catch(const license_error &) {
            ADD_FAILURE() << code << " is not a license code";
        } catch(const solver_error &) {
        }
    }
}
TEST_F(mosek_lp_test, column_less_model_is_optimal_only_if_its_rows_hold) {
    SkipOnLicenseError([this]() {
        expect_column_less_status_follows_rows(new_model(), new_model(),
                                               new_model());
    });
}
TEST_F(mosek_lp_test, reset_status_drops_the_solution) {
    SkipOnLicenseError(
        [this]() { expect_reset_status_drops_the_solution(new_model()); });
}
// Under a raised MSK_DPAR_DATA_TOL_BOUND_INF, infinity() passed as a value
// would stay a finite bound that the getters still read as infinite: only the
// bound key tells the side was freed.
TEST_F(mosek_lp_test,
       infinity_frees_a_row_side_under_a_raised_bound_tolerance) {
    using namespace operators;
    using namespace mosek::impl::v1;
    auto model = new_model();
    const MSKtask_t task = model.native_model().second;
    api->_check(api->putdouparam(task, MSK_DPAR_DATA_TOL_BOUND_INF, 1e40));
    auto x = model.add_variable({.lower_bound = -7.0, .upper_bound = 7.0});
    auto c1 = model.add_constraint(x == 0.0);
    auto c2 = model.add_constraint(x == 0.0);
    model.set_constraint_lower_bound(c1, -model.infinity());
    model.set_constraint_upper_bound(c2, model.infinity());
    MSKboundkeye key;
    double lb, ub;
    api->_check(api->getconbound(task, c1.id(), &key, &lb, &ub));
    EXPECT_EQ(key, MSK_BK_UP);
    api->_check(api->getconbound(task, c2.id(), &key, &lb, &ub));
    EXPECT_EQ(key, MSK_BK_LO);
    model.set_constraint_upper_bound(c1, model.infinity());
    api->_check(api->getconbound(task, c1.id(), &key, &lb, &ub));
    EXPECT_EQ(key, MSK_BK_FR);
}
TEST_F(mosek_lp_test,
       infinity_frees_a_column_side_under_a_raised_bound_tolerance) {
    using namespace mosek::impl::v1;
    auto model = new_model();
    const MSKtask_t task = model.native_model().second;
    api->_check(api->putdouparam(task, MSK_DPAR_DATA_TOL_BOUND_INF, 1e40));
    auto x = model.add_variable({.lower_bound = -7.0, .upper_bound = 7.0});
    auto y = model.add_variable({.lower_bound = -7.0, .upper_bound = 7.0});
    model.set_variable_lower_bound(x, -model.infinity());
    model.set_variable_upper_bound(y, model.infinity());
    MSKboundkeye key;
    double lb, ub;
    api->_check(api->getvarbound(task, x.id(), &key, &lb, &ub));
    EXPECT_EQ(key, MSK_BK_UP);
    api->_check(api->getvarbound(task, y.id(), &key, &lb, &ub));
    EXPECT_EQ(key, MSK_BK_LO);
    model.set_variable_upper_bound(x, model.infinity());
    api->_check(api->getvarbound(task, x.id(), &key, &lb, &ub));
    EXPECT_EQ(key, MSK_BK_FR);
    EXPECT_EQ(model.get_variable_lower_bound(x), -model.infinity());
    EXPECT_EQ(model.get_variable_upper_bound(x), model.infinity());
}
// mosek_lp has no iteration limit: MOSEK's own counts prove the warm start
TEST_F(mosek_lp_test, set_basis_starts_the_next_solve_at_the_optimum) {
    using namespace mosek::impl::v1;
    SkipOnLicenseError([this]() {
        auto a = new_model();
        lp_basis_cases::build_dense(a);
        a.solve();
        ASSERT_TRUE(is_a<status::optimal>(a.get_status()));
        ASSERT_GT(iteration_count(*api, a, MSK_IINF_INTPNT_ITER), 0);
        auto b = new_model();
        const auto x = lp_basis_cases::build_dense(b);
        b.set_basis(a.get_basis());
        b.solve();
        ASSERT_TRUE(is_a<status::optimal>(b.get_status()));
        ASSERT_NEAR(b.get_solution_value(), a.get_solution_value(),
                    lp_basis_cases::tolerance(a.get_solution_value()));
        ASSERT_EQ(iteration_count(*api, b, MSK_IINF_SIM_PRIMAL_ITER) +
                      iteration_count(*api, b, MSK_IINF_SIM_DUAL_ITER),
                  0);
        ASSERT_EQ(iteration_count(*api, b, MSK_IINF_INTPNT_ITER), 0);
        ASSERT_EQ(optimizer(*api, b), MSK_OPTIMIZER_FREE);
        b.set_variable_upper_bound(x[0], 1.0);
        b.solve();
        ASSERT_TRUE(is_a<status::optimal>(b.get_status()));
        ASSERT_GT(iteration_count(*api, b, MSK_IINF_INTPNT_ITER), 0);
    });
}
TEST_F(mosek_lp_test, set_basis_keeps_an_optimizer_chosen_natively) {
    using namespace mosek::impl::v1;
    SkipOnLicenseError([this]() {
        auto a = new_model();
        lp_basis_cases::build_dense(a);
        a.solve();
        auto b = new_model();
        lp_basis_cases::build_dense(b);
        api->_check(api->putintparam(b.native_model().second,
                                     MSK_IPAR_OPTIMIZER, MSK_OPTIMIZER_INTPNT));
        b.set_basis(a.get_basis());
        b.solve();
        ASSERT_TRUE(is_a<status::optimal>(b.get_status()));
        ASSERT_GT(iteration_count(*api, b, MSK_IINF_INTPNT_ITER), 0);
        ASSERT_EQ(optimizer(*api, b), MSK_OPTIMIZER_INTPNT);
    });
}
// MOSEK stores the keys as written, which checks the keys set_basis()
// writes without the solve get_basis() needs
TEST_F(mosek_lp_test, set_basis_writes_the_keys_before_a_solve) {
    using namespace mosek::impl::v1;
    SkipOnLicenseError([this]() {
        using namespace operators;
        const auto basic_keys = [](mosek_lp & m) {
            std::vector<MSKstakeye> skx(m.num_variables()),
                skc(m.num_constraints());
            api->_check(api->getsolution(
                m.native_model().second, MSK_SOL_BAS, nullptr, nullptr,
                skc.data(), skx.data(), nullptr, nullptr, nullptr, nullptr,
                nullptr, nullptr, nullptr, nullptr, nullptr));
            return std::pair{skx, skc};
        };
        auto solved = new_model();
        const auto v = lp_basis_cases::build_vertex(solved);
        solved.solve();
        const auto basis = solved.get_basis();
        ASSERT_TRUE(is<basis_status::nonbasic_fixed>(basis.get_status(v.x2)));
        ASSERT_TRUE(is<basis_status::nonbasic_fixed>(basis.get_status(v.eq)));
        auto model = new_model();
        lp_basis_cases::build_vertex(model);
        const auto y = model.add_variable();
        const auto c = model.add_constraint(y + v.x3 <= 50);
        model.set_basis(basis);
        // both models hold build_vertex's entities first, in the same order
        const auto [solved_skx, solved_skc] = basic_keys(solved);
        const auto [skx, skc] = basic_keys(model);
        for(std::size_t j = 0; j < solved_skx.size(); ++j)
            EXPECT_EQ(skx[j], solved_skx[j]);
        for(std::size_t i = 0; i < solved_skc.size(); ++i)
            EXPECT_EQ(skc[i], solved_skc[i]);
        EXPECT_EQ(skx[static_cast<std::size_t>(model.native_id(y))],
                  MSK_SK_LOW);
        EXPECT_EQ(skc[static_cast<std::size_t>(model.native_id(c))],
                  MSK_SK_BAS);
    });
}
// an empty vector's data() may be null, on the dimension set_basis() writes
// nothing for
TEST_F(mosek_lp_test, set_basis_accepts_a_model_with_an_empty_dimension) {
    SkipOnLicenseError([this]() {
        using namespace operators;
        auto columns_only = new_model();
        columns_only.add_variable();
        columns_only.add_variable();
        auto rows_only = new_model();
        constexpr auto & no_terms =
            empty_linear_expression<model_variable_t<mosek_lp>, double>;
        rows_only.add_constraint(no_terms <= 1.0);
        for(mosek_lp * model : {&columns_only, &rows_only}) {
            ASSERT_NO_THROW(model->set_basis(model_basis_t<mosek_lp>{}));
            model->solve();
            ASSERT_TRUE(is_a<status::optimal>(model->get_status()));
        }
    });
}
// Keys written past mippp after a solve, which get_basis() needs
TEST_F(mosek_lp_test, get_basis_throws_on_an_unknown_or_infeasible_key) {
    using namespace mosek::impl::v1;
    SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = new_model();
        const auto x = model.add_variable();
        const auto y = model.add_variable();
        model.add_constraint(x + y <= 1);
        model.solve();
        ASSERT_NO_THROW(model.get_basis());
        const auto task = model.native_model().second;
        const std::vector<MSKstakeye> columns(2, MSK_SK_LOW);
        const std::vector<MSKstakeye> rows(1, MSK_SK_BAS);
        const std::vector<MSKstakeye> unknown_row(1, MSK_SK_UNK);
        api->_check(api->putskx(task, MSK_SOL_BAS, columns.data()));
        api->_check(api->putskc(task, MSK_SOL_BAS, unknown_row.data()));
        ASSERT_THROW(model.get_basis(), solver_error);
        const std::vector<MSKstakeye> infeasible{MSK_SK_INF, MSK_SK_LOW};
        api->_check(api->putskx(task, MSK_SOL_BAS, infeasible.data()));
        api->_check(api->putskc(task, MSK_SOL_BAS, rows.data()));
        ASSERT_THROW(model.get_basis(), solver_error);
        api->_check(api->putskx(task, MSK_SOL_BAS, columns.data()));
        ASSERT_NO_THROW(model.get_basis());
    });
}
INSTANTIATE_TEST(MOSEK_lp, LpModelTest, mosek_lp_test);
INSTANTIATE_TEST(MOSEK_lp, EnumerableEntitiesTest, mosek_lp_test);
INSTANTIATE_TEST(MOSEK_lp, ReadableObjectiveTest, mosek_lp_test);
INSTANTIATE_TEST(MOSEK_lp, ModifiableObjectiveTest, mosek_lp_test);
INSTANTIATE_TEST(MOSEK_lp, ReadableVariablesBoundsTest, mosek_lp_test);
INSTANTIATE_TEST(MOSEK_lp, ModifiableVariablesBoundsTest, mosek_lp_test);
INSTANTIATE_TEST(MOSEK_lp, NamedVariablesTest, mosek_lp_test);
INSTANTIATE_TEST(MOSEK_lp, AddColumnTest, mosek_lp_test);
INSTANTIATE_TEST(MOSEK_lp, ReadableConstraintBoundsTest, mosek_lp_test);
INSTANTIATE_TEST(MOSEK_lp, ModifiableConstraintBoundsTest, mosek_lp_test);
INSTANTIATE_TEST(MOSEK_lp, IisByDeletionTest, mosek_lp_test);
INSTANTIATE_TEST(MOSEK_lp, DualSolutionTest, mosek_lp_test);
INSTANTIATE_TEST(MOSEK_lp, ReducedCostsTest, mosek_lp_test);
INSTANTIATE_TEST(MOSEK_lp, LpStatusTest, mosek_lp_test);
INSTANTIATE_TEST(MOSEK_lp, CuttingStockTest, mosek_lp_test);
INSTANTIATE_TEST(MOSEK_lp, TimeLimitTest, mosek_lp_test);
INSTANTIATE_TEST(MOSEK_lp, VerbosityTest, mosek_lp_test);
INSTANTIATE_TEST(MOSEK_lp, ThreadLimitTest, mosek_lp_test);
INSTANTIATE_TEST(MOSEK_lp, LpBasisTest, mosek_lp_test);
INSTANTIATE_TEST(MOSEK_lp, LpBasisWarmStartTest, mosek_lp_test);

struct mosek_milp_test : public model_test<mosek_api, mosek_milp> {
    static void SetUpTestSuite() { construct_api("MOSEK"); }
};
static_assert(!has_iis<mosek_milp>);
static_assert(!has_lp_basis<mosek_milp>);
// The code set_basis writes for each tag the statuses list, pinned without a
// library: a tag that loses its line in the table writes as its nearest
// base, which still compiles.
struct mosek_basis_codes : mosek_lp {
    using column_status = model_variable_basis_status_t<mosek_lp>;
    using row_status = model_constraint_basis_status_t<mosek_lp>;
    template <typename Tag>
    static constexpr auto column() {
        return _basis_key(column_status(std::in_place_type<Tag>));
    }
    template <typename Tag>
    static constexpr auto row() {
        return _basis_key(row_status(std::in_place_type<Tag>));
    }
};
TEST(mosek_lp_basis_codes, one_per_listed_tag) {
    using namespace mosek::impl::v1;
    using namespace basis_status;
    using codes = mosek_basis_codes;
    // clang-format off
    static_assert(std::variant_size_v<codes::column_status> == 6);
    static_assert(std::variant_size_v<codes::row_status>    == 6);
    static_assert(codes::column<basic>()                   == MSK_SK_BAS);
    static_assert(codes::column<nonbasic_at_lower_bound>() == MSK_SK_LOW);
    static_assert(codes::column<nonbasic_at_upper_bound>() == MSK_SK_UPR);
    static_assert(codes::column<nonbasic_fixed>()          == MSK_SK_FIX);
    static_assert(codes::column<superbasic>()              == MSK_SK_SUPBAS);
    static_assert(codes::column<nonbasic>()                == MSK_SK_LOW);
    static_assert(codes::row<basic>()                   == MSK_SK_BAS);
    static_assert(codes::row<nonbasic_at_lower_bound>() == MSK_SK_LOW);
    static_assert(codes::row<nonbasic_at_upper_bound>() == MSK_SK_UPR);
    static_assert(codes::row<nonbasic_fixed>()          == MSK_SK_FIX);
    static_assert(codes::row<superbasic>()              == MSK_SK_SUPBAS);
    static_assert(codes::row<nonbasic>()                == MSK_SK_LOW);
    // clang-format on
}
TEST_F(mosek_milp_test, column_less_model_is_optimal_only_if_its_rows_hold) {
    SkipOnLicenseError([this]() {
        expect_column_less_status_follows_rows(new_model(), new_model(),
                                               new_model());
    });
}
TEST_F(mosek_milp_test, reset_status_drops_the_solution) {
    SkipOnLicenseError(
        [this]() { expect_reset_status_drops_the_solution(new_model()); });
}
INSTANTIATE_TEST(MOSEK_milp, LpModelTest, mosek_milp_test);
INSTANTIATE_TEST(MOSEK_milp, MilpModelTest, mosek_milp_test);
INSTANTIATE_TEST(MOSEK_milp, EnumerableEntitiesTest, mosek_milp_test);
INSTANTIATE_TEST(MOSEK_milp, ReadableObjectiveTest, mosek_milp_test);
INSTANTIATE_TEST(MOSEK_milp, ModifiableObjectiveTest, mosek_milp_test);
INSTANTIATE_TEST(MOSEK_milp, ReadableVariablesBoundsTest, mosek_milp_test);
INSTANTIATE_TEST(MOSEK_milp, ModifiableVariablesBoundsTest, mosek_milp_test);
INSTANTIATE_TEST(MOSEK_milp, NamedVariablesTest, mosek_milp_test);
INSTANTIATE_TEST(MOSEK_milp, AddColumnTest, mosek_milp_test);
INSTANTIATE_TEST(MOSEK_milp, ReadableConstraintBoundsTest, mosek_milp_test);
INSTANTIATE_TEST(MOSEK_milp, ModifiableConstraintBoundsTest, mosek_milp_test);
INSTANTIATE_TEST(MOSEK_milp, IisByDeletionTest, mosek_milp_test);
INSTANTIATE_TEST(MOSEK_milp, SudokuTest, mosek_milp_test);
// INSTANTIATE_TEST(MOSEK_milp, MipStartTest, mosek_milp_test);
INSTANTIATE_TEST(MOSEK_milp, TimeLimitTest, mosek_milp_test);
INSTANTIATE_TEST(MOSEK_milp, TimeLimitIncumbentTest, mosek_milp_test);
// INSTANTIATE_TEST(MOSEK_milp, OptimalityToleranceTest, mosek_milp_test);
// INSTANTIATE_TEST(MOSEK_milp, MipGapTest, mosek_milp_test);
INSTANTIATE_TEST(MOSEK_milp, DualBoundTest, mosek_milp_test);
INSTANTIATE_TEST(MOSEK_milp, NodeCountTest, mosek_milp_test);
// INSTANTIATE_TEST(MOSEK_milp, IntegralityToleranceTest, mosek_milp_test);
INSTANTIATE_TEST(MOSEK_milp, VerbosityTest, mosek_milp_test);
INSTANTIATE_TEST(MOSEK_milp, ProgressCallbackTest, mosek_milp_test);
INSTANTIATE_TEST(MOSEK_milp, ProgressCallbackNodeCountTest, mosek_milp_test);
// no candidate solution callback
// INSTANTIATE_TEST(MOSEK_milp, ProgressCallbackSharingTest, mosek_milp_test);
INSTANTIATE_TEST(MOSEK_milp, ThreadLimitTest, mosek_milp_test);
