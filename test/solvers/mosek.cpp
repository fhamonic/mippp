#include <optional>
#include <stdexcept>
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
    EXPECT_THROW(model.get_solution_value(), solver_error);
    EXPECT_THROW(model.get_solution(), solver_error);
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

struct mosek_milp_test : public model_test<mosek_api, mosek_milp> {
    static void SetUpTestSuite() { construct_api("MOSEK"); }
};
static_assert(!has_iis<mosek_milp>);
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
// INSTANTIATE_TEST(MOSEK_milp, IntegralityToleranceTest, mosek_milp_test);
INSTANTIATE_TEST(MOSEK_milp, VerbosityTest, mosek_milp_test);
