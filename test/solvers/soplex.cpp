#include <initializer_list>
#include <limits>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/solvers/soplex/all.hpp"

using namespace mippp;

#include "test_suites/all.hpp"

MIPPP_API_VERSION_TEST(SoPlex_api, soplex_api, "SOPLEX")

struct soplex_lp_test : public model_test<soplex_api, soplex_lp> {
    static void SetUpTestSuite() { construct_api("SOPLEX"); }
};
// The column's single entry sits on row 1, at its nonzero count: a length
// taken from that count would drop it.
TEST_F(soplex_lp_test, column_entry_past_its_nonzero_count_is_kept) {
    using namespace operators;
    auto model = new_model();
    auto x = model.add_variable();
    model.add_constraint(x <= 4.0);
    auto c1 = model.add_constraint(x <= 4.0);
    auto y = model.add_column({{c1, 2.0}});
    model.set_maximization();
    model.set_objective(y);
    model.solve();
    ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
    EXPECT_NEAR(model.get_solution_value(), 2.0, TEST_EPSILON);
}
// SoPlex reports this LP unbounded when its absent column sides reach it as
// IEEE infinity instead of its own 1e100.
TEST_F(soplex_lp_test, ieee_infinity_is_an_absent_column_side) {
    using namespace operators;
    constexpr double inf = std::numeric_limits<double>::infinity();
    auto model = new_model();
    auto x = model.add_variable({.lower_bound = -inf, .upper_bound = 3.0});
    auto y = model.add_variable({.lower_bound = 1.0, .upper_bound = inf});
    auto z = model.add_variable({.lower_bound = -inf, .upper_bound = 1.0});
    model.add_constraint(x >= -2.0);
    model.add_constraint(x - y <= 5.0);
    model.set_maximization();
    model.set_objective(-2 * x - y + 2 * z);
    model.solve();
    ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
    EXPECT_NEAR(model.get_solution_value(), 5.0, TEST_EPSILON);
    EXPECT_EQ(model.get_variable_lower_bound(x), -model.infinity());
    EXPECT_EQ(model.get_variable_upper_bound(y), model.infinity());
    model.set_variable_lower_bound(z, 0.0);
    model.set_variable_lower_bound(z, -inf);
    model.solve();
    ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
    EXPECT_NEAR(model.get_solution_value(), 5.0, TEST_EPSILON);
    EXPECT_EQ(model.get_variable_lower_bound(z), -model.infinity());
}
// The coefficients give the columns scale exponents of both signs, and the
// solve scales the LP in place: an infinite bound written as 1e100 then read
// back as 1.2e96 or 2.6e102.
TEST_F(soplex_lp_test, infinite_column_bounds_survive_the_scaling) {
    using namespace operators;
    auto model = new_model();
    auto x = model.add_variable({});
    auto y = model.add_variable({});
    auto z = model.add_variable({});
    model.add_constraint(1e3 * x + 1e-3 * y + 5e2 * z == 1.0);
    model.add_constraint(2e3 * x - 3e-3 * y + 1e-4 * z <= 5.0);
    model.add_constraint(7e3 * x + 2e-3 * y - 3e2 * z >= -2.0);
    model.set_minimization();
    model.set_objective(x + y + z);
    model.solve();
    ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
    for(auto v : {x, y, z}) {
        EXPECT_EQ(model.get_variable_lower_bound(v), -model.infinity());
        EXPECT_EQ(model.get_variable_upper_bound(v), model.infinity());
    }
    model.set_variable_lower_bound(x, 3.0);
    model.set_variable_lower_bound(x, -model.infinity());
    model.set_variable_upper_bound(y, 2e100);
    model.set_variable_lower_bound(z, 3.0);
    EXPECT_EQ(model.get_variable_lower_bound(x), -model.infinity());
    EXPECT_EQ(model.get_variable_upper_bound(y), model.infinity());
    EXPECT_EQ(model.get_variable_lower_bound(z), 3.0);
}
// Each relaxation leaves the freed row nonbasic: SoPlex's own warm start then
// stops with running after the first and reports 7 after the second. The
// reload clears the LP, which resets the sense to maximization.
TEST_F(soplex_lp_test, freeing_an_active_row_keeps_the_resolve_right) {
    using namespace operators;
    for(const bool maximize : {true, false}) {
        const double sign = maximize ? 1.0 : -1.0;
        for(const bool first_row : {true, false}) {
            auto model = new_model();
            auto x =
                model.add_variable({.lower_bound = 0.0, .upper_bound = 10.0});
            auto y =
                model.add_variable({.lower_bound = 0.0, .upper_bound = 10.0});
            auto z =
                model.add_variable({.lower_bound = 0.0, .upper_bound = 10.0});
            auto c1 = model.add_constraint(x + y <= 4.0);
            auto c2 = model.add_constraint(x - y >= -2.0);
            model.add_constraint(z == 1.0);
            if(maximize) {
                model.set_maximization();
                model.set_objective(x + 2 * y + z);
            } else {
                model.set_minimization();
                model.set_objective(-x - 2 * y - z);
            }
            model.solve();
            ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
            ASSERT_NEAR(model.get_solution_value(), sign * 8.0, TEST_EPSILON);
            if(first_row)
                model.set_constraint_upper_bound(c1, model.infinity());
            else
                model.set_constraint_lower_bound(c2, -model.infinity());
            model.solve();
            ASSERT_TRUE(is_a<status::optimal>(model.get_status()))
                << maximize << first_row;
            EXPECT_NEAR(model.get_solution_value(),
                        sign * (first_row ? 31.0 : 9.0), TEST_EPSILON)
                << maximize << first_row;
            EXPECT_EQ(model.get_variable_upper_bound(x), 10.0);
            EXPECT_EQ(model.get_constraint_upper_bound(c1),
                      first_row ? model.infinity() : 4.0);
            model.set_constraint_upper_bound(c1, 4.0);
            model.set_constraint_lower_bound(c2, -2.0);
            model.solve();
            ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
            EXPECT_NEAR(model.get_solution_value(), sign * 8.0, TEST_EPSILON);
        }
    }
}
INSTANTIATE_TEST(SoPlex, LpModelTest, soplex_lp_test);
INSTANTIATE_TEST(SoPlex, EnumerableEntitiesTest, soplex_lp_test);
INSTANTIATE_TEST(SoPlex, ReadableObjectiveTest, soplex_lp_test);
INSTANTIATE_TEST(SoPlex, ReadableVariablesBoundsTest, soplex_lp_test);
INSTANTIATE_TEST(SoPlex, ModifiableVariablesBoundsTest, soplex_lp_test);
INSTANTIATE_TEST(SoPlex, AddColumnTest, soplex_lp_test);
INSTANTIATE_TEST(SoPlex, ReadableConstraintBoundsTest, soplex_lp_test);
INSTANTIATE_TEST(SoPlex, ModifiableConstraintBoundsTest, soplex_lp_test);
INSTANTIATE_TEST(SoPlex, DualSolutionTest, soplex_lp_test);
INSTANTIATE_TEST(SoPlex, CuttingStockTest, soplex_lp_test);
INSTANTIATE_TEST(SoPlex, TimeLimitTest, soplex_lp_test);
INSTANTIATE_TEST(SoPlex, VerbosityTest, soplex_lp_test);
