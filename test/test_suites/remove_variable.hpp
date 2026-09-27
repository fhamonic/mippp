#pragma once

#include <gtest/gtest.h>

#include <ranges>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"

#include "assert_helper.hpp"

namespace mippp {

template <typename T>
struct RemoveVariableTest : public T {
    using typename T::model_type;
    static_assert(has_remove_variable<model_type>);
    static_assert(has_enumerable_variables<model_type>);
};
TYPED_TEST_SUITE_P(RemoveVariableTest);
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(RemoveVariableTest);

TYPED_TEST_P(RemoveVariableTest, remove_solve) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x1 = model.add_variable();
        auto x2 = model.add_variable();
        auto x3 = model.add_variable();
        model.set_minimization();
        model.set_objective(5 * x1 + 4 * x2 + 3 * x3);
        model.add_constraint(2 * x1 + 2 * x2 - x3 >= 5);
        model.add_constraint(4 * x1 + x2 + 2 * x3 <= 11);
        model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 == 8);

        model.remove_variable(x2);
        ASSERT_EQ(model.num_variables(), 2);

        model.solve();
        ASSERT_NEAR(model.get_solution_value(), 93.0 / 7.0, TEST_EPSILON);
        {
            auto solution = model.get_solution();
            ASSERT_NEAR(solution[x1], 18.0 / 7.0, TEST_EPSILON);
            ASSERT_NEAR(solution[x3], 1.0 / 7.0, TEST_EPSILON);
        }
    });
}

TYPED_TEST_P(RemoveVariableTest, solve_remove_solve) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x1 = model.add_variable();
        auto x2 = model.add_variable();
        auto x3 = model.add_variable();
        model.set_minimization();
        model.set_objective(5 * x1 + 4 * x2 + 3 * x3);
        model.add_constraint(2 * x1 + 2 * x2 - x3 >= 5);
        model.add_constraint(4 * x1 + x2 + 2 * x3 <= 11);
        model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 == 8);
        model.solve();
        ASSERT_NEAR(model.get_solution_value(), 12.0, TEST_EPSILON);
        {
            auto solution = model.get_solution();
            ASSERT_NEAR(solution[x1], 2.0, TEST_EPSILON);
            ASSERT_NEAR(solution[x2], 0.5, TEST_EPSILON);
            ASSERT_NEAR(solution[x3], 0.0, TEST_EPSILON);
        }

        model.remove_variable(x2);
        ASSERT_EQ(model.num_variables(), 2);

        model.solve();
        ASSERT_NEAR(model.get_solution_value(), 93.0 / 7.0, TEST_EPSILON);
        {
            auto solution = model.get_solution();
            ASSERT_NEAR(solution[x1], 18.0 / 7.0, TEST_EPSILON);
            ASSERT_NEAR(solution[x3], 1.0 / 7.0, TEST_EPSILON);
        }
    });
}

TYPED_TEST_P(RemoveVariableTest, remove_addvar_solve) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x1 = model.add_variable();
        auto x2 = model.add_variable();
        auto x3 = model.add_variable();
        model.set_minimization();
        model.set_objective(5 * x1 + 4 * x2 + 3 * x3);
        model.add_constraint(2 * x1 + 2 * x2 - x3 >= 5);
        model.add_constraint(4 * x1 + x2 + 2 * x3 <= 11);
        model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 == 8);

        model.remove_variable(x2);
        ASSERT_EQ(model.num_variables(), 2);
        auto x4 = model.add_variable(
            {.obj_coef = 2, .lower_bound = -1, .upper_bound = std::nullopt});
        ASSERT_EQ(model.num_variables(), 3);
        ASSERT_EQ(x4, x2);
        // the recycled handle maps to a live column of its own
        ASSERT_NE(model.native_id(x4), model.native_id(x1));
        ASSERT_NE(model.native_id(x4), model.native_id(x3));

        model.solve();
        ASSERT_NEAR(model.get_solution_value(), 93.0 / 7.0 - 2, TEST_EPSILON);
        {
            auto solution = model.get_solution();
            ASSERT_NEAR(solution[x1], 18.0 / 7.0, TEST_EPSILON);
            ASSERT_NEAR(solution[x3], 1.0 / 7.0, TEST_EPSILON);
            ASSERT_NEAR(solution[x4], -1.0, TEST_EPSILON);
        }
    });
}

TYPED_TEST_P(RemoveVariableTest, solve_remove_addvar_solve) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x1 = model.add_variable();
        auto x2 = model.add_variable();
        auto x3 = model.add_variable();
        model.set_minimization();
        model.set_objective(5 * x1 + 4 * x2 + 3 * x3);
        model.add_constraint(2 * x1 + 2 * x2 - x3 >= 5);
        model.add_constraint(4 * x1 + x2 + 2 * x3 <= 11);
        model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 == 8);
        model.solve();
        ASSERT_NEAR(model.get_solution_value(), 12.0, TEST_EPSILON);
        {
            auto solution = model.get_solution();
            ASSERT_NEAR(solution[x1], 2.0, TEST_EPSILON);
            ASSERT_NEAR(solution[x2], 0.5, TEST_EPSILON);
            ASSERT_NEAR(solution[x3], 0.0, TEST_EPSILON);
        }

        model.remove_variable(x2);
        ASSERT_EQ(model.num_variables(), 2);
        auto x4 = model.add_variable(
            {.obj_coef = 2, .lower_bound = -1, .upper_bound = std::nullopt});
        ASSERT_EQ(model.num_variables(), 3);
        ASSERT_EQ(x4, x2);
        // the recycled handle maps to a live column of its own
        ASSERT_NE(model.native_id(x4), model.native_id(x1));
        ASSERT_NE(model.native_id(x4), model.native_id(x3));

        model.solve();
        ASSERT_NEAR(model.get_solution_value(), 93.0 / 7.0 - 2, TEST_EPSILON);
        {
            auto solution = model.get_solution();
            ASSERT_NEAR(solution[x1], 18.0 / 7.0, TEST_EPSILON);
            ASSERT_NEAR(solution[x3], 1.0 / 7.0, TEST_EPSILON);
            ASSERT_NEAR(solution[x4], -1.0, TEST_EPSILON);
        }
    });
}

TYPED_TEST_P(RemoveVariableTest, remove_addcol_solve) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        if constexpr(!has_column_generation<decltype(model)>) GTEST_SKIP();
        auto x1 = model.add_variable();
        auto x2 = model.add_variable();
        auto x3 = model.add_variable();
        model.set_minimization();
        model.set_objective(5 * x1 + 4 * x2 + 3 * x3);
        auto c1 = model.add_constraint(2 * x1 + 2 * x2 - x3 >= 5);
        auto c2 = model.add_constraint(4 * x1 + x2 + 2 * x3 <= 11);
        auto c3 = model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 == 8);

        model.remove_variable(x2);
        ASSERT_EQ(model.num_variables(), 2);
        auto x4 = model.add_column({{c1, 2.0}, {c2, 1.0}, {c3, 4.0}},
                                   {.obj_coef = 4});
        ASSERT_EQ(model.num_variables(), 3);
        ASSERT_EQ(x4, x2);

        model.solve();
        ASSERT_NEAR(model.get_solution_value(), 12.0, TEST_EPSILON);
        {
            auto solution = model.get_solution();
            ASSERT_NEAR(solution[x1], 2.0, TEST_EPSILON);
            ASSERT_NEAR(solution[x2], 0.5, TEST_EPSILON);
            ASSERT_NEAR(solution[x3], 0.0, TEST_EPSILON);
        }
    });
}

TYPED_TEST_P(RemoveVariableTest, solve_remove_addcol_solve) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        if constexpr(!has_column_generation<decltype(model)>) GTEST_SKIP();
        auto x1 = model.add_variable();
        auto x2 = model.add_variable();
        auto x3 = model.add_variable();
        model.set_minimization();
        model.set_objective(5 * x1 + 4 * x2 + 3 * x3);
        auto c1 = model.add_constraint(2 * x1 + 2 * x2 - x3 >= 5);
        auto c2 = model.add_constraint(4 * x1 + x2 + 2 * x3 <= 11);
        auto c3 = model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 == 8);
        model.solve();
        ASSERT_NEAR(model.get_solution_value(), 12.0, TEST_EPSILON);
        {
            auto solution = model.get_solution();
            ASSERT_NEAR(solution[x1], 2.0, TEST_EPSILON);
            ASSERT_NEAR(solution[x2], 0.5, TEST_EPSILON);
            ASSERT_NEAR(solution[x3], 0.0, TEST_EPSILON);
        }

        model.remove_variable(x2);
        ASSERT_EQ(model.num_variables(), 2);
        auto x4 = model.add_column({{c1, 2.0}, {c2, 1.0}, {c3, 4.0}},
                                   {.obj_coef = 4});
        ASSERT_EQ(model.num_variables(), 3);
        ASSERT_EQ(x4, x2);

        model.solve();
        ASSERT_NEAR(model.get_solution_value(), 12.0, TEST_EPSILON);
        {
            auto solution = model.get_solution();
            ASSERT_NEAR(solution[x1], 2.0, TEST_EPSILON);
            ASSERT_NEAR(solution[x2], 0.5, TEST_EPSILON);
            ASSERT_NEAR(solution[x3], 0.0, TEST_EPSILON);
        }
    });
}

TYPED_TEST_P(RemoveVariableTest, remove_addnamedvar_solve) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        if constexpr(!has_named_variables<decltype(model)>) GTEST_SKIP();
        auto x1 = model.add_variable();
        auto x2 = model.add_named_variable("x2");
        auto x3 = model.add_variable();
        model.set_minimization();
        model.set_objective(5 * x1 + 4 * x2 + 3 * x3);
        model.add_constraint(2 * x1 + 2 * x2 - x3 >= 5);
        model.add_constraint(4 * x1 + x2 + 2 * x3 <= 11);
        model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 == 8);

        model.remove_variable(x2);
        ASSERT_EQ(model.num_variables(), 2);
        auto x4 = model.add_named_variable(
            "x4",
            {.obj_coef = 2, .lower_bound = -1, .upper_bound = std::nullopt});
        ASSERT_EQ(model.num_variables(), 3);
        ASSERT_EQ(x4, x2);
        ASSERT_STREQ(model.get_variable_name(x4).c_str(), "x4");

        model.solve();
        ASSERT_NEAR(model.get_solution_value(), 93.0 / 7.0 - 2, TEST_EPSILON);
        {
            auto solution = model.get_solution();
            ASSERT_NEAR(solution[x1], 18.0 / 7.0, TEST_EPSILON);
            ASSERT_NEAR(solution[x3], 1.0 / 7.0, TEST_EPSILON);
            ASSERT_NEAR(solution[x4], -1.0, TEST_EPSILON);
        }
    });
}

TYPED_TEST_P(RemoveVariableTest, solve_remove_addnamedvar_solve) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        if constexpr(!has_named_variables<decltype(model)>) GTEST_SKIP();
        auto x1 = model.add_variable();
        auto x2 = model.add_named_variable("x2");
        auto x3 = model.add_variable();
        model.set_minimization();
        model.set_objective(5 * x1 + 4 * x2 + 3 * x3);
        model.add_constraint(2 * x1 + 2 * x2 - x3 >= 5);
        model.add_constraint(4 * x1 + x2 + 2 * x3 <= 11);
        model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 == 8);
        model.solve();
        ASSERT_NEAR(model.get_solution_value(), 12.0, TEST_EPSILON);
        {
            auto solution = model.get_solution();
            ASSERT_NEAR(solution[x1], 2.0, TEST_EPSILON);
            ASSERT_NEAR(solution[x2], 0.5, TEST_EPSILON);
            ASSERT_NEAR(solution[x3], 0.0, TEST_EPSILON);
        }

        model.remove_variable(x2);
        ASSERT_EQ(model.num_variables(), 2);
        auto x4 = model.add_named_variable(
            "x4",
            {.obj_coef = 2, .lower_bound = -1, .upper_bound = std::nullopt});
        ASSERT_EQ(model.num_variables(), 3);
        ASSERT_EQ(x4, x2);
        ASSERT_STREQ(model.get_variable_name(x4).c_str(), "x4");

        model.solve();
        ASSERT_NEAR(model.get_solution_value(), 93.0 / 7.0 - 2, TEST_EPSILON);
        {
            auto solution = model.get_solution();
            ASSERT_NEAR(solution[x1], 18.0 / 7.0, TEST_EPSILON);
            ASSERT_NEAR(solution[x3], 1.0 / 7.0, TEST_EPSILON);
            ASSERT_NEAR(solution[x4], -1.0, TEST_EPSILON);
        }
    });
}

// The removed column has three entries and the variable recycling it wants
// to grow, so any coefficient left behind caps the optimum. A backend that
// deletes the entries one by one while walking the solver's own index array
// (which Clp packs down on each deletion) leaves the middle one.
TYPED_TEST_P(RemoveVariableTest, remove_three_entries_addvar_solve) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x1 = model.add_variable({.upper_bound = 3.0});
        auto x2 = model.add_variable();
        model.add_constraint(x1 + x2 <= 3);
        model.add_constraint(x1 + 2 * x2 <= 3);
        model.add_constraint(x1 + 3 * x2 <= 3);
        model.remove_variable(x2);
        auto x3 = model.add_variable({.upper_bound = 4.0});
        model.set_maximization();
        model.set_objective(x1 + x3);
        model.solve();
        ASSERT_NEAR(model.get_solution_value(), 7.0, TEST_EPSILON);
        auto solution = model.get_solution();
        ASSERT_NEAR(solution[x1], 3.0, TEST_EPSILON);
        ASSERT_NEAR(solution[x3], 4.0, TEST_EPSILON);
    });
}

// These cases remove the first variable: removing the last one leaves every
// surviving column index equal to its handle id.
TYPED_TEST_P(RemoveVariableTest, remove_settype_solve) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        if constexpr(!milp_model<decltype(model)>) {
            GTEST_SKIP();
        } else {
            auto x0 = model.add_variable();
            auto a = model.add_integer_variable();
            auto b = model.add_variable();
            auto c = model.add_variable();
            auto d = model.add_variable({.lower_bound = 0, .upper_bound = 0.5});
            model.remove_variable(x0);
            model.set_continuous(a);
            model.set_integer(b);
            model.set_binary(c);
            // rows, not bounds: Gurobi rounds the bounds of an integer column
            model.add_constraint(2 * a <= 3);
            model.add_constraint(2 * b <= 5);
            model.add_constraint(2 * c <= 1);
            model.set_maximization();
            model.set_objective(a + b + c + d);

            model.solve();
            ASSERT_NEAR(model.get_solution_value(), 4.0, TEST_EPSILON);
            auto solution = model.get_solution();
            ASSERT_NEAR(solution[a], 1.5, TEST_EPSILON);
            ASSERT_NEAR(solution[b], 2.0, TEST_EPSILON);
            ASSERT_NEAR(solution[c], 0.0, TEST_EPSILON);
            ASSERT_NEAR(solution[d], 0.5, TEST_EPSILON);
        }
    });
}

TYPED_TEST_P(RemoveVariableTest, remove_addindicator_solve) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        if constexpr(!has_indicator_constraints<decltype(model)>) {
            GTEST_SKIP();
        } else {
            auto x0 = model.add_variable();
            auto z = model.add_binary_variable();
            auto w = model.add_binary_variable();
            auto x = model.add_variable({.lower_bound = 0, .upper_bound = 10});
            model.remove_variable(x0);
            model.add_constraint(z >= 1);
            model.add_indicator_constraint(z, true, x <= 1);
            model.set_maximization();
            model.set_objective(x + w);

            model.solve();
            ASSERT_NEAR(model.get_solution_value(), 2.0, TEST_EPSILON);
            auto solution = model.get_solution();
            ASSERT_NEAR(solution[x], 1.0, TEST_EPSILON);
            ASSERT_NEAR(solution[w], 1.0, TEST_EPSILON);
        }
    });
}

namespace detail {
template <typename M>
concept has_candidate_lazy_constraints =
    has_candidate_solution_callback<M> &&
    has_lazy_constraints<candidate_solution_callback_handle_t<M>, M>;

template <typename Test, typename Separate>
void remove_then_cut_off_candidates(Test & test, Separate && separate) {
    auto model = test.new_model();
    if constexpr(!has_candidate_lazy_constraints<decltype(model)>) {
        GTEST_SKIP();
    } else {
        using namespace operators;
        auto x0 = model.add_binary_variable();
        auto a = model.add_binary_variable();
        auto b = model.add_binary_variable();
        auto c = model.add_binary_variable();
        model.remove_variable(x0);
        model.set_maximization();
        model.set_objective(3 * a + 2 * b + c);

        int num_cuts = 0;
        model.set_candidate_solution_callback([&](auto & handle) {
            using namespace operators;
            auto solution = handle.get_solution();
            if(solution[a] + solution[b] + solution[c] <= 2 + TEST_EPSILON)
                return;
            ++num_cuts;
            separate(handle, a + b + c <= 2);
        });
        model.solve();

        ASSERT_GE(num_cuts, 1);
        ASSERT_NEAR(model.get_solution_value(), 5.0, TEST_EPSILON);
        auto solution = model.get_solution();
        ASSERT_NEAR(solution[a], 1.0, TEST_EPSILON);
        ASSERT_NEAR(solution[b], 1.0, TEST_EPSILON);
        ASSERT_NEAR(solution[c], 0.0, TEST_EPSILON);
    }
}
}  // namespace detail

TYPED_TEST_P(RemoveVariableTest, remove_addlazy_solve) {
    this->SkipOnLicenseError([this]() {
        detail::remove_then_cut_off_candidates(
            *this,
            [](auto & handle, auto && lc) { handle.add_lazy_constraint(lc); });
    });
}

TYPED_TEST_P(RemoveVariableTest, remove_addlazy_distinct_variables_solve) {
    this->SkipOnLicenseError([this]() {
        detail::remove_then_cut_off_candidates(
            *this, [](auto & handle, auto && lc) {
                handle.add_lazy_constraint(distinct_variables, lc);
            });
    });
}

// A tail removal keeps the identity mapping on HiGHS and Gurobi, and the
// perforating one then switches to remapped ids.
TYPED_TEST_P(RemoveVariableTest, enumeration_skips_removed_variables) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto xs = model.add_variables(5);
        auto c = model.add_constraint(xsum(xs) >= 1);
        model.remove_variable(xs[4]);
        EXPECT_ENUMERATED_VARIABLES(model,
                                    std::vector{xs[0], xs[1], xs[2], xs[3]});
        model.remove_variable(xs[1]);
        EXPECT_ENUMERATED_VARIABLES(model, std::vector{xs[0], xs[2], xs[3]});
        EXPECT_ENUMERATED_CONSTRAINTS(model, std::vector{c});
    });
}

TYPED_TEST_P(RemoveVariableTest, enumeration_lists_recycled_ids_in_order) {
    this->SkipOnLicenseError([this]() {
        auto model = this->new_model();
        auto xs = model.add_variables(4);
        model.remove_variable(xs[1]);
        auto y = model.add_variable();
        auto zs = model.add_variables(2);
        EXPECT_ENUMERATED_VARIABLES(
            model, std::vector{xs[0], y, xs[2], xs[3], zs[0], zs[1]});
    });
}

TYPED_TEST_P(RemoveVariableTest, solution_reads_every_enumerated_variable) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto xs = model.add_variables(6);
        model.remove_variable(xs[1]);
        model.remove_variable(xs[3]);
        model.add_variable();
        for(auto v : model.variables())
            model.add_constraint(v >= static_cast<double>(v.uid()));
        model.set_minimization();
        model.set_objective(xsum(model.variables()));
        model.solve();
        ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
        auto solution = model.get_solution();
        for(auto v : model.variables())
            EXPECT_NEAR(solution[v], static_cast<double>(v.uid()),
                        TEST_EPSILON);
    });
}

TYPED_TEST_P(RemoveVariableTest,
             removing_every_enumerated_variable_empties_the_model) {
    this->SkipOnLicenseError([this]() {
        auto model = this->new_model();
        model.add_variables(3);
        for(auto v : model.variables()) model.remove_variable(v);
        EXPECT_EQ(model.num_variables(), 0u);
        EXPECT_TRUE(std::ranges::empty(model.variables()));
        model.add_variables(2);
        model.remove_variables(model.variables());
        EXPECT_EQ(model.num_variables(), 0u);
        EXPECT_TRUE(std::ranges::empty(model.variables()));
    });
}

REGISTER_TYPED_TEST_SUITE_P(
    RemoveVariableTest, remove_solve, solve_remove_solve, remove_addvar_solve,
    solve_remove_addvar_solve, remove_addcol_solve, solve_remove_addcol_solve,
    remove_addnamedvar_solve, solve_remove_addnamedvar_solve,
    remove_three_entries_addvar_solve, remove_settype_solve,
    remove_addindicator_solve, remove_addlazy_solve,
    remove_addlazy_distinct_variables_solve,
    enumeration_skips_removed_variables,
    enumeration_lists_recycled_ids_in_order,
    solution_reads_every_enumerated_variable,
    removing_every_enumerated_variable_empties_the_model);

}  // namespace mippp