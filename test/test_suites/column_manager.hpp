#pragma once

#include <gtest/gtest.h>
#include "assert_helper.hpp"

#include <cmath>
#include <cstddef>
#include <optional>
#include <ranges>
#include <utility>
#include <vector>

#include "melon/algorithm/unbounded_knapsack_bnb.hpp"

#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/utility/column_manager.hpp"

namespace mippp {

template <typename T>
struct ColumnManagerTest : public T {
    using typename T::model_type;
    static_assert(lp_model<model_type>);
    static_assert(has_dual_solution<model_type>);
    static_assert(has_column_generation<model_type>);
    static_assert(has_remove_variable<model_type>);
    static_assert(has_enumerable_variables<model_type>);
};
TYPED_TEST_SUITE_P(ColumnManagerTest);
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(ColumnManagerTest);

namespace column_manager_test {

// cutting stock: 4 orders of (quantity, length) to cut from rolls of 100
using pattern_t = std::vector<int>;  // order_id -> quantity satisfied
constexpr int roll_length = 100;
inline const std::vector<std::pair<int, int>> orders = {
    {211, 14}, {395, 31}, {610, 36}, {97, 45}};
constexpr double optimal_value = 452.25;

// Runs column generation to convergence, each round activating every
// improving column first, then, if evict_tolerance is set and something was
// activated, evicting the master columns whose reduced cost exceeds it.
template <typename M, typename Manager>
void cutting_stock_column_generation(M & model, Manager & columns,
                                     std::optional<double> evict_tolerance) {
    using namespace operators;
    auto order_ids = std::ranges::iota_view(0ul, orders.size());

    model.set_minimization();
    for(auto order_id : order_ids) {
        pattern_t pattern(orders.size(), 0);
        pattern[order_id] = roll_length / orders[order_id].second;
        columns.emplace_master_column(
            std::move(pattern),
            model.add_variable({.obj_coef = 1, .lower_bound = 0}));
    }
    auto satisfaction_constrs =
        model.add_constraints(order_ids, [&](auto order_id) {
            auto && demanded_quantity = orders[order_id].first;
            // `using namespace operators` again, inside the lambda:
            // the function-level directive is enough for GCC, Clang and a
            // one-level lambda on MSVC, but MSVC 14.41 loses it two generic
            // lambdas deep and then finds no operator* for scalar * variable.
            return xsum(columns.master_columns(),
                        [&, order_id](auto && column) {
                            using namespace operators;
                            auto && [pattern, state] = column;
                            return pattern[order_id] * state.var;
                        }) >= demanded_quantity;
        });

    auto add_pattern_column = [&](auto & model_, const auto & entry) {
        const pattern_t & pattern = entry.seed;
        return model_.add_column(
            std::views::transform(
                std::views::filter(
                    order_ids,
                    [&](auto order_id) { return pattern[order_id] != 0; }),
                [&](auto order_id) {
                    return std::make_pair(satisfaction_constrs(order_id),
                                          pattern[order_id]);
                }),
            {.obj_coef = 1, .lower_bound = 0});
    };
    auto expect_master_variables = [&] {
        std::vector<model_variable_t<M>> master_vars;
        for(auto && [pattern, state] : columns.master_columns())
            master_vars.push_back(state.var);
        EXPECT_ENUMERATED_VARIABLES(model, master_vars);
    };

    for(;;) {
        model.solve();
        auto dual_solution = model.get_dual_solution();
        auto price = [&](const auto & entry) {
            double value = 1.0;
            for(auto order_id : order_ids)
                value -= dual_solution[satisfaction_constrs(order_id)] *
                         entry.seed[order_id];
            return colgen::priced{value};
        };

        melon::unbounded_knapsack_bnb knapsack(
            order_ids,
            [&](auto order_id) {
                return dual_solution[satisfaction_constrs(order_id)];
            },
            [&](auto order_id) { return orders[order_id].second; },
            roll_length);
        knapsack.run();
        if(knapsack.solution_value() - 1 > TEST_EPSILON) {
            pattern_t pattern(orders.size(), 0);
            for(auto && [order_id, satisfaction] : knapsack.solution_items())
                pattern[order_id] = static_cast<int>(satisfaction);
            columns.emplace_pool_column(std::move(pattern));
        }

        columns.update_columns(price);
        const std::size_t num_activated = columns.activate_columns(
            model, colgen::negative<colgen::reduced_cost>{},
            add_pattern_column);
        expect_master_variables();
        if(num_activated == 0) break;
        if(evict_tolerance.has_value()) {
            columns.evict_columns(model, colgen::positive<colgen::reduced_cost>{
                                             *evict_tolerance});
            expect_master_variables();
        }
    }
}

}  // namespace column_manager_test

TYPED_TEST_P(ColumnManagerTest, test) {
    this->SkipOnLicenseError([this]() {
        using namespace column_manager_test;
        auto model = this->new_model();
        // the column states embed exactly the properties the predicates read
        colgen::column_manager<decltype(model), pattern_t,
                               colgen::property_list<colgen::reduced_cost>,
                               colgen::property_list<colgen::reduced_cost>>
            columns;
        cutting_stock_column_generation(model, columns, std::nullopt);

        ASSERT_NEAR(model.get_solution_value(), optimal_value, TEST_EPSILON);
        auto solution = model.get_solution();
        int actual_roll_count = 0;
        for(auto && [pattern, state] : columns.master_columns())
            actual_roll_count +=
                static_cast<int>(std::ceil(solution[state.var]));
        ASSERT_EQ(actual_roll_count, 454);
    });
}

// Evicting after activating leaves the master as it was solved last once the
// generation converges, so its solution is still readable: evicting first
// made that read throw on Gurobi and CPLEX and return 0 on HiGHS.
TYPED_TEST_P(ColumnManagerTest, evicting_each_round) {
    this->SkipOnLicenseError([this]() {
        using namespace column_manager_test;
        auto model = this->new_model();
        colgen::column_manager<decltype(model), pattern_t,
                               colgen::property_list<colgen::reduced_cost>,
                               colgen::property_list<colgen::reduced_cost>>
            columns;
        // above the rounding of reduced costs recomputed from the duals, which
        // would otherwise evict basic columns
        cutting_stock_column_generation(model, columns, 1e-6);

        ASSERT_NEAR(model.get_solution_value(), optimal_value, TEST_EPSILON);
        auto solution = model.get_solution();
        double value = 0.0;
        for(auto && [pattern, state] : columns.master_columns())
            value += solution[state.var];
        EXPECT_NEAR(value, optimal_value, TEST_EPSILON);
    });
}

REGISTER_TYPED_TEST_SUITE_P(ColumnManagerTest, test, evicting_each_round);

}  // namespace mippp
