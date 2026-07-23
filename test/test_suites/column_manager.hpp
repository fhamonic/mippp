#pragma once

#undef NDEBUG
#include <gtest/gtest.h>
#include "assert_helper.hpp"

#include <cmath>
#include <ranges>
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
    static_assert(has_add_column<model_type>);
    static_assert(has_remove_variable<model_type>);
};
TYPED_TEST_SUITE_P(ColumnManagerTest);
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(ColumnManagerTest);

TYPED_TEST_P(ColumnManagerTest, test) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();

        constexpr int roll_length = 100;
        std::vector<std::pair<int, int>> orders = {
            {211, 14},
            {395, 31},
            {610, 36},
            {97, 45}};  // pairs (quantity,length)
        auto order_ids = std::ranges::iota_view(0ul, orders.size());
        using pattern_t = std::vector<int>;  // order_id -> quantity satisfied

        struct pattern_hash {
            std::size_t operator()(const pattern_t & pattern) const noexcept {
                std::size_t h = pattern.size();
                for(int i : pattern)
                    h ^= static_cast<std::size_t>(i) + 0x9e3779b97f4a7c15 +
                         (h << 6) + (h >> 2);
                return h;
            }
        };

        // adds every column with improving reduced cost, evicts columns that
        // stayed unattractive for their 8 last pricing rounds in the master ;
        // the column states embed exactly the properties these strategies read
        column_manager<decltype(model), pattern_t, property_list<reduced_cost>,
                       property_list<reduced_cost>, pattern_hash>
            columns;

        model.set_minimization();
        for(auto order_id : order_ids) {
            auto && [quantity, length] = orders[order_id];
            pattern_t pattern(orders.size(), 0);
            pattern[order_id] = roll_length / length;
            columns.emplace_master_column(
                std::move(pattern),
                model.add_variable({.obj_coef = 1, .lower_bound = 0}));
        }
        auto satisfaction_constrs =
            model.add_constraints(order_ids, [&](auto order_id) {
                auto && demanded_quantity = orders[order_id].first;
                return xsum(columns.master_columns(),
                            [&, order_id](auto && column) {
                                auto && [pattern, state] = column;
                                return pattern[order_id] * state.var;
                            }) >= demanded_quantity;
            });

        auto add_pattern_column = [&](auto & model_,
                                      const pattern_t & pattern) {
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

        for(;;) {
            model.solve();
            auto dual_solution = model.get_dual_solution();
            auto price = [&](const pattern_t & pattern) {
                double value = 1.0;
                for(auto order_id : order_ids)
                    value -= dual_solution[satisfaction_constrs(order_id)] *
                             pattern[order_id];
                return priced{value};
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
                for(auto && [order_id, satisfaction] :
                    knapsack.solution_items())
                    pattern[order_id] = static_cast<int>(satisfaction);
                columns.emplace_column(std::move(pattern));
            }

            columns.update_columns(price);
            auto result = columns.manage_columns(
                model, all<negative<reduced_cost>>{}, add_pattern_column);
            if(result.num_activated == 0) break;
        }

        ASSERT_NEAR(model.get_solution_value(), 452.25, TEST_EPSILON);

        auto solution = model.get_solution();
        int actual_roll_count = 0;
        for(auto && [pattern, state] : columns.master_columns())
            actual_roll_count +=
                static_cast<int>(std::ceil(solution[state.var]));
        ASSERT_EQ(actual_roll_count, 454);
    });
}

REGISTER_TYPED_TEST_SUITE_P(ColumnManagerTest, test);

}  // namespace mippp
