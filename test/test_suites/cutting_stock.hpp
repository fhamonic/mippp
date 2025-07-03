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

namespace fhamonic::mippp {

template <typename T>
struct CuttingStockTest : public T {
    using typename T::model_type;
    static_assert(lp_model<model_type>);
    static_assert(has_dual_solution<model_type>);
    static_assert(has_add_column<model_type>);
};
TYPED_TEST_SUITE_P(CuttingStockTest);

TYPED_TEST_P(CuttingStockTest, test) {
    using namespace operators;
    auto model = this->new_model();
    using variable = typename decltype(model)::variable;

    constexpr int roll_length = 100;
    std::vector<std::pair<int, int>> orders = {
        {211, 14}, {395, 31}, {610, 36}, {97, 45}};  // pairs (quantity,length)

    auto order_ids = std::ranges::iota_view(0ul, orders.size());
    using pattern_t = std::vector<int>;  // order_id -> quantity satisfied
    std::vector<std::pair<variable, pattern_t>> patterns;

    model.set_minimization();
    for(auto order_id : order_ids) {
        auto && [quantity, length] = orders[order_id];
        std::vector<int> pattern(orders.size(), 0);
        pattern[order_id] = roll_length / length;
        patterns.emplace_back(
            model.add_variable({.obj_coef = 1, .lower_bound = 0}),
            std::move(pattern));
    }
    auto satisfaction_constrs =
        model.add_constraints(order_ids, [&](auto order_id) {
            auto && demanded_quantity = orders[order_id].first;
            return xsum(patterns, [&, order_id](auto && p) {
                       auto && [var, pattern] = p;
                       return pattern[order_id] * var;
                   }) >= demanded_quantity;
        });

    for(;;) {
        model.solve();
        auto dual_solution = model.get_dual_solution();

        melon::unbounded_knapsack_bnb knapsack(
            order_ids,
            [&](auto order_id) {
                return dual_solution[satisfaction_constrs(order_id)];
            },
            [&](auto order_id) { return orders[order_id].second; },
            roll_length);
        knapsack.run();
        if(knapsack.solution_value() - 1 <= TEST_EPSILON) break;

        std::vector<int> pattern(orders.size(), 0);
        for(auto && [order_id, satisfaction] : knapsack.solution_items())
            pattern[order_id] = static_cast<int>(satisfaction);
        patterns.emplace_back(
            model.add_column(
                std::views::transform(knapsack.solution_items(),
                                      [&](auto p) {
                                          auto && [order_id, quantity] = p;
                                          return std::make_pair(
                                              satisfaction_constrs(order_id),
                                              quantity);
                                      }),
                {.obj_coef = 1, .lower_bound = 0}),
            std::move(pattern));
    }

    ASSERT_NEAR(model.get_solution_value(), 452.25, TEST_EPSILON);

    auto solution = model.get_solution();

    int actual_roll_count = 0;
    for(auto && [var, pattern] : patterns) {
        actual_roll_count += static_cast<int>(std::ceil(solution[var]));
    }
    ASSERT_EQ(actual_roll_count, 454);
}

REGISTER_TYPED_TEST_SUITE_P(CuttingStockTest, test);

}  // namespace fhamonic::mippp