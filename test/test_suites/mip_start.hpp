#pragma once
#undef NDEBUG
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "assert_helper.hpp"

#include <chrono>
#include <ranges>

#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"

namespace fhamonic::mippp {

template <typename T>
struct MipStartTest : public T {
    using typename T::model_type;
    static_assert(milp_model<model_type>);
    static_assert(has_mip_start<model_type>);
};
TYPED_TEST_SUITE_P(MipStartTest);

TYPED_TEST_P(MipStartTest, test) {
    using namespace operators;
    // clang-format off
    std::vector<int> grid_hints = {
        0, 0, 8, 0, 1, 0, 0, 0, 9,
        6, 0, 0, 0, 0, 0, 3, 0, 0,
        0, 4, 0, 0, 3, 0, 0, 0, 5,
        0, 0, 0, 0, 0, 8, 0, 0, 0,
        0, 0, 0, 6, 0, 0, 8, 0, 0,
        0, 0, 4, 0, 0, 1, 0, 5, 0,
        5, 0, 0, 3, 0, 0, 0, 8, 0,
        0, 9, 0, 0, 0, 0, 0, 0, 6,
        1, 0, 0, 0, 6, 0, 9, 0, 0};
    // clang-format on

    auto indices = std::views::iota(0, 9);
    auto values = std::views::iota(1, 10);
    auto coords = std::views::cartesian_product(std::views::iota(0, 3),
                                                std::views::iota(0, 3));

    auto fill_model = [&](auto & model, auto & X_vars) {
        auto single_value_constrs = model.add_constraints(
            std::views::cartesian_product(indices, indices), [&](auto && p) {
                auto && [i, j] = p;
                return xsum(values,
                            [&](auto && v) { return X_vars(i, j, v); }) == 1;
            });
        auto one_per_row_constrs = model.add_constraints(
            std::views::cartesian_product(values, indices), [&](auto && p) {
                auto && [v, i] = p;
                return xsum(indices,
                            [&](auto && j) { return X_vars(i, j, v); }) == 1;
            });
        auto one_per_col_constrs = model.add_constraints(
            std::views::cartesian_product(values, indices), [&](auto && p) {
                auto && [v, j] = p;
                return xsum(indices,
                            [&](auto && i) { return X_vars(i, j, v); }) == 1;
            });
        auto one_per_block_constrs = model.add_constraints(
            std::views::cartesian_product(values, coords), [&](auto && p) {
                auto && [v, b] = p;
                return xsum(coords, [&](auto && p2) {
                           return X_vars(3 * std::get<0>(b) + std::get<0>(p2),
                                         3 * std::get<1>(b) + std::get<1>(p2),
                                         v);
                       }) == 1;
            });
        for(auto i : indices) {
            for(auto j : indices) {
                auto value = grid_hints[static_cast<std::size_t>(9 * i + j)];
                if(value == 0) continue;
                model.add_constraint(X_vars(i, j, value) == 1);
            }
        }
    };

    auto default_model = this->new_model();
    auto default_X_vars = default_model.add_binary_variables(
        9 * 9 * 9, [](int i, int j, int value) {
            return (81 * i) + (9 * j) + (value - 1);
        });
    fill_model(default_model, default_X_vars);

    auto default_start = std::chrono::system_clock::now();
    default_model.solve();
    auto default_end = std::chrono::system_clock::now();
    auto default_time_us =
        std::chrono::duration_cast<std::chrono::microseconds>(default_end -
                                                              default_start);

    auto solution = default_model.get_solution();
    std::vector<int> grid_solution;
    grid_solution.reserve(9 * 9);
    for(auto i : indices)
        for(auto j : indices)
            for(auto v : values)
                if(solution[default_X_vars(i, j, v)])
                    grid_solution.push_back(v);

    auto mipstarted_model = this->new_model();
    auto mipstarted_X_vars = mipstarted_model.add_binary_variables(
        9 * 9 * 9, [](int i, int j, int value) {
            return (81 * i) + (9 * j) + (value - 1);
        });
    fill_model(mipstarted_model, mipstarted_X_vars);

    mipstarted_model.set_mip_start(std::views::transform(
        std::views::cartesian_product(indices, indices), [&](auto && p) {
            auto && [i, j] = p;
            auto v = grid_solution[static_cast<std::size_t>(9 * i + j)];
            return std::make_pair(mipstarted_X_vars(i, j, v), 1.0);
        }));

    auto mipstarted_start = std::chrono::system_clock::now();
    mipstarted_model.solve();
    auto mipstarted_end = std::chrono::system_clock::now();
    auto mipstarted_time_us =
        std::chrono::duration_cast<std::chrono::microseconds>(mipstarted_end -
                                                              mipstarted_start);

    std::cout << default_time_us << std::endl;
    std::cout << mipstarted_time_us << std::endl;
    ASSERT_TRUE(default_time_us > 3.0 * mipstarted_time_us);
}

REGISTER_TYPED_TEST_SUITE_P(MipStartTest, test);

}  // namespace fhamonic::mippp