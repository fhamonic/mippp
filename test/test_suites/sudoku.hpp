#pragma once
#undef NDEBUG
#include <gtest/gtest.h>
#include "assert_helper.hpp"

#include <ranges>

#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"

namespace fhamonic::mippp {

template <typename T>
struct SudokuTest : public T {
    using typename T::model_type;
    static_assert(milp_model<model_type>);
};
TYPED_TEST_SUITE_P(SudokuTest);

TYPED_TEST_P(SudokuTest, test) {
    using namespace operators;
    auto model = this->new_model();

    // clang-format off
    std::vector<int> grid_hints = {
        0, 0, 8, 0, 1, 0, 0, 0, 9,
        6, 0, 1, 0, 9, 0, 3, 2, 0,
        0, 4, 0, 0, 3, 7, 0, 0, 5,
        0, 3, 5, 0, 0, 8, 2, 0, 0,
        0, 0, 2, 6, 5, 0, 8, 0, 0,
        0, 0, 4, 0, 0, 1, 7, 5, 0,
        5, 0, 0, 3, 4, 0, 0, 8, 0,
        0, 9, 0, 0, 8, 0, 5, 0, 6,
        1, 0, 0, 0, 6, 0, 9, 0, 0};
    std::vector<int> expected_grid_solution = {
        3, 5, 8, 2, 1, 6, 4, 7, 9,
        6, 7, 1, 5, 9, 4, 3, 2, 8,
        2, 4, 9, 8, 3, 7, 6, 1, 5,
        9, 3, 5, 4, 7, 8, 2, 6, 1,
        7, 1, 2, 6, 5, 3, 8, 9, 4,
        8, 6, 4, 9, 2, 1, 7, 5, 3,
        5, 2, 6, 3, 4, 9, 1, 8, 7,
        4, 9, 7, 1, 8, 2, 5, 3, 6,
        1, 8, 3, 7, 6, 5, 9, 4, 2};
    // clang-format on

    auto indices = std::views::iota(0, 9);
    auto values = std::views::iota(1, 10);
    auto coords = std::views::cartesian_product(std::views::iota(0, 3),
                                                std::views::iota(0, 3));

    auto X_vars =
        model.add_binary_variables(9 * 9 * 9, [](int i, int j, int value) {
            return (81 * i) + (9 * j) + (value - 1);
        });

    auto single_value_constrs = model.add_constraints(
        std::views::cartesian_product(indices, indices), [&](auto && p) {
            auto && [i, j] = p;
            return xsum(values, [&](auto && v) { return X_vars(i, j, v); }) ==
                   1;
        });
    auto one_per_row_constrs = model.add_constraints(
        std::views::cartesian_product(values, indices), [&](auto && p) {
            auto && [v, i] = p;
            return xsum(indices, [&](auto && j) { return X_vars(i, j, v); }) ==
                   1;
        });
    auto one_per_col_constrs = model.add_constraints(
        std::views::cartesian_product(values, indices), [&](auto && p) {
            auto && [v, j] = p;
            return xsum(indices, [&](auto && i) { return X_vars(i, j, v); }) ==
                   1;
        });
    auto one_per_block_constrs = model.add_constraints(
        std::views::cartesian_product(values, coords), [&](auto && p) {
            auto && [v, b] = p;
            return xsum(coords, [&](auto && p2) {
                       return X_vars(3 * std::get<0>(b) + std::get<0>(p2),
                                     3 * std::get<1>(b) + std::get<1>(p2), v);
                   }) == 1;
        });

    for(auto i : indices) {
        for(auto j : indices) {
            auto value = grid_hints[static_cast<std::size_t>(9 * i + j)];
            if(value == 0) continue;
            model.add_constraint(X_vars(i, j, value) == 1);
        }
    }

    model.solve();
    auto solution = model.get_solution();

    for(auto i : indices) {
        for(auto j : indices) {
            for(auto v : values) {
                if(solution[X_vars(i, j, v)]) std::cout << ' ' << v;
            }
        }
        std::cout << std::endl;
    }

    std::vector<int> grid_solution;
    grid_solution.reserve(9 * 9);
    for(auto i : indices)
        for(auto j : indices)
            for(auto v : values)
                if(solution[X_vars(i, j, v)]) grid_solution.push_back(v);

    ASSERT_EQ_RANGES(grid_solution, expected_grid_solution);
}

REGISTER_TYPED_TEST_SUITE_P(SudokuTest, test);

}  // namespace fhamonic::mippp