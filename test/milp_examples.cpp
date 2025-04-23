#undef NDEBUG
#include <gtest/gtest.h>

#include "assert_helper.hpp"
#include "models.hpp"

using namespace fhamonic::mippp;
using namespace fhamonic::mippp::operators;

#define EPSILON 1e-13
#define INFTY 1e100

// clang-format off
using Models = ::testing::Types<
        grb_milp_test,
        cbc_milp_test,
        glpk_milp_test,
        // scip_milp_test,
        highs_milp_test
        // ,mosek_milp_test
        // ,cplex_milp_test
        // ,copt_milp_test
        // ,xprs_milp_test
        >;
// clang-format on

template <typename T>
class MilpModelExamples : public ::testing::Test, public T {};

TYPED_TEST_SUITE(MilpModelExamples, Models);

TYPED_TEST(MilpModelExamples, sudoku) {
    // mosek_api api("mosek64",
    // "/home/plaiseek/Softwares/mosek/11.0/tools/platform/linux64x86/bin");
    // cplex_api api(
    //     "cplex2212",
    // "/home/plaiseek/Softwares/cplex-community/cplex/bin/x86-64_linux");

    using T = TypeParam::model_type;
    if constexpr(std::same_as<T, glpk5_milp>) {
        GTEST_SKIP();
    }
    auto model = this->construct_model();

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
    auto coords = std::views::transform(
        indices, [](auto && i) { return std::make_pair(i / 3, i % 3); });

    auto X_vars =
        model.add_binary_variables(9 * 9 * 9, [](int i, int j, int value) {
            return (81 * i) + (9 * j) + (value - 1);
        });

    for(auto i : indices) {
        for(auto j : indices) {
            model.add_constraint(
                xsum(values, [&](auto && v) { return X_vars(i, j, v); }) == 1);
        }
    }
    for(auto v : values) {
        for(auto i : indices) {
            model.add_constraint(
                xsum(indices, [&](auto && j) { return X_vars(i, j, v); }) == 1);
            model.add_constraint(
                xsum(indices, [&](auto && j) { return X_vars(j, i, v); }) == 1);
        }
        for(auto && [bi, bj] : coords) {
            model.add_constraint(xsum(coords, [&](auto && p) {
                                     return X_vars(3 * bi + p.first,
                                                   3 * bj + p.second, v);
                                 }) == 1);
        }
    }

    for(auto i : indices) {
        for(auto j : indices) {
            auto value = grid_hints[static_cast<std::size_t>(9 * i + j)];
            if(value == 0) continue;
            // model.set_variable_lower_bound(X_vars(i, j, value), 1);
            model.add_constraint(X_vars(i, j, value) >= 1);
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