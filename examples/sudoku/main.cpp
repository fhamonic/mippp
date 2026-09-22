// Sudoku as a MILP.
//
// Showcases multi-dimensional lambda indexing -- X(i, j, v) is 1 iff cell (i,
// j) holds value v -- and constraint families composed over cartesian-product
// ranges.
//
// Swap the two aliases to use another MILP backend.

#include <print>
#include <ranges>
#include <vector>

#include "mippp/solvers/highs/all.hpp"

using namespace mippp;
using namespace mippp::operators;

using milp_type = highs_milp;

int main() {
    // 0 denotes an empty cell.
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
    // clang-format on

    auto indices = std::views::iota(0, 9);
    auto values = std::views::iota(1, 10);
    auto coords = std::views::cartesian_product(std::views::iota(0, 3),
                                                std::views::iota(0, 3));

    milp_type model;

    auto X = model.add_binary_variables(
        std::views::cartesian_product(indices, indices, values));

    // Exactly one value per cell.
    model.add_constraints(
        std::views::cartesian_product(indices, indices), [&](int i, int j) {
            return xsum(values, [&, i, j](int v) { return X(i, j, v); }) == 1;
        });
    // Each value appears once per row and once per column.
    model.add_constraints(
        std::views::cartesian_product(values, indices), [&](int v, int i) {
            return xsum(indices, [&, v, i](int j) { return X(i, j, v); }) == 1;
        });
    model.add_constraints(
        std::views::cartesian_product(values, indices), [&](int v, int j) {
            return xsum(indices, [&, v, j](int i) { return X(i, j, v); }) == 1;
        });
    // Each value appears once per 3x3 block.
    model.add_constraints(
        std::views::cartesian_product(values, coords), [&](int v, auto block) {
            auto [bi, bj] = block;
            return xsum(coords, [&, v, bi, bj](int di, int dj) {
                       return X(3 * bi + di, 3 * bj + dj, v);
                   }) == 1;
        });

    // Fix the given hints.
    for(auto [i, j] : std::views::cartesian_product(indices, indices)) {
        int v = grid_hints[static_cast<std::size_t>(9 * i + j)];
        if(v != 0) model.add_constraint(X(i, j, v) == 1);
    }

    model.solve();
    auto solution = model.get_solution();

    for(int i : indices) {
        for(int j : indices)
            for(int v : values)
                if(solution[X(i, j, v)] > 0.5) std::print(" {}", v);
        std::println("");
    }
    return 0;
}
