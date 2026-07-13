// N-Queens as a MILP.
//
// Showcases lambda-indexed variables -- X(row, col) maps to a variable by plain
// arithmetic -- and whole constraint families built functionally over ranges
// with xsum. This is the same model used in the mippp_nqueens benchmark
// (https://github.com/fhamonic/mippp_nqueens).
//
// Swap the two aliases to use another MILP backend.

#include <cstdlib>
#include <iostream>
#include <ranges>

#include "mippp/solvers/highs/all.hpp"

using namespace mippp;
using namespace mippp::operators;

using api_type = highs_api;
using milp_type = highs_milp;

int main(int argc, char ** argv) {
    const int n = (argc > 1) ? std::atoi(argv[1]) : 8;
    auto indices = std::views::iota(0, n);

    api_type api;
    milp_type model(api);

    // One binary variable per board cell; X(row, col) is an O(1) lookup.
    auto X = model.add_binary_variables(
        n * n, [n](int row, int col) { return row * n + col; });

    // Exactly one queen per row and per column. The inner lambda captures the
    // outer loop parameter *by value* (e.g. [&, row]) because add_constraints
    // registers the xsum expression lazily: capturing it by reference would
    // dangle once the generator lambda returns.
    model.add_constraints(indices, [&](int row) {
        return xsum(indices, [&, row](int col) { return X(row, col); }) == 1;
    });
    model.add_constraints(indices, [&](int col) {
        return xsum(indices, [&, col](int row) { return X(row, col); }) == 1;
    });
    // At most one queen on each diagonal, in both directions.
    model.add_constraints(std::views::iota(0, n - 1), [&](int top_col) {
        return xsum(std::views::iota(0, n - top_col),
                    [&, top_col](int row) { return X(row, top_col + row); }) <=
               1;
    });
    model.add_constraints(std::views::iota(1, n - 1), [&](int left_row) {
        return xsum(std::views::iota(0, n - left_row),
                    [&, left_row](int col) { return X(left_row + col, col); }) <=
               1;
    });
    model.add_constraints(std::views::iota(1, n), [&](int left_row) {
        return xsum(std::views::iota(0, left_row + 1),
                    [&, left_row](int col) { return X(left_row - col, col); }) <=
               1;
    });
    model.add_constraints(std::views::iota(1, n - 1), [&](int bottom_col) {
        return xsum(std::views::iota(bottom_col, n), [&, bottom_col](int col) {
                   return X(n - 1 - (col - bottom_col), col);
               }) <= 1;
    });

    model.solve();

    auto solution = model.get_solution();
    for(int row : indices) {
        for(int col : indices)
            std::cout << (solution[X(row, col)] > 0.5 ? " Q" : " .");
        std::cout << '\n';
    }
    return 0;
}
