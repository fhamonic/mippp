// Travelling Salesman Problem solved with lazy subtour-elimination constraints.
//
// Showcases MIP++'s branch-and-cut callback: the candidate-solution callback
// receives a typed handle exposing the incumbent and add_lazy_constraint, so
// subtours are cut off on the fly.
//
// NOTE: the candidate-solution callback is currently validated on Gurobi,
// CPLEX and COPT. Set the two aliases below to one of those backends; the
// example defaults to Gurobi.

#include <iostream>
#include <ranges>
#include <vector>

#include "mippp/solvers/gurobi/all.hpp"

using namespace mippp;
using namespace mippp::operators;

using api_type = gurobi_api;
using milp_type = gurobi_milp;

int main() {
    // Symmetric distance matrix; the optimal tour has length 76.
    // clang-format off
    const std::vector<std::vector<int>> dist = {
        { 0, 12, 29, 22, 13, 24},
        {12,  0, 19,  3, 25,  6},
        {29, 19,  0, 21, 23, 28},
        {22,  3, 21,  0,  4,  5},
        {13, 25, 23,  4,  0, 16},
        {24,  6, 28,  5, 16,  0}};
    // clang-format on
    const int n = static_cast<int>(dist.size());
    auto cities = std::views::iota(0, n);

    api_type api;
    milp_type model(api);

    // X(i, j) == 1 iff the tour travels directly from city i to city j.
    auto X = model.add_binary_variables(
        n * n, [n](int i, int j) { return i * n + j; });

    model.set_minimization();
    model.set_objective(
        xsum(std::views::cartesian_product(cities, cities), [&](auto && p) {
            auto && [i, j] = p;
            return dist[i][j] * X(i, j);
        }));

    // No self-loops, and exactly one outgoing and one incoming arc per city.
    // The inner lambda captures the outer loop parameter by value ([&, i]):
    // add_constraints registers the xsum expression lazily, so a by-reference
    // capture would dangle once the generator lambda returns.
    for(int i : cities) model.add_constraint(X(i, i) == 0);
    model.add_constraints(cities, [&](int i) {
        return xsum(cities, [&, i](int j) { return X(i, j); }) == 1;
    });
    model.add_constraints(cities, [&](int j) {
        return xsum(cities, [&, j](int i) { return X(i, j); }) == 1;
    });

    // Lazily forbid any incumbent that splits into several subtours.
    model.set_candidate_solution_callback([&](auto & handle) {
        auto sol = handle.get_solution();
        // successor[i] = the city visited right after i in the incumbent.
        std::vector<int> successor(static_cast<std::size_t>(n), 0);
        for(int i : cities)
            for(int j : cities)
                if(i != j && sol[X(i, j)] > 0.5)
                    successor[static_cast<std::size_t>(i)] = j;

        std::vector<bool> seen(static_cast<std::size_t>(n), false);
        for(int start : cities) {
            if(seen[static_cast<std::size_t>(start)]) continue;
            std::vector<int> subtour;
            for(int v = start; !seen[static_cast<std::size_t>(v)];
                v = successor[static_cast<std::size_t>(v)]) {
                seen[static_cast<std::size_t>(v)] = true;
                subtour.push_back(v);
            }
            if(static_cast<int>(subtour.size()) == n) return;  // single tour
            // Arcs staying inside the subtour must number at most |S| - 1.
            handle.add_lazy_constraint(
                xsum(std::views::cartesian_product(subtour, subtour),
                     [&](auto && p) {
                         auto && [i, j] = p;
                         return X(i, j);
                     }) <= static_cast<int>(subtour.size()) - 1);
        }
    });

    model.solve();

    std::cout << "optimal tour length: " << model.get_solution_value() << '\n';
    auto sol = model.get_solution();
    std::cout << "tour: 0";
    int v = 0;
    for(int step = 0; step < n; ++step) {
        for(int j : cities)
            if(j != v && sol[X(v, j)] > 0.5) {
                v = j;
                break;
            }
        std::cout << " -> " << v;
    }
    std::cout << '\n';
    return 0;
}
