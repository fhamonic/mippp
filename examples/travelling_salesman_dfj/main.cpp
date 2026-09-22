// Travelling Salesman Problem solved with lazy subtour-elimination constraints.
//
// Showcases MIP++'s branch-and-cut callback: the candidate-solution callback
// receives a typed handle exposing the incumbent and add_lazy_constraint, so
// subtours are cut off on the fly.
//
// NOTE: the candidate-solution callback is currently validated on Gurobi,
// CPLEX and COPT. Set the two aliases below to one of those backends; the
// example defaults to Gurobi.

#include <print>
#include <ranges>
#include <vector>

#include "mippp/solvers/gurobi/all.hpp"

#include "melon/algorithm/traversal_forest.hpp"
#include "melon/views/complete_digraph.hpp"
#include "melon/views/subgraph.hpp"

using namespace mippp;
using namespace mippp::operators;

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
    auto graph = melon::views::complete_digraph(n);

    using vertex = melon::vertex_t<decltype(graph)>;
    using arc = melon::arc_t<decltype(graph)>;

    auto arc_length = [&](const arc & a) {
        return dist[graph.arc_source(a)][graph.arc_target(a)];
    };

    milp_type model;

    auto X = model.add_binary_variables(graph.arcs());
    model.set_minimization();
    model.set_objective(xsum(
        graph.arcs(), [&](const arc & a) { return arc_length(a) * X(a); }));

    model.add_constraints(graph.vertices(), [&](auto && v) {
        return xsum(graph.in_arcs(v), X) == 1;
    });
    model.add_constraints(graph.vertices(), [&](auto && v) {
        return xsum(graph.in_arcs(v), X) == xsum(graph.out_arcs(v), X);
    });

    // Lazily forbid any incumbent that splits into several subtours.
    model.set_candidate_solution_callback([&](auto & handle) {
        auto solution = handle.get_solution();
        auto solution_graph = melon::views::subgraph(
            graph, {}, [&](auto a) { return solution[X(a)] > 0.5; });

        for(auto && tour : melon::traversal_forest(solution_graph)) {
            if(tour.size() == graph.num_vertices()) return;
            auto tour_induced_subgraph =
                melon::views::induced_subgraph(graph, tour);
            handle.add_lazy_constraint(
                xsum(melon::arcs(tour_induced_subgraph), X) <=
                static_cast<int>(tour.size()) - 1);
        }
    });

    model.solve();

    std::println("optimal tour length: {}", model.get_solution_value());
    auto solution = model.get_solution();
    for(auto && v : melon::breadth_first_search(melon::views::subgraph(
            graph, {}, [&](auto a) { return solution[X(a)] > 0.5; }))) {
        std::print(" -> {}", v);
    }
    std::println("");
    return 0;
}
