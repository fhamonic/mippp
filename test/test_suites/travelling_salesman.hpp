#pragma once
#undef NDEBUG
#include <gtest/gtest.h>
#include "assert_helper.hpp"

#include <cmath>
#include <ranges>
#include <vector>

#include "melon/algorithm/breadth_first_search.hpp"
#include "melon/algorithm/depth_first_search.hpp"
#include "melon/algorithm/strongly_connected_components.hpp"
#include "melon/container/static_digraph.hpp"
#include "melon/utility/static_digraph_builder.hpp"
#include "melon/views/subgraph.hpp"

#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"

namespace fhamonic::mippp {

template <typename T>
struct TravellingSalesmanTest : public T {
    using typename T::model_type;
    static_assert(milp_model<model_type>);
};
TYPED_TEST_SUITE_P(TravellingSalesmanTest);

TYPED_TEST_P(TravellingSalesmanTest, test) {
    using namespace operators;
    auto model = this->new_model();

    // https://jlmartin.ku.edu/courses/math105-F14/chapter6-part6.pdf
    std::vector<std::string> vertex_names = {"A", "B", "C", "D", "E", "F"};
    // clang-format off
    std::vector<std::vector<int>> distances = {
        { 0, 12, 29, 22, 13, 24},
        {12,  0, 19,  3, 25,  6},
        {29, 19,  0, 21, 23, 28},
        {22,  3, 21,  0,  4,  5},
        {13, 25, 23,  4,  0, 16},
        {24,  6, 28,  5, 16,  0}};
    // clang-format on

    melon::static_digraph_builder<melon::static_digraph, double> builder(
        vertex_names.size());
    for(unsigned int i = 0; i < vertex_names.size(); ++i) {
        for(unsigned int j = 0; j < vertex_names.size(); ++j) {
            if(i == j) continue;
            builder.add_arc(i, j, distances[i][j]);
        }
    }
    auto [graph, length_map] = builder.build();

    auto X_vars = model.add_binary_variables(graph.num_arcs());

    model.set_minimization();
    model.set_objective(xsum(
        graph.arcs(), [&](auto && a) { return length_map[a] * X_vars(a); }));

    model.add_constraints(graph.vertices(), [&](auto && u) {
        return xsum(graph.in_arcs(u), [&](auto a) { return X_vars(a); }) == 1;
    });
    model.add_constraints(graph.vertices(), [&](auto && u) {
        return xsum(graph.out_arcs(u), [&](auto a) { return X_vars(a); }) == 1;
    });

    model.set_candidate_solution_callback([&](auto & handle) {
        auto solution = handle.get_solution();
        auto solution_graph = melon::views::subgraph(
            graph, {}, [&](auto a) { return solution[X_vars(a)] > 0.5; });

        for(auto component :
            melon::strongly_connected_components(solution_graph)) {
            auto component_vertex_filter =
                melon::create_vertex_map<bool>(graph, false);
            std::size_t tour_size = 0u;
            for(const auto & v : component) {
                component_vertex_filter[v] = true;
                ++tour_size;
            }
            if(tour_size == graph.num_vertices()) return;
            auto tour_induced_subgraph =
                melon::views::subgraph(graph, component_vertex_filter, {});
            handle.add_lazy_constraint(
                xsum(melon::arcs(tour_induced_subgraph), [&](auto a) {
                    return X_vars(a);
                }) <= static_cast<int>(tour_size) - 1);
        }
    });

    model.solve();

    ASSERT_NEAR(model.get_solution_value(), 76, TEST_EPSILON);

    auto solution = model.get_solution();
    auto solution_graph = melon::views::subgraph(graph, {}, [&](auto a) {
        return solution[X_vars(a)] > 0.5;
    });

    for(auto && v : melon::breadth_first_search(solution_graph, 0u))
        std::cout << " -> " << vertex_names[v];
    std::cout << std::endl;
}

REGISTER_TYPED_TEST_SUITE_P(TravellingSalesmanTest, test);

}  // namespace fhamonic::mippp