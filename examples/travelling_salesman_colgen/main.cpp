// The Held-Karp lower bound of a travelling salesman instance, by column
// generation over 1-trees.
//
// Showcases colgen::column_manager on columns that are combinatorial objects. A
// 1-tree is a spanning tree of the cities other than city 0, plus two edges at
// city 0; a tour is a 1-tree whose cities all have degree 2. The master LP
// finds the cheapest convex combination of 1-trees in which every city has
// degree 2: its optimum is the Held-Karp bound, equal to the bound of the
// subtour elimination LP. Pricing is a minimum spanning tree under the reduced
// edge lengths, from MELON's kruskal. The manager identifies each 1-tree by its
// edges and keeps its length beside it. Each round recomputes the reduced cost
// of every known 1-tree from the duals, and evicts the columns that have been
// in the master long enough and are now unattractive, waiting longer each time
// a column comes back.
//
// The number of cities, at least 3, is an optional argument, 25 by default.
// Swap the alias to use another LP backend with dual values and
// remove_variable, e.g. clp_lp, gurobi_lp or cplex_lp.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <print>
#include <random>
#include <ranges>
#include <utility>
#include <vector>

#include "mippp/solvers/highs/all.hpp"
#include "mippp/utility/column_manager.hpp"

#include "melon/algorithm/kruskal.hpp"
#include "melon/views/complete_digraph.hpp"
#include "melon/views/undirect.hpp"

using namespace mippp;
using namespace mippp::operators;

using lp_type = highs_lp;

using distance_matrix = std::vector<std::vector<int>>;
using edge = std::pair<std::size_t, std::size_t>;
// Edges as make_edge gives them, (min, max), and sorted. Kruskal returns
// either orientation of an edge: stored as given, the same 1-tree would get
// several seeds, and the manager would add it to the master more than once.
using one_tree = std::vector<edge>;

edge make_edge(std::size_t u, std::size_t v) {
    return {std::min(u, v), std::max(u, v)};
}

// n cities on a 1000 x 1000 square, at rounded Euclidean distances (TSPLIB's
// EUC_2D). The coordinates are std::mt19937's raw output, whose sequence the
// standard specifies, unlike that of the distributions: the instance is the
// same on every platform.
distance_matrix random_cities(std::size_t n) {
    std::mt19937 gen;
    std::vector<std::pair<double, double>> city(n);
    for(auto & [x, y] : city) {
        x = static_cast<double>(gen() % 1000);
        y = static_cast<double>(gen() % 1000);
    }
    distance_matrix dist(n, std::vector<int>(n));
    for(std::size_t u = 0; u < n; ++u)
        for(std::size_t v = 0; v < n; ++v) {
            const double dx = city[u].first - city[v].first;
            const double dy = city[u].second - city[v].second;
            dist[u][v] =
                static_cast<int>(std::lround(std::sqrt(dx * dx + dy * dy)));
        }
    return dist;
}

int length(const distance_matrix & dist, const one_tree & tree) {
    int sum = 0;
    for(auto [u, v] : tree) sum += dist[u][v];
    return sum;
}

std::vector<int> degrees(const one_tree & tree, std::size_t n) {
    std::vector<int> degree(n, 0);
    for(auto [u, v] : tree) {
        ++degree[u];
        ++degree[v];
    }
    return degree;
}

one_tree nearest_neighbour_tour(const distance_matrix & dist) {
    const std::size_t n = dist.size();
    std::vector<bool> visited(n, false);
    one_tree tour;
    std::size_t city = 0;
    for(std::size_t step = 1; step < n; ++step) {
        visited[city] = true;
        const std::size_t next = std::ranges::min(
            std::views::iota(std::size_t{0}, n) |
                std::views::filter([&](std::size_t v) { return !visited[v]; }),
            {}, [&](std::size_t v) { return dist[city][v]; });
        tour.push_back(make_edge(city, next));
        city = next;
    }
    tour.push_back(make_edge(city, 0));
    std::ranges::sort(tour);
    return tour;
}

// The 1-tree of least length under the reduced edge lengths
// dist[u][v] - pi[u] - pi[v]: a minimum spanning tree of the cities 1..n-1,
// plus the two shortest edges at city 0.
one_tree shortest_one_tree(const distance_matrix & dist,
                           const std::vector<double> & pi) {
    const std::size_t n = dist.size();
    auto reduced = [&](std::size_t u, std::size_t v) {
        return dist[u][v] - pi[u] - pi[v];
    };
    // Vertex i is the city i + 1. The undirected view has an edge per arc,
    // so each pair of cities appears twice; kruskal skips the second copy.
    const auto others =
        melon::views::undirect(melon::views::complete_digraph(n - 1));
    one_tree tree;
    for(auto && e : melon::kruskal(others, [&](auto a) {
            auto [u, v] = melon::edge_endpoints(others, a);
            return reduced(u + 1, v + 1);
        })) {
        auto [u, v] = melon::edge_endpoints(others, e);
        tree.push_back(make_edge(u + 1, v + 1));
    }
    auto from_0 = [&](std::size_t v) { return reduced(0, v); };
    auto rest = std::views::iota(std::size_t{1}, n);
    const std::size_t nearest = std::ranges::min(rest, {}, from_0);
    const std::size_t second = std::ranges::min(
        rest | std::views::filter([&](std::size_t v) { return v != nearest; }),
        {}, from_0);
    tree.push_back(make_edge(0, nearest));
    tree.push_back(make_edge(0, second));
    std::ranges::sort(tree);
    return tree;
}

int main(int argc, char ** argv) {
    const std::size_t n =
        (argc > 1) ? static_cast<std::size_t>(std::atoi(argv[1])) : 25;
    const distance_matrix dist = random_cities(n);
    auto cities = std::views::iota(std::size_t{0}, n);

    lp_type model;
    model.set_minimization();
    // A 1-tree has n edges, so summing these rows gives sum(lambda) = 1: the
    // convexity row is implied, and has no dual in the reduced costs.
    auto degree_rows = model.add_constraints(cities, [](std::size_t) {
        return empty_linear_expression<model_variable_t<lp_type>, double> ==
               2.0;
    });

    // A column's seed is its 1-tree, and its Data the 1-tree's length.
    // times_activated is listed in both states, or it restarts at each
    // activation.
    colgen::column_manager<
        lp_type, one_tree,
        colgen::property_list<colgen::reduced_cost, colgen::times_activated>,
        colgen::property_list<colgen::reduced_cost, colgen::age,
                              colgen::times_activated>,
        int>
        columns;
    auto add_one_tree = [&](lp_type & master, const auto & entry) {
        const std::vector<int> degree = degrees(entry.seed, n);
        return master.add_column(
            cities | std::views::transform([&](std::size_t v) {
                return std::make_pair(degree_rows(v),
                                      static_cast<double>(degree[v]));
            }),
            {.obj_coef = static_cast<double>(entry.data), .lower_bound = 0});
    };

    one_tree tour = nearest_neighbour_tour(dist);
    int best_tour = length(dist, tour);
    columns.emplace_master_column(model, std::move(tour), add_one_tree,
                                  best_tour);

    // Reduced costs recomputed from the duals are off by the solver's dual
    // errors, of either sign (about 1e-5 with HiGHS at 60 cities). Without a
    // tolerance above them, a basic master column, at zero, reads
    // unattractive, and a pool column at zero reads improving.
    constexpr double tolerance = 1e-4;
    std::size_t rounds = 0;
    for(;;) {
        model.solve();
        ++rounds;
        // read the master before activate_columns and evict_columns modify it
        auto duals = model.get_dual_solution();
        const auto pi = cities | std::views::transform([&](std::size_t v) {
                            return duals[degree_rows(v)];
                        }) |
                        std::ranges::to<std::vector>();

        one_tree tree = shortest_one_tree(dist, pi);
        const int tree_length = length(dist, tree);
        if(tree_length < best_tour &&
           std::ranges::all_of(degrees(tree, n), [](int d) { return d == 2; }))
            best_tour = tree_length;
        columns.emplace_pool_column(std::move(tree), tree_length);

        // pool columns too: an evicted column that became improving again
        // returns to the master without the pricer finding it again
        columns.update_columns([&](const auto & entry) {
            double reduced_cost = entry.data;
            for(auto [u, v] : entry.seed) reduced_cost -= pi[u] + pi[v];
            return colgen::priced{reduced_cost};
        });
        // no improving column: the master is still as solved last
        if(columns.activate_columns(
               model, colgen::negative<colgen::reduced_cost>{tolerance},
               add_one_tree) == 0)
            break;
        // columns in the master for more than 20 rounds plus 20 per time they
        // entered it, and now unattractive
        columns.evict_columns(
            model, colgen::conjunction{
                       colgen::age_above_with_backoff{.base = 20, .step = 20},
                       colgen::positive<colgen::reduced_cost>{tolerance}});
    }

    if(!is_a<status::optimal>(model.get_status())) {
        std::println("the last master solve is not optimal: no bound");
        return 1;
    }
    const double bound = model.get_solution_value();
    // Tour lengths are integers, so no tour is shorter than the bound rounded
    // up. Activation stops at reduced costs above -tolerance, which leaves the
    // master's dual value, 2 sum(pi), at most tolerance above the Held-Karp
    // bound, and its value as close to that as the solver's primal-dual gap:
    // rounded up as is, a value just above an integer would pass the optimum.
    const double gap = (best_tour - std::ceil(bound - tolerance)) / best_tour;
    std::println("cities          : {}", n);
    std::println("Held-Karp bound : {:.2f}", bound);
    std::println("best tour found : {} (gap {:.2f}%)", best_tour, 100.0 * gap);
    std::println("rounds          : {}", rounds);
    std::println("columns         : {}, {} of them in the master",
                 columns.num_columns(), columns.num_master_columns());
    return 0;
}
