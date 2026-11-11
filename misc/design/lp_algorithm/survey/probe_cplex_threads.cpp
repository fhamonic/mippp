// The dense LP of ThreadLimitTest (size 900): cores cplex_lp keeps busy under
// each algorithm at the default thread limit, and under a limit of 1.
#include <chrono>
#include <ctime>
#include <print>
#include <random>
#include <ranges>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/solvers/cplex/all.hpp"

using namespace mippp;
using namespace mippp::operators;

template <class M>
void build_dense_lp(M & model, std::size_t size) {
    std::mt19937 rng(42);
    std::uniform_int_distribution<int> coef(1, 1000);
    auto columns = std::views::iota(std::size_t{0}, size);
    auto x = model.add_variables(size);
    std::vector<int> row(size);
    for(int & a : row) a = coef(rng);
    model.set_maximization();
    model.set_objective(xsum(columns, [&](auto j) { return row[j] * x(j); }));
    for(std::size_t i = 0; i < size; ++i) {
        for(int & a : row) a = coef(rng);
        model.add_constraint(xsum(columns, [&, row](auto j) { return row[j] * x(j); }) <=
                             1000.0 * static_cast<double>(size));
    }
}

template <class A>
void run(const char * name, A algorithm, std::size_t threads) {
    cplex_lp model;
    build_dense_lp(model, 900);
    model.set_lp_algorithm(algorithm);
    model.set_thread_limit(threads);
    const std::clock_t cpu_start = std::clock();
    const auto wall_start = std::chrono::steady_clock::now();
    model.solve();
    const double cpu = static_cast<double>(std::clock() - cpu_start) / CLOCKS_PER_SEC;
    const std::chrono::duration<double> wall = std::chrono::steady_clock::now() - wall_start;
    std::println("{:36} threads={}: {:.2f} cores, {:.3f} s wall, optimal={}", name, threads,
                 cpu / wall.count(), wall.count(), is_a<status::optimal>(model.get_status()));
}

int main() {
    for(std::size_t threads : {0u, 1u}) {
        run("automatic", lp_algorithm::automatic{}, threads);
        run("primal_simplex", lp_algorithm::primal_simplex{}, threads);
        run("dual_simplex", lp_algorithm::dual_simplex{}, threads);
        run("interior_point", lp_algorithm::interior_point{}, threads);
        run("interior_point_without_crossover", lp_algorithm::interior_point_without_crossover{}, threads);
    }
}
