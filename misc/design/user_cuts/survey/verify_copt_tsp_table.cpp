// Shipped API only: degree-only ATSP, subtours eliminated in a candidate callback, with the textbook row
// Sum_{i,j in S} x_ij <= |S| - 1 written two ways. "filter": only the arcs inside S. "table": one xsum over every
// arc with a coefficient that is 1 inside S and 0 outside, what one writes when a coefficient is data.
// usage: verify_copt_tsp_table <filter|table|ge_filter|ge_table|dup|empty_xsum> n seed
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <print>
#include <random>
#include <ranges>
#include <string>
#include <vector>
#include "mippp/solvers/copt/all.hpp"
using namespace mippp;
using namespace mippp::operators;
int main(int argc, char ** argv) {
    const std::string kase = argv[1];
    const std::size_t n = std::atoi(argv[2]);
    const unsigned seed = std::atoi(argv[3]);
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> P(0, 100), noise(1.0, 1.3);
    std::vector<std::pair<double, double>> pts(n);
    for(auto & p : pts) p = {P(rng), P(rng)};
    std::vector<double> d(n * n);
    for(std::size_t i = 0; i < n; ++i) for(std::size_t j = 0; j < n; ++j) if(i != j) d[i * n + j] = std::round(std::hypot(pts[i].first - pts[j].first, pts[i].second - pts[j].second) * noise(rng));
    copt_milp m;
    m.set_thread_limit(1);
    m.set_time_limit(std::chrono::duration<double>(30.0));
    auto x = m.add_binary_variables(n * n);
    const auto ids = std::views::iota(std::size_t{0}, n);
    const auto all = std::views::iota(std::size_t{0}, n * n);
    m.set_minimization();
    m.set_objective(xsum(all, [&](std::size_t a) { return d[a] * x(a); }));
    for(std::size_t i : ids) {
        m.set_variable_upper_bound(x(i * n + i), 0);
        m.add_constraint(xsum(ids, [&, i](std::size_t j) { return 1.0 * x(i * n + j); }) == 1);
        m.add_constraint(xsum(ids, [&, i](std::size_t j) { return 1.0 * x(j * n + i); }) == 1);
    }
    long calls = 0, lazies = 0;
    m.set_candidate_solution_callback([&](auto & h) {
        ++calls;
        auto sol = h.get_solution();
        std::vector<int> comp(n, -1);
        int nc = 0;
        for(std::size_t s : ids) {
            if(comp[s] >= 0) continue;
            for(std::size_t v = s; comp[v] < 0;) {
                comp[v] = nc;
                for(std::size_t w : ids) if(sol[x(v * n + w)] > 0.5) { v = w; break; }
            }
            ++nc;
        }
        if(nc == 1) return;
        for(int k = 0; k < nc; ++k) {
            std::vector<double> coef(n * n, 0.0);
            std::vector<std::size_t> inside;
            double size = 0;
            for(std::size_t v : ids) if(comp[v] == k) ++size;
            for(std::size_t i : ids) for(std::size_t j : ids) if(i != j && comp[i] == k && comp[j] == k) { coef[i * n + j] = 1.0; inside.push_back(i * n + j); }
            ++lazies;
            if(kase == "filter") h.add_lazy_constraint(xsum(inside, [&](std::size_t a) { return 1.0 * x(a); }) <= size - 1);
            else if(kase == "table") h.add_lazy_constraint(xsum(all, [&](std::size_t a) { return coef[a] * x(a); }) <= size - 1);
            else if(kase == "ge_filter") h.add_lazy_constraint(xsum(inside, [&](std::size_t a) { return -1.0 * x(a); }) >= 1 - size);
            else if(kase == "ge_table") h.add_lazy_constraint(xsum(all, [&](std::size_t a) { return (-coef[a]) * x(a); }) >= 1 - size);
            else if(kase == "dup") h.add_lazy_constraint(distinct_variables, xsum(inside, [&](std::size_t a) { return 0.5 * x(a); }) + xsum(inside, [&](std::size_t a) { return 0.5 * x(a); }) <= size - 1);
            else if(kase == "nan") { h.add_lazy_constraint(xsum(inside, [&](std::size_t a) { return std::nan("") * x(a); }) <= size - 1); }
            else if(kase == "empty_xsum") { const std::vector<std::size_t> none; h.add_lazy_constraint(xsum(none, [&](std::size_t a) { return 1.0 * x(a); }) <= 1.0); h.add_lazy_constraint(xsum(inside, [&](std::size_t a) { return 1.0 * x(a); }) <= size - 1); }
        }
    });
    try { m.solve(); } catch(const std::exception & e) { std::println(stderr, "solve threw {}", e.what()); }
    if(!status::solution_available(m.get_status())) { std::println(stderr, "{:10} n={} seed={} status_index={} (3 = infeasible) NO SOLUTION nodes={} calls={} lazies={}", kase, n, seed, m.get_status().index(), m.get_node_count(), calls, lazies); return 0; }
    auto sol = m.get_solution();
    std::vector<char> seen(n, 0); std::size_t tour = 0;
    for(std::size_t v = 0; !seen[v];) { seen[v] = 1; ++tour; for(std::size_t w : ids) if(sol[x(v * n + w)] > 0.5) { v = w; break; } }
    std::println(stderr, "{:10} n={} seed={} optimal={} value={} dual={} nodes={} calls={} lazies={} first cycle covers {}/{} cities", kase, n, seed, is_a<status::optimal>(m.get_status()), m.get_solution_value(), m.get_dual_bound(), m.get_node_count(), calls, lazies, tour, n);
}
