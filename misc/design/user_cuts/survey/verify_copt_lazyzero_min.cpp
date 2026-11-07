// Standalone: only the shipped header, public API, no access-control flag.
// usage: verify_copt_lazyzero_min <compact|compact_coal|plain|coal|coal_bin|times0|xsum_zero> N extra K seed
//   compact      all link rows in the model, no callback (the true optimum)
//   compact_coal same, the rows written with x - x, through add_constraint (COPT_AddRow)
//   plain        link rows hidden, added by add_lazy_constraint when a candidate violates them
//   coal         same rows, written  row - cap y + s - s <= 0  (s a flow column of another arc)
//   coal_bin     same with s a binary of another arc
//   times0       same with 0.0 * s
//   xsum_zero    same, the row spelt as an xsum over a coefficient table holding a 0
#include <cstdio>
#include <print>
#include <random>
#include <ranges>
#include <string>
#include <vector>
#include <chrono>
#include <cmath>
#include <algorithm>
#include "mippp/solvers/copt/all.hpp"
using namespace mippp;
using namespace mippp::operators;
int main(int argc, char ** argv) {
    const std::string kase = argv[1];
    const std::size_t N = std::atoi(argv[2]), EX = std::atoi(argv[3]), K = std::atoi(argv[4]);
    const unsigned seed = std::atoi(argv[5]);
    std::mt19937 rng(seed);
    std::uniform_int_distribution<std::size_t> node(0, N - 1);
    std::uniform_int_distribution<int> dem(1, 9), cost(1, 8), fix(20, 90);
    std::vector<std::pair<std::size_t, std::size_t>> arcs;
    for(std::size_t i = 0; i < N; ++i) { arcs.emplace_back(i, (i + 1) % N); arcs.emplace_back((i + 1) % N, i); }
    for(std::size_t a = 0; a < EX; ++a) { auto u = node(rng), v = node(rng); if(u != v) arcs.emplace_back(u, v); }
    const std::size_t A = arcs.size();
    struct commodity { std::size_t o, t; double d; };
    std::vector<commodity> com;
    double total = 0;
    while(com.size() < K) { auto o = node(rng), t = node(rng); if(o == t) continue; const double dk = dem(rng); com.push_back({o, t, dk}); total += dk; }
    const double cap = std::ceil(total * 0.5);
    std::vector<double> ce(A), fe(A);
    for(auto & v : ce) v = cost(rng);
    for(auto & v : fe) v = fix(rng);

    copt_milp m;
    m.set_thread_limit(1);
    m.set_time_limit(std::chrono::duration<double>(20.0));
    auto y = m.add_binary_variables(A);
    auto x = m.add_variables(K * A);
    const auto id = [A](std::size_t k, std::size_t e) { return k * A + e; };
    const auto Es = std::views::iota(std::size_t{0}, A);
    const auto Ks = std::views::iota(std::size_t{0}, K);
    m.set_minimization();
    m.set_objective(xsum(Es, [&](std::size_t e) { return fe[e] * y(e); }) +
                    xsum(Ks, [&](std::size_t k) { return xsum(Es, [&, k](std::size_t e) { return ce[e] * x(id(k, e)); }); }));
    for(std::size_t k : Ks) {
        for(std::size_t v = 0; v < N; ++v) {
            const double b = v == com[k].o ? com[k].d : v == com[k].t ? -com[k].d : 0.0;
            m.add_constraint(xsum(Es | std::views::filter([&, v](std::size_t e) { return arcs[e].first == v; }), [&, k](std::size_t e) { return 1.0 * x(id(k, e)); })
                           - xsum(Es | std::views::filter([&, v](std::size_t e) { return arcs[e].second == v; }), [&, k](std::size_t e) { return 1.0 * x(id(k, e)); }) == b);
        }
    }
    const auto link = [&](std::size_t e) { return xsum(Ks, [&, e](std::size_t k) { return 1.0 * x(id(k, e)); }) - cap * y(e); };
    const auto cs = [&](std::size_t e) { return x(id((e * 3 + 1) % K, (e + 5) % A)); };
    if(kase == "compact") for(std::size_t e : Es) m.add_constraint(link(e) <= 0.0);
    if(kase == "compact_coal") for(std::size_t e : Es) m.add_constraint(link(e) + 1.0 * cs(e) - 1.0 * cs(e) <= 0.0);
    long calls = 0, lazies = 0;
    if(!kase.starts_with("compact")) m.set_candidate_solution_callback([&](auto & h) {
        ++calls;
        auto sol = h.get_solution();
        for(std::size_t e : Es) {
            double s = 0;
            for(std::size_t k : Ks) s += sol[x(id(k, e))];
            if(s <= cap * sol[y(e)] + 1e-6) continue;
            ++lazies;
            if(kase == "plain") h.add_lazy_constraint(link(e) <= 0.0);
            else if(kase == "coal") h.add_lazy_constraint(link(e) + 1.0 * cs(e) - 1.0 * cs(e) <= 0.0);
            else if(kase == "coal_bin") h.add_lazy_constraint(link(e) + 1.0 * y((e + 5) % A) - 1.0 * y((e + 5) % A) <= 0.0);
            else if(kase == "times0") h.add_lazy_constraint(link(e) + 0.0 * cs(e) <= 0.0);
            else if(kase == "xsum_zero") {
                // a coefficient table with one 0: what a user writes when a coefficient is data
                std::vector<double> coef(K, 1.0);
                h.add_lazy_constraint(xsum(Ks, [&, e](std::size_t k) { return coef[k] * x(id(k, e)); }) - cap * y(e) + xsum(Ks, [&, e](std::size_t k) { return (k == 0 ? 0.0 : 0.0) * x(id(k, (e + 5) % A)); }) <= 0.0);
            }
        }
    });
    try { m.solve(); } catch(const std::exception & e) { std::println(stderr, "solve threw {}", e.what()); }
    const bool opt = is_a<status::optimal>(m.get_status());
    if(!status::solution_available(m.get_status())) {
        std::println(stderr, "{:12} seed={} optimal={} status_index={} NO SOLUTION, value={} nodes={} calls={} lazies={}", kase, seed, opt, m.get_status().index(), m.get_solution_value(), m.get_node_count(), calls, lazies);
        return 0;
    }
    auto sol = m.get_solution();
    double worst = 0, obj = 0;
    for(std::size_t e : Es) {
        double s = 0;
        for(std::size_t k : Ks) { s += sol[x(id(k, e))]; obj += ce[e] * sol[x(id(k, e))]; }
        obj += fe[e] * sol[y(e)];
        worst = std::max(worst, s - cap * sol[y(e)]);
    }
    std::println(stderr, "{:12} seed={} optimal={} value={} dual={} nodes={} calls={} lazies={} | recomputed obj={} worst link violation={}", kase, seed, opt, m.get_solution_value(), m.get_dual_bound(), m.get_node_count(), calls, lazies, obj, worst);
}
