// X5: reject_solution() through the public API only. No native call anywhere.
#include <chrono>
#include <cstdio>
#include <print>
#include <random>
#include <ranges>
#include <string>
#include <typeinfo>
#include <variant>
#include <cxxabi.h>
#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/solvers/xpress/all.hpp"
using namespace mippp;
using namespace mippp::operators;
using M = mippp::xpress_milp;
static std::string st(const auto & s) {
    return std::visit([](const auto & t) { int e = 0; char * d = abi::__cxa_demangle(typeid(t).name(), nullptr, nullptr, &e); std::string n = d ? d : "?"; std::free(d); return n; }, s);
}
static std::string val(M & m) { try { return std::format("{}", m.get_solution_value()); } catch(const std::exception & e) { return "throws"; } }

// A: x,y integer in [0,5], x+y<=8, max x+y. Rejects only value 8: (5,2) = 7 is feasible and accepted.
static void tiny_int() {
    for(int threads : {1, 0}) {
        M m;
        if(threads) m.set_thread_limit(1);
        m.set_time_limit(std::chrono::duration<double>(10));
        auto x = m.add_integer_variable({.lower_bound = 0., .upper_bound = 5.});
        auto y = m.add_integer_variable({.lower_bound = 0., .upper_bound = 5.});
        m.add_constraint(x + y <= 8.);
        m.set_maximization();
        m.set_objective(x + y);
        int fired = 0, rej = 0;
        m.set_candidate_solution_callback([&](auto & h) {
            ++fired;
            auto s = h.get_solution();
            std::println(stderr, "   candidate x={} y={} value={}", s[x], s[y], h.get_solution_value());
            if(s[x] + s[y] > 7.5) { ++rej; h.reject_solution(); }
        });
        m.solve();
        std::println(stderr, "A tiny int threads={}: status={} value={} candidates={} rejected={} (truth: optimum 7 at (5,2) under the rejection rule)", threads ? 1 : 0, st(m.get_status()), val(m), fired, rej);
    }
}
// B: two binaries: max x + y, reject (1,1); (1,0) is feasible.
static void two_bin() {
    M m; m.set_thread_limit(1); m.set_time_limit(std::chrono::duration<double>(10));
    auto x = m.add_binary_variable(); auto y = m.add_binary_variable();
    m.set_maximization(); m.set_objective(x + y);
    int fired = 0, rej = 0;
    m.set_candidate_solution_callback([&](auto & h) { ++fired; auto s = h.get_solution(); if(s[x] + s[y] > 1.5) { ++rej; h.reject_solution(); } });
    m.solve();
    std::println(stderr, "B two binaries no row: status={} value={} candidates={} rejected={} (truth 1)", st(m.get_status()), val(m), fired, rej);
}
// B2: same, a row so that presolve has something: x + y <= 2 redundant; plus an LP-binding row.
static void two_bin_row() {
    M m; m.set_thread_limit(1); m.set_time_limit(std::chrono::duration<double>(10));
    auto x = m.add_binary_variable(); auto y = m.add_binary_variable(); auto z = m.add_binary_variable();
    m.add_constraint(x + y + z <= 3);
    m.set_maximization(); m.set_objective(3 * x + 2 * y + z);
    int fired = 0, rej = 0;
    m.set_candidate_solution_callback([&](auto & h) { ++fired; auto s = h.get_solution(); if(s[x] + s[y] > 1.5) { ++rej; h.reject_solution(); } });
    m.solve();
    std::println(stderr, "B2 three binaries redundant row: status={} value={} candidates={} rejected={} (truth 4 = x+z)", st(m.get_status()), val(m), fired, rej);
}
// C: 5x25 knapsack of the shipped test build_small (seed 7), minimize or maximize as there:
// refused_candidates_stay_refused rejects x0+x1 > 1.5 and only checks the point it gets.
template <class F>
static auto knapsack(M & m, std::size_t rows, std::size_t cols, bool maximize, F && extra) {
    std::mt19937 rng(7);
    std::uniform_int_distribution<int> coef(1, 1000), noise(0, 500);
    std::vector<std::vector<double>> a(rows, std::vector<double>(cols));
    for(auto & row : a) for(auto & v : row) v = coef(rng);
    std::vector<double> c(cols);
    for(std::size_t j = 0; j < cols; ++j) { double sum = 0; for(const auto & row : a) sum += row[j]; c[j] = std::floor(sum / static_cast<double>(rows) + noise(rng)); }
    auto x = m.add_binary_variables(cols);
    const auto ids = std::views::iota(std::size_t{0}, cols);
    if(maximize) m.set_maximization(); else m.set_minimization();
    m.set_objective(xsum(ids, [&](auto j) { return c[j] * x(j); }));
    for(const auto & row : a) {
        double sum = 0; for(double v : row) sum += v;
        if(maximize) m.add_constraint(xsum(ids, [&](auto j) { return row[j] * x(j); }) <= std::floor(sum / 2));
        else m.add_constraint(xsum(ids, [&](auto j) { return row[j] * x(j); }) >= std::floor(sum / 2));
    }
    extra(m, x);
    return x;
}
static void knapsack_gap(bool maximize) {
    double truth = NAN, plain = NAN;
    {
        M m; m.set_thread_limit(1); m.set_time_limit(std::chrono::duration<double>(60));
        auto x = knapsack(m, 5, 25, maximize, [](M &, auto &) {});
        m.solve(); plain = m.get_solution_value();
        std::println(stderr, "C{} no rule: status={} value={}", maximize ? "max" : "min", st(m.get_status()), val(m));
    }
    {
        M m; m.set_thread_limit(1); m.set_time_limit(std::chrono::duration<double>(60));
        auto x = knapsack(m, 5, 25, maximize, [](M & mm, auto & xx) { mm.add_constraint(xx(std::size_t{0}) + xx(std::size_t{1}) <= 1); });
        m.solve(); truth = m.get_solution_value();
        std::println(stderr, "C{} hard row x0+x1<=1: status={} value={}", maximize ? "max" : "min", st(m.get_status()), val(m));
    }
    {
        M m; m.set_thread_limit(1); m.set_time_limit(std::chrono::duration<double>(60));
        auto x = knapsack(m, 5, 25, maximize, [](M &, auto &) {});
        int fired = 0, rej = 0;
        m.set_candidate_solution_callback([&, x](auto & h) { ++fired; auto s = h.get_solution(); if(s[x(std::size_t{0})] + s[x(std::size_t{1})] > 1.5) { ++rej; h.reject_solution(); } });
        m.solve();
        std::println(stderr, "C{} rejection of x0+x1>1.5: status={} value={} candidates={} rejected={} (truth with the row {}, unconstrained {})", maximize ? "max" : "min", st(m.get_status()), val(m), fired, rej, truth, plain);
    }
}
// D: rejection that leaves an optimal status at a worse value: scan rules on small knapsacks.
static void scan() {
    int total = 0, infeas = 0, lost = 0, fine = 0, noreject = 0;
    std::string first_lost, first_inf;
    for(int seed = 1; seed <= 60; ++seed) {
        const std::size_t cols = 12;
        std::mt19937 rng(static_cast<unsigned>(seed));
        std::uniform_int_distribution<int> A(1, 30), C(1, 40), P(0, 11);
        std::vector<double> a(cols), c(cols);
        for(auto & v : a) v = A(rng);
        for(auto & v : c) v = C(rng);
        double cap = 0; for(double v : a) cap += v; cap = std::floor(cap * 0.5);
        const std::size_t i0 = static_cast<std::size_t>(P(rng)), i1 = (i0 + 1 + static_cast<std::size_t>(P(rng)) % 10) % cols;
        const auto ids = std::views::iota(std::size_t{0}, cols);
        const auto build = [&](M & m) {
            m.set_thread_limit(1); m.set_time_limit(std::chrono::duration<double>(10));
            auto x = m.add_binary_variables(cols);
            m.set_maximization();
            m.set_objective(xsum(ids, [&](auto j) { return c[j] * x(j); }));
            m.add_constraint(xsum(ids, [&](auto j) { return a[j] * x(j); }) <= cap);
            return x;
        };
        double truth;
        { M m; auto x = build(m); m.add_constraint(x(i0) + x(i1) <= 1); m.solve(); truth = m.get_solution_value(); }
        M m; auto x = build(m);
        int rej = 0;
        m.set_candidate_solution_callback([&, x](auto & h) { auto s = h.get_solution(); if(s[x(i0)] + s[x(i1)] > 1.5) { ++rej; h.reject_solution(); } });
        m.solve();
        ++total;
        const bool opt = is_a<status::optimal>(m.get_status());
        const bool inf = is_a<status::infeasible>(m.get_status());
        const double v = opt ? m.get_solution_value() : NAN;
        if(!rej) { ++noreject; continue; }
        if(inf) { ++infeas; if(first_inf.empty()) first_inf = std::format("seed {} rejected {} truth {}", seed, rej, truth); }
        else if(opt && std::fabs(v - truth) > 1e-6) { ++lost; if(first_lost.empty()) first_lost = std::format("seed {} status=optimal value {} truth {} rejected {}", seed, v, truth, rej); }
        else ++fine;
    }
    std::println(stderr, "D scan 12-col knapsacks: total={} no_rejection(no power)={} right={} false_infeasible={} false_optimal={} | first false optimal: {} | first false infeasible: {}", total, noreject, fine, infeas, lost, first_lost, first_inf);
}
int main() {
    tiny_int();
    two_bin();
    two_bin_row();
    knapsack_gap(false);
    knapsack_gap(true);
    scan();
}
