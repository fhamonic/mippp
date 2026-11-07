#pragma once
// Shared harness: each backend probe defines its configs and counters, then
// calls run_all<M>(configs, counters).
#include <cxxabi.h>
#include <dlfcn.h>

#include <chrono>
#include <cmath>
#include <format>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <print>
#include <random>
#include <string>
#include <typeinfo>
#include <utility>
#include <variant>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"

using namespace mippp;
using namespace mippp::operators;

inline std::string short_name(const char * mangled) {
    int st = 0;
    char * d = abi::__cxa_demangle(mangled, nullptr, nullptr, &st);
    std::string s = d ? d : mangled;
    std::free(d);
    auto p = s.rfind("::");
    return p == std::string::npos ? s : s.substr(p + 2);
}
template <class S>
std::string status_name(const S & s) {
    return std::visit(
        [](const auto & t) { return short_name(typeid(t).name()); }, s);
}

inline void * sym(const std::filesystem::path & lib, const char * name) {
    void * h = dlopen(lib.c_str(), RTLD_NOW | RTLD_NOLOAD);
    if(!h) {
        std::println("dlopen NOLOAD failed: {}", dlerror());
        std::exit(2);
    }
    void * f = dlsym(h, name);
    if(!f) {
        std::println("missing symbol {}", name);
        std::exit(2);
    }
    return f;
}

template <class M>
struct probe_config {
    std::string name;
    std::function<void(M &)> apply;
};

constexpr int N = 120, R = 80;

// max c.x, A x <= b, 0 <= x <= 10, A >= 0: feasible and bounded
template <class M>
auto build_feasible(M & m, std::vector<double> * row_out = nullptr) {
    std::mt19937 rng(12345);
    std::uniform_real_distribution<double> c(1, 10), a(0, 5), b(50, 150),
        u(0, 1);
    auto x = m.add_variables(N, {.lower_bound = 0, .upper_bound = 10});
    m.set_maximization();
    std::vector<double> obj(N);
    for(auto & v : obj) v = c(rng);
    m.set_objective(xsum(std::views::iota(0, N),
                         [&](int j) { return obj[std::size_t(j)] * x(j); }));
    for(int i = 0; i < R; ++i) {
        std::vector<double> row(N);
        for(auto & v : row) v = u(rng) < 0.5 ? a(rng) : 0.0;
        const double rhs = b(rng);
        m.add_constraint(xsum(std::views::iota(0, N), [&, row](int j) {
                             return row[std::size_t(j)] * x(j);
                         }) <= rhs);
    }
    if(row_out) {
        row_out->resize(N);
        for(auto & v : *row_out) v = a(rng);
    }
    return x;
}

// feasible system plus a' x >= 1.2 * max a' x: presolve cannot see it
template <class M>
void build_infeasible(M & m, double max_row) {
    std::vector<double> row;
    auto x = build_feasible(m, &row);
    m.add_constraint(xsum(std::views::iota(0, N), [&](int j) {
                         return row[std::size_t(j)] * x(j);
                     }) >= 1.2 * max_row);
}
template <class M>
double max_of_extra_row() {
    M m;
    std::vector<double> row;
    auto x = build_feasible(m, &row);
    m.set_objective(xsum(std::views::iota(0, N),
                         [&](int j) { return row[std::size_t(j)] * x(j); }));
    m.solve();
    return m.get_solution_value();
}

// x >= 0, A x <= b with mixed signs, ray d = (1, ..., 1): A d = 0 and
// c.d = 1, with the last column cancelling the others in every row
template <class M>
void build_unbounded(M & m) {
    std::mt19937 rng(777);
    std::uniform_real_distribution<double> c(1, 10), a(-1, 5), b(50, 150);
    auto x = m.add_variables(N);
    m.set_maximization();
    std::vector<double> obj(N);
    double sum = 0;
    for(int j = 0; j < N - 1; ++j) sum += (obj[std::size_t(j)] = c(rng));
    obj[N - 1] = 1.0 - sum;
    m.set_objective(xsum(std::views::iota(0, N),
                         [&](int j) { return obj[std::size_t(j)] * x(j); }));
    for(int i = 0; i < R; ++i) {
        std::vector<double> row(N);
        double rs = 0;
        for(int j = 0; j < N - 1; ++j) rs += (row[std::size_t(j)] = a(rng));
        row[N - 1] = -rs;
        const double rhs = b(rng);
        m.add_constraint(xsum(std::views::iota(0, N), [&, row](int j) {
                             return row[std::size_t(j)] * x(j);
                         }) <= rhs);
    }
}

template <class F>
std::string attempt(F && f) {
    try {
        f();
        return "ok";
    } catch(const std::exception & e) {
        std::string w = e.what();
        if(w.size() > 70) w = w.substr(0, 70) + "...";
        return "THROWS(" + w + ")";
    }
}


// the instances of test/test_suites/lp_status.hpp, refined as the suite does
template <class M>
std::string tiny_status(const probe_config<M> & cfg, int which) {
    M m;
    auto x = m.add_variable({.upper_bound = 3});
    auto y = m.add_variable({.lower_bound = 1});
    std::string expected;
    switch(which) {
        case 0: m.set_maximization(); m.set_objective(2 * x + 1.5 * y);
                m.add_constraint(x + y <= 5); expected = "optimal"; break;
        case 1: m.set_minimization(); m.set_objective(-2 * x - 1.5 * y);
                m.add_constraint(x + y <= 5); expected = "optimal"; break;
        case 2: m.set_maximization(); m.set_objective(-2 * x + -1.5 * y);
                m.add_constraint(x + y <= 5); expected = "unbounded"; break;
        case 3: m.set_minimization(); m.set_objective(2 * x + 1.5 * y);
                m.add_constraint(x + y <= 5); expected = "unbounded"; break;
        case 4: {
            M n;
            auto a = n.add_variable();
            auto b = n.add_variable({.upper_bound = 1});
            n.set_maximization(); n.set_objective(-2 * a + -1.5 * b);
            n.add_constraint(b >= -2 + 2 * a); n.add_constraint(b >= 3 - a);
            cfg.apply(n);
            if constexpr(has_time_limit<M>) n.set_time_limit(std::chrono::seconds(5));
            std::string r = attempt([&] { n.solve(); });
            if constexpr(has_refinable_lp_status<M>)
                if(is<status::infeasible_or_unbounded>(n.get_status()))
                    r += "+refine " + attempt([&] { n.refine_lp_status(); });
            auto got = status_name(n.get_status());
            return (got == "infeasible" ? "pass" : "FAIL(" + got + " " + r + ")");
        }
    }
    cfg.apply(m);
    if constexpr(has_time_limit<M>) m.set_time_limit(std::chrono::seconds(5));
    std::string r = attempt([&] { m.solve(); });
    if constexpr(has_refinable_lp_status<M>)
        if(is<status::infeasible_or_unbounded>(m.get_status()))
            r += "+refine " + attempt([&] { m.refine_lp_status(); });
    auto got = status_name(m.get_status());
    std::string extra;
    if(which < 2 && got == "optimal") {
        auto sol = m.get_solution();
        if(std::abs(sol[x] - 3) > 1e-6 || std::abs(sol[y] - 2) > 1e-6)
            extra = std::format(" WRONG x={} y={}", sol[x], sol[y]);
    }
    return (got == expected && extra.empty()) ? "pass"
                                              : "FAIL(" + got + " " + r + extra + ")";
}

inline std::function<void()> reset_hooks = [] {};

template <class M>
void run_all(const std::vector<probe_config<M>> & configs,
             std::function<std::string(M &)> counters) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    std::println("library: {}", M().native_api().library_path().string());
    reset_hooks();
    const double max_row = max_of_extra_row<M>();
    for(const auto & cfg : configs) {
        std::println("=== {}", cfg.name);
        {
            reset_hooks();
            M m;
            build_feasible(m);
            std::string applied = attempt([&] { cfg.apply(m);
            if constexpr(has_time_limit<M>) m.set_time_limit(std::chrono::seconds(10)); });
            if(applied != "ok") {
                std::println("  apply: {}", applied);
                continue;
            }
            const auto t0 = std::chrono::steady_clock::now();
            std::string solved = attempt([&] { m.solve(); });
            const double ms = std::chrono::duration<double, std::milli>(
                                  std::chrono::steady_clock::now() - t0)
                                  .count();
            std::println("  feasible : solve {} status={} {:.0f}ms {}", solved,
                         status_name(m.get_status()), ms, counters(m));
            double obj = 0;
            std::string o = attempt([&] { obj = m.get_solution_value(); });
            std::println("    obj={} ({}) sol={} dual={} rc={}", obj, o,
                         attempt([&] { (void)m.get_solution(); }),
                         attempt([&] {
                             if constexpr(has_dual_solution<M>) (void)m.get_dual_solution();
                             else throw std::runtime_error("n/a");
                         }),
                         attempt([&] {
                             if constexpr(has_reduced_costs<M>) (void)m.get_reduced_costs();
                             else throw std::runtime_error("n/a");
                         }));
            if constexpr(has_lp_basis<M>)
                std::println("    basis={}",
                             attempt([&] { (void)m.get_basis(); }));
        }
        {
            reset_hooks();
            M m;
            build_infeasible(m, max_row);
            cfg.apply(m);
            if constexpr(has_time_limit<M>) m.set_time_limit(std::chrono::seconds(10));
            std::string solved = attempt([&] { m.solve(); });
            std::println("  infeasible: solve {} status={} {}", solved,
                         status_name(m.get_status()), counters(m));
            if constexpr(has_refinable_lp_status<M>) {
                std::string r = attempt([&] { m.refine_lp_status(); });
                std::println("    refine_lp_status {} -> {}", r,
                             status_name(m.get_status()));
            }
        }
        {
            reset_hooks();
            M m;
            build_unbounded(m);
            cfg.apply(m);
            if constexpr(has_time_limit<M>) m.set_time_limit(std::chrono::seconds(10));
            std::string solved = attempt([&] { m.solve(); });
            std::println("  unbounded: solve {} status={} {}", solved,
                         status_name(m.get_status()), counters(m));
            if constexpr(has_refinable_lp_status<M>) {
                std::string r = attempt([&] { m.refine_lp_status(); });
                std::println("    refine_lp_status {} -> {}", r,
                             status_name(m.get_status()));
            }
        }
        if constexpr(has_iteration_limit<M>) {
            reset_hooks();
            M m;
            build_feasible(m);
            cfg.apply(m);
            if constexpr(has_time_limit<M>) m.set_time_limit(std::chrono::seconds(10));
            m.set_iteration_limit(1);
            std::string solved = attempt([&] { m.solve(); });
            std::println("  iterlim=1: solve {} status={} {}", solved,
                         status_name(m.get_status()), counters(m));
        }
        if constexpr(has_lp_basis_warm_start<M>) {
            reset_hooks();
            M m;
            build_feasible(m);
            m.solve();
            auto basis = m.get_basis();
            m.set_basis(basis);
            cfg.apply(m);
            if constexpr(has_time_limit<M>) m.set_time_limit(std::chrono::seconds(10));
            std::string solved = attempt([&] { m.solve(); });
            std::println("  warm from optimal basis: solve {} status={} {}",
                         solved, status_name(m.get_status()), counters(m));
        }
        {
            reset_hooks();
            M m;
            build_feasible(m);
            m.solve();
            cfg.apply(m);
            if constexpr(has_time_limit<M>) m.set_time_limit(std::chrono::seconds(10));
            std::println("  results after setting it post-solve: sol={}",
                         attempt([&] { (void)m.get_solution(); }));
        }
        reset_hooks();
        std::print("  lp_status suite:");
        for(int w = 0; w < 5; ++w) std::print(" {}", tiny_status(cfg, w));
        std::println("");
    }
}
