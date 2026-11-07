// X5, smallest models. Public API only: no native call, no native control.
#include <chrono>
#include <cstdio>
#include <print>
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
static std::string val(M & m) { try { return std::format("{}", m.get_solution_value()); } catch(const std::exception &) { return "throws"; } }
int main() {
    { // 1 integer column, 0 rows
        M m; m.set_time_limit(std::chrono::duration<double>(10));
        auto x = m.add_integer_variable({.lower_bound = 0., .upper_bound = 5.});
        m.set_maximization(); m.set_objective(1.0 * x);
        int fired = 0, rej = 0; m.set_candidate_solution_callback([&](auto & h) { ++fired; auto s = h.get_solution(); if(s[x] > 4.5) { ++rej; h.reject_solution(); } });
        m.solve();
        std::println(stderr, "M1 max x, integer x in [0,5], rejects x=5: status={} value={} candidates={} rejected={} (x=4 is feasible, 4 expected)", st(m.get_status()), val(m), fired, rej);
    }
    { // 1 integer column, 1 row
        M m; m.set_time_limit(std::chrono::duration<double>(10));
        auto x = m.add_integer_variable({.lower_bound = 0., .upper_bound = 5.});
        m.add_constraint(2.0 * x <= 10.);
        m.set_maximization(); m.set_objective(1.0 * x);
        int fired = 0, rej = 0; m.set_candidate_solution_callback([&](auto & h) { ++fired; auto s = h.get_solution(); if(s[x] > 4.5) { ++rej; h.reject_solution(); } });
        m.solve();
        std::println(stderr, "M2 same with row 2x<=10: status={} value={} candidates={} rejected={} (4 expected)", st(m.get_status()), val(m), fired, rej);
    }
    { // 2 binaries, 1 row, optimum (1,1) has one sibling-free LP
        M m; m.set_time_limit(std::chrono::duration<double>(10));
        auto x = m.add_binary_variable(); auto y = m.add_binary_variable();
        m.add_constraint(x + y <= 2);
        m.set_maximization(); m.set_objective(x + y);
        int fired = 0, rej = 0; m.set_candidate_solution_callback([&](auto & h) { ++fired; auto s = h.get_solution(); if(s[x] + s[y] > 1.5) { ++rej; h.reject_solution(); } });
        m.solve();
        std::println(stderr, "M3 max x+y, binaries, row x+y<=2, rejects (1,1): status={} value={} candidates={} rejected={} (1 expected)", st(m.get_status()), val(m), fired, rej);
    }
    { // 2 integers, 1 row: integer LP vertex at root
        M m; m.set_time_limit(std::chrono::duration<double>(10));
        auto x = m.add_integer_variable({.lower_bound = 0., .upper_bound = 5.}); auto y = m.add_integer_variable({.lower_bound = 0., .upper_bound = 5.});
        m.add_constraint(x + y <= 8.);
        m.set_maximization(); m.set_objective(x + y);
        int fired = 0, rej = 0; m.set_candidate_solution_callback([&](auto & h) { ++fired; auto s = h.get_solution(); if(s[x] + s[y] > 7.5) { ++rej; h.reject_solution(); } });
        m.solve();
        std::println(stderr, "M4 max x+y, integers in [0,5], x+y<=8, rejects value 8: status={} value={} candidates={} rejected={} (7 expected)", st(m.get_status()), val(m), fired, rej);
    }
    { // same model, the rejection written as a row: what the docs promise
        M m; m.set_time_limit(std::chrono::duration<double>(10));
        auto x = m.add_integer_variable({.lower_bound = 0., .upper_bound = 5.}); auto y = m.add_integer_variable({.lower_bound = 0., .upper_bound = 5.});
        m.add_constraint(x + y <= 7.);
        m.set_maximization(); m.set_objective(x + y);
        m.solve();
        std::println(stderr, "M4 control, rule as a row x+y<=7: status={} value={}", st(m.get_status()), val(m));
    }
    { // 2 binaries, no row
        M m; m.set_time_limit(std::chrono::duration<double>(10));
        auto x = m.add_binary_variable(); auto y = m.add_binary_variable();
        m.set_maximization(); m.set_objective(x + y);
        int fired = 0, rej = 0; m.set_candidate_solution_callback([&](auto & h) { ++fired; auto s = h.get_solution(); if(s[x] + s[y] > 1.5) { ++rej; h.reject_solution(); } });
        m.solve();
        std::println(stderr, "M5 max x+y, 2 binaries, no row, rejects (1,1): status={} value={} candidates={} rejected={} (1 expected)", st(m.get_status()), val(m), fired, rej);
    }
    { // 3 binaries, the wrong optimal status: rule on the top item
        M m; m.set_time_limit(std::chrono::duration<double>(10));
        auto x = m.add_binary_variables(3);
        const std::size_t z = 0;
        m.add_constraint(5.0 * x(z) + 4.0 * x(z + 1) + 3.0 * x(z + 2) <= 7.0);
        m.set_maximization(); m.set_objective(6.0 * x(z) + 5.0 * x(z + 1) + 4.0 * x(z + 2));
        int fired = 0, rej = 0; m.set_candidate_solution_callback([&, x](auto & h) { ++fired; auto s = h.get_solution(); if(s[x(z + 1)] + s[x(z + 2)] > 1.5) { ++rej; h.reject_solution(); } });
        m.solve();
        std::println(stderr, "M6 knapsack 5a+4b+3c<=7, max 6a+5b+4c, rejects b+c=2: status={} value={} candidates={} rejected={} (6 expected, the rule leaves a alone or b or c)", st(m.get_status()), val(m), fired, rej);
    }
}
