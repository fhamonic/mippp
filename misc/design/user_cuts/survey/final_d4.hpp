#pragma once
// D4 through the public API only (after own_gurobi_late2.cpp): max x0 + 2 x1 + 3 x2,
// x0 + x1 + x2 <= 2, binaries; optimum 5 at (0,1,1). A candidate callback forbids
// x1 + x2 = 2 (lazy row x1 + x2 <= 1, or reject_solution on Xpress): the truth is 4.
#include <chrono>
#include <print>
#include <string>
#include <variant>
using namespace mippp;
using namespace mippp::operators;
template <class M>
void d4_run(const char * name, const int when) {  // 0: set after a finished solve; 1: same, then a model change; 2: set before the first solve; 3: same as 0, then a parameter write
    M m;
    auto x = m.add_binary_variables(3);
    m.set_maximization();
    m.set_objective(1.0 * x(std::size_t{0}) + 2.0 * x(std::size_t{1}) + 3.0 * x(std::size_t{2}));
    m.add_constraint(x(std::size_t{0}) + x(std::size_t{1}) + x(std::size_t{2}) <= 2);
    long calls = 0, acts = 0;
    const auto set = [&] {
        m.set_candidate_solution_callback([&, y1 = x(std::size_t{1}), y2 = x(std::size_t{2})](auto & h) {
            ++calls;
            auto sol = h.get_solution();
            if(sol[y1] + sol[y2] > 1.5) {
                ++acts;
                if constexpr(requires { h.add_lazy_constraint(y1 + y2 <= 1); }) h.add_lazy_constraint(y1 + y2 <= 1);
                else h.reject_solution();
            }
        });
    };
    double first = -1;
    if(when == 2) set();
    else {
        m.solve();
        first = m.get_solution_value();
        set();
        if(when == 1) m.set_objective_coefficient(x(std::size_t{0}), 1.0);
        if(when == 3) m.set_time_limit(std::chrono::duration<double>(100.0));
    }
    m.solve();
    const bool has = std::visit([](const auto & s) { if constexpr(requires { s.solution_available; }) return bool(s.solution_available); else return false; }, m.get_status());
    std::string point = "none";
    double second = -1;
    if(has) {
        second = m.get_solution_value();
        auto sol = m.get_solution();
        point = std::format("({},{},{})", sol[x(std::size_t{0})], sol[x(std::size_t{1})], sol[x(std::size_t{2})]);
    }
    std::println("{}: first={} second={} optimal={} infeasible={} calls={} acted={} point={}", name, first, second, is_a<status::optimal>(m.get_status()), is_a<status::infeasible>(m.get_status()), calls, acts, point);
}
template <class M>
int d4_main(const char * backend) {
    std::println("== D4 {}", backend);
    d4_run<M>("set after a finished solve, no change", 0);
    d4_run<M>("control: set after a finished solve, then set_objective_coefficient(x0, 1)", 1);
    d4_run<M>("set after a finished solve, then set_time_limit(100 s) only", 3);
    d4_run<M>("reference: set before the first solve", 2);
    return 0;
}
