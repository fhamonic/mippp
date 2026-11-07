// Public API only. max x, x integer in [0,5]; the callback refuses the single candidate x = 5.
// Every other integer point stays feasible, so the right answer is 4.
#include <print>
#include "mippp/solvers/xpress/all.hpp"
using namespace mippp; using namespace mippp::operators;
template <class B> void run(const char * name, B build) {
    xpress_milp m;
    auto x = build(m);
    m.set_maximization();
    m.set_candidate_solution_callback([&](auto & h) {
        auto sol = h.get_solution();
        if(sol[x] > 4.5) h.reject_solution();
    });
    m.solve();
    const bool has = std::visit([](const auto & s) { if constexpr(requires { s.solution_available; }) return bool(s.solution_available); else return false; }, m.get_status());
    std::println("{}: optimal={} infeasible={} value={}", name, is_a<status::optimal>(m.get_status()), is_a<status::infeasible>(m.get_status()), has ? m.get_solution_value() : -1.0);
}
int main() {
    run("no rows", [](auto & m) { auto x = m.add_integer_variable({.obj_coef = 1, .lower_bound = 0, .upper_bound = 5}); return x; });
    run("row 2x<=10", [](auto & m) { auto x = m.add_integer_variable({.obj_coef = 1, .lower_bound = 0, .upper_bound = 5}); m.add_constraint(2.0 * x <= 10); return x; });
    run("row 2x<=9 (x<=4 is the truth)", [](auto & m) { auto x = m.add_integer_variable({.obj_coef = 1, .lower_bound = 0, .upper_bound = 5}); m.add_constraint(2.0 * x <= 9); return x; });
}
