#include <print>
#include "mippp/solvers/gurobi/all.hpp"
using namespace mippp; using namespace mippp::operators;
template <class After> void run(const char * name, After after) {
    gurobi_milp m;
    auto x = m.add_binary_variables(3);
    m.set_maximization();
    m.set_objective(1.0 * x(std::size_t{0}) + 2.0 * x(std::size_t{1}) + 3.0 * x(std::size_t{2}));
    m.add_constraint(x(std::size_t{0}) + x(std::size_t{1}) + x(std::size_t{2}) <= 2);
    m.solve();
    const double first = m.get_solution_value();
    long calls = 0, adds = 0;
    m.set_candidate_solution_callback([&, y1 = x(std::size_t{1}), y2 = x(std::size_t{2})](auto & h) {
        ++calls;
        auto sol = h.get_solution();
        if(sol[y1] + sol[y2] > 1.5) { ++adds; h.add_lazy_constraint(y1 + y2 <= 1); }
    });
    after(m, x);
    m.solve();
    auto sol = m.get_solution();
    std::println("{}: first={} second={} calls={} lazy_added={} point=({},{},{})", name, first, m.get_solution_value(), calls, adds, sol[x(std::size_t{0})], sol[x(std::size_t{1})], sol[x(std::size_t{2})]);
}
int main() {
    run("set after a finished solve", [](auto &, auto &) {});
    run("control: set before any solve needs no second model", [](auto & m, auto & x) { m.set_objective_coefficient(x(std::size_t{0}), 1.0); });
}
