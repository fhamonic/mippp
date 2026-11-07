// A separator with memory keeps the last point it saw, to compare the next one
// with it. On Gurobi the candidate handle's mapping captures the handle's this,
// which lives in main_callback's frame (gurobi_milp.hpp:167-170, 209-211).
#include <optional>
#include <print>
#include "mippp/solvers/gurobi/all.hpp"
using namespace mippp;
using namespace mippp::operators;

int main() {
    gurobi_milp m;
    auto x = m.add_binary_variables(3);
    m.set_maximization();
    m.set_objective(1.0 * x(std::size_t{0}) + 2.0 * x(std::size_t{1}) +
                    3.0 * x(std::size_t{2}));
    m.add_constraint(x(std::size_t{0}) + x(std::size_t{1}) + x(std::size_t{2}) <=
                     2);
    m.set_time_limit(std::chrono::seconds(10));
    using H = candidate_solution_callback_handle_t<gurobi_milp>;
    std::optional<decltype(std::declval<H &>().get_solution())> last;
    long calls = 0;
    m.set_candidate_solution_callback([&](auto & h) {
        ++calls;
        if(last) std::println("previous point x2 = {}", (*last)[x(std::size_t{2})]);
        last.emplace(h.get_solution());
    });
    m.solve();
    std::println("calls={} after solve: stored x2 = {}", calls,
                 (*last)[x(std::size_t{2})]);
}
