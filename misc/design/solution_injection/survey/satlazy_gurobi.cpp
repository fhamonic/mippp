// What does Gurobi do with a candidate when the callback adds a lazy constraint
// that the candidate satisfies? Variants: the same row each time, or a fresh
// satisfied row each time (rhs 100 + call number).
#include "mippp/solvers/gurobi/all.hpp"
#include <print>
#include <string>
using namespace mippp;
using namespace mippp::operators;
int main(int argc, char ** argv) {
    const bool fresh = argc > 1;
    gurobi_milp model;
    auto x = model.add_integer_variable({.lower_bound = 0, .upper_bound = 5});
    auto y = model.add_integer_variable({.lower_bound = 0, .upper_bound = 5});
    model.set_maximization();
    model.set_objective(x + y);
    model.add_constraint(x + y <= 7);
    model.set_time_limit(std::chrono::duration<double>(10));
    int calls = 0;
    std::string seq;
    model.set_candidate_solution_callback([&](auto & handle) {
        ++calls;
        auto s = handle.get_solution();
        seq += std::format("({},{}) ", s[x], s[y]);
        if(calls <= 40) handle.add_lazy_constraint(x + y <= 100 + (fresh ? calls : 0));
    });
    model.solve();
    std::println("fresh={} calls={} candidates: {}-> value {}", fresh, calls, seq,
                 status::solution_available(model.get_status())
                     ? std::to_string(model.get_solution_value()) : "none");
}
