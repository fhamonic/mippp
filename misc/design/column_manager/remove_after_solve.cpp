// Probe (see column_manager_tree_rr.md): what the solution accessors return
// after removing a nonbasic column once the model is solved, per LP backend.
// From this directory: g++ -std=c++26 -I../../../include remove_after_solve.cpp -ldl
#include <exception>
#include <print>
#include <string>

#include "mippp/solvers/clp/all.hpp"
#include "mippp/solvers/cplex/all.hpp"
#include "mippp/solvers/gurobi/all.hpp"
#include "mippp/solvers/highs/all.hpp"

using namespace mippp;
using namespace mippp::operators;

template <typename M>
void probe(const char * name) {
    try {
        M model;
        model.set_maximization();
        auto x = model.add_variable(
            {.obj_coef = 1, .lower_bound = 0, .upper_bound = 1});
        auto y = model.add_variable(
            {.obj_coef = 2, .lower_bound = 0, .upper_bound = 1});
        auto z = model.add_variable(
            {.obj_coef = 0.1, .lower_bound = 0, .upper_bound = 1});
        model.add_constraint(x + y + z <= 1);
        model.solve();
        std::println("{:7} before: value {} y {}", name,
                     model.get_solution_value(), model.get_solution()[y]);
        model.remove_variable(z);  // nonbasic at 0
        try {
            auto s = model.get_solution();
            std::println("{:7} after : value {} y {} x {}", name,
                         model.get_solution_value(), s[y], s[x]);
        } catch(const std::exception & e) {
            std::println("{:7} after : throws '{}'", name, e.what());
        }
    } catch(const std::exception & e) {
        std::println("{:7} setup throws '{}'", name, e.what());
    }
}

int main() {
    probe<gurobi_lp>("gurobi");
    probe<highs_lp>("highs");
    probe<clp_lp>("clp");
    probe<cplex_lp>("cplex");
}
