// Probe (see column_manager_tree_rr.md): what the solution accessors return
// after adding a column once the model is solved, per LP backend.
// From this directory: g++ -std=c++26 -I../../../include add_after_solve.cpp -ldl
#include <exception>
#include <print>

#include "mippp/solvers/clp/all.hpp"
#include "mippp/solvers/copt/all.hpp"
#include "mippp/solvers/cplex/all.hpp"
#include "mippp/solvers/glpk/all.hpp"
#include "mippp/solvers/gurobi/all.hpp"
#include "mippp/solvers/highs/all.hpp"
#include "mippp/solvers/mosek/all.hpp"
#include "mippp/solvers/xpress/all.hpp"

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
        auto c = model.add_constraint(x + y <= 1);
        model.solve();
        std::println("{:7} before: value {} y {}", name,
                     model.get_solution_value(), model.get_solution()[y]);
        model.add_column({{c, 1.0}},
                         {.obj_coef = 3, .lower_bound = 0, .upper_bound = 1});
        try {
            const double v = model.get_solution_value();
            auto s = model.get_solution();
            std::println("{:7} after : value {} y {}", name, v, s[y]);
        } catch(const std::exception & e) {
            std::println("{:7} after : throws '{}'", name, e.what());
        }
    } catch(const std::exception & e) {
        std::println("{:7} setup throws '{}'", name, e.what());
    }
}

int main() {
    probe<gurobi_lp>("gurobi");
    probe<cplex_lp>("cplex");
    probe<copt_lp>("copt");
    probe<xpress_lp>("xpress");
    probe<mosek_lp>("mosek");
    probe<highs_lp>("highs");
    probe<clp_lp>("clp");
    probe<glpk_lp>("glpk");
}
