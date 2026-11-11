// Replays the operations of fuzz seed 98, after which gurobi_lp under the
// primal simplex read `unbounded` on a bounded LP without rows.
#include <cxxabi.h>
#include <cstdlib>
#include <print>
#include <string>
#include <typeinfo>
#include <variant>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/solvers/gurobi/all.hpp"

using namespace mippp;
using namespace mippp::operators;

template <class S>
std::string status_name(const S & s) {
    return std::visit(
        [](const auto & t) {
            int st = 0;
            char * d = abi::__cxa_demangle(typeid(t).name(), nullptr, nullptr, &st);
            std::string n = d ? d : "?";
            std::free(d);
            return n.substr(n.rfind("::") + 2);
        },
        s);
}

template <class A>
void run(const char * name, A algorithm, bool first_solve, bool remove) {
    gurobi_lp m;
    m.set_lp_algorithm(algorithm);
    const double inf = m.infinity();
    auto v = [&](double c, double lb, double ub) {
        return m.add_variable({.obj_coef = c, .lower_bound = lb, .upper_bound = ub});
    };
    auto x0 = v(-2.5, -1, inf), x1 = v(-2.5, -1, inf);
    auto x2 = v(2, -5, 3);
    m.set_objective(0.5 * x0 + -0.5 * x2 + -1 * x2 + 2 * x1 + 1);
    m.set_objective_coefficient(x2, 2.5);
    m.set_objective(2.5 * x0 + 1 * x2 + -2.5 * x2 + -8);
    m.set_objective_offset(-10);
    if(first_solve) m.solve();
    auto x3 = v(2.5, 1, 8);
    m.set_objective_offset(6);
    m.set_variable_lower_bound(x3, -1);
    m.set_variable_upper_bound(x3, 7);
    auto x4 = v(-0.5, -inf, inf);
    m.set_objective(-0.5 * x4 + -10);
    auto x5 = v(-1, 2, 10);
    m.set_objective_offset(5);
    m.set_variable_lower_bound(x4, 1);
    m.set_variable_upper_bound(x4, 7);
    auto x6 = v(0.5, -5, -1);
    auto x7 = v(-2, -1, 0);
    m.set_variable_lower_bound(x1, -2);
    m.set_variable_upper_bound(x1, -2);
    m.set_variable_lower_bound(x2, -3);
    m.set_variable_upper_bound(x2, 3);
    auto x8 = v(0.5, 0, 7);
    auto x9 = v(0, 1, 7);
    m.set_objective_coefficient(x6, -0.5);
    m.set_objective_coefficient(x9, -2.5);
    m.set_objective_offset(-4);
    m.set_variable_lower_bound(x1, -5);
    m.set_variable_upper_bound(x1, -3);
    if(remove) {
        auto x10 = v(1, -4, -2);
        m.remove_variable(x10);
    }
    (void)x5; (void)x7; (void)x8;
    m.solve();
    const auto [env, native] = m.native_model();
    const gurobi_api & grb = m.native_api();
    int status = -1, numvars = -1;
    grb.getintattr(native, "Status", &status);
    grb.getintattr(native, "NumVars", &numvars);
    double it = -1;
    grb.getdblattr(native, "IterCount", &it);
    std::println("{:26} first_solve={} remove={}: {} (Status {}, NumVars {}, IterCount {})", name,
                 first_solve, remove, status_name(m.get_status()), status, numvars, it);
    if(status == 5) {
        std::vector<double> obj(numvars), lb(numvars), ub(numvars);
        grb.getdblattrarray(native, "Obj", 0, numvars, obj.data());
        grb.getdblattrarray(native, "LB", 0, numvars, lb.data());
        grb.getdblattrarray(native, "UB", 0, numvars, ub.data());
        for(int j = 0; j < numvars; ++j)
            std::println("    col {}: obj {} in [{}, {}]", j, obj[j], lb[j], ub[j]);
        // the same Gurobi model, solved again by the dual simplex
        m.set_lp_algorithm(lp_algorithm::dual_simplex{});
        m.reset_status();
        m.solve();
        std::println("    same model under dual_simplex: {}", status_name(m.get_status()));
    }
}

int main() {
    for(bool first : {true, false})
        for(bool remove : {true, false}) {
            run("primal_simplex", lp_algorithm::primal_simplex{}, first, remove);
            run("dual_simplex", lp_algorithm::dual_simplex{}, first, remove);
            run("automatic", lp_algorithm::automatic{}, first, remove);
        }
}
