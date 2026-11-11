// What each model reports, under each tag it lists, on an LP that is primal
// and dual infeasible: max x + y over free x, y and z >= 0, under
// x - y + z <= -1 and y - x + z <= -1. refine_lp_status() is called where
// the model has it and the status is infeasible_or_unbounded.
#include <cxxabi.h>
#include <cstdlib>
#include <print>
#include <string>
#include <typeinfo>
#include <variant>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/solvers/clp/all.hpp"
#include "mippp/solvers/copt/all.hpp"
#include "mippp/solvers/cplex/all.hpp"
#include "mippp/solvers/glpk/all.hpp"
#include "mippp/solvers/gurobi/all.hpp"
#include "mippp/solvers/highs/all.hpp"
#include "mippp/solvers/mosek/all.hpp"
#include "mippp/solvers/soplex/all.hpp"
#include "mippp/solvers/xpress/all.hpp"

using namespace mippp;
using namespace mippp::operators;

template <class T>
std::string type_name() {
    int st = 0;
    char * d = abi::__cxa_demangle(typeid(T).name(), nullptr, nullptr, &st);
    std::string n = d ? d : "?";
    std::free(d);
    return n.substr(n.rfind("::") + 2);
}
template <class S>
std::string status_name(const S & s) {
    return std::visit([]<class T>(const T &) { return type_name<T>(); }, s);
}

template <class M>
std::string solved(M & m) {
    const double inf = m.infinity();
    auto x = m.add_variable({.lower_bound = -inf, .upper_bound = inf});
    auto y = m.add_variable({.lower_bound = -inf, .upper_bound = inf});
    auto z = m.add_variable();
    m.set_maximization();
    m.set_objective(x + y);
    m.add_constraint(x - y + z <= -1);
    m.add_constraint(y - x + z <= -1);
    try {
        m.solve();
        std::string first = status_name(m.get_status());
        if constexpr(has_refinable_lp_status<M>) {
            if(is<status::infeasible_or_unbounded>(m.get_status())) {
                m.refine_lp_status();
                return first + ", refined to " + status_name(m.get_status());
            }
        }
        return first;
    } catch(const std::exception & e) {
        return std::string("THROWS ") + e.what();
    }
}

template <class M, class A>
void run_tag() {
    if constexpr(has_lp_algorithm<M, A>) {
        M m;
        m.set_lp_algorithm(A{});
        std::println("{:10} {:33} {}", type_name<M>(), type_name<A>(), solved(m));
    }
}
template <class M>
void run() {
    if constexpr(has_lp_algorithm<M>) {
        run_tag<M, lp_algorithm::automatic>();
        run_tag<M, lp_algorithm::simplex>();
        run_tag<M, lp_algorithm::primal_simplex>();
        run_tag<M, lp_algorithm::dual_simplex>();
        run_tag<M, lp_algorithm::interior_point>();
        run_tag<M, lp_algorithm::interior_point_without_crossover>();
    } else {
        M m;
        std::println("{:10} {:33} {}", type_name<M>(), "(no selector)", solved(m));
    }
}

int main() {
    run<clp_lp>();
    run<glpk_lp>();
    run<soplex_lp>();
    run<highs_lp>();
    run<gurobi_lp>();
    run<cplex_lp>();
    run<xpress_lp>();
    run<copt_lp>();
    run<mosek_lp>();
}
