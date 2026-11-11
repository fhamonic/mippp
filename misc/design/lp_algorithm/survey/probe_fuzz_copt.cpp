// The LP on which COPT's barrier without crossover read `optimal` in the fuzz
// run of seed 2243283373, built directly: it is feasible and unbounded, x12
// to x14 being free below, in no row and of positive cost. copt_lp does not
// list the tag, so the two parameters are written through the native handle.
#include <cxxabi.h>
#include <cstdlib>
#include <limits>
#include <print>
#include <string>
#include <typeinfo>
#include <variant>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/solvers/clp/all.hpp"
#include "mippp/solvers/copt/all.hpp"
#include "mippp/solvers/gurobi/all.hpp"
#include "mippp/solvers/highs/all.hpp"

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

template <class M>
void build(M & m) {
    const double inf = m.infinity();
    auto v = [&](double lb, double ub, double c = 0.) {
        return m.add_variable({.obj_coef = c, .lower_bound = lb, .upper_bound = ub});
    };
    auto x0 = v(-2, 2), x1 = v(-5, -4), x2 = v(-5, -4), x3 = v(2, 2), x4 = v(2, 4);
    auto x5 = v(3, 10, -0.5), x6 = v(-inf, inf), x7 = v(-3, 0), x8 = v(-5, -2);
    auto x9 = v(-4, 2, 2.), x10 = v(-inf, 2), x11 = v(-1, 4), x12 = v(-inf, 2, 2.5);
    auto x13 = v(-inf, -4, 3.), x14 = v(-inf, -4, 3.), x15 = v(-1, -1, -2.5);
    (void)x2; (void)x7; (void)x8; (void)x10; (void)x12; (void)x13; (void)x14; (void)x15; (void)x5;
    m.set_minimization();
    m.set_objective_offset(-8);
    m.add_constraint(-3 * x1 + 2.5 * x6 + 2.5 * x9 - 0.5 * x11 <= 10);
    m.add_constraint(-0.5 * x0 - 3 * x6 + 2.5 * x9 == -5);
    m.add_constraint(2.5 * x3 + 2 * x6 + 2.5 * x9 + x11 >= -1);
    m.add_constraint(-1 * x4 <= 0);
    m.add_constraint(-1 * x4 + 3 * x0 == 3);
}

template <class M, class F>
void run(const char * name, F configure) {
    M m;
    build(m);
    configure(m);
    try {
        m.solve();
        std::string value = "-";
        if(is_a<status::optimal>(m.get_status())) value = std::to_string(m.get_solution_value());
        std::println("{:45} {} value={}", name, status_name(m.get_status()), value);
    } catch(const std::exception & e) {
        std::println("{:45} THROWS {}", name, e.what());
    }
}

int main() {
    run<clp_lp>("clp primal", [](auto &) {});
    run<highs_lp>("highs automatic", [](auto &) {});
    run<highs_lp>("highs interior_point", [](auto & m) { m.set_lp_algorithm(lp_algorithm::interior_point{}); });
    run<gurobi_lp>("gurobi automatic", [](auto &) {});
    run<gurobi_lp>("gurobi without crossover", [](auto & m) { m.set_lp_algorithm(lp_algorithm::interior_point_without_crossover{}); });
    run<copt_lp>("copt automatic", [](auto &) {});
    run<copt_lp>("copt dual", [](auto & m) { m.set_lp_algorithm(lp_algorithm::dual_simplex{}); });
    run<copt_lp>("copt interior_point", [](auto & m) { m.set_lp_algorithm(lp_algorithm::interior_point{}); });
    {
        copt_lp m;
        build(m);
        const auto [env, prob] = m.native_model();
        m.native_api()._check(env, m.native_api().SetIntParam(prob, "LpMethod", 2));
        m.native_api()._check(env, m.native_api().SetIntParam(prob, "Crossover", 0));
        m.solve();
        int lpstatus = -1, has = -1, bar = -1;
        m.native_api().GetIntAttr(prob, "LpStatus", &lpstatus);
        m.native_api().GetIntAttr(prob, "HasLpSol", &has);
        m.native_api().GetIntAttr(prob, "BarrierIter", &bar);
        double obj = 0;
        m.native_api().GetDblAttr(prob, "LpObjval", &obj);
        std::println("{:45} {} native LpStatus={} HasLpSol={} BarrierIter={} LpObjval={}",
                     "copt without crossover", status_name(m.get_status()), lpstatus, has, bar, obj);
    }
}
