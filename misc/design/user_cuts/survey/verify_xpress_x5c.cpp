// X5 follow-up, public API only: the deletion filter's premise is a callback whose decision "depends on that point alone".
// This one does. On Xpress a rejection drops the node, so does the filter still say feasible?
#include <chrono>
#include <cstdio>
#include <print>
#include <string>
#include <typeinfo>
#include <variant>
#include <cxxabi.h>
#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/utility/iis_by_deletion.hpp"
#include "mippp/solvers/xpress/all.hpp"
using namespace mippp;
using namespace mippp::operators;
using M = mippp::xpress_milp;
static std::string nm(const auto & s) {
    return std::visit([](const auto & t) { int e = 0; char * d = abi::__cxa_demangle(typeid(t).name(), nullptr, nullptr, &e); std::string n = d ? d : "?"; std::free(d); return n; }, s);
}
int main() {
    for(const double rhs : {8., 11.}) {
        M m;
        auto x = m.add_integer_variable({.lower_bound = 0., .upper_bound = 5.});
        auto y = m.add_integer_variable({.lower_bound = 0., .upper_bound = 5.});
        auto r = rhs < 10 ? m.add_constraint(x + y <= rhs) : m.add_constraint(x + y >= rhs);
        m.set_maximization(); m.set_objective(x + y);
        int fired = 0;
        // rejects value 8 only: a rule on the point alone; (5,2) passes it and is feasible
        m.set_candidate_solution_callback([&](auto & h) { ++fired; auto s = h.get_solution(); if(s[x] + s[y] > 7.5) h.reject_solution(); });
        const auto iis = compute_iis_by_deletion(m, iis_limits{.time_limit = std::chrono::duration<double>(10)});
        std::println(stderr, "filter, row x+y {} {}, rule rejects value 8: outcome {} members vars={} rows={} candidates seen={} | x {} y {} row {}", rhs < 10 ? "<=" : ">=", rhs, nm(iis.get_outcome()), iis.num_variable_members(), iis.num_constraint_members(), fired, nm(iis.get_status(x)), nm(iis.get_status(y)), nm(iis.get_status(r)));
    }
}
