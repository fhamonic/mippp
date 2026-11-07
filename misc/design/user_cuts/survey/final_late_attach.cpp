// D4 on every backend with a candidate callback, through the public API only:
// a callback set after a finished solve, does the next solve() run it?
#include <cstdio>
#include <print>
#include "mippp/solvers/copt/all.hpp"
#include "mippp/solvers/cplex/all.hpp"
#include "mippp/solvers/gurobi/all.hpp"
#include "mippp/solvers/xpress/all.hpp"
using namespace mippp;
using namespace mippp::operators;

template <class M, class After>
void run(const char * backend, const char * name, After after) {
    try {
        M m;
        auto x = m.add_binary_variables(3);
        m.set_maximization();
        m.set_objective(1.0 * x(std::size_t{0}) + 2.0 * x(std::size_t{1}) +
                        3.0 * x(std::size_t{2}));
        m.add_constraint(x(std::size_t{0}) + x(std::size_t{1}) +
                             x(std::size_t{2}) <=
                         2);
        m.set_time_limit(std::chrono::seconds(20));
        m.solve();
        const double first = m.get_solution_value();
        long calls = 0, refusals = 0;
        using H = candidate_solution_callback_handle_t<M>;
        m.set_candidate_solution_callback(
            [&, y1 = x(std::size_t{1}), y2 = x(std::size_t{2})](auto & h) {
                ++calls;
                auto sol = h.get_solution();
                if(sol[y1] + sol[y2] > 1.5) {
                    ++refusals;
                    if constexpr(has_lazy_constraints<H, M>)
                        h.add_lazy_constraint(y1 + y2 <= 1);
                    else
                        h.reject_solution();
                }
            });
        after(m, x);
        m.solve();
        const auto & st = m.get_status();
        const char * sname = is<status::optimal>(st) ? "optimal"
                             : is_a<status::infeasible_or_unbounded>(st)
                                 ? "infeasible_or_unbounded or below"
                                 : "other";
        if(!status::solution_available(st)) {
            std::println("{} {}: first={} status={} calls={} refusals={}",
                         backend, name, first, sname, calls, refusals);
            std::fflush(stdout);
            return;
        }
        auto sol = m.get_solution();
        std::println("{} {}: status={} first={} second={} calls={} refusals={} "
                     "point=({},{},{}) {}",
                     backend, name, sname, first, m.get_solution_value(), calls,
                     refusals, sol[x(std::size_t{0})], sol[x(std::size_t{1})],
                     sol[x(std::size_t{2})],
                     sol[x(std::size_t{1})] + sol[x(std::size_t{2})] > 1.5
                         ? "VIOLATES THE CALLBACK'S RULE"
                         : "ok");
    } catch(const std::exception & e) {
        std::println("{} {}: exception {}", backend, name, e.what());
    }
    std::fflush(stdout);
}
template <class M>
void all(const char * backend) {
    run<M>(backend, "set after a finished solve", [](auto &, auto &) {});
    run<M>(backend, "set after, then a time limit write",
           [](auto & m, auto &) { m.set_time_limit(std::chrono::seconds(19)); });
    run<M>(backend, "control: set after, then an objective write",
           [](auto & m, auto & x) {
               m.set_objective_coefficient(x(std::size_t{0}), 1.0);
           });
}
int main(int argc, char ** argv) {
    const std::string which = argc > 1 ? argv[1] : "all";
    if(which == "all" || which == "gurobi") all<gurobi_milp>("gurobi");
    if(which == "all" || which == "cplex") all<cplex_milp>("cplex");
    if(which == "all" || which == "copt") all<copt_milp>("copt");
    if(which == "all" || which == "xpress") all<xpress_milp>("xpress");
}
