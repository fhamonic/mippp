// Which candidate handles already return a point that can be written?
#include "mippp/solvers/copt/all.hpp"
#include "mippp/solvers/cplex/all.hpp"
#include "mippp/solvers/gurobi/all.hpp"
#include "mippp/solvers/xpress/all.hpp"
#include <print>
#include <type_traits>
using namespace mippp;
template <class M>
using elem_t = decltype(std::declval<candidate_solution_callback_handle_t<M> &>()
                            .get_solution()[std::declval<model_variable_t<M>>()]);
int main() {
    std::println("copt   writable: {}", std::is_lvalue_reference_v<elem_t<copt_milp>>);
    std::println("xpress writable: {}", std::is_lvalue_reference_v<elem_t<xpress_milp>>);
    std::println("gurobi writable: {}", std::is_lvalue_reference_v<elem_t<gurobi_milp>>);
    std::println("cplex  writable: {}", std::is_lvalue_reference_v<elem_t<cplex_milp>>);
}
