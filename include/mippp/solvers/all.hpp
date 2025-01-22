#ifndef MIPPP_SOLVER_TRAITS_HPP
#define MIPPP_SOLVER_TRAITS_HPP

#include "mippp/solvers/cli_cbc_solver.hpp"
#include "mippp/solvers/cli_grb_solver.hpp"
#include "mippp/solvers/cli_scip_solver.hpp"

#ifdef MIPPP_LINK_COINOR
#include "mippp/solvers/linked_cbc_solver.hpp"
#ifndef MIPPP_DEFAULT_SOLVER
#define MIPPP_DEFAULT_SOLVER linked_cbc_solver
#define MIPPP_DEFAULT_MODEL_TRAITS linked_cbc_model_traits
#endif
#endif
#ifdef MIPPP_LINK_CPLEX
#include "mippp/solvers/linked_cplex_solver.hpp"
#ifndef MIPPP_DEFAULT_SOLVER
#define MIPPP_DEFAULT_SOLVER linked_cplex_solver
#define MIPPP_DEFAULT_MODEL_TRAITS linked_cplex_model_traits
#endif
#endif
#ifdef MIPPP_LINK_GUROBI
#include "mippp/solvers/linked_grb_solver.hpp"
#ifndef MIPPP_DEFAULT_SOLVER
#define MIPPP_DEFAULT_SOLVER linked_grb_solver
#define MIPPP_DEFAULT_MODEL_TRAITS linked_grb_model_traits
#endif
#endif
#ifdef MIPPP_LINK_SCIP
#include "mippp/solvers/linked_scip_solver.hpp"
#ifndef MIPPP_DEFAULT_SOLVER
#define MIPPP_DEFAULT_SOLVER linked_scip_solver
#define MIPPP_DEFAULT_MODEL_TRAITS linked_scip_model_traits
#endif
#endif

#ifndef MIPPP_DEFAULT_SOLVER
#define MIPPP_DEFAULT_SOLVER cli_cbc_solver
#define MIPPP_DEFAULT_MODEL_TRAITS cli_solver_model_traits
#endif

namespace fhamonic {
namespace mippp {
using default_solver = MIPPP_DEFAULT_SOLVER;
using default_model_traits = MIPPP_DEFAULT_MODEL_TRAITS;
}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_SOLVER_TRAITS_HPP