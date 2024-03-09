#ifndef MIPPP_SOLVER_TRAITS_HPP
#define MIPPP_SOLVER_TRAITS_HPP

#ifdef MIPPP_FOUND_SHARED_GUROBI
#include "mippp/solver_traits/shared_grb_traits.hpp"
#define MIPPP_GUROBI 1
#endif
#ifdef MIPPP_FOUND_SHARED_CPLEX
#include "mippp/solver_traits/shared_cplex_traits.hpp"
#define MIPPP_CPLEX 2
#endif
#ifdef MIPPP_FOUND_SHARED_SCIP
#include "mippp/solver_traits/shared_scip_traits.hpp"
#define MIPPP_SCIP 3
#endif
#ifdef MIPPP_FOUND_SHARED_COINOR
#include "mippp/solver_traits/shared_cbc_traits.hpp"
#define MIPPP_COINOR 4
#endif

#include "mippp/solver_traits/cli_cbc_traits.hpp"
#include "mippp/solver_traits/cli_grb_traits.hpp"
#include "mippp/solver_traits/cli_scip_traits.hpp"

#include "mippp/mip_model.hpp"

#if !defined(MIPPP_DEFAULT_SOLVER_TRAITS) && \
    (!defined(MIPPP_PREFERED_SOLVER) || MIPPP_PREFERED_SOLVER == MIPPP_GUROBI)
#ifdef MIPPP_FOUND_SHARED_GUROBI
#define MIPPP_DEFAULT_SOLVER_TRAITS shared_grb_traits;
#else
#define MIPPP_DEFAULT_SOLVER_TRAITS cli_grb_traits;
#endif
#endif

/*#if !defined(MIPPP_DEFAULT_SOLVER_TRAITS) && \
    (!defined(MIPPP_PREFERED_SOLVER) || MIPPP_PREFERED_SOLVER == MIPPP_CPLEX)
#ifdef MIPPP_FOUND_SHARED_CPLEX
#define MIPPP_DEFAULT_SOLVER_TRAITS shared_cplex_traits;
#else
#define MIPPP_DEFAULT_SOLVER_TRAITS cli_cplex_traits;
#endif
#endif*/

#if !defined(MIPPP_DEFAULT_SOLVER_TRAITS) && \
    (!defined(MIPPP_PREFERED_SOLVER) || MIPPP_PREFERED_SOLVER == MIPPP_SCIP)
#ifdef MIPPP_FOUND_SHARED_SCIP
#define MIPPP_DEFAULT_SOLVER_TRAITS shared_scip_traits;
#else
#define MIPPP_DEFAULT_SOLVER_TRAITS cli_scip_traits;
#endif
#endif

#if !defined(MIPPP_DEFAULT_SOLVER_TRAITS) && \
    (!defined(MIPPP_PREFERED_SOLVER) || MIPPP_PREFERED_SOLVER == MIPPP_CBC)
#ifdef MIPPP_FOUND_SHARED_CBC
#define MIPPP_DEFAULT_SOLVER_TRAITS shared_cbc_traits;
#else
#define MIPPP_DEFAULT_SOLVER_TRAITS cli_cbc_traits;
#endif
#endif

namespace fhamonic {
namespace mippp {
using default_solver_traits = MIPPP_DEFAULT_SOLVER_TRAITS;
}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_SOLVER_TRAITS_HPP