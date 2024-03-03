#ifndef MIPPP_SOLVER_TRAITS_HPP
#define MIPPP_SOLVER_TRAITS_HPP

#ifdef MIPPP_GUROBI_FOUND
#include "mippp/solver_traits/shared_grb_traits.hpp"
#define MIPPP_GUROBI 1
#endif
#ifdef MIPPP_CPLEX_FOUND
#include "mippp/solver_traits/shared_cplex_traits.hpp"
#define MIPPP_CPLEX 2
#endif
#ifdef MIPPP_SCIP_FOUND
#include "mippp/solver_traits/shared_scip_traits.hpp"
#define MIPPP_SCIP 3
#endif
#ifdef MIPPP_COINOR_FOUND
#include "mippp/solver_traits/shared_cbc_traits.hpp"
#define MIPPP_COINOR 4
#endif

#include "mippp/solver_traits/cli_cbc_traits.hpp"
#include "mippp/solver_traits/cli_grb_traits.hpp"
#include "mippp/solver_traits/cli_scip_traits.hpp"

namespace fhamonic {
namespace mippp {
#if defined(MIPPP_GUROBI_FOUND) && \
    (!defined(MIPPP_PREFERED_SOLVER) || MIPPP_PREFERED_SOLVER == MIPPP_GUROBI)
using default_solver_traits = shared_grb_traits;
#elif defined(MIPPP_CPLEX_FOUND) && \
    (!defined(MIPPP_PREFERED_SOLVER) || MIPPP_PREFERED_SOLVER == MIPPP_CPLEX)
using default_solver_traits = shared_cplex_traits;
#elif defined(MIPPP_SCIP_FOUND) && \
    (!defined(MIPPP_PREFERED_SOLVER) || MIPPP_PREFERED_SOLVER == MIPPP_SCIP)
using default_solver_traits = shared_scip_traits;
#elif defined(MIPPP_COINOR_FOUND) && \
    (!defined(MIPPP_PREFERED_SOLVER) || MIPPP_PREFERED_SOLVER == MIPPP_COINOR)
using default_solver_traits = shared_cbc_traits;
#elif defined(MIPPP_PREFERED_SOLVER)
static_assert(false, "The prefered MIP solver has not been found.");
#else
static_assert(false, "No thirdparty MIP solver has been found.");
#endif
}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_SOLVER_TRAITS_HPP