#ifndef MIPPP_SOLVER_TRAITS_HPP
#define MIPPP_SOLVER_TRAITS_HPP

#ifdef MIPPP_GUROBI_FOUND
#include "grb_traits.hpp"
#define MIPPP_GUROBI 1
#endif
#ifdef MIPPP_CPLEX_FOUND
#include "cplex_traits.hpp"
#define MIPPP_CPLEX 2
#endif
#ifdef MIPPP_SCIP_FOUND
#include "scip_traits.hpp"
#define MIPPP_SCIP 3
#endif
#ifdef MIPPP_COINOR_FOUND
#include "cbc_traits.hpp"
#define MIPPP_COINOR 4
#endif

namespace fhamonic {
namespace mippp {
#if defined(MIPPP_GUROBI_FOUND) && \
    (!defined(MIPPP_PREFERED_SOLVER) || MIPPP_PREFERED_SOLVER == MIPPP_GUROBI)
using default_solver_traits = grb_traits;
#elif defined(MIPPP_CPLEX_FOUND) && \
    (!defined(MIPPP_PREFERED_SOLVER) || MIPPP_PREFERED_SOLVER == MIPPP_CPLEX)
using default_solver_traits = cplex_traits;
#elif defined(MIPPP_SCIP_FOUND) && \
    (!defined(MIPPP_PREFERED_SOLVER) || MIPPP_PREFERED_SOLVER == MIPPP_SCIP)
using default_solver_traits = scip_traits;
#elif defined(MIPPP_COINOR_FOUND) && \
    (!defined(MIPPP_PREFERED_SOLVER) || MIPPP_PREFERED_SOLVER == MIPPP_COINOR)
using default_solver_traits = cbc_traits;
#elif defined(MIPPP_PREFERED_SOLVER)
static_assert(false, "The prefered MIP solver has not been found.");
#else
static_assert(false, "No thirdparty MIP solver has been found.");
#endif
}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_SOLVER_TRAITS_HPP