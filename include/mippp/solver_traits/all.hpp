#ifndef MIPPP_SOLVER_TRAITS_HPP
#define MIPPP_SOLVER_TRAITS_HPP

#ifdef GUROBI_FOUND
#include "grb_traits.hpp"
#endif
#ifdef CPLEX_FOUND
#include "cplex_traits.hpp"
#endif
#ifdef SCIP_FOUND
#include "scip_traits.hpp"
#endif
#ifdef COIN_FOUND
#include "cbc_traits.hpp"
#endif

namespace fhamonic {
namespace mippp {
#if defined(GUROBI_FOUND)
using default_solver_traits = grb_traits;
#elif defined(CPLEX_FOUND)
using default_solver_traits = cplex_traits;
#elif defined(SCIP_FOUND)
using default_solver_traits = scip_traits;
#elif defined(COIN_FOUND)
using default_solver_traits = cbc_traits;
#else
static_assert(false, "No MIP solver has been found.");
#endif
}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_SOLVER_TRAITS_HPP