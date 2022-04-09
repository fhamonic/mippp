#ifndef MIPPP_SOLVER_TRAITS_HPP
#define MIPPP_SOLVER_TRAITS_HPP

#ifdef FOUND_GUROBI
#include "grb_traits.hpp"
#endif
#ifdef FOUND_CPLEX
#include "cplex_traits.hpp"
#endif
#ifdef FOUND_SCIP
#include "scip_traits.hpp"
#endif
#ifdef FOUND_CBC
#include "cbc_traits.hpp"
#endif

#endif // MIPPP_SOLVER_TRAITS_HPP