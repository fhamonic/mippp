#ifndef MIPPP_SOLVER_TRAITS_HPP
#define MIPPP_SOLVER_TRAITS_HPP

#include "mippp/solver_traits/cli_cbc_traits.hpp"
#include "mippp/solver_traits/cli_grb_traits.hpp"
#include "mippp/solver_traits/cli_scip_traits.hpp"

#ifdef MIPPP_FOUND_SHARED_COINOR
#include "mippp/solver_traits/shared_cbc_traits.hpp"
#ifndef MIPPP_DEFAULT_SOLVER_TRAITS
#define MIPPP_DEFAULT_SOLVER_TRAITS shared_cbc_traits
#endif
#endif
#ifdef MIPPP_FOUND_SHARED_CPLEX
#include "mippp/solver_traits/shared_cplex_traits.hpp"
#ifndef MIPPP_DEFAULT_SOLVER_TRAITS
#define MIPPP_DEFAULT_SOLVER_TRAITS shared_cplex_traits
#endif
#endif
#ifdef MIPPP_FOUND_SHARED_GUROBI
#include "mippp/solver_traits/shared_grb_traits.hpp"
#ifndef MIPPP_DEFAULT_SOLVER_TRAITS
#define MIPPP_DEFAULT_SOLVER_TRAITS shared_grb_traits
#endif
#endif
#ifdef MIPPP_FOUND_SHARED_SCIP
#include "mippp/solver_traits/shared_scip_traits.hpp"
#ifndef MIPPP_DEFAULT_SOLVER_TRAITS
#define MIPPP_DEFAULT_SOLVER_TRAITS shared_scip_traits
#endif
#endif

#ifndef MIPPP_DEFAULT_SOLVER_TRAITS
#define MIPPP_DEFAULT_SOLVER_TRAITS cli_cbc_traits
#endif

namespace fhamonic {
namespace mippp {
using default_solver_traits = MIPPP_DEFAULT_SOLVER_TRAITS;
}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_SOLVER_TRAITS_HPP