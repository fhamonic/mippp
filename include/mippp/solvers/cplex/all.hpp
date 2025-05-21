#ifndef MIPPP_CPLEX_ALL_HPP
#define MIPPP_CPLEX_ALL_HPP

#include "mippp/solvers/cplex/v22_12/cplex_api.hpp"
#include "mippp/solvers/cplex/v22_12/cplex_lp.hpp"
#include "mippp/solvers/cplex/v22_12/cplex_milp.hpp"

namespace fhamonic {
namespace mippp {

using cplex_api = cplex::v22_12::cplex_api;
using cplex_lp = cplex::v22_12::cplex_lp;
using cplex_milp = cplex::v22_12::cplex_milp;

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_CPLEX_ALL_HPP
