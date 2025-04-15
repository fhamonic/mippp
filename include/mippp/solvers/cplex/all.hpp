#ifndef MIPPP_CPLEX_ALL_HPP
#define MIPPP_CPLEX_ALL_HPP

#include "mippp/solvers/cplex/22/cplex22_api.hpp"
#include "mippp/solvers/cplex/22/cplex22_lp.hpp"
// #include "mippp/solvers/cplex/22/cplex22_milp.hpp"

namespace fhamonic {
namespace mippp {

using cplex_api = cplex22_api;
using cplex_lp = cplex22_lp;
// using cplex_milp = cplex22_milp;

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_CPLEX_ALL_HPP
