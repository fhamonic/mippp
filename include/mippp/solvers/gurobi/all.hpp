#ifndef MIPPP_GUROBI_ALL_HPP
#define MIPPP_GUROBI_ALL_HPP

#include "mippp/solvers/gurobi/12/gurobi12_api.hpp"
#include "mippp/solvers/gurobi/12/gurobi12_lp.hpp"
#include "mippp/solvers/gurobi/12/gurobi12_milp.hpp"

namespace fhamonic {
namespace mippp {

using gurobi_api = gurobi12_api;
using gurobi_lp = gurobi12_lp;
using gurobi_milp = gurobi12_milp;

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_GUROBI_ALL_HPP
