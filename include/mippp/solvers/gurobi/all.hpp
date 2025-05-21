#ifndef MIPPP_GUROBI_ALL_HPP
#define MIPPP_GUROBI_ALL_HPP

#include "mippp/solvers/gurobi/v12_0/gurobi_api.hpp"
#include "mippp/solvers/gurobi/v12_0/gurobi_lp.hpp"
#include "mippp/solvers/gurobi/v12_0/gurobi_milp.hpp"

namespace fhamonic {
namespace mippp {

using gurobi_api = gurobi::v12_0::gurobi_api;
using gurobi_lp = gurobi::v12_0::gurobi_lp;
using gurobi_milp = gurobi::v12_0::gurobi_milp;

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_GUROBI_ALL_HPP
