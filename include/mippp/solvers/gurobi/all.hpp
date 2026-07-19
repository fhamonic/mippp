#pragma once

#include "mippp/solvers/gurobi/v12_0/gurobi_api.hpp"
#include "mippp/solvers/gurobi/v12_0/gurobi_lp.hpp"
#include "mippp/solvers/gurobi/v12_0/gurobi_milp.hpp"

namespace mippp {

using gurobi_api = gurobi::v12_0::gurobi_api;
using gurobi_lp = gurobi::v12_0::gurobi_lp;
using gurobi_milp = gurobi::v12_0::gurobi_milp;

}  // namespace mippp
