#pragma once

#include "mippp/solvers/gurobi/impl/v1/gurobi_api.hpp"
#include "mippp/solvers/gurobi/impl/v1/gurobi_lp.hpp"
#include "mippp/solvers/gurobi/impl/v1/gurobi_milp.hpp"

namespace mippp {

using gurobi_api = gurobi::impl::v1::gurobi_api;
using gurobi_lp = gurobi::impl::v1::gurobi_lp;
using gurobi_milp = gurobi::impl::v1::gurobi_milp;

}  // namespace mippp
