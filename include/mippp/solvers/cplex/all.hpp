#pragma once

#include "mippp/solvers/cplex/impl/v1/cplex_api.hpp"
#include "mippp/solvers/cplex/impl/v1/cplex_lp.hpp"
#include "mippp/solvers/cplex/impl/v1/cplex_milp.hpp"

namespace mippp {

using cplex_api = cplex::impl::v1::cplex_api;
using cplex_lp = cplex::impl::v1::cplex_lp;
using cplex_milp = cplex::impl::v1::cplex_milp;

}  // namespace mippp
