#pragma once

#include "mippp/solvers/mosek/v11/mosek_api.hpp"
#include "mippp/solvers/mosek/v11/mosek_lp.hpp"
#include "mippp/solvers/mosek/v11/mosek_milp.hpp"

namespace mippp {

using mosek_api = mosek::v11::mosek_api;
using mosek_lp = mosek::v11::mosek_lp;
using mosek_milp = mosek::v11::mosek_milp;

}  // namespace mippp
