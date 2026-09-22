#pragma once

#include "mippp/solvers/glpk/impl/v1/glpk_api.hpp"
#include "mippp/solvers/glpk/impl/v1/glpk_lp.hpp"
#include "mippp/solvers/glpk/impl/v1/glpk_milp.hpp"

namespace mippp {

using glpk_api = glpk::impl::v1::glpk_api;
using glpk_lp = glpk::impl::v1::glpk_lp;
using glpk_milp = glpk::impl::v1::glpk_milp;

}  // namespace mippp
