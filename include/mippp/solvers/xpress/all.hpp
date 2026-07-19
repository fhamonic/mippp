#pragma once

#include "mippp/solvers/xpress/v45_1/xpress_api.hpp"
#include "mippp/solvers/xpress/v45_1/xpress_lp.hpp"
#include "mippp/solvers/xpress/v45_1/xpress_milp.hpp"

namespace mippp {

using xpress_api = xpress::v45_1::xpress_api;
using xpress_lp = xpress::v45_1::xpress_lp;
using xpress_milp = xpress::v45_1::xpress_milp;

}  // namespace mippp
