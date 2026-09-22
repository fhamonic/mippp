#pragma once

#include "mippp/solvers/xpress/impl/v1/xpress_api.hpp"
#include "mippp/solvers/xpress/impl/v1/xpress_lp.hpp"
#include "mippp/solvers/xpress/impl/v1/xpress_milp.hpp"

namespace mippp {

using xpress_api = xpress::impl::v1::xpress_api;
using xpress_lp = xpress::impl::v1::xpress_lp;
using xpress_milp = xpress::impl::v1::xpress_milp;

}  // namespace mippp
