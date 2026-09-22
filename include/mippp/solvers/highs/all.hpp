#pragma once

#include "mippp/solvers/highs/impl/v1/highs_api.hpp"
#include "mippp/solvers/highs/impl/v1/highs_lp.hpp"
#include "mippp/solvers/highs/impl/v1/highs_milp.hpp"
#include "mippp/solvers/highs/impl/v1/highs_qp.hpp"

namespace mippp {

using highs_api = highs::impl::v1::highs_api;
using highs_lp = highs::impl::v1::highs_lp;
using highs_milp = highs::impl::v1::highs_milp;
using highs_qp = highs::impl::v1::highs_qp;

}  // namespace mippp
