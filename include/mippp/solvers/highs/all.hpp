#pragma once

#include "mippp/solvers/highs/v1_10/highs_api.hpp"
#include "mippp/solvers/highs/v1_10/highs_lp.hpp"
#include "mippp/solvers/highs/v1_10/highs_milp.hpp"
#include "mippp/solvers/highs/v1_10/highs_qp.hpp"

namespace mippp {

using highs_api = highs::v1_10::highs_api;
using highs_lp = highs::v1_10::highs_lp;
using highs_milp = highs::v1_10::highs_milp;
using highs_qp = highs::v1_10::highs_qp;

}  // namespace mippp
