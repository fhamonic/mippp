#ifndef MIPPP_HIGHS_ALL_HPP
#define MIPPP_HIGHS_ALL_HPP

#include "mippp/solvers/highs/v1_10/highs_api.hpp"
#include "mippp/solvers/highs/v1_10/highs_lp.hpp"
#include "mippp/solvers/highs/v1_10/highs_milp.hpp"
#include "mippp/solvers/highs/v1_10/highs_qp.hpp"

namespace fhamonic {
namespace mippp {

using highs_api = highs::v1_10::highs_api;
using highs_lp = highs::v1_10::highs_lp;
using highs_milp = highs::v1_10::highs_milp;
using highs_qp = highs::v1_10::highs_qp;

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_HIGHS_ALL_HPP
