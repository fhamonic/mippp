#ifndef MIPPP_SCIP_ALL_HPP
#define MIPPP_SCIP_ALL_HPP

#include "mippp/solvers/scip/v8/scip_api.hpp"
#include "mippp/solvers/scip/v8/scip_milp.hpp"

namespace fhamonic {
namespace mippp {

using scip_api = scip::v8::scip_api;
using scip_milp = scip::v8::scip_milp;

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_SCIP_ALL_HPP
