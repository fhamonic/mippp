#ifndef MIPPP_COPT_ALL_HPP
#define MIPPP_COPT_ALL_HPP

#include "mippp/solvers/copt/v7_2/copt_api.hpp"
#include "mippp/solvers/copt/v7_2/copt_lp.hpp"
#include "mippp/solvers/copt/v7_2/copt_milp.hpp"

namespace fhamonic {
namespace mippp {

using copt_api = copt::v7_2::copt_api;
using copt_lp = copt::v7_2::copt_lp;
using copt_milp = copt::v7_2::copt_milp;

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_COPT_ALL_HPP
