#ifndef MIPPP_GUROBI_ALL_HPP
#define MIPPP_GUROBI_ALL_HPP

#include "mippp/solvers/gurobi/12/grb12_api.hpp"
#include "mippp/solvers/gurobi/12/grb12_lp.hpp"
#include "mippp/solvers/gurobi/12/grb12_milp.hpp"

namespace fhamonic {
namespace mippp {

using grb_api = grb12_api;
using grb_lp = grb12_lp;
using grb_milp = grb12_milp;

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_GUROBI_ALL_HPP
