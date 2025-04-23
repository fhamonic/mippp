#ifndef MIPPP_GLPK_ALL_HPP
#define MIPPP_GLPK_ALL_HPP

#include "mippp/solvers/glpk/5/glpk5_api.hpp"
#include "mippp/solvers/glpk/5/glpk5_lp.hpp"
#include "mippp/solvers/glpk/5/glpk5_milp.hpp"

namespace fhamonic {
namespace mippp {

using glpk_api = glpk5_api;
using glpk_lp = glpk5_lp;
using glpk_milp = glpk5_milp;

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_GLPK_ALL_HPP
