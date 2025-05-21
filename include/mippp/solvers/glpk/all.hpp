#ifndef MIPPP_GLPK_ALL_HPP
#define MIPPP_GLPK_ALL_HPP

#include "mippp/solvers/glpk/v5/glpk_api.hpp"
#include "mippp/solvers/glpk/v5/glpk_lp.hpp"
#include "mippp/solvers/glpk/v5/glpk_milp.hpp"

namespace fhamonic {
namespace mippp {

using glpk_api = glpk::v5::glpk_api;
using glpk_lp = glpk::v5::glpk_lp;
using glpk_milp = glpk::v5::glpk_milp;

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_GLPK_ALL_HPP
