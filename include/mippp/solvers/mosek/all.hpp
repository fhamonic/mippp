#ifndef MIPPP_MOSEK_ALL_HPP
#define MIPPP_MOSEK_ALL_HPP

#include "mippp/solvers/mosek/11/mosek11_api.hpp"
#include "mippp/solvers/mosek/11/mosek11_lp.hpp"
#include "mippp/solvers/mosek/11/mosek11_milp.hpp"

namespace fhamonic {
namespace mippp {

using mosek_api = mosek11_api;
using mosek_lp = mosek11_lp;
using mosek_milp = mosek11_milp;

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_MOSEK_ALL_HPP
