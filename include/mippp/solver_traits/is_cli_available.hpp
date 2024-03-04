#ifndef MIPPP_IS_CLI_AVAILABLE_HPP
#define MIPPP_IS_CLI_AVAILABLE_HPP

#include <cstdlib>

bool is_grb_available() { return std::system("gurobi_cl") == 0; }
bool is_scip_available() { return std::system("scip -c quit") == 0; }
bool is_cbc_available() { return std::system("cbc quit") == 0; }

#endif  // MIPPP_IS_CLI_AVAILABLE_HPP