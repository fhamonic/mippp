#include "mippp/solvers/copt/all.hpp"
#include "probe_common.hpp"
int main() { run_lp_slices<mippp::copt_lp>(3000, 4500, 0.5, 30); }
