#include "mippp/solvers/soplex/all.hpp"
#include "probe_common.hpp"
using L = mippp::soplex_lp;
int main() {
    { L m; std::println(stderr, "library {}", m.native_api().library_path().string()); }
    run_lp_slices<L>(3000, 4500, 0.5);
}
