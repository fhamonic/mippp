// SoPlex_readBasisFile on a path that cannot be opened, and a free row's code
// after an optimal solve.
#include <dlfcn.h>

#include <cstdio>
#include <cstring>

#include "mippp/solvers/soplex/all.hpp"

using namespace mippp;
using namespace mippp::operators;

int main(int argc, char ** argv) {
    const soplex_api & api = soplex_api::load();
    void * h = dlopen(api.library_path().c_str(), RTLD_NOW | RTLD_NOLOAD);
    auto readBasisFile =
        (int (*)(void *, const char *))dlsym(h, "SoPlex_readBasisFile");
    auto colStatus = (int (*)(void *, int))dlsym(h, "SoPlex_basisColStatus");
    soplex_lp m(api);
    m.set_time_limit(std::chrono::duration<double>(10.0));
    auto x = m.add_variable({.lower_bound = 0.0, .upper_bound = 10.0});
    auto y = m.add_variable({.lower_bound = 0.0, .upper_bound = 10.0});
    m.add_constraint(x + y <= 4.0);
    // a row free from the start: both sides infinite
    auto f = m.add_constraint(x - y <= 3.0);
    m.set_constraint_upper_bound(f, m.infinity());
    m.set_maximization();
    m.set_objective(x + 2 * y);
    m.solve();
    std::printf("free row from the start, after optimal solve: row codes %d %d,"
                " col codes %d %d, obj %g\n",
                api.basisRowStatus(m.native_model(), 0),
                api.basisRowStatus(m.native_model(), 1),
                colStatus(m.native_model(), 0), colStatus(m.native_model(), 1),
                m.get_solution_value());
    if(argc > 1 && std::strcmp(argv[1], "missing") == 0) {
        std::printf("calling SoPlex_readBasisFile on a missing path...\n");
        std::fflush(stdout);
        const int r = readBasisFile(m.native_model(), "/nonexistent/dir/x.bas");
        std::printf("returned %d (no abort)\n", r);
    }
    return 0;
}
