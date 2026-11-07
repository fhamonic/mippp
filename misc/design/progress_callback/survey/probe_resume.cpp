// With a progress callback set, does a re-solve still resume the tree? Attaching per solve must not drop it.
#include SOLVER_HEADER
#include "probe_common.hpp"
using M = SOLVER_MODEL;
int main() {
    M m; build_mkp(m, make_mkp(30, 100), +1, 100.0);
    long calls = 0;
    m.set_progress_callback([&](auto &) { ++calls; });
    for(int i = 1; i <= 5; ++i) {
        m.set_time_limit(std::chrono::duration<double>(GROW ? i : 1));
        auto o = timed_solve(m);
        std::println(stderr, "slice {}: {:.2f}s dual={:.7g} nodes={} calls={}", i, o.solve_s, o.dual, m.get_node_count(), calls);
    }
}
