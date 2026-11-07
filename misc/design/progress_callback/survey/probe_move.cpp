// Does a candidate callback survive a move of the model? Run with the stale object overwritten.
#include SOLVER_HEADER
#include "probe_common.hpp"
#include <new>
using M = SOLVER_MODEL;
int main() {
    alignas(M) static unsigned char storage[sizeof(M)];
    auto * first = new(storage) M();
    build_mkp(*first, make_mkp(5, 40), +1, 100.0);
    long calls = 0;
    first->set_candidate_solution_callback([&](auto & h) { ++calls; (void)h.get_solution_value(); });
    M moved(std::move(*first));
    first->~M();
    std::memset(storage, 0xAB, sizeof(M));  // what a freed or reused object may hold
    moved.solve();
    std::println(stderr, "calls={} status={}", calls, status_name(moved.get_status()));
}
