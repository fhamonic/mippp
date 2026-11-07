// Which atsp sizes leave a tree under DFJ and keep a no-hook baseline finished?
#include "verify_cplex_common.hpp"
using namespace cp;
using std::chrono::duration;
int main() {
    for(const int n : {20, 22, 23, 24, 25, 27, 28, 29, 30, 31}) for(const unsigned seed : {10u, 11u, 1u, 2u, 3u}) {
        M m; auto a = build_atsp(m, n, false, seed); m.set_time_limit(duration<double>(10));
        auto s = std::make_shared<stats>();
        recipe r; auto & h = vattach(m, watched(cf::dfj<M>(a), s), r);
        const double t = timed_solve(m);
        std::println(stderr, "n={} seed={} default threads: {} | calls={} cuts={} threads={} overlaps={} max={}", n, seed, summary(m, t), s->calls.load(), s->cuts.load(), s->threads.size(), s->overlaps.load(), s->max_inflight.load());
        (void)h;
    }
}
