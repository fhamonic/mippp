// How big must the ATSP be for cases to exercise the hook? baseline and DFJ at sizes, seeds.
#include "verify_copt_common.hpp"
using namespace std::chrono;
int main(int argc, char ** argv) {
    for(int a = 1; a < argc; ++a) for(unsigned seed : {1u, 2u, 3u}) {
        const int n = std::atoi(argv[a]);
        V b; b.iparam("Threads", 1); cp::build_atsp(b, n, false, seed); b.set_time_limit(duration<double>(30.0));
        const double tb = cp::timed_solve(b);
        V m; m.iparam("Threads", 1); auto at = cp::build_atsp(m, n, false, seed); m.set_time_limit(duration<double>(30.0));
        m.attach([&](const std::vector<double> & x, const cp::add_fn & add) { for(const auto & c : at.separate(x)) add(c); });
        const double t = cp::timed_solve(m);
        std::println(stderr, "n={} seed={} base {} | dfj {} | {}", n, seed, cp::summary(b, tb), cp::summary(m, t), m.last->line());
    }
}
