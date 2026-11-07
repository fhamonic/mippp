#include "verify_copt_common.hpp"
using namespace std::chrono;
int main(int argc, char ** argv) {
    for(unsigned seed : {1u, 2u, 3u}) {
        const std::size_t N = std::atoi(argv[1]), EX = std::atoi(argv[2]), K = std::atoi(argv[3]);
        V b; b.iparam("Threads", 1); build_mcnd(b, N, EX, K, seed); b.set_time_limit(duration<double>(20.0));
        const double tb = cp::timed_solve(b);
        V m; m.iparam("Threads", 1); auto v = build_mcnd(m, N, EX, K, seed); m.set_time_limit(duration<double>(20.0));
        m.attach([&](const std::vector<double> & x, const cp::add_fn & add) { for(const auto & c : v.g.links(x)) add(c); });
        const double t = cp::timed_solve(m);
        std::println(stderr, "N={} EX={} K={} seed={} cols={} base {} | links {} | {}", N, EX, K, seed, m.num_variables(), cp::summary(b, tb), cp::summary(m, t), m.last->line());
    }
}
