// Mutex control: a separator with ordinary shared state, the pattern a user writes when told nothing about threads.
#include "verify_cplex_common.hpp"
using namespace cp;
using std::chrono::duration;
int main(int, char ** argv) {
    { M m; print_version(m); }
    const bool mx = std::atoi(argv[1]);
    const int th = std::atoi(argv[2]);
    M m; build_mkp(m, 30, 100, 7); set_threads(m, th); vparam(m, CPXPARAM_Parallel, -1); m.set_time_limit(duration<double>(8));
    long plain_counter = 0;
    std::set<std::vector<int>> seen;
    std::vector<std::vector<int>> log;
    std::atomic<long> calls{0};
    recipe R; R.use_mutex = mx;
    auto & h = vattach(m, [&](const std::vector<double> & x, const add_fn &) {
        ++calls; ++plain_counter;
        std::vector<int> key;
        for(std::size_t j = 0; j < x.size() && key.size() < 12; ++j) if(x[j] > 1e-6 && x[j] < 1 - 1e-6) key.push_back(static_cast<int>(j));
        if(seen.insert(key).second) log.push_back(key);
    }, R);
    const double t = timed_solve(m);
    std::println(stderr, "RACY mutex={} threads={}: {} | calls={} plain_counter={} lost_updates={} seen={} log={} | {}", mx, th, summary(m, t), calls.load(), plain_counter, calls.load() - plain_counter, seen.size(), log.size(), h.line());
}
