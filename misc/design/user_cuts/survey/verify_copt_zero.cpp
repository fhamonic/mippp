// O3 on my own instance family: multicommodity fixed-charge network design. usage: verify_copt_zero N extra K seed [limit]
#include "verify_copt_common.hpp"
using namespace std::chrono;
struct Kase { std::string name; double zv; int where; int pos; };  // where 0 cont 1 bin; pos 0 last 1 first
int main(int argc, char ** argv) {
    const std::size_t N = std::atoi(argv[1]), EX = std::atoi(argv[2]), K = std::atoi(argv[3]);
    const unsigned seed = std::atoi(argv[4]);
    const double lim = argc > 5 ? std::atof(argv[5]) : 30.0;
    const std::vector<Kase> kases{
        {"plain", NAN, 0, 0},
        {"zero cont last", 0.0, 0, 0}, {"zero cont first", 0.0, 0, 1},
        {"zero bin last", 0.0, 1, 0}, {"zero bin first", 0.0, 1, 1},
        {"-0.0 cont last", -0.0, 0, 0},
        {"1e-300 cont", 1e-300, 0, 0}, {"1e-20 cont", 1e-20, 0, 0}, {"1e-17 cont", 1e-17, 0, 0}, {"1e-13 cont", 1e-13, 0, 0}, {"1e-9 cont", 1e-9, 0, 0},
    };
    double base = NAN;
    {
        V m; m.iparam("Threads", 1); auto f = build_mcnd(m, N, EX, K, seed); m.set_time_limit(duration<double>(lim));
        const double t = cp::timed_solve(m);
        std::println(stderr, "baseline no hook N={} extra={} K={} seed={}: {} cols={}", N, EX, K, seed, cp::summary(m, t), m.num_variables());
        base = m.get_solution_value();
    }
    for(const auto & k : kases) {
        V m; m.iparam("Threads", 1); auto v = build_mcnd(m, N, EX, K, seed); auto & f = v.g; m.set_time_limit(duration<double>(lim));
        long adds = 0;
        const auto cont = f.all_cont();
        recipe_opts o; o.strip_zero = false; o.fold = false; o.drop_satisfied = false;
        m.attach([&](const std::vector<double> & p, const cp::add_fn & add) {
            for(auto c : f.links(p)) {
                if(!std::isnan(k.zv)) {
                    const auto & pool = k.where == 0 ? cont : f.y;
                    int spare = pool[(std::size_t)(adds * 7 + 3) % pool.size()];
                    if(spare == c.ind[0] || spare == c.ind[1]) spare = pool[(std::size_t)(adds * 7 + 4) % pool.size()];
                    if(k.pos == 0) { c.ind.push_back(spare); c.val.push_back(k.zv); }
                    else { c.ind.insert(c.ind.begin(), spare); c.val.insert(c.val.begin(), k.zv); }
                }
                add(c); ++adds;
            }
        }, o);
        const double t = cp::timed_solve(m);
        double val = cf::value_of(m);
        const bool ok = cf::same(val, base) && is_a<status::optimal>(m.get_status());
        std::println(stderr, "{:18}: {} {} | {}", k.name, ok ? "OK   " : "WRONG", cp::summary(m, t), m.last ? m.last->line() : "");
    }
}
