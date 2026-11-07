// Which sign of zero and which sense hurts, on user cuts. A valid cut x - d y <= 0 is written
// L: x - d y + z s <= 0, G: d y - x + z s >= 0, with z = +0.0 or -0.0. Also how the node dies:
// heuristics off gives no incumbent, so a false prune at the root must read infeasible.
// usage: verify_copt_zeromech N extra K seed
#include "verify_copt_common.hpp"
using namespace std::chrono;
struct Kase { std::string name; char sense; double z; bool coef_tiny; };
int main(int argc, char ** argv) {
    const std::size_t N = std::atoi(argv[1]), EX = std::atoi(argv[2]), K = std::atoi(argv[3]);
    const unsigned seed = std::atoi(argv[4]);
    const std::vector<Kase> kases{
        {"L plain", 'L', NAN, false}, {"G plain", 'G', NAN, false},
        {"L +0.0", 'L', 0.0, false}, {"L -0.0", 'L', -0.0, false},
        {"G +0.0", 'G', 0.0, false}, {"G -0.0", 'G', -0.0, false},
        {"L +1e-300", 'L', 1e-300, false}, {"L -1e-300", 'L', -1e-300, false},
        {"G +1e-300", 'G', 1e-300, false}, {"G -1e-300", 'G', -1e-300, false},
        {"L denorm", 'L', 5e-324, false}, {"L -denorm", 'L', -5e-324, false},
    };
    for(int heur = 1; heur >= 0; --heur) {
        double base = NAN;
        {
            V m; m.iparam("Threads", 1); build_mcnd(m, N, EX, K, seed); m.set_time_limit(duration<double>(30.0));
            if(!heur) { for(const char * n : {"HeurLevel", "PreRootHeurLevel", "RoundingHeurLevel", "DivingHeurLevel", "SubMipHeurLevel", "FAPHeurLevel"}) m.iparam(n, 0); }
            const double t = cp::timed_solve(m);
            std::println(stderr, "baseline heur={} seed={}: {}", heur, seed, cp::summary(m, t));
            base = m.get_solution_value();
        }
        for(const auto & k : kases) {
            V m; m.iparam("Threads", 1); auto v = build_mcnd(m, N, EX, K, seed); auto & f = v.g; m.set_time_limit(duration<double>(30.0));
            if(!heur) { for(const char * n : {"HeurLevel", "PreRootHeurLevel", "RoundingHeurLevel", "DivingHeurLevel", "SubMipHeurLevel", "FAPHeurLevel"}) m.iparam(n, 0); }
            long adds = 0;
            const auto cont = f.all_cont();
            recipe_opts o; o.strip_zero = false; o.fold = false; o.drop_satisfied = false;
            m.attach([&](const std::vector<double> & p, const cp::add_fn & add) {
                for(auto c : f.links(p)) {
                    if(k.sense == 'G') { for(auto & w : c.val) w = -w; c.sense = 'G'; c.rhs = -c.rhs + 0.0; }
                    if(!std::isnan(k.z)) { int s = cont[(std::size_t)(adds * 7 + 3) % cont.size()]; if(s == c.ind[0]) s = cont[(std::size_t)(adds * 7 + 4) % cont.size()]; c.ind.push_back(s); c.val.push_back(k.z); }
                    add(c); ++adds;
                }
            }, o);
            const double t = cp::timed_solve(m);
            double val = cf::value_of(m);
            const bool ok = cf::same(val, base) && is_a<status::optimal>(m.get_status());
            std::println(stderr, "heur={} {:12}: {} {} | relax={} adds={} err={}", heur, k.name, ok ? "OK   " : "WRONG", cp::summary(m, t), m.last->relax.load(), m.last->native_adds.load(), m.last->native_err.load());
        }
    }
}
