// O6, a valid one-term cut that presolve cannot derive: z <= 5 + 10 s, z <= 35 - 10 s, s = y1 + y2 + y3 over binaries.
// The LP has s = 1.5, z = 20; every integer s gives z <= 15, and probing one y leaves 20. So z <= 15 is valid and
// violated. usage: verify_copt_oneterm2 <none|one|two|one_all|two_all> after G seed [Presolve] [RootCutLevel]
#include "verify_copt_common.hpp"
using namespace std::chrono;
int main(int argc, char ** argv) {
    const std::string form = argv[1];
    const int after = std::atoi(argv[2]);
    const std::size_t G = std::atoi(argv[3]);
    const unsigned seed = std::atoi(argv[4]);
    V m; m.iparam("Threads", 1); m.set_time_limit(duration<double>(20.0));
    if(argc > 5 && std::atoi(argv[5]) >= 0) m.iparam("Presolve", std::atoi(argv[5]));
    if(argc > 6 && std::atoi(argv[6]) >= 0) m.iparam("RootCutLevel", std::atoi(argv[6]));
    std::mt19937 rng(seed); std::uniform_real_distribution<double> W(1.0, 2.0), E(0.0, 0.4);
    auto z = m.add_variables(G, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 100});
    auto y = m.add_binary_variables(3 * G);
    std::vector<double> w(G), e(3 * G);
    for(auto & v : w) v = W(rng); for(auto & v : e) v = E(rng);
    const auto Gs = std::views::iota(std::size_t{0}, G);
    m.set_maximization();
    m.set_objective(xsum(Gs, [&](std::size_t g) { return w[g] * z(g) - e[3 * g] * y(3 * g) - e[3 * g + 1] * y(3 * g + 1) - e[3 * g + 2] * y(3 * g + 2); }));
    for(std::size_t g : Gs) {
        m.add_constraint(z(g) - 10.0 * y(3 * g) - 10.0 * y(3 * g + 1) - 10.0 * y(3 * g + 2) <= 5.0);
        m.add_constraint(z(g) + 10.0 * y(3 * g) + 10.0 * y(3 * g + 1) + 10.0 * y(3 * g + 2) <= 35.0);
    }
    std::vector<int> zc; for(std::size_t g : Gs) zc.push_back(m.native_id(z(g)));
    std::atomic<long> calls{0}, adds{0}, rc_err{0}, later{0}, later_viol{0}, same_node_viol{0}, same_node{0}, root_calls{0}, viol_seen{0};
    std::atomic<int> add_node{-1}; std::atomic<int> last_rc{0};
    const int z0 = zc[0], z1 = zc[1];
    cp::raw_cut cut;
    if(form.starts_with("one")) cut = cp::raw_cut{{z0}, {1.0}, 'L', form == "one14" ? 14.0 : 15.0};
    if(form.starts_with("two")) cut = cp::raw_cut{{z0, z1}, {1.0, 1.0}, 'L', 30.0};
    const bool all = form.ends_with("_all");
    if(form != "base") {
        m.raw_mask = c::COPT_CBCONTEXT_MIPRELAX | c::COPT_CBCONTEXT_MIPNODE;
        m.raw = [&](c::copt_prob *, void * cb, int ctx) {
            if(ctx != c::COPT_CBCONTEXT_MIPRELAX) return 0;
            ++calls;
            int nc = 0; m.native_api().GetCallbackInfo(cb, "NodeCnt", &nc);
            if(nc == 0) ++root_calls;
            std::vector<double> p(m.num_variables());
            if(m.native_api().GetCallbackInfo(cb, c::COPT_CBINFO_RELAXSOLUTION, p.data())) return 0;
            if(p[(std::size_t)z0] > 15 + 1e-6) ++viol_seen;
            if(form == "none") return 0;
            const bool v = cut.violation(p) > 1e-6;
            if(adds > 0) { ++later; if(v) ++later_viol; if(nc == add_node) { ++same_node; if(v) ++same_node_viol; } }
            if(all) {
                std::vector<cp::raw_cut> cs;
                for(std::size_t g = 0; g + 1 < G; ++g) {
                    cp::raw_cut c1 = form == "one_all" ? cp::raw_cut{{zc[g]}, {1.0}, 'L', 15.0} : cp::raw_cut{{zc[g], zc[g + 1]}, {1.0, 1.0}, 'L', 30.0};
                    if(c1.violation(p) > 1e-6 && nc >= after) cs.push_back(c1);
                }
                for(auto & c1 : cs) { const int rc = m.native_api().AddCallbackUserCut(cb, (int)c1.ind.size(), c1.ind.data(), c1.val.data(), c1.sense, c1.rhs); ++adds; if(rc) { ++rc_err; last_rc = rc; } }
            } else if(adds == 0 && nc >= after && v) {
                const int rc = m.native_api().AddCallbackUserCut(cb, (int)cut.ind.size(), cut.ind.data(), cut.val.data(), cut.sense, cut.rhs);
                ++adds; add_node = nc; if(rc) { ++rc_err; last_rc = rc; }
            }
            return 0;
        };
    }
    const double t = cp::timed_solve(m);
    std::println(stderr, "odd2 {:7} after={:4} G={} seed={} pre={} rootcut={}: {} calls={} root_calls={} calls with z0>15:{} | adds={} node={} rc_err={}(last {}) | later calls={} violated={} | same node later={} violated={}", form, after, G, seed, argc > 5 ? argv[5] : "-", argc > 6 ? argv[6] : "-", cp::summary(m, t), calls.load(), root_calls.load(), viol_seen.load(), adds.load(), add_node.load(), rc_err.load(), last_rc.load(), later.load(), later_viol.load(), same_node.load(), same_node_viol.load());
}
