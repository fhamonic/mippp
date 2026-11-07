// O4: the SHIPPED copt_milp, public API only (no -fno-access-control), add_lazy_constraint in MIPSOL.
// Model: network design with the capacity link rows HIDDEN, so each lazy row is really needed:
// a lost or mis-read lazy gives a wrong value. Zeros reach AddCallbackLazyConstr by x - x (coalesced +0.0)
// or 0.0 * x. usage: verify_copt_lazyzero <case> N extra K seed [limit]
#include "mippp/solvers/copt/all.hpp"
#include "verify_copt_common.hpp"
using namespace std::chrono;
int main(int argc, char ** argv) {
    const std::string kase = argv[1];
    const std::size_t N = std::atoi(argv[2]), EX = std::atoi(argv[3]), K = std::atoi(argv[4]);
    const unsigned seed = std::atoi(argv[5]);
    const double lim = argc > 6 ? std::atof(argv[6]) : 20.0;
    using MM = c::copt_milp;
    MM m; m.set_thread_limit(1); m.set_time_limit(duration<double>(lim));
    if(std::getenv("CUT_LOG")) m.native_api().SetIntParam(m.native_model().second, "Logging", 1);
    const bool compact = kase == "base";
    auto v = build_mcnd(m, N, EX, K, seed, !compact);
    auto & g = v.g;
    auto sl = m.add_variables(g.A);   // slack columns in no row, for the equality forms
    const auto Ks = std::views::iota(std::size_t{0}, K);
    long calls = 0, lazies = 0, zeros = 0, exc = 0;
    if(!compact) m.set_candidate_solution_callback([&](auto & h) {
        ++calls;
        auto sol = h.get_solution();
        std::vector<double> p(m.num_variables());
        for(std::size_t e = 0; e < g.A; ++e) { p[(std::size_t)g.y[e]] = sol[v.yv(e)]; for(std::size_t k : Ks) p[(std::size_t)g.x[k][e]] = sol[v.xv(v.idx(k, e))]; }
        if(kase.starts_with("eq")) {
            for(std::size_t e = 0; e < g.A; ++e) {
                double s = sol[sl(e)]; for(std::size_t k : Ks) s += p[(std::size_t)g.x[k][e]];
                if(std::fabs(s - g.cap[e] * p[(std::size_t)g.y[e]]) <= 1e-6) continue;
                const auto row = xsum(Ks, [&, e](std::size_t k) { return 1.0 * v.xv(v.idx(k, e)); });
                const auto cs = v.xv(v.idx((e * 3 + 1) % K, (e + 5) % g.A));
                ++lazies;
                if(kase == "eq_plain") h.add_lazy_constraint(row + 1.0 * sl(e) - g.cap[e] * v.yv(e) == 0.0);
                if(kase == "eq_coal") { ++zeros; h.add_lazy_constraint(row + 1.0 * sl(e) - g.cap[e] * v.yv(e) + 1.0 * cs - 1.0 * cs == 0.0); }
                if(kase == "eq_zero_neg") { ++zeros; h.add_lazy_constraint(distinct_variables, row + 1.0 * sl(e) - g.cap[e] * v.yv(e) + (-0.0) * cs == 0.0); }
            }
            return;
        }
        for(std::size_t e : g.violated_caps(p)) {
            const auto row = xsum(Ks, [&, e](std::size_t k) { return 1.0 * v.xv(v.idx(k, e)); });
            const double cap = g.cap[e];
            const auto cs = v.xv(v.idx((e * 3 + 1) % K, (e + 5) % g.A));   // a flow column on another arc
            const auto bs = v.yv((e + 5) % g.A);                           // a binary on another arc
            ++lazies;
            try {
                if(kase == "plain") h.add_lazy_constraint(row - cap * v.yv(e) <= 0.0);
                else if(kase == "coal_cont_last") { ++zeros; h.add_lazy_constraint(row - cap * v.yv(e) + 1.0 * cs - 1.0 * cs <= 0.0); }
                else if(kase == "coal_cont_first") { ++zeros; h.add_lazy_constraint(1.0 * cs - 1.0 * cs + row - cap * v.yv(e) <= 0.0); }
                else if(kase == "coal_bin_last") { ++zeros; h.add_lazy_constraint(row - cap * v.yv(e) + 1.0 * bs - 1.0 * bs <= 0.0); }
                else if(kase == "coal_bin_first") { ++zeros; h.add_lazy_constraint(1.0 * bs - 1.0 * bs + row - cap * v.yv(e) <= 0.0); }
                else if(kase == "zero_times_cont") { ++zeros; h.add_lazy_constraint(row - cap * v.yv(e) + 0.0 * cs <= 0.0); }
                else if(kase == "distinct_zero_cont") { ++zeros; h.add_lazy_constraint(distinct_variables, row - cap * v.yv(e) + 0.0 * cs <= 0.0); }
                else if(kase == "distinct_zero_bin") { ++zeros; h.add_lazy_constraint(distinct_variables, row - cap * v.yv(e) + 0.0 * bs <= 0.0); }
                else if(kase == "ge_coal_cont") { ++zeros; h.add_lazy_constraint(cap * v.yv(e) - row + 1.0 * cs - 1.0 * cs >= 0.0); }
                else if(kase == "ge_coal_bin") { ++zeros; h.add_lazy_constraint(cap * v.yv(e) - row + 1.0 * bs - 1.0 * bs >= 0.0); }
                else if(kase == "many_zeros") {
                    zeros += 3;
                    const auto c2 = v.xv(v.idx((e * 5 + 2) % K, (e + 9) % g.A));
                    const auto b2 = v.yv((e + 9) % g.A);
                    h.add_lazy_constraint(1.0 * c2 - 1.0 * c2 + row - cap * v.yv(e) + 1.0 * cs - 1.0 * cs + 1.0 * b2 - 1.0 * b2 <= 0.0);
                }
                else if(kase == "self_cancel") {
                    // a column of the row itself cancelled: 2 x - x - x + the rest
                    ++zeros;
                    const auto own = v.xv(v.idx(0, e));
                    h.add_lazy_constraint(row - 1.0 * own - cap * v.yv(e) <= 0.0 * own + 0.0);   // x_0e drops out: invalid weakening is fine? keeps validity only if x>=0
                }
                else if(kase == "plus_empty_le") { ++zeros; h.add_lazy_constraint(row - cap * v.yv(e) <= 0.0); h.add_lazy_constraint(1.0 * cs - 1.0 * cs <= 1.0); }
                else if(kase == "plus_empty_ge") { ++zeros; h.add_lazy_constraint(row - cap * v.yv(e) <= 0.0); h.add_lazy_constraint(1.0 * cs - 1.0 * cs >= -1.0); }
                else if(kase == "only_empty") { ++zeros; h.add_lazy_constraint(1.0 * cs - 1.0 * cs <= 1.0); }
                else { std::println(stderr, "unknown case"); std::exit(9); }
            } catch(const std::exception & e) { ++exc; std::println(stderr, "add threw: {}", e.what()); }
        }
    });
    const double t = cp::timed_solve(m);
    std::println(stderr, "{:20} seed={} {} calls={} lazies={} zeros={} exc={}", kase, seed, cp::summary(m, t), calls, lazies, zeros, exc);
}
