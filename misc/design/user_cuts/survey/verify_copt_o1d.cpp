// O1 at true default settings: maximize c'x on a knapsack, integer c. At a tree node whose LP bound L has floor(L) below
// the true optimum, c'x <= floor(L) is valid in that subtree and removes the optimum elsewhere. Added once, rc read.
// Baseline run with RelGap=0 so the true optimum is known.
#include "verify_copt_common.hpp"
using namespace std::chrono;
int main() {
    for(unsigned seed = 1; seed <= 8; ++seed) for(long gate : {0L, 50L, 400L}) {
        const auto bld = [&](V & m, std::vector<int> & cols, std::vector<double> & obj) {
            m.iparam("Threads", 1); m.set_time_limit(duration<double>(20.0)); m.set_optimality_tolerance(0.0);
            const std::size_t C = 50, R = 5;
            std::mt19937 rng(seed);
            std::uniform_int_distribution<int> A(1, 100), Q(0, 50);
            std::vector<std::vector<double>> a(R, std::vector<double>(C));
            for(auto & r : a) for(auto & v : r) v = A(rng);
            obj.assign(C, 0);
            for(std::size_t j = 0; j < C; ++j) { double s = 0; for(auto & r : a) s += r[j]; obj[j] = std::floor(s / R + Q(rng)); }
            auto x = m.add_binary_variables(C);
            const auto ids = std::views::iota(std::size_t{0}, C);
            m.set_maximization();
            m.set_objective(xsum(ids, [&](std::size_t j) { return obj[j] * x(j); }));
            for(std::size_t i = 0; i < R; ++i) { double s = 0; for(double v : a[i]) s += v; m.add_constraint(xsum(ids, [&, i](std::size_t j) { return a[i][j] * x(j); }) <= std::floor(s / 2)); }
            for(std::size_t j = 0; j < C; ++j) cols.push_back(m.native_id(x(j)));
        };
        double base; std::string bs;
        { V b; std::vector<int> cols; std::vector<double> o; bld(b, cols, o); const double t = cp::timed_solve(b); base = b.get_solution_value(); bs = cp::summary(b, t); if(!is_a<status::optimal>(b.get_status())) continue; }
        V m; std::vector<int> cols; std::vector<double> obj; bld(m, cols, obj);
        std::atomic<long> calls{0}; long adds = 0; int rc = 0, node = -1; double lpv = 0, rhs = 0;
        m.raw_mask = c::COPT_CBCONTEXT_MIPRELAX | c::COPT_CBCONTEXT_MIPNODE;
        m.raw = [&](c::copt_prob *, void * cb, int ctx) {
            if(ctx != c::COPT_CBCONTEXT_MIPRELAX) return 0;
            const long k = ++calls;
            if(adds || k <= gate) return 0;
            int nc = 0; m.native_api().GetCallbackInfo(cb, "NodeCnt", &nc);
            if(nc < 1) return 0;
            std::vector<double> p(m.num_variables());
            if(m.native_api().GetCallbackInfo(cb, c::COPT_CBINFO_RELAXSOLUTION, p.data())) return 0;
            double L = 0; for(std::size_t j = 0; j < cols.size(); ++j) L += obj[j] * p[(std::size_t)cols[j]];
            if(std::floor(L + 1e-9) >= base - 1e-6 || L - std::floor(L) < 1e-3) return 0;
            rhs = std::floor(L); lpv = L; node = nc; ++adds;
            rc = m.native_api().AddCallbackUserCut(cb, (int)cols.size(), cols.data(), obj.data(), 'L', rhs);
            return 0;
        };
        const double t = cp::timed_solve(m);
        const double v = cf::value_of(m);
        std::println(stderr, "seed={} gate={}: {} base {} | adds={} at node={} L={:.2f} rhs={} rc={} => {}", seed, gate, cp::summary(m, t), base, adds, node, lpv, rhs, rc, adds == 0 ? "NO POWER" : std::fabs(v - base) < 1e-6 ? "KEPT" : "LOST");
    }
}
