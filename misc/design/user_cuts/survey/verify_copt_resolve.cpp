// Why did the fuzz oracle return a max below a feasible point's value? The same random objectives on one model
// re-solved, and on a fresh model each time. usage: verify_copt_resolve seed rows cols count [gap0|gapdef]
#include "verify_copt_common.hpp"
using namespace std::chrono;
int main(int argc, char ** argv) {
    const unsigned seed = std::atoi(argv[1]);
    const int rows = std::atoi(argv[2]), cols = std::atoi(argv[3]), count = std::atoi(argv[4]);
    const bool gap0 = argc <= 5 || std::string(argv[5]) == "gap0";
    const auto cfg = [&](V & m) {
        m.iparam("Threads", 1); m.set_time_limit(duration<double>(10.0));
        if(gap0) { m.set_optimality_tolerance(0.0); m.native_api()._check(nullptr, m.native_api().SetDblParam(m.native_model().second, "AbsGap", 0.0)); }
    };
    V o; cfg(o); cp::build_mkp(o, rows, cols, seed);
    auto first = *o.variables().begin();
    std::vector<decltype(first)> vs; for(auto v : o.variables()) vs.push_back(v);
    std::vector<double> objc(vs.size());
    {
        using getcol_fn = int (*)(c::copt_prob *, const char *, int, const int *, double *);
        auto getcol = cp::symf<getcol_fn>(o.native_api().library_path(), "COPT_GetColInfo");
        std::vector<int> ids(vs.size()); for(std::size_t i = 0; i < vs.size(); ++i) ids[i] = o.native_id(vs[i]);
        getcol(o.native_model().second, "Obj", (int)ids.size(), ids.data(), objc.data());
    }
    std::mt19937 rng(seed * 131u + 7u);
    std::uniform_real_distribution<double> noise(0.8, 1.2), coin(0.0, 1.0);
    int mismatches = 0;
    for(int t = 0; t < count; ++t) {
        std::vector<std::pair<int, double>> A;
        const int mode = t % 3;
        for(std::size_t i = 0; i < vs.size(); ++i) {
            if(mode == 0) A.emplace_back((int)i, objc[i]);
            else if(mode == 1) A.emplace_back((int)i, std::round(objc[i] * noise(rng) * 100) / 100);
            else if(coin(rng) < 0.6) A.emplace_back((int)i, objc[i]);
        }
        o.set_maximization();
        o.set_objective(xsum(A, [&](const std::pair<int, double> & e) { return e.second * vs[(std::size_t)e.first]; }));
        const double t1 = cp::timed_solve(o);
        const double r_v = o.get_solution_value(), r_d = o.get_dual_bound(); const std::string r_s = cp::status_name(o.get_status()); const auto r_n = o.get_node_count();
        V f; cfg(f); cp::build_mkp(f, rows, cols, seed);
        auto ff = *f.variables().begin(); std::vector<decltype(ff)> fv; for(auto v : f.variables()) fv.push_back(v);
        f.set_maximization();
        f.set_objective(xsum(A, [&](const std::pair<int, double> & e) { return e.second * fv[(std::size_t)e.first]; }));
        const double t2 = cp::timed_solve(f);
        const double f_v = f.get_solution_value(), f_d = f.get_dual_bound(); const std::string f_s = cp::status_name(f.get_status());
        const bool mism = std::fabs(r_v - f_v) > 1e-6 * (1 + std::fabs(f_v));
        mismatches += mism;
        std::println(stderr, "t={} mode={} terms={} REUSED {} value={} dual={} nodes={} {:.2f}s | FRESH {} value={} dual={} nodes={} {:.2f}s {}", t, mode, A.size(), r_s, r_v, r_d, r_n, t1, f_s, f_v, f_d, f.get_node_count(), t2, mism ? "MISMATCH" : "");
    }
    std::println(stderr, "== seed={} {}x{} gap0={} count={} mismatches={}", seed, rows, cols, gap0, count, mismatches);
}
