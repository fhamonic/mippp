// A plain 5x30 binary knapsack, no hook at all, objective read from a file: does COPT prove a false optimum?
// Cross-checked against Gurobi on the same data, and the better point is checked against the rows by hand.
// usage: verify_copt_falseopt <file> seed
#include "verify_copt_common.hpp"
#include "mippp/solvers/gurobi/all.hpp"
using namespace std::chrono;
int main(int argc, char ** argv) {
    const std::string file = argv[1];
    const unsigned seed = std::atoi(argv[2]);
    std::vector<std::pair<int, double>> A;
    { FILE * f = std::fopen(file.c_str(), "r"); int j; double w; while(std::fscanf(f, "%d %lg", &j, &w) == 2) A.emplace_back(j, w); std::fclose(f); }
    // the rows, generated exactly as build_mkp does, to check a point by hand
    const int rows = 5, cols = 30;
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> AA(1, 1000), R(0, 500);
    std::vector<std::vector<double>> a(rows, std::vector<double>(cols));
    for(auto & row : a) for(auto & v : row) v = AA(rng);
    std::vector<double> cap(rows); for(int i = 0; i < rows; ++i) { double s = 0; for(double v : a[i]) s += v; cap[i] = std::floor(s / 2); }
    const auto evaluate = [&](const std::vector<double> & x) { double o = 0; for(auto [j, w] : A) o += w * x[(std::size_t)j]; double worst = -1e30; for(int i = 0; i < rows; ++i) { double s = 0; for(int j = 0; j < cols; ++j) s += a[i][j] * x[j]; worst = std::max(worst, s - cap[i]); } return std::pair{o, worst}; };
    std::vector<std::vector<double>> points;
    const auto run_copt = [&](const char * name, std::function<void(V &)> cfg, bool empty) {
        V m; m.iparam("Threads", 1); m.set_time_limit(duration<double>(30.0)); cp::build_mkp(m, rows, cols, seed);
        cfg(m);
        auto ff = *m.variables().begin(); std::vector<decltype(ff)> fv; for(auto v : m.variables()) fv.push_back(v);
        m.set_maximization();
        m.set_objective(xsum(A, [&](const std::pair<int, double> & e) { return e.second * fv[(std::size_t)e.first]; }));
        m.empty_slot = empty;
        const double t = cp::timed_solve(m);
        auto s = m.get_solution(); std::vector<double> x(m.num_variables()); for(auto v : m.variables()) x[(std::size_t)m.native_id(v)] = s[v];
        points.push_back(x);
        auto [o, w] = evaluate(x);
        std::println(stderr, "COPT {:34} {} simplex={} | recomputed obj={:.6f} worst row slack={:.3g}", name, cp::summary(m, t), m.simplex_iters(), o, w);
    };
    const auto gap0 = [](V & m) { m.set_optimality_tolerance(0.0); m.native_api().SetDblParam(m.native_model().second, "AbsGap", 0.0); };
    run_copt("defaults, shipped solve", [](V &) {}, false);
    run_copt("defaults, empty slot", [](V &) {}, true);
    run_copt("RelGap=0 AbsGap=0, shipped", gap0, false);
    run_copt("RelGap=0 AbsGap=0, empty slot", gap0, true);
    run_copt("gap0, Presolve=0", [&](V & m) { gap0(m); m.iparam("Presolve", 0); }, false);
    run_copt("gap0, CutLevel=0", [&](V & m) { gap0(m); m.iparam("CutLevel", 0); }, false);
    run_copt("gap0, HeurLevel=0", [&](V & m) { gap0(m); m.iparam("HeurLevel", 0); }, false);
    run_copt("gap0, Threads default", [&](V & m) { gap0(m); m.iparam("Threads", -1); }, false);
    run_copt("gap0, DualReductions? ConflictAnalysis=0", [&](V & m) { gap0(m); m.iparam("ConflictAnalysis", 0); }, false);
    // Gurobi as the second opinion
    {
        mippp::gurobi_milp g; g.set_thread_limit(1); g.set_time_limit(duration<double>(30.0)); cp::build_mkp(g, rows, cols, seed);
        g.set_optimality_tolerance(0.0);
        auto ff = *g.variables().begin(); std::vector<decltype(ff)> fv; for(auto v : g.variables()) fv.push_back(v);
        g.set_maximization();
        g.set_objective(xsum(A, [&](const std::pair<int, double> & e) { return e.second * fv[(std::size_t)e.first]; }));
        const double t = cp::timed_solve(g);
        auto s = g.get_solution(); std::vector<double> x(g.num_variables()); for(auto v : g.variables()) x[(std::size_t)g.native_id(v)] = s[v];
        auto [o, w] = evaluate(x);
        std::println(stderr, "GUROBI RelGap=0: {} | recomputed obj={:.6f} worst row slack={:.3g}", cp::summary(g, t), o, w);
        points.push_back(x);
    }
}
