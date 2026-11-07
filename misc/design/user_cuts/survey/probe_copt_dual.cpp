// Q3: a symmetry-breaking cut removes integer points but keeps an optimum, so it is "dominance
// valid". MIPRELAX leaves dual reductions on. Can presolve keep only the optimum the cut removes?
// A knapsack with every item twice; cut x_j <= x'_j or x'_j <= x_j on the violated pairs.
// Usage: probe_copt_dual <none|first|second> <LazyConstraints -1|0|1>
#include "probe_copt_common.cpp"
using namespace std::chrono;
int main(int argc, char ** argv) {
    const std::string dir = argv[1];
    M m; m.set_thread_limit(1); m.set_time_limit(duration<double>(25.0));
    m.native_api().SetIntParam(m.native_model().second, "LazyConstraints", std::atoi(argv[2]));
    const std::size_t rows = 4, items = 24;
    std::mt19937 rng(11);
    std::uniform_int_distribution<int> A(5, 100), V(10, 200);
    std::vector<std::vector<double>> a(rows, std::vector<double>(items));
    std::vector<double> v(items);
    for(auto & r : a) for(auto & e : r) e = A(rng);
    for(auto & e : v) e = V(rng);
    auto x = m.add_binary_variables(items);
    auto xp = m.add_binary_variables(items);
    const auto Is = std::views::iota(std::size_t{0}, items);
    m.set_maximization();
    m.set_objective(xsum(Is, [&](std::size_t j) { return v[j] * x(j) + v[j] * xp(j); }));
    for(const auto & r : a) {
        double s = 0; for(double e : r) s += e;
        m.add_constraint(xsum(Is, [&](std::size_t j) { return r[j] * x(j) + r[j] * xp(j); }) <= std::floor(s * 0.6));
    }
    long adds = 0, bad = 0, calls = 0;
    if(dir != "none") attach(m, [&](const std::vector<double> & p, const cp::add_fn & add) {
        ++calls;
        for(std::size_t j : Is) {
            const int c1 = m.native_id(x(j)), c2 = m.native_id(xp(j));
            cp::raw_cut c = dir == "first" ? cp::raw_cut{{c1, c2}, {1.0, -1.0}, 'L', 0.0} : cp::raw_cut{{c2, c1}, {1.0, -1.0}, 'L', 0.0};
            if(c.violation(p) > 1e-3) { if(add(c)) ++bad; else ++adds; }
        }
    }, "usercut");
    else m.set_verbose(false);
    if(std::getenv("CUT_LOG")) m.native_api().SetIntParam(m.native_model().second, "Logging", 1);
    const double t = cp::timed_solve(m);
    std::println(stderr, "dual dir={:6} LazyConstraints={:2}: {} calls={} adds={} errors={}", dir, argv[2], cp::summary(m, t), calls, adds, bad);
}
