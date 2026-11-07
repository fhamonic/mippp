// A capacitated facility location with the aggregated linking row, at DEFAULT presolve: continuous
// columns survive and a tree exists, unlike the gadgets. The cut x_ij - d_i y_j <= 0 is valid and
// violated by the LP. Do an explicit zero or a duplicated column hurt here as they do on the gadgets?
// Usage: probe_copt_cfl <plain|zero|zero_y|dup|none> <min nodes done before adding> [RootCutLevel]
#include "probe_copt_common.cpp"
using namespace std::chrono;
int main(int argc, char ** argv) {
    const std::string name = argv[1];
    const long after = std::atol(argv[2]);
    M m; m.set_thread_limit(1); m.set_time_limit(duration<double>(20.0));
    if(argc > 3) m.native_api().SetIntParam(m.native_model().second, "RootCutLevel", std::atoi(argv[3]));
    const std::size_t F = 12, C = 40;
    std::mt19937 rng(3);
    std::uniform_real_distribution<double> P(0, 100), dem(5, 35), fix(100, 300);
    std::vector<std::pair<double, double>> fp(F), cp_(C);
    for(auto & p : fp) p = {P(rng), P(rng)};
    for(auto & p : cp_) p = {P(rng), P(rng)};
    std::vector<double> d(C), f(F), cap(F, 0);
    for(auto & v : d) v = std::round(dem(rng));
    for(auto & v : f) v = std::round(fix(rng));
    double total = 0; for(double v : d) total += v;
    for(auto & v : cap) v = std::round(total * 3.0 / static_cast<double>(F));
    auto y = m.add_binary_variables(F);
    auto x = m.add_variables(F * C);
    const auto id = [&](std::size_t i, std::size_t j) { return i * F + j; };
    const auto Cs = std::views::iota(std::size_t{0}, C);
    const auto Fs = std::views::iota(std::size_t{0}, F);
    m.set_minimization();
    m.set_objective(xsum(Fs, [&](std::size_t j) { return f[j] * y(j); }) + xsum(Cs, [&](std::size_t i) { return xsum(Fs, [&, i](std::size_t j) { return (0.2 * std::hypot(cp_[i].first - fp[j].first, cp_[i].second - fp[j].second)) * x(id(i, j)); }); }));
    for(std::size_t i : Cs) m.add_constraint(xsum(Fs, [&, i](std::size_t j) { return 1.0 * x(id(i, j)); }) == d[i]);
    for(std::size_t j : Fs) m.add_constraint(xsum(Cs, [&, j](std::size_t i) { return 1.0 * x(id(i, j)); }) - cap[j] * y(j) <= 0);
    long done_first = -1, adds = 0, bad = 0; int rc = 0;
    if(name != "none") attach(m, [&](const std::vector<double> & v, const cp::add_fn & add) {
        const long nd = m.hook->node_calls.load();
        if(nd < after) return;
        long here = 0;
        for(std::size_t i : Cs) for(std::size_t j : Fs) {
            const int xc = m.native_id(x(id(i, j))), yc = m.native_id(y(j));
            cp::raw_cut c{{xc, yc}, {1.0, -d[i]}, 'L', 0.0};
            if(c.violation(v) < 1e-2 || here >= 10) continue;
            ++here;
            const int spare = m.native_id(x(id((i + 1) % C, (j + 1) % F)));
            if(name == "zero") { c.ind.push_back(spare); c.val.push_back(0.0); }
            if(name == "zero_y") { c.ind.push_back(m.native_id(y((j + 1) % F))); c.val.push_back(0.0); }
            if(name == "dup") { c.val[0] = 0.5; c.ind.push_back(xc); c.val.push_back(0.5); }
            const int e = add(c);
            if(e) { ++bad; rc = e; } else ++adds;
            if(done_first < 0) done_first = nd;
        }
    }, "usercut");
    const double t = cp::timed_solve(m);
    std::println(stderr, "cfl {:7} after>={} RootCutLevel={}: {} first add at nodes_done={} adds={} rc errors={} (rc {}) relax={} node={}", name, after, argc > 3 ? argv[3] : "auto", cp::summary(m, t), done_first, adds, bad, rc, m.hook ? m.hook->relax_calls.load() : 0, m.hook ? m.hook->node_calls.load() : 0);
}
