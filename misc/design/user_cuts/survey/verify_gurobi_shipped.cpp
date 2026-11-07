// Shipped wrapper paths only, no native hook.
// lazyafter: is a candidate callback set after a solve honoured by the next solve()?
// dupdistinct: add_lazy_constraint(distinct_variables, ...) with a repeated variable.
#include "mippp/solvers/gurobi/all.hpp"
#include "cut_common.hpp"
#include <set>
using M = mippp::gurobi_milp;
using namespace mippp;
using namespace mippp::operators;
struct tsp {
    int n; std::vector<std::vector<double>> d;
    size_t id(int i, int j) const { return static_cast<size_t>(i) * static_cast<size_t>(n - 1) + static_cast<size_t>(j < i ? j : j - 1); }
};
static tsp make_tsp(int n, unsigned seed) {
    tsp T; T.n = n; const auto N = static_cast<size_t>(n);
    std::mt19937 rng(seed); std::uniform_real_distribution<double> P(0, 100), noise(1.0, 1.3);
    std::vector<std::pair<double, double>> pts(N); for(auto & p : pts) p = {P(rng), P(rng)};
    T.d.assign(N, std::vector<double>(N, 0));
    for(size_t i = 0; i < N; ++i) for(size_t j = 0; j < N; ++j) if(i != j) T.d[i][j] = std::round(std::hypot(pts[i].first - pts[j].first, pts[i].second - pts[j].second) * noise(rng));
    return T;
}
static int cycles(const tsp & T, const std::vector<double> & v, std::vector<std::vector<int>> * out = nullptr) {
    std::vector<char> seen(static_cast<size_t>(T.n), 0); int c = 0;
    for(int s = 0; s < T.n; ++s) if(!seen[static_cast<size_t>(s)]) {
        ++c; std::vector<int> cyc; int u = s;
        while(!seen[static_cast<size_t>(u)]) { seen[static_cast<size_t>(u)] = 1; cyc.push_back(u); int nx = -1; for(int j = 0; j < T.n; ++j) if(j != u && v[T.id(u, j)] > 0.5) { nx = j; break; } if(nx < 0) break; u = nx; }
        if(out) out->push_back(cyc);
    }
    return c;
}
static void lazyafter(const std::string & variant, int n, unsigned seed) {
    const tsp T = make_tsp(n, seed);
    const auto N = static_cast<size_t>(n), arcs = N * (N - 1);
    M m; m.set_thread_limit(1); m.set_time_limit(std::chrono::duration<double>(20.0));
    auto x = m.add_binary_variables(arcs);
    const auto nodes = std::views::iota(0, n);
    m.set_minimization();
    std::vector<size_t> ids; std::vector<double> cost;
    for(int i : nodes) for(int j : nodes) if(i != j) { ids.push_back(T.id(i, j)); cost.push_back(T.d[static_cast<size_t>(i)][static_cast<size_t>(j)]); }
    m.set_objective(xsum(std::views::iota(std::size_t{0}, ids.size()), [&](std::size_t k) { return cost[k] * x(ids[k]); }));
    for(int i : nodes) {
        std::vector<size_t> o, in; for(int j : nodes) if(j != i) { o.push_back(T.id(i, j)); in.push_back(T.id(j, i)); }
        m.add_constraint(xsum(o, [&](std::size_t k) { return 1.0 * x(k); }) == 1);
        m.add_constraint(xsum(in, [&](std::size_t k) { return 1.0 * x(k); }) == 1);
    }
    long calls = 0, lazies = 0;
    const auto set_cb = [&] {
        m.set_candidate_solution_callback([&](auto & h) {
            ++calls;
            auto sol = h.get_solution();
            std::vector<double> v(arcs); for(size_t k = 0; k < arcs; ++k) v[k] = sol[x(k)];
            std::vector<std::vector<int>> cyc; if(cycles(T, v, &cyc) <= 1) return;
            for(const auto & S : cyc) {
                std::set<int> in(S.begin(), S.end()); std::vector<size_t> cross;
                for(int i : S) for(int j = 0; j < T.n; ++j) if(!in.count(j)) cross.push_back(T.id(i, j));
                h.add_lazy_constraint(xsum(std::views::iota(std::size_t{0}, cross.size()), [&](std::size_t q) { return 1.0 * x(cross[q]); }) >= 1);
                ++lazies;
            }
        });
    };
    double first = NAN; std::string first_st = "-";
    if(variant == "before") set_cb();
    else {
        if(variant == "after_stop_node") m.set_node_limit(1);
        if(variant == "after_stop_sol") m.set_solution_limit(1);
        m.solve();
        first_st = cp::status_name(m.get_status());
        try { first = m.get_solution_value(); } catch(const std::exception &) {}
        set_cb();
        if(variant == "after_stop_node") m.set_node_limit(1000000000);
        if(variant == "after_stop_sol") m.set_solution_limit(2000000000);
        if(variant == "after_change") { m.set_objective_coefficient(x(0), cost[0] + 1e-9); }
    }
    m.solve();
    std::string st = cp::status_name(m.get_status());
    double val = NAN; int c = -1;
    try { val = m.get_solution_value(); auto s = m.get_solution(); std::vector<double> v(arcs); for(size_t k = 0; k < arcs; ++k) v[k] = s[x(k)]; c = cycles(T, v); } catch(const std::exception &) {}
    std::println(stderr, "tsp n={} seed={} {:18}: first {} {} | then {} value={} callback calls={} lazy added={} cycles in the returned point={} {}", n, seed, variant, first_st, first, st, val, calls, lazies, c, c == 1 ? "TOUR" : "NOT A TOUR");
}
static void dupdistinct(int mult) {
    // A small knapsack; at the first candidate, x0 repeated in a distinct-tagged lazy constraint.
    const int n = 40;
    M m; m.set_thread_limit(1); m.set_time_limit(std::chrono::duration<double>(10.0));
    auto x = m.add_binary_variables(static_cast<size_t>(n));
    std::mt19937 rng(3); std::uniform_int_distribution<int> A(1, 100);
    std::vector<double> c(static_cast<size_t>(n)), w(static_cast<size_t>(n)); double t = 0; for(int j = 0; j < n; ++j) { c[static_cast<size_t>(j)] = A(rng); w[static_cast<size_t>(j)] = A(rng); t += w[static_cast<size_t>(j)]; }
    const auto ids = std::views::iota(std::size_t{0}, static_cast<std::size_t>(n));
    m.set_maximization(); m.set_objective(xsum(ids, [&](std::size_t j) { return c[j] * x(j); }));
    m.add_constraint(xsum(ids, [&](std::size_t j) { return w[j] * x(j); }) <= std::floor(t / 2));
    long calls = 0; const int count = n * mult;
    m.set_candidate_solution_callback([&](auto & h) {
        if(calls++) return;
        h.add_lazy_constraint(distinct_variables, xsum(std::views::iota(0, count), [&](int) { return 1.0 * x(std::size_t{0}); }) <= 2.0 * count);
    });
    m.solve();
    std::println(stderr, "dupdistinct mult={} (x0 {} times, {} columns): {} calls={}", mult, count, n, cp::status_name(m.get_status()), calls);
}
int main(int argc, char ** argv) {
    const std::string w = argv[1];
    if(w == "lazyafter") { for(unsigned seed : {1u, 2u}) for(const char * v : {"before", "after_finish", "after_stop_node", "after_stop_sol", "after_change"}) lazyafter(v, 30, seed); }
    if(w == "dup") dupdistinct(std::atoi(argv[2]));
    return 0;
}
