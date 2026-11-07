// The honest fallback where no user-cut hook exists: a cutting-plane loop between solves,
// through add_constraint. Compile with -DUSE_HIGHS or -DUSE_MOSEK.
// usage: probe n <stages: letters of ABCDE> [cuts per round] [budget seconds per stage] [simplex|default for the MOSEK LP]
//   A  MTZ MIP, no cuts: what the user has without the loop
//   B  MTZ LP relaxation, DFJ cuts, re-solved until no cut is violated
//   C  MTZ MIP with the cuts of B as model rows from the start
//   D  degree-only: LP loop, then MIP loop on subtours of integer solutions
//   E  degree-only: MIP loop alone, the lazy-constraint fallback
#ifdef USE_MOSEK
#include "mippp/solvers/mosek/all.hpp"
using Milp = mippp::mosek_milp;
using Lp = mippp::mosek_lp;
static const char * const NAME = "mosek";
template <class X> static void one_thread(X & m) { m.set_thread_limit(1); }
static bool g_simplex = true;
#else
#include "mippp/solvers/highs/all.hpp"
using Milp = mippp::highs_milp;
using Lp = mippp::highs_lp;
static const char * const NAME = "highs";
template <class X> static void one_thread(X & m) { (void)m.native_api().setIntOptionValue(m.native_model(), "threads", 1); }
static bool g_simplex = false;
#endif
#include "cut_common.hpp"

static void lp_setup(Lp & m) {
    one_thread(m);
#ifdef USE_MOSEK
    if(g_simplex) {
        auto f = cp::symf<int (*)(void *, const char *, int)>(m.native_api().library_path(), "MSK_putnaintparam");
        // MSK_OPTIMIZER_DUAL_SIMPLEX = 1 in mosek.h: warm starts from the previous basis
        if(f(m.native_model().second, "MSK_IPAR_OPTIMIZER", 1)) std::println(stderr, "could not set the dual simplex");
    }
#else
    (void)m;
#endif
}
struct inst { int n; std::vector<std::vector<double>> d; };
static inst make_inst(const int n, const unsigned seed = 5) {
    const auto N = static_cast<std::size_t>(n);
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> P(0, 100), noise(1.0, 1.3);
    std::vector<std::pair<double, double>> pts(N);
    for(auto & p : pts) p = {P(rng), P(rng)};
    inst I; I.n = n; I.d.assign(N, std::vector<double>(N, 0));
    for(std::size_t i = 0; i < N; ++i) for(std::size_t j = 0; j < N; ++j) if(i != j)
        I.d[i][j] = std::round(std::hypot(pts[i].first - pts[j].first, pts[i].second - pts[j].second) * noise(rng));
    return I;
}
// the model of cp::build_atsp, also over continuous x, and without the MTZ rows
template <class M, class Xs>
static cp::atsp fill(M & m, Xs & x, const inst & I, const bool mtz) {
    const int n = I.n;
    const auto N = static_cast<std::size_t>(n);
    const auto id = [N](std::size_t i, std::size_t j) { return i * (N - 1) + (j < i ? j : j - 1); };
    cp::atsp a; a.n = n; a.d = I.d;
    a.col.assign(N, std::vector<int>(N, -1));
    a.comp.assign(N, std::vector<int>(N, -1));
    for(std::size_t i = 0; i < N; ++i) for(std::size_t j = 0; j < N; ++j) if(i != j) a.col[i][j] = m.native_id(x(id(i, j)));
    m.set_minimization();
    const auto nodes = std::views::iota(std::size_t{0}, N);
    m.set_objective(xsum(nodes, [&](std::size_t i) {
        return xsum(nodes | std::views::filter([i](std::size_t j) { return j != i; }), [&, i](std::size_t j) { return a.d[i][j] * x(id(i, j)); });
    }));
    for(std::size_t i : nodes) {
        m.add_constraint(xsum(nodes | std::views::filter([i](std::size_t j) { return j != i; }), [&, i](std::size_t j) { return 1.0 * x(id(i, j)); }) == 1);
        m.add_constraint(xsum(nodes | std::views::filter([i](std::size_t j) { return j != i; }), [&, i](std::size_t j) { return 1.0 * x(id(j, i)); }) == 1);
    }
    if(mtz) {
        auto u = m.add_variables(N, {.obj_coef = 0, .lower_bound = 0, .upper_bound = static_cast<double>(n - 1)});
        for(std::size_t i = 1; i < N; ++i) for(std::size_t j = 1; j < N; ++j) if(i != j)
            m.add_constraint(u(i) - u(j) + static_cast<double>(n - 1) * x(id(i, j)) <= static_cast<double>(n - 2));
    }
    return a;
}
// a raw cut over x, whose native columns are the first ones
template <class M, class Xs>
static void add_row(M & m, Xs & x, const cp::raw_cut & c) {
    const auto K = std::views::iota(std::size_t{0}, c.ind.size());
    m.add_constraint(xsum(K, [&](std::size_t k) { return c.val[k] * x(static_cast<std::size_t>(c.ind[k])); }) >= c.rhs);
}
template <class M, class Xs>
static std::vector<double> read_x(M & m, Xs & x, const std::size_t arcs) {
    auto sol = m.get_solution();
    std::vector<double> v(arcs);
    for(std::size_t j = 0; j < arcs; ++j) v[j] = sol[x(j)];
    return v;
}
static double secs(clk::time_point a, clk::time_point b) { return std::chrono::duration<double>(b - a).count(); }

struct lp_loop_result {
    int rounds = 0;
    std::vector<cp::raw_cut> cuts;
    double solve_time = 0, sep_time = 0, first_solve = 0, last_solve = 0, bound0 = NAN, bound = NAN;
    bool finished = false;
};
static lp_loop_result lp_loop(const inst & I, const bool mtz, const std::size_t per_round, const double budget) {
    lp_loop_result R;
    Lp m; lp_setup(m);
    const std::size_t arcs = static_cast<std::size_t>(I.n * (I.n - 1));
    auto x = m.add_variables(arcs, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 1});
    auto a = fill(m, x, I, mtz);
    m.set_time_limit(std::chrono::duration<double>(budget));
    while(R.solve_time + R.sep_time < budget) {
        const auto t0 = clk::now();
        m.solve();
        const auto t1 = clk::now();
        ++R.rounds;
        const double st = secs(t0, t1);
        R.solve_time += st;
        if(R.rounds == 1) R.first_solve = st;
        R.last_solve = st;
        R.bound = m.get_solution_value();
        if(R.rounds == 1) R.bound0 = R.bound;
        const auto point = read_x(m, x, arcs);
        auto cuts = a.separate(point, per_round);
        R.sep_time += secs(t1, clk::now());
        if(cuts.empty()) { R.finished = true; break; }
        for(auto & c : cuts) { add_row(m, x, c); R.cuts.push_back(std::move(c)); }
    }
    return R;
}
static std::vector<std::vector<int>> cycles_of(const cp::atsp & a, const std::vector<double> & x) {
    const auto N = static_cast<std::size_t>(a.n);
    std::vector<int> next(N, -1);
    for(std::size_t i = 0; i < N; ++i) for(std::size_t j = 0; j < N; ++j) if(i != j && x[static_cast<std::size_t>(a.col[i][j])] > 0.5) next[i] = static_cast<int>(j);
    std::vector<char> seen(N, 0);
    std::vector<std::vector<int>> out;
    for(std::size_t s = 0; s < N; ++s) if(!seen[s]) {
        std::vector<int> c;
        for(int v = static_cast<int>(s); v >= 0 && !seen[static_cast<std::size_t>(v)]; v = next[static_cast<std::size_t>(v)]) { seen[static_cast<std::size_t>(v)] = 1; c.push_back(v); }
        out.push_back(std::move(c));
    }
    return out;
}
struct mip_loop_result { int solves = 0, rows = 0; double time = 0, max_solve = 0; double value = NAN; bool finished = false; long nodes = 0; std::string last; };
static mip_loop_result mip_loop(const inst & I, const std::vector<cp::raw_cut> & seed_cuts, const double budget) {
    mip_loop_result R;
    Milp m; one_thread(m);
    const std::size_t arcs = static_cast<std::size_t>(I.n * (I.n - 1));
    auto x = m.add_binary_variables(arcs);
    auto a = fill(m, x, I, false);
    for(const auto & c : seed_cuts) add_row(m, x, c);
    const auto N = static_cast<std::size_t>(I.n);
    while(R.time < budget) {
        m.set_time_limit(std::chrono::duration<double>(std::max(1.0, budget - R.time)));
        const double st = cp::timed_solve(m);
        ++R.solves;
        R.time += st;
        R.max_solve = std::max(R.max_solve, st);
        if constexpr(has_node_count<Milp>) { try { R.nodes += static_cast<long>(m.get_node_count()); } catch(const std::exception &) {} }
        R.last = cp::summary(m, st);
        if(!std::holds_alternative<status::optimal>(m.get_status())) break;
        R.value = m.get_solution_value();
        const auto point = read_x(m, x, arcs);
        const auto cs = cycles_of(a, point);
        if(cs.size() == 1) { R.finished = true; break; }
        for(const auto & S : cs) {
            std::uint64_t mask = 0;
            for(int v : S) mask |= std::uint64_t{1} << v;
            cp::raw_cut c; c.sense = 'G'; c.rhs = 1.0;
            for(std::size_t i = 0; i < N; ++i) for(std::size_t j = 0; j < N; ++j) if(i != j && (mask >> i & 1) && !(mask >> j & 1)) { c.ind.push_back(a.col[i][j]); c.val.push_back(1.0); }
            add_row(m, x, c);
            ++R.rows;
        }
    }
    return R;
}
// The trap a user can reach with the wrapper as it stands: the lambda of set_progress_callback
// captures the model and calls add_constraint. Every subtour row of a small degree-only ATSP,
// so the right answer is a tour, and it is known from the same rows added before the solve.
static void wrapper_trap(const int n, const unsigned seed, const bool from_callback) {
    const inst I = make_inst(n, seed);
    Milp m; one_thread(m);
    const std::size_t arcs = static_cast<std::size_t>(n * (n - 1));
    auto x = m.add_binary_variables(arcs);
    auto a = fill(m, x, I, false);
    std::vector<cp::raw_cut> rows;
    for(std::uint32_t mask = 1; mask < (1u << (n - 1)); ++mask) {
        std::vector<int> S;
        for(int b = 0; b < n - 1; ++b) if(mask >> b & 1) S.push_back(b + 1);
        if(S.size() < 2) continue;
        cp::raw_cut c; c.sense = 'G'; c.rhs = 1.0;
        std::uint32_t full = mask << 1;
        for(int i = 0; i < n; ++i) for(int j = 0; j < n; ++j) if(i != j && (full >> i & 1) && !(full >> j & 1)) {
            c.ind.push_back(a.col[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]);
            c.val.push_back(1.0);
        }
        rows.push_back(std::move(c));
    }
    m.set_time_limit(std::chrono::duration<double>(30));
    bool done = false;
    std::string in_hook = "exception: none";
    if(from_callback) m.set_progress_callback([&](auto &) {
        if(done) return;
        done = true;
        try { for(const auto & c : rows) add_row(m, x, c); }
        catch(const std::exception & e) { in_hook = std::string("exception: ") + e.what(); }
    });
    else for(const auto & c : rows) add_row(m, x, c);
    const double t = cp::timed_solve(m);
    std::string cyc = "no solution to read";
    try { if(m.get_status().index() != 0) { const auto point = read_x(m, x, arcs); cyc = std::format("{} cycle(s)", cycles_of(a, point).size()); } } catch(const std::exception & e) { cyc = e.what(); }
    std::println(stderr, "T {} n={} seed={} rows={} {}: {} final point: {} hook ran={} {}", NAME, n, seed, rows.size(), from_callback ? "add_constraint from the progress lambda" : "add_constraint before solve", cp::summary(m, t), cyc, done, in_hook);
}
int main(int argc, char ** argv) {
    if(argc > 1 && std::string(argv[1]) == "trap") {
        for(unsigned seed : {1u, 2u, 3u}) { wrapper_trap(10, seed, false); wrapper_trap(10, seed, true); }
        return 0;
    }
    const int n = argc > 1 ? std::atoi(argv[1]) : 26;
    const std::string stages = argc > 2 ? argv[2] : "ABCDE";
    const std::size_t per_round = argc > 3 ? static_cast<std::size_t>(std::atoi(argv[3])) : 5;
    const double budget = argc > 4 ? std::atof(argv[4]) : 60;
    if(argc > 5) g_simplex = std::string(argv[5]) == "simplex";
    const inst I = make_inst(n);
    { Milp m; std::println(stderr, "=== {} {} n={} cuts per round={} budget={}s lp={}", NAME, m.native_api().library_path().string(), n, per_round, budget, g_simplex ? "simplex" : "default"); }
    const auto has = [&](char c) { return stages.find(c) != std::string::npos; };
    if(has('A')) {
        Milp m; one_thread(m);
        auto x = m.add_binary_variables(static_cast<std::size_t>(n * (n - 1)));
        auto a = fill(m, x, I, true);
        m.set_time_limit(std::chrono::duration<double>(budget));
        const double t = cp::timed_solve(m);
        std::println(stderr, "A MTZ MIP, no cuts: {}", cp::summary(m, t));
    }
    std::vector<cp::raw_cut> cuts;
    if(has('B') || has('C')) {
        const auto R = lp_loop(I, true, per_round, budget);
        std::println(stderr, "B MTZ LP loop: rounds={} cuts={} finished={} lp_solve={:.3f}s separate={:.3f}s first_solve={:.4f}s last_solve={:.4f}s bound {:.6g} -> {:.6g}",
                     R.rounds, R.cuts.size(), R.finished, R.solve_time, R.sep_time, R.first_solve, R.last_solve, R.bound0, R.bound);
        cuts = R.cuts;
    }
    if(has('C')) {
        Milp m; one_thread(m);
        auto x = m.add_binary_variables(static_cast<std::size_t>(n * (n - 1)));
        auto a = fill(m, x, I, true);
        for(const auto & c : cuts) add_row(m, x, c);
        m.set_time_limit(std::chrono::duration<double>(budget));
        const double t = cp::timed_solve(m);
        std::println(stderr, "C MTZ MIP with the {} cuts as rows from the start: {}", cuts.size(), cp::summary(m, t));
    }
    if(has('D')) {
        const auto R = lp_loop(I, false, per_round, budget);
        std::println(stderr, "D1 degree-only LP loop: rounds={} cuts={} finished={} lp_solve={:.3f}s separate={:.3f}s first_solve={:.4f}s last_solve={:.4f}s bound {:.6g} -> {:.6g}",
                     R.rounds, R.cuts.size(), R.finished, R.solve_time, R.sep_time, R.first_solve, R.last_solve, R.bound0, R.bound);
        const auto L = mip_loop(I, R.cuts, budget);
        std::println(stderr, "D2 then MIP loop on subtours: solves={} rows_added={} finished={} mip_time={:.3f}s max_solve={:.3f}s nodes={} value={:.6g} | last: {}", L.solves, L.rows, L.finished, L.time, L.max_solve, L.nodes, L.value, L.last);
    }
    if(has('E')) {
        const auto L = mip_loop(I, {}, budget);
        std::println(stderr, "E degree-only MIP loop alone: solves={} rows_added={} finished={} mip_time={:.3f}s max_solve={:.3f}s nodes={} value={:.6g} | last: {}", L.solves, L.rows, L.finished, L.time, L.max_solve, L.nodes, L.value, L.last);
    }
}
