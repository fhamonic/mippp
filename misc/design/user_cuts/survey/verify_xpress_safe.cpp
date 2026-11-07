// D: truly valid cuts vs the recipes, at default settings. One process per case (fork), so a segfault is a result.
// Every cut below is valid for every integer feasible point of the model by a stated argument.
#include "verify_xpress_hook.hpp"
#include <sys/wait.h>
#include <unistd.h>
using namespace mippp; using namespace mippp::operators;
using std::chrono::duration;
struct outcome { std::string text; };
static void one_thread(M & m) { m.set_thread_limit(1); }
static bool same(double a, double b) { return std::fabs(a - b) <= 1e-6 * (1 + std::fabs(a)); }
// Runs baseline (no hook), then with the cuts; prints a verdict. Build returns separator.
template <class Build>
static void kase(const std::string & name, const std::string & spec, Build build, double limit = 30) {
    double base; std::string bs;
    {
        M m; one_thread(m);
        auto keep = build(m); m.set_time_limit(duration<double>(limit));
        const double t = cp::timed_solve(m);
        base = cf::value_of(m); bs = cp::summary(m, t);
        if(!is_a<status::optimal>(m.get_status())) { std::println(stderr, "NOBASE {} [{}]: {}", name, spec, bs); }
    }
    M m; one_thread(m);
    auto sep = build(m); m.set_time_limit(duration<double>(limit));
    auto s = std::make_shared<cp::stats>();
    vx::recipe r = vx::parse(spec);
    auto & c = vx::attach(m, cp::watched(sep, s), r);
    const double t = cp::timed_solve(m);
    const double v = cf::value_of(m);
    const bool opt = is_a<status::optimal>(m.get_status());
    std::println(stderr, "{} {} [{}]: base {} | {} | calls={} cuts={} errs={} (last {})\n   {}", (opt && same(v, base)) ? "OK  " : "BAD ", name, spec, base, cp::summary(m, t), s->calls.load(), s->cuts.load(), s->add_errors.load(), s->last_error, c.line());
}
// Runs each in a child: rc is reported, a crash does not stop the sweep.
template <class F>
static void isolated(const std::string & name, const std::string & spec, F f, int seconds = 70) {
    std::fflush(stderr);
    const pid_t p = fork();
    if(p == 0) { alarm(static_cast<unsigned>(seconds)); vx::quiet_flush = true; f(); std::fflush(stderr); _exit(0); }
    int st = 0; waitpid(p, &st, 0);
    if(WIFSIGNALED(st)) std::println(stderr, "CRASH {} [{}]: killed by signal {}", name, spec, WTERMSIG(st));
    else if(WEXITSTATUS(st)) std::println(stderr, "CHILD {} [{}]: exit {}", name, spec, WEXITSTATUS(st));
}
// A: one-shot injection. Applies `cuts` the first time the hook runs at the root, and then nothing else; on top of the DFJ separator when asked.
static cp::separator once_then(std::vector<cp::raw_cut> cuts, cp::separator rest = {}) {
    auto done = std::make_shared<bool>(false);
    return [cuts, done, rest](const std::vector<double> & x, const cp::add_fn & add) {
        if(!*done) { *done = true; for(const auto & c : cuts) add(c); }
        if(rest) rest(x, add);
    };
}
static cp::separator every_call(std::vector<cp::raw_cut> cuts, cp::separator rest = {}) {
    return [cuts, rest](const std::vector<double> & x, const cp::add_fn & add) { for(const auto & c : cuts) add(c); if(rest) rest(x, add); };
}
static cp::separator dfj(const cp::atsp & a) { return [a](const std::vector<double> & x, const cp::add_fn & add) { for(const auto & c : a.separate(x)) add(c); }; }

// A gadget block keeps a tree alive: a 30 x 12 knapsack whose baseline needs nodes.
template <class Mo> static auto tree_block(Mo & m, unsigned seed = 3, std::size_t cols = 40, std::size_t rows = 5) {
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> A(1, 100), R(0, 50);
    std::vector<std::vector<double>> a(rows, std::vector<double>(cols));
    for(auto & r : a) for(auto & v : r) v = A(rng);
    std::vector<double> c(cols), cap(rows);
    for(std::size_t j = 0; j < cols; ++j) { double s = 0; for(auto & r : a) s += r[j]; c[j] = std::floor(s / static_cast<double>(rows) + R(rng)); }
    for(std::size_t i = 0; i < rows; ++i) { double s = 0; for(double v : a[i]) s += v; cap[i] = std::floor(s / 2); }
    auto x = m.add_binary_variables(cols);
    const auto ids = std::views::iota(std::size_t{0}, cols);
    m.set_maximization();
    m.set_objective(xsum(ids, [&](std::size_t j) { return c[j] * x(j); }));
    for(std::size_t i = 0; i < rows; ++i) m.add_constraint(xsum(ids, [&, i](std::size_t j) { return a[i][j] * x(j); }) <= cap[i]);
    std::vector<int> col; for(std::size_t j = 0; j < cols; ++j) col.push_back(m.native_id(x(j)));
    return std::tuple{x, col, c, a, cap};
}
// Reductions block: a small independent part with columns presolve removes, added to the tree block's objective.
// dup t,s: identical binaries, in t + s + q <= 1 with q another binary, both worth +10 (maximize): duplicate columns
// dual w: continuous [0,1], cost -3 in maximize and in no row: dual fix at 0
// sing v: continuous [0,5], singleton in v + p == 1 with p binary: free-singleton style elimination
// fix f: binary with f == 1
// sub k,h: general integer k in [0,10], continuous h == 2 k + 3: substitution; k <= 4 by row 
struct block { int t, s, q, w, v, p, f, k, h; };
template <class X>
static block add_block(M & m, const X & obj_expr_add) {
    auto b = m.add_binary_variables(5);  // t s q p f
    auto w = m.add_variables(1, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 1});
    auto v = m.add_variables(1, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 5});
    auto k = m.add_integer_variables(1, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 10});
    auto h = m.add_variables(1, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 100});
    const std::size_t z = 0;
    block B{m.native_id(b(z)), m.native_id(b(z + 1)), m.native_id(b(z + 2)), m.native_id(w(z)), m.native_id(v(z)), m.native_id(b(z + 3)), m.native_id(b(z + 4)), m.native_id(k(z)), m.native_id(h(z))};
    m.add_constraint(b(z) + b(z + 1) + b(z + 2) <= 1);
    m.add_constraint(v(z) + b(z + 3) == 1);
    m.add_constraint(1.0 * b(z + 4) == 1);
    m.add_constraint(h(z) - 2.0 * k(z) == 3.0);
    m.add_constraint(1.0 * k(z) <= 4.0);
    m.set_objective_coefficient(b(z), 10); m.set_objective_coefficient(b(z + 1), 10); m.set_objective_coefficient(b(z + 2), 7);
    m.set_objective_coefficient(w(z), -3); m.set_objective_coefficient(v(z), 1); m.set_objective_coefficient(b(z + 3), 2);
    m.set_objective_coefficient(b(z + 4), 1); m.set_objective_coefficient(h(z), 1);
    (void)obj_expr_add;
    return B;
}
using cut = cp::raw_cut;
static std::vector<std::string> specs = {"R1", "R2"};
static void reductions_cases() {
    // Each row: name and the VALID cut on reduced columns of `block`.
    struct K { std::string name; std::function<std::vector<cut>(const block &)> make; };
    const std::vector<K> ks = {
        {"dup: t + s <= 1 (implied, L)", [](const block & b) { return std::vector<cut>{{{b.t, b.s}, {1, 1}, 'L', 1}}; }},
        {"dup: t <= 1 (one-term, single duplicate)", [](const block & b) { return std::vector<cut>{{{b.t}, {1}, 'L', 1}}; }},
        {"dup: t + 2 s <= 2 (unequal coefficients, valid since t + s <= 1)", [](const block & b) { return std::vector<cut>{{{b.t, b.s}, {1, 2}, 'L', 2}}; }},
        {"dup: t + s + q <= 1 as E? no: L", [](const block & b) { return std::vector<cut>{{{b.t, b.s, b.q}, {1, 1, 1}, 'L', 1}}; }},
        {"dup: t >= 0 (G, one term)", [](const block & b) { return std::vector<cut>{{{b.t}, {1}, 'G', 0}}; }},
        {"dual: w >= 0 (G, on a dual-fixed column)", [](const block & b) { return std::vector<cut>{{{b.w}, {1}, 'G', 0}}; }},
        {"dual: w <= 1 (L, on a dual-fixed column)", [](const block & b) { return std::vector<cut>{{{b.w}, {1}, 'L', 1}}; }},
        {"sing: v + p <= 1 (the row as L)", [](const block & b) { return std::vector<cut>{{{b.v, b.p}, {1, 1}, 'L', 1}}; }},
        {"sing: v + p >= 1 (the row as G)", [](const block & b) { return std::vector<cut>{{{b.v, b.p}, {1, 1}, 'G', 1}}; }},
        {"sing: v <= 1 (implied by the row, tighter than the bound 5)", [](const block & b) { return std::vector<cut>{{{b.v}, {1}, 'L', 1}}; }},
        {"fix: f >= 1 (one term on a fixed column)", [](const block & b) { return std::vector<cut>{{{b.f}, {1}, 'G', 1}}; }},
        {"fix: f <= 1", [](const block & b) { return std::vector<cut>{{{b.f}, {1}, 'L', 1}}; }},
        {"fix: 0.1 f >= 0.1 (constant row with a rounding residue)", [](const block & b) { return std::vector<cut>{{{b.f}, {0.1}, 'G', 0.1}}; }},
        {"sub: h <= 11 (implied through k <= 4)", [](const block & b) { return std::vector<cut>{{{b.h}, {1}, 'L', 11}}; }},
        {"sub: h - 2 k <= 3 (the row as L, on substituted columns)", [](const block & b) { return std::vector<cut>{{{b.h, b.k}, {1, -2}, 'L', 3}}; }},
        {"sub: h - 2 k >= 3 (the row as G)", [](const block & b) { return std::vector<cut>{{{b.h, b.k}, {1, -2}, 'G', 3}}; }},
        {"sub: k <= 4 (tight at the optimum)", [](const block & b) { return std::vector<cut>{{{b.k}, {1}, 'L', 4}}; }},
        {"E: v + p == 1 (valid equality on eliminated columns)", [](const block & b) { return std::vector<cut>{{{b.v, b.p}, {1, 1}, 'E', 1}}; }},
        {"E: h - 2 k == 3 (valid equality on substituted columns)", [](const block & b) { return std::vector<cut>{{{b.h, b.k}, {1, -2}, 'E', 3}}; }},
        {"E: f == 1 (valid equality on a fixed column)", [](const block & b) { return std::vector<cut>{{{b.f}, {1}, 'E', 1}}; }},
    };
    for(const auto & spec : specs) for(const auto & k : ks) for(const bool everycall : {false, true}) {
        const std::string name = std::format("reduced {} {}", k.name, everycall ? "(every call)" : "(once)");
        isolated(name, spec, [&] {
            kase(name, spec, [&](M & m) {
                auto tb = tree_block(m);
                block b = add_block(m, 0);
                return everycall ? every_call(k.make(b)) : once_then(k.make(b));
            });
        });
    }
}
// Duplicates that remain duplicates after the block: valid equalities. t + s + w' == 1 is a real equality row when q is replaced by the slack of the row.
static void equality_on_duplicates() {
    struct K { std::string name; bool valid; std::function<std::vector<cut>(int, int, int)> make; };
    const std::vector<K> ks = {
        {"VALID t + s + q == 1 (the row itself) ", true, [](int t, int s, int q) { return std::vector<cut>{{{t, s, q}, {1, 1, 1}, 'E', 1}}; }},
        {"VALID 2t + 2s + 2q == 2 (scaled) ", true, [](int t, int s, int q) { return std::vector<cut>{{{t, s, q}, {2, 2, 2}, 'E', 2}}; }},
        {"VALID t + s + q <= 1 and >= 1 (as two inequalities)", true, [](int t, int s, int q) { return std::vector<cut>{{{t, s, q}, {1, 1, 1}, 'L', 1}, {{t, s, q}, {1, 1, 1}, 'G', 1}}; }},
        {"INVALID-DOMINANCE t == 0 (not valid for t=1)", false, [](int t, int, int) { return std::vector<cut>{{{t}, {1}, 'E', 0}}; }},
    };
    for(const auto & spec : {"R1,no_split_eq", "R2,no_split_eq", "R1", "R2"}) for(const auto & k : ks) for(const char * mode : {"default", "dualred0"}) {
        const std::string name = std::format("dupE {} {}", k.name, mode);
        isolated(name, spec, [&] {
            kase(name, spec, [&](M & m) {
                if(std::string(mode) == "dualred0") vx::setint(m, vx::C_MIPDUALREDUCTIONS, 0);
                auto tb = tree_block(m);
                auto b = m.add_binary_variables(3);
                const std::size_t z = 0;
                m.add_constraint(b(z) + b(z + 1) + b(z + 2) == 1);
                m.set_objective_coefficient(b(z), 10); m.set_objective_coefficient(b(z + 1), 10); m.set_objective_coefficient(b(z + 2), 7);
                return once_then(k.make(m.native_id(b(z)), m.native_id(b(z + 1)), m.native_id(b(z + 2))));
            }, 15);
        }, 40);
    }
}
// C4-style parallel columns: an arc is a + b; the degree row as an equality cut.
static void equality_on_parallel() {
    for(const auto & spec : {"R1,no_split_eq", "R2,no_split_eq", "R1", "R2"}) {
        const std::string name = "parallel columns: degree row of node 1 as E";
        isolated(name, spec, [&] {
            kase(name, spec, [&](M & m) {
                const int n = 16; const auto N = static_cast<std::size_t>(n);
                std::mt19937 rng(2);
                std::uniform_real_distribution<double> P(0, 100);
                std::vector<std::vector<double>> d(N, std::vector<double>(N));
                for(auto & r : d) for(auto & v : r) v = std::round(P(rng));
                auto a = m.add_binary_variables(N * N); auto b = m.add_binary_variables(N * N);
                auto u = m.add_variables(N, {.obj_coef = 0, .lower_bound = 0, .upper_bound = static_cast<double>(n - 1)});
                const auto nodes = std::views::iota(std::size_t{0}, N);
                const auto id = [N](std::size_t i, std::size_t j) { return i * N + j; };
                m.set_minimization();
                m.set_objective(xsum(nodes, [&](std::size_t i) { return xsum(nodes, [&, i](std::size_t j) { return d[i][j] * (a(id(i, j)) + b(id(i, j))); }); }));
                for(std::size_t i : nodes) {
                    m.set_variable_upper_bound(a(id(i, i)), 0); m.set_variable_upper_bound(b(id(i, i)), 0);
                    m.add_constraint(xsum(nodes, [&, i](std::size_t j) { return a(id(i, j)) + b(id(i, j)); }) == 1);
                    m.add_constraint(xsum(nodes, [&, i](std::size_t j) { return a(id(j, i)) + b(id(j, i)); }) == 1);
                }
                for(std::size_t i = 1; i < N; ++i) for(std::size_t j = 1; j < N; ++j) if(i != j)
                    m.add_constraint(u(i) - u(j) + static_cast<double>(n - 1) * (a(id(i, j)) + b(id(i, j))) <= static_cast<double>(n - 2));
                cut c{{}, {}, 'E', 1.0};
                for(std::size_t j : nodes) if(j != 1) { c.ind.push_back(m.native_id(a(id(1, j)))); c.val.push_back(1); c.ind.push_back(m.native_id(b(id(1, j)))); c.val.push_back(1); }
                return once_then({c});
            }, 30);
        }, 90);
    }
}
// Scaling a valid DFJ cut: scale=... in the recipe is applied just before the native call, so sanitize sees the true cut.
// Scaled by 1e-12 a coefficient may fall under the matrix tolerance: 0 >= tiny is a false infeasible row.
static void scaled_dfj(const std::string & spec_base, unsigned seed, int n) {
    for(const char * sc : {"1e-12", "1e-9", "1e-7", "1e-5", "1e-3", "1e3", "1e5", "1e7", "1e9", "1e12"}) {
        const std::string spec = spec_base + ",scale=" + sc;
        const std::string name = std::format("scaled DFJ x {} n={} seed={}", sc, n, seed);
        isolated(name, spec, [&] {
            kase(name, spec, [&](M & m) { auto a = cp::build_atsp(m, n, false, seed); return dfj(a); }, 15);
        }, 45);
    }
}
// Near-duplicate cuts: at each call every DFJ cut goes in 5 times, perturbed by relative 1e-12 .. 1e-7, in rhs and in a coefficient.
static void near_duplicates(const std::string & spec) {
    for(const bool every : {false, true}) {
        const std::string name = std::format("near-duplicate DFJ, relaxed by tiny amounts ({})", every ? "re-offers all past cuts at every call" : "new cuts only");
        isolated(name, spec, [&] {
            kase(name, spec, [&](M & m) {
                auto a = cp::build_atsp(m, 20, false, 3);
                auto all = std::make_shared<std::vector<cut>>();
                return cp::separator([a, every, all](const std::vector<double> & x, const cp::add_fn & add) {
                    std::vector<cut> found = a.separate(x);
                    for(const auto & c : found) for(double eps : {0.0, 1e-12, 1e-10, 1e-9, 1e-8, 1e-7}) {
                        cut k = c; k.rhs -= eps;  // >= rhs - eps is weaker, so valid
                        all->push_back(k);
                    }
                    if(every) { for(const auto & c : *all) add(c); }
                    else { for(const auto & c : found) for(double eps : {0.0, 1e-12, 1e-10, 1e-9, 1e-8, 1e-7}) { cut k = c; k.rhs -= eps; add(k); } }
                });
            }, 20);
        }, 60);
    }
}
// The objective cut: c x <= opt on the knapsack, tight at every optimum; and c x <= opt on the root LP point at each call.
static void tight_cuts(const std::string & spec) {
    for(unsigned seed : {1u, 2u, 3u}) {
        double truth;
        { M m; one_thread(m); m.set_time_limit(duration<double>(30)); tree_block(m, seed); m.solve(); truth = m.get_solution_value(); }
        const std::string name = std::format("objective cut c x <= opt={} seed={}", truth, seed);
        isolated(name, spec, [&] {
            kase(name, spec, [&](M & m) {
                auto [x, col, c, a, cap] = tree_block(m, seed);
                cut k{col, c, 'L', truth};
                return every_call({k});
            });
        });
        // Tight at the optimum and with rhs at the exact optimum minus 1e-9 of rounding error: the cut is valid up to float noise.
        const std::string n2 = std::format("objective cut scaled by 1/3 seed={}", seed);
        isolated(n2, spec, [&] {
            kase(n2, spec, [&](M & m) {
                auto [x, col, c, a, cap] = tree_block(m, seed);
                std::vector<double> v = c; for(auto & e : v) e /= 3;
                cut k{col, v, 'L', truth / 3};
                return every_call({k});
            });
        });
    }
}
// One-term cuts on a column that every integer feasible point holds at one value, on tree-bearing models.
static void one_term(const std::string & spec) {
    // Model: x0 + x1 >= 2 forces both to 1 without presolve seeing it? presolve sees it. Use a weaker form:
    // 3 x0 + 3 x1 + 3 x2 <= 4 and x0 + x1 + x2 >= 1 implies exactly one; a one-term valid cut needs a forced value:
    // knapsack with item weight > capacity: item j never fits, so x_j <= 0 is valid; presolve fixes it.
    for(const bool fractional_rhs : {false, true}) {
        const std::string name = std::format("one term: x_big <= {} on an item that never fits", fractional_rhs ? "0.5" : "0");
        isolated(name, spec, [&] {
            kase(name, spec, [&](M & m) {
                auto [x, col, c, a, cap] = tree_block(m, 3);
                auto big = m.add_binary_variable();
                m.add_constraint(1000.0 * big + xsum(std::views::iota(std::size_t{0}, std::size_t{5}), [&, xx = x](std::size_t j) { return 1.0 * xx(j); }) <= 500);
                m.set_objective_coefficient(big, 50);
                return every_call({cut{{m.native_id(big)}, {1}, 'L', fractional_rhs ? 0.5 : 0.0}});
            });
        });
    }
    // General integer: 2 k <= 9 with k integer gives the CG cut k <= 4, one term, cuts the LP point; rhs at a shifted column.
    {
        const std::string name = "one term: CG k <= 4 from 2 k <= 9";
        isolated(name, spec, [&] {
            kase(name, spec, [&](M & m) {
                auto [x, col, c, a, cap] = tree_block(m, 3);
                auto k = m.add_integer_variable({.lower_bound = 0., .upper_bound = 20.});
                m.add_constraint(2.0 * k <= 9.0);
                m.set_objective_coefficient(k, 5);
                return every_call({cut{{m.native_id(k)}, {1}, 'L', 4}});
            });
        });
    }
    {
        // x <= 1 scaled huge on a binary: 1e8 x <= 1e8, and coefficients at 1e-8: 1e-8 x <= 1e-8 (under the matrix tolerance?)
        for(double sc : {1e8, 1e-8, 1e-11}) {
            const std::string name = std::format("big/small coefficient: {} x_j + {} x_k <= {} on x_j + x_k <= 1 (conflict)", sc, sc, sc);
            isolated(name, spec, [&] {
                kase(name, spec, [&](M & m) {
                    auto [x, col, c, a, cap] = tree_block(m, 3);
                    auto p = m.add_binary_variable(); auto q = m.add_binary_variable();
                    m.add_constraint(p + q <= 1);
                    m.add_constraint(xsum(std::views::iota(std::size_t{0}, std::size_t{5}), [&, xx = x](std::size_t j) { return 1.0 * xx(j); }) + p - q <= 3);
                    m.set_objective_coefficient(p, 20); m.set_objective_coefficient(q, 25);
                    return every_call({cut{{m.native_id(p), m.native_id(q)}, {sc, sc}, 'L', sc}});
                });
            });
        }
    }
    {
        // Huge rhs, not violated: dropped by sanitize. Without sanitize? A rhs at the solver's infinity.
        for(const char * sp2 : {"", ",keep_satisfied"}) {
            const std::string s3 = spec + sp2;
            for(double rhs : {1e20, 1e30, 1e40}) {
                const std::string name = std::format("rhs {} not violated", rhs);
                isolated(name, s3, [&] {
                    kase(name, s3, [&](M & m) {
                        auto [x, col, c, a, cap] = tree_block(m, 3);
                        return once_then({cut{{col[0], col[1]}, {1, 1}, 'L', rhs}, cut{{col[0], col[1]}, {1, 1}, 'G', -rhs}});
                    });
                });
            }
        }
    }
}
// Cuts at a node about to be pruned: the cut makes the node LP infeasible although it is valid, because no integer point lives there.
// Model: y binary; if y = 1 then an odd-parity gadget has no integer point but the LP does. y = 0 worse: 10 less.
// Valid cut: y <= 0. Added only when the hook sees y = 1 at a node, and also a cut with violation at incumbents' cutoff.
static void pruned(const std::string & spec) {
    for(const int variant : {0, 1, 2}) {
        const std::string name = variant == 0 ? "prune: y <= 0 at nodes with y = 1 (no integer point there)" : variant == 1 ? "prune: objective cut violated above the incumbent at every node" : "all-integer-infeasible model, cut y <= 0 and y >= 1 valid vacuously";
        isolated(name, spec, [&] {
            M m0; (void)m0;
            kase(name, spec, [&](M & m) {
                auto [x, col, c, a, cap] = tree_block(m, 3, 30, 4);
                auto y = m.add_binary_variable();
                auto o = m.add_binary_variables(5);
                const std::size_t z = 0;
                // y = 1 forces 2 (o0 + o1 + o2 + o3 + o4) = 5: no integer point, LP fine.
                m.add_constraint(2.0 * (o(z) + o(z + 1) + o(z + 2) + o(z + 3) + o(z + 4)) - 5.0 * y >= 0.0);
                m.add_constraint(2.0 * (o(z) + o(z + 1) + o(z + 2) + o(z + 3) + o(z + 4)) + 5.0 * y <= 10.0);
                m.set_objective_coefficient(y, 40);
                const int iy = m.native_id(y);
                if(variant == 2) {
                    m.add_constraint(1.0 * y == 1.0);  // now no integer point at all: both cuts are vacuously valid
                    return every_call({cut{{iy}, {1}, 'L', 0}, cut{{iy}, {1}, 'G', 1}});
                }
                if(variant == 0) return every_call({cut{{iy}, {1}, 'L', 0}});
                return cp::separator([iy, col, c](const std::vector<double> & p, const cp::add_fn & add) { add(cut{{iy}, {1}, 'L', 0}); (void)p; (void)col; (void)c; });
            }, 15);
        }, 45);
    }
}
int main(int argc, char ** argv) {
    const std::string w = argc > 1 ? argv[1] : "all";
    { M m; std::println(stderr, "release {}", vx::version(m)); }
    vx::quiet_flush = true;
    if(w == "red" || w == "all") reductions_cases();
    if(w == "dup" || w == "all") { equality_on_duplicates(); equality_on_parallel(); }
    if(w == "scale" || w == "all") for(const auto & s : specs) { scaled_dfj(s, 5, 26); }
    if(w == "near" || w == "all") for(const auto & s : specs) near_duplicates(s);
    if(w == "tight" || w == "all") for(const auto & s : specs) tight_cuts(s);
    if(w == "oneterm" || w == "all") for(const auto & s : specs) one_term(s);
    if(w == "prune" || w == "all") for(const auto & s : specs) pruned(s);
}
