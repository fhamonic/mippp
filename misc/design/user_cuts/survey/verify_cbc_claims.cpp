// Own experiments on claims B1, B3-B6, B9, B10. Dispatch on argv[1]; see main().
#include "verify_cbc_recipe.hpp"
using vc::M;
using std::chrono::duration;
using cp::raw_cut;

static std::vector<double> best_point(M & m) {
    const double * s = m.Cbc->bestSolution(m._solved_model());
    if(!s) s = m.Cbc->getColSolution(m._solved_model());
    return std::vector<double>(s, s + m.num_variables());
}
static double row_value(const raw_cut & c, const std::vector<double> & x) { return c.activity(x); }
static std::string st(M & m) { return cp::status_name(m.get_status()); }

// A random multidimensional knapsack over `cols` binaries, the first native columns,
// padded with rowless columns so that rows + cols >= 500 and 2.10 keeps its normal nodes.
struct mkp_model { int cols; std::vector<std::vector<double>> a; std::vector<double> b, c; };
static mkp_model build_mkp2(M & m, int rows, int cols, unsigned seed, int pad = 480, double tight = 0.5) {
    mkp_model o; o.cols = cols;
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> A(1, 100), R(0, 40);
    const auto C = static_cast<std::size_t>(cols);
    o.a.assign(static_cast<std::size_t>(rows), std::vector<double>(C));
    for(auto & r : o.a) for(auto & v : r) v = A(rng);
    o.c.resize(C);
    for(std::size_t j = 0; j < C; ++j) { double s = 0; for(auto & r : o.a) s += r[j]; o.c[j] = std::floor(s / rows + R(rng)); }
    auto x = m.add_binary_variables(C);
    const auto ids = std::views::iota(std::size_t{0}, C);
    m.set_maximization();
    m.set_objective(xsum(ids, [&](std::size_t j) { return o.c[j] * x(j); }));
    for(const auto & r : o.a) {
        double s = 0; for(double v : r) s += v;
        o.b.push_back(std::floor(s * tight));
        m.add_constraint(xsum(ids, [&r, &x](std::size_t j) { return r[j] * x(j); }) <= o.b.back());
    }
    if(pad > 0) m.add_variables(static_cast<std::size_t>(pad), {.obj_coef = 0, .lower_bound = 0, .upper_bound = 1});
    return o;
}

// ------------------------------------------------------------------- B1 ---
// x* = the known optimum. An invalid cut Sum_S x <= a - 0.01, S the support of x*, a the
// LP activity where it is added, removes x*. At the root arm it is added at the first
// call. In the below-root arm it is added at the first call whose box excludes x*, so
// local means x* survives and global means it dies. With heuristics off, every earlier
// node contains x* and none had x* integral, so x* is not yet an incumbent; checked.
static void cmd_b1(int seeds) {
    for(int seed = 1; seed <= seeds; ++seed) {
        const auto U = static_cast<unsigned>(seed);
        const int rows = 5, cols = 35;
        double OPT; std::vector<double> xs;
        { M m; build_mkp2(m, rows, cols, U); m.param("heuristics", "off"); m.set_time_limit(duration<double>(60)); cp::timed_solve(m);
          OPT = m.get_solution_value(); xs = best_point(m);
          std::println(stderr, "seed {} baseline status {} value {} nodes {}", seed, st(m), OPT, m.get_node_count()); }
        std::vector<int> S;
        for(int j = 0; j < cols; ++j) if(xs[static_cast<std::size_t>(j)] > 0.5) S.push_back(j);
        const auto make = [&](const std::vector<double> & x) { raw_cut c{S, std::vector<double>(S.size(), 1.0), 'L', 0.0}; c.rhs = c.activity(x) - 0.01; return c; };
        // root arm
        {
            M m; build_mkp2(m, rows, cols, U); m.param("heuristics", "off"); m.set_time_limit(duration<double>(60));
            std::optional<raw_cut> cut; long later = 0, later_viol = 0;
            m.attach([&](const std::vector<double> & x, const cp::add_fn & add) {
                if(!cut) { cut = make(x); add(*cut); }
                else { ++later; later_viol += cut->violation(x) > 1e-6; }
            });
            cp::timed_solve(m);
            const double v = m.get_solution_value();
            std::println(stderr, "  ROOT seed {}: {} value {} vs opt {} => {}; later calls {} violating {}; returned point violation {:.4g}; {}", seed, st(m), v, OPT, cf::same(v, OPT) ? "KEPT" : "LOST", later, later_viol, cut ? cut->violation(best_point(m)) : NAN, m.hook->c.line());
        }
        // below-root arm
        {
            M m; build_mkp2(m, rows, cols, U); m.param("heuristics", "off"); m.set_time_limit(duration<double>(60));
            struct { std::optional<raw_cut> cut; std::vector<std::tuple<int, double, double>> box; long sub = 0, sub_viol = 0, sib = 0, sib_viol = 0, before = 0; bool star_seen = false; int depth = -1; } t;
            auto & hookm = m;
            m.attach([](const auto &, const auto &) {});
            m.hook->observer = [&t, &make, &xs, &hookm, cols](void * osi, void * cuts, int, int) {
                const auto & A = *hookm.hook->abi;
                const int n = A.ncols(osi);
                if(n != hookm.hook->cols) return;
                const double * lo = A.lb(osi); const double * up = A.ub(osi); const double * s = A.sol(osi);
                std::vector<double> x(s, s + n);
                if(!t.cut) {
                    bool contains = true, integral_star = true;
                    for(int j = 0; j < cols; ++j) {
                        const auto J = static_cast<std::size_t>(j);
                        if(xs[J] < lo[j] - 1e-9 || xs[J] > up[j] + 1e-9) contains = false;
                        if(std::fabs(x[J] - xs[J]) > 1e-6) integral_star = false;
                    }
                    if(integral_star) t.star_seen = true;
                    if(contains) { ++t.before; return; }
                    raw_cut c = make(x);
                    A.add_row(cuts, static_cast<int>(c.ind.size()), c.ind.data(), c.val.data(), c.sense, c.rhs);
                    t.cut = c; t.depth = 0;
                    for(int j = 0; j < cols; ++j) if(lo[j] == up[j]) { t.box.emplace_back(j, lo[j], up[j]); ++t.depth; }
                    return;
                }
                bool in = true;
                for(auto [j, l, u] : t.box) if(lo[j] > u + 1e-9 || up[j] < l - 1e-9) in = false;
                const bool v = t.cut->violation(x) > 1e-6;
                if(in) { ++t.sub; t.sub_viol += v; } else { ++t.sib; t.sib_viol += v; }
            };
            cp::timed_solve(m);
            const double v = m.get_solution_value();
            std::println(stderr, "  BELOW seed {}: {} value {} vs opt {} => {}; x* seen before adding={} calls before={} fixed at add={}; subtree viol/calls {}/{}; elsewhere viol/calls {}/{}; returned point violation {:.4g}; {}", seed, st(m), v, OPT, cf::same(v, OPT) ? "KEPT" : "LOST", t.star_seen, t.before, t.depth, t.sub_viol, t.sub, t.sib_viol, t.sib, t.cut ? t.cut->violation(best_point(m)) : NAN, m.hook->c.line());
        }
    }
}

// ------------------------------------------------------------------- B5 ---
// Continuous column: the invalid cut z <= 1 added at the first call, on a model whose
// incumbent's continuous part is re-solved by checkSolution.
static void cmd_b5() {
    for(unsigned seed = 1; seed <= 5; ++seed) {
        const auto build = [&](M & m) {
            std::mt19937 rng(seed);
            std::uniform_int_distribution<int> A(5, 40), C(5, 60);
            const std::size_t K = 25;
            auto y = m.add_binary_variables(K);
            auto z = m.add_variables(1, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 50});
            std::vector<double> a(K), c(K);
            for(auto & v : a) v = A(rng);
            for(auto & v : c) v = C(rng);
            const auto ids = std::views::iota(std::size_t{0}, K);
            m.set_maximization();
            m.set_objective(xsum(ids, [&](std::size_t j) { return c[j] * y(j); }) + 3.0 * z(std::size_t{0}));
            double s = 0; for(double v : a) s += v;
            m.add_constraint(xsum(ids, [&](std::size_t j) { return a[j] * y(j); }) + z(std::size_t{0}) <= std::floor(s / 2));
            m.add_constraint(xsum(ids, [&](std::size_t j) { return c[j] * y(j); }) <= std::floor(s));
            return m.native_id(z(std::size_t{0}));
        };
        double base;
        { M m; build(m); m.param("heuristics", "off"); m.set_time_limit(duration<double>(30)); cp::timed_solve(m); base = m.get_solution_value(); }
        M m; const int zc = build(m); m.param("heuristics", "off"); m.set_time_limit(duration<double>(30));
        raw_cut cut{{zc}, {1.0}, 'L', 0.0}; bool added = false; long later = 0, viol = 0;
        m.attach([&](const std::vector<double> & x, const cp::add_fn & add) {
            cut.rhs = std::max(0.0, x[static_cast<std::size_t>(zc)] - 0.5);
            static thread_local double kept = 0;
            if(!added) { added = true; kept = cut.rhs; add(cut); }
            else { ++later; const double v = x[static_cast<std::size_t>(zc)] - kept; if(v > 1e-6) ++viol; }
            cut.rhs = kept;
        });
        cp::timed_solve(m);
        const auto x = best_point(m);
        std::println(stderr, "B5 seed {}: base {} | {} value {} z={} cut z<={} returned violation {:.4g}; later LP calls {} violating {} | {}", seed, base, st(m), m.get_solution_value(), x[static_cast<std::size_t>(zc)], cut.rhs, x[static_cast<std::size_t>(zc)] - cut.rhs, later, viol, m.hook->c.line());
    }
}

// ------------------------------------------------------------------- B6 ---
// A duplicate column reaches the C entry point with the sanitizer off; the catch is on.
static void cmd_b6() {
    M m; auto a = cp::build_atsp(m, 22, false, 3); m.set_time_limit(duration<double>(60));
    long tries = 0, errors = 0, ok = 0;
    m.attach([&](const std::vector<double> & x, const cp::add_fn & add) {
        for(auto c : a.separate(x)) {
            c.ind.push_back(c.ind.front()); c.val.push_back(0.0);  // x_j twice, valid
            ++tries;
            if(add(c)) ++errors; else ++ok;
        }
    });
    cp::timed_solve(m);
    std::println(stderr, "B6 duplicate column: {} value {} nodes {}: tries {} add ok {} add threw {} | {}", st(m), m.get_solution_value(), m.get_node_count(), tries, ok, errors, m.hook->c.line());
}

// ------------------------------------------------------------------ B10 ---
// Degree rows only: the assignment LP is integral at the root.
static void cmd_b10() {
    for(int n : {20, 26}) for(unsigned seed : {5u, 6u}) {
        const auto N = static_cast<std::size_t>(n);
        M m;
        std::mt19937 rng(seed);
        std::uniform_real_distribution<double> P(0, 100);
        auto x = m.add_binary_variables(N * N);
        const auto id = [N](std::size_t i, std::size_t j) { return i * N + j; };
        std::vector<double> d(N * N);
        for(auto & v : d) v = std::round(P(rng));
        const auto nodes = std::views::iota(std::size_t{0}, N);
        m.set_minimization();
        m.set_objective(xsum(nodes, [&](std::size_t i) { return xsum(nodes, [&, i](std::size_t j) { return d[id(i, j)] * x(id(i, j)); }); }));
        cp::atsp a; a.n = n; a.col.assign(N, std::vector<int>(N, -1));
        for(std::size_t i : nodes) {
            m.set_variable_upper_bound(x(id(i, i)), 0);
            m.add_constraint(xsum(nodes, [&, i](std::size_t j) { return 1.0 * x(id(i, j)); }) == 1);
            m.add_constraint(xsum(nodes, [&, i](std::size_t j) { return 1.0 * x(id(j, i)); }) == 1);
            for(std::size_t j : nodes) a.col[i][j] = m.native_id(x(id(i, j)));
        }
        m.set_time_limit(duration<double>(30));
        long seen = 0;
        m.attach([&](const std::vector<double> & p, const cp::add_fn & add) { ++seen; for(const auto & c : a.separate(p)) add(c); });
        cp::timed_solve(m);
        const auto sol = best_point(m);
        int cycles = 0; std::vector<char> done(N, 0);
        for(std::size_t i : nodes) if(!done[i]) { ++cycles; std::size_t v = i; while(!done[v]) { done[v] = 1; for(std::size_t j : nodes) if(sol[id(v, j)] > 0.5) { v = j; break; } } }
        std::println(stderr, "B10 n={} seed={}: {} value {} nodes {} cycles in the point {} | hook {}", n, seed, st(m), m.get_solution_value(), m.get_node_count(), cycles, m.hook->c.line());
    }
}

// ------------------------------------------------------------------- B3 ---
// A generalized assignment problem: a small gap, so reduced costs fix many columns once
// an incumbent exists. Cover cuts on the capacity rows, by original index.
struct gap_model { int J, K; std::vector<std::vector<int>> col; std::vector<std::vector<double>> a; std::vector<double> b; };
static gap_model build_gap(M & m, int J, int K, unsigned seed, double tight = 0.8) {
    gap_model g{J, K, {}, {}, {}};
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> C(10, 50), A(5, 25);
    const auto Js = static_cast<std::size_t>(J), Ks = static_cast<std::size_t>(K);
    auto x = m.add_binary_variables(Js * Ks);
    const auto id = [Ks](std::size_t i, std::size_t j) { return i * Ks + j; };
    std::vector<double> c(Js * Ks);
    g.a.assign(Js, std::vector<double>(Ks));
    g.col.assign(Js, std::vector<int>(Ks));
    for(auto & v : c) v = C(rng);
    for(auto & r : g.a) for(auto & v : r) v = A(rng);
    const auto Ji = std::views::iota(std::size_t{0}, Js);
    const auto Ki = std::views::iota(std::size_t{0}, Ks);
    m.set_minimization();
    m.set_objective(xsum(Ji, [&](std::size_t i) { return xsum(Ki, [&, i](std::size_t j) { return c[id(i, j)] * x(id(i, j)); }); }));
    for(std::size_t i : Ji) {
        m.add_constraint(xsum(Ki, [&, i](std::size_t j) { return 1.0 * x(id(i, j)); }) == 1);
        for(std::size_t j : Ki) g.col[i][j] = m.native_id(x(id(i, j)));
    }
    for(std::size_t j : Ki) {
        double s = 0; for(std::size_t i : Ji) s += g.a[i][j];
        g.b.push_back(std::floor(tight * s / K));
        m.add_constraint(xsum(Ji, [&, j](std::size_t i) { return g.a[i][j] * x(id(i, j)); }) <= g.b.back());
    }
    return g;
}
static cp::separator gap_covers(const gap_model & g) {
    return [g](const std::vector<double> & x, const cp::add_fn & add) {
        for(std::size_t j = 0; j < static_cast<std::size_t>(g.K); ++j) {
            std::vector<std::size_t> order(static_cast<std::size_t>(g.J));
            std::iota(order.begin(), order.end(), std::size_t{0});
            std::ranges::sort(order, [&](std::size_t l, std::size_t r) { return x[static_cast<std::size_t>(g.col[l][j])] > x[static_cast<std::size_t>(g.col[r][j])]; });
            double w = 0; raw_cut c{{}, {}, 'L', 0};
            for(std::size_t i : order) {
                w += g.a[i][j]; c.ind.push_back(g.col[i][j]); c.val.push_back(1);
                if(w > g.b[j]) { c.rhs = static_cast<double>(c.ind.size()) - 1; if(c.violation(x) > 1e-4) add(c); break; }
            }
        }
    };
}
// budgeted ATSP, complete through MTZ: a knapsack over random weights on the arcs
static void add_budget2(M & m, const cp::atsp & a, double frac, unsigned seed) {
    std::mt19937 rng(seed * 31u + 17u);
    std::uniform_int_distribution<int> W(1, 100);
    const auto N = static_cast<std::size_t>(a.n);
    std::vector<std::pair<int, double>> t; double total = 0;
    for(std::size_t i = 0; i < N; ++i) for(std::size_t j = 0; j < N; ++j) if(i != j) { const double w = W(rng); t.emplace_back(a.col[i][j], w); total += w; }
    const double b = std::floor(frac * total / static_cast<double>(a.n - 1));
    auto keys = std::views::iota(std::size_t{0}, t.size());
    m.add_constraint(xsum(keys, [&t](std::size_t k) { return t[k].second * mippp::model_variable<int, double>(t[k].first); }) <= b);
}
static void cmd_b3(const std::string & family, int s0, int s1, double limit, bool cuts) {
    for(int seed = s0; seed <= s1; ++seed) {
        const auto U = static_cast<unsigned>(seed);
        const auto build = [&](M & m) -> cp::separator {
            if(family == "gap") { auto g = build_gap(m, 40, 6, U, 0.85); return gap_covers(g); }
            if(family == "gapl") { auto g = build_gap(m, 60, 8, U, 0.85); return gap_covers(g); }
            if(family == "gaps") { auto g = build_gap(m, 30, 5, U, 0.85); return gap_covers(g); }
            if(family == "gapx") { auto g = build_gap(m, 100, 10, U, 0.9); return gap_covers(g); }
            if(family.starts_with("q")) {  // q<n>: budgeted ATSP; x(i,j)+x(i,k)<=1, implied by a degree row, valid and by original index
                const int n = std::atoi(family.c_str() + 1);
                auto a = cp::build_atsp(m, n, false, U); add_budget2(m, a, 0.72, U);
                return cp::separator([a, n](const std::vector<double> & x, const cp::add_fn & add) {
                    for(const auto & c : a.separate(x, 1)) add(c);
                    int count = 0;
                    for(int i = 0; i < n && count < 20; ++i) for(int j = 0; j < n && count < 20; ++j) if(j != i) for(int k = j + 1; k < n && count < 20; ++k) if(k != i) {
                        const int c1 = a.col[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)], c2 = a.col[static_cast<std::size_t>(i)][static_cast<std::size_t>(k)];
                        if(x[static_cast<std::size_t>(c1)] + x[static_cast<std::size_t>(c2)] > 1 + 1e-4) { add(raw_cut{{c1, c2}, {1.0, 1.0}, 'L', 1.0}); ++count; }
                    }
                });
            }
            if(family.starts_with("b")) {  // b<n>: budgeted ATSP, one DFJ per call
                auto a = cp::build_atsp(m, std::atoi(family.c_str() + 1), false, U); add_budget2(m, a, 0.72, U);
                return cp::separator([a](const std::vector<double> & x, const cp::add_fn & add) { for(const auto & c : a.separate(x, 1)) add(c); });
            }
            auto a = cp::build_atsp(m, std::atoi(family.c_str()), false, U);
            return cf::dfj<M>(a);
        };
        double base = NAN; std::string bs;
        if(cuts) { M m; build(m); m.set_time_limit(duration<double>(limit)); cp::timed_solve(m); base = m.get_solution_value(); bs = std::format("{} nodes {}", st(m), m.get_node_count()); }
        M m; auto sep = build(m); m.set_time_limit(duration<double>(limit));
        if(std::getenv("B3_VERBOSE")) m.set_verbose(true);
        const int limit_full = vc::env_int("B3_CUTLIMIT", -1);
        std::atomic<int> full_adds{0}, last_n{0};
        struct { long sub_calls = 0; int first_sub_rows = -1, last_full_rows = -1, sub_added = 0; } tr;
        m.attach(!cuts ? cp::separator([](const auto &, const auto &) {}) : cp::separator([&](const std::vector<double> & x, const cp::add_fn & add) {
            const bool full = last_n.load() == static_cast<int>(m.num_variables());
            if(!full) { sep(x, [&](const raw_cut & c) { ++tr.sub_added; return add(c); }); return; }
            if(limit_full >= 0 && full_adds.load() >= limit_full) return;
            sep(x, [&](const raw_cut & c) { if(limit_full >= 0 && full_adds.load() >= limit_full) return 0; const int e = add(c); ++full_adds; return e; });
        }));
        m.hook->observer = [&](void * osi, void *, int, int) {
            const int n = m.hook->abi->ncols(osi); last_n.store(n);
            if(n != m.hook->cols) { if(!tr.sub_calls++) tr.first_sub_rows = m.hook->abi->nrows(osi); }
            else tr.last_full_rows = m.hook->abi->nrows(osi);
        };
        const double t = cp::timed_solve(m);
        std::println(stderr, "B3 {} seed {}: base {} {} | {} value {} dual {} nodes {} t={:.1f} {} | model rows {} full-model cuts {} first sub-model rows {} sub adds {} | {}", family, seed, base, bs, st(m), m.get_solution_value(), m.get_dual_bound(), m.get_node_count(), t, (cuts && bs.starts_with("optimal") && mippp::is_a<mippp::status::optimal>(m.get_status()) && !cf::same(base, m.get_solution_value())) ? "DIFFERS" : "", m.num_constraints(), full_adds.load(), tr.first_sub_rows, tr.sub_added, m.hook->c.line());
    }
}

// ------------------------------------------------------------------- B4 ---
// No-op hook; how far down do the calls go. Only branching fixes a binary on the gadgets
// until the first incumbent, so the number of fixed binaries is the depth up to there.
static void cmd_b4() {
    for(int k : {8, 10, 12}) {
        M m; auto g = cp::build_gadgets(m, k, false);
        m.param("cuts", "off"); m.param("heuristics", "off");
        m.set_time_limit(duration<double>(60));
        m.attach([](const auto &, const auto &) {});
        std::map<int, long> levels; std::mutex lm;
        m.hook->observer = [&](void *, void *, int level, int) { std::lock_guard l(lm); levels[level]++; };
        const double t = cp::timed_solve(m);
        std::string hist; for(auto [f, v] : m.hook->c.by_fixed) hist += std::format(" {}:{}", f, v);
        std::string lv; for(auto [f, v] : levels) lv += std::format(" {}:{}", f, v);
        std::println(stderr, "B4 gadgets k={}: {} value {} nodes {} t={:.1f} calls {} | calls by #fixed binaries:{} | by level:{}", k, st(m), m.get_solution_value(), m.get_node_count(), t, m.hook->c.calls.load(), hist, lv);
    }
}

// ------------------------------------------------------------------- B9 ---
// Same parameters in both arms; only the registration of a hook that does nothing differs.
static void cmd_b9() {
    const auto run = [](const std::string & name, const std::function<void(M &)> & build, bool hook, long node_limit) {
        for(int rep = 0; rep < 2; ++rep) {
            M m; build(m); m.param("preprocess", "off");
            if(const char * e = std::getenv("B9_PASSTREE")) m.param("passTreeCuts", e);
            m.set_time_limit(duration<double>(60));
            if(node_limit > 0) m.set_node_limit(static_cast<std::size_t>(node_limit));
            if(hook) m.attach([](const auto &, const auto &) {});
            const double t = cp::timed_solve(m);
            std::println(stderr, "B9 {} {} rep{}: {} value {} dual {} nodes {} t={:.2f}{}", name, hook ? "NOOP" : "NONE", rep, st(m), m.get_solution_value(), m.get_dual_bound(), m.get_node_count(), t, hook ? std::format(" calls={}", m.hook->c.calls.load()) : "");
        }
    };
    for(unsigned seed : {7u, 8u, 9u}) {
        const auto mkp = [seed](M & m) { cp::build_mkp(m, 5, 100, seed); };
        run(std::format("mkp5x100 seed {} nodelimit 5000", seed), mkp, false, 5000); run(std::format("mkp5x100 seed {} nodelimit 5000", seed), mkp, true, 5000);
        const auto small = [seed](M & m) { cp::build_mkp(m, 5, 35, seed); };
        run(std::format("mkp5x35 seed {} to optimality", seed), small, false, 0); run(std::format("mkp5x35 seed {} to optimality", seed), small, true, 0);
        const auto at = [seed](M & m) { cp::build_atsp(m, 22, false, seed); };
        run(std::format("atsp22 seed {} to optimality", seed), at, false, 0); run(std::format("atsp22 seed {} to optimality", seed), at, true, 0);
    }
}

// ------------------------------------------------------------------- B2 ---
// C2's builder with a chosen size and seed; run under the recipe's toggles.
static void cmd_b2(int n, unsigned s0, unsigned s1) {
    cf::setup<M> B; B.attach = [](M & m, cp::separator sep) { m.attach(std::move(sep)); }; B.one_thread = [](M &) {}; B.limit = 120;
    for(unsigned seed = s0; seed <= s1; ++seed)
        cf::compare(B, std::format("B2 30 fixed columns first, n={} seed={}", n, seed), [=](M & m) {
            auto f = m.add_binary_variables(30);
            for(std::size_t i = 0; i < 30; ++i) m.add_constraint(1.0 * f(i) == 1.0);
            auto a = cp::build_atsp(m, n, false, seed);
            return cf::dfj<M>(a);
        });
}

// ------------------------------------------------------------------- B7 ---
static void cmd_b7(int n, double limit, bool with_cuts) {
    M m; auto a = cp::build_atsp(m, n, false, 5); m.set_time_limit(duration<double>(limit));
    auto s = std::make_shared<cp::stats>();
    cp::separator inner = with_cuts ? cp::separator([&a](const std::vector<double> & x, const cp::add_fn & add) { for(const auto & c : a.separate(x, 1)) add(c); }) : cp::separator([](const auto &, const auto &) {});
    m.attach(cp::watched(inner, s, {}, {}, {}, std::chrono::microseconds(20)));
    std::mutex om; std::set<void *> osis; std::map<int, long> by_cols;
    m.hook->observer = [&](void * osi, void *, int, int) { std::lock_guard l(om); osis.insert(osi); by_cols[m.hook->abi->ncols(osi)]++; };
    const double t = cp::timed_solve(m);
    std::string cols; for(auto [k, v] : by_cols) cols += std::format(" {}:{}", k, v);
    std::println(stderr, "B7 threads={} n={} cuts={}: {} t={:.1f} | distinct osi {} calls by #columns{} | {}", vc::tg().threads, n, with_cuts, cp::summary(m, t), t, osis.size(), cols, s->line());
}

// -------------------------------------------------------------------- E ---
// Nodes the hook never sees. Distinct boxes = distinct bound vectors over the integer
// columns at a call, so several passes at one node count once.
static void firing_one(const std::string & name, const std::function<void(M &)> & build, double limit, cp::separator sep = {}) {
    M m; build(m); m.set_time_limit(duration<double>(limit));
    m.attach(sep ? sep : cp::separator([](const auto &, const auto &) {}));
    std::set<std::uint64_t> boxes; long full = 0, frac = 0; std::map<int, long> levels;
    m.hook->observer = [&](void * osi, void *, int level, int) {
        levels[level]++;
        const auto & A = *m.hook->abi; const int n = A.ncols(osi);
        if(n != m.hook->cols) return;
        ++full;
        const double * lo = A.lb(osi); const double * up = A.ub(osi); const double * x = A.sol(osi);
        std::uint64_t h = 1469598103934665603ull; bool fr = false;
        for(int j = 0; j < n; ++j) if(m.hook->is_int[static_cast<std::size_t>(j)]) {
            for(double v : {lo[j], up[j]}) { h ^= static_cast<std::uint64_t>(std::llround(v * 16) + 1024); h *= 1099511628211ull; }
            if(std::fabs(x[j] - std::round(x[j])) > 1e-6) fr = true;
        }
        boxes.insert(h); if(fr) ++frac;
    };
    const double t = cp::timed_solve(m);
    const auto nodes = m.get_node_count();
    std::string lv; for(auto [k, v] : levels) lv += std::format(" {}:{}", k, v);
    std::println(stderr, "E {}: rows+cols={} {} value {} nodes={} t={:.1f} | calls={} added={} fractional_calls={} distinct_boxes={} | missed {:.1f}% of nodes | levels{}", name, m.num_variables() + m.num_constraints(), st(m), m.get_solution_value(), nodes, t, m.hook->c.calls.load(), m.hook->c.added.load(), frac, boxes.size(), nodes ? 100.0 * (1.0 - std::min(1.0, static_cast<double>(boxes.size()) / static_cast<double>(nodes))) : 0.0, lv.substr(0, 160));
}
static void cmd_e2(double limit) {
    // no-op hook and a real separator on a model whose search lasts, over and under 500
    for(const auto & [n, seed] : std::vector<std::pair<int, unsigned>>{{18, 2u}, {20, 2u}, {14, 2u}, {12, 2u}}) {
        firing_one(std::format("atsp n={} seed {} no-op", n, seed), [n, seed](M & m) { cp::build_atsp(m, n, false, seed); }, limit);
        cp::atsp a;
        firing_one(std::format("atsp n={} seed {} one DFJ per call", n, seed), [&a, n, seed](M & m) { a = cp::build_atsp(m, n, false, seed); }, limit,
            [&a](const std::vector<double> & x, const cp::add_fn & add) { for(const auto & c : a.separate(x, 1)) add(c); });
        auto count = std::make_shared<long>(0);
        firing_one(std::format("atsp n={} seed {} one DFJ per call after 25 calls", n, seed), [&a, n, seed](M & m) { a = cp::build_atsp(m, n, false, seed); }, limit,
            [&a, count](const std::vector<double> & x, const cp::add_fn & add) { if((*count)++ < 25) return; for(const auto & c : a.separate(x, 1)) add(c); });
    }
    firing_one("gadgets k=10, cuts off, no-op", [](M & m) { cp::build_gadgets(m, 10, false); m.param("cuts", "off"); }, limit);
}
// Edge models: no integer point (any cut is valid), no rows, an integer column with huge bounds.
static void cmd_edge() {
    {
        M m; auto x = m.add_binary_variables(4);
        m.set_maximization(); m.set_objective(1.0 * x(std::size_t{0}) + 1.0 * x(std::size_t{1}));
        m.add_constraint(2.0 * x(std::size_t{0}) + 2.0 * x(std::size_t{1}) + 2.0 * x(std::size_t{2}) + 2.0 * x(std::size_t{3}) == 3.0);
        m.set_time_limit(duration<double>(10));
        const int a = m.native_id(x(std::size_t{0})), b = m.native_id(x(std::size_t{1})), c = m.native_id(x(std::size_t{2})), d = m.native_id(x(std::size_t{3}));
        m.attach([=](const std::vector<double> & p, const cp::add_fn & add) {
            add(raw_cut{{a, b, c, d}, {1.0, 1.0, 1.0, 1.0}, 'G', 2.0});  // vacuously valid
            add(raw_cut{{a, b, c, d}, {1.0, 1.0, 1.0, 1.0}, 'L', 1.0});
            (void)p;
        });
        cp::timed_solve(m);
        std::println(stderr, "edge no integer point, contradicting valid cuts: {} | {}", st(m), m.hook->c.line());
    }
    {
        M m; auto x = m.add_binary_variables(4);
        m.set_maximization(); m.set_objective(1.0 * x(std::size_t{0}) + 1.0 * x(std::size_t{1}));
        m.set_time_limit(duration<double>(10));
        m.attach([](const auto &, const auto &) {});
        cp::timed_solve(m);
        std::println(stderr, "edge no rows: {} value {} | {}", st(m), m.get_solution_value(), m.hook->c.line());
    }
    {
        M m; auto x = m.add_integer_variable({.obj_coef = 1, .lower_bound = -1e6, .upper_bound = 1e6});
        auto y = m.add_integer_variable({.obj_coef = 1, .lower_bound = -1e6, .upper_bound = 1e6});
        m.set_maximization();
        m.add_constraint(2.0 * x + 2.0 * y <= 7.0);
        m.add_constraint(2.0 * x - 2.0 * y <= 3.0);
        m.set_time_limit(duration<double>(10));
        const int a = m.native_id(x), b = m.native_id(y);
        m.attach([=](const std::vector<double> &, const cp::add_fn & add) { add(raw_cut{{a, b}, {1.0, 1.0}, 'L', 3.0}); });
        cp::timed_solve(m);
        std::println(stderr, "edge general integers, a CG cut x+y<=3: {} value {} expected 3 | {}", st(m), m.get_solution_value(), m.hook->c.line());
    }
}
static void cmd_e3(double limit) {
    firing_one("atsp n=18 seed 2 no-op", [](M & m) { cp::build_atsp(m, 18, false, 2); }, limit);
    firing_one("atsp n=26 seed 5 no-op", [](M & m) { cp::build_atsp(m, 26, false, 5); }, limit);
    firing_one("mkp 5x100 seed 7 no-op", [](M & m) { cp::build_mkp(m, 5, 100, 7); }, limit);
    firing_one("mkp 5x35 seed 1 padded no-op", [](M & m) { build_mkp2(m, 5, 35, 1, 480); }, limit);
}
static void cmd_e(double limit) {
    for(unsigned seed : {1u, 2u, 3u}) {
        firing_one(std::format("mkp 5x35 seed {}", seed), [seed](M & m) { build_mkp2(m, 5, 35, seed, 0); }, limit);
        firing_one(std::format("mkp 5x35 seed {} padded to 520", seed), [seed](M & m) { build_mkp2(m, 5, 35, seed, 480); }, limit);
        firing_one(std::format("mkp 5x100 seed {}", seed), [seed](M & m) { build_mkp2(m, 5, 100, seed, 0); }, limit);
        firing_one(std::format("gap 30x5 seed {}", seed), [seed](M & m) { build_gap(m, 30, 5, seed, 0.85); }, limit);
    }
    for(int n : {12, 14, 16}) for(unsigned seed : {1u, 2u}) firing_one(std::format("atsp n={} seed {}", n, seed), [n, seed](M & m) { cp::build_atsp(m, n, false, seed); }, limit);
    firing_one("gadgets k=10", [](M & m) { cp::build_gadgets(m, 10, false); }, limit);
    firing_one("atsp n=18 seed 2 (over 500)", [](M & m) { cp::build_atsp(m, 18, false, 2); }, limit);
}

// ------------------------------------------- controls without a conformance case ---
// A hooked solve, then a solve with no hook, on the same model object.
static void cmd_sticky() {
    M m; auto a = cp::build_atsp(m, 22, false, 3); m.set_time_limit(duration<double>(60));
    m.attach(cf::dfj<M>(a));
    cp::timed_solve(m);
    const auto saved = m.hook;
    const long first = saved->c.calls.load();
    const auto first_summary = std::format("{} nodes {}", st(m), m.get_node_count());
    m.hook = nullptr;
    cp::timed_solve(m);
    std::println(stderr, "sticky: hooked solve {} calls {} | plain solve {} nodes {} calls during it {}", first_summary, first, st(m), m.get_node_count(), saved->c.calls.load() - first);
}
// A model with no integer column: solved in place on every release.
static void cmd_lp() {
    M m; auto x = m.add_variables(5, {.obj_coef = 1, .lower_bound = 0, .upper_bound = 1});
    m.set_maximization();
    m.add_constraint(xsum(std::views::iota(std::size_t{0}, std::size_t{5}), [&](std::size_t j) { return 1.0 * x(j); }) <= 2.5);
    m.attach([](const auto &, const auto &) {});
    cp::timed_solve(m); m.attach([](const auto &, const auto &) {}); cp::timed_solve(m);
    std::println(stderr, "lp-only, hook attached twice: {} value {} calls {}", st(m), m.get_solution_value(), m.hook->c.calls.load());
}
// A native-model registration on a release that solves on a copy.
static void cmd_how(double limit) {
    M m; const auto sep = [](M & mm) { build_mkp2(mm, 5, 35, 1, 480); }; sep(m); m.set_time_limit(duration<double>(limit));
    m.attach([](const auto &, const auto &) {});
    const double t = cp::timed_solve(m);
    std::println(stderr, "how often: {} t={:.2f} | {}", cp::summary(m, t), t, m.hook->c.line());
}

int main(int argc, char ** argv) {
    vc::banner();
    const std::string cmd = argc > 1 ? argv[1] : "";
    if(cmd == "b1") cmd_b1(argc > 2 ? std::atoi(argv[2]) : 5);
    else if(cmd == "b3") cmd_b3(argv[2], std::atoi(argv[3]), std::atoi(argv[4]), std::atof(argv[5]), argc > 6 && std::atoi(argv[6]));
    else if(cmd == "b4") cmd_b4();
    else if(cmd == "b9") cmd_b9();
    else if(cmd == "b2") { cmd_b2(std::atoi(argv[2]), static_cast<unsigned>(std::atoi(argv[3])), static_cast<unsigned>(std::atoi(argv[4]))); cf::finish(); }
    else if(cmd == "b7") cmd_b7(std::atoi(argv[2]), std::atof(argv[3]), argc > 4 && std::atoi(argv[4]));
    else if(cmd == "e") cmd_e(argc > 2 ? std::atof(argv[2]) : 30);
    else if(cmd == "e2") cmd_e2(argc > 2 ? std::atof(argv[2]) : 20);
    else if(cmd == "e3") cmd_e3(argc > 2 ? std::atof(argv[2]) : 20);
    else if(cmd == "edge") cmd_edge();
    else if(cmd == "sticky") cmd_sticky();
    else if(cmd == "lp") cmd_lp();
    else if(cmd == "how") cmd_how(argc > 2 ? std::atof(argv[2]) : 30);
    else if(cmd == "b5") cmd_b5();
    else if(cmd == "b6") cmd_b6();
    else if(cmd == "b10") cmd_b10();
    else { std::println(stderr, "unknown command"); return 2; }
    return 0;
}
