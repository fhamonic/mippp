#pragma once
// Second phase of the user-cut survey: one recipe per backend, the same conformance
// set everywhere. attach() must implement the backend's recommended recipe for
// "add_user_cut(c): c is valid for every integer feasible point of the model".
// Every case compares against the same model solved with no hook; cuts must never
// change the optimum, on any backend, whatever they do to the search.
#include "cut_common.hpp"

namespace cf {
using namespace cp;

// What the handle of a wrapper would do before the native call: fold duplicate
// columns, strip zeros, skip what is empty, and ignore a cut the point satisfies
// (several solvers spin forever on one at the root). Returns nullopt for a no-op.
inline std::optional<raw_cut> sanitize(const raw_cut & in, const std::vector<double> & x, const bool drop_satisfied = true, const double tol = 1e-9) {
    std::map<int, double> folded;
    for(std::size_t k = 0; k < in.ind.size(); ++k) folded[in.ind[k]] += in.val[k];
    raw_cut out{{}, {}, in.sense, in.rhs};
    for(auto [j, v] : folded) if(v != 0.0) { out.ind.push_back(j); out.val.push_back(v); }
    if(out.ind.empty()) return std::nullopt;
    if(drop_satisfied && out.violation(x) <= tol) return std::nullopt;
    return out;
}

struct verdict { std::string name; bool pass; std::string detail; };
inline std::vector<verdict> & verdicts() { static std::vector<verdict> v; return v; }
inline void record(std::string name, bool pass, std::string detail) {
    std::println(stderr, "{} {}: {}", pass ? "PASS" : "FAIL", name, detail);
    verdicts().push_back({std::move(name), pass, std::move(detail)});
}
template <class M>
double value_of(M & m) { try { return m.get_solution_value(); } catch(const std::exception &) { return NAN; } }
inline bool same(double a, double b) { return std::fabs(a - b) <= 1e-6 * (1 + std::fabs(a)); }

template <class M>
struct setup {
    std::function<void(M &, separator)> attach;  // the recipe under test
    std::function<void(M &)> one_thread;
    std::function<void(M &, double)> relax_rhs;  // unused by most: see case P
    std::vector<int> atsp_sizes{12, 16, 20};
    std::vector<unsigned> seeds{1, 2, 3};
    double limit = 60;
    int stop_size = 20;        // C8: an instance that a stop at first_limit really interrupts
    double first_limit = 0.02;
    int fuzz_seeds = 20;
};

// Solves twice, no hook then cuts, and compares. Model builders return the separator.
template <class M, class Build>
void compare(const setup<M> & B, const std::string & name, Build build, const bool threads_default = false) {
    using std::chrono::duration;
    double base;
    long base_nodes = -1;
    {
        M m; if(!threads_default) B.one_thread(m);
        auto keep = build(m); m.set_time_limit(duration<double>(B.limit));
        const double t = timed_solve(m);
        base = value_of(m);
        if(!is_a<status::optimal>(m.get_status())) { record(name, false, std::format("NO BASELINE: {}", summary(m, t))); return; }
        if constexpr(has_node_count<M>) base_nodes = static_cast<long>(m.get_node_count());
    }
    M m; if(!threads_default) B.one_thread(m);
    auto sep = build(m); m.set_time_limit(duration<double>(B.limit));
    auto s = std::make_shared<stats>();
    B.attach(m, watched(sep, s));
    const double t = timed_solve(m);
    const double v = value_of(m);
    const bool opt = is_a<status::optimal>(m.get_status());
    record(name, opt && same(v, base), std::format("baseline value {} nodes {} | {} | {}", base, base_nodes, summary(m, t), s->line()));
}

// A separator over the DFJ family; columns chosen by `writer`.
template <class M>
separator dfj(const atsp & a) { return [a](const std::vector<double> & x, const add_fn & add) { for(const auto & c : a.separate(x)) add(c); }; }

template <class M>
void run(const setup<M> & B) {
    using std::chrono::duration;
    for(int n : B.atsp_sizes) for(unsigned seed : B.seeds) {
        // C1 the plain use, under every default the solver has
        compare(B, std::format("C1 atsp n={} seed={}", n, seed), [=](M & m) { auto a = build_atsp(m, n, false, seed); return dfj<M>(a); });
    }
    for(unsigned seed : B.seeds) {
        // C2 columns before the arcs that presolve removes: shifts every presolved index
        compare(B, std::format("C2 fixed columns first seed={}", seed), [=](M & m) {
            auto f = m.add_binary_variables(30);
            for(std::size_t i = 0; i < 30; ++i) m.add_constraint(1.0 * f(i) == 1.0);
            auto a = build_atsp(m, B.atsp_sizes.front(), false, seed);
            return dfj<M>(a);
        });
        // C3 complements presolve eliminates, cuts written on them
        compare(B, std::format("C3 eliminated complements seed={}", seed), [=](M & m) { auto a = build_atsp(m, B.atsp_sizes.front(), true, seed); return dfj<M>(a); });
    }
    // C4 parallel columns: each arc is a + b, which presolve merges. A cut on a + b is
    // valid and has to survive the merge, or be dropped without harm.
    for(unsigned seed : B.seeds) compare(B, std::format("C4 parallel columns seed={}", seed), [=](M & m) {
        const int n = B.atsp_sizes.front();
        const auto N = static_cast<std::size_t>(n);
        std::mt19937 rng(seed);
        std::uniform_real_distribution<double> P(0, 100);
        std::vector<std::vector<double>> d(N, std::vector<double>(N));
        for(auto & r : d) for(auto & v : r) v = std::round(P(rng));
        auto a = m.add_binary_variables(N * N);
        auto b = m.add_binary_variables(N * N);
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
        atsp view; view.n = n; view.col.assign(N, std::vector<int>(N, -1));
        std::vector<std::vector<int>> ca(N, std::vector<int>(N)), cb(N, std::vector<int>(N));
        for(std::size_t i : nodes) for(std::size_t j : nodes) { ca[i][j] = m.native_id(a(id(i, j))); cb[i][j] = m.native_id(b(id(i, j))); }
        return separator([n, N, ca, cb](const std::vector<double> & x, const add_fn & add) {
            // Fold a + b into a scratch column vector, separate on it, write back on a + b.
            atsp s; s.n = n; s.col.assign(N, std::vector<int>(N, -1));
            std::vector<double> sum; int next = 0;
            for(std::size_t i = 0; i < N; ++i) for(std::size_t j = 0; j < N; ++j) if(i != j) {
                s.col[i][j] = next++; sum.push_back(x[static_cast<std::size_t>(ca[i][j])] + x[static_cast<std::size_t>(cb[i][j])]);
            }
            for(const auto & c : s.separate(sum)) {
                raw_cut out; out.sense = c.sense; out.rhs = c.rhs;
                std::vector<std::pair<std::size_t, std::size_t>> arcs;
                for(std::size_t i = 0; i < N; ++i) for(std::size_t j = 0; j < N; ++j) if(i != j)
                    if(std::ranges::find(c.ind, s.col[i][j]) != c.ind.end()) arcs.emplace_back(i, j);
                for(auto [i, j] : arcs) { out.ind.push_back(ca[i][j]); out.val.push_back(1); out.ind.push_back(cb[i][j]); out.val.push_back(1); }
                add(out);
            }
        });
    });
    // C5 maximize with an offset, knapsack covers: sign and offset of nothing but the
    // optimum; a cover found on the LP point is valid for the whole model.
    for(unsigned seed : B.seeds) compare(B, std::format("C5 knapsack covers, maximize, offset seed={}", seed), [=](M & m) {
        const std::size_t cols = 40, rows = 5;
        std::mt19937 rng(seed);
        std::uniform_int_distribution<int> A(1, 100), R(0, 50);
        std::vector<std::vector<double>> a(rows, std::vector<double>(cols));
        for(auto & r : a) for(auto & v : r) v = A(rng);
        std::vector<double> c(cols), cap(rows);
        for(std::size_t j = 0; j < cols; ++j) { double s = 0; for(auto & r : a) s += r[j]; c[j] = std::floor(s / rows + R(rng)); }
        for(std::size_t i = 0; i < rows; ++i) { double s = 0; for(double v : a[i]) s += v; cap[i] = std::floor(s / 2); }
        auto x = m.add_binary_variables(cols);
        const auto ids = std::views::iota(std::size_t{0}, cols);
        m.set_maximization();
        m.set_objective(xsum(ids, [&](std::size_t j) { return c[j] * x(j); }) + 100.0);
        for(std::size_t i = 0; i < rows; ++i) m.add_constraint(xsum(ids, [&, i](std::size_t j) { return a[i][j] * x(j); }) <= cap[i]);
        std::vector<int> col;
        for(std::size_t j = 0; j < cols; ++j) col.push_back(m.native_id(x(j)));
        return separator([a, cap, col](const std::vector<double> & p, const add_fn & add) {
            for(std::size_t i = 0; i < a.size(); ++i) {
                std::vector<std::size_t> order(col.size());
                for(std::size_t j = 0; j < order.size(); ++j) order[j] = j;
                std::ranges::sort(order, [&](std::size_t l, std::size_t r) { return p[static_cast<std::size_t>(col[l])] > p[static_cast<std::size_t>(col[r])]; });
                double w = 0; raw_cut cut{{}, {}, 'L', 0};
                for(std::size_t j : order) {
                    w += a[i][j]; cut.ind.push_back(col[j]); cut.val.push_back(1);
                    if(w > cap[i]) { cut.rhs = static_cast<double>(cut.ind.size()) - 1; if(cut.violation(p) > 1e-4) add(cut); break; }
                }
            }
        });
    });
    // C6 a separator that re-offers everything it ever found, at every call: the usual
    // pattern for a stateless or lazy separator. It must end; sanitize drops what holds.
    compare(B, "C6 re-offers all its cuts at every call", [=](M & m) {
        auto a = build_atsp(m, B.atsp_sizes.front(), false, 7);
        auto all = std::make_shared<std::vector<raw_cut>>();
        auto mx = std::make_shared<std::mutex>();
        return separator([a, all, mx](const std::vector<double> & x, const add_fn & add) {
            std::lock_guard l(*mx);
            for(const auto & c : a.separate(x)) all->push_back(c);
            for(const auto & c : *all) add(c);
        });
    });
    // C7 odd but legal terms: x - x, a one-term cut, an equality, and something empty.
    compare(B, "C7 cancelling terms, one term, equality, empty", [=](M & m) {
        auto a = build_atsp(m, B.atsp_sizes.front(), false, 8);
        const int c01 = a.col[0][1], c02 = a.col[0][2];
        return separator([a, c01, c02](const std::vector<double> & x, const add_fn & add) {
            for(auto c : a.separate(x)) { c.ind.push_back(c01); c.val.push_back(1); c.ind.push_back(c01); c.val.push_back(-1); add(c); }
            add(raw_cut{{c02}, {1.0}, 'L', 1.0});                     // implied by the bound
            add(raw_cut{{c01, c02}, {1.0, 1.0}, 'L', 1.0});           // implied by a degree row
            add(raw_cut{{c01, c01}, {1.0, -1.0}, 'L', 0.0});          // folds to nothing
            add(raw_cut{{}, {}, 'L', 1.0});
        });
    });
    // C8 stop, then resume: nothing may be lost by re-solving an unchanged model.
    {
        M base; B.one_thread(base); build_atsp(base, B.stop_size, false, 9); base.set_time_limit(duration<double>(B.limit));
        timed_solve(base);
        const double want = value_of(base);
        M m; B.one_thread(m); auto a = build_atsp(m, B.stop_size, false, 9);
        auto s = std::make_shared<stats>();
        B.attach(m, watched(dfj<M>(a), s));
        m.set_time_limit(duration<double>(B.first_limit)); const double t1 = timed_solve(m);
        std::string first = summary(m, t1);
        if(is_a<status::optimal>(m.get_status())) first = "NO POWER, the first solve finished: " + first;
        B.attach(m, watched(dfj<M>(a), s));
        m.set_time_limit(duration<double>(B.limit)); const double t2 = timed_solve(m);
        record("C8 stop at 0.02 s, then re-solve", same(value_of(m), want) && is_a<status::optimal>(m.get_status()), std::format("baseline {} | first {} | then {} | {}", want, first, summary(m, t2), s->line()));
    }
    // C10 fuzz: Chvatal-Gomory cuts and covers on random binary models.
    // Valid for every integer point by construction (x >= 0: floor(c) x <= c x), so a
    // lost optimum, a wrong status or a crash is the solver's or the recipe's.
    for(int seed = 1; seed <= B.fuzz_seeds; ++seed) compare(B, std::format("C10 CG and cover fuzz seed={}", seed), [=](M & m) {
        const std::size_t cols = 35, rows = 15;
        std::mt19937 rng(static_cast<unsigned>(seed));
        std::uniform_int_distribution<int> A(-5, 20), C(1, 40);
        std::vector<std::vector<double>> a(rows, std::vector<double>(cols));
        for(auto & r : a) for(auto & v : r) v = A(rng);
        std::vector<double> b(rows), c(cols);
        for(std::size_t i = 0; i < rows; ++i) { double t = 0; for(double v : a[i]) t += std::max(v, 0.0); b[i] = std::floor(t * 0.4); }
        for(auto & v : c) v = C(rng);
        auto x = m.add_binary_variables(cols);
        const auto ids = std::views::iota(std::size_t{0}, cols);
        m.set_maximization();
        m.set_objective(xsum(ids, [&](std::size_t j) { return c[j] * x(j); }));
        for(std::size_t i = 0; i < rows; ++i) m.add_constraint(xsum(ids, [&, i](std::size_t j) { return a[i][j] * x(j); }) <= b[i]);
        std::vector<int> col;
        for(std::size_t j = 0; j < cols; ++j) col.push_back(m.native_id(x(j)));
        auto gen = std::make_shared<std::mt19937>(static_cast<unsigned>(seed) * 977u);
        auto mx = std::make_shared<std::mutex>();
        return separator([a, b, col, gen, mx](const std::vector<double> & p, const add_fn & add) {
            std::lock_guard l(*mx);
            // Multipliers k/d on rows tight at the point: the only ones that can be
            // violated, since a slack row adds its slack to what the rounding must beat.
            std::vector<std::size_t> tight;
            for(std::size_t i = 0; i < a.size(); ++i) {
                double act = 0;
                for(std::size_t j = 0; j < col.size(); ++j) act += a[i][j] * p[static_cast<std::size_t>(col[j])];
                if(act >= b[i] - 1e-6) tight.push_back(i);
            }
            if(tight.empty()) return;
            const int denominators[] = {2, 3, 4, 5, 7, 11};
            std::uniform_int_distribution<std::size_t> D(0, std::size(denominators) - 1);
            for(int attempt = 0, added = 0; attempt < 300 && added < 3; ++attempt) {
                const int d = denominators[D(*gen)];
                std::uniform_int_distribution<int> K(0, d - 1);
                std::vector<double> lambda(a.size(), 0.0);
                for(std::size_t i : tight) lambda[i] = static_cast<double>(K(*gen)) / d;
                raw_cut cut{{}, {}, 'L', 0};
                double rhs = 0;
                for(std::size_t i = 0; i < a.size(); ++i) rhs += lambda[i] * b[i];
                cut.rhs = std::floor(rhs + 1e-9);
                for(std::size_t j = 0; j < col.size(); ++j) {
                    double t = 0;
                    for(std::size_t i = 0; i < a.size(); ++i) t += lambda[i] * a[i][j];
                    cut.ind.push_back(col[j]); cut.val.push_back(std::floor(t + 1e-9));
                }
                if(cut.violation(p) > 1e-4) { add(cut); ++added; }
            }
            // Covers of a row with negative coefficients: S with all x_S = 1 cannot be
            // feasible once the row's best case exceeds its rhs, so Sum_S x <= |S| - 1.
            for(std::size_t i : tight) {
                std::vector<std::size_t> order;
                double best = 0;
                for(std::size_t j = 0; j < col.size(); ++j) { if(a[i][j] > 0) order.push_back(j); else best += a[i][j]; }
                std::ranges::sort(order, [&](std::size_t l, std::size_t r) { return p[static_cast<std::size_t>(col[l])] > p[static_cast<std::size_t>(col[r])]; });
                raw_cut cover{{}, {}, 'L', 0};
                for(std::size_t j : order) {
                    best += a[i][j]; cover.ind.push_back(col[j]); cover.val.push_back(1);
                    if(best > b[i] + 1e-9) { cover.rhs = static_cast<double>(cover.ind.size()) - 1; if(cover.violation(p) > 1e-4) add(cover); break; }
                }
            }
        });
    });
    // C9 default threads, repeated: a cut from one thread must not hurt another.
    for(int rep = 0; rep < 5; ++rep) compare(B, std::format("C9 default threads rep={}", rep), [=](M & m) { auto a = build_atsp(m, B.atsp_sizes.back(), false, 10); return dfj<M>(a); }, true);
}

inline int finish() {
    int fails = 0;
    for(const auto & v : verdicts()) fails += !v.pass;
    std::println(stderr, "== {} cases, {} failed", verdicts().size(), fails);
    return fails ? 1 : 0;
}
}  // namespace cf
