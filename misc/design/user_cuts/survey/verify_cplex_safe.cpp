// D: try to break "any valid cut is safe under the recipe". Every cut below is valid for every integer feasible
// point (by construction, or by enumeration of the feasible set in the fuzz).
#include "verify_cplex_common.hpp"
using namespace cp;
using std::chrono::duration;
static cf::setup<M> mkB(double limit = 15, recipe R = {}) {
    cf::setup<M> B;
    if(std::getenv("RECIPE_GUARD")) R.same_point_guard = true;
    B.attach = [R](M & m, separator sep) { vattach(m, std::move(sep), R); };
    B.one_thread = [](M & m) { m.set_thread_limit(1); };
    B.limit = limit;
    return B;
}
template <class Build>
static void cmp(const cf::setup<M> & B, const std::string & name, Build build, const bool threads_default = false) {
    cf::compare(B, name, build, threads_default);
    if(!vkeep().empty()) std::println(stderr, "    [hook] {}", vkeep().back()->line());
}
static std::vector<double> objective_of(M & m) {
    auto [env, lp] = m.native_model();
    const int n = static_cast<int>(m.num_variables());
    std::vector<double> c(static_cast<std::size_t>(n));
    m.native_api()._check(env, m.native_api().getobj(env, lp, c.data(), 0, n - 1));
    return c;
}
// T1: a constant valid cut re-offered every call, violated by v at the first point.
static void tol() {
    const auto B = mkB(10);
    const std::vector<double> vs = {3e-9, 1e-8, 1e-7, 3e-7, 1e-6, 3e-6, 1e-5, 1e-4, 1e-3, 1e-2};
    for(const int settings : {0, 1}) for(const auto & [iname, rows, cols, seed] : std::initializer_list<std::tuple<const char *, int, int, unsigned>>{{"mkp5x60s3", 5, 60, 3}, {"mkp5x100s3", 5, 100, 3}}) for(const double v : vs) {
        cmp(B, std::format("T1 floor-v cut, v={:g}, {}, {}", v, iname, settings ? "root LP frozen (MIR on, EACHCUTLIM=0)" : "defaults"), [=](M & m) {
            build_mkp(m, rows, cols, seed);
            if(settings) { all_families_off(m); vparam(m, CPX_PARAM_MIRCUTS, 2); vparam(m, CPX_PARAM_EACHCUTLIM, 0); }
            const auto c = objective_of(m);
            auto init = std::make_shared<std::optional<double>>();
            auto mx = std::make_shared<std::mutex>();
            return separator([=](const std::vector<double> & x, const add_fn & add) {
                raw_cut cut{{}, {}, 'L', 0}; double act = 0;
                for(std::size_t j = 0; j < c.size(); ++j) if(c[j] != 0) { cut.ind.push_back(static_cast<int>(j)); cut.val.push_back(c[j]); act += c[j] * x[j]; }
                { std::lock_guard l(*mx); if(!*init) *init = act - v; cut.rhs = **init; }
                add(cut);
            });
        });
    }
}
// T5: DFJ cuts scaled by S, fresh only and re-offered every call.
static void scale() {
    const auto B = mkB(10);
    for(const bool reoffer : {false, true}) for(const double S : {1e-9, 1e-8, 1e-7, 1e-6, 1e-5, 1e-4, 1e-3, 1e-2, 1e2, 1e3, 1e4, 1e6, 1e8, 1e9, 1e12}) for(const auto & [n, seed] : std::initializer_list<std::pair<int, unsigned>>{{26, 5}, {22, 1}}) {
        cmp(B, std::format("T5 DFJ x {:g} {} atsp{}s{}", S, reoffer ? "re-offered" : "fresh", n, seed), [=](M & m) {
            auto a = build_atsp(m, n, false, seed);
            auto all = std::make_shared<std::vector<raw_cut>>();
            auto mx = std::make_shared<std::mutex>();
            return separator([=](const std::vector<double> & x, const add_fn & add) {
                std::lock_guard l(*mx);
                for(auto c : a.separate(x)) {
                    for(auto & w : c.val) w *= S;
                    c.rhs *= S;
                    if(reoffer) all->push_back(c); else add(c);
                }
                if(reoffer) for(const auto & c : *all) add(c);
            });
        });
    }
}
// T2: tight at the optimum, c'x <= opt on a max, c'x >= opt on a min, added whenever violated.
static void tight() {
    const auto B = mkB(20);
    for(const auto & [iname, rows, cols, seed] : std::initializer_list<std::tuple<const char *, int, int, unsigned>>{{"mkp5x60s3", 5, 60, 3}, {"mkp5x40s1", 5, 40, 1}, {"mkp5x40s2", 5, 40, 2}}) {
        double opt; { M m; m.set_thread_limit(1); build_mkp(m, rows, cols, seed); m.set_time_limit(duration<double>(60)); timed_solve(m); opt = m.get_solution_value(); }
        for(const double eps : {0.0, 1e-9, -0.0}) cmp(B, std::format("T2 c'x <= opt+{:g} {} opt={}", eps, iname, opt), [=](M & m) {
            build_mkp(m, rows, cols, seed);
            const auto c = objective_of(m);
            return separator([=](const std::vector<double> & x, const add_fn & add) {
                raw_cut cut{{}, {}, 'L', opt + eps};
                for(std::size_t j = 0; j < c.size(); ++j) if(c[j] != 0) { cut.ind.push_back(static_cast<int>(j)); cut.val.push_back(c[j]); }
                add(cut);
            });
        });
    }
    for(const auto & [n, seed] : std::initializer_list<std::pair<int, unsigned>>{{26, 5}, {22, 1}, {31, 11}}) {
        double opt; { M m; m.set_thread_limit(1); build_atsp(m, n, false, seed); m.set_time_limit(duration<double>(60)); timed_solve(m); opt = m.get_solution_value(); }
        cmp(B, std::format("T2 c'x >= opt atsp{}s{} opt={}", n, seed, opt), [=](M & m) {
            auto a = build_atsp(m, n, false, seed);
            return separator([=](const std::vector<double> & x, const add_fn & add) {
                raw_cut cut{{}, {}, 'G', opt};
                for(int i = 0; i < n; ++i) for(int j = 0; j < n; ++j) if(i != j) { cut.ind.push_back(a.col[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]); cut.val.push_back(a.d[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]); }
                add(cut);
            });
        });
        // the same with DFJ alongside: the bound closes at once, nodes about to be pruned get cuts
        cmp(B, std::format("T3 c'x >= opt + DFJ at every call atsp{}s{}", n, seed), [=](M & m) {
            auto a = build_atsp(m, n, false, seed);
            return separator([=](const std::vector<double> & x, const add_fn & add) {
                raw_cut cut{{}, {}, 'G', opt};
                for(int i = 0; i < n; ++i) for(int j = 0; j < n; ++j) if(i != j) { cut.ind.push_back(a.col[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]); cut.val.push_back(a.d[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]); }
                add(cut);
                for(const auto & c : a.separate(x)) add(c);
            });
        });
    }
}
// T6: near duplicates. A DFJ cut, then K copies: coefficients 1+k e (weaker, x >= 0), rhs 1-k e.
static void near() {
    const auto B = mkB(20);
    for(const double e : {1e-14, 1e-12, 1e-10, 1e-8, 1e-6}) for(const int K : {2, 20, 200}) {
        cmp(B, std::format("T6 near-duplicates e={:g} K={} atsp26s5", e, K), [=](M & m) {
            auto a = build_atsp(m, 26, false, 5);
            return separator([=](const std::vector<double> & x, const add_fn & add) {
                for(const auto & c : a.separate(x)) {
                    add(c);
                    for(int k = 1; k <= K; ++k) { auto d = c; for(auto & w : d.val) w *= 1 + k * e; d.rhs *= 1 - k * e; add(d); }
                }
            });
        });
    }
    for(const int K : {1000, 10000}) cmp(B, std::format("T6 identical cut {} times in one call atsp26s5", K), [=](M & m) {
        auto a = build_atsp(m, 26, false, 5);
        return separator([=](const std::vector<double> & x, const add_fn & add) {
            for(const auto & c : a.separate(x, 1)) for(int k = 0; k < K; ++k) add(c);
        });
    });
}
// T4: one-term cuts and equalities that hold only at integer points, by parity, so that the LP really violates them.
//   2(a0+a1+a2+a3) + 3w == 4  => w == 0 and Sum a == 2
//   2(b0+b1+b2+b3) + 3z == 9, z integer in [0,3] => z in {1,3}, so z >= 1
static void oneterm() {
    const auto B = mkB(10);
    struct ids { int w, z; std::vector<int> a; };
    const auto build = [](M & m, const unsigned seed) {
        const std::size_t C = 30;
        std::mt19937 rng(seed);
        std::uniform_int_distribution<int> A(1, 40), P(5, 60);
        auto x = m.add_binary_variables(C);
        auto as = m.add_binary_variables(4);
        auto bs = m.add_binary_variables(4);
        auto zz = m.add_integer_variables(1, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 3});
        auto ww = m.add_binary_variables(1);
        std::vector<double> c(C); for(auto & v : c) v = P(rng);
        const auto I = std::views::iota(std::size_t{0}, C);
        const auto F = std::views::iota(std::size_t{0}, std::size_t{4});
        m.set_maximization();
        m.set_objective(xsum(I, [&](std::size_t j) { return c[j] * x(j); }) - 400.0 * zz(std::size_t{0}) + 400.0 * ww(std::size_t{0}));
        for(int r = 0; r < 3; ++r) {
            std::vector<double> a(C); double t = 0; for(auto & v : a) { v = A(rng); t += v; }
            m.add_constraint(xsum(I, [&](std::size_t j) { return a[j] * x(j); }) <= std::floor(t * 0.4));
        }
        m.add_constraint(xsum(F, [&](std::size_t j) { return 2.0 * as(j); }) + 3.0 * ww(std::size_t{0}) == 4);
        m.add_constraint(xsum(F, [&](std::size_t j) { return 2.0 * bs(j); }) + 3.0 * zz(std::size_t{0}) == 9);
        ids d; d.w = m.native_id(ww(std::size_t{0})); d.z = m.native_id(zz(std::size_t{0}));
        for(std::size_t j = 0; j < 4; ++j) d.a.push_back(m.native_id(as(j)));
        return d;
    };
    struct cs { const char * name; std::function<std::vector<raw_cut>(const ids &)> f; };
    const std::vector<cs> cases = {
        {"w <= 0", [](const ids & d) { return std::vector<raw_cut>{raw_cut{{d.w}, {1.0}, 'L', 0.0}}; }},
        {"w == 0", [](const ids & d) { return std::vector<raw_cut>{raw_cut{{d.w}, {1.0}, 'E', 0.0}}; }},
        {"-w >= 0", [](const ids & d) { return std::vector<raw_cut>{raw_cut{{d.w}, {-1.0}, 'G', 0.0}}; }},
        {"1e9 w <= 0", [](const ids & d) { return std::vector<raw_cut>{raw_cut{{d.w}, {1e9}, 'L', 0.0}}; }},
        {"1e-6 w <= 0", [](const ids & d) { return std::vector<raw_cut>{raw_cut{{d.w}, {1e-6}, 'L', 0.0}}; }},
        {"1e-7 w <= 0", [](const ids & d) { return std::vector<raw_cut>{raw_cut{{d.w}, {1e-7}, 'L', 0.0}}; }},
        {"1e-8 w <= 0", [](const ids & d) { return std::vector<raw_cut>{raw_cut{{d.w}, {1e-8}, 'L', 0.0}}; }},
        {"z >= 1", [](const ids & d) { return std::vector<raw_cut>{raw_cut{{d.z}, {1.0}, 'G', 1.0}}; }},
        {"3 z >= 3", [](const ids & d) { return std::vector<raw_cut>{raw_cut{{d.z}, {3.0}, 'G', 3.0}}; }},
        {"z >= 0.5", [](const ids & d) { return std::vector<raw_cut>{raw_cut{{d.z}, {1.0}, 'G', 0.5}}; }},
        {"1e9 z >= 1e9", [](const ids & d) { return std::vector<raw_cut>{raw_cut{{d.z}, {1e9}, 'G', 1e9}}; }},
        {"1e-7 z >= 1e-7", [](const ids & d) { return std::vector<raw_cut>{raw_cut{{d.z}, {1e-7}, 'G', 1e-7}}; }},
        {"Sum a == 2 (only at integer points)", [](const ids & d) { return std::vector<raw_cut>{raw_cut{d.a, {1.0, 1.0, 1.0, 1.0}, 'E', 2.0}}; }},
        {"Sum a >= 2", [](const ids & d) { return std::vector<raw_cut>{raw_cut{d.a, {1.0, 1.0, 1.0, 1.0}, 'G', 2.0}}; }},
        {"all together", [](const ids & d) { return std::vector<raw_cut>{raw_cut{{d.w}, {1.0}, 'L', 0.0}, raw_cut{{d.z}, {1.0}, 'G', 1.0}, raw_cut{d.a, {1.0, 1.0, 1.0, 1.0}, 'E', 2.0}}; }},
    };
    for(const unsigned seed : {1u, 2u, 3u}) for(const auto & cc : cases) for(const int st : {0, 1}) {
        cmp(B, std::format("T4 {} seed={} {}", cc.name, seed, st ? "no presolve, MIR+Gomory only" : "defaults"), [=](M & m) {
            const auto d = build(m, seed);
            if(st) { scope_settings(m); vparam(m, CPX_PARAM_MIRCUTS, 2); vparam(m, CPX_PARAM_FRACCUTS, 2); }
            const auto cuts = cc.f(d);
            return separator([cuts](const std::vector<double> &, const add_fn & add) { for(const auto & c : cuts) add(c); });
        });
    }
}
// T7: cuts through columns presolve removes: fixed columns in the cut, complements, a dual-fixable column, u aggregates.
static void removed() {
    const auto B = mkB(20);
    for(const auto & [n, seed] : std::initializer_list<std::pair<int, unsigned>>{{22, 1}, {22, 3}, {22, 5}}) {
        cmp(B, std::format("T7 DFJ + fixed column in the cut (f==1, rhs+1), atsp{}s{}", n, seed), [=](M & m) {
            auto f = m.add_binary_variables(5);
            for(std::size_t i = 0; i < 5; ++i) m.add_constraint(1.0 * f(i) == 1.0);
            std::vector<int> fc; for(std::size_t i = 0; i < 5; ++i) fc.push_back(m.native_id(f(i)));
            auto a = build_atsp(m, n, false, seed);
            return separator([=](const std::vector<double> & x, const add_fn & add) {
                for(auto c : a.separate(x)) { c.ind.push_back(fc[0]); c.val.push_back(1.0); c.ind.push_back(fc[3]); c.val.push_back(-2.0); c.rhs += 1.0 - 2.0; add(c); }
            });
        });
        cmp(B, std::format("T7 DFJ on complements and originals mixed, atsp{}s{}", n, seed), [=](M & m) {
            auto a = build_atsp(m, n, true, seed);
            return separator([=](const std::vector<double> & x, const add_fn & add) {
                // half the terms on x, half on 1 - xbar: same cut after aggregation
                for(const auto & c : a.separate(x)) {
                    raw_cut out{{}, {}, 'G', c.rhs};
                    int k = 0;
                    for(std::size_t t = 0; t < c.ind.size(); ++t) {
                        if(k++ % 2 == 0) { out.ind.push_back(c.ind[t]); out.val.push_back(c.val[t]); continue; }
                        // c is on complements (val -1): flip this term back to the original arc
                        int oi = -1, oj = -1;
                        for(int i = 0; i < n && oi < 0; ++i) for(int j = 0; j < n; ++j) if(i != j && a.comp[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] == c.ind[t]) { oi = i; oj = j; break; }
                        out.ind.push_back(a.col[static_cast<std::size_t>(oi)][static_cast<std::size_t>(oj)]); out.val.push_back(1.0); out.rhs += 1.0;
                    }
                    add(out);
                }
            });
        });
        cmp(B, std::format("T7 DFJ + dual-fixable column y in the cut, atsp{}s{}", n, seed), [=](M & m) {
            auto y = m.add_variables(1, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 10});
            const int yc = m.native_id(y(std::size_t{0}));
            auto a = build_atsp(m, n, false, seed);
            return separator([=](const std::vector<double> & x, const add_fn & add) {
                for(auto c : a.separate(x)) { c.ind.push_back(yc); c.val.push_back(1.0); add(c); }  // adding y >= 0 to the left of a >= keeps it valid
            });
        });
        cmp(B, std::format("T7 u_i + x_0i >= 1 on the MTZ potentials, atsp{}s{}", n, seed), [=](M & m) {
            auto a = build_atsp(m, n, false, seed);
            // the potentials are the n columns after the arcs
            std::vector<int> uc; for(int i = 0; i < n; ++i) uc.push_back(n * (n - 1) + i);
            return separator([=](const std::vector<double> & x, const add_fn & add) {
                for(int i = 1; i < n; ++i) add(raw_cut{{uc[static_cast<std::size_t>(i)], a.col[0][static_cast<std::size_t>(i)]}, {1.0, 1.0}, 'G', 1.0});
                for(const auto & c : a.separate(x)) add(c);
            });
        });
    }
}
// Integer infeasible, LP feasible: every cut is valid. A cut emptying the LP must give infeasible, not a crash or a value.
static void infeasible() {
    int baseinf = 0, ok = 0, bad = 0, nopower = 0;
    for(unsigned seed = 1; seed <= 200; ++seed) {
        const std::size_t C = 18;
        const auto build = [&](M & m) {
            std::mt19937 rng(seed);
            std::uniform_int_distribution<int> A(1, 9);
            auto x = m.add_binary_variables(C);
            const auto ids = std::views::iota(std::size_t{0}, C);
            std::vector<double> a(C), b(C), c(C);
            double t = 0; for(auto & v : a) { v = 2 * A(rng); t += v; }
            for(auto & v : c) v = A(rng);
            m.set_maximization();
            m.set_objective(xsum(ids, [&](std::size_t j) { return c[j] * x(j); }));
            // even coefficients, odd rhs: LP feasible, no integer point; presolve's gcd test may catch it
            m.add_constraint(xsum(ids, [&](std::size_t j) { return a[j] * x(j); }) == std::floor(t / 4) * 2 + 1);
            std::vector<double> r(C); double s = 0; for(auto & v : r) { v = A(rng); s += v; }
            m.add_constraint(xsum(ids, [&](std::size_t j) { return r[j] * x(j); }) <= std::floor(s / 2));
            std::vector<int> col; for(std::size_t j = 0; j < C; ++j) col.push_back(m.native_id(x(j)));
            return col;
        };
        { M m; m.set_thread_limit(1); build(m); vparam(m, CPX_PARAM_PREIND, 0); all_families_off(m); m.set_time_limit(duration<double>(5)); timed_solve(m); if(is_a<status::infeasible>(m.get_status())) ++baseinf; }
        for(const int cutkind : {0, 1}) {
            M m; m.set_thread_limit(1); auto col = build(m); vparam(m, CPX_PARAM_PREIND, 0); m.set_time_limit(duration<double>(5));
            auto s = std::make_shared<stats>();
            auto & h = vattach(m, watched([=](const std::vector<double> & x, const add_fn & add) {
                if(cutkind == 0) { raw_cut c{{}, {}, 'L', -1}; for(int j : col) { c.ind.push_back(j); c.val.push_back(1.0); } add(c); }
                else add(raw_cut{{col[0]}, {1.0}, 'G', 2.0});  // x0 >= 2: past its own bound
            }, s));
            const double t = timed_solve(m); (void)t;
            if(h.hook_calls == 0) ++nopower;
            else if(is_a<status::infeasible>(m.get_status())) ++ok;
            else { ++bad; std::println(stderr, "T8 seed={} kind={} NOT infeasible: {} | {}", seed, cutkind, summary(m, t), h.line()); }
        }
    }
    std::println(stderr, "T8 integer-infeasible models: baseline infeasible {}/200; emptying cuts: infeasible={} other={} no_hook_call={}", baseinf, ok, bad, nopower);
}
// T9: very many distinct, violated, valid cuts in one call: covers of row 0 with random weights.
static void many() {
    const auto B = mkB(30);
    for(const int K : {1000, 20000}) cmp(B, std::format("T9 {} distinct violated cover-derived cuts per root call (first 3 calls), mkp5x60s3", K), [=](M & m) {
        build_mkp(m, 5, 60, 3);
        auto [env, lp] = m.native_model();
        const int n = static_cast<int>(m.num_variables());
        std::vector<int> ind(static_cast<std::size_t>(n)); std::vector<double> val(static_cast<std::size_t>(n)); int nz, beg, surplus; double b;
        m.native_api()._check(env, m.native_api().getrows(env, lp, &nz, &beg, ind.data(), val.data(), n, &surplus, 0, 0));
        m.native_api()._check(env, m.native_api().getrhs(env, lp, &b, 0, 0));
        std::vector<double> a(static_cast<std::size_t>(n), 0.0);
        for(int k = 0; k < nz; ++k) a[static_cast<std::size_t>(ind[static_cast<std::size_t>(k)])] = val[static_cast<std::size_t>(k)];
        auto cnt = std::make_shared<std::atomic<int>>(0);
        return separator([=](const std::vector<double> & x, const add_fn & add) {
            if(cnt->fetch_add(1) >= 3) return;
            std::vector<std::size_t> order(a.size()); for(std::size_t j = 0; j < order.size(); ++j) order[j] = j;
            std::ranges::sort(order, [&](std::size_t p, std::size_t r) { return x[p] > x[r]; });
            std::vector<int> cover; double w = 0;
            for(std::size_t j : order) { w += a[j]; cover.push_back(static_cast<int>(j)); if(w > b) break; }
            std::mt19937 rng(9);
            for(int k = 0; k < K; ++k) {
                raw_cut cut{{}, {}, 'L', 0}; double top = 0, low = 1e30;
                for(int j : cover) { const double wj = 1.0 + static_cast<double>(rng() % 1000) * 1e-4; cut.ind.push_back(j); cut.val.push_back(wj); top += wj; low = std::min(low, wj); }
                cut.rhs = top - low;  // at most |C|-1 of C at 1
                add(cut);
            }
        });
    });
}
// ---- Fuzz: feasible set by enumeration, every cut's rhs is the true maximum over it.
struct fm { std::vector<int> col; std::vector<double> c; std::vector<std::vector<unsigned char>> F; double opt; bool maximize; };
static std::optional<fm> make_fm(const unsigned seed) {
    // Column layout: core[k], f[2] (fixed), comp[3] (complement of core 0,1,2), par[2] (parallel to core 3,4 in every row)
    std::mt19937 rng(seed);
    const int k = 16 + static_cast<int>(rng() % 5);
    const int rows = 4 + static_cast<int>(rng() % 6);
    std::uniform_int_distribution<int> A(-5, 20), C(1, 40);
    std::vector<std::vector<double>> a(static_cast<std::size_t>(rows), std::vector<double>(static_cast<std::size_t>(k) + 2, 0.0));
    std::vector<double> b(static_cast<std::size_t>(rows));
    for(int i = 0; i < rows; ++i) { double t = 0; for(int j = 0; j < k + 2; ++j) { const double v = A(rng); a[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] = v; t += std::max(v, 0.0); } b[static_cast<std::size_t>(i)] = std::floor(t * (0.25 + 0.3 * static_cast<double>(rng() % 100) / 100)); }
    std::vector<double> c(static_cast<std::size_t>(k) + 7, 0.0);
    for(int j = 0; j < k; ++j) c[static_cast<std::size_t>(j)] = C(rng);
    fm f; f.maximize = rng() % 2 == 0;
    if(!f.maximize) for(auto & v : c) v = -v + 3;
    const int N = k + 7;  // k core, 2 fixed, 3 complements, 2 parallel
    f.c = c;
    double best = f.maximize ? -1e100 : 1e100;
    for(std::uint32_t mask = 0; mask < (std::uint32_t{1} << (k + 2)); ++mask) {
        bool ok = true;
        for(int i = 0; i < rows && ok; ++i) {
            double t = 0;
            for(int j = 0; j < k; ++j) if(mask >> j & 1) t += a[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)];
            // parallel columns k+5, k+6 share the rows of core 3 and 4
            if(mask >> k & 1) t += a[static_cast<std::size_t>(i)][3];
            if(mask >> (k + 1) & 1) t += a[static_cast<std::size_t>(i)][4];
            ok = t <= b[static_cast<std::size_t>(i)] + 1e-9;
        }
        if(!ok) continue;
        std::vector<unsigned char> p(static_cast<std::size_t>(N), 0);
        for(int j = 0; j < k; ++j) p[static_cast<std::size_t>(j)] = mask >> j & 1;
        p[static_cast<std::size_t>(k)] = 1; p[static_cast<std::size_t>(k + 1)] = 0;
        for(int t = 0; t < 3; ++t) p[static_cast<std::size_t>(k + 2 + t)] = !(mask >> t & 1);
        p[static_cast<std::size_t>(k + 5)] = mask >> k & 1; p[static_cast<std::size_t>(k + 6)] = mask >> (k + 1) & 1;
        double obj = 0; for(int j = 0; j < N; ++j) obj += c[static_cast<std::size_t>(j)] * p[static_cast<std::size_t>(j)];
        best = f.maximize ? std::max(best, obj) : std::min(best, obj);
        f.F.push_back(std::move(p));
    }
    if(f.F.empty() || f.F.size() > 20000) return std::nullopt;
    f.opt = best;
    // store rows for the builder
    f.col.assign(1, k);
    static thread_local std::vector<std::vector<double>> rows_store; rows_store = a;
    return f;
}
static void fuzz(const int first, const int last, const int threads, const int purge, const int weak) {
    long cases = 0, fails = 0, nopower = 0, total_calls = 0, total_cuts = 0, total_1121 = 0, withtree = 0, skipped = 0, baseline_mismatch = 0, total_viol_check = 0;
    for(int seed = first; seed <= last; ++seed) {
        auto fo = make_fm(static_cast<unsigned>(seed));
        if(!fo) { ++skipped; continue; }
        auto & f = *fo;
        const int k = f.col[0];
        const int N = k + 7;
        // rebuild rows deterministically as make_fm did
        std::mt19937 rng(static_cast<unsigned>(seed));
        (void)rng();
        const auto build = [&](M & m) {
            std::mt19937 r2(static_cast<unsigned>(seed));
            const int kk = 16 + static_cast<int>(r2() % 5);
            const int rows = 4 + static_cast<int>(r2() % 6);
            std::uniform_int_distribution<int> A(-5, 20), C(1, 40);
            std::vector<std::vector<double>> a(static_cast<std::size_t>(rows), std::vector<double>(static_cast<std::size_t>(kk) + 2, 0.0));
            std::vector<double> b(static_cast<std::size_t>(rows));
            for(int i = 0; i < rows; ++i) { double t = 0; for(int j = 0; j < kk + 2; ++j) { const double v = A(r2); a[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] = v; t += std::max(v, 0.0); } b[static_cast<std::size_t>(i)] = std::floor(t * (0.25 + 0.3 * static_cast<double>(r2() % 100) / 100)); }
            auto x = m.add_binary_variables(static_cast<std::size_t>(kk) + 7);
            const auto ids = std::views::iota(std::size_t{0}, static_cast<std::size_t>(kk));
            const auto X = [&](int j) { return x(static_cast<std::size_t>(j)); };
            if(f.maximize) m.set_maximization(); else m.set_minimization();
            m.set_objective(xsum(std::views::iota(std::size_t{0}, static_cast<std::size_t>(kk) + 7), [&](std::size_t j) { return f.c[j] * x(j); }));
            for(int i = 0; i < rows; ++i) m.add_constraint(xsum(ids, [&, i](std::size_t j) { return a[static_cast<std::size_t>(i)][j] * x(j); }) + a[static_cast<std::size_t>(i)][3] * X(kk + 5) + a[static_cast<std::size_t>(i)][4] * X(kk + 6) <= b[static_cast<std::size_t>(i)]);
            m.add_constraint(1.0 * X(kk) == 1.0);
            m.add_constraint(1.0 * X(kk + 1) == 0.0);
            for(int t = 0; t < 3; ++t) m.add_constraint(X(t) + X(kk + 2 + t) == 1.0);
        };
        double base = NAN;
        { M m; m.set_thread_limit(1); build(m); m.set_time_limit(duration<double>(20)); timed_solve(m); base = cf::value_of(m); }
        if(!cf::same(base, f.opt)) { ++baseline_mismatch; std::println(stderr, "FUZZ seed={} no-hook solver value {} != enumeration {} ({} points)", seed, base, f.opt, f.F.size()); }
        M m; if(threads) set_threads(m, threads); else m.set_thread_limit(1);
        build(m);
        if(weak) { all_families_off(m); vparam(m, CPX_PARAM_MIRCUTS, 2); vparam(m, CPX_PARAM_EACHCUTLIM, 0); vparam(m, CPX_PARAM_HEURFREQ, -1); vparam(m, CPX_PARAM_RINSHEUR, -1); vparam(m, CPX_PARAM_FPHEUR, -1); }
        m.set_time_limit(duration<double>(20));
        // the separator: projects a little toward feasible points, keeps directions that separate
        auto gen = std::make_shared<std::mt19937>(static_cast<unsigned>(seed) * 131u);
        auto cnt = std::make_shared<std::atomic<long>>(0);
        const double opt = f.opt; const auto & F = f.F; const auto & cc = f.c; const bool mx = f.maximize;
        auto s = std::make_shared<stats>();
        const auto best_rhs = [&F, N](const std::vector<double> & w) {
            double top = -1e100;
            for(const auto & p : F) { double t = 0; for(int j = 0; j < N; ++j) if(p[static_cast<std::size_t>(j)]) t += w[static_cast<std::size_t>(j)]; top = std::max(top, t); }
            return top;
        };
        recipe R; R.purgeable = purge; if(std::getenv("RECIPE_GUARD")) R.same_point_guard = true;
        auto & h = vattach(m, watched([&, N, opt, mx](const std::vector<double> & x, const add_fn & add) {
            auto & g = *gen;
            const long budget = cnt->fetch_add(1);
            if(budget > 100) return;  // bounded work in each case
            // (b) objective direction, tight at the optimum
            { raw_cut c{{}, {}, mx ? 'L' : 'G', opt}; for(int j = 0; j < N; ++j) if(cc[static_cast<std::size_t>(j)] != 0) { c.ind.push_back(j); c.val.push_back(cc[static_cast<std::size_t>(j)]); } add(c); }
            // (c) one-term cuts implied by the feasible set
            if(budget < 3) for(int j = 0; j < N; ++j) {
                bool all0 = true, all1 = true;
                for(const auto & p : F) { if(p[static_cast<std::size_t>(j)]) all0 = false; else all1 = false; if(!all0 && !all1) break; }
                if(all0) add(raw_cut{{j}, {1.0}, 'L', 0.0});
                if(all1) add(raw_cut{{j}, {1.0}, 'G', 1.0});
            }
            // (a) directions from a random feasible point to the LP point, rounded, rhs exact
            for(int tries = 0; tries < 12; ++tries) {
                const auto & p0 = F[g() % F.size()];
                std::vector<double> w(static_cast<std::size_t>(N));
                const double mult = static_cast<double>(1 + g() % 12);
                for(int j = 0; j < N; ++j) w[static_cast<std::size_t>(j)] = std::round(mult * (x[static_cast<std::size_t>(j)] - p0[static_cast<std::size_t>(j)]));
                if(g() % 2) for(int j = 0; j < N; ++j) if(g() % 3 == 0) w[static_cast<std::size_t>(j)] = 0;
                const double top = best_rhs(w);
                double act = 0; for(int j = 0; j < N; ++j) act += w[static_cast<std::size_t>(j)] * x[static_cast<std::size_t>(j)];
                if(act <= top + 1e-6) continue;
                static constexpr double scales[] = {1.0, 1.0, 1.0, 1e3, 1e6, 1e9, 1e-3, 1e-6};
                const double S = scales[g() % std::size(scales)];
                raw_cut c{{}, {}, 'L', top * S};
                for(int j = 0; j < N; ++j) if(w[static_cast<std::size_t>(j)] != 0) { c.ind.push_back(j); c.val.push_back(w[static_cast<std::size_t>(j)] * S); }
                if(c.ind.empty()) continue;
                add(c);
                // (e) a near duplicate: same cut, rhs relaxed a hair, one coefficient bumped by e with the exact new rhs
                if(g() % 3 == 0) { auto d = c; d.rhs += 1e-9 * std::max(1.0, std::fabs(d.rhs)); add(d); }
                if(g() % 3 == 0) {
                    auto w2 = w; const int jj = static_cast<int>(g() % static_cast<unsigned>(N)); w2[static_cast<std::size_t>(jj)] += 1e-7;
                    raw_cut d{{}, {}, 'L', best_rhs(w2)};
                    for(int j = 0; j < N; ++j) if(w2[static_cast<std::size_t>(j)] != 0) { d.ind.push_back(j); d.val.push_back(w2[static_cast<std::size_t>(j)]); }
                    add(d);
                }
            }
        }, s), R);
        const double t = timed_solve(m);
        const double v = cf::value_of(m);
        const bool ok = is_a<status::optimal>(m.get_status()) && cf::same(v, f.opt);
        ++cases; total_calls += h.hook_calls.load(); total_cuts += h.added.load(); total_1121 += h.err_1121.load();
        if(h.added.load() == 0) ++nopower;
        if(m.get_node_count() > 0) ++withtree;
        if(!ok || h.err_other.load() || h.exceptions.load()) { ++fails; std::println(stderr, "FUZZ FAIL seed={} {} enumeration opt {} | {} | {}", seed, summary(m, t), f.opt, h.line(), s->line()); }
        (void)total_viol_check;
    }
    std::println(stderr, "FUZZ seeds {}..{} threads={} purge={} weak={}: cases={} skipped={} fails={} no_cut_added={} with_tree={} hook_calls={} cuts_added={} err1121={} baseline!=enumeration={}", first, last, threads, purge, weak, cases, skipped, fails, nopower, withtree, total_calls, total_cuts, total_1121, baseline_mismatch);
}
int main(int argc, char ** argv) {
    { M m; print_version(m); }
    const std::string what = argc > 1 ? argv[1] : "";
    if(what == "tol") tol();
    if(what == "scale") scale();
    if(what == "tight") tight();
    if(what == "near") near();
    if(what == "oneterm") oneterm();
    if(what == "removed") removed();
    if(what == "infeasible") infeasible();
    if(what == "many") many();
    if(what == "fuzz") fuzz(std::atoi(argv[2]), std::atoi(argv[3]), std::atoi(argv[4]), argc > 5 ? std::atoi(argv[5]) : CPX_USECUT_PURGE, argc > 6 ? std::atoi(argv[6]) : 0);
    return cf::finish();
}
