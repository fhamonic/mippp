// D, second pass: cuts that REACH the native call. DFJ cuts are violated by construction, so sanitize keeps them,
// and each one carries a piece over columns presolve removes: fixed, dual-fixed, duplicate, singleton, substituted.
// All valid: DFJ + piece, where piece >= its added rhs at every integer feasible point. One process per case.
#include "verify_xpress_hook.hpp"
#include <sys/wait.h>
#include <unistd.h>
using namespace mippp; using namespace mippp::operators;
using std::chrono::duration;
using cut = cp::raw_cut;
static bool close_(double a, double b) { return std::fabs(a - b) <= 1e-6 * (1 + std::fabs(a)); }
template <class F>
static void isolated(const std::string & name, F f, unsigned seconds = 80) {
    std::fflush(stderr);
    const pid_t p = fork();
    if(p == 0) { alarm(seconds); vx::quiet_flush = true; f(); std::fflush(stderr); _exit(0); }
    int st = 0; waitpid(p, &st, 0);
    if(WIFSIGNALED(st)) std::println(stderr, "CRASH {}: signal {}", name, WTERMSIG(st));
    else if(WEXITSTATUS(st)) std::println(stderr, "CHILD {}: exit {}", name, WEXITSTATUS(st));
}
template <class Build>
static void kase(const std::string & name, const std::string & spec, Build build, double limit = 20, double known = NAN) {
    double base = known; std::string bs;
    if(std::isnan(known)) {
        M m; m.set_thread_limit(1);
        auto keep = build(m); m.set_time_limit(duration<double>(limit * 3));
        const double t = cp::timed_solve(m);
        base = cf::value_of(m); bs = cp::summary(m, t);
        if(!is_a<status::optimal>(m.get_status())) std::println(stderr, "NOBASE {} [{}]: {}", name, spec, bs);
    }
    M m; m.set_thread_limit(1);
    auto sep = build(m); m.set_time_limit(duration<double>(limit));
    auto s = std::make_shared<cp::stats>();
    vx::recipe r = vx::parse(spec);
    auto & c = vx::attach(m, cp::watched(sep, s), r);
    const double t = cp::timed_solve(m);
    const double v = cf::value_of(m);
    const bool opt = is_a<status::optimal>(m.get_status());
    std::string ps; for(auto [k, n] : c.presolve_status) ps += std::format("{}:{} ", k, n);
    std::string er; for(auto [k, n] : c.errors) er += std::format("{}:{} ", k, n);
    std::println(stderr, "{} {} [{}]: base {} | {} | calls={} native={} dropped={} rows {}->{} CUTS_max={} nodes_seen={} depth={} presolve{{{}}} empty={} errors{{{}}}{}", (opt && close_(v, base)) ? "OK  " : "BAD ", name, spec, base, cp::summary(m, t), s->calls.load(), c.added_native.load(), c.dropped.load(), c.rows_first, c.rows_max, c.cuts_max, c.nodes.size(), c.max_depth, ps, c.presolve_empty.load(), er, c.first_error.empty() ? "" : " first=[" + c.first_error + "]");
}
struct blk { int t, s, q, w, v, p, f, k, h; };
// Minimize. t,s duplicates: identical in rows, cost -10. w: cost +3 in no row, dual fixed at 0. v: continuous singleton in v + p == 1.
// f: f == 1. h == 2 k + 3, k integer in [0,4].
static blk add_block(M & m) {
    auto b = m.add_binary_variables(5);  // t s q p f
    auto w = m.add_variables(1, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 1});
    auto v = m.add_variables(1, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 5});
    auto k = m.add_integer_variables(1, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 10});
    auto h = m.add_variables(1, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 100});
    const std::size_t z = 0;
    blk B{m.native_id(b(z)), m.native_id(b(z + 1)), m.native_id(b(z + 2)), m.native_id(w(z)), m.native_id(v(z)), m.native_id(b(z + 3)), m.native_id(b(z + 4)), m.native_id(k(z)), m.native_id(h(z))};
    m.add_constraint(b(z) + b(z + 1) + b(z + 2) <= 1);
    m.add_constraint(v(z) + b(z + 3) == 1);
    m.add_constraint(1.0 * b(z + 4) == 1);
    m.add_constraint(h(z) - 2.0 * k(z) == 3.0);
    m.add_constraint(1.0 * k(z) <= 4.0);
    m.set_objective_coefficient(b(z), -10); m.set_objective_coefficient(b(z + 1), -10); m.set_objective_coefficient(b(z + 2), -7);
    m.set_objective_coefficient(w(z), 3); m.set_objective_coefficient(v(z), 1); m.set_objective_coefficient(b(z + 3), 2);
    m.set_objective_coefficient(b(z + 4), 1); m.set_objective_coefficient(h(z), -1);
    return B;
}
struct piece { std::string name; std::function<void(cut &, const blk &)> apply; };
static std::vector<piece> pieces() {
    return {
        {"none", [](cut &, const blk &) {}},
        {"+ f, rhs + 1 (fixed column)", [](cut & c, const blk & b) { c.ind.push_back(b.f); c.val.push_back(1); c.rhs += 1; }},
        {"+ 0.1 f, rhs + 0.1 (fixed, rounding)", [](cut & c, const blk & b) { c.ind.push_back(b.f); c.val.push_back(0.1); c.rhs += 0.1; }},
        {"+ w (dual-fixed at 0, weaker)", [](cut & c, const blk & b) { c.ind.push_back(b.w); c.val.push_back(1); }},
        {"- 3 w, rhs - 3 (dual-fixed, weaker, w<=1)", [](cut & c, const blk & b) { c.ind.push_back(b.w); c.val.push_back(-3); c.rhs -= 3; }},
        {"+ t + s + q, rhs (duplicates, equal, weaker)", [](cut & c, const blk & b) { c.ind.insert(c.ind.end(), {b.t, b.s, b.q}); c.val.insert(c.val.end(), {1, 1, 1}); }},
        {"+ t + 2 s (duplicates, unequal, weaker)", [](cut & c, const blk & b) { c.ind.insert(c.ind.end(), {b.t, b.s}); c.val.insert(c.val.end(), {1, 2}); }},
        {"- t - 2 s, rhs - 2 (duplicates, unequal, t+s<=1)", [](cut & c, const blk & b) { c.ind.insert(c.ind.end(), {b.t, b.s}); c.val.insert(c.val.end(), {-1, -2}); c.rhs -= 2; }},
        {"+ t (one duplicate, weaker)", [](cut & c, const blk & b) { c.ind.push_back(b.t); c.val.push_back(1); }},
        {"+ v + p, rhs + 1 (singleton, row added)", [](cut & c, const blk & b) { c.ind.insert(c.ind.end(), {b.v, b.p}); c.val.insert(c.val.end(), {1, 1}); c.rhs += 1; }},
        {"+ v (singleton, weaker)", [](cut & c, const blk & b) { c.ind.push_back(b.v); c.val.push_back(1); }},
        {"+ h - 2 k, rhs + 3 (substituted, row added)", [](cut & c, const blk & b) { c.ind.insert(c.ind.end(), {b.h, b.k}); c.val.insert(c.val.end(), {1, -2}); c.rhs += 3; }},
        {"+ h, rhs + 3 (substituted, weaker since h>=3)", [](cut & c, const blk & b) { c.ind.push_back(b.h); c.val.push_back(1); c.rhs += 3; }},
        {"+ k (general integer, weaker)", [](cut & c, const blk & b) { c.ind.push_back(b.k); c.val.push_back(1); }},
    };
}
static void reduced_dfj(const std::vector<std::string> & specs_, const std::vector<unsigned> & seeds_) {
    for(const auto & spec : specs_) for(const auto & pc : pieces()) for(const unsigned seed : seeds_) {
        const std::string name = std::format("DFJ {} n=26 seed={}", pc.name, seed);
        const std::string sp = spec;
        isolated(name + " [" + sp + "]", [&] {
            kase(name, sp, [&](M & m) {
                auto a = cp::build_atsp(m, 26, false, seed);
                blk b = add_block(m);
                const auto ap = pc.apply;
                return cp::separator([a, b, ap](const std::vector<double> & x, const cp::add_fn & add) { for(cut c : a.separate(x)) { ap(c, b); add(c); } });
            });
        });
    }
}
// Same on a tree: covers of the knapsack, with the piece. Covers are <=, so a piece must keep it valid in that sense.
template <class Mo> static auto tree_block(Mo & m, unsigned seed, std::size_t cols, std::size_t rows) {
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
static cp::separator covers(std::vector<std::vector<double>> a, std::vector<double> cap, std::vector<int> col, double sc = 1.0, std::function<void(cut &)> post = {}) {
    return [=](const std::vector<double> & p, const cp::add_fn & add) {
        for(std::size_t i = 0; i < a.size(); ++i) {
            std::vector<std::size_t> order(col.size());
            for(std::size_t j = 0; j < order.size(); ++j) order[j] = j;
            std::ranges::sort(order, [&](std::size_t l, std::size_t r) { return p[static_cast<std::size_t>(col[l])] > p[static_cast<std::size_t>(col[r])]; });
            double w = 0; cut k{{}, {}, 'L', 0};
            for(std::size_t j : order) {
                w += a[i][j]; k.ind.push_back(col[j]); k.val.push_back(1);
                if(w > cap[i]) { k.rhs = static_cast<double>(k.ind.size()) - 1; if(k.violation(p) > 1e-4) { for(auto & v : k.val) v *= sc; k.rhs *= sc; if(post) post(k); add(k); } break; }
            }
        }
    };
}
// Scaling applied by the user, so before sanitize: a re-offering separator sees its own violation at scale.
static void scaled_reoffer() {
    for(const char * spec : {"R1", "R1,add2", "R2", "R2cr"}) for(double sc : {1e-9, 1e-6, 1e-3, 1.0, 1e3, 1e5, 1e6, 1e7, 1e8, 1e10}) for(const int family : {0, 1}) {
        const std::string name = std::format("{} re-offered at every call, user scale {}", family == 0 ? "DFJ n=26" : "covers 40x5", sc);
        const std::string sp = spec;
        isolated(name + " [" + sp + "]", [&] {
            kase(name, sp, [&](M & m) -> cp::separator {
                auto all = std::make_shared<std::vector<cut>>();
                if(family == 0) {
                    auto a = cp::build_atsp(m, 26, false, 5);
                    return [a, all, sc](const std::vector<double> & x, const cp::add_fn & add) {
                        for(cut c : a.separate(x)) { for(auto & v : c.val) v *= sc; c.rhs *= sc; all->push_back(c); }
                        for(const auto & c : *all) add(c);
                    };
                }
                auto [x, col, c, a, cap] = tree_block(m, 3, 40, 5);
                auto inner = covers(a, cap, col, sc);
                return [inner, all](const std::vector<double> & p, const cp::add_fn & add) {
                    inner(p, [&](const cut & k) { all->push_back(k); return 0; });
                    for(const auto & k : *all) add(k);
                };
            }, 10);
        }, 50);
    }
}
// Near-duplicates of a violated cut, a new near-duplicate at every call, forever: rhs moves by tiny amounts, all valid.
static void near_dup() {
    for(const char * spec : {"R1", "R1,add2", "R2", "R2cr"}) for(const double eps : {1e-12, 1e-9, 1e-7}) for(const int family : {0, 1}) {
        const std::string name = std::format("{} each cut offered in 6 relaxed near-duplicates (rhs + k*{})", family == 0 ? "DFJ n=26" : "covers 40x5", eps);
        const std::string sp = spec;
        isolated(name + " [" + sp + "]", [&] {
            kase(name, sp, [&](M & m) -> cp::separator {
                if(family == 0) {
                    auto a = cp::build_atsp(m, 26, false, 5);
                    return [a, eps](const std::vector<double> & x, const cp::add_fn & add) { for(const cut & c : a.separate(x)) for(int k = 0; k < 6; ++k) { cut d = c; d.rhs -= k * eps; add(d); } };
                }
                auto [x, col, c, a, cap] = tree_block(m, 3, 40, 5);
                auto inner = covers(a, cap, col);
                return [inner, eps](const std::vector<double> & p, const cp::add_fn & add) {
                    inner(p, [&](const cut & q) { for(int k = 0; k < 6; ++k) { cut d = q; d.rhs += k * eps; add(d); } return 0; });
                };
            }, 15);
        }, 60);
    }
}
// Objective cut, tight at every optimum, as a user who knows opt would write it. Reaches native when an LP point exceeds opt.
static void objective_tight() {
    for(const char * spec : {"R1", "R1,add2", "R2", "R2cr"}) for(unsigned seed : {1u, 2u, 3u}) for(const double frac : {1.0, 1.0 / 3.0, 1e4}) {
        const std::string sp = spec;
        double truth;
        { M m; m.set_thread_limit(1); m.set_time_limit(duration<double>(30)); tree_block(m, seed, 40, 5); m.solve(); truth = m.get_solution_value(); }
        const std::string name = std::format("objective cut c x <= opt={} scaled {} seed={}", truth, frac, seed);
        isolated(name + " [" + sp + "]", [&] {
            kase(name, sp, [&](M & m) -> cp::separator {
                auto [x, col, c, a, cap] = tree_block(m, seed, 40, 5);
                std::vector<double> v = c; for(auto & e : v) e *= frac;
                const cut k{col, v, 'L', truth * frac};
                return [k](const std::vector<double> &, const cp::add_fn & add) { add(k); };
            }, 20, truth);
        });
    }
}
// Every family's one-term and pruned cases, reaching native: keep_satisfied, once per firing is unbounded so limit by count.
static void one_term_and_prune() {
    struct K { std::string name; std::function<cp::separator(M &)> build; };
    const std::vector<K> ks = {
        {"one term: x_big <= 0, item never fits", [](M & m) -> cp::separator {
            auto [x, col, c, a, cap] = tree_block(m, 3, 40, 5);
            auto big = m.add_binary_variable();
            m.add_constraint(1000.0 * big + xsum(std::views::iota(std::size_t{0}, std::size_t{5}), [&, xx = x](std::size_t j) { return 1.0 * xx(j); }) <= 500);
            m.set_objective_coefficient(big, 50);
            const cut k{{m.native_id(big)}, {1}, 'L', 0};
            return [k](const std::vector<double> &, const cp::add_fn & add) { add(k); };
        }},
        {"one term: CG k <= 4 from 2 k <= 9", [](M & m) -> cp::separator {
            auto [x, col, c, a, cap] = tree_block(m, 3, 40, 5);
            auto k = m.add_integer_variable({.lower_bound = 0., .upper_bound = 20.});
            m.add_constraint(2.0 * k <= 9.0);
            m.set_objective_coefficient(k, 5);
            const cut q{{m.native_id(k)}, {1}, 'L', 4};
            return [q](const std::vector<double> &, const cp::add_fn & add) { add(q); };
        }},
        {"one term: x_big >= 0, x <= 1 on every column, all at each call", [](M & m) -> cp::separator {
            auto [x, col, c, a, cap] = tree_block(m, 3, 40, 5);
            std::vector<cut> v; for(int j : col) { v.push_back(cut{{j}, {1}, 'L', 1}); v.push_back(cut{{j}, {1}, 'G', 0}); }
            return [v](const std::vector<double> &, const cp::add_fn & add) { for(const auto & q : v) add(q); };
        }},
        // y = 1 forces 2 Sum o = 5: no integer point, LP fine. y <= 0 valid, violated at every y = 1 node, closes that subtree.
        {"prune: y <= 0 where y=1 has no integer point (parity)", [](M & m) -> cp::separator {
            auto [x, col, c, a, cap] = tree_block(m, 3, 30, 4);
            auto y = m.add_binary_variable(); auto o = m.add_binary_variables(5);
            const std::size_t z = 0;
            const auto sum = o(z) + o(z + 1) + o(z + 2) + o(z + 3) + o(z + 4);
            m.add_constraint(2.0 * sum - 5.0 * y >= 0.0);
            m.add_constraint(2.0 * sum + 5.0 * y <= 10.0);
            m.set_objective_coefficient(y, 40);
            const cut q{{m.native_id(y)}, {1}, 'L', 0};
            return [q](const std::vector<double> &, const cp::add_fn & add) { add(q); };
        }},
        // no integer point at all: a valid cut may close the search, status must stay infeasible, baseline also infeasible
        {"prune: integer-infeasible model, y <= 0 with y == 1 row", [](M & m) -> cp::separator {
            auto [x, col, c, a, cap] = tree_block(m, 3, 30, 4);
            auto y = m.add_binary_variable(); auto o = m.add_binary_variables(5);
            const std::size_t z = 0;
            const auto sum = o(z) + o(z + 1) + o(z + 2) + o(z + 3) + o(z + 4);
            m.add_constraint(2.0 * sum - 5.0 * y >= 0.0);
            m.add_constraint(2.0 * sum + 5.0 * y <= 10.0);
            m.add_constraint(1.0 * y >= 0.5);
            const cut q{{m.native_id(y)}, {1}, 'L', 0};
            return [q](const std::vector<double> &, const cp::add_fn & add) { add(q); };
        }},
        // Rounded and tiny: a valid cut with a tiny coefficient that the matrix tolerance may drop and a rhs that then bites
        {"tiny term: x0 + x1 + 1e-10 x2 <= 1 + 1e-10 on x0 + x1 <= 1", [](M & m) -> cp::separator {
            auto [x, col, c, a, cap] = tree_block(m, 3, 30, 4);
            auto p = m.add_binary_variable(); auto q = m.add_binary_variable(); auto r = m.add_binary_variable();
            m.add_constraint(p + q <= 1);
            m.add_constraint(xsum(std::views::iota(std::size_t{0}, std::size_t{5}), [&, xx = x](std::size_t j) { return 1.0 * xx(j); }) + p - q + r <= 3);
            m.set_objective_coefficient(p, 20); m.set_objective_coefficient(q, 25); m.set_objective_coefficient(r, 9);
            const cut k{{m.native_id(p), m.native_id(q), m.native_id(r)}, {1, 1, 1e-10}, 'L', 1 + 1e-10};
            return [k](const std::vector<double> &, const cp::add_fn & add) { add(k); };
        }},
        {"tiny all: 1e-11 (p + q) <= 1e-11", [](M & m) -> cp::separator {
            auto [x, col, c, a, cap] = tree_block(m, 3, 30, 4);
            auto p = m.add_binary_variable(); auto q = m.add_binary_variable();
            m.add_constraint(p + q <= 1);
            m.add_constraint(xsum(std::views::iota(std::size_t{0}, std::size_t{5}), [&, xx = x](std::size_t j) { return 1.0 * xx(j); }) + p - q <= 3);
            m.set_objective_coefficient(p, 20); m.set_objective_coefficient(q, 25);
            const cut k{{m.native_id(p), m.native_id(q)}, {1e-11, 1e-11}, 'L', 1e-11};
            return [k](const std::vector<double> &, const cp::add_fn & add) { add(k); };
        }},
        // the tiny G form: dropped coefficients leave 0 >= tiny: a false constant if the rhs is kept
        {"tiny G: 1e-11 (p + q + r) >= 1e-11 where p + q + r >= 1 is a row", [](M & m) -> cp::separator {
            auto [x, col, c, a, cap] = tree_block(m, 3, 30, 4);
            auto p = m.add_binary_variable(); auto q = m.add_binary_variable(); auto r = m.add_binary_variable();
            m.add_constraint(p + q + r >= 1);
            m.add_constraint(p + q + r + xsum(std::views::iota(std::size_t{0}, std::size_t{5}), [&, xx = x](std::size_t j) { return 1.0 * xx(j); }) <= 5);
            m.set_objective_coefficient(p, -20); m.set_objective_coefficient(q, -25); m.set_objective_coefficient(r, -9);
            const cut k{{m.native_id(p), m.native_id(q), m.native_id(r)}, {1e-11, 1e-11, 1e-11}, 'G', 1e-11};
            return [k](const std::vector<double> &, const cp::add_fn & add) { add(k); };
        }},
        // big coefficient beyond 1e20 / infinity rhs
        {"huge: 1e12 (p + q) <= 1e12", [](M & m) -> cp::separator {
            auto [x, col, c, a, cap] = tree_block(m, 3, 30, 4);
            auto p = m.add_binary_variable(); auto q = m.add_binary_variable();
            m.add_constraint(p + q <= 1);
            m.add_constraint(xsum(std::views::iota(std::size_t{0}, std::size_t{5}), [&, xx = x](std::size_t j) { return 1.0 * xx(j); }) + p - q <= 3);
            m.set_objective_coefficient(p, 20); m.set_objective_coefficient(q, 25);
            const cut k{{m.native_id(p), m.native_id(q)}, {1e12, 1e12}, 'L', 1e12};
            return [k](const std::vector<double> &, const cp::add_fn & add) { add(k); };
        }},
        {"huge rhs: x0 + x1 <= 1e40", [](M & m) -> cp::separator {
            auto [x, col, c, a, cap] = tree_block(m, 3, 30, 4);
            const cut k{{col[0], col[1]}, {1, 1}, 'L', 1e40};
            return [k](const std::vector<double> &, const cp::add_fn & add) { add(k); };
        }},
    };
    for(const char * spec : {"R1", "R1,keep_satisfied", "R1,add2", "R2", "R2,keep_satisfied", "R2cr", "R2cr,keep_satisfied"}) for(const auto & k : ks) {
        const std::string sp = spec;
        isolated(k.name + " [" + sp + "]", [&] { kase(k.name, sp, k.build, 10); }, 45);
    }
}
// Once-only injection with keep_satisfied of the implied rows over reduced columns: reaches native even if satisfied.
static void implied_once() {
    struct K { std::string name; std::function<cut(const blk &)> make; };
    const std::vector<K> ks = {
        {"E v + p == 1 (eliminated)", [](const blk & b) { return cut{{b.v, b.p}, {1, 1}, 'E', 1}; }},
        {"E h - 2 k == 3 (substituted)", [](const blk & b) { return cut{{b.h, b.k}, {1, -2}, 'E', 3}; }},
        {"E f == 1 (fixed)", [](const blk & b) { return cut{{b.f}, {1}, 'E', 1}; }},
        {"G f >= 1 (fixed)", [](const blk & b) { return cut{{b.f}, {1}, 'G', 1}; }},
        {"G 0.1 f >= 0.1 (fixed, residue)", [](const blk & b) { return cut{{b.f}, {0.1}, 'G', 0.1}; }},
        {"G 1/3 f >= 1/3 (fixed, residue)", [](const blk & b) { return cut{{b.f}, {1.0 / 3}, 'G', 1.0 / 3}; }},
        {"L t + s <= 1 (duplicates)", [](const blk & b) { return cut{{b.t, b.s}, {1, 1}, 'L', 1}; }},
        {"L t + 2 s <= 2 (duplicates unequal)", [](const blk & b) { return cut{{b.t, b.s}, {1, 2}, 'L', 2}; }},
        {"G t >= 0 (one duplicate)", [](const blk & b) { return cut{{b.t}, {1}, 'G', 0}; }},
        {"G w >= 0 (dual-fixed)", [](const blk & b) { return cut{{b.w}, {1}, 'G', 0}; }},
        {"L w <= 1 (dual-fixed)", [](const blk & b) { return cut{{b.w}, {1}, 'L', 1}; }},
        {"L v <= 1 (singleton, tighter than its bound 5)", [](const blk & b) { return cut{{b.v}, {1}, 'L', 1}; }},
        {"L h <= 11 (substituted, tight)", [](const blk & b) { return cut{{b.h}, {1}, 'L', 11}; }},
        {"L k <= 4 (tight at the optimum)", [](const blk & b) { return cut{{b.k}, {1}, 'L', 4}; }},
    };
    for(const char * spec : {"R1,keep_satisfied", "R1,keep_satisfied,add2", "R2,keep_satisfied", "R2cr,keep_satisfied", "R2,keep_satisfied,no_split_eq"}) for(const auto & k : ks) {
        const std::string sp = spec;
        isolated(k.name + " [" + sp + "]", [&] {
            kase(k.name, sp, [&](M & m) -> cp::separator {
                auto [x, col, c, a, cap] = tree_block(m, 3, 40, 5);
                blk b = add_block(m);
                auto done = std::make_shared<bool>(false);
                const cut q = k.make(b);
                return [q, done](const std::vector<double> &, const cp::add_fn & add) { if(!*done) { *done = true; add(q); } };
            }, 15);
        }, 50);
    }
}
// A valid cut violated by exactly viol at the LP point, inside the solver's FEASTOL when viol < 1e-6: a cover relaxed by slack.
// Valid because relaxing a valid <= keeps it valid. Offered at every call, new at each point.
static void tolerance_zone() {
    for(const char * spec : {"R1,first2"}) for(const double viol : {1e-4, 1e-5, 2e-6, 5e-7, 1e-7, 1e-8, 2e-9}) for(unsigned seed : {1u, 2u}) {
        const std::string sp = spec;
        const std::string name = std::format("cover relaxed to violation {} seed={}", viol, seed);
        isolated(name + " [" + sp + "]", [&] {
            kase(name, sp, [&](M & m) -> cp::separator {
                auto [x, col, c, a, cap] = tree_block(m, seed, 40, 5);
                auto inner = covers(a, cap, col);
                return [inner, viol](const std::vector<double> & p, const cp::add_fn & add) {
                    inner(p, [&](const cut & q) { cut d = q; d.rhs += q.violation(p) - viol; add(d); return 0; });
                };
            }, 10);
        }, 40);
    }
}
// Big coefficient, tolerance in the wrapper: objective cut scaled, with absolute and relative sanitize tolerances.
static void big_tol() {
    for(const char * spec : {"R1,first2"}) for(unsigned seed : {1u, 2u, 3u}) for(const double frac : {1e2, 1e4, 1e6, 1e8}) {
        const std::string sp = spec;
        double truth;
        { M m; m.set_thread_limit(1); m.set_time_limit(duration<double>(30)); tree_block(m, seed, 40, 5); m.solve(); truth = m.get_solution_value(); }
        const std::string name = std::format("objective cut x {} seed={}", frac, seed);
        isolated(name + " [" + sp + "]", [&] {
            kase(name, sp, [&](M & m) -> cp::separator {
                auto [x, col, c, a, cap] = tree_block(m, seed, 40, 5);
                std::vector<double> v = c; for(auto & e : v) e *= frac;
                const cut k{col, v, 'L', truth * frac};
                return [k](const std::vector<double> &, const cp::add_fn & add) { add(k); };
            }, 10, truth);
        }, 40);
    }
}
// A valid cut with a coefficient under the matrix tolerance and a rhs above the feasibility tolerance.
// k integer in [0,10], x == 2e4 k, row x >= 1e4: every integer point has x >= 2e4. Cut: s x >= s 2e4.
static void small_coeff() {
    for(const char * spec : {"R1", "R1,keep_satisfied", "R2", "R2,keep_satisfied", "R2cr", "R2cr,keep_satisfied"}) for(const double sc : {1e-8, 1e-9, 1e-10, 1e-11, 1e-12}) for(const double big : {2e4, 2e6}) {
        const std::string sp = spec;
        const std::string name = std::format("{} x >= {} from x == {} k, x >= {}", sc, sc * big, big, big / 2);
        isolated(name + " [" + sp + "]", [&] {
            kase(name, sp, [&](M & m) -> cp::separator {
                auto [q, col, c, a, cap] = tree_block(m, 3, 30, 4);
                auto k = m.add_integer_variable({.lower_bound = 0., .upper_bound = 10.});
                auto x = m.add_variable({.lower_bound = 0., .upper_bound = big * 10});
                m.add_constraint(x - big * k == 0.0);
                m.add_constraint(1.0 * x >= big / 2);
                m.set_objective_coefficient(x, -1e-3 / big * 1000);
                const cut z{{m.native_id(x)}, {sc}, 'G', sc * big};
                auto done = std::make_shared<bool>(false);
                return [z, done](const std::vector<double> &, const cp::add_fn & add) { if(!*done) { *done = true; add(z); } };
            }, 10);
        }, 40);
    }
}
int main(int argc, char ** argv) {
    const std::string w = argc > 1 ? argv[1] : "";
    { M m; std::println(stderr, "release {}", vx::version(m)); }
    vx::quiet_flush = true;
    if(w == "dfj") reduced_dfj({"R1", "R1,add2", "R2", "R2cr"}, {5u, 1u});
    if(w == "dfj2") reduced_dfj({"R2", "R2cr"}, {5u});
    if(w == "scale") scaled_reoffer();
    if(w == "near") near_dup();
    if(w == "obj") objective_tight();
    if(w == "one") one_term_and_prune();
    if(w == "implied") implied_once();
    if(w == "zone") tolerance_zone();
    if(w == "bigtol") big_tol();
    if(w == "small") small_coeff();
}
