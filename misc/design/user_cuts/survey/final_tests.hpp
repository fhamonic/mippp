#pragma once
// Experiments (2)-(5) on the single preparation, the same on every backend. A backend
// file supplies bed<M>: its recipe (which calls fp::prepare), one thread, presolve off.
// Every instance has a key; final_oracle.txt holds Gurobi's no-hook optimum for each key
// (MIPGap 0), written by final_conf_gurobi oracle.
#include <fstream>
#include <sstream>
#include <sys/wait.h>
#include <unistd.h>

#include "cut_conformance.hpp"
#include "final_prepare.hpp"

namespace ft {
using namespace cp;
using std::chrono::duration;

template <class M>
struct bed {
    std::string name;
    std::function<void(M &, separator)> attach;
    std::function<void(M &)> one_thread;
    std::function<void(M &)> presolve_off;
    std::function<std::string()> recipe_line;  // the recipe's own counters for the last solve
    double limit = 60;
};

inline const char * oracle_path() { return "/tmp/claude-1000/-home-plaiseek-Projects-mippp/b99a8387-c5e9-4bc6-9141-d4502a992e5d/scratchpad/user_cuts/final_oracle.txt"; }
inline std::map<std::string, double> & oracle() {
    static std::map<std::string, double> o = [] {
        std::map<std::string, double> r;
        std::ifstream f(oracle_path());
        std::string k; double v;
        while(f >> k >> v) r[k] = v;
        return r;
    }();
    return o;
}
inline double oracle_of(const std::string & k) { auto it = oracle().find(k); return it == oracle().end() ? NAN : it->second; }

// ------------------------------------------------------------------ instances
// C5's covers, with the separator's own minimum violation exposed.
template <class M>
separator covers(M & m, const std::size_t cols, const std::size_t rows, const unsigned seed, const double min_viol = 1e-4, std::vector<int> * out_cols = nullptr) {
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
    m.set_objective(xsum(ids, [&](std::size_t j) { return c[j] * x(j); }) + 100.0);
    for(std::size_t i = 0; i < rows; ++i) m.add_constraint(xsum(ids, [&, i](std::size_t j) { return a[i][j] * x(j); }) <= cap[i]);
    std::vector<int> col;
    for(std::size_t j = 0; j < cols; ++j) col.push_back(m.native_id(x(j)));
    if(out_cols) *out_cols = col;
    return separator([a, cap, col, min_viol](const std::vector<double> & p, const add_fn & add) {
        for(std::size_t i = 0; i < a.size(); ++i) {
            std::vector<std::size_t> order(col.size());
            for(std::size_t j = 0; j < order.size(); ++j) order[j] = j;
            std::ranges::sort(order, [&](std::size_t l, std::size_t r) { return p[static_cast<std::size_t>(col[l])] > p[static_cast<std::size_t>(col[r])]; });
            double w = 0; raw_cut cut{{}, {}, 'L', 0};
            for(std::size_t j : order) {
                w += a[i][j]; cut.ind.push_back(col[j]); cut.val.push_back(1);
                if(w > cap[i]) { cut.rhs = static_cast<double>(cut.ind.size()) - 1; if(cut.violation(p) > min_viol) add(cut); break; }
            }
        }
    });
}
// C10's CG and cover fuzz, same exposure.
template <class M>
separator cgfuzz(M & m, const int seed, const double min_viol = 1e-4) {
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
    return separator([a, b, col, gen, mx, min_viol](const std::vector<double> & p, const add_fn & add) {
        std::lock_guard l(*mx);
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
            if(cut.violation(p) > min_viol) { add(cut); ++added; }
        }
        for(std::size_t i : tight) {
            std::vector<std::size_t> order;
            double best = 0;
            for(std::size_t j = 0; j < col.size(); ++j) { if(a[i][j] > 0) order.push_back(j); else best += a[i][j]; }
            std::ranges::sort(order, [&](std::size_t l, std::size_t r) { return p[static_cast<std::size_t>(col[l])] > p[static_cast<std::size_t>(col[r])]; });
            raw_cut cover{{}, {}, 'L', 0};
            for(std::size_t j : order) {
                best += a[i][j]; cover.ind.push_back(col[j]); cover.val.push_back(1);
                if(best > b[i] + 1e-9) { cover.rhs = static_cast<double>(cover.ind.size()) - 1; if(cover.violation(p) > min_viol) add(cover); break; }
            }
        }
    });
}
template <class M>
separator atsp_dfj(M & m, const int n, const unsigned seed) { auto a = build_atsp(m, n, false, seed); return cf::dfj<M>(a); }

// Fixed-charge transportation, x continuous. The model links x <= Mmodel y (or the tight
// min(s_i, d_j) y for the oracle); the separator offers x - Mcut y <= 0, valid because
// x <= min(s_i, d_j) <= Mcut, and violated wherever the LP has y = x / Mmodel.
// unit scales supplies and demands, so x reaches about Mcut when unit = Mcut / 30; the
// objective is c x + unit f y.
template <class M>
separator fctp(M & m, const unsigned seed, const double unit, const double Mmodel, const double Mcut, const bool tight_model) {
    const std::size_t S = 4, D = 8;
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> dem(5, 30), cost(1, 20), fix(40, 200);
    std::vector<double> d(D), s(S);
    double total = 0;
    for(auto & v : d) { v = dem(rng); total += v; }
    for(auto & v : s) v = std::ceil(total * 1.2 / static_cast<double>(S));
    for(auto & v : d) v *= unit;
    for(auto & v : s) v *= unit;
    std::vector<double> cx(S * D), cy(S * D);
    for(auto & v : cx) v = cost(rng);
    for(auto & v : cy) v = fix(rng) * unit;
    auto x = m.add_variables(S * D);
    auto y = m.add_binary_variables(S * D);
    const auto idx = [](std::size_t i, std::size_t j) { return i * D + j; };
    const auto all = std::views::iota(std::size_t{0}, S * D);
    const auto Is = std::views::iota(std::size_t{0}, S);
    const auto Js = std::views::iota(std::size_t{0}, D);
    m.set_minimization();
    m.set_objective(xsum(all, [&](std::size_t a) { return cx[a] * x(a) + cy[a] * y(a); }));
    for(std::size_t i : Is) m.add_constraint(xsum(Js, [&, i](std::size_t j) { return 1.0 * x(idx(i, j)); }) <= s[i]);
    for(std::size_t j : Js) m.add_constraint(xsum(Is, [&, j](std::size_t i) { return 1.0 * x(idx(i, j)); }) == d[j]);
    std::vector<int> xc, yc;
    for(std::size_t a : all) { xc.push_back(m.native_id(x(a))); yc.push_back(m.native_id(y(a))); }
    for(std::size_t i : Is) for(std::size_t j : Js) {
        const double link = tight_model ? std::min(s[i], d[j]) : Mmodel;
        m.add_constraint(x(idx(i, j)) - link * y(idx(i, j)) <= 0.0);
    }
    return separator([xc, yc, Mcut](const std::vector<double> & p, const add_fn & add) {
        std::vector<std::pair<double, std::size_t>> v;
        for(std::size_t a = 0; a < xc.size(); ++a) {
            const double viol = p[static_cast<std::size_t>(xc[a])] - Mcut * p[static_cast<std::size_t>(yc[a])];
            if(viol > 1e-6) v.emplace_back(viol, a);
        }
        std::ranges::sort(v, std::greater<>{});
        for(std::size_t k = 0; k < v.size() && k < 10; ++k) add(raw_cut{{xc[v[k].second], yc[v[k].second]}, {1.0, -Mcut}, 'L', 0.0});
    });
}

// The SCIP skeptic's TINY2, with the cut written large: Mbig (x_cycle) - w <= 2 Mbig, w = 3 Mbig z.
// Valid: z = 0 gives w = 0 and the odd cycle allows 2; z = 1 allows all 5. Scaling to
// max|a| = 1 gives sum x_cycle - w / Mbig <= 2: a tiny term whose silent removal makes the cut
// sum x_cycle <= 2, invalid where z = 1 is optimal.
template <class M>
separator tiny2(M & m, const unsigned seed, const double Mbig) {
    std::mt19937 rng(seed);
    const std::size_t N = 40;
    auto x = m.add_binary_variables(N);
    auto zv = m.add_binary_variables(1);
    const double U = 3.0 * Mbig;
    auto wv = m.add_variables(1, {.obj_coef = 0, .lower_bound = 0, .upper_bound = U});
    const auto z = zv(std::size_t{0}), w = wv(std::size_t{0});
    std::uniform_int_distribution<int> W(1, 9);
    std::vector<double> c(N);
    for(auto & v : c) v = W(rng);
    for(std::size_t i = 0; i < 5; ++i) { c[i] = 12; c[5 + i] = 0.1; }
    m.set_maximization();
    m.set_objective(xsum(std::views::iota(std::size_t{0}, N), [&](std::size_t i) { return c[i] * x(i); }) - 33.0 * z);
    for(std::size_t i = 0; i < 5; ++i) m.add_constraint(x(i) + x((i + 1) % 5) - 1.0 * z <= 1);
    m.add_constraint(w - U * z == 0);
    std::uniform_int_distribution<std::size_t> P(5, N - 1);
    for(int k = 0; k < 90; ++k) { const auto i = P(rng), j = P(rng); if(i != j) m.add_constraint(x(i) + x(j) <= 1); }
    for(std::size_t i = 0; i < 5; ++i) m.add_constraint(x(i) + x(5 + i) <= 1);
    std::vector<int> cyc;
    for(std::size_t i = 0; i < 5; ++i) cyc.push_back(m.native_id(x(i)));
    const int iw = m.native_id(w);
    return separator([cyc, iw, Mbig](const std::vector<double> & p, const add_fn & add) {
        raw_cut cut{cyc, {Mbig, Mbig, Mbig, Mbig, Mbig}, 'L', 2.0 * Mbig};
        cut.ind.push_back(iw); cut.val.push_back(-1.0);
        if(cut.violation(p) > 1e-6 * Mbig) add(cut);
    });
}

// Fixed-charge transportation with the model's link at the total demand (loose but sane:
// no trickle flow) and a separator for the tight links x - min(s_i, d_j) y <= 0, whose
// coefficients range over 5..30: the scaling step changes every one of them.
template <class M>
separator fctp_links(M & m, const unsigned seed, const std::size_t S, const std::size_t D, const double min_viol = 1e-4) {
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> dem(5, 30), cost(1, 20), fix(40, 200);
    std::vector<double> d(D), s(S);
    double total = 0;
    for(auto & v : d) { v = dem(rng); total += v; }
    for(auto & v : s) v = std::ceil(total * 1.2 / static_cast<double>(S));
    std::vector<double> cx(S * D), cy(S * D);
    for(auto & v : cx) v = cost(rng);
    for(auto & v : cy) v = fix(rng);
    auto x = m.add_variables(S * D);
    auto y = m.add_binary_variables(S * D);
    const auto idx = [D](std::size_t i, std::size_t j) { return i * D + j; };
    const auto all = std::views::iota(std::size_t{0}, S * D);
    const auto Is = std::views::iota(std::size_t{0}, S);
    const auto Js = std::views::iota(std::size_t{0}, D);
    m.set_minimization();
    m.set_objective(xsum(all, [&](std::size_t a) { return cx[a] * x(a) + cy[a] * y(a); }));
    for(std::size_t i : Is) m.add_constraint(xsum(Js, [&, i](std::size_t j) { return 1.0 * x(idx(i, j)); }) <= s[i]);
    for(std::size_t j : Js) m.add_constraint(xsum(Is, [&, j](std::size_t i) { return 1.0 * x(idx(i, j)); }) == d[j]);
    std::vector<int> xc, yc; std::vector<double> tight;
    for(std::size_t i : Is) for(std::size_t j : Js) {
        xc.push_back(m.native_id(x(idx(i, j)))); yc.push_back(m.native_id(y(idx(i, j)))); tight.push_back(std::min(s[i], d[j]));
        m.add_constraint(x(idx(i, j)) - total * y(idx(i, j)) <= 0.0);
    }
    return separator([xc, yc, tight, min_viol](const std::vector<double> & p, const add_fn & add) {
        std::vector<std::pair<double, std::size_t>> v;
        for(std::size_t a = 0; a < xc.size(); ++a) {
            const double viol = p[static_cast<std::size_t>(xc[a])] - tight[a] * p[static_cast<std::size_t>(yc[a])];
            if(viol > min_viol) v.emplace_back(viol, a);
        }
        std::ranges::sort(v, std::greater<>{});
        for(std::size_t k = 0; k < v.size() && k < 10; ++k) add(raw_cut{{xc[v[k].second], yc[v[k].second]}, {1.0, -tight[v[k].second]}, 'L', 0.0});
    });
}
// General-integer knapsack rows, x in [0, 7], with Chvatal-Gomory cuts floor(a/d) x <= floor(b/d):
// coefficients up to 30, so scaling changes them too (verify_conf_cplex X4 without the duplicates).
template <class M>
separator genint_cg(M & m, const unsigned seed, const double min_viol = 1e-4) {
    const std::size_t C = 25, R = 6;
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> A(1, 60), P(5, 80);
    std::vector<std::vector<double>> a(R, std::vector<double>(C));
    for(auto & r : a) for(auto & v : r) v = A(rng);
    std::vector<double> c(C), b(R);
    for(auto & v : c) v = P(rng);
    for(std::size_t i = 0; i < R; ++i) { double t = 0; for(double v : a[i]) t += v; b[i] = std::floor(t * 1.3); }
    auto x = m.add_integer_variables(C, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 7});
    const auto ids = std::views::iota(std::size_t{0}, C);
    m.set_maximization();
    m.set_objective(xsum(ids, [&](std::size_t j) { return c[j] * x(j); }));
    for(std::size_t i = 0; i < R; ++i) m.add_constraint(xsum(ids, [&, i](std::size_t j) { return a[i][j] * x(j); }) <= b[i]);
    std::vector<int> col;
    for(std::size_t j = 0; j < C; ++j) col.push_back(m.native_id(x(j)));
    auto gen = std::make_shared<std::mt19937>(seed * 31u);
    auto mx = std::make_shared<std::mutex>();
    return separator([a, b, col, gen, mx, min_viol](const std::vector<double> & p, const add_fn & add) {
        std::lock_guard l(*mx);
        const int ds[] = {2, 3, 4, 5, 7, 11, 13, 17, 19, 23};
        for(int attempt = 0, added = 0; attempt < 200 && added < 3; ++attempt) {
            const std::size_t i = (*gen)() % a.size();
            const double d = ds[(*gen)() % std::size(ds)];
            raw_cut cut{{}, {}, 'L', std::floor(b[i] / d)};
            for(std::size_t j = 0; j < col.size(); ++j) {
                const double f = std::floor(a[i][j] / d);
                if(f == 0) continue;
                cut.ind.push_back(col[j]); cut.val.push_back(f);
            }
            if(cut.violation(p) > min_viol) { add(cut); ++added; }
        }
    });
}
// Multicommodity fixed-charge network design, the COPT skeptic's build_mcnd (verify_copt_common.hpp):
// a ring both ways plus random arcs, the capacity row is the only link; the separator offers the
// strong links x^k_e - d_k y_e <= 0, d_k in 1..9, which no presolve derives.
template <class M>
separator mcnd(M & m, const unsigned seed, const std::size_t N = 10, const std::size_t extra = 15, const std::size_t K = 10, const double min_viol = 1e-4) {
    std::mt19937 rng(seed);
    std::uniform_int_distribution<std::size_t> node(0, N - 1);
    std::uniform_int_distribution<int> dem(1, 9), cost(1, 8), fix(20, 90);
    std::vector<std::pair<std::size_t, std::size_t>> arcs;
    for(std::size_t i = 0; i < N; ++i) { arcs.emplace_back(i, (i + 1) % N); arcs.emplace_back((i + 1) % N, i); }
    for(std::size_t a = 0; a < extra; ++a) { auto u = node(rng), v = node(rng); if(u != v) arcs.emplace_back(u, v); }
    const std::size_t A = arcs.size();
    std::vector<std::tuple<std::size_t, std::size_t, double>> com;
    double total = 0;
    while(com.size() < K) { auto o = node(rng), t = node(rng); if(o == t) continue; const double dk = dem(rng); com.emplace_back(o, t, dk); total += dk; }
    auto yv = m.add_binary_variables(A);
    auto xv = m.add_variables(K * A);
    const auto idx = [A](std::size_t k, std::size_t e) { return k * A + e; };
    const auto Es = std::views::iota(std::size_t{0}, A);
    const auto Ks = std::views::iota(std::size_t{0}, K);
    std::vector<double> ce(A), fe(A);
    for(auto & v : ce) v = cost(rng);
    for(auto & v : fe) v = fix(rng);
    const double cap = std::ceil(total * 0.5);
    m.set_minimization();
    m.set_objective(xsum(Es, [&](std::size_t e) { return fe[e] * yv(e); }) +
                    xsum(Ks, [&](std::size_t k) { return xsum(Es, [&, k](std::size_t e) { return ce[e] * xv(idx(k, e)); }); }));
    for(std::size_t k : Ks) {
        auto [o, t, dk] = com[k];
        for(std::size_t v = 0; v < N; ++v) {
            const double b = v == o ? dk : v == t ? -dk : 0.0;
            m.add_constraint(xsum(Es | std::views::filter([&, v](std::size_t e) { return arcs[e].first == v; }), [&, k](std::size_t e) { return 1.0 * xv(idx(k, e)); })
                           - xsum(Es | std::views::filter([&, v](std::size_t e) { return arcs[e].second == v; }), [&, k](std::size_t e) { return 1.0 * xv(idx(k, e)); }) == b);
        }
        for(std::size_t e : Es) m.set_variable_upper_bound(xv(idx(k, e)), dk);
    }
    for(std::size_t e : Es) m.add_constraint(xsum(Ks, [&, e](std::size_t k) { return 1.0 * xv(idx(k, e)); }) - cap * yv(e) <= 0.0);
    std::vector<int> yc; std::vector<std::vector<int>> xc(K); std::vector<double> d;
    for(std::size_t e : Es) yc.push_back(m.native_id(yv(e)));
    for(std::size_t k : Ks) { d.push_back(std::get<2>(com[k])); for(std::size_t e : Es) xc[k].push_back(m.native_id(xv(idx(k, e)))); }
    return separator([yc, xc, d, min_viol](const std::vector<double> & p, const add_fn & add) {
        std::vector<std::tuple<double, std::size_t, std::size_t>> v;
        for(std::size_t k = 0; k < xc.size(); ++k) for(std::size_t e = 0; e < yc.size(); ++e) {
            const double g = p[static_cast<std::size_t>(xc[k][e])] - d[k] * p[static_cast<std::size_t>(yc[e])];
            if(g > min_viol) v.emplace_back(g, k, e);
        }
        std::ranges::sort(v, std::greater<>{});
        for(std::size_t q = 0; q < v.size() && q < 10; ++q) { auto [g, k, e] = v[q]; add(raw_cut{{xc[k][e], yc[e]}, {1.0, -d[k]}, 'L', 0.0}); }
    });
}
// Offers, for every violated cut of s, only a weakened copy that the point violates by
// v times max|a|: still valid, and below the 1e-6 drop after scaling when v < 1e-6.
inline separator nearsat(separator s, const double v) {
    return [s, v](const std::vector<double> & x, const add_fn & add) {
        s(x, [&](const raw_cut & c) {
            double big = 0; for(double w : c.val) big = std::max(big, std::fabs(w));
            const double viol = c.violation(x), want = v * big;
            if(viol <= want || c.sense == 'E') return 0;
            raw_cut d = c;
            d.rhs += c.sense == 'L' ? viol - want : -(viol - want);
            return add(d);
        });
    };
}

inline separator scaled_by(separator s, const double f) {
    return [s, f](const std::vector<double> & x, const add_fn & add) {
        s(x, [&](const raw_cut & c) { raw_cut d = c; for(double & v : d.val) v *= f; d.rhs *= f; return add(d); });
    };
}
// Re-offers every cut it ever found, at every call: the usual stateless separator.
inline separator eager(separator s) {
    auto all = std::make_shared<std::vector<raw_cut>>();
    auto mx = std::make_shared<std::mutex>();
    return [s, all, mx](const std::vector<double> & x, const add_fn & add) {
        std::vector<raw_cut> fresh;
        s(x, [&](const raw_cut & c) { fresh.push_back(c); return 0; });
        std::lock_guard l(*mx);
        for(auto & c : fresh) all->push_back(std::move(c));
        for(const auto & c : *all) add(c);
    };
}

// --------------------------------------------------------------------- runs
struct outcome {
    std::string status;
    bool optimal = false;
    double value = NAN, dual = NAN, secs = 0;
    long nodes = -1, calls = 0;
    std::string prep, rline;
};
template <class M>
outcome solve_one_here(const bed<M> & B, const std::function<separator(M &)> & build, const bool hook, const bool presolve_off, const bool threads1);
// FT_FORK=1 runs each solve in a child, so that a solver abort is one verdict, not the end of the run.
template <class M>
outcome solve_one(const bed<M> & B, const std::type_identity_t<std::function<separator(M &)>> & build, const bool hook, const bool presolve_off = false, const bool threads1 = true) {
    if(!std::getenv("FT_FORK")) return solve_one_here<M>(B, build, hook, presolve_off, threads1);
    int fd[2];
    if(pipe(fd) != 0) std::abort();
    std::fflush(stdout); std::fflush(stderr);
    const pid_t pid = fork();
    if(pid == 0) {
        close(fd[0]);
        const outcome o = solve_one_here<M>(B, build, hook, presolve_off, threads1);
        const std::string line = std::format("{}\x1f{}\x1f{:.17g}\x1f{:.17g}\x1f{:.17g}\x1f{}\x1f{}\x1f{}\x1f{}", o.status, int(o.optimal), o.value, o.dual, o.secs, o.nodes, o.calls, o.prep, o.rline);
        std::size_t done = 0;
        while(done < line.size()) { const auto w = write(fd[1], line.data() + done, line.size() - done); if(w <= 0) break; done += static_cast<std::size_t>(w); }
        close(fd[1]);
        std::fflush(stdout); std::fflush(stderr);
        _exit(0);
    }
    close(fd[1]);
    std::string buf; char tmp[4096];
    for(ssize_t r; (r = read(fd[0], tmp, sizeof tmp)) > 0;) buf.append(tmp, static_cast<std::size_t>(r));
    close(fd[0]);
    int ws = 0; waitpid(pid, &ws, 0);
    outcome o;
    if(WIFSIGNALED(ws) || buf.empty()) { o.status = std::format("ABORT(signal {})", WIFSIGNALED(ws) ? WTERMSIG(ws) : -1); return o; }
    std::vector<std::string> f; std::string cur;
    for(char ch : buf) { if(ch == '\x1f') { f.push_back(cur); cur.clear(); } else cur += ch; }
    f.push_back(cur);
    if(f.size() < 9) { o.status = "ABORT(short)"; return o; }
    o.status = f[0]; o.optimal = f[1] == "1"; o.value = std::stod(f[2]); o.dual = std::stod(f[3]); o.secs = std::stod(f[4]);
    o.nodes = std::stol(f[5]); o.calls = std::stol(f[6]); o.prep = f[7]; o.rline = f[8];
    return o;
}
template <class M>
outcome solve_one_here(const bed<M> & B, const std::function<separator(M &)> & build, const bool hook, const bool presolve_off, const bool threads1) {
    M m;
    if(threads1) B.one_thread(m);
    if(presolve_off && B.presolve_off) B.presolve_off(m);
    auto sep = build(m);
    m.set_time_limit(duration<double>(B.limit));
    auto s = std::make_shared<stats>();
    fp::stats().reset();
    if(hook) B.attach(m, watched(sep, s));
    outcome o;
    o.secs = timed_solve(m);
    o.status = status_name(m.get_status());
    o.optimal = is_a<status::optimal>(m.get_status());
    o.value = cf::value_of(m);
    if constexpr(has_dual_bound<M>) { try { o.dual = m.get_dual_bound(); } catch(const std::exception &) {} }
    if constexpr(has_node_count<M>) { try { o.nodes = static_cast<long>(m.get_node_count()); } catch(const std::exception &) {} }
    o.calls = s->calls.load();
    o.prep = fp::stats().line();
    if(hook && B.recipe_line) o.rline = B.recipe_line();
    return o;
}
// OK: equal; GAP: within the 1e-4 relative gap the solvers default to; WRONG otherwise.
inline std::string judge(const outcome & o, const double ref) {
    if(std::isnan(ref)) return "NOREF";
    if(!o.optimal) return "NOTOPT";
    if(cf::same(o.value, ref)) return "OK";
    if(std::fabs(o.value - ref) <= 1e-4 * std::fabs(ref) + 1e-6) return "GAP";
    return "WRONG";
}
inline std::string fmt(const outcome & o) {
    return std::format("status={} value={:.10g} dual={:.10g} nodes={} t={:.2f}s calls={} {}{}", o.status, o.value, o.dual, o.nodes, o.secs, o.calls, o.prep, o.rline.empty() ? "" : " | " + o.rline);
}

inline std::string sci(double v) { return std::format("{:g}", v); }

// (2) scale sweep: valid cuts multiplied by s before the preparation.
template <class M>
void sweep(const bed<M> & B, const std::vector<double> & scales, const bool control_no_scale) {
    struct fam { std::string key; std::function<separator(M &)> build; };
    std::vector<fam> fams;
    for(unsigned seed = 1; seed <= 5; ++seed) fams.push_back({std::format("cov40x5s{}", seed), [seed](M & m) { return covers(m, 40, 5, seed); }});
    for(unsigned seed = 1; seed <= 5; ++seed) fams.push_back({std::format("atsp16s{}", seed), [seed](M & m) { return atsp_dfj(m, 16, seed); }});
    long bad = 0, runs = 0;
    for(auto & f : fams) {
        const auto base = solve_one(B, f.build, false);
        const double orc = oracle_of(f.key);
        std::println(stderr, "SWEEPBASE {} {} oracle={:.10g} base_vs_oracle={} | {}", B.name, f.key, orc, judge(base, orc), fmt(base));
        for(const bool ns : control_no_scale ? std::vector<bool>{false, true} : std::vector<bool>{false}) {
            fp::opts().scale = !ns;
            for(double s : scales) {
                const auto o = solve_one(B, [&](M & m) { return scaled_by(f.build(m), s); }, true);
                const auto vo = judge(o, orc), vb = judge(o, base.optimal ? base.value : NAN);
                ++runs; if(vo == "WRONG" || vo == "NOTOPT") ++bad;
                std::println(stderr, "SWEEP {} {} s={} cfg={}: vs_oracle={} vs_base={} | {}", B.name, f.key, sci(s), ns ? "NOSCALE" : "PIPE", vo, vb, fmt(o));
            }
            fp::opts().scale = true;
        }
    }
    std::println(stderr, "SWEEPTOTAL {} runs={} wrong_or_notopt={}", B.name, runs, bad);
}

// (3) big-M: flavors W (x up to 30, Mcut >> x) and S (x up to Mcut), Mmodel = 10 Mcut.
template <class M>
void bigm(const bed<M> & B, const std::vector<unsigned> & seeds) {
    for(const bool strong : {false, true}) for(unsigned seed : seeds) for(double Mc : {1e3, 1e6, 1e8, 1e10}) {
        const double unit = strong ? Mc / 30.0 : 1.0;
        const std::string key = std::format("fctp_u{}_s{}", sci(unit), seed);
        const double orc = oracle_of(key);
        for(const bool pre_off : {false, true}) {
            const auto build = [=](M & m) { return fctp(m, seed, unit, 10 * Mc, Mc, false); };
            const auto base = solve_one(B, build, false, pre_off);
            std::println(stderr, "BIGMBASE {} {} seed={} Mcut={} presolve={}: oracle={:.10g} vs_oracle={} | {}", B.name, strong ? "S" : "W", seed, sci(Mc), pre_off ? "off" : "on", orc, judge(base, orc), fmt(base));
            for(const bool ns : {false, true}) {
                fp::opts().scale = !ns;
                const auto o = solve_one(B, build, true, pre_off);
                fp::opts().scale = true;
                std::println(stderr, "BIGM {} {} seed={} Mcut={} presolve={} cfg={}: vs_oracle={} vs_base={} | {}", B.name, strong ? "S" : "W", seed, sci(Mc), pre_off ? "off" : "on", ns ? "NOSCALE" : "PIPE", judge(o, orc), judge(o, base.optimal ? base.value : NAN), fmt(o));
            }
        }
    }
}
// (3) more seeds on the S flavor, with the Cbc skeptic's geometric scaling as a third arm.
// The S model is the unit model scaled by unit in x and in the objective: its optimum is
// unit times Gurobi's fctp_u1 optimum.
struct arm { const char * name; bool scale; bool geometric; };
inline const arm arms3[] = {{"PIPE", true, false}, {"NOSCALE", false, false}, {"GEOM", true, true}};
template <class M>
void bigm_more(const bed<M> & B) {
    for(unsigned seed = 1; seed <= 10; ++seed) for(double Mc : {1e8, 1e9, 1e10}) {
        const double unit = Mc / 30.0;
        const double orc = unit * oracle_of(std::format("fctp_u1_s{}", seed));
        for(const bool pre_off : {false, true}) {
            const auto build = [=](M & m) { return fctp(m, seed, unit, 10 * Mc, Mc, false); };
            const auto base = solve_one(B, build, false, pre_off);
            std::println(stderr, "BIGMBASE {} S seed={} Mcut={} presolve={}: oracle={:.10g} vs_oracle={} | {}", B.name, seed, sci(Mc), pre_off ? "off" : "on", orc, judge(base, orc), fmt(base));
            for(const auto & a : arms3) {
                fp::opts().scale = a.scale; fp::opts().geometric = a.geometric;
                const auto o = solve_one(B, build, true, pre_off);
                fp::opts() = fp::options{};
                std::println(stderr, "BIGM {} S seed={} Mcut={} presolve={} cfg={}: vs_oracle={} vs_base={} | {}", B.name, seed, sci(Mc), pre_off ? "off" : "on", a.name, judge(o, orc), judge(o, base.optimal ? base.value : NAN), fmt(o));
            }
        }
    }
}
template <class M>
void tiny3(const bed<M> & B) {
    for(const bool pre_off : {false, true}) for(unsigned seed : {1u, 2u, 3u}) for(double Mb : {1e8, 1e10, 1e12}) {
        const double orc = oracle_of(std::format("tiny2_s{}_M{}", seed, sci(Mb)));
        const auto build = [=](M & m) { return tiny2(m, seed, Mb); };
        for(const auto & a : arms3) {
            fp::opts().scale = a.scale; fp::opts().geometric = a.geometric;
            const auto o = solve_one(B, build, true, pre_off);
            fp::opts() = fp::options{};
            std::println(stderr, "TINY {} seed={} M={} presolve={} cfg={}: vs_oracle={} | {}", B.name, seed, sci(Mb), pre_off ? "off" : "on", a.name, judge(o, orc), fmt(o));
        }
    }
}
// (3c) the GLPK skeptic's conditional form (verify_glpk_safe.cpp:172, bigm_cut): every violated
// cover sum_C x <= |C| - 1 is offered as sum_C x - K x_b <= |C| - 1 on a column b outside C that
// the point has at 0. Valid (weaker than the cover), violated exactly as much as the cover; after
// scaling to max|a| = 1 its violation is v / K, so the 1e-6 drop removes it once K > v / 1e-6.
inline separator conditional(separator s, const std::vector<int> & cols, const double K) {
    return [s, cols, K](const std::vector<double> & x, const add_fn & add) {
        s(x, [&](const raw_cut & c) {
            int b = -1;
            for(int j : cols) if(std::ranges::find(c.ind, j) == c.ind.end() && std::fabs(x[static_cast<std::size_t>(j)]) < 1e-9) { b = j; break; }
            if(b < 0 || c.sense != 'L') return 0;
            raw_cut d = c;
            d.ind.push_back(b); d.val.push_back(-K);
            return add(d);
        });
    };
}
template <class M>
void bigm_cond(const bed<M> & B) {
    const arm arms[] = {{"PIPE", true, false}, {"NOSCALE", false, false}, {"DROP9", true, false}};
    for(unsigned seed = 1; seed <= 5; ++seed) {
        const std::string key = std::format("cov40x5s{}", seed);
        const double orc = oracle_of(key);
        std::vector<int> cols;
        const auto build_plain = [&cols, seed](M & m) { return covers(m, 40, 5, seed, 1e-4, &cols); };
        const auto base = solve_one(B, build_plain, false);
        const auto plain = solve_one(B, build_plain, true);
        std::println(stderr, "CONDBASE {} {}: oracle={:.10g} base {} | {}", B.name, key, orc, judge(base, orc), fmt(base));
        std::println(stderr, "CONDPLAIN {} {}: vs_oracle={} | {}", B.name, key, judge(plain, orc), fmt(plain));
        for(double K : {1e3, 1e6, 1e8, 1e10}) for(const auto & a : arms) {
            fp::opts().scale = a.scale; if(std::string(a.name) == "DROP9") fp::opts().drop_tol = 1e-9;
            const auto o = solve_one(B, [&](M & m) { auto sep = build_plain(m); return conditional(sep, cols, K); }, true);
            fp::opts() = fp::options{};
            std::println(stderr, "COND {} {} K={} cfg={}: vs_oracle={} | {}", B.name, key, sci(K), a.name, judge(o, orc), fmt(o));
        }
    }
}
// (3b) the reverse direction: scaling makes a tiny negative term.
template <class M>
void tiny(const bed<M> & B, const std::vector<unsigned> & seeds) {
    for(const bool pre_off : {false, true}) for(unsigned seed : seeds) for(double Mb : {1e3, 1e6, 1e8, 1e10, 1e12}) {
        const std::string key = std::format("tiny2_s{}_M{}", seed, sci(Mb));
        const double orc = oracle_of(key);
        const auto build = [=](M & m) { return tiny2(m, seed, Mb); };
        const auto base = solve_one(B, build, false, pre_off);
        std::println(stderr, "TINYBASE {} seed={} M={} presolve={}: oracle={:.10g} vs_oracle={} | {}", B.name, seed, sci(Mb), pre_off ? "off" : "on", orc, judge(base, orc), fmt(base));
        for(const bool ns : {false, true}) {
            fp::opts().scale = !ns;
            const auto o = solve_one(B, build, true, pre_off);
            fp::opts().scale = true;
            std::println(stderr, "TINY {} seed={} M={} presolve={} cfg={}: vs_oracle={} vs_base={} | {}", B.name, seed, sci(Mb), pre_off ? "off" : "on", ns ? "NOSCALE" : "PIPE", judge(o, orc), judge(o, base.optimal ? base.value : NAN), fmt(o));
        }
    }
}
// (4) and (5): nodes and time, one thread, scale on/off and drop 1e-6/1e-9, plain and eager separators.
template <class M>
void harm(const bed<M> & B, const std::vector<std::pair<std::string, std::function<separator(M &, double)>>> & inst) {
    struct cfg { const char * name; bool scale; double drop; };
    const cfg cfgs[] = {{"PIPE", true, 1e-6}, {"NOSCALE", false, 1e-6}, {"DROP9", true, 1e-9}, {"OLD", false, 1e-9}};
    for(auto & [key, mk] : inst) {
        const double orc = oracle_of(key);
        const auto base = solve_one(B, [&](M & m) { return mk(m, 1e-4); }, false);
        std::println(stderr, "HARMBASE {} {}: oracle={:.10g} vs_oracle={} | {}", B.name, key, orc, judge(base, orc), fmt(base));
        for(const char * sm : {"plain", "eager", "nearsat"}) for(const auto & c : cfgs) {
            const std::string mode = sm;
            if(mode == "nearsat" && (c.name == std::string("NOSCALE") || c.name == std::string("OLD"))) continue;
            fp::opts().scale = c.scale; fp::opts().drop_tol = c.drop;
            const auto o = solve_one(B, [&](M & m) { return mode == "eager" ? eager(mk(m, 0.0)) : mode == "nearsat" ? nearsat(mk(m, 1e-4), 5e-7) : mk(m, 1e-4); }, true);
            fp::opts() = fp::options{};
            std::println(stderr, "HARM {} {} sep={} cfg={}: vs_oracle={} | {}", B.name, key, sm, c.name, judge(o, orc), fmt(o));
        }
    }
}
template <class M>
std::vector<std::pair<std::string, std::function<separator(M &, double)>>> harm_instances() {
    return {
        {"cov40x5s9", [](M & m, double v) { return covers(m, 40, 5, 9, v); }},
        {"cov40x5s10", [](M & m, double v) { return covers(m, 40, 5, 10, v); }},
        {"cg7", [](M & m, double v) { return cgfuzz(m, 7, v); }},
        {"cg23", [](M & m, double v) { return cgfuzz(m, 23, v); }},
        {"cg33", [](M & m, double v) { return cgfuzz(m, 33, v); }},
        {"mcnd12s1", [](M & m, double v) { return mcnd(m, 1, 12, 24, 15, v); }},
        {"mcnd12s2", [](M & m, double v) { return mcnd(m, 2, 12, 24, 15, v); }},
        {"mcnd12s3", [](M & m, double v) { return mcnd(m, 3, 12, 24, 15, v); }},
        {"gi25x6s1", [](M & m, double v) { return genint_cg(m, 1, v); }},
    };
}

inline std::vector<double> sweep_scales() { return {1e-8, 1e-4, 1e4, 1e6, 1e9, 1e12}; }

// Dispatches argv[1]: sweep, sweep_ns (with the no-scale control), bigm, tiny, harm.
template <class M>
bool run_mode(const bed<M> & B, const std::string & mode) {
    if(mode == "sweep") { sweep(B, sweep_scales(), false); return true; }
    if(mode == "sweep_ns") { sweep(B, sweep_scales(), true); return true; }
    if(mode == "bigm") { bigm(B, {1, 2, 3}); return true; }
    if(mode == "tiny") { tiny(B, {1, 2, 3}); return true; }
    if(mode == "harm") { harm(B, harm_instances<M>()); return true; }
    if(mode == "bigm_more") { bigm_more(B); return true; }
    if(mode == "bigm_cond") { bigm_cond(B); return true; }
    if(mode == "tiny3") { tiny3(B); return true; }
    if(mode == "harm_new") { auto all = harm_instances<M>(); all.erase(all.begin(), all.begin() + 5); harm(B, all); return true; }
    if(mode == "harm_cov") { auto all = harm_instances<M>(); all.resize(5); harm(B, all); return true; }
    return false;
}
}  // namespace ft
