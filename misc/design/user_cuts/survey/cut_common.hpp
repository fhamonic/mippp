#pragma once
// Shared harness for the user-cut survey. A backend probe supplies its native
// hook (attach), its native settings (scope_settings) and the list of cut modes
// it can add; the experiments below are the same on every backend, so results
// compare. Results go to stderr, solver logs to stdout.
//
// A hook must: run only on an optimal node relaxation; read the relaxation point
// over the ORIGINAL columns, indexed by m.native_id(variable); call the separator;
// never throw out of the native callback (add returns the native error code).
#include <cxxabi.h>
#include <dlfcn.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <print>
#include <random>
#include <ranges>
#include <set>
#include <string>
#include <thread>
#include <typeinfo>
#include <variant>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"

using namespace mippp;
using namespace mippp::operators;
using clk = std::chrono::steady_clock;

namespace cp {

inline void * sym(const std::filesystem::path & lib, const char * name) {
    void * h = dlopen(lib.c_str(), RTLD_NOW | RTLD_NOLOAD);
    if(!h) { std::println(stderr, "dlopen NOLOAD failed: {}", dlerror()); std::exit(2); }
    void * f = dlsym(h, name);
    if(!f) { std::println(stderr, "missing symbol {}", name); std::exit(2); }
    return f;
}
template <class F>
F symf(const std::filesystem::path & lib, const char * name) {
    return reinterpret_cast<F>(sym(lib, name));
}
inline std::string short_name(const char * mangled) {
    int st = 0;
    char * d = abi::__cxa_demangle(mangled, nullptr, nullptr, &st);
    std::string s = d ? d : mangled;
    std::free(d);
    auto p = s.rfind("::");
    return p == std::string::npos ? s : s.substr(p + 2);
}
template <class St>
std::string status_name(const St & s) {
    return std::visit([](const auto & t) {
        std::string n = short_name(typeid(t).name());
        if constexpr(requires { t.solution_available; })
            n += std::format("{{sol={}}}", bool(t.solution_available));
        return n;
    }, s);
}

struct raw_cut {
    std::vector<int> ind;
    std::vector<double> val;
    char sense;  // 'L', 'G' or 'E'
    double rhs;
    double activity(const std::vector<double> & x) const {
        double a = 0;
        for(std::size_t k = 0; k < ind.size(); ++k) a += val[k] * x[static_cast<std::size_t>(ind[k])];
        return a;
    }
    // positive when x violates the cut
    double violation(const std::vector<double> & x) const {
        const double a = activity(x);
        return sense == 'L' ? a - rhs : sense == 'G' ? rhs - a : std::fabs(a - rhs);
    }
};
using add_fn = std::function<int(const raw_cut &)>;
using separator = std::function<void(const std::vector<double> &, const add_fn &)>;

struct stats {
    std::atomic<long> calls{0}, cuts{0}, add_errors{0}, overlaps{0}, integral{0}, bad_points{0};
    std::atomic<int> inflight{0}, max_inflight{0};
    std::mutex m;
    std::set<std::thread::id> threads;
    std::vector<double> objs;  // relaxation objective of the first calls
    int last_error = 0;
    std::string line() {
        double first = objs.empty() ? NAN : objs.front(), lo = first, hi = first;
        for(double v : objs) { lo = std::min(lo, v); hi = std::max(hi, v); }
        return std::format("calls={} cuts={} add_errors={}(last {}) integral_points={} bad_points={} threads={} overlaps={} max_inflight={} obj_first={:.6g} obj_range_first{}=[{:.6g},{:.6g}]",
            calls.load(), cuts.load(), add_errors.load(), last_error, integral.load(), bad_points.load(), threads.size(), overlaps.load(), max_inflight.load(), first, objs.size(), lo, hi);
    }
};
// Wraps a separator with the bookkeeping every experiment reports. spin widens
// the window in which two calls could overlap.
inline separator watched(separator inner, std::shared_ptr<stats> s,
                         std::function<double(const std::vector<double> &)> objective = {},
                         std::function<bool(const std::vector<double> &)> point_ok = {},
                         std::function<bool(const std::vector<double> &)> integral = {},
                         std::chrono::microseconds spin = std::chrono::microseconds(0)) {
    return [=](const std::vector<double> & x, const add_fn & add) {
        const int now = s->inflight.fetch_add(1) + 1;
        if(now > 1) s->overlaps.fetch_add(1);
        int seen = s->max_inflight.load();
        while(now > seen && !s->max_inflight.compare_exchange_weak(seen, now)) {}
        s->calls.fetch_add(1);
        {
            std::lock_guard l(s->m);
            s->threads.insert(std::this_thread::get_id());
            if(objective && s->objs.size() < 50) s->objs.push_back(objective(x));
        }
        if(point_ok && !point_ok(x)) s->bad_points.fetch_add(1);
        if(integral && integral(x)) s->integral.fetch_add(1);
        if(spin.count()) {
            const auto until = clk::now() + spin;
            while(clk::now() < until) {}
        }
        const add_fn counting = [&](const raw_cut & c) {
            const int e = add(c);
            if(e) { s->add_errors.fetch_add(1); std::lock_guard l(s->m); s->last_error = e; }
            else s->cuts.fetch_add(1);
            return e;
        };
        inner(x, counting);
        s->inflight.fetch_sub(1);
    };
}

// ---------------------------------------------------------------- gadgets ---
// Independent one-binary gadgets, maximized. Each has two continuous variables
// whose bound functions peak at opposite ends of y:
//   z_a <= c min(A0 + 20 y, 9 - 8 y) ... value va0 at y=0, 1 at y=1
//   z_b <= c min(1 + 30 y, 11 - 8 y) ... value 1 at y=0, vb1 at y=1
// Bound tightening by probing only gives z_a <= c max(..), z_b <= c max(..), and
// the LP still reaches both with y strictly inside (0,1). So y integral in a
// relaxation means branching fixed it at this node, and only branching or a real
// cut closes the gap: every integer assignment loses against the LP.
// Gadget 0 is the largest and has the largest gap, so that y[0] splits the tree
// first on every rule tried.
//   flip=false: the optimum has y[0]=0, and y[0]=1 is worse by 2 c0
//   flip=true : the optimum has y[0]=1, and y[0]=0 is worse by 2 c0
// A cut Sum z <= opt-1 is therefore valid in the worse subtree and globally
// invalid. Only meaningful under scope_settings (no presolve, cuts, heuristics).
// Three more gadgets, tied to y[0] so that they are free only in the worse
// subtree, are worth nothing to an integer point and a lot to the LP. The worse
// subtree then has the better LP bound, so best-first visits it first, before
// any incumbent exists; without them it is visited last and a global cut can no
// longer lose the optimum, which already sits in the incumbent.
template <class M>
struct gadgets {
    using constraint_t = model_constraint_t<M>;
    int k;
    bool flip;
    std::vector<int> y, za, zb, z;
    int dummy;
    std::vector<double> c, total0, total1;  // objective of gadget i at y=0 and y=1, without c
    std::vector<constraint_t> rows;  // four per gadget
    double opt = 0;
    int worse_state() const { return flip ? 0 : 1; }
    int state(const std::vector<double> & x) const {
        const double v = x[static_cast<std::size_t>(y[0])];
        return v < 1e-6 ? 0 : v > 1 - 1e-6 ? 1 : -1;
    }
    raw_cut invalid_global_cut() const {
        return raw_cut{z, std::vector<double>(z.size(), 1.0), 'L', opt - 1};
    }
    // z_a + z_b <= c (t0 + (t1 - t0) y): the convex hull facet of gadget i, valid
    // everywhere, violated by the LP while y_i is free.
    raw_cut hull_cut(int i) const {
        const auto I = static_cast<std::size_t>(i);
        return raw_cut{{za[I], zb[I], y[I]}, {1.0, 1.0, -c[I] * (total1[I] - total0[I])}, 'L', c[I] * total0[I]};
    }
    double objective(const std::vector<double> & x) const {
        double s = 0;
        for(int j : z) s += x[static_cast<std::size_t>(j)];
        return s;
    }
    bool point_ok(const std::vector<double> & x) const {
        for(int j : z) if(x[static_cast<std::size_t>(j)] < -1e-6) return false;
        for(int j : y) if(x[static_cast<std::size_t>(j)] < -1e-6 || x[static_cast<std::size_t>(j)] > 1 + 1e-6) return false;
        return true;
    }
};
template <class M>
gadgets<M> build_gadgets(M & m, const int k, const bool flip) {
    gadgets<M> g{};
    g.k = k; g.flip = flip;
    auto y = m.add_binary_variables(static_cast<std::size_t>(k));
    auto za = m.add_variables(static_cast<std::size_t>(k));
    auto zb = m.add_variables(static_cast<std::size_t>(k));
    auto d = m.add_variables(1, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 1});
    const auto Is = std::views::iota(std::size_t{0}, static_cast<std::size_t>(k));
    m.set_maximization();
    m.set_objective(xsum(Is, [&](std::size_t i) { return za(i) + zb(i); }));
    g.dummy = m.native_id(d(std::size_t{0}));
    for(std::size_t I : Is) {
        const double c = I == 0 ? 5.0 : 1.0 + 0.03 * static_cast<double>(I);
        // z_a: va0 at y=0, 1 at y=1; z_b: 1 at y=0, vb1 at y=1
        const double va0 = (I == 0 && !flip) ? 5 : 3;
        const double vb1 = (I == 0 && flip) ? 5 : 3;
        g.y.push_back(m.native_id(y(I)));
        g.za.push_back(m.native_id(za(I)));
        g.zb.push_back(m.native_id(zb(I)));
        g.z.push_back(g.za.back()); g.z.push_back(g.zb.back());
        g.c.push_back(c);
        g.total0.push_back(va0 + 1); g.total1.push_back(1 + vb1);
        // z_a <= c (va0 + 30 y), z_a <= c (va0 + 4 - (va0 + 3) y)
        g.rows.push_back(m.add_constraint(za(I) - (c * 30) * y(I) <= c * va0));
        g.rows.push_back(m.add_constraint(za(I) + (c * (va0 + 3)) * y(I) <= c * (va0 + 4)));
        // the mirror image, in 1 - y
        g.rows.push_back(m.add_constraint(zb(I) + (c * 30) * y(I) <= c * (vb1 + 30)));
        g.rows.push_back(m.add_constraint(zb(I) - (c * (vb1 + 3)) * y(I) <= c * 1));
        g.opt += c * std::max(va0 + 1, 1 + vb1);
    }
    auto yb = m.add_binary_variables(3);
    auto zab = m.add_variables(3);
    auto zbb = m.add_variables(3);
    for(std::size_t J = 0; J < 3; ++J) {
        const double c = 3;
        g.za.push_back(m.native_id(zab(J)));
        g.zb.push_back(m.native_id(zbb(J)));
        g.z.push_back(g.za.back()); g.z.push_back(g.zb.back());
        g.rows.push_back(m.add_constraint(zab(J) - (c * 30) * yb(J) <= c * 3));
        g.rows.push_back(m.add_constraint(zab(J) + (c * 6) * yb(J) <= c * 7));
        g.rows.push_back(m.add_constraint(zbb(J) + (c * 30) * yb(J) <= c * 33));
        g.rows.push_back(m.add_constraint(zbb(J) - (c * 6) * yb(J) <= c * 1));
        if(flip) m.add_constraint(yb(J) + y(std::size_t{0}) <= 1);
        else m.add_constraint(yb(J) - y(std::size_t{0}) <= 0);
        g.opt += c * 4;
    }
    m.set_objective(xsum(Is, [&](std::size_t i) { return za(i) + zb(i); }) +
                    xsum(std::views::iota(std::size_t{0}, std::size_t{3}), [&](std::size_t j) { return zab(j) + zbb(j); }));
    return g;
}

// A cut added once, in the worse subtree, then watched everywhere. Tells which
// nodes obey it afterwards. A global cut must hold in the sibling; a local cut
// must not; a cut the solver dropped holds nowhere.
struct scope_tracker {
    raw_cut cut;
    int trigger;
    std::function<int(const std::vector<double> &)> state;
    std::atomic<bool> added{false};
    long add_error = 0, same_checks = 0, same_viol = 0, sib_checks = 0, sib_viol = 0, st0 = 0, st1 = 0, stf = 0;
    std::mutex m;
    bool wants_add(const std::vector<double> & x) {
        std::lock_guard l(m);
        const int s = state(x);
        (s == 0 ? st0 : s == 1 ? st1 : stf)++;
        if(!added) {
            if(s == trigger && cut.violation(x) > 1e-4) { added = true; return true; }
            return false;
        }
        const bool v = cut.violation(x) > 1e-6;
        if(s == trigger) { ++same_checks; same_viol += v; }
        else if(s == 1 - trigger) { ++sib_checks; sib_viol += v; }
        return false;
    }
    std::string line() const {
        std::string verdict;
        if(!added) verdict = "NO POWER: cut never added";
        else if(same_checks == 0) verdict = "NO POWER: nothing seen in the subtree after the cut";
        else if(same_viol == same_checks) verdict = "DROPPED OR NOT ENFORCED in its own subtree";
        else if(sib_checks == 0) verdict = "NO POWER on the sibling: read final value only";
        else if(sib_viol == 0) verdict = "HELD IN THE SIBLING (global in force)";
        else if(sib_viol == sib_checks) verdict = "NOT HELD IN THE SIBLING (local in force)";
        else verdict = "HELD IN PART OF THE SIBLING (pool? read the counts)";
        return std::format("added={} add_error={} subtree viol/checks={}/{} sibling viol/checks={}/{} states y0=0:{} y0=1:{} frac:{} -> {}",
            added.load(), add_error, same_viol, same_checks, sib_viol, sib_checks, st0, st1, stf, verdict);
    }
};

// ------------------------------------------------------------------- ATSP ---
// Compact MTZ, so a complete model: DFJ subtour cuts are valid for every integer
// feasible point yet not implied by the LP, and the solvers do not derive them.
// With complements=true, each arc also has a complement variable x + xbar == 1
// which presolve eliminates, and cuts are written on the complements: a wrapper
// that passes presolved columns, or a hook that loses eliminated ones, shows.
struct atsp {
    int n;
    std::vector<std::vector<int>> col, comp;  // native columns; comp[i][j] = -1 when absent
    std::vector<std::vector<double>> d;
    bool complements = false;
    double objective(const std::vector<double> & x) const {
        double s = 0;
        for(int i = 0; i < n; ++i) for(int j = 0; j < n; ++j) if(i != j)
            s += d[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] * x[static_cast<std::size_t>(col[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)])];
        return s;
    }
    double X(const std::vector<double> & x, int i, int j) const { return x[static_cast<std::size_t>(col[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)])]; }
    bool point_ok(const std::vector<double> & x) const {
        for(int i = 0; i < n; ++i) {
            double out = 0, in = 0;
            for(int j = 0; j < n; ++j) if(i != j) { out += X(x, i, j); in += X(x, j, i); }
            if(std::fabs(out - 1) > 1e-5 || std::fabs(in - 1) > 1e-5) return false;
        }
        if(complements) for(int i = 0; i < n; ++i) for(int j = 0; j < n; ++j) if(i != j)
            if(std::fabs(X(x, i, j) + x[static_cast<std::size_t>(comp[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)])] - 1) > 1e-5) return false;
        return true;
    }
    bool integral(const std::vector<double> & x) const {
        for(int i = 0; i < n; ++i) for(int j = 0; j < n; ++j) if(i != j) {
            const double v = X(x, i, j);
            if(std::fabs(v - std::round(v)) > 1e-5) return false;
        }
        return true;
    }
    // Up to max_cuts most violated DFJ cuts, by max-flow from 0 to each t and back.
    std::vector<raw_cut> separate(const std::vector<double> & x, const std::size_t max_cuts = 5) const {
        const auto N = static_cast<std::size_t>(n);
        if(n > 64) { std::println(stderr, "atsp::separate packs sets in 64 bits: n <= 64"); std::abort(); }
        std::map<std::uint64_t, double> found;
        std::vector<std::vector<double>> cap0(N, std::vector<double>(N, 0));
        for(std::size_t i = 0; i < N; ++i) for(std::size_t j = 0; j < N; ++j) if(i != j)
            cap0[i][j] = std::max(0.0, x[static_cast<std::size_t>(col[i][j])]);
        const auto flow = [&](std::size_t s, std::size_t t, bool add_set) {
            auto cap = cap0;
            double total = 0;
            std::vector<int> pred(N);
            while(true) {
                std::fill(pred.begin(), pred.end(), -1);
                pred[s] = static_cast<int>(s);
                std::vector<std::size_t> q{s};
                for(std::size_t h = 0; h < q.size() && pred[t] < 0; ++h)
                    for(std::size_t v = 0; v < N; ++v)
                        if(pred[v] < 0 && cap[q[h]][v] > 1e-9) { pred[v] = static_cast<int>(q[h]); q.push_back(v); }
                if(pred[t] < 0) break;
                double f = 1e9;
                for(std::size_t v = t; v != s; v = static_cast<std::size_t>(pred[v])) f = std::min(f, cap[static_cast<std::size_t>(pred[v])][v]);
                for(std::size_t v = t; v != s; v = static_cast<std::size_t>(pred[v])) { cap[static_cast<std::size_t>(pred[v])][v] -= f; cap[v][static_cast<std::size_t>(pred[v])] += f; }
                total += f;
                if(total >= 1) return;
            }
            if(total < 1 - 1e-4 && add_set) {
                std::uint64_t mask = 0;
                for(std::size_t v = 0; v < N; ++v) if(pred[v] >= 0) mask |= std::uint64_t{1} << v;
                auto & best = found[mask];
                best = std::max(best, 1 - total);
            }
        };
        for(std::size_t t = 1; t < N; ++t) { flow(0, t, true); flow(t, 0, true); }
        std::vector<std::pair<double, std::uint64_t>> order;
        for(auto [mask, v] : found) order.emplace_back(v, mask);
        std::ranges::sort(order, std::greater<>{});
        std::vector<raw_cut> out;
        for(auto [v, mask] : order) {
            if(out.size() == max_cuts) break;
            raw_cut c;
            c.sense = 'G';
            int count = 0;
            for(std::size_t i = 0; i < N; ++i) for(std::size_t j = 0; j < N; ++j)
                if(i != j && (mask >> i & 1) && !(mask >> j & 1)) {
                    ++count;
                    if(complements) { c.ind.push_back(comp[i][j]); c.val.push_back(-1); }
                    else { c.ind.push_back(col[i][j]); c.val.push_back(1); }
                }
            c.rhs = complements ? 1.0 - count : 1.0;
            out.push_back(std::move(c));
        }
        return out;
    }
};
template <class M>
atsp build_atsp(M & m, const int n, const bool complements = false, const unsigned seed = 5) {
    atsp a;
    a.n = n; a.complements = complements;
    const auto N = static_cast<std::size_t>(n);
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> P(0, 100), noise(1.0, 1.3);
    std::vector<std::pair<double, double>> pts(N);
    for(auto & p : pts) p = {P(rng), P(rng)};
    a.d.assign(N, std::vector<double>(N, 0));
    for(std::size_t i = 0; i < N; ++i) for(std::size_t j = 0; j < N; ++j) if(i != j)
        a.d[i][j] = std::round(std::hypot(pts[i].first - pts[j].first, pts[i].second - pts[j].second) * noise(rng));
    const auto arcs = N * (N - 1);
    const auto id = [N](std::size_t i, std::size_t j) { return i * (N - 1) + (j < i ? j : j - 1); };
    auto x = m.add_binary_variables(arcs);
    auto u = m.add_variables(N, {.obj_coef = 0, .lower_bound = 0, .upper_bound = static_cast<double>(n - 1)});
    a.col.assign(N, std::vector<int>(N, -1));
    a.comp.assign(N, std::vector<int>(N, -1));
    for(std::size_t i = 0; i < N; ++i) for(std::size_t j = 0; j < N; ++j) if(i != j)
        a.col[i][j] = m.native_id(x(id(i, j)));
    m.set_minimization();
    const auto nodes = std::views::iota(std::size_t{0}, N);
    m.set_objective(xsum(nodes, [&](std::size_t i) {
        return xsum(nodes | std::views::filter([i](std::size_t j) { return j != i; }),
                    [&, i](std::size_t j) { return a.d[i][j] * x(id(i, j)); });
    }));
    for(std::size_t i : nodes) {
        m.add_constraint(xsum(nodes | std::views::filter([i](std::size_t j) { return j != i; }), [&, i](std::size_t j) { return 1.0 * x(id(i, j)); }) == 1);
        m.add_constraint(xsum(nodes | std::views::filter([i](std::size_t j) { return j != i; }), [&, i](std::size_t j) { return 1.0 * x(id(j, i)); }) == 1);
    }
    for(std::size_t i = 1; i < N; ++i) for(std::size_t j = 1; j < N; ++j) if(i != j)
        m.add_constraint(u(i) - u(j) + static_cast<double>(n - 1) * x(id(i, j)) <= static_cast<double>(n - 2));
    if(complements) {
        auto xb = m.add_binary_variables(arcs);
        for(std::size_t i = 0; i < N; ++i) for(std::size_t j = 0; j < N; ++j) if(i != j) {
            a.comp[i][j] = m.native_id(xb(id(i, j)));
            m.add_constraint(x(id(i, j)) + xb(id(i, j)) == 1);
        }
    }
    return a;
}

// A Chu-Beasley multidimensional knapsack, as in the progress survey: stopped by a
// time limit, so a search that lasts.
template <class M>
void build_mkp(M & m, const int rows, const int cols, const unsigned seed = 7) {
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> A(1, 1000), R(0, 500);
    const auto C = static_cast<std::size_t>(cols);
    std::vector<std::vector<double>> a(static_cast<std::size_t>(rows), std::vector<double>(C));
    for(auto & row : a) for(auto & v : row) v = A(rng);
    std::vector<double> c(C);
    for(std::size_t j = 0; j < C; ++j) {
        double sum = 0;
        for(const auto & row : a) sum += row[j];
        c[j] = std::floor(sum / rows + R(rng));
    }
    auto x = m.add_binary_variables(C);
    const auto ids = std::views::iota(std::size_t{0}, C);
    m.set_maximization();
    m.set_objective(xsum(ids, [&](std::size_t j) { return c[j] * x(j); }));
    for(const auto & row : a) {
        double sum = 0;
        for(double v : row) sum += v;
        m.add_constraint(xsum(ids, [&row, &x](std::size_t j) { return row[j] * x(j); }) <= std::floor(sum / 2));
    }
}

// ----------------------------------------------------------------- report ---
template <class M>
std::string summary(M & m, const double seconds) {
    std::string s = std::format("status={} solve={:.2f}s", status_name(m.get_status()), seconds);
    try { s += std::format(" value={:.6g}", m.get_solution_value()); } catch(const std::exception & e) { s += " value=throws"; }
    if constexpr(has_dual_bound<M>) { try { s += std::format(" dual={:.6g}", m.get_dual_bound()); } catch(const std::exception &) {} }
    if constexpr(has_node_count<M>) { try { s += std::format(" nodes={}", m.get_node_count()); } catch(const std::exception &) {} }
    return s;
}
template <class M>
double timed_solve(M & m) {
    const auto t0 = clk::now();
    try { m.solve(); }
    catch(const std::exception & e) { std::println(stderr, "   solve() threw: {}", e.what()); }
    return std::chrono::duration<double>(clk::now() - t0).count();
}

// Everything a probe has to provide.
template <class M>
struct backend {
    // Registers the hook on m for its next solve(), adding cuts as `mode` says.
    std::function<void(M &, separator, const std::string &)> attach;
    // No presolve, no solver cuts, no heuristics, no symmetry, no disconnected or
    // restart handling, one thread. Native, per solver.
    std::function<void(M &)> scope_settings;
    std::function<void(M &)> one_thread;
    std::vector<std::string> cut_modes;  // first = the backend's plain global cut
    int atsp_n = 26;
    int atsp_complement_n = 26;  // about twice the columns of atsp_n
    double atsp_limit = 60;
    int gadgets_k = 10;
    double gadget_limit = 30;
};

template <class M>
void run_matrix(const backend<M> & B) {
    using std::chrono::duration;
    const auto TL = [&](M & m, double l) { m.set_time_limit(duration<double>(l)); };
    double atsp_value = NAN;
    // CUT_QUICK=1, for other releases of a solver: E1, E3, S1 and S2 only.
    const bool quick = std::getenv("CUT_QUICK") != nullptr;
    // --- E1-E3: does the hook fire, change the path, and do cuts bite?
    {
        M m; B.one_thread(m); auto a = build_atsp(m, B.atsp_n); TL(m, B.atsp_limit);
        const double t = timed_solve(m);
        std::println(stderr, "== E1 atsp n={} default settings, 1 thread, no hook: {}", B.atsp_n, summary(m, t));
        atsp_value = m.get_solution_value();
    }
    if(!quick) {
        M m; B.one_thread(m); auto a = build_atsp(m, B.atsp_n); TL(m, B.atsp_limit);
        auto s = std::make_shared<stats>();
        B.attach(m, watched([](const auto &, const auto &) {}, s,
            [&a](const auto & x) { return a.objective(x); }, [&a](const auto & x) { return a.point_ok(x); }, [&a](const auto & x) { return a.integral(x); }), B.cut_modes.front());
        const double t = timed_solve(m);
        std::println(stderr, "== E2 same, no-op hook: {}\n   {}", summary(m, t), s->line());
    }
    for(const auto & mode : B.cut_modes) {
        M m; B.one_thread(m); auto a = build_atsp(m, B.atsp_n); TL(m, B.atsp_limit);
        auto s = std::make_shared<stats>();
        B.attach(m, watched([&a](const auto & x, const add_fn & add) { for(const auto & c : a.separate(x)) add(c); }, s,
            [&a](const auto & x) { return a.objective(x); }, [&a](const auto & x) { return a.point_ok(x); }, [&a](const auto & x) { return a.integral(x); }), mode);
        const double t = timed_solve(m);
        std::println(stderr, "== E3 same, DFJ cuts, mode {}: {} (baseline value {})\n   {}", mode, summary(m, t), atsp_value, s->line());
    }
    // --- E4: cuts written on columns presolve eliminates.
    if(!quick) {
        M m; B.one_thread(m); auto a = build_atsp(m, B.atsp_complement_n, true); TL(m, B.atsp_limit);
        auto s = std::make_shared<stats>();
        B.attach(m, watched([&a](const auto & x, const add_fn & add) { for(const auto & c : a.separate(x)) add(c); }, s,
            [&a](const auto & x) { return a.objective(x); }, [&a](const auto & x) { return a.point_ok(x); }, [&a](const auto & x) { return a.integral(x); }), B.cut_modes.front());
        const double t = timed_solve(m);
        std::println(stderr, "== E4 atsp n={} + eliminated complements, cuts on complements, mode {}: {} (baseline value at n={} is {})\n   {}", B.atsp_complement_n, B.cut_modes.front(), summary(m, t), B.atsp_n, atsp_value, s->line());
    }
    // --- E5: can two calls overlap at default threads? A search that lasts, on a
    // model the hook leaves alone: a long no-op hook, so overlaps have a window.
    if(!quick) {
        M m; build_mkp(m, 30, 100); TL(m, 10.0);
        auto s = std::make_shared<stats>();
        B.attach(m, watched([](const auto &, const auto &) {}, s, {}, {}, {}, std::chrono::microseconds(200)), B.cut_modes.front());
        const double t = timed_solve(m);
        std::println(stderr, "== E5 knapsack 30x100, default threads, 200 us spin in the hook: {}\n   {}", summary(m, t), s->line());
    }
    if(!quick) {
        M m; B.one_thread(m); build_mkp(m, 30, 100); TL(m, 10.0);
        auto s = std::make_shared<stats>();
        B.attach(m, watched([](const auto &, const auto &) {}, s), B.cut_modes.front());
        const double t = timed_solve(m);
        std::println(stderr, "== E5b same, 1 thread, no spin, no-op hook: {}\n   {}", summary(m, t), s->line());
        M n; B.one_thread(n); build_mkp(n, 30, 100); TL(n, 10.0);
        const double t2 = timed_solve(n);
        std::println(stderr, "== E5c same, no hook: {}", summary(n, t2));
    }
    // --- S0: is there a tree, and does it split on y0 as designed?
    if(!quick) for(const bool flip : {false, true}) {
        { M m; B.scope_settings(m); auto g = build_gadgets(m, B.gadgets_k, flip); TL(m, B.gadget_limit);
          const double t = timed_solve(m);
          std::println(stderr, "== S0 gadgets k={} flip={} no hook: {} (known opt {})", g.k, flip, summary(m, t), g.opt); }
        { M m; B.scope_settings(m); auto g = build_gadgets(m, B.gadgets_k, flip); TL(m, B.gadget_limit);
          auto s = std::make_shared<stats>();
          long st[3] = {0, 0, 0};
          B.attach(m, watched([&](const auto & x, const auto &) { st[g.state(x) + 1]++; }, s, [&g](const auto & x) { return g.objective(x); }, [&g](const auto & x) { return g.point_ok(x); }), B.cut_modes.front());
          const double t = timed_solve(m);
          std::println(stderr, "== S0 same, no-op hook: {}\n   {} y0 frac:{} y0=0:{} y0=1:{}", summary(m, t), s->line(), st[0], st[1], st[2]); }
    }
    // --- S1: a cut valid in the worse subtree only. Global semantics lose the optimum.
    for(const auto & mode : B.cut_modes) for(const bool flip : {false, true}) {
        M m; B.scope_settings(m); auto g = build_gadgets(m, B.gadgets_k, flip); TL(m, B.gadget_limit);
        auto s = std::make_shared<stats>();
        auto tr = std::make_shared<scope_tracker>();
        tr->cut = g.invalid_global_cut(); tr->trigger = g.worse_state(); tr->state = [&g](const auto & x) { return g.state(x); };
        B.attach(m, watched([tr](const auto & x, const add_fn & add) { if(tr->wants_add(x)) tr->add_error = add(tr->cut); }, s, [&g](const auto & x) { return g.objective(x); }, [&g](const auto & x) { return g.point_ok(x); }), mode);
        const double t = timed_solve(m);
        double value = NAN; try { value = m.get_solution_value(); } catch(const std::exception &) {}
        std::println(stderr, "== S1 mode {} flip={}: {} (known opt {}) => {}\n   {}\n   {}", mode, flip, summary(m, t), g.opt,
            std::fabs(value - g.opt) < 1e-4 ? "optimum KEPT" : "optimum LOST", tr->line(), s->line());
    }
    // --- S2: a globally valid cut added once in a subtree. Does the sibling see it?
    for(const auto & mode : B.cut_modes) for(const bool flip : {false, true}) {
        M m; B.scope_settings(m); auto g = build_gadgets(m, B.gadgets_k, flip); TL(m, B.gadget_limit);
        auto s = std::make_shared<stats>();
        auto tr = std::make_shared<scope_tracker>();
        tr->cut = g.hull_cut(3); tr->trigger = g.worse_state(); tr->state = [&g](const auto & x) { return g.state(x); };
        B.attach(m, watched([tr](const auto & x, const add_fn & add) { if(tr->wants_add(x)) tr->add_error = add(tr->cut); }, s, [&g](const auto & x) { return g.objective(x); }, [&g](const auto & x) { return g.point_ok(x); }), mode);
        const double t = timed_solve(m);
        std::println(stderr, "== S2 mode {} flip={}: {} (known opt {})\n   {}\n   {}", mode, flip, summary(m, t), g.opt, tr->line(), s->line());
    }
    // --- P: does a cut outlive the solve? It becomes invalid once the model changes.
    if constexpr(requires(M & m, gadgets<M> & g) { m.set_constraint_rhs(g.rows[0], 1.0); }) if(!quick) {
        M m; B.scope_settings(m); auto g = build_gadgets(m, B.gadgets_k, false); TL(m, B.gadget_limit);
        auto s = std::make_shared<stats>();
        auto tr = std::make_shared<scope_tracker>();
        const raw_cut cut = g.hull_cut(3);
        bool once = false;
        B.attach(m, watched([&](const auto & x, const add_fn & add) { if(!once && cut.violation(x) > 1e-4) { once = true; add(cut); } }, s), B.cut_modes.front());
        const double t = timed_solve(m);
        std::println(stderr, "== P first solve with the hull cut of gadget 3: {} (known opt {})\n   {}", summary(m, t), g.opt, s->line());
        // every bound of gadget 3 up by 50: the hull cut now cuts off the optimum
        for(std::size_t r = 12; r < 16; ++r) m.set_constraint_rhs(g.rows[r], m.get_constraint_rhs(g.rows[r]) + 50);
        const double t2 = timed_solve(m);
        double value = NAN; try { value = m.get_solution_value(); } catch(const std::exception &) {}
        std::println(stderr, "== P re-solve, rhs raised so that the cut is invalid: {} (new opt {}) => {}", summary(m, t2), g.opt + 100,
            std::fabs(value - (g.opt + 100)) < 1e-4 ? "cut did NOT persist" : "cut PERSISTED (wrong optimum)");
    }
    // --- W: what the solver does with odd cuts. Native error codes are reported, not thrown.
    if(!quick) {
        const auto weird = [&](const std::string & name, std::function<raw_cut(const gadgets<M> &)> make, const bool every_call = false) {
            M m; B.scope_settings(m); auto g = build_gadgets(m, B.gadgets_k, false); TL(m, std::min(B.gadget_limit, 15.0));
            auto s = std::make_shared<stats>();
            const raw_cut cut = make(g);
            bool once = false;
            int err = 0;
            B.attach(m, watched([&](const auto &, const add_fn & add) { if(every_call || !once) { once = true; const int e = add(cut); if(e) err = e; } }, s, [&g](const auto & x) { return g.objective(x); }), B.cut_modes.front());
            const double t = timed_solve(m);
            std::println(stderr, "== W {}: {} (known opt {}) native error={}\n   {}", name, summary(m, t), g.opt, err, s->line());
        };
        weird("equality, dummy == 1", [](const auto & g) { return raw_cut{{g.dummy}, {1.0}, 'E', 1.0}; });
        weird("no terms, 0 <= 1", [](const auto &) { return raw_cut{{}, {}, 'L', 1.0}; });
        weird("a zero coefficient", [](const auto & g) { auto c = g.hull_cut(3); c.ind.push_back(g.za[1]); c.val.push_back(0.0); return c; });
        weird("duplicate column in the hull cut", [](const auto & g) { auto c = g.hull_cut(3); c.val[0] = 0.5; c.ind.push_back(c.ind[0]); c.val.push_back(0.5); return c; });
        weird("not violated, za3 <= 1e6", [](const auto & g) { return raw_cut{{g.za[3]}, {1.0}, 'L', 1e6}; });
        weird("the same cut at every call", [](const auto & g) { return g.hull_cut(3); }, true);
        weird("empties the problem, za3 <= -1", [](const auto & g) { return raw_cut{{g.za[3]}, {1.0}, 'L', -1.0}; });
    }
}

}  // namespace cp
