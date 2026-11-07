// D: try to break "any truly valid cut is safe" under the recipe at default settings.
// Every offered cut is checked against the baseline optimum: a violation there is a harness
// bug, not a finding (invalid_offers must stay 0).
#include "verify_gurobi_common.hpp"
using std::chrono::duration;
using offer_fn = std::function<void(const raw_cut &)>;
using sep_t = std::function<void(hctx &, const std::vector<double> &, const offer_fn &)>;

static std::map<std::string, std::array<long, 4>> tally;  // pass, fail, gap, nopower
static std::string g_family;
static int g_threads = 1;
static double g_limit = 30;

static sol base_of(const rawm & r, int threads = 1, double gap = -1) {
    M m; seti(m, "OutputFlag", 0); if(threads) seti(m, "Threads", threads); setd(m, "TimeLimit", g_limit); if(gap >= 0) setd(m, "MIPGap", gap); r.load(m);
    return solve_native(m);
}
struct topt { bool keep = false; bool expect_status_only = false; std::function<void(M &, const sol &)> before; };
static void trial(const std::string & group, const std::string & name, const rawm & r, const sol & base, const sep_t & sep, topt o = {}) {
    M m; seti(m, "OutputFlag", 0); if(g_threads) seti(m, "Threads", g_threads); setd(m, "TimeLimit", g_limit); recipe(m); r.load(m);
    if(o.before) o.before(m, base);
    long invalid = 0, offers = 0;
    hctx c; c.drop_satisfied = !o.keep;
    c.on_node = [&](hctx & h, const std::vector<double> & x) {
        const offer_fn offer = [&](const raw_cut & k) {
            ++offers;
            if(!base.x.empty()) {
                double scale = 1; for(double v : k.val) scale = std::max(scale, std::fabs(v));
                if(k.violation(base.x) > 1e-7 * scale) ++invalid;
            }
            h.add(k);
        };
        sep(h, x, offer);
    };
    attach(m, c);
    const sol s = solve_native(m, false);
    const bool status_ok = s.status == base.status;
    const bool val_ok = !(base.status == 2) || std::fabs(s.value - base.value) <= 1e-6 * (1 + std::fabs(base.value));
    const bool within_gap = !(base.status == 2) || std::fabs(s.value - base.value) <= 2e-4 * (1 + std::fabs(base.value));
    const bool pass = status_ok && val_ok;
    const char * verdict = pass ? "PASS" : (status_ok && within_gap) ? "GAPDIFF" : "FAIL";
    const bool power = c.cuts > 0;
    auto & t = tally[group];
    if(pass) ++t[0]; else if(status_ok && within_gap) ++t[2]; else ++t[1];
    if(!power) ++t[3];
    std::println(stderr, "{} [{}] {}: base {} {:.10g} nodes={} | {} | calls={} tree_calls={} offers={} cuts={} native_err={}(first {}) sanitized_out={} invalid_offers={} swallowed={}{}", verdict, group, name, sname(base.status), base.value, base.nodes, line(s), c.opt_calls, c.tree_calls, offers, c.cuts, c.native_err, c.first_err, c.sanitized_out, invalid, c.swallowed, power ? "" : " NO-POWER");
    std::fflush(stderr);
}

// ------------------------------------------------------------------ families
static rawm make_atsp_raw(int n, unsigned seed, cp::atsp & view, std::vector<int> & u) {
    rawm r; r.sense = 1;
    const auto N = static_cast<size_t>(n);
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> P(0, 100), noise(1.0, 1.3);
    std::vector<std::pair<double, double>> pts(N);
    for(auto & p : pts) p = {P(rng), P(rng)};
    view.n = n; view.col.assign(N, std::vector<int>(N, -1)); view.comp = view.col; view.d.assign(N, std::vector<double>(N, 0));
    for(size_t i = 0; i < N; ++i) for(size_t j = 0; j < N; ++j) if(i != j) {
        const double d = std::round(std::hypot(pts[i].first - pts[j].first, pts[i].second - pts[j].second) * noise(rng));
        view.d[i][j] = d; view.col[i][j] = r.bin(d);
    }
    u.resize(N); for(auto & v : u) v = r.var(0, 0, n - 1, 'C');
    for(size_t i = 0; i < N; ++i) {
        std::vector<int> o, in; for(size_t j = 0; j < N; ++j) if(i != j) { o.push_back(view.col[i][j]); in.push_back(view.col[j][i]); }
        r.add(o, std::vector<double>(o.size(), 1.0), '=', 1); r.add(in, std::vector<double>(in.size(), 1.0), '=', 1);
    }
    for(size_t i = 1; i < N; ++i) for(size_t j = 1; j < N; ++j) if(i != j)
        r.add({u[i], u[j], view.col[i][j]}, {1, -1, static_cast<double>(n - 1)}, '<', n - 2);
    return r;
}
// Capacitated fixed charge: f_ij <= d_j y_i is valid and has a zero rhs.
struct flm { rawm r; std::vector<std::vector<int>> f; std::vector<int> y; std::vector<double> d; };
static flm make_fl(unsigned seed, size_t fac = 14, size_t cli = 24) {
    flm D; D.r.sense = 1;
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> C(1, 40), F(100, 300), DD(5, 30);
    D.d.resize(cli); for(auto & v : D.d) v = DD(rng);
    double tot = 0; for(double v : D.d) tot += v;
    for(size_t i = 0; i < fac; ++i) D.y.push_back(D.r.bin(F(rng)));
    D.f.assign(fac, std::vector<int>(cli));
    for(size_t i = 0; i < fac; ++i) for(size_t j = 0; j < cli; ++j) D.f[i][j] = D.r.var(C(rng), 0, 1e9, 'C');
    for(size_t j = 0; j < cli; ++j) { std::vector<int> ind; for(size_t i = 0; i < fac; ++i) ind.push_back(D.f[i][j]); D.r.add(ind, std::vector<double>(ind.size(), 1.0), '=', D.d[j]); }
    const double cap = std::ceil(tot / 3);
    for(size_t i = 0; i < fac; ++i) { std::vector<int> ind; std::vector<double> val; for(size_t j = 0; j < cli; ++j) { ind.push_back(D.f[i][j]); val.push_back(1); } ind.push_back(D.y[i]); val.push_back(-cap); D.r.add(ind, val, '<', 0); }
    return D;
}
static std::vector<raw_cut> fl_cuts(const flm & D, const std::vector<double> & x, double K = 1.0, size_t maxc = 20) {
    std::vector<raw_cut> out;
    for(size_t i = 0; i < D.y.size(); ++i) for(size_t j = 0; j < D.d.size(); ++j) {
        raw_cut c{{D.f[i][j], D.y[i]}, {K, -K * D.d[j]}, 'L', 0};
        if(c.violation(x) > 1e-4 * K) out.push_back(std::move(c));
    }
    if(out.size() > maxc) { std::ranges::sort(out, [&](const raw_cut & l, const raw_cut & r) { return l.violation(x) > r.violation(x); }); out.resize(maxc); }
    return out;
}
// Pairs: 2 a - 2 b + c <= 1 gives a <= b only through integrality; a = b = 1 is worth taking,
// so an optimum sits on the cut and a solver that kills a or b loses value.
struct pairs_m { rawm r; std::vector<int> a, b; };
static pairs_m make_pairs(unsigned seed, int m_ = 10) {
    pairs_m D; D.r.sense = -1;
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> V(5, 15), W(1, 30);
    std::vector<int> k; std::vector<double> kv;
    double tot = 0;
    for(int i = 0; i < m_; ++i) {
        const double v = V(rng);
        const int a = D.r.bin(v), b = D.r.bin(-std::floor(v * 0.6)), c = D.r.bin(1);
        D.a.push_back(a); D.b.push_back(b);
        D.r.add({a, b, c}, {2, -2, 1}, '<', 1);
        const double w = W(rng); k.push_back(a); kv.push_back(w); k.push_back(b); kv.push_back(w); tot += 2 * w;
    }
    for(int j = 0; j < 20; ++j) { const double w = W(rng); k.push_back(D.r.bin(w + std::uniform_int_distribution<int>(0, 5)(rng))); kv.push_back(w); tot += w; }
    D.r.add(k, kv, '<', std::floor(tot * 0.4));
    return D;
}
// A core cg plus columns that presolve removes.
struct rdm {
    rawm r; cgm G;
    std::vector<int> fixed;   // value 1, by bounds or by a row
    std::vector<int> zero;    // lb = ub = 0
    int w = -1;               // dual-fixed at 4
    struct eq { std::vector<int> ind; std::vector<double> val; double rhs; };
    std::vector<eq> eqs;      // equality rows: any multiple can be added to a cut
    int y1 = -1;              // valid: y1 <= 1 through four rows and integrality
    int pr = -1;              // pr = 0 by probing
    int unused = -1, s = -1, t = -1, p1 = -1, p2 = -1, d1 = -1, d2 = -1;
};
static rdm make_rd(unsigned seed) {
    rdm D; D.G = make_cg(seed, 30, 12); D.r = D.G.r; rawm & r = D.r;
    { int f = r.var(3, 1, 1, 'B'); D.fixed.push_back(f); }
    { int f = r.bin(2); r.add({f}, {1}, '=', 1); D.fixed.push_back(f); }
    { int f = r.bin(1); D.fixed.push_back(f); r.add({f}, {1}, '>', 1); }
    { int z = r.var(5, 0, 0, 'B'); D.zero.push_back(z); }
    D.w = r.var(7, 0, 4, 'C');
    { int a = r.bin(11), ab = r.bin(0); r.add({a, ab}, {1, 1}, '=', 1); D.eqs.push_back({{a, ab}, {1, 1}, 1}); }
    { int y = r.var(1, 1, 7, 'I'), t = r.var(0, 0, 3, 'I'); r.add({y, t}, {1, -2}, '=', 1); D.eqs.push_back({{y, t}, {1, -2}, 1}); }
    { auto i = std::vector<int>(D.G.x.begin(), D.G.x.begin() + 8); std::vector<double> v(8, 1.0); D.s = r.var(0, 0, 1e9, 'C'); i.push_back(D.s); v.push_back(1); r.add(i, v, '=', 7); D.eqs.push_back({i, v, 7}); }
    { auto i = std::vector<int>(D.G.x.begin() + 8, D.G.x.begin() + 16); std::vector<double> v; for(int k = 0; k < 8; ++k) v.push_back(k + 1.0); D.t = r.var(0, -1e100, 1e100, 'C'); i.push_back(D.t); v.push_back(-1); r.add(i, v, '=', 0); D.eqs.push_back({i, v, 0}); }
    { D.p1 = r.var(9, 0, 3, 'I'); D.p2 = r.var(9, 0, 3, 'I'); auto i = D.G.x; std::vector<double> v(D.G.x.size(), 1.0); i.push_back(D.p1); v.push_back(5); i.push_back(D.p2); v.push_back(5); r.add(i, v, '<', 30); }
    { D.d1 = r.bin(6); D.d2 = r.bin(4); auto i = std::vector<int>(D.G.x.begin() + 16, D.G.x.begin() + 24); std::vector<double> v(8, 1.0); i.push_back(D.d1); v.push_back(3); i.push_back(D.d2); v.push_back(3); r.add(i, v, '<', 6); }
    D.unused = r.bin(0);
    { int a = r.bin(0); D.pr = r.bin(13); r.add({D.pr, a}, {1, 1}, '<', 1); r.add({D.pr, a}, {1, -1}, '<', 0); }
    { D.y1 = r.var(6, 0, 10, 'I'); int t1 = r.bin(0), t2 = r.bin(0);
      r.add({D.y1, t1, t2}, {5, 6, 6}, '<', 17); r.add({D.y1, t1, t2}, {5, -6, 6}, '<', 11); r.add({D.y1, t1, t2}, {5, 6, -6}, '<', 11); r.add({D.y1, t1, t2}, {5, -6, -6}, '<', 5); }
    return D;
}

// ------------------------------------------------------------------ the battery
static void d_scale() {
    const std::vector<double> Ks = {1e-30, 1e-20, 1e-13, 1e-9, 1e-6, 1e-3, 1e3, 1e6, 1e9, 1e12, 1e15, 1e20, 1e30, 1e50, 1e99, 1e100, 1e101, 1e150, 1e300};
    for(unsigned seed : {1u, 2u, 3u}) {
        const cgm G = make_cg(seed);
        const sol b = base_of(G.r);
        for(double K : Ks) {
            trial("scale cover", std::format("cg seed={} K={:g} K*cover<=K*rhs", seed, K), G.r, b, [&G, K](hctx &, const std::vector<double> & x, const offer_fn & o) { for(auto k : covers(G, x)) { for(auto & v : k.val) v *= K; k.rhs *= K; o(k); } });
            trial("scale cover G", std::format("cg seed={} K={:g} -K*cover>=-K*rhs", seed, K), G.r, b, [&G, K](hctx &, const std::vector<double> & x, const offer_fn & o) { for(auto k : covers(G, x)) { for(auto & v : k.val) v *= -K; k.rhs *= -K; k.sense = 'G'; o(k); } });
        }
    }
    for(unsigned seed : {1u, 2u, 3u}) {
        const flm D = make_fl(seed);
        const sol b = base_of(D.r);
        for(double K : Ks) trial("scale rhs0 fl", std::format("fl seed={} K={:g} K f - K d y <= 0", seed, K), D.r, b, [&D, K](hctx &, const std::vector<double> & x, const offer_fn & o) { for(const auto & k : fl_cuts(D, x, K)) o(k); });
    }
    for(unsigned seed : {1u, 2u, 3u}) {
        const pairs_m D = make_pairs(seed);
        const sol b = base_of(D.r);
        for(double K : Ks) trial("scale rhs0 pairs", std::format("pairs seed={} K={:g} K a - K b <= 0", seed, K), D.r, b, [&D, K](hctx &, const std::vector<double> & x, const offer_fn & o) {
            for(size_t i = 0; i < D.a.size(); ++i) { raw_cut k{{D.a[i], D.b[i]}, {K, -K}, 'L', 0}; if(k.violation(x) > 1e-4 * K) o(k); } });
    }
}
static void d_range() {
    for(unsigned seed : {1u, 2u, 3u, 4u, 5u}) {
        const cgm G = make_cg(seed);
        const sol b = base_of(G.r);
        for(double K : {1e3, 1e4, 1e6, 1e8, 1e9, 1e10, 1e12, 1e15, 1e20, 1e30, 1e50, 1e99, 1e100, 1e150}) {
            trial("range cover", std::format("cg seed={} cover + {:g} x_k <= rhs + {:g} (x_k at 1)", seed, K, K), G.r, b, [&G, K](hctx &, const std::vector<double> & x, const offer_fn & o) {
                for(auto k : covers(G, x)) {
                    int pick = -1;
                    for(int j : G.x) if(x[static_cast<size_t>(j)] > 1 - 1e-9 && std::ranges::find(k.ind, j) == k.ind.end()) { pick = j; break; }
                    if(pick < 0) continue;
                    k.ind.push_back(pick); k.val.push_back(K);
                    const double base_rhs = k.rhs; double rhs = base_rhs + K;
                    if(rhs - K < base_rhs) rhs = std::nextafter(rhs, std::numeric_limits<double>::infinity());
                    k.rhs = rhs; o(k);
                } });
        }
        for(double d : {1e-13, 1e-11, 1e-9, 1e-7, 1e-5, 1e-3}) {
            trial("tiny term +", std::format("cg seed={} cover + {:g} x_k <= rhs + {:g}", seed, d, d), G.r, b, [&G, d](hctx &, const std::vector<double> & x, const offer_fn & o) {
                for(auto k : covers(G, x)) { int pick = -1; for(int j : G.x) if(std::ranges::find(k.ind, j) == k.ind.end()) { pick = j; break; } if(pick < 0) continue; k.ind.push_back(pick); k.val.push_back(d); k.rhs += d; o(k); } });
            trial("tiny term -", std::format("cg seed={} cover - {:g} x_k <= rhs", seed, d), G.r, b, [&G, d](hctx &, const std::vector<double> & x, const offer_fn & o) {
                for(auto k : covers(G, x)) { int pick = -1; for(int j : G.x) if(std::ranges::find(k.ind, j) == k.ind.end()) { pick = j; break; } if(pick < 0) continue; k.ind.push_back(pick); k.val.push_back(-d); o(k); } });
        }
    }
}
static void d_objective() {
    const auto objcut = [](const rawm & r, double z, double extra, bool max) { raw_cut k; k.sense = max ? 'L' : 'G'; k.rhs = z + (max ? extra : -extra) - r.offset; for(size_t j = 0; j < r.obj.size(); ++j) if(r.obj[j] != 0) { k.ind.push_back(static_cast<int>(j)); k.val.push_back(r.obj[j]); } return k; };
    for(unsigned seed = 1; seed <= 12; ++seed) {
        const cgm G = make_cg(seed);
        const sol b = base_of(G.r, 1, 0.0);
        trial("objective tight", std::format("cg seed={} c'x<=z*", seed), G.r, b, [=, &G](hctx &, const std::vector<double> & x, const offer_fn & o) { const auto k = objcut(G.r, b.value, 0, true); if(k.violation(x) > 1e-7) o(k); });
        trial("objective tight", std::format("cg seed={} c'x<=z*+1e-7", seed), G.r, b, [=, &G](hctx &, const std::vector<double> & x, const offer_fn & o) { const auto k = objcut(G.r, b.value, 1e-7, true); if(k.violation(x) > 1e-7) o(k); });
        trial("objective neardup", std::format("cg seed={} 50 copies c'x<=z*+k*1e-9", seed), G.r, b, [=, &G](hctx &, const std::vector<double> & x, const offer_fn & o) { for(int e = 0; e < 50; ++e) { const auto k = objcut(G.r, b.value, e * 1e-9, true); if(k.violation(x) > 1e-7) o(k); } });
        trial("objective+covers", std::format("cg seed={} objective, covers, CG", seed), G.r, b, [=, &G, gen = std::make_shared<std::mt19937>(seed)](hctx &, const std::vector<double> & x, const offer_fn & o) { const auto k = objcut(G.r, b.value, 0, true); if(k.violation(x) > 1e-7) o(k); for(const auto & c : covers(G, x)) o(c); for(const auto & c : cg_cuts(G, x, *gen)) o(c); });
    }
    for(unsigned seed : {1u, 3u, 9u, 10u}) {
        auto view = std::make_shared<cp::atsp>(); std::vector<int> u; const rawm r = make_atsp_raw(20, seed, *view, u);
        const sol b = base_of(r, 1, 0.0);
        trial("objective tight min", std::format("atsp20 seed={} c'x>=z*", seed), r, b, [=, &r](hctx &, const std::vector<double> & x, const offer_fn & o) { const auto k = objcut(r, b.value, 0, false); if(k.violation(x) > 1e-7) o(k); });
        trial("objective tight min", std::format("atsp20 seed={} c'x>=z* + DFJ", seed), r, b, [=, &r](hctx &, const std::vector<double> & x, const offer_fn & o) { const auto k = objcut(r, b.value, 0, false); if(k.violation(x) > 1e-7) o(k); for(const auto & c : view->separate(x)) o(c); });
    }
    for(unsigned seed : {1u, 2u, 3u}) {
        const flm D = make_fl(seed); const sol b = base_of(D.r, 1, 0.0);
        trial("objective tight min", std::format("fl seed={} c'x>=z*", seed), D.r, b, [=, &D](hctx &, const std::vector<double> & x, const offer_fn & o) { const auto k = objcut(D.r, b.value, 0, false); if(k.violation(x) > 1e-7) o(k); });
    }
}
static void d_neardup() {
    for(unsigned seed = 1; seed <= 10; ++seed) {
        const cgm G = make_cg(seed);
        const sol b = base_of(G.r);
        trial("neardup rhs", std::format("cg seed={} each cover x200, rhs+k*1e-9", seed), G.r, b, [&G](hctx &, const std::vector<double> & x, const offer_fn & o) { for(const auto & c : covers(G, x)) for(int e = 0; e < 200; ++e) { auto k = c; k.rhs += e * 1e-9; o(k); } });
        trial("neardup coef", std::format("cg seed={} each cover x50, coefficients perturbed, valid", seed), G.r, b, [&G, gen = std::make_shared<std::mt19937>(seed)](hctx &, const std::vector<double> & x, const offer_fn & o) {
            std::uniform_real_distribution<double> U(0, 1e-7);
            for(const auto & c : covers(G, x)) for(int e = 0; e < 50; ++e) { auto k = c; double up = 0; for(auto & v : k.val) { const double d = U(*gen); v += d; up += d; } k.rhs += up; o(k); } });
        trial("same cut every call", std::format("cg seed={} every cover ever seen, every call, no drop", seed), G.r, b, [&G, seen = std::make_shared<std::vector<raw_cut>>()](hctx &, const std::vector<double> & x, const offer_fn & o) { for(const auto & c : covers(G, x)) seen->push_back(c); for(const auto & c : *seen) o(c); }, topt{.keep = true});
    }
}
static void d_rows_and_terms() {
    for(unsigned seed = 1; seed <= 8; ++seed) {
        const rdm D = make_rd(seed);
        const sol b = base_of(D.r);
        const rawm & r = D.r;
        const int nv = static_cast<int>(r.obj.size());
        trial("re-offer rows", std::format("rd seed={} every model row, every call", seed), r, b, [&r](hctx &, const std::vector<double> &, const offer_fn & o) { for(const auto & w : r.rows) o(raw_cut{w.ind, w.val, w.sense == '<' ? 'L' : w.sense == '>' ? 'G' : 'E', w.rhs}); }, topt{.keep = true});
        trial("one-term bounds", std::format("rd seed={} x<=ub and x>=lb on every column, every call", seed), r, b, [&r, nv](hctx &, const std::vector<double> &, const offer_fn & o) { for(int j = 0; j < nv; ++j) { const auto J = static_cast<size_t>(j); if(r.ub[J] < 1e20) o(raw_cut{{j}, {1.0}, 'L', r.ub[J]}); if(r.lb[J] > -1e20) o(raw_cut{{j}, {1.0}, 'G', r.lb[J]}); } }, topt{.keep = true});
        trial("one-term on removed", std::format("rd seed={} equalities and huge terms on fixed columns", seed), r, b, [&D](hctx &, const std::vector<double> &, const offer_fn & o) {
            for(int f : D.fixed) { o(raw_cut{{f}, {1.0}, 'E', 1.0}); o(raw_cut{{f}, {1e6}, 'G', 1e6}); o(raw_cut{{f}, {1e6}, 'E', 1e6}); }
            for(int z : D.zero) { o(raw_cut{{z}, {1.0}, 'L', 0.0}); o(raw_cut{{z}, {1.0}, 'E', 0.0}); o(raw_cut{{z}, {1e6}, 'L', 0.0}); }
            o(raw_cut{{D.w}, {1.0}, 'L', 4.0}); o(raw_cut{{D.pr}, {1.0}, 'L', 0.0}); o(raw_cut{{D.pr}, {1.0}, 'E', 0.0}); o(raw_cut{{D.unused}, {1.0}, 'L', 1.0}); o(raw_cut{{D.s}, {1.0}, 'L', 7.0}); }, topt{.keep = true});
        trial("float on all-fixed", std::format("rd seed={} 0.1f1+0.2f2+0.3f3<=0.6 and >=0.6 (sum is 0.6+1e-16)", seed), r, b, [&D](hctx &, const std::vector<double> &, const offer_fn & o) {
            o(raw_cut{D.fixed, {0.1, 0.2, 0.3}, 'L', 0.6}); o(raw_cut{D.fixed, {0.1, 0.2, 0.3}, 'G', 0.6}); o(raw_cut{D.fixed, {0.1, 0.2, 0.3}, 'E', 0.6}); }, topt{.keep = true});
        trial("one-term by integrality", std::format("rd seed={} y1<=1, violated at the root", seed), r, b, [&D](hctx &, const std::vector<double> & x, const offer_fn & o) { raw_cut k{{D.y1}, {1.0}, 'L', 1.0}; if(k.violation(x) > 1e-6) o(k); });
    }
}
// Rewrites a valid cut with terms tight at the point: multiples of equalities (either sign), bound terms on
// columns at that bound. Stays valid, stays violated, now mentions columns presolve removes.
static void d_rewrite(unsigned seeds) {
    for(unsigned seed = 1; seed <= seeds; ++seed) {
        const rdm D = make_rd(seed);
        const sol b = base_of(D.r);
        for(double scale : {1.0, 1e3, 1e6}) {
            auto gen = std::make_shared<std::mt19937>(seed * 101u);
            trial("rewriter", std::format("rd seed={} lambda up to {:g}", seed, scale), D.r, b, [&D, gen, scale](hctx &, const std::vector<double> & x, const offer_fn & o) {
                std::vector<raw_cut> base = covers(D.G, x);
                std::mt19937 local((*gen)());
                for(auto & c : cg_cuts(D.G, x, local, 2)) base.push_back(std::move(c));
                raw_cut y{{D.y1}, {1.0}, 'L', 1.0}; if(y.violation(x) > 1e-4) base.push_back(y);
                std::uniform_real_distribution<double> L(0.25, 2.0);
                for(auto k : base) {
                    for(const auto & e : D.eqs) { const double l = L(*gen) * scale * ((*gen)() % 2 ? 1 : -1); for(size_t i = 0; i < e.ind.size(); ++i) { k.ind.push_back(e.ind[i]); k.val.push_back(l * e.val[i]); } k.rhs += l * e.rhs; }
                    const auto bound = [&](int j, double ub, double lb) {
                        const double v = x[static_cast<size_t>(j)], l = L(*gen) * scale;
                        if(v >= ub - 1e-9) { k.ind.push_back(j); k.val.push_back(l); k.rhs += l * ub; }
                        else if(v <= lb + 1e-9) { k.ind.push_back(j); k.val.push_back(-l); k.rhs += -l * lb; }
                    };
                    for(int f : D.fixed) bound(f, 1, 1);
                    for(int z : D.zero) bound(z, 0, 0);
                    bound(D.w, 4, 0); bound(D.unused, 1, 0); bound(D.p1, 3, 0); bound(D.p2, 3, 0); bound(D.d1, 1, 0); bound(D.d2, 1, 0); bound(D.pr, 1, 0);
                    o(k);
                } });
        }
    }
}
static void d_prune() {
    for(unsigned seed = 1; seed <= 12; ++seed) {
        const cgm G = make_cg(seed, 45, 15);
        const sol b = base_of(G.r);
        long hits = 0;
        trial("prune", std::format("cg45 seed={} cuts only where the LP is within 1 of the incumbent", seed), G.r, b, [&G, &hits](hctx & h, const std::vector<double> & x, const offer_fn & o) {
            if(!(h.best > -1e99)) return;
            double v = G.r.eval(x);
            if(v > h.best + 1.0) return;
            ++hits;
            for(const auto & c : covers(G, x, 1e-6)) o(c);
        });
    }
    for(unsigned seed : {1u, 9u}) {
        auto view = std::make_shared<cp::atsp>(); std::vector<int> u; const rawm r = make_atsp_raw(44, seed, *view, u);
        const sol b = base_of(r);
        if(b.status != 2) { std::println(stderr, "prune atsp44 seed={}: baseline {} skipped", seed, sname(b.status)); continue; }
        trial("prune", std::format("atsp44 seed={} DFJ only where LP within 1 of the incumbent", seed), r, b, [&r, view](hctx & h, const std::vector<double> & x, const offer_fn & o) {
            if(!(h.best < 1e99)) return;
            if(r.eval(x) < h.best - 1.0) return;
            for(const auto & c : view->separate(x)) o(c);
        });
    }
}
static void d_flood() {
    for(unsigned seed = 1; seed <= 3; ++seed) {
        const cgm G = make_cg(seed);
        const sol b = base_of(G.r);
        for(int n : {1000, 20000}) trial("flood", std::format("cg seed={} {} near-duplicate covers at the first call", seed, n), G.r, b, [&G, n, done = std::make_shared<bool>(false)](hctx &, const std::vector<double> & x, const offer_fn & o) {
            if(*done) return; const auto cv = covers(G, x); if(cv.empty()) return; *done = true;
            for(int e = 0; e < n; ++e) { auto k = cv[static_cast<size_t>(e) % cv.size()]; k.rhs += e * 1e-10; o(k); } });
    }
}
static void d_mtz() {
    for(auto [n, seed] : {std::pair{12, 1u}, std::pair{14, 2u}, std::pair{16, 3u}, std::pair{18, 4u}, std::pair{20, 5u}}) {
        auto view = std::make_shared<cp::atsp>(); std::vector<int> u; const rawm r = make_atsp_raw(n, seed, *view, u);
        const sol b = base_of(r, 1, 0.0);
        std::vector<raw_cut> all;
        const auto N = static_cast<size_t>(n);
        for(size_t i = 1; i < N; ++i) for(size_t j = 1; j < N; ++j) if(i != j)
            all.push_back(raw_cut{{u[i], u[j], view->col[i][j], view->col[j][i]}, {1, -1, static_cast<double>(n - 1), static_cast<double>(n - 4)}, 'L', static_cast<double>(n - 2)});
        // control: as hard rows
        { rawm h = r; for(const auto & k : all) h.add(k.ind, k.val, '<', k.rhs); const sol s = base_of(h, 1, 0.0); std::println(stderr, "control lifted MTZ as hard rows n={} seed={}: {} {} vs baseline {} => {}", n, seed, sname(s.status), s.value, b.value, std::fabs(s.value - b.value) < 1e-6 ? "valid" : "INVALID"); }
        trial("lifted MTZ", std::format("atsp{} seed={} lifted MTZ on continuous u", n, seed), r, b, [all](hctx &, const std::vector<double> & x, const offer_fn & o) { int c = 0; for(const auto & k : all) if(k.violation(x) > 1e-4 && c++ < 20) o(k); });
        trial("arcs equality", std::format("atsp{} seed={} Sum all arcs == n, x_ij+x_ji<=1, degree rows, DFJ, no drop", n, seed), r, b, [&r, view, n](hctx &, const std::vector<double> & x, const offer_fn & o) {
            raw_cut k{{}, {}, 'E', static_cast<double>(n)};
            for(int i = 0; i < n; ++i) for(int j = 0; j < n; ++j) if(i != j) { k.ind.push_back(view->col[static_cast<size_t>(i)][static_cast<size_t>(j)]); k.val.push_back(1); }
            o(k);
            for(int i = 0; i < n; ++i) for(int j = i + 1; j < n; ++j) o(raw_cut{{view->col[static_cast<size_t>(i)][static_cast<size_t>(j)], view->col[static_cast<size_t>(j)][static_cast<size_t>(i)]}, {1.0, 1.0}, 'L', 1.0});
            for(int t = 0; t < 2 * n; ++t) o(raw_cut{r.rows[static_cast<size_t>(t)].ind, r.rows[static_cast<size_t>(t)].val, 'E', 1.0});
            for(const auto & c : view->separate(x)) o(c);
        }, topt{.keep = true});
    }
}
static void d_infeasible() {
    // Market split rows: LP feasible, no subset sums to the right hand sides (checked by enumeration).
    for(unsigned seed = 1; seed <= 6; ++seed) {
        const int n = 18, rows = 2;
        std::mt19937 rng(seed);
        std::uniform_int_distribution<int> A(1, 99);
        std::vector<std::vector<int>> a(rows, std::vector<int>(n));
        for(auto & v : a) for(auto & e : v) e = A(rng);
        std::vector<int> rhs(rows); for(int i = 0; i < rows; ++i) { int s = 0; for(int e : a[static_cast<size_t>(i)]) s += e; rhs[static_cast<size_t>(i)] = s / 2; }
        bool feasible = true;
        for(int tries = 0; tries < 200; ++tries) {
            feasible = false;
            for(int m = 0; m < (1 << n) && !feasible; ++m) { bool ok = true; for(int i = 0; i < rows && ok; ++i) { int s = 0; for(int j = 0; j < n; ++j) if(m >> j & 1) s += a[static_cast<size_t>(i)][static_cast<size_t>(j)]; ok = s == rhs[static_cast<size_t>(i)]; } feasible = ok; }
            if(!feasible) break;
            ++rhs[0];
        }
        if(feasible) { std::println(stderr, "market split seed={}: no infeasible rhs found", seed); continue; }
        rawm r; r.sense = -1; std::vector<int> x; for(int j = 0; j < n; ++j) x.push_back(r.bin(1 + j % 3));
        for(int i = 0; i < rows; ++i) { std::vector<double> v; for(int e : a[static_cast<size_t>(i)]) v.push_back(e); r.add(x, v, '=', rhs[static_cast<size_t>(i)]); }
        const sol b = base_of(r);
        std::println(stderr, "market split seed={}: enumeration says infeasible; baseline {} nodes={}", seed, sname(b.status), b.nodes);
        trial("infeasible model", std::format("market split seed={} vacuous cut x0<=-1", seed), r, b, [x](hctx &, const std::vector<double> &, const offer_fn & o) { o(raw_cut{{x[0]}, {1.0}, 'L', -1.0}); });
        trial("infeasible model", std::format("market split seed={} vacuous cut Sum x <= -1", seed), r, b, [x](hctx &, const std::vector<double> &, const offer_fn & o) { o(raw_cut{x, std::vector<double>(x.size(), 1.0), 'L', -1.0}); });
        trial("infeasible model", std::format("market split seed={} no cut, hook only", seed), r, b, [](hctx &, const std::vector<double> &, const offer_fn &) {});
    }
}
static void d_indicator() {
    for(unsigned seed : {1u, 2u, 3u, 4u}) for(double K : {1.0, 1e3, 1e6, 1e9}) {
        // y_i = 0 => Sum f_ij <= 0, as an indicator, no big-M row
        flm D = make_fl(seed, 10, 16);
        rawm r = D.r; const size_t rows_before = r.rows.size(); (void)rows_before;
        const size_t fac = D.y.size(), cli = D.d.size();
        for(size_t i = 0; i < fac; ++i) r.rows.pop_back();
        double tot = 0; for(double v : D.d) tot += v;
        for(size_t i = 0; i < fac; ++i) { std::vector<int> ind; std::vector<double> val; for(size_t j = 0; j < cli; ++j) { ind.push_back(D.f[i][j]); val.push_back(1); } r.add(ind, val, '<', std::ceil(tot / 3)); }
        const auto add_ind = [&](M & m) {
            auto [env, model] = m.native_model();
            auto & api = m.native_api();
            for(size_t i = 0; i < fac; ++i) { std::vector<int> ind; std::vector<double> val; for(size_t j = 0; j < cli; ++j) { ind.push_back(D.f[i][j]); val.push_back(1); } api._check(env, api.addgenconstrIndicator(model, nullptr, D.y[i], 0, static_cast<int>(ind.size()), ind.data(), val.data(), '<', 0.0)); }
            api._check(env, api.updatemodel(model));
        };
        M bm; seti(bm, "OutputFlag", 0); seti(bm, "Threads", 1); setd(bm, "TimeLimit", g_limit); r.load(bm); add_ind(bm); const sol b = solve_native(bm);
        trial("indicator", std::format("fl+indicator seed={} K={:g} K f - K d y <= 0", seed, K), r, b, [&D, K](hctx &, const std::vector<double> & x, const offer_fn & o) { for(const auto & k : fl_cuts(D, x, K)) o(k); }, topt{.before = [&](M & m, const sol &) { add_ind(m); }});
    }
}
static void d_mipstart() {
    for(unsigned seed = 1; seed <= 6; ++seed) {
        const cgm G = make_cg(seed);
        const sol b = base_of(G.r);
        trial("mip start", std::format("cg seed={} optimum as MIP start, covers and CG", seed), G.r, b, [&G, gen = std::make_shared<std::mt19937>(seed)](hctx &, const std::vector<double> & x, const offer_fn & o) { for(const auto & c : covers(G, x)) o(c); for(const auto & c : cg_cuts(G, x, *gen)) o(c); },
              topt{.before = [](M & m, const sol & bb) { auto [env, model] = m.native_model(); auto x = bb.x; m.native_api().setdblattrarray(model, "Start", 0, static_cast<int>(x.size()), x.data()); m.native_api().updatemodel(model); }});
    }
}
static void d_fuzz(unsigned seeds, size_t cols, size_t rows) {
    for(unsigned seed = 1; seed <= seeds; ++seed) {
        const cgm G = make_cg(seed + 1000, cols, rows);
        const sol b = base_of(G.r);
        if(b.status != 2) continue;
        auto gen = std::make_shared<std::mt19937>(seed);
        trial("fuzz", std::format("cg{}x{} seed={}", cols, rows, seed), G.r, b, [&G, gen](hctx &, const std::vector<double> & x, const offer_fn & o) {
            for(const auto & c : covers(G, x)) o(c);
            for(const auto & c : cg_cuts(G, x, *gen, 5)) o(c);
        });
    }
}

// Non power-of-two scales: K*rhs rounds; an integer-rounding solver could floor |S|-1 - 1e-16 down.
static void d_nasty() {
    const std::vector<double> Ks = {0.1, 1.0 / 3, 0.7, 1.0 / 7, 1.1, 3.3, 1.0 / 49, 1e-3 / 7, 1e3 / 7, 1.0 / 3e5, 12345.678};
    for(unsigned seed = 1; seed <= 8; ++seed) {
        const cgm G = make_cg(seed);
        const sol b = base_of(G.r);
        for(double K : Ks) trial("scale nasty", std::format("cg seed={} K={:.6g}", seed, K), G.r, b, [&G, K](hctx &, const std::vector<double> & x, const offer_fn & o) { for(auto k : covers(G, x)) { for(auto & v : k.val) v *= K; k.rhs *= K; o(k); } });
    }
}
// Cuts tight at the known optimum with the rhs a hair below the integer: not valid in exact arithmetic, but
// within tolerance. What a float separator produces. Records whether rounding turns it into a lost optimum.
static void d_roundoff() {
    for(unsigned seed = 1; seed <= 12; ++seed) {
        const cgm G = make_cg(seed);
        const sol b = base_of(G.r, 1, 0.0);
        raw_cut obj{G.x, G.c, 'L', b.value};
        for(double e : {1e-14, 1e-12, 1e-10, 1e-9, 1e-8, 1e-7, 1e-6, 1e-5}) {
            trial("objective minus eps", std::format("cg seed={} c'x<=z*-{:g}", seed, e), G.r, b, [obj, e](hctx &, const std::vector<double> & x, const offer_fn & o) { auto k = obj; k.rhs -= e; if(k.violation(x) > 1e-7) o(k); });
        }
        // Rows tight at the optimum, written back with a slack of -eps: same class.
        std::vector<raw_cut> tight;
        for(size_t i = 0; i < G.a.size(); ++i) { double act = 0; for(size_t j = 0; j < G.x.size(); ++j) act += G.a[i][j] * b.x[static_cast<size_t>(G.x[j])]; if(act >= G.b[i] - 1e-9) tight.push_back(raw_cut{G.x, G.a[i], 'L', G.b[i]}); }
        for(double e : {1e-12, 1e-9, 1e-7}) trial("tight row minus eps", std::format("cg seed={} {} tight rows, rhs-{:g}", seed, tight.size(), e), G.r, b, [tight, e](hctx &, const std::vector<double> &, const offer_fn & o) { for(auto k : tight) { k.rhs -= e; o(k); } }, topt{.keep = true});
    }
}
// MIR on tight rows: fractional coefficients, valid for binary x >= 0.
static std::vector<raw_cut> mir_cuts(const cgm & G, const std::vector<double> & p, std::mt19937 & gen, int want = 4) {
    // Complemented MIR on one tight row, or a sum of two: x_T = 1 - xbar_T for columns above 0.5, divisor d among
    // the coefficients of fractional columns. Valid for binary x; coefficients are fractional doubles.
    std::vector<raw_cut> out;
    std::vector<size_t> tight;
    for(size_t i = 0; i < G.a.size(); ++i) { double act = 0; for(size_t j = 0; j < G.x.size(); ++j) act += G.a[i][j] * p[static_cast<size_t>(G.x[j])]; if(act >= G.b[i] - 1e-6) tight.push_back(i); }
    if(tight.empty()) return out;
    std::uniform_int_distribution<size_t> R(0, tight.size() - 1);
    const auto F = [](double t, double f0) { const double fj = t - std::floor(t); return std::floor(t) + std::max(fj - f0, 0.0) / (1 - f0); };
    for(int attempt = 0; attempt < 60 && static_cast<int>(out.size()) < want; ++attempt) {
        std::vector<double> row = G.a[tight[R(gen)]]; double rhs = 0;
        { const size_t i0 = tight[R(gen)]; (void)i0; }
        rhs = 0; { size_t i = tight[R(gen)]; row = G.a[i]; rhs = G.b[i]; if(attempt % 2) { size_t i2 = tight[R(gen)]; if(i2 != i) { for(size_t j = 0; j < row.size(); ++j) row[j] += G.a[i2][j]; rhs += G.b[i2]; } } }
        std::vector<char> T(row.size(), 0); double b2 = rhs;
        for(size_t j = 0; j < row.size(); ++j) if(p[static_cast<size_t>(G.x[j])] > 0.5) { T[j] = 1; b2 -= row[j]; }
        std::vector<double> ds;
        for(size_t j = 0; j < row.size(); ++j) { const double v = p[static_cast<size_t>(G.x[j])]; if(v > 1e-6 && v < 1 - 1e-6 && std::fabs(row[j]) > 1e-9) ds.push_back(std::fabs(row[j])); }
        if(ds.empty()) continue;
        for(double d : ds) {
            const double br = b2 / d; const double f0 = br - std::floor(br);
            if(f0 < 0.02 || f0 > 0.98) continue;
            raw_cut c{{}, {}, 'L', std::floor(br)};
            for(size_t j = 0; j < row.size(); ++j) {
                if(T[j]) { const double co = F(-row[j] / d, f0); c.ind.push_back(G.x[j]); c.val.push_back(-co); c.rhs -= co; }
                else { c.ind.push_back(G.x[j]); c.val.push_back(F(row[j] / d, f0)); }
            }
            if(c.violation(p) > 1e-4) { out.push_back(std::move(c)); break; }
        }
    }
    return out;
}
static void d_mir(unsigned seeds) {
    for(unsigned seed = 1; seed <= seeds; ++seed) {
        const cgm G = make_cg(seed + 2000, 45, 15);
        const sol b = base_of(G.r);
        trial("MIR fractional", std::format("cg45 seed={}", seed), G.r, b, [&G, gen = std::make_shared<std::mt19937>(seed)](hctx &, const std::vector<double> & x, const offer_fn & o) { for(const auto & c : mir_cuts(G, x, *gen)) o(c); });
    }
}
// Independent components, and valid cuts that span them.
static void d_disconnected() {
    for(unsigned seed = 1; seed <= 8; ++seed) {
        rawm r; std::vector<cgm> comps;
        for(unsigned c = 0; c < 4; ++c) comps.push_back(make_cg(seed * 10 + c, 20, 8));
        // concatenate with shifted columns
        std::vector<int> shift; for(const auto & G : comps) { shift.push_back(static_cast<int>(r.obj.size())); for(size_t j = 0; j < G.x.size(); ++j) r.bin(G.c[j]); }
        for(size_t c = 0; c < comps.size(); ++c) for(const auto & w : comps[c].r.rows) { auto i = w.ind; for(auto & v : i) v += shift[c]; r.add(i, w.val, w.sense, w.rhs); }
        r.sense = -1;
        const sol b = base_of(r, 1, 0.0);
        std::vector<int> all; std::vector<double> allc; for(size_t j = 0; j < r.obj.size(); ++j) { all.push_back(static_cast<int>(j)); allc.push_back(r.obj[j]); }
        trial("disconnected", std::format("4 components seed={} objective cut spanning all", seed), r, b, [=](hctx &, const std::vector<double> & x, const offer_fn & o) { raw_cut k{all, allc, 'L', b.value}; if(k.violation(x) > 1e-7) o(k); });
        trial("disconnected", std::format("4 components seed={} sums of covers across components", seed), r, b, [&comps, shift](hctx &, const std::vector<double> & x, const offer_fn & o) {
            std::vector<std::vector<raw_cut>> per;
            for(size_t c = 0; c < comps.size(); ++c) {
                std::vector<double> sub(comps[c].x.size()); for(size_t j = 0; j < sub.size(); ++j) sub[j] = x[static_cast<size_t>(shift[c]) + j];
                auto cv = covers(comps[c], sub, -1e9);
                for(auto & k : cv) for(auto & i : k.ind) i += shift[c];
                per.push_back(std::move(cv));
            }
            if(per[0].empty() || per[1].empty()) return;
            for(const auto & a : per[0]) for(const auto & d : per[1]) { raw_cut k = a; k.ind.insert(k.ind.end(), d.ind.begin(), d.ind.end()); k.val.insert(k.val.end(), d.val.begin(), d.val.end()); k.rhs += d.rhs; if(k.violation(x) > 1e-4) o(k); }
        });
        trial("disconnected", std::format("4 components seed={} covers per component", seed), r, b, [&comps, shift](hctx &, const std::vector<double> & x, const offer_fn & o) {
            for(size_t c = 0; c < comps.size(); ++c) { std::vector<double> sub(comps[c].x.size()); for(size_t j = 0; j < sub.size(); ++j) sub[j] = x[static_cast<size_t>(shift[c]) + j]; for(auto k : covers(comps[c], sub)) { for(auto & i : k.ind) i += shift[c]; o(k); } }
        });
    }
}
// Bin packing: bins are symmetric, cuts on one bin only are asymmetric and valid.
static void d_symmetry() {
    for(unsigned seed = 1; seed <= 6; ++seed) {
        rawm r; r.sense = 1;
        std::mt19937 rng(seed);
        std::uniform_int_distribution<int> W(20, 50);
        const int items = 14, bins = 7; const double cap = 100;
        std::vector<double> w(items); double tot = 0; for(auto & v : w) { v = W(rng); tot += v; }
        std::vector<int> y; for(int j = 0; j < bins; ++j) y.push_back(r.bin(1));
        std::vector<std::vector<int>> x(items, std::vector<int>(bins));
        for(auto & row : x) for(auto & v : row) v = r.bin(0);
        for(int i = 0; i < items; ++i) { std::vector<int> ind; for(int j = 0; j < bins; ++j) ind.push_back(x[static_cast<size_t>(i)][static_cast<size_t>(j)]); r.add(ind, std::vector<double>(ind.size(), 1.0), '=', 1); }
        for(int j = 0; j < bins; ++j) { std::vector<int> ind; std::vector<double> val; for(int i = 0; i < items; ++i) { ind.push_back(x[static_cast<size_t>(i)][static_cast<size_t>(j)]); val.push_back(w[static_cast<size_t>(i)]); } ind.push_back(y[static_cast<size_t>(j)]); val.push_back(-cap); r.add(ind, val, '<', 0); }
        const sol b = base_of(r, 1, 0.0);
        trial("symmetry", std::format("bin packing seed={} x_ij<=y_j only on bin 0", seed), r, b, [=](hctx &, const std::vector<double> & p, const offer_fn & o) { for(int i = 0; i < items; ++i) { raw_cut k{{x[static_cast<size_t>(i)][0], y[0]}, {1.0, -1.0}, 'L', 0.0}; if(k.violation(p) > 1e-4) o(k); } });
        trial("symmetry", std::format("bin packing seed={} x_ij<=y_j on all bins", seed), r, b, [=](hctx &, const std::vector<double> & p, const offer_fn & o) { for(int i = 0; i < items; ++i) for(int j = 0; j < bins; ++j) { raw_cut k{{x[static_cast<size_t>(i)][static_cast<size_t>(j)], y[static_cast<size_t>(j)]}, {1.0, -1.0}, 'L', 0.0}; if(k.violation(p) > 1e-4) o(k); } });
        trial("symmetry", std::format("bin packing seed={} Sum y >= ceil(W/C), objective tight", seed), r, b, [=](hctx &, const std::vector<double> & p, const offer_fn & o) { raw_cut k{y, std::vector<double>(y.size(), 1.0), 'G', std::ceil(tot / cap)}; if(k.violation(p) > 1e-4) o(k); raw_cut z{y, std::vector<double>(y.size(), 1.0), 'G', b.value}; if(z.violation(p) > 1e-4) o(z); });
    }
}
// The G4 model, where PreCrush=1 fixes b1 = 3. Valid cuts through b1, b2 only.
static void d_dualfix() {
    for(unsigned seed = 1; seed <= 8; ++seed) for(double bobj : {45.0, 150.0}) {
        // core knapsack with b1, b2 worth a lot
        rawm r; r.sense = -1; const size_t cols = 40, rows = 5;
        std::mt19937 rng(seed);
        std::uniform_int_distribution<int> A(1, 1000), R(0, 500);
        std::vector<std::vector<double>> a(rows, std::vector<double>(cols));
        for(auto & row : a) for(auto & v : row) v = A(rng);
        std::vector<int> xs; for(size_t j = 0; j < cols; ++j) { double t = 0; for(auto & row : a) t += row[j]; xs.push_back(r.bin(std::floor(t / rows + R(rng)))); }
        for(size_t i = 0; i < rows; ++i) { double t = 0; for(double v : a[i]) t += v; r.add(xs, a[i], '<', std::floor(t / 2)); }
        std::vector<int> order; { const sol sc = base_of(r, 1); for(size_t j = 0; j < sc.x.size(); ++j) if(sc.x[j] > 0.5) order.push_back(static_cast<int>(j)); if(order.size() < 4) continue; }
        const int b1 = r.var(bobj, 0, 3, 'I'), b2 = r.var(bobj, 0, 3, 'I');
        r.add({b1, b2, order[0], order[1]}, {50, 50, 500, 700}, '<', 1500);
        r.add({b1, b2, order[2], order[3]}, {70, 70, 300, 900}, '<', 1500);
        const sol b = base_of(r, 1, 0.0);
        rawm ob = r; std::ranges::fill(ob.obj, 0.0); ob.obj[static_cast<size_t>(b1)] = 1; ob.obj[static_cast<size_t>(b2)] = 1;
        const double kmax = std::round(base_of(ob, 1).value);
        rawm oc = r; std::ranges::fill(oc.obj, 0.0); oc.obj[static_cast<size_t>(b1)] = 1;
        const double b1max = std::round(base_of(oc, 1).value);
        std::println(stderr, "dualfix seed={} bobj={}: k at optimum {} kmax={} b1max={}", seed, bobj, b.x[static_cast<size_t>(b1)] + b.x[static_cast<size_t>(b2)], kmax, b1max);
        trial("dualfix valid", std::format("seed={} bobj={} b1+b2<=kmax={}", seed, bobj, kmax), r, b, [=](hctx &, const std::vector<double> &, const offer_fn & o) { o(raw_cut{{b1, b2}, {1.0, 1.0}, 'L', kmax}); }, topt{.keep = true});
        trial("dualfix valid", std::format("seed={} bobj={} b1<=b1max, b2<=b1max", seed, bobj), r, b, [=](hctx &, const std::vector<double> &, const offer_fn & o) { o(raw_cut{{b1}, {1.0}, 'L', b1max}); o(raw_cut{{b2}, {1.0}, 'L', b1max}); }, topt{.keep = true});
        trial("dualfix valid", std::format("seed={} bobj={} objective tight, sums over b", seed, bobj), r, b, [=, &r](hctx &, const std::vector<double> & x, const offer_fn & o) { std::vector<int> i; std::vector<double> v; for(size_t j = 0; j < r.obj.size(); ++j) { i.push_back(static_cast<int>(j)); v.push_back(r.obj[j]); } raw_cut k{i, v, 'L', b.value}; if(k.violation(x) > 1e-7) o(k); o(raw_cut{{b1, b2}, {1.0, 1.0}, 'L', kmax}); });
    }
}
// Is the relaxation point feasible for the user model? Manual: "not necessarily".
static void d_relcheck() {
    for(unsigned seed = 1; seed <= 6; ++seed) for(int precrush : {0, 1}) {
        const rdm D = make_rd(seed);
        M m; seti(m, "OutputFlag", 0); seti(m, "Threads", 1); setd(m, "TimeLimit", 20); seti(m, "PreCrush", precrush); D.r.load(m);
        long calls = 0, bad_row = 0, bad_bound = 0; double worst = 0;
        hctx c; c.on_node = [&](hctx &, const std::vector<double> & x) {
            ++calls; bool br = false, bb = false;
            for(const auto & w : D.r.rows) { double a = 0; for(size_t k = 0; k < w.ind.size(); ++k) a += w.val[k] * x[static_cast<size_t>(w.ind[k])]; const double v = w.sense == '<' ? a - w.rhs : w.sense == '>' ? w.rhs - a : std::fabs(a - w.rhs); worst = std::max(worst, v); br |= v > 1e-5; }
            for(size_t j = 0; j < x.size(); ++j) { const double v = std::max(D.r.lb[j] - x[j], x[j] - D.r.ub[j]); worst = std::max(worst, v); bb |= v > 1e-5; }
            bad_row += br; bad_bound += bb;
        };
        attach(m, c); const sol s = solve_native(m, false);
        std::println(stderr, "relcheck rd seed={} PreCrush={}: {} | calls={} points violating a user row={} a user bound={} worst={:.3g}", seed, precrush, line(s), calls, bad_row, bad_bound, worst);
    }
    for(unsigned seed : {1u, 2u}) for(int precrush : {0, 1}) {
        cp::atsp view; std::vector<int> u; rawm r = make_atsp_raw(26, seed, view, u);
        M m; seti(m, "OutputFlag", 0); seti(m, "Threads", 1); setd(m, "TimeLimit", 20); seti(m, "PreCrush", precrush); r.load(m);
        long calls = 0, bad_row = 0; double worst = 0;
        hctx c; c.on_node = [&](hctx &, const std::vector<double> & x) { ++calls; bool br = false; for(const auto & w : r.rows) { double a = 0; for(size_t k = 0; k < w.ind.size(); ++k) a += w.val[k] * x[static_cast<size_t>(w.ind[k])]; const double v = w.sense == '<' ? a - w.rhs : w.sense == '>' ? w.rhs - a : std::fabs(a - w.rhs); worst = std::max(worst, v); br |= v > 1e-5; } bad_row += br; };
        attach(m, c); const sol s = solve_native(m, false);
        std::println(stderr, "relcheck atsp26 seed={} PreCrush={}: {} | calls={} points violating a user row={} worst={:.3g}", seed, precrush, line(s), calls, bad_row, worst);
    }
}

// Cuts only where the LP is at or past the incumbent (the node is about to be pruned), or within 1 of it.
static void d_prune2() {
    for(unsigned seed = 1; seed <= 20; ++seed) {
        const cgm G = make_cg(seed, 45, 15);
        const sol b = base_of(G.r, 1, 0.0);
        raw_cut obj{G.x, G.c, 'L', b.value};
        for(double width : {1.0, 0.0}) {
            auto hits = std::make_shared<long>(0);
            trial("prune2", std::format("cg45 seed={} objective, covers, CG, MIR where LP <= incumbent+{}", seed, width), G.r, b, [&G, obj, width, hits, gen = std::make_shared<std::mt19937>(seed)](hctx & h, const std::vector<double> & x, const offer_fn & o) {
                if(!(h.best > -1e99)) return;
                if(G.r.eval(x) > h.best + width) return;
                ++*hits;
                if(obj.violation(x) > 1e-7) o(obj);
                for(const auto & c : covers(G, x)) o(c);
                for(const auto & c : cg_cuts(G, x, *gen)) o(c);
                for(const auto & c : mir_cuts(G, x, *gen)) o(c);
            });
            std::println(stderr, "   prune hits={}", *hits);
        }
    }
    for(unsigned seed : {3u, 9u, 10u}) {
        auto view = std::make_shared<cp::atsp>(); std::vector<int> u; const rawm r = make_atsp_raw(30, seed, *view, u);
        const sol b = base_of(r);
        if(b.status != 2) continue;
        auto hits = std::make_shared<long>(0);
        trial("prune2", std::format("atsp30 seed={} DFJ where LP >= incumbent-1", seed), r, b, [&r, view, hits](hctx & h, const std::vector<double> & x, const offer_fn & o) {
            if(!(h.best < 1e99)) return;
            if(r.eval(x) < h.best - 1.0) return;
            ++*hits;
            for(const auto & c : view->separate(x)) o(c);
        });
        std::println(stderr, "   prune hits={}", *hits);
    }
}
int main(int argc, char ** argv) {
    { M p; print_release(p); }
    const std::string w = argc > 1 ? argv[1] : "all";
    if(argc > 2) g_threads = std::atoi(argv[2]);
    const auto on = [&](const char * n) { return w == "all" || w == n; };
    if(on("scale")) d_scale();
    if(on("range")) d_range();
    if(on("objective")) d_objective();
    if(on("neardup")) d_neardup();
    if(on("rows")) d_rows_and_terms();
    if(on("rewrite")) d_rewrite(30);
    if(on("prune")) d_prune();
    if(on("flood")) d_flood();
    if(on("mtz")) d_mtz();
    if(on("infeasible")) d_infeasible();
    if(on("indicator")) d_indicator();
    if(on("mipstart")) d_mipstart();
    if(on("fuzz")) d_fuzz(120, 45, 15);
    if(w == "fuzzbig") d_fuzz(40, 80, 25);
    if(on("nasty")) d_nasty();
    if(on("roundoff")) d_roundoff();
    if(on("mir")) d_mir(100);
    if(on("disconnected")) d_disconnected();
    if(on("symmetry")) d_symmetry();
    if(on("dualfix")) d_dualfix();
    if(w == "relcheck") d_relcheck();
    if(on("prune2")) d_prune2();
    for(auto [k, v] : tally) std::println(stderr, "TALLY {}: pass={} fail={} gapdiff={} no_power={}", k, v[0], v[1], v[2], v[3]);
    return 0;
}
