// G2: valid cuts on parallel columns under default presolve, PreCrush 0 vs 1.
#include "verify_gurobi_common.hpp"
using std::chrono::duration;
// ATSP in which every arc is two parallel binaries a, b; euclidean as cp::build_atsp, so that the LP has subtours.
struct datsp { rawm r; cp::atsp view; std::vector<std::vector<int>> ca, cb; int n; };
static datsp make_datsp(int n, unsigned seed) {
    datsp D; D.n = n; D.r.sense = 1;
    const auto N = static_cast<size_t>(n);
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> P(0, 100), noise(1.0, 1.3);
    std::vector<std::pair<double, double>> pts(N);
    for(auto & p : pts) p = {P(rng), P(rng)};
    D.ca.assign(N, std::vector<int>(N, -1)); D.cb = D.ca;
    for(size_t i = 0; i < N; ++i) for(size_t j = 0; j < N; ++j) if(i != j) {
        const double d = std::round(std::hypot(pts[i].first - pts[j].first, pts[i].second - pts[j].second) * noise(rng));
        D.ca[i][j] = D.r.bin(d); D.cb[i][j] = D.r.bin(d);
    }
    std::vector<int> u(N); for(auto & v : u) v = D.r.var(0, 0, n - 1, 'C');
    for(size_t i = 0; i < N; ++i) {
        std::vector<int> o, in; for(size_t j = 0; j < N; ++j) if(i != j) { o.push_back(D.ca[i][j]); o.push_back(D.cb[i][j]); in.push_back(D.ca[j][i]); in.push_back(D.cb[j][i]); }
        D.r.add(o, std::vector<double>(o.size(), 1.0), '=', 1); D.r.add(in, std::vector<double>(in.size(), 1.0), '=', 1);
    }
    for(size_t i = 1; i < N; ++i) for(size_t j = 1; j < N; ++j) if(i != j)
        D.r.add({u[i], u[j], D.ca[i][j], D.cb[i][j]}, {1, -1, static_cast<double>(n - 1), static_cast<double>(n - 1)}, '<', n - 2);
    D.view.n = n; D.view.col.assign(N, std::vector<int>(N, -1));
    int next = 0; for(size_t i = 0; i < N; ++i) for(size_t j = 0; j < N; ++j) if(i != j) D.view.col[i][j] = next++;
    return D;
}
static std::vector<raw_cut> sep(const datsp & D, const std::vector<double> & x, size_t maxc = 5) {
    const auto N = static_cast<size_t>(D.n);
    std::vector<double> sum(N * (N - 1));
    for(size_t i = 0; i < N; ++i) for(size_t j = 0; j < N; ++j) if(i != j) sum[static_cast<size_t>(D.view.col[i][j])] = x[static_cast<size_t>(D.ca[i][j])] + x[static_cast<size_t>(D.cb[i][j])];
    std::vector<raw_cut> out;
    for(const auto & c : D.view.separate(sum, maxc)) {
        raw_cut k; k.sense = c.sense; k.rhs = c.rhs;
        for(size_t i = 0; i < N; ++i) for(size_t j = 0; j < N; ++j) if(i != j)
            if(std::ranges::find(c.ind, D.view.col[i][j]) != c.ind.end()) { k.ind.push_back(D.ca[i][j]); k.val.push_back(1); k.ind.push_back(D.cb[i][j]); k.val.push_back(1); }
        out.push_back(std::move(k));
    }
    return out;
}
// Knapsack rows over parallel general integers z_j = p_j + q_j. For a weight threshold w,
// S = {j : a_j >= w}: any k+1 units in S weigh at least (k+1) w, so Sum_S z <= k with
// k = floor((b - neg)/w) is valid (neg <= 0 the best case of the negative terms).
struct dcg { rawm r; std::vector<int> p, q; std::vector<std::vector<double>> a; std::vector<double> b; int u; };
static dcg make_dcg(unsigned seed, size_t cols, size_t rows, int u) {
    dcg D; D.u = u; D.r.sense = -1;
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> A(1, 60), C(1, 40);
    D.a.assign(rows, std::vector<double>(cols));
    for(auto & r : D.a) for(auto & v : r) v = A(rng);
    for(size_t j = 0; j < cols; ++j) { const double c = C(rng); D.p.push_back(D.r.var(c, 0, u, 'I')); D.q.push_back(D.r.var(c, 0, u, 'I')); }
    D.b.resize(rows);
    for(size_t i = 0; i < rows; ++i) {
        double t = 0; std::vector<int> ind; std::vector<double> val;
        for(size_t j = 0; j < cols; ++j) { t += D.a[i][j]; ind.push_back(D.p[j]); val.push_back(D.a[i][j]); ind.push_back(D.q[j]); val.push_back(D.a[i][j]); }
        D.b[i] = std::floor(t * u * 0.6);
        D.r.add(ind, val, '<', D.b[i]);
    }
    return D;
}
static std::vector<raw_cut> sep_dcg(const dcg & D, const std::vector<double> & x) {
    std::vector<raw_cut> out;
    for(size_t i = 0; i < D.a.size(); ++i) {
        double bestv = 1e-4; raw_cut best;
        for(double w = 1; w <= 60; w += 1) {
            raw_cut c{{}, {}, 'L', 0}; double act = 0; int cnt = 0;
            for(size_t j = 0; j < D.p.size(); ++j) if(D.a[i][j] >= w) { c.ind.push_back(D.p[j]); c.val.push_back(1); c.ind.push_back(D.q[j]); c.val.push_back(1); act += x[static_cast<size_t>(D.p[j])] + x[static_cast<size_t>(D.q[j])]; ++cnt; }
            if(!cnt) continue;
            c.rhs = std::floor(D.b[i] / w + 1e-9);
            if(act - c.rhs > bestv) { bestv = act - c.rhs; best = std::move(c); }
        }
        if(bestv > 1e-4 && !best.ind.empty()) out.push_back(std::move(best));
    }
    return out;
}
template <class Build, class Sep>
static void run(const std::string & name, Build build, Sep separator, double limit = 15) {
    for(const int cuts : {-1, 0}) for(const int precrush : {0, 1}) {
        M m; seti(m, "OutputFlag", 1); seti(m, "LogToConsole", 0); seti(m, "Threads", 1); setd(m, "TimeLimit", limit); seti(m, "PreCrush", precrush); if(cuts == 0) seti(m, "Cuts", 0);
        build().load(m);
        std::vector<raw_cut> added; std::set<std::string> keys; long checks = 0, viol = 0, first_checks = 0, first_viol = 0, callsviol = 0, calls = 0; std::string dims;
        hctx c;
        c.on_msg = [&](hctx &, const char * s) { std::string t(s); if(t.starts_with("Presolved:")) { dims = t; if(dims.back() == '\n') dims.pop_back(); } };
        c.on_node = [&](hctx & h, const std::vector<double> & x) {
            bool any = false;
            for(const auto & a : added) { ++checks; const bool v = a.violation(x) > 1e-6; viol += v; any |= v; }
            if(!added.empty()) { ++calls; callsviol += any; ++first_checks; first_viol += added.front().violation(x) > 1e-6; }
            for(const auto & k : separator(x)) {
                std::string key; for(size_t i = 0; i < k.ind.size(); ++i) key += std::format("{},", k.ind[i]); key += std::format("{}", k.rhs);
                if(h.add(k) == 0 && keys.insert(key).second) added.push_back(k);
            }
        };
        attach(m, c);
        const sol s = solve_native(m, false);
        std::println(stderr, "{} PreCrush={} Cuts={} | {} | {} | calls={} cuts_passed={} distinct={} first cut later viol/checks={}/{} all earlier cuts later viol/checks={}/{} calls with an earlier cut violated={}/{} native_err={}", name, precrush, cuts, line(s), dims, c.opt_calls, c.cuts, added.size(), first_viol, first_checks, viol, checks, callsviol, calls, c.native_err);
    }
}

// CG cuts over parallel general integers: floor(lam a_j) is the same on p_j and q_j, so each
// cut has equal coefficients on the pair. x >= 0 and integer make it valid.
static cgm as_cgm(const dcg & D) {
    cgm G; G.r = D.r;
    for(size_t j = 0; j < D.p.size(); ++j) { G.x.push_back(D.p[j]); G.x.push_back(D.q[j]); }
    G.a.assign(D.a.size(), std::vector<double>(G.x.size()));
    for(size_t i = 0; i < D.a.size(); ++i) for(size_t j = 0; j < D.p.size(); ++j) { G.a[i][2 * j] = D.a[i][j]; G.a[i][2 * j + 1] = D.a[i][j]; }
    G.b = D.b; G.c = D.r.obj;
    return G;
}
// Facility location with each flow two parallel continuous columns. f_ij <= d_j y_i is valid.
struct dfl { rawm r; std::vector<std::vector<int>> fa, fb; std::vector<int> y; std::vector<double> d; };
static dfl make_dfl(unsigned seed, size_t fac, size_t cli, bool parallel) {
    dfl D; D.r.sense = 1;
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> C(1, 40), F(100, 300), DD(5, 30);
    D.d.resize(cli); for(auto & v : D.d) v = DD(rng);
    double tot = 0; for(double v : D.d) tot += v;
    for(size_t i = 0; i < fac; ++i) D.y.push_back(D.r.bin(F(rng)));
    D.fa.assign(fac, std::vector<int>(cli)); D.fb = D.fa;
    for(size_t i = 0; i < fac; ++i) for(size_t j = 0; j < cli; ++j) { const double c = C(rng); D.fa[i][j] = D.r.var(c, 0, 1e9, 'C'); D.fb[i][j] = parallel ? D.r.var(c, 0, 1e9, 'C') : -1; }
    for(size_t j = 0; j < cli; ++j) { std::vector<int> ind; for(size_t i = 0; i < fac; ++i) { ind.push_back(D.fa[i][j]); if(parallel) ind.push_back(D.fb[i][j]); } D.r.add(ind, std::vector<double>(ind.size(), 1.0), '=', D.d[j]); }
    const double cap = std::ceil(tot / 3);
    for(size_t i = 0; i < fac; ++i) { std::vector<int> ind; std::vector<double> val; for(size_t j = 0; j < cli; ++j) { ind.push_back(D.fa[i][j]); val.push_back(1); if(parallel) { ind.push_back(D.fb[i][j]); val.push_back(1); } } ind.push_back(D.y[i]); val.push_back(-cap); D.r.add(ind, val, '<', 0); }
    return D;
}
static std::vector<raw_cut> sep_dfl(const dfl & D, const std::vector<double> & x, bool parallel) {
    std::vector<raw_cut> out;
    for(size_t i = 0; i < D.y.size(); ++i) for(size_t j = 0; j < D.d.size(); ++j) {
        raw_cut c{{D.fa[i][j]}, {1.0}, 'L', 0}; if(parallel) { c.ind.push_back(D.fb[i][j]); c.val.push_back(1); }
        c.ind.push_back(D.y[i]); c.val.push_back(-D.d[j]);
        if(c.violation(x) > 1e-4) out.push_back(std::move(c));
    }
    if(out.size() > 20) { std::ranges::sort(out, [&](const raw_cut & l, const raw_cut & r) { return l.violation(x) > r.violation(x); }); out.resize(20); }
    return out;
}

// Groups: z = p + q, p, q integers in [0,3], y1, y2 binary. The four rows sum to z <= 2.2 and every
// integer y gives z <= 1, one-variable probing only z <= 2. So p + q <= 1 and p <= 1 are valid; the LP has
// z = 2.2. A side knapsack keeps a tree.
struct zgm { rawm r; std::vector<int> p, q; };
static zgm make_zg(unsigned seed, int groups) {
    zgm D; D.r.sense = -1;
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> C(5, 25), A(1, 40);
    std::vector<int> link; std::vector<double> linkv;
    for(int g = 0; g < groups; ++g) {
        const double c = C(rng);
        const int p = D.r.var(c, 0, 3, 'I'), q = D.r.var(c, 0, 3, 'I'), y1 = D.r.bin(0), y2 = D.r.bin(0);
        D.p.push_back(p); D.q.push_back(q);
        D.r.add({p, q, y1, y2}, {5, 5, 6, 6}, '<', 17);
        D.r.add({p, q, y1, y2}, {5, 5, -6, 6}, '<', 11);
        D.r.add({p, q, y1, y2}, {5, 5, 6, -6}, '<', 11);
        D.r.add({p, q, y1, y2}, {5, 5, -6, -6}, '<', 5);
        const double w = A(rng); link.push_back(p); linkv.push_back(w); link.push_back(q); linkv.push_back(w);
    }
    double tot = 0; for(size_t k = 0; k < linkv.size(); k += 2) tot += linkv[k];
    for(int j = 0; j < 30; ++j) { const double w = A(rng); link.push_back(D.r.bin(w + std::uniform_int_distribution<int>(0, 10)(rng))); linkv.push_back(w); tot += w; }
    D.r.add(link, linkv, '<', std::floor(tot * 0.5));
    return D;
}
static void run_zg() {
    for(const int kind : {0, 1}) for(unsigned seed : {1u, 2u, 3u}) for(const int cuts : {-1, 0}) for(const int precrush : {0, 1}) {
        const zgm D = make_zg(seed, 8);
        M b; seti(b, "OutputFlag", 0); seti(b, "Threads", 1); setd(b, "TimeLimit", 20); if(cuts == 0) seti(b, "Cuts", 0); seti(b, "PreCrush", precrush); D.r.load(b);
        const sol s0 = solve_native(b, false);
        M m; seti(m, "OutputFlag", 1); seti(m, "LogToConsole", 0); seti(m, "Threads", 1); setd(m, "TimeLimit", 20); seti(m, "PreCrush", precrush); if(cuts == 0) seti(m, "Cuts", 0);
        D.r.load(m);
        std::vector<raw_cut> added; std::set<int> have; long checks = 0, viol = 0, calls_any = 0, calls = 0; std::string dims; double maxz_later = 0;
        hctx c;
        c.on_msg = [&](hctx &, const char * s) { std::string t(s); if(t.starts_with("Presolved:")) { dims = t; if(dims.back() == '\n') dims.pop_back(); } };
        c.on_node = [&](hctx & h, const std::vector<double> & x) {
            bool any = false;
            for(const auto & a : added) { ++checks; const bool v = a.violation(x) > 1e-6; viol += v; any |= v; }
            if(!added.empty()) { ++calls; calls_any += any; }
            for(size_t g = 0; g < D.p.size(); ++g) {
                raw_cut k = kind == 0 ? raw_cut{{D.p[g], D.q[g]}, {1.0, 1.0}, 'L', 1.0} : raw_cut{{D.p[g]}, {1.0}, 'L', 1.0};
                if(k.violation(x) > 1e-4 && h.add(k) == 0 && have.insert(static_cast<int>(g)).second) added.push_back(k);
            }
        };
        attach(m, c);
        const sol s = solve_native(m, false);
        std::println(stderr, "zg kind={} seed={} PreCrush={} Cuts={} | nohook {} {} nodes={} | hook {} | {} | calls={} cuts_passed={} distinct={} earlier-cut later viol/checks={}/{} calls with one violated={}/{} native_err={}", kind == 0 ? "p+q<=1" : "p<=1", seed, precrush, cuts, sname(s0.status), s0.value, s0.nodes, line(s), dims, c.opt_calls, c.cuts, added.size(), viol, checks, calls_any, calls, c.native_err);
    }
}

// Parallel integer pairs, all merged by default presolve at PreCrush=0. Truly valid cuts, proved by
// auxiliary solves: objective cut, sum bound, single-column bound, unequal coefficients on a pair.
static void run_dcgb() {
    for(unsigned seed : {1u, 2u, 3u}) {
        const dcg D = make_dcg(seed, 20, 8, 3);
        M b; seti(b, "OutputFlag", 0); seti(b, "Threads", 1); setd(b, "TimeLimit", 30); setd(b, "MIPGap", 0.0); D.r.load(b);
        const sol s0 = solve_native(b, true);
        std::vector<double> zmax;
        for(size_t j = 0; j < D.p.size(); ++j) {
            rawm r = D.r; std::ranges::fill(r.obj, 0.0); r.obj[static_cast<size_t>(D.p[j])] = 1; r.obj[static_cast<size_t>(D.q[j])] = 1;
            M a; seti(a, "OutputFlag", 0); seti(a, "Threads", 1); setd(a, "TimeLimit", 30); r.load(a);
            const sol sa = solve_native(a, false); zmax.push_back(sa.status == 2 ? std::round(sa.value) : 6.0);
        }
        std::vector<int> all; std::vector<double> allc; for(size_t j = 0; j < D.r.obj.size(); ++j) { all.push_back(static_cast<int>(j)); allc.push_back(D.r.obj[j]); }
        for(const int kind : {0, 1, 2, 3}) for(const int cuts : {-1, 0}) for(const int precrush : {0, 1}) {
            M m; seti(m, "OutputFlag", 1); seti(m, "LogToConsole", 0); seti(m, "Threads", 1); setd(m, "TimeLimit", 15); seti(m, "PreCrush", precrush); setd(m, "MIPGap", 0.0); if(cuts == 0) seti(m, "Cuts", 0);
            D.r.load(m);
            std::vector<raw_cut> pool;
            if(kind == 0) pool.push_back(raw_cut{all, allc, 'L', s0.value});
            for(size_t j = 0; j < D.p.size(); ++j) {
                if(kind == 1) pool.push_back(raw_cut{{D.p[j], D.q[j]}, {1.0, 1.0}, 'L', zmax[j]});
                if(kind == 2) { pool.push_back(raw_cut{{D.p[j]}, {1.0}, 'L', zmax[j]}); pool.push_back(raw_cut{{D.q[j]}, {1.0}, 'L', zmax[j]}); }
                if(kind == 3) pool.push_back(raw_cut{{D.p[j], D.q[j]}, {1.0, 2.0}, 'L', 2 * zmax[j]});
            }
            std::vector<char> added(pool.size(), 0); long checks = 0, viol = 0, calls_any = 0, calls = 0, first_after = 0, first_viol = 0; std::string dims; int first = -1;
            hctx c;
            c.on_msg = [&](hctx &, const char * t) { std::string l(t); if(l.starts_with("Presolved:")) { dims = l; if(dims.back() == '\n') dims.pop_back(); } };
            c.on_node = [&](hctx & h, const std::vector<double> & x) {
                bool any = false, seen = false;
                for(size_t k = 0; k < pool.size(); ++k) if(added[k]) { seen = true; ++checks; const bool v = pool[k].violation(x) > 1e-6; viol += v; any |= v; }
                if(seen) { ++calls; calls_any += any; }
                if(first >= 0) { ++first_after; first_viol += pool[static_cast<size_t>(first)].violation(x) > 1e-6; }
                for(size_t k = 0; k < pool.size(); ++k) if(pool[k].violation(x) > 1e-4) { if(h.add(pool[k]) == 0) { added[k] = 1; if(first < 0) first = static_cast<int>(k); } }
            };
            attach(m, c);
            const sol s = solve_native(m, true);
            int dist = 0; for(char a : added) dist += a;
            const char * nm[] = {"objective c'x<=z*", "p+q<=zmax", "p<=zmax,q<=zmax", "p+2q<=2zmax"};
            std::println(stderr, "dcg seed={} {:18} PreCrush={} Cuts={} | baseline {} | {} {} nodes={} | {} | calls={} cuts_passed={} distinct={} first cut later viol/checks={}/{} any earlier violated calls={}/{} viol/checks={}/{} native_err={}", seed, nm[kind], precrush, cuts, s0.value, sname(s.status), s.value, s.nodes, dims, c.opt_calls, c.cuts, dist, first_viol, first_after, calls_any, calls, viol, checks, c.native_err);
        }
    }
}

// Parallel continuous: valid cuts on one column of the pair, and with unequal coefficients.
static void run_fl1() {
    for(unsigned seed : {1u, 2u, 3u, 4u}) for(const int kind : {0, 1, 2}) for(const int cuts : {-1, 0}) for(const int precrush : {0, 1}) {
        const dfl D = make_dfl(seed, 12, 20, true);
        M m; seti(m, "OutputFlag", 1); seti(m, "LogToConsole", 0); seti(m, "Threads", 1); setd(m, "TimeLimit", 10); seti(m, "PreCrush", precrush); if(cuts == 0) seti(m, "Cuts", 0);
        D.r.load(m);
        std::vector<raw_cut> added; std::set<std::pair<size_t, size_t>> have; long checks = 0, viol = 0, firstc = 0, firstv = 0; std::string dims;
        hctx c;
        c.on_msg = [&](hctx &, const char * s) { std::string t(s); if(t.starts_with("Presolved:")) { dims = t; if(dims.back() == '\n') dims.pop_back(); } };
        c.on_node = [&](hctx & h, const std::vector<double> & x) {
            for(const auto & a : added) { ++checks; viol += a.violation(x) > 1e-6; }
            if(!added.empty()) { ++firstc; firstv += added.front().violation(x) > 1e-6; }
            int n = 0;
            for(size_t i = 0; i < D.y.size(); ++i) for(size_t j = 0; j < D.d.size(); ++j) {
                raw_cut k;
                if(kind == 0) k = raw_cut{{D.fa[i][j], D.fb[i][j], D.y[i]}, {1.0, 1.0, -D.d[j]}, 'L', 0};
                if(kind == 1) { const bool ua = x[static_cast<size_t>(D.fa[i][j])] >= x[static_cast<size_t>(D.fb[i][j])]; k = raw_cut{{ua ? D.fa[i][j] : D.fb[i][j], D.y[i]}, {1.0, -D.d[j]}, 'L', 0}; }
                if(kind == 2) k = raw_cut{{D.fa[i][j], D.fb[i][j], D.y[i]}, {1.0, 2.0, -2 * D.d[j]}, 'L', 0};
                if(k.violation(x) > 1e-4 && n < 20) { ++n; if(h.add(k) == 0 && have.insert({i, j}).second) added.push_back(k); }
            }
        };
        attach(m, c);
        const sol s = solve_native(m, false);
        const char * nm[] = {"fa+fb<=d y", "one column <=d y", "fa+2fb<=2d y"};
        std::println(stderr, "fl seed={} {:17} PreCrush={} Cuts={} | {} {} nodes={} | {} | calls={} cuts={} distinct={} first later viol/checks={}/{} all later viol/checks={}/{} native_err={}", seed, nm[kind], precrush, cuts, sname(s.status), s.value, s.nodes, dims, c.opt_calls, c.cuts, added.size(), firstv, firstc, viol, checks, c.native_err);
    }
}
int main(int argc, char ** argv) {
    { M p; print_release(p); }
    const std::string w = argc > 1 ? argv[1] : "all";
    if(w == "all" || w == "atsp") for(unsigned seed : {1u, 2u, 3u}) {
        run(std::format("parallel binary atsp n=30 seed={}", seed), [=] { return make_datsp(30, seed).r; }, [D = make_datsp(30, seed)](const std::vector<double> & x) { return sep(D, x); });
    }
    if(w == "all" || w == "cg") for(unsigned seed : {1u, 2u, 3u, 4u}) {
        const dcg D = make_dcg(seed, 20, 8, 3);
        auto gen = std::make_shared<std::mt19937>(seed * 31u);
        run(std::format("parallel ints, CG cuts, seed={}", seed), [=] { return D.r; }, [G = as_cgm(D), gen](const std::vector<double> & x) { return cg_cuts(G, x, *gen, 3); }, 10);
    }
    if(w == "all" || w == "fl") for(unsigned seed : {1u, 2u, 3u}) {
        run(std::format("parallel continuous facility location seed={}", seed), [=] { return make_dfl(seed, 12, 20, true).r; }, [D = make_dfl(seed, 12, 20, true)](const std::vector<double> & x) { return sep_dfl(D, x, true); }, 10);
        run(std::format("control no parallel facility location seed={}", seed), [=] { return make_dfl(seed, 12, 20, false).r; }, [D = make_dfl(seed, 12, 20, false)](const std::vector<double> & x) { return sep_dfl(D, x, false); }, 10);
    }
    if(w == "zg") run_zg();
    if(w == "dcgb") run_dcgb();
    if(w == "fl1") run_fl1();
    return 0;
}
