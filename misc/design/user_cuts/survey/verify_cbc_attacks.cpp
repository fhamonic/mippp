// Section D: try to break "any truly valid cut is safe under the recipe". The models are
// small enough to enumerate, so validity of every cut and the true optimum are exact.
#include "verify_cbc_recipe.hpp"
using vc::M;
using std::chrono::duration;
using cp::raw_cut;
using var_t = mippp::model_variable<int, double>;

struct emodel {
    int n = 0;
    std::vector<int> lo, hi, col;
    std::vector<std::vector<double>> a; std::vector<double> b;
    std::vector<double> c;
    bool maximize = true; double offset = 0;
    std::vector<std::int8_t> F;  // feasible points, n per point
    std::size_t count = 0;
    double opt = NAN;            // with the offset and the extras
    std::vector<int> pstar;
    // extras
    int rowless0 = -1, rowless1 = -1, fixedc = -1, slack = -1; double extra_obj = 0;
    std::vector<int> comp;  // complement columns of x_0, x_1, ...
    // gadgets
    int eq0 = -1, eq1 = -1, force0 = -1, force1 = -1;
    double dot(const std::vector<double> & d, std::size_t k) const { double s = 0; for(int j = 0; j < n; ++j) s += d[static_cast<std::size_t>(j)] * F[k * static_cast<std::size_t>(n) + static_cast<std::size_t>(j)]; return s; }
    double maxdot(const std::vector<double> & d) const { double r = -INFINITY; for(std::size_t k = 0; k < count; ++k) r = std::max(r, dot(d, k)); return r; }
};
struct variant { const char * name; int n; int rows; int pad; bool box; bool extras; bool maximize; double offset; bool market = false; };

static emodel make(M & m, const variant & V, unsigned seed) {
    emodel e; e.n = V.n; e.maximize = V.maximize; e.offset = V.offset;
    std::mt19937 rng(seed * 7919u + 13u);
    std::uniform_int_distribution<int> A(-3, 12), C(1, 30), B(0, 1), Bx(0, 2);
    const auto N = static_cast<std::size_t>(V.n);
    e.lo.assign(N, 0); e.hi.assign(N, 1);
    if(V.box) for(auto j = 0u; j < N; ++j) { const int t = Bx(rng); if(t == 1) e.hi[j] = 3; else if(t == 2) { e.lo[j] = -2; e.hi[j] = 1; } }
    e.c.resize(N); for(auto & v : e.c) v = C(rng) * (V.maximize ? 1.0 : 1.0);
    e.a.assign(static_cast<std::size_t>(V.rows), std::vector<double>(N, 0));
    for(auto & r : e.a) for(auto & v : r) v = A(rng);
    for(auto & r : e.a) {
        double up = 0; for(std::size_t j = 0; j < N; ++j) up += r[j] > 0 ? r[j] * e.hi[j] : r[j] * e.lo[j];
        e.b.push_back(std::floor(up * 0.45));
    }
    // minimizing with the same rows: require a floor, otherwise 0 is optimal
    if(!V.maximize) for(std::size_t i = 0; i < e.a.size(); ++i) { for(auto & v : e.a[i]) v = -v; double up = 0; for(std::size_t j = 0; j < N; ++j) up += e.a[i][j] > 0 ? e.a[i][j] * e.hi[j] : e.a[i][j] * e.lo[j]; e.b[i] = std::floor(up * 0.45); }
    if(V.market) {
        // near-equalities around a random point: few integer points, a weak LP, many nodes
        std::uniform_int_distribution<int> AM(1, 60);
        e.a.clear(); e.b.clear();
        std::vector<int> x0(N, 0); for(auto & v : x0) v = B(rng);
        for(int i = 0; i < V.rows; ++i) {
            std::vector<double> r(N); for(auto & v : r) v = AM(rng);
            double b0 = 0; for(std::size_t j = 0; j + 4 < N; ++j) if(x0[j]) b0 += r[j];
            for(std::size_t j = N - 4; j < N; ++j) r[j] = 0;
            e.a.push_back(r); e.b.push_back(b0 + 1);
            std::vector<double> q(N); for(std::size_t j = 0; j < N; ++j) q[j] = -r[j];
            e.a.push_back(q); e.b.push_back(-(b0 - 1));
        }
    }
    // hidden equality x_{n-2} + x_{n-1} == 1: a row pair the LP can sit strictly inside
    e.eq0 = V.n - 2; e.eq1 = V.n - 1;
    e.a.push_back(std::vector<double>(N, 0)); e.a.back()[N - 2] = 1; e.a.back()[N - 1] = 1; e.b.push_back(1.5);
    e.a.push_back(std::vector<double>(N, 0)); e.a.back()[N - 2] = -1; e.a.back()[N - 1] = -1; e.b.push_back(-0.5);
    e.lo[N - 2] = 0; e.hi[N - 2] = 1; e.lo[N - 1] = 0; e.hi[N - 1] = 1;
    // forced ones: 3 x + 3 y >= 5, the LP may keep x = 2/3 when the objective pushes down
    e.force0 = V.n - 4; e.force1 = V.n - 3;
    e.a.push_back(std::vector<double>(N, 0)); e.a.back()[N - 4] = -3; e.a.back()[N - 3] = -3; e.b.push_back(-5);
    e.lo[N - 4] = 0; e.hi[N - 4] = 1; e.lo[N - 3] = 0; e.hi[N - 3] = 1;
    e.c[N - 4] = V.maximize ? -40 : 40; e.c[N - 3] = V.maximize ? -40 : 40;
    // enumerate
    std::vector<int> p(N);
    for(std::size_t j = 0; j < N; ++j) p[j] = e.lo[j];
    double best = V.maximize ? -INFINITY : INFINITY;
    while(true) {
        bool ok = true;
        for(std::size_t i = 0; i < e.a.size() && ok; ++i) { double s = 0; for(std::size_t j = 0; j < N; ++j) s += e.a[i][j] * p[j]; ok = s <= e.b[i] + 1e-9; }
        if(ok) {
            for(int v : p) e.F.push_back(static_cast<std::int8_t>(v));
            ++e.count;
            double o = 0; for(std::size_t j = 0; j < N; ++j) o += e.c[j] * p[j];
            if(V.maximize ? o > best : o < best) { best = o; e.pstar = p; }
        }
        std::size_t j = 0;
        while(j < N && ++p[j] > e.hi[j]) { p[j] = e.lo[j]; ++j; }
        if(j == N) break;
    }
    // the model
    for(std::size_t j = 0; j < N; ++j) {
        auto v = m.add_integer_variable({.obj_coef = e.c[j], .lower_bound = static_cast<double>(e.lo[j]), .upper_bound = static_cast<double>(e.hi[j])});
        e.col.push_back(m.native_id(v));
    }
    if(V.maximize) m.set_maximization(); else m.set_minimization();
    if(V.offset != 0) m.set_objective_offset(V.offset);
    const auto X = [&](std::size_t j) { return var_t(e.col[j]); };
    const auto ids = std::views::iota(std::size_t{0}, N);
    for(std::size_t i = 0; i < e.a.size(); ++i) {
        const auto & r = e.a[i];
        if(V.extras && i == 0) {
            // row 0 as an equality with a continuous slack: presolve removes the slack
            auto s = m.add_variable({.obj_coef = 0, .lower_bound = 0, .upper_bound = m.infinity()});
            e.slack = m.native_id(s);
            m.add_constraint(xsum(ids, [&](std::size_t j) { return r[j] * X(j); }) + 1.0 * s == e.b[i]);
        } else m.add_constraint(xsum(ids, [&](std::size_t j) { return r[j] * X(j); }) <= e.b[i]);
    }
    if(V.extras) {
        auto r0 = m.add_binary_variable(); e.rowless0 = m.native_id(r0);                         // zero cost, no row
        auto r1 = m.add_variable({.obj_coef = V.maximize ? 5.0 : -5.0, .lower_bound = 0, .upper_bound = 1});
        e.rowless1 = m.native_id(r1); e.extra_obj = V.maximize ? 5.0 : -5.0;                      // dual-fixed at 1
        auto f = m.add_binary_variable(); e.fixedc = m.native_id(f); m.add_constraint(1.0 * f == 1.0);
        for(std::size_t j = 0; j < 3; ++j) { auto xb = m.add_binary_variable(); e.comp.push_back(m.native_id(xb)); m.add_constraint(X(j) + 1.0 * xb == 1.0); }
    }
    if(V.pad > 0) m.add_variables(static_cast<std::size_t>(V.pad), {.obj_coef = 0, .lower_bound = 0, .upper_bound = 1});
    e.opt = best + V.offset + e.extra_obj;
    return e;
}

struct tally { long runs = 0, ok = 0, timeouts = 0, calls = 0, added = 0, below = 0, nodes = 0, caught = 0; std::vector<std::string> fails; };
static std::map<std::string, tally> T;

static raw_cut with_col(const emodel & e, const std::vector<double> & d, double r) {
    raw_cut c{{}, {}, 'L', r};
    for(int j = 0; j < e.n; ++j) if(d[static_cast<std::size_t>(j)] != 0) { c.ind.push_back(e.col[static_cast<std::size_t>(j)]); c.val.push_back(d[static_cast<std::size_t>(j)]); }
    return c;
}
// A random direction that the point violates, its rhs the exact maximum over integer points.
static std::optional<raw_cut> violated_tight(const emodel & e, const std::vector<double> & x, std::mt19937 & rng, int tries = 12, bool at_opt = false) {
    if(e.count == 0) return std::nullopt;
    const auto N = static_cast<std::size_t>(e.n);
    std::uniform_int_distribution<int> K(1, 3), Z(0, 2);
    for(int t = 0; t < tries; ++t) {
        std::vector<double> d(N, 0);
        for(std::size_t j = 0; j < N; ++j) {
            const double v = x[static_cast<std::size_t>(e.col[j])];
            const double mid = 0.5 * (e.lo[j] + e.hi[j]);
            if(Z(rng) == 0) continue;
            d[j] = (v >= mid ? 1.0 : -1.0) * K(rng);
        }
        const double r = e.maxdot(d);
        double act = 0; for(std::size_t j = 0; j < N; ++j) act += d[j] * x[static_cast<std::size_t>(e.col[j])];
        if(act <= r + 1e-4) continue;
        if(at_opt) { double ps = 0; for(std::size_t j = 0; j < N; ++j) ps += d[j] * e.pstar[j]; if(ps < r - 1e-9) continue; }
        return with_col(e, d, r);
    }
    return std::nullopt;
}
static std::string st_name(M & m) { return cp::status_name(m.get_status()); }
struct box_view { std::vector<double> lo, up; };

using gen_fn = std::function<void(const emodel &, const std::vector<double> &, const cp::add_fn &, std::mt19937 &, const box_view &, long)>;

static void attack(const std::string & name, const variant & V, int seeds, const gen_fn & gen, const std::function<void(M &)> & cfg = {}) {
    for(int s = 1; s <= seeds; ++s) {
        const auto U = static_cast<unsigned>(s);
        auto & t = T[std::format("{} / {}", name, V.name)];
        M m; emodel e = make(m, V, U);
        m.set_time_limit(duration<double>(30));
        if(cfg) cfg(m);
        if(e.count == 0) continue;
        auto rng = std::make_shared<std::mt19937>(U * 104729u);
        auto box = std::make_shared<box_view>();
        long call = 0;
        m.attach([&, rng, box](const std::vector<double> & x, const cp::add_fn & add) {
            if(++call > 2500) return;   // enumeration costs: the tally records what ran
            static const int skip = vc::env_int("ATTACK_SKIP", 0);  // cuts only after the first calls
            if(call <= skip) return;
            gen(e, x, add, *rng, *box, call);
        });
        m.hook->observer = [&m, box](void * osi, void *, int, int) {
            const auto & A = *m.hook->abi; const int n = A.ncols(osi);
            box->lo.assign(A.lb(osi), A.lb(osi) + n); box->up.assign(A.ub(osi), A.ub(osi) + n);
        };
        cp::timed_solve(m);
        ++t.runs; t.calls += m.hook->c.calls.load(); t.added += m.hook->c.added.load(); t.below += m.hook->c.below_root.load(); t.caught += m.hook->c.caught.load(); t.nodes += static_cast<long>(m.get_node_count());
        const bool optimal = mippp::is_a<mippp::status::optimal>(m.get_status());
        const double v = cf::value_of(m);
        if(mippp::is_a<mippp::status::time_limit>(m.get_status())) { ++t.timeouts; continue; }
        if(optimal && cf::same(v, e.opt)) ++t.ok;
        else t.fails.push_back(std::format("seed {} value {} opt {} status {} nodes {} {}", s, v, e.opt, cp::status_name(m.get_status()), m.get_node_count(), m.hook->c.line()));
    }
}
static void report() {
    for(const auto & [k, t] : T) {
        std::println(stderr, "{}: runs {} ok {} timeouts {} fails {} | calls {} below_root {} added {} nodes {} caught {}", k, t.runs, t.ok, t.timeouts, t.fails.size(), t.calls, t.below, t.added, t.nodes, t.caught);
        for(const auto & f : t.fails) std::println(stderr, "   FAIL {}", f);
    }
}

// One failing run, taken apart: are the cuts valid, and do the same rows fail with no hook?
static void cmd_diag(const std::vector<variant> & vars, int vi, unsigned seed, double f) {
    const variant & V = vars[static_cast<std::size_t>(vi)];
    std::vector<raw_cut> added;
    double opt;
    {
        M m; emodel e = make(m, V, seed); m.set_time_limit(duration<double>(30)); opt = e.opt;
        auto rng = std::make_shared<std::mt19937>(seed * 104729u);
        long call = 0;
        m.attach([&](const std::vector<double> & x, const cp::add_fn & add) {
            if(++call > 2500) return;
            if(auto c = violated_tight(e, x, *rng)) { for(auto & v : c->val) v *= f; c->rhs *= f; added.push_back(*c); add(*c); }
        });
        cp::timed_solve(m);
        double worst = -INFINITY, worst_rel = -INFINITY; std::size_t tight_at_star = 0;
        std::vector<double> pst(m.num_variables(), 0.0); for(int j = 0; j < e.n; ++j) pst[static_cast<std::size_t>(e.col[static_cast<std::size_t>(j)])] = e.pstar[static_cast<std::size_t>(j)];
        for(const auto & c : added) {
            std::vector<double> d(static_cast<std::size_t>(e.n), 0.0);
            for(std::size_t k = 0; k < c.ind.size(); ++k) for(int j = 0; j < e.n; ++j) if(e.col[static_cast<std::size_t>(j)] == c.ind[k]) d[static_cast<std::size_t>(j)] = c.val[k];
            const double mx = e.maxdot(d);
            worst = std::max(worst, mx - c.rhs); worst_rel = std::max(worst_rel, (mx - c.rhs) / std::max(1e-300, std::fabs(f)));
            if(std::fabs(c.activity(pst) - c.rhs) <= 1e-9 * std::fabs(f)) ++tight_at_star;
        }
        std::println(stderr, "DIAG hooked: {} value {} opt {} nodes {} | {} cuts; max over integer points minus rhs: {} (unscaled {}); {} tight at x* | {}", st_name(m), cf::value_of(m), e.opt, m.get_node_count(), added.size(), worst, worst_rel, tight_at_star, m.hook->c.line());
    }
    {
        M m; emodel e = make(m, V, seed); m.set_time_limit(duration<double>(30)); m.param("preprocess", "off");
        for(const auto & c : added) {
            auto keys = std::views::iota(std::size_t{0}, c.ind.size());
            m.add_constraint(xsum(keys, [&c](std::size_t k) { return c.val[k] * var_t(c.ind[k]); }) <= c.rhs);
        }
        cp::timed_solve(m);
        std::println(stderr, "DIAG same rows in the model, no hook: {} value {} opt {} nodes {}", st_name(m), cf::value_of(m), opt, m.get_node_count());
    }
    {
        M m; emodel e = make(m, V, seed); m.set_time_limit(duration<double>(30)); m.param("preprocess", "off"); m.param("cuts", "off"); m.param("heuristics", "off"); m.param("presolve", "off");
        for(const auto & c : added) {
            auto keys = std::views::iota(std::size_t{0}, c.ind.size());
            m.add_constraint(xsum(keys, [&c](std::size_t k) { return c.val[k] * var_t(c.ind[k]); }) <= c.rhs);
        }
        cp::timed_solve(m);
        std::println(stderr, "DIAG same rows, cuts/heuristics/presolve off, no hook: {} value {} opt {} nodes {}", st_name(m), cf::value_of(m), opt, m.get_node_count());
    }
}

// Larger models where validity is by construction: covers on knapsacks, DFJ on ATSP.
static cp::separator mkp_covers(M & m, unsigned seed, int pad) {
    const std::size_t cols = 35, rows = 5;
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> A(1, 100), R(0, 40);
    std::vector<std::vector<double>> a(rows, std::vector<double>(cols));
    for(auto & r : a) for(auto & v : r) v = A(rng);
    std::vector<double> c(cols), cap(rows);
    for(std::size_t j = 0; j < cols; ++j) { double t = 0; for(auto & r : a) t += r[j]; c[j] = std::floor(t / rows + R(rng)); }
    for(std::size_t i = 0; i < rows; ++i) { double t = 0; for(double v : a[i]) t += v; cap[i] = std::floor(t / 2); }
    auto x = m.add_binary_variables(cols);
    const auto ids = std::views::iota(std::size_t{0}, cols);
    m.set_maximization();
    m.set_objective(xsum(ids, [&](std::size_t j) { return c[j] * x(j); }) + 100.0);
    for(std::size_t i = 0; i < rows; ++i) m.add_constraint(xsum(ids, [&, i](std::size_t j) { return a[i][j] * x(j); }) <= cap[i]);
    if(pad) m.add_variables(static_cast<std::size_t>(pad), {.obj_coef = 0, .lower_bound = 0, .upper_bound = 1});
    std::vector<int> col;
    for(std::size_t j = 0; j < cols; ++j) col.push_back(m.native_id(x(j)));
    return [a, cap, col](const std::vector<double> & p, const cp::add_fn & add) {
        for(std::size_t i = 0; i < a.size(); ++i) {
            std::vector<std::size_t> order(col.size());
            std::iota(order.begin(), order.end(), std::size_t{0});
            std::ranges::sort(order, [&](std::size_t l, std::size_t r) { return p[static_cast<std::size_t>(col[l])] > p[static_cast<std::size_t>(col[r])]; });
            double w = 0; raw_cut cut{{}, {}, 'L', 0};
            for(std::size_t j : order) {
                w += a[i][j]; cut.ind.push_back(col[j]); cut.val.push_back(1);
                if(w > cap[i]) { cut.rhs = static_cast<double>(cut.ind.size()) - 1; if(cut.violation(p) > 1e-4) add(cut); break; }
            }
        }
    };
}
static cp::separator atsp_late(M & m, int n, unsigned seed, int skip) {
    auto a = cp::build_atsp(m, n, false, seed);
    auto count = std::make_shared<long>(0);
    return [a, count, skip](const std::vector<double> & x, const cp::add_fn & add) { if((*count)++ < skip) return; for(const auto & c : a.separate(x, 1)) add(c); };
}
using xform_fn = std::function<void(const raw_cut &, const std::vector<double> &, const cp::add_fn &)>;
static xform_fn scale_by(double f, bool flip = false) {
    return [f, flip](const raw_cut & c, const std::vector<double> &, const cp::add_fn & add) {
        raw_cut d = c; for(auto & v : d.val) v *= f; d.rhs *= f;
        if(flip) { for(auto & v : d.val) v = -v; d.rhs = -d.rhs; d.sense = d.sense == 'L' ? 'G' : 'L'; }
        add(d);
    };
}
static void cmd_big(int seeds, const std::string & only) {
    std::map<std::string, xform_fn> X;
    X["none"] = [](const raw_cut & c, const std::vector<double> &, const cp::add_fn & add) { add(c); };
    for(double f : {1e4, 1e7, 1e9, 1e11, 1e12}) { X[std::format("x{}", f)] = scale_by(f); X[std::format("x{} flipped", f)] = scale_by(f, true); }
    for(double f : {1e-5, 1e-8}) X[std::format("x{}", f)] = scale_by(f);
    X["near-duplicates 1e-12 and the same again"] = [](const raw_cut & c, const std::vector<double> &, const cp::add_fn & add) {
        add(c);
        raw_cut d = c; double slack = 0; for(auto & v : d.val) { const double dv = 1e-12 * std::fabs(v); v += dv; slack += dv; }
        d.rhs += d.sense == 'L' ? slack : -slack; if(d.sense == 'G') { d.rhs = c.rhs; for(std::size_t k = 0; k < d.val.size(); ++k) d.val[k] = c.val[k] * (1 + 1e-12); }
        add(d); add(c);
    };
    X["one huge relaxing term on a zero column"] = [](const raw_cut & c, const std::vector<double> & x, const cp::add_fn & add) {
        // x_k >= 0, so -M x_k on the small side keeps it valid. Only when the column is not in the cut.
        std::set<int> in(c.ind.begin(), c.ind.end());
        int k = -1; for(int j = 0; j < static_cast<int>(x.size()); ++j) if(!in.count(j) && x[static_cast<std::size_t>(j)] < 1e-9) { k = j; break; }
        raw_cut d = c; if(k >= 0) { d.ind.push_back(k); d.val.push_back(c.sense == 'L' ? -1e8 : 1e8); }
        add(d);
    };
    X["a tiny term and an inflated rhs"] = [](const raw_cut & c, const std::vector<double> & x, const cp::add_fn & add) {
        std::set<int> in(c.ind.begin(), c.ind.end());
        int k = -1; for(int j = 0; j < static_cast<int>(x.size()); ++j) if(!in.count(j)) { k = j; break; }
        raw_cut d = c; if(k >= 0) { d.ind.push_back(k); d.val.push_back(c.sense == 'L' ? 1e-13 : -1e-13); d.rhs += c.sense == 'L' ? 1e-13 : -1e-13; }
        add(d);
    };
    struct fam { std::string name; std::function<cp::separator(M &, unsigned)> build; };
    const std::vector<fam> fams = {
        {"mkp covers padded", [](M & m, unsigned s) { return mkp_covers(m, s, 480); }},
        {"mkp covers", [](M & m, unsigned s) { return mkp_covers(m, s, 0); }},
        {"atsp18 DFJ after 25 calls", [](M & m, unsigned s) { return atsp_late(m, 18, s, 25); }},
    };
    std::map<std::pair<std::string, int>, double> bases;
    for(const auto & [xn, xf] : X) {
        if(!only.empty() && xn.find(only) == std::string::npos) continue;
        for(const auto & F : fams) {
            auto & t = T[std::format("BIG {} / {}", xn, F.name)];
            for(int s = 1; s <= seeds; ++s) {
                const auto U = static_cast<unsigned>(s);
                const auto key = std::make_pair(F.name, s);
                if(!bases.count(key)) { M b; F.build(b, U); b.set_time_limit(duration<double>(40)); cp::timed_solve(b); bases[key] = mippp::is_a<mippp::status::optimal>(b.get_status()) ? cf::value_of(b) : NAN; }
                const double base = bases[key];
                if(std::isnan(base)) continue;
                M m; auto sep = F.build(m, U); m.set_time_limit(duration<double>(40));
                m.attach([sep, xf](const std::vector<double> & x, const cp::add_fn & add) { sep(x, [&](const raw_cut & c) { xf(c, x, add); return 0; }); });
                cp::timed_solve(m);
                ++t.runs; t.calls += m.hook->c.calls.load(); t.added += m.hook->c.added.load(); t.below += m.hook->c.below_root.load(); t.caught += m.hook->c.caught.load(); t.nodes += static_cast<long>(m.get_node_count());
                const double v = cf::value_of(m);
                if(mippp::is_a<mippp::status::time_limit>(m.get_status())) { ++t.timeouts; continue; }
                if(mippp::is_a<mippp::status::optimal>(m.get_status()) && cf::same(v, base)) ++t.ok;
                else t.fails.push_back(std::format("seed {} value {} base {} status {} nodes {} {}", s, v, base, cp::status_name(m.get_status()), m.get_node_count(), m.hook->c.line()));
            }
            std::println(stderr, "{}: runs {} ok {} timeouts {} fails {} | calls {} added {} nodes {}", std::format("BIG {} / {}", xn, F.name), t.runs, t.ok, t.timeouts, t.fails.size(), t.calls, t.added, t.nodes);
            for(const auto & f : t.fails) std::println(stderr, "   FAIL {}", f);
        }
    }
}

int main(int argc, char ** argv) {
    vc::banner();
    if(argc > 1 && std::string(argv[1]) == "big") { cmd_big(argc > 2 ? std::atoi(argv[2]) : 3, argc > 3 ? argv[3] : ""); return 0; }
    const bool diag = argc > 1 && std::string(argv[1]) == "diag";
    const bool grid = argc > 1 && std::string(argv[1]) == "grid";
    const int seeds = diag ? 0 : grid ? std::atoi(argv[2]) : argc > 1 ? std::atoi(argv[1]) : 20;
    const std::string only = (!diag && !grid && argc > 2) ? argv[2] : "";
    const std::vector<variant> vars = {
        {"binary", 16, 6, 0, false, false, true, 0},
        {"binary padded", 16, 6, 480, false, false, true, 0},
        {"binary min offset", 16, 6, 0, false, false, false, 17.5},
        {"binary extras padded", 16, 6, 480, false, true, true, 3.0},
        {"box padded", 11, 6, 480, true, false, true, 0},
        {"market padded", 20, 3, 480, false, false, true, 0, true},
        {"market", 20, 3, 0, false, false, true, 0, true},
        {"market min offset padded", 20, 3, 480, false, false, false, 17.5, true},
    };
    if(diag) { cmd_diag(vars, std::atoi(argv[2]), static_cast<unsigned>(std::atoi(argv[3])), std::atof(argv[4])); return 0; }
    const auto plain = [](const emodel & e, const std::vector<double> & x, const cp::add_fn & add, std::mt19937 & r, const box_view &, long) {
        for(int k = 0; k < 2; ++k) if(auto c = violated_tight(e, x, r)) add(*c);
    };
    const auto scaled = [](double f, char flip) {
        return [f, flip](const emodel & e, const std::vector<double> & x, const cp::add_fn & add, std::mt19937 & r, const box_view &, long) {
            if(auto c = violated_tight(e, x, r)) {
                for(auto & v : c->val) v *= f; c->rhs *= f;
                if(flip == 'G') { for(auto & v : c->val) v = -v; c->rhs = -c->rhs; c->sense = 'G'; }
                add(*c);
            }
        };
    };
    const auto run = [&](const std::string & name, const gen_fn & g, const std::function<bool(const variant &)> & use = {}, const std::function<void(M &)> & cfg = {}) {
        if(!only.empty() && name.find(only) == std::string::npos) return;
        for(const auto & V : vars) if(!use || use(V)) attack(name, V, seeds, g, cfg);
    };
    if(grid) {
        const auto sc = [](double f) { return [f](const emodel & e, const std::vector<double> & x, const cp::add_fn & add, std::mt19937 & r, const box_view &, long) {
            if(auto c = violated_tight(e, x, r)) { for(auto & v : c->val) v *= f; c->rhs *= f; add(*c); } }; };
        for(double f : {1e3, 1e4, 1e5, 1e6, 1e7, 1e8, 1e9, 1e-2, 1e-3, 1e-4, 1e-5, 1e-6, 1e-7, 1e-8, 1e-9, 1e-10})
            run(std::format("G uniform scale {}", f), sc(f));
        // Coefficients spanning a range, tight by exact enumeration: normalizing cannot help these.
        for(double range : {1e2, 1e3, 1e4, 1e5, 1e6, 1e8})
            run(std::format("G range {}", range), [range](const emodel & e, const std::vector<double> & x, const cp::add_fn & add, std::mt19937 & r, const box_view &, long) {
                const auto N = static_cast<std::size_t>(e.n);
                std::uniform_int_distribution<int> K(1, 3), Z(0, 2), W(0, 1);
                for(int t = 0; t < 20; ++t) {
                    std::vector<double> d(N, 0);
                    for(std::size_t j = 0; j < N; ++j) { if(Z(r) == 0) continue; const double v = x[static_cast<std::size_t>(e.col[j])]; const double mid = 0.5 * (e.lo[j] + e.hi[j]); d[j] = (v >= mid ? 1.0 : -1.0) * K(r) * (W(r) ? range : 1.0); }
                    const double rhs = e.maxdot(d);
                    double act = 0; for(std::size_t j = 0; j < N; ++j) act += d[j] * x[static_cast<std::size_t>(e.col[j])];
                    if(act > rhs + 1e-4) { add(with_col(e, d, rhs)); return; }
                }
            });
        report();
        return 0;
    }
    run("A0 tight valid cuts", plain);
    for(double f : {1e4, 1e7, 1e9, 1e12}) run(std::format("A1 scaled by {}", f), scaled(f, 'L'));
    run("A1 scaled by 1e12 as >=", scaled(1e12, 'G'));
    run("A1 scaled by 1e15 as >=", scaled(1e15, 'G'));
    for(double f : {1e-5, 1e-8, 1e-11}) run(std::format("A2 scaled by {}", f), scaled(f, 'L'));
    run("A3 near-duplicates at 1e-12", [](const emodel & e, const std::vector<double> & x, const cp::add_fn & add, std::mt19937 & r, const box_view &, long) {
        if(auto c = violated_tight(e, x, r)) {
            add(*c);
            for(int k = 0; k < 3; ++k) {
                raw_cut d = *c; double slack = 0; std::uniform_real_distribution<double> U(-1e-12, 1e-12);
                for(auto & v : d.val) { const double dv = U(r) * std::fabs(v); v += dv; slack += std::fabs(dv) * 3.0; }
                d.rhs += slack; add(d);
            }
            add(*c);  // the very same again
        }
    });
    run("A4 cuts through removed columns", [](const emodel & e, const std::vector<double> & x, const cp::add_fn & add, std::mt19937 & r, const box_view &, long) {
        if(auto c = violated_tight(e, x, r)) {
            // complement of x_0..2, rowless cost-free column, rowless dual-fixed one, fixed column, slack substitution
            raw_cut out{{}, {}, 'L', c->rhs};
            for(std::size_t k = 0; k < c->ind.size(); ++k) {
                int comp = -1;
                for(std::size_t q = 0; q < e.comp.size(); ++q) if(e.col[q] == c->ind[k]) comp = e.comp[q];
                if(comp >= 0) { out.ind.push_back(comp); out.val.push_back(-c->val[k]); out.rhs -= c->val[k]; }
                else { out.ind.push_back(c->ind[k]); out.val.push_back(c->val[k]); }
            }
            out.ind.push_back(e.rowless0); out.val.push_back(-1.0);
            out.ind.push_back(e.rowless1); out.val.push_back(1.0); out.rhs += 1.0;
            out.ind.push_back(e.fixedc); out.val.push_back(1.0); out.rhs += 1.0;
            if(cp::raw_cut(out).violation(x) > 1e-6) add(out);
            // the slack form of the same cut: add row 0 as an equality
            raw_cut sl = *c; sl.rhs += e.b[0];
            for(int j = 0; j < e.n; ++j) if(e.a[0][static_cast<std::size_t>(j)] != 0) { sl.ind.push_back(e.col[static_cast<std::size_t>(j)]); sl.val.push_back(e.a[0][static_cast<std::size_t>(j)]); }
            sl.ind.push_back(e.slack); sl.val.push_back(1.0);
            add(sl);   // duplicates folded by sanitize
        }
    }, [](const variant & V) { return V.extras; });
    run("A5 one-term cuts", [](const emodel & e, const std::vector<double> & x, const cp::add_fn & add, std::mt19937 & r, const box_view &, long) {
        for(int j = 0; j < e.n; ++j) {
            const auto J = static_cast<std::size_t>(j);
            int mn = 100, mx = -100;
            for(std::size_t k = 0; k < e.count; ++k) { const int v = e.F[k * static_cast<std::size_t>(e.n) + J]; mn = std::min(mn, v); mx = std::max(mx, v); }
            const double v = x[static_cast<std::size_t>(e.col[J])];
            if(v > mx + 1e-6) add(raw_cut{{e.col[J]}, {1.0}, 'L', static_cast<double>(mx)});
            if(v < mn - 1e-6) add(raw_cut{{e.col[J]}, {1.0}, 'G', static_cast<double>(mn)});
        }
        (void)r;
    });
    run("A5b one-term cuts scaled 1e9", [](const emodel & e, const std::vector<double> & x, const cp::add_fn & add, std::mt19937 &, const box_view &, long) {
        for(int j : {e.force0, e.force1}) {
            const auto J = static_cast<std::size_t>(j);
            if(x[static_cast<std::size_t>(e.col[J])] < 1 - 1e-6) add(raw_cut{{e.col[J]}, {1e9}, 'G', 1e9});
        }
    });
    run("A6 equality cut", [plain](const emodel & e, const std::vector<double> & x, const cp::add_fn & add, std::mt19937 & r, const box_view & b, long c) {
        add(raw_cut{{e.col[static_cast<std::size_t>(e.eq0)], e.col[static_cast<std::size_t>(e.eq1)]}, {1.0, 1.0}, 'E', 1.0});
        plain(e, x, add, r, b, c);
    });
    run("A7 objective cut tight at the optimum", [](const emodel & e, const std::vector<double> & x, const cp::add_fn & add, std::mt19937 &, const box_view &, long) {
        raw_cut c = with_col(e, e.c, e.opt - e.offset - e.extra_obj);
        c.sense = e.maximize ? 'L' : 'G';
        add(c);
        (void)x;
    });
    run("A7b objective cut, exact and scaled by 1e6, at every call", [](const emodel & e, const std::vector<double> & x, const cp::add_fn & add, std::mt19937 &, const box_view &, long) {
        raw_cut c = with_col(e, e.c, 1e6 * (e.opt - e.offset - e.extra_obj));
        for(auto & v : c.val) v *= 1e6;
        c.sense = e.maximize ? 'L' : 'G';
        add(c);
        (void)x;
    });
    run("A8 random cuts tight at an optimum", [](const emodel & e, const std::vector<double> & x, const cp::add_fn & add, std::mt19937 & r, const box_view &, long) {
        for(int k = 0; k < 2; ++k) if(auto c = violated_tight(e, x, r, 30, true)) add(*c);
    });
    run("A9 no-good on a box with no integer point: the node about to die", [](const emodel & e, const std::vector<double> & x, const cp::add_fn & add, std::mt19937 &, const box_view & b, long) {
        if(b.lo.empty()) return;
        std::vector<int> fixed;
        for(int j = 0; j < e.n; ++j) if(b.lo[static_cast<std::size_t>(e.col[static_cast<std::size_t>(j)])] == b.up[static_cast<std::size_t>(e.col[static_cast<std::size_t>(j)])]) fixed.push_back(j);
        if(fixed.empty()) return;
        for(std::size_t k = 0; k < e.count; ++k) {
            bool in = true;
            for(int j : fixed) if(e.F[k * static_cast<std::size_t>(e.n) + static_cast<std::size_t>(j)] != b.lo[static_cast<std::size_t>(e.col[static_cast<std::size_t>(j)])]) { in = false; break; }
            if(in) return;
        }
        raw_cut c{{}, {}, 'G', 1.0};
        for(int j : fixed) {
            const bool one = b.lo[static_cast<std::size_t>(e.col[static_cast<std::size_t>(j)])] > 0.5;
            c.ind.push_back(e.col[static_cast<std::size_t>(j)]); c.val.push_back(one ? -1.0 : 1.0);
            if(one) c.rhs -= 1.0;
        }
        add(c); (void)x;
    }, [](const variant & V) { return !V.box; });
    run("A10 200 random valid cuts per call", [](const emodel & e, const std::vector<double> & x, const cp::add_fn & add, std::mt19937 & r, const box_view &, long call) {
        if(call > 60) return;
        const auto N = static_cast<std::size_t>(e.n);
        std::uniform_int_distribution<int> K(-3, 3);
        for(int k = 0; k < 200; ++k) { std::vector<double> d(N); for(auto & v : d) v = K(r); add(with_col(e, d, e.maxdot(d))); }
        (void)x;
    }, [](const variant & V) { return std::string_view(V.name) == "binary padded"; });
    report();
    return 0;
}
