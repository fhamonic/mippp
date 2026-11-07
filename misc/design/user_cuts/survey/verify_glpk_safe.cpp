// D, the safe direction. Every cut below is valid for every integer feasible point by
// the oracle, which enumerates them, so a lost optimum, a wrong status, an abort or a
// spin is the recipe's or the solver's. One forked child per case. Default settings of
// the wrapper (generators and heuristics on), recipe path, pool at ICUTGEN.
#include "verify_glpk_inst.hpp"
#include <array>
#include <unistd.h>
using namespace vg;
// The hook offers nothing once the LP objective has stopped moving at this node: GLPK has no
// round limit on user cuts, and a separator that keeps finding tiny cuts never lets the node branch.
struct stall_guard {
    int k = [] { const char * e = std::getenv("VG_STALL"); return e ? std::atoi(e) : 0; }();
    double eps = [] { const char * e = std::getenv("VG_STALL_EPS"); return e ? std::atof(e) : 1e-4; }();
    int node = -1, count = 0; double last = 0; long stops = 0;
    bool stop(int cur, double lp) {
        if(k == 0) return false;
        if(cur == node && std::fabs(lp - last) < eps * (1 + std::fabs(last))) ++count; else count = 0;
        node = cur; last = lp;
        if(count >= k) { ++stops; return true; }
        return false;
    }
};
struct run_opts { std::string fam; unsigned seed = 1; double p = 1; double lim = 8; bool normalize = false; double tol = 1e-9; bool gens = true; bool heur = true; };

struct bed {
    M m;
    oracle o;
    double objscale = 1;
    long fired = 0, offered = 0, passed = 0; int orig = -1, maxrows = 0;
    std::mt19937 rng;
    std::vector<int> ind; std::vector<double> val;
    bool normalize = false; double tol = 1e-9;
    long invalid_offers = 0, lp_below_opt = 0; double lp_min_seen = 1e300;
    void check_valid(const cp::raw_cut & c) {
        // the optimum must satisfy every cut, exactly as it will be passed on
        std::uint32_t a = o.arg; double act = 0;
        for(std::size_t k = 0; k < c.ind.size(); ++k) for(int q = 0; q < o.n; ++q) if(o.col[static_cast<std::size_t>(q)] == c.ind[k] && (a >> q & 1)) act += c.val[k];
        const bool bad = c.sense == 'L' ? act > c.rhs : act < c.rhs;
        if(bad) ++invalid_offers;
    }
    glp_tree * T = nullptr;
    glp_prob * P = nullptr;
    std::vector<double> x;   // 1-based native
    void offer(const std::vector<int> & cols, const std::vector<double> & w, char sense, double rhs) {
        ++offered;
        cp::raw_cut c{cols, w, sense, rhs};
        if(normalize) {
            double big = 0;
            for(double v : c.val) big = std::max(big, std::fabs(v));
            if(big > 0) { for(double & v : c.val) v /= big; c.rhs /= big; }
        }
        cut_rules r; r.tol = tol;
        const auto t = tidy(c, x, r);
        if(!t) return;
        cp::raw_cut out = *t;
        if(!o.col.empty()) check_valid(out);
        ind.assign(1, 0); val.assign(1, 0.0);
        for(std::size_t k = 0; k < out.ind.size(); ++k) { ind.push_back(out.ind[k]); val.push_back(out.val[k]); }
        natives::of(m).ios_add_row(T, nullptr, 0, 0, static_cast<int>(out.ind.size()), ind.data(), val.data(), out.sense == 'L' ? v1::GLP_UP : v1::GLP_LO, out.rhs);
        ++passed;
    }
    // offer w'x <= support(w) * k over the bits
    void tight(const std::vector<double> & w, double k = 1.0) {
        std::vector<int> cols; std::vector<double> sc;
        for(int q = 0; q < o.n; ++q) if(w[static_cast<std::size_t>(q)] != 0.0) { cols.push_back(o.col[static_cast<std::size_t>(q)]); sc.push_back(w[static_cast<std::size_t>(q)] * k); }
        offer(cols, sc, 'L', o.support(w) * k);
    }
    void tight_with(const std::vector<double> & w, double sup) {
        std::vector<int> cols; std::vector<double> sc;
        for(int q = 0; q < o.n; ++q) if(w[static_cast<std::size_t>(q)] != 0.0) { cols.push_back(o.col[static_cast<std::size_t>(q)]); sc.push_back(w[static_cast<std::size_t>(q)]); }
        offer(cols, sc, 'L', sup);
    }
    double xbit(int q) const { return x[static_cast<std::size_t>(o.col[static_cast<std::size_t>(q)])]; }
};

static void run_case(const run_opts & R) {
    bed B;
    M & m = B.m;
    B.rng.seed(R.seed * 7919u + 13u);
    B.normalize = R.normalize; B.tol = R.tol;
    const std::string & F = R.fam;
    const int N = 18;
    std::vector<int> extra;
    std::function<void()> gen;
    std::function<void()> finish_check;
    std::string note;
    auto & rng = B.rng;
    const auto dirs_from_x = [&](std::vector<double> & w) {
        std::uniform_real_distribution<double> U(0.5, 1.5);
        w.resize(static_cast<std::size_t>(B.o.n));
        for(int q = 0; q < B.o.n; ++q) w[static_cast<std::size_t>(q)] = std::round(B.o.c[static_cast<std::size_t>(q)] * (B.xbit(q) + 0.1) * U(rng));
    };
    double expected_opt = NAN;
    bool expect_infeasible = false;

    if(F == "obj_viol") { // objective x 1e6, unit cuts, violation p: under the LP's tolerance when p < 1e-7
        B.o = build_oracle(m, R.seed, N, 4);
        for(int q = 0; q < B.o.n; ++q) { B.o.c[static_cast<std::size_t>(q)] *= 1e6; m.native_api().set_obj_coef(m.native_model(), B.o.col[static_cast<std::size_t>(q)], B.o.c[static_cast<std::size_t>(q)]); }
        B.o.opt = -1e300; B.o.feas.clear(); B.o.enumerate();
        gen = [&] {
            const int len = std::min(B.o.n, 8);
            std::vector<int> order(static_cast<std::size_t>(B.o.n));
            for(int q = 0; q < B.o.n; ++q) order[static_cast<std::size_t>(q)] = q;
            std::ranges::sort(order, [&](int l, int r) { return B.xbit(l) > B.xbit(r); });
            std::vector<double> w(static_cast<std::size_t>(B.o.n), 0.0);
            double act = 0;
            for(int t = 0; t < len; ++t) { w[static_cast<std::size_t>(order[static_cast<std::size_t>(t)])] = 1.0; act += B.xbit(order[static_cast<std::size_t>(t)]); }
            if(act - R.p < B.o.support(w)) return;
            std::vector<int> cols; std::vector<double> val;
            for(int t = 0; t < len; ++t) { cols.push_back(B.o.col[static_cast<std::size_t>(order[static_cast<std::size_t>(t)])]); val.push_back(1.0); }
            B.offer(cols, val, 'L', act - R.p);
        };
    } else if(F == "obj_big") { // objective scaled, unit cuts tight at the optimum
        B.objscale = R.p;
        // build_oracle with a scaled objective, re-set below
        B.o = build_oracle(m, R.seed, N, 4);
        for(int q = 0; q < B.o.n; ++q) { B.o.c[static_cast<std::size_t>(q)] *= R.p; m.native_api().set_obj_coef(m.native_model(), B.o.col[static_cast<std::size_t>(q)], B.o.c[static_cast<std::size_t>(q)]); }
        B.o.opt = -1e300; B.o.feas.clear(); B.o.enumerate();
        gen = [&] { std::vector<double> w; dirs_from_x(w); B.tight(w); B.tight(B.o.c); };
    } else if(F == "one_term") {
        // a heavy column cannot be 1: x_j <= 0 is valid, the LP puts it fractional.
        oracle tmp;
        std::mt19937 g(R.seed);
        std::uniform_int_distribution<int> A(1, 100), C(20, 100);
        const auto Ns = static_cast<std::size_t>(N);
        tmp.n = N;
        tmp.a.assign(4, std::vector<double>(Ns));
        for(auto & r : tmp.a) for(auto & v : r) v = A(g);
        for(auto & r : tmp.a) { double s = 0; for(double v : r) s += v; tmp.b.push_back(std::floor(s / 2)); }
        tmp.c.resize(Ns);
        for(auto & v : tmp.c) v = C(g);
        for(std::size_t j = 0; j < 4; ++j) { tmp.a[0][j] = tmp.b[0] + 1; tmp.c[j] = 300; }
        auto x = m.add_binary_variables(Ns);
        const auto ids = std::views::iota(std::size_t{0}, Ns);
        m.set_maximization();
        m.set_objective(xsum(ids, [&](std::size_t j) { return tmp.c[j] * x(j); }));
        for(std::size_t i = 0; i < 4; ++i) m.add_constraint(xsum(ids, [&, i](std::size_t j) { return tmp.a[i][j] * x(j); }) <= tmp.b[i]);
        for(std::size_t j = 0; j < Ns; ++j) tmp.col.push_back(m.native_id(x(j)));
        tmp.enumerate();
        B.o = std::move(tmp);
        gen = [&] {
            for(int q = 0; q < 4; ++q) B.offer({B.o.col[static_cast<std::size_t>(q)]}, {1.0}, 'L', 0.0);
            // and the mirror image of a one-term cut with a negative coefficient
            for(int q = 0; q < 4; ++q) B.offer({B.o.col[static_cast<std::size_t>(q)]}, {-2.0}, 'G', 0.0);
            // and a one-term bound the model implies, on a light column
            for(int q = 4; q < 8; ++q) { const double s = B.o.support_in_box(std::vector<double>(static_cast<std::size_t>(B.o.n), 1.0), 1u << q, 0); if(s < -1e200) B.offer({B.o.col[static_cast<std::size_t>(q)]}, {1.0}, 'L', 0.0); }
        };
    } else {
        B.o = build_oracle(m, R.seed, N, 4);
        if(F == "obj_tight") gen = [&] { B.tight(B.o.c); };
        else if(F == "tight_x") gen = [&] { std::vector<double> w; for(int k = 0; k < 3; ++k) { dirs_from_x(w); B.tight(w); } };
        else if(F == "tight_sign") gen = [&] { std::vector<double> w(static_cast<std::size_t>(B.o.n)); for(int q = 0; q < B.o.n; ++q) w[static_cast<std::size_t>(q)] = B.xbit(q) > 0.5 ? B.o.c[static_cast<std::size_t>(q)] : (B.xbit(q) < 1e-6 ? -1.0 : 0.0); B.tight(w); };
        else if(F == "scale") gen = [&] { std::vector<double> w; for(int k = 0; k < 2; ++k) { dirs_from_x(w); B.tight(w, R.p); } B.tight(B.o.c, R.p); };
        else if(F == "range") gen = [&] { std::uniform_real_distribution<double> E(-R.p, R.p); std::vector<double> w; dirs_from_x(w); for(auto & v : w) v *= std::pow(10.0, E(rng)); B.tight(w); B.tight(B.o.c); };
        else if(F == "near_dup") {
            // one base direction per instance; each call offers perturbed copies, each with its
            // own support: near-parallel across rounds and within one round.
            auto base = std::make_shared<std::vector<double>>();
            gen = [&, base] {
                if(base->empty()) { *base = B.o.c; }
                std::uniform_real_distribution<double> U(-R.p, R.p);
                std::vector<double> w(*base);
                for(int k = 0; k < 4; ++k) { for(std::size_t q = 0; q < w.size(); ++q) w[q] = (*base)[q] * (1 + U(rng)); B.tight(w); }
            };
        } else if(F == "many") {
            // the same pool of directions, supports computed once: the cost here must be the solver's
            auto dirs = std::make_shared<std::vector<std::pair<std::vector<double>, double>>>();
            gen = [&, dirs] {
                if(dirs->empty()) { std::uniform_real_distribution<double> U(0, 1); std::vector<double> w(static_cast<std::size_t>(B.o.n)); for(int k = 0; k < static_cast<int>(R.p); ++k) { for(int q = 0; q < B.o.n; ++q) w[static_cast<std::size_t>(q)] = std::round(100 * U(rng)); dirs->emplace_back(w, B.o.support(w)); } }
                for(const auto & a : *dirs) B.tight_with(a.first, a.second);
            };
        } else if(F == "bigm_cut") {
            // w'x <= r + K (1 - x_b) on a column the LP has at 1, and w'x - K x_b <= r on one it has at
            // 0: valid for K large, and exactly as violated as w'x <= r: a conditional cut, as users
            // write them. Normalizing cannot help: 1 and K sit in the same row.
            gen = [&] {
                std::vector<double> w; dirs_from_x(w);
                for(int q = 0; q < B.o.n; ++q) w[static_cast<std::size_t>(q)] = std::round(w[static_cast<std::size_t>(q)]);
                const double r = B.o.support(w);
                int at1 = -1, at0 = -1;
                for(int q = 0; q < B.o.n; ++q) { if(B.xbit(q) > 1 - 1e-9) at1 = q; else if(B.xbit(q) < 1e-9) at0 = q; }
                std::vector<int> cols; std::vector<double> sc; double rhs = r;
                for(int q = 0; q < B.o.n; ++q) if(w[static_cast<std::size_t>(q)] != 0 && q != at1 && q != at0) { cols.push_back(B.o.col[static_cast<std::size_t>(q)]); sc.push_back(w[static_cast<std::size_t>(q)]); }
                // the terms on the bound columns stay valid by the support, K comes on top
                if(at1 >= 0) { cols.push_back(B.o.col[static_cast<std::size_t>(at1)]); sc.push_back(w[static_cast<std::size_t>(at1)] + R.p); rhs += R.p; }
                if(at0 >= 0) { cols.push_back(B.o.col[static_cast<std::size_t>(at0)]); sc.push_back(w[static_cast<std::size_t>(at0)] - R.p); }
                if(cols.size() > 1) B.offer(cols, sc, 'L', rhs);
            };
        }
        else if(F == "reoffer") {
            auto all = std::make_shared<std::vector<std::pair<std::vector<double>, double>>>();
            gen = [&, all] { std::vector<double> w; dirs_from_x(w); all->emplace_back(w, B.o.support(w)); for(const auto & a : *all) B.tight_with(a.first, a.second); };
        } else if(F == "nogood") {
            // The node's box as a direction: w = +1 on what branching fixed at 1, -1 on what it fixed
            // at 0. Valid everywhere. If the box holds no feasible point the rhs is below what the box
            // forces, so the node's LP goes infeasible: a cut at a node about to be pruned.
            gen = [&] {
                std::vector<double> w(static_cast<std::size_t>(B.o.n), 0.0);
                auto & api = m.native_api();
                int fixed = 0;
                for(int q = 0; q < B.o.n; ++q) {
                    const int j = B.o.col[static_cast<std::size_t>(q)];
                    if(api.get_col_lb(B.P, j) > 0.5) { w[static_cast<std::size_t>(q)] = 1; ++fixed; }
                    else if(api.get_col_ub(B.P, j) < 0.5) { w[static_cast<std::size_t>(q)] = -1; ++fixed; }
                }
                if(fixed) B.tight(w);
                B.tight(B.o.c);
            };
        } else if(F == "viol") {
            // A cut weaker than the support, violated by exactly p at the LP point: valid as long as
            // the point is above the support by more than p. The separator re-finds it while the
            // point stays put. Coefficient scale in the seed's bed: objective scale 1 here.
            gen = [&] {
                double act = 0; for(int q = 0; q < B.o.n; ++q) act += B.o.c[static_cast<std::size_t>(q)] * B.xbit(q);
                const double sup = B.o.opt;
                if(act - R.p < sup) return;
                std::vector<int> cols; std::vector<double> val;
                for(int q = 0; q < B.o.n; ++q) { cols.push_back(B.o.col[static_cast<std::size_t>(q)]); val.push_back(B.o.c[static_cast<std::size_t>(q)]); }
                B.offer(cols, val, 'L', act - R.p);
            };
        } else if(F == "tiny_viol") {
            // p scales the cut down: unit violation stays 0.05, the row's raw violation falls under
            // the LP's feasibility tolerance when p <= 1e-6. Re-found while the point stays put.
            gen = [&] {
                const int len = std::min(B.o.n, 8);
                std::vector<int> order(static_cast<std::size_t>(B.o.n));
                for(int q = 0; q < B.o.n; ++q) order[static_cast<std::size_t>(q)] = q;
                std::ranges::sort(order, [&](int l, int r) { return B.xbit(l) > B.xbit(r); });
                std::vector<double> w(static_cast<std::size_t>(B.o.n), 0.0);
                double act = 0;
                for(int t = 0; t < len; ++t) { w[static_cast<std::size_t>(order[static_cast<std::size_t>(t)])] = 1.0; act += B.xbit(order[static_cast<std::size_t>(t)]); }
                const double sup = B.o.support(w);
                const double rhs = std::max(sup, act - 0.05);
                if(act - rhs < 1e-3) return;
                std::vector<int> cols; std::vector<double> val;
                for(int t = 0; t < len; ++t) { cols.push_back(B.o.col[static_cast<std::size_t>(order[static_cast<std::size_t>(t)])]); val.push_back(R.p); }
                B.offer(cols, val, 'L', rhs * R.p);
            };
        } else { std::println(stderr, "unknown family {}", F); _exit(9); }
    }
    if(F == "objective_scale_viol") {}
    if(!R.gens) generators_off(m);
    if(!R.heur) heuristics_off(m);
    m.p.presolve_off = true;
    auto & c = attach(m, [](const std::vector<double> &, const cp::add_fn &) {});
    auto & api = m.native_api();
    // at the root path the LP bound of a maximization can only stay above the optimum: only the root
    // is checked, deeper nodes may legitimately exclude the optimum through branching
    auto & nat = natives::of(m);
    glp_tree * cur = nullptr;
    stall_guard guard;
    const auto n_below = [&](double lp) { return nat.node_level(cur, nat.curr_node(cur)) == 0 && lp < B.o.opt - 1e-6 * (1 + std::fabs(B.o.opt)); };
    c.also = [&](glp_tree * t, int why) {
        cur = t;
        glp_prob * P = api.ios_get_prob(t);
        if(why == R_ROWGEN) {
            if(B.orig < 0) B.orig = api.get_num_rows(P);
            B.maxrows = std::max(B.maxrows, api.get_num_rows(P) - B.orig);
            const double lp = api.get_obj_val(P);
            if(n_below(lp)) { ++B.lp_below_opt; B.lp_min_seen = std::min(B.lp_min_seen, lp); }
        }
        if(why != R_CUTGEN) return;
        B.T = t; B.P = P;
        ++B.fired;
        if(guard.stop(nat.curr_node(t), api.get_obj_val(P))) return;
        const int n = api.get_num_cols(P);
        B.x.assign(static_cast<std::size_t>(n) + 1, 0.0);
        for(int j = 1; j <= n; ++j) B.x[static_cast<std::size_t>(j)] = api.get_col_prim(P, j);
        gen();
    };
    m.set_time_limit(std::chrono::duration<double>(R.lim));
    const auto t0 = clk::now();
    m.solve();
    const double secs = std::chrono::duration<double>(clk::now() - t0).count();
    const double v = m.get_solution_value();
    const double opt = B.o.opt;
    const bool is_opt = cp::status_name(m.get_status()).starts_with("optimal");
    const bool good = is_opt && std::fabs(v - opt) <= 1e-4 * (1 + std::fabs(opt));
    (void)expected_opt; (void)expect_infeasible;
    std::println(stderr, "R {} seed={} p={}: {} value={} opt={} => {} | fired={} offered={} passed={} maxrows={} nodes={} it={} {:.2f}s invalid_offers={} root_lp_below_opt={} lp_min={}",
        F, R.seed, R.p, st(m), v, opt, good ? "KEPT" : is_opt ? "LOST" : "WRONG-STATUS", B.fired, B.offered, B.passed, B.maxrows, m.closed, m.iters, secs, B.invalid_offers, B.lp_below_opt, B.lp_min_seen);
}

// ---- shared by the families below: pool only, through tidy, split equalities.
static bool g_normalize = false;
static double g_tol = 1e-9;
struct small {
    glp_tree * T = nullptr; glp_prob * P = nullptr; std::vector<double> x;
    long fired = 0, offered = 0, passed = 0; int orig = -1, maxrows = 0;
    std::vector<int> ind; std::vector<double> val;
    M * m = nullptr;
    bool split = true;
    bool normalize = false;
    void offer(cp::raw_cut c) {
        ++offered;
        if(c.sense == 'E' && split) {
            for(const char s : {'L', 'G'}) { cp::raw_cut one = c; one.sense = s; --offered; offer(one); }
            return;
        }
        if(g_normalize) { double big = 0; for(double v : c.val) big = std::max(big, std::fabs(v)); if(big > 0) { for(double & v : c.val) v /= big; c.rhs /= big; } }
        cut_rules r; r.tol = g_tol;
        auto t = tidy(c, x, r);
        if(!t) return;
        ind.assign(1, 0); val.assign(1, 0.0);
        for(std::size_t k = 0; k < t->ind.size(); ++k) { ind.push_back(t->ind[k]); val.push_back(t->val[k]); }
        natives::of(*m).ios_add_row(T, nullptr, 0, 0, static_cast<int>(t->ind.size()), ind.data(), val.data(), t->sense == 'L' ? v1::GLP_UP : t->sense == 'G' ? v1::GLP_LO : v1::GLP_FX, t->rhs);
        ++passed;
    }
};
static void arm(M & m, small & S, std::function<void()> gen) {
    S.m = &m;
    m.p.presolve_off = true;
    auto & c = attach(m, [](const std::vector<double> &, const cp::add_fn &) {});
    auto & api = m.native_api();
    auto guard = std::make_shared<stall_guard>();
    auto & nat = natives::of(m);
    c.also = [&S, &api, gen, guard, &nat](glp_tree * t, int why) {
        glp_prob * P = api.ios_get_prob(t);
        if(why == R_ROWGEN) { if(S.orig < 0) S.orig = api.get_num_rows(P); S.maxrows = std::max(S.maxrows, api.get_num_rows(P) - S.orig); }
        if(why != R_CUTGEN) return;
        S.T = t; S.P = P; ++S.fired;
        if(guard->stop(nat.curr_node(t), api.get_obj_val(P))) return;
        const int n = api.get_num_cols(P);
        S.x.assign(static_cast<std::size_t>(n) + 1, 0.0);
        for(int j = 1; j <= n; ++j) S.x[static_cast<std::size_t>(j)] = api.get_col_prim(P, j);
        gen();
    };
}
static void report(const std::string & F, const run_opts & R, M & m, small & S, double opt, bool expect_infeasible, double secs) {
    const std::string name = st(m);
    bool good;
    const double v = m.get_solution_value();
    if(expect_infeasible) good = name.starts_with("infeasible");
    else good = name.starts_with("optimal") && std::fabs(v - opt) <= 1e-4 * (1 + std::fabs(opt));
    std::println(stderr, "R {} seed={} p={}: {} value={} opt={} => {} | fired={} offered={} passed={} maxrows={} nodes={} it={} {:.2f}s", F, R.seed, R.p, name, v, opt,
        good ? "KEPT" : name.starts_with("optimal") ? "LOST" : "WRONG-STATUS", S.fired, S.offered, S.passed, S.maxrows, m.closed, m.iters, secs);
}
static double timed(M & m, double lim) {
    m.set_time_limit(std::chrono::duration<double>(lim));
    const auto t0 = clk::now();
    m.solve();
    return std::chrono::duration<double>(clk::now() - t0).count();
}

// Columns presolve would remove come first, cuts are written on them with the rhs moved to match:
// fixed by a row, fixed by bounds, a binary whose singleton row implies 0, an empty binary, and
// complements x + xb == 1 that replace half of the terms. Plus a free row and an empty row.
static void removed(const run_opts & R) {
    M m;
    auto ff = m.add_binary_variables(5);
    for(std::size_t i = 0; i < 5; ++i) m.add_constraint(1.0 * ff(i) == 1.0);
    auto fb1 = m.add_binary_variables(1); m.set_variable_lower_bound(fb1(std::size_t{0}), 1);
    auto fb0 = m.add_binary_variables(1); m.set_variable_upper_bound(fb0(std::size_t{0}), 0);
    auto emp = m.add_binary_variables(1);
    auto sng = m.add_binary_variables(1);
    m.add_constraint(1.0 * sng(std::size_t{0}) <= 0.5);
    const int cf[] = {m.native_id(ff(std::size_t{0})), m.native_id(ff(std::size_t{4})), m.native_id(fb1(std::size_t{0}))};
    const int c0 = m.native_id(fb0(std::size_t{0})), ce = m.native_id(emp(std::size_t{0})), cs = m.native_id(sng(std::size_t{0}));
    oracle o = build_oracle(m, R.seed, 18, 4);
    // the objective rewards the singleton: the LP takes it to 0.5
    m.native_api().set_obj_coef(m.native_model(), cs, 500.0);
    auto xb = m.add_binary_variables(18);
    std::vector<int> cb;
    auto & api = m.native_api();
    for(std::size_t q = 0; q < 18; ++q) {
        cb.push_back(m.native_id(xb(q)));
        const int i = api.add_rows(m.native_model(), 1);
        std::vector<int> ind{0, o.col[q], cb[q]}; std::vector<double> val{0, 1, 1};
        api.set_mat_row(m.native_model(), i, 2, ind.data(), val.data());
        api.set_row_bnds(m.native_model(), i, v1::GLP_FX, 1, 1);
    }
    // a free row, an empty row: presolve drops them
    { const int i = api.add_rows(m.native_model(), 2); std::vector<int> ind{0, o.col[0], o.col[1]}; std::vector<double> val{0, 1, 1}; api.set_mat_row(m.native_model(), i, 2, ind.data(), val.data()); api.set_row_bnds(m.native_model(), i + 1, v1::GLP_UP, 5, 5); }
    small S;
    auto rng = std::make_shared<std::mt19937>(R.seed * 101);
    arm(m, S, [&, rng] {
        std::uniform_real_distribution<double> U(0.5, 1.5);
        std::vector<double> w(18);
        for(std::size_t q = 0; q < 18; ++q) w[q] = std::round(o.c[q] * (S.x[static_cast<std::size_t>(o.col[q])] + 0.1) * U(*rng));
        double rhs = o.support(w);
        cp::raw_cut cut; cut.sense = 'L';
        for(std::size_t q = 0; q < 18; ++q) {
            if(q % 2 == 0) { cut.ind.push_back(o.col[q]); cut.val.push_back(w[q]); }
            else { cut.ind.push_back(cb[q]); cut.val.push_back(-w[q]); rhs -= w[q]; }
        }
        // fixed at 1: moves the rhs; fixed at 0 and implied 0: nothing moves; the empty column
        // can only hurt the left-hand side with a minus sign
        for(int j : cf) { cut.ind.push_back(j); cut.val.push_back(3.0); rhs += 3.0; }
        cut.ind.push_back(c0); cut.val.push_back(7.0);
        cut.ind.push_back(cs); cut.val.push_back(11.0);
        cut.ind.push_back(ce); cut.val.push_back(-2.0);
        cut.rhs = rhs;
        S.offer(cut);
        // one-term cuts on the singleton, which the LP keeps at 0.5
        S.offer(cp::raw_cut{{cs}, {1.0}, 'L', 0.0});
    });
    const double secs = timed(m, R.lim);
    report("removed", R, m, S, o.opt + 0.0, false, secs);
}

// Loose big-M links: y_j <= M x_j, y_j <= u_j. y_j <= u_j x_j is valid and violated.
static void mixed(const run_opts & R) {
    M m;
    oracle o = build_oracle(m, R.seed, 18, 4);
    const auto Ns = static_cast<std::size_t>(o.n);
    std::vector<double> u(Ns), d(Ns);
    std::mt19937 g(R.seed + 5);
    std::uniform_real_distribution<double> U(1, 20), D(1, 30);
    for(std::size_t j = 0; j < Ns; ++j) { u[j] = std::round(U(g)); d[j] = std::round(D(g)); }
    auto y = m.add_variables(Ns);
    std::vector<int> cy;
    auto & api = m.native_api();
    const double big = R.p;
    for(std::size_t j = 0; j < Ns; ++j) {
        cy.push_back(m.native_id(y(j)));
        api.set_col_bnds(m.native_model(), cy[j], v1::GLP_DB, 0, u[j]);
        api.set_obj_coef(m.native_model(), cy[j], d[j]);
        const int i = api.add_rows(m.native_model(), 1);
        std::vector<int> ind{0, cy[j], o.col[j]}; std::vector<double> val{0, 1, -big};
        api.set_mat_row(m.native_model(), i, 2, ind.data(), val.data());
        api.set_row_bnds(m.native_model(), i, v1::GLP_UP, 0, 0);
    }
    std::vector<double> eff(Ns);
    for(std::size_t j = 0; j < Ns; ++j) eff[j] = o.c[j] + d[j] * u[j];
    const double opt = o.support(eff);
    small S;
    auto rng = std::make_shared<std::mt19937>(R.seed * 103);
    arm(m, S, [&, rng] {
        for(std::size_t j = 0; j < Ns; ++j) S.offer(cp::raw_cut{{cy[j], o.col[j]}, {1.0, -u[j]}, 'L', 0.0});
        // a tight global cut over both kinds of columns
        std::uniform_real_distribution<double> V(0.5, 1.5);
        std::vector<double> w(Ns), v(Ns), e(Ns);
        for(std::size_t j = 0; j < Ns; ++j) { w[j] = std::round(o.c[j] * V(*rng)); v[j] = std::round(d[j] * V(*rng)); e[j] = w[j] + v[j] * u[j]; }
        cp::raw_cut c; c.sense = 'L'; c.rhs = o.support(e);
        for(std::size_t j = 0; j < Ns; ++j) { c.ind.push_back(o.col[j]); c.val.push_back(w[j]); c.ind.push_back(cy[j]); c.val.push_back(v[j]); }
        S.offer(c);
    });
    const double secs = timed(m, R.lim);
    report("mixed", R, m, S, opt, false, secs);
}

// Hidden equalities: 2p + 2q <= 3 and >= 1 make p + q == 1 valid, the LP sits at .75 + .75.
static void hidden_eq(const run_opts & R, bool split) {
    M m;
    const std::size_t pairs = 8;
    std::mt19937 g(R.seed);
    std::uniform_int_distribution<int> A(1, 40), C(5, 60);
    auto x = m.add_binary_variables(2 * pairs);
    std::vector<int> col;
    for(std::size_t j = 0; j < 2 * pairs; ++j) col.push_back(m.native_id(x(j)));
    std::vector<double> c(2 * pairs);
    for(auto & v : c) v = C(g);
    m.set_maximization();
    const auto ids = std::views::iota(std::size_t{0}, 2 * pairs);
    m.set_objective(xsum(ids, [&](std::size_t j) { return c[j] * x(j); }));
    for(std::size_t k = 0; k < pairs; ++k) {
        m.add_constraint(2.0 * x(2 * k) + 2.0 * x(2 * k + 1) <= 3);
        m.add_constraint(2.0 * x(2 * k) + 2.0 * x(2 * k + 1) >= 1);
    }
    std::vector<double> a(2 * pairs);
    for(auto & v : a) v = A(g);
    double cap = 0; for(double v : a) cap += v; cap = std::floor(cap * 0.4);
    m.add_constraint(xsum(ids, [&](std::size_t j) { return a[j] * x(j); }) <= cap);
    // exact optimum: exactly one of each pair
    double opt = -1e300;
    for(std::uint32_t mask = 0; mask < (1u << pairs); ++mask) {
        double v = 0, w = 0;
        for(std::size_t k = 0; k < pairs; ++k) { const std::size_t j = 2 * k + (mask >> k & 1); v += c[j]; w += a[j]; }
        if(w <= cap) opt = std::max(opt, v);
    }
    small S;
    S.split = split;
    arm(m, S, [&] {
        for(std::size_t k = 0; k < pairs; ++k) S.offer(cp::raw_cut{{col[2 * k], col[2 * k + 1]}, {1.0, 1.0}, 'E', 1.0});
    });
    const double secs = timed(m, R.lim);
    report(split ? "hidden_eq" : "hidden_eq_nosplit", R, m, S, opt, false, secs);
}

// LP feasible, no integer point: any cut is vacuously valid. Status must stay infeasible.
static void infeasible_model(const run_opts & R, const int kind) {
    M m;
    const std::size_t n = 14;
    auto x = m.add_binary_variables(n);
    std::vector<int> col;
    for(std::size_t j = 0; j < n; ++j) col.push_back(m.native_id(x(j)));
    const auto ids = std::views::iota(std::size_t{0}, n);
    std::mt19937 g(R.seed);
    std::uniform_int_distribution<int> C(1, 30);
    std::vector<double> c(n);
    for(auto & v : c) v = C(g);
    m.set_maximization();
    m.set_objective(xsum(ids, [&](std::size_t j) { return c[j] * x(j); }));
    m.add_constraint(xsum(ids, [&](std::size_t j) { return 2.0 * x(j); }) == 7.0 + 2.0 * (R.seed % 3));
    small S;
    bool once = false;
    arm(m, S, [&] {
        if(kind == 0) return;
        if(kind == 1 && !once) { once = true; S.offer(cp::raw_cut{{col[0]}, {1.0}, 'L', -1.0}); }                  // empties the root
        if(kind == 2) { S.offer(cp::raw_cut{{col[0], col[1]}, {1.0, 1.0}, 'L', 0.0}); S.offer(cp::raw_cut{{col[0], col[1]}, {1.0, 1.0}, 'G', 2.0}); } // contradiction
        if(kind == 3) { if(S.fired > 2) { std::vector<int> a(col); std::vector<double> v(n, 1.0); S.offer(cp::raw_cut{a, v, 'L', -5.0}); } }          // empties a deep node
    });
    const double secs = timed(m, R.lim);
    report(std::format("infeasible_model_{}", kind), R, m, S, 0, true, secs);
}

// Minimization with >= rows and >= cuts.
static void min_cover(const run_opts & R) {
    M m;
    const std::size_t n = 18, rows = 5;
    std::mt19937 g(R.seed);
    std::uniform_int_distribution<int> A(1, 50), C(10, 90);
    std::vector<std::vector<double>> a(rows, std::vector<double>(n));
    std::vector<double> b(rows), c(n);
    for(auto & r : a) for(auto & v : r) v = A(g);
    for(std::size_t i = 0; i < rows; ++i) { double s = 0; for(double v : a[i]) s += v; b[i] = std::floor(s * 0.6); }
    for(auto & v : c) v = C(g);
    auto x = m.add_binary_variables(n);
    const auto ids = std::views::iota(std::size_t{0}, n);
    m.set_minimization();
    m.set_objective(xsum(ids, [&](std::size_t j) { return c[j] * x(j); }));
    for(std::size_t i = 0; i < rows; ++i) m.add_constraint(xsum(ids, [&, i](std::size_t j) { return a[i][j] * x(j); }) >= b[i]);
    std::vector<int> col;
    for(std::size_t j = 0; j < n; ++j) col.push_back(m.native_id(x(j)));
    std::vector<std::uint32_t> feas;
    double opt = 1e300;
    for(std::uint32_t mask = 0; mask < (1u << n); ++mask) {
        bool ok = true;
        for(std::size_t i = 0; i < rows && ok; ++i) { double s = 0; for(std::size_t j = 0; j < n; ++j) if(mask >> j & 1) s += a[i][j]; ok = s >= b[i]; }
        if(!ok) continue;
        feas.push_back(mask);
        double v = 0; for(std::size_t j = 0; j < n; ++j) if(mask >> j & 1) v += c[j];
        opt = std::min(opt, v);
    }
    const auto low = [&](const std::vector<double> & w) { double best = 1e300; for(auto mask : feas) { double v = 0; for(std::size_t j = 0; j < n; ++j) if(mask >> j & 1) v += w[j]; best = std::min(best, v); } return best; };
    small S;
    auto rng = std::make_shared<std::mt19937>(R.seed * 107);
    arm(m, S, [&, rng] {
        std::uniform_real_distribution<double> U(0.5, 1.5);
        std::vector<double> w(n);
        for(std::size_t j = 0; j < n; ++j) w[j] = std::round(c[j] * (1.1 - S.x[static_cast<std::size_t>(col[j])]) * U(*rng));
        S.offer(cp::raw_cut{col, w, 'G', low(w)});
        S.offer(cp::raw_cut{col, c, 'G', opt});
    });
    const double secs = timed(m, R.lim);
    report("min_cover", R, m, S, opt, false, secs);
}

// General integers 0..3.
static void general_int(const run_opts & R) {
    M m;
    const std::size_t n = 8, rows = 3;
    std::mt19937 g(R.seed);
    std::uniform_int_distribution<int> A(1, 30), C(5, 60);
    std::vector<std::vector<double>> a(rows, std::vector<double>(n));
    std::vector<double> b(rows), c(n);
    for(auto & r : a) for(auto & v : r) v = A(g);
    for(std::size_t i = 0; i < rows; ++i) { double s = 0; for(double v : a[i]) s += v; b[i] = std::floor(s * 1.2); }
    for(std::size_t j = 0; j < n; ++j) c[j] = C(g);
    auto x = m.add_integer_variables(n, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 3});
    const auto ids = std::views::iota(std::size_t{0}, n);
    m.set_maximization();
    m.set_objective(xsum(ids, [&](std::size_t j) { return c[j] * x(j); }));
    for(std::size_t i = 0; i < rows; ++i) m.add_constraint(xsum(ids, [&, i](std::size_t j) { return a[i][j] * x(j); }) <= b[i]);
    std::vector<int> col;
    for(std::size_t j = 0; j < n; ++j) col.push_back(m.native_id(x(j)));
    std::vector<std::array<std::uint8_t, 8>> feas;
    double opt = -1e300;
    for(std::uint32_t code = 0; code < (1u << (2 * n)); ++code) {
        std::array<std::uint8_t, 8> v{};
        for(std::size_t j = 0; j < n; ++j) v[j] = (code >> (2 * j)) & 3;
        bool ok = true;
        for(std::size_t i = 0; i < rows && ok; ++i) { double s = 0; for(std::size_t j = 0; j < n; ++j) s += a[i][j] * v[j]; ok = s <= b[i]; }
        if(!ok) continue;
        feas.push_back(v);
        double o = 0; for(std::size_t j = 0; j < n; ++j) o += c[j] * v[j];
        opt = std::max(opt, o);
    }
    const auto sup = [&](const std::vector<double> & w) { double best = -1e300; for(const auto & p : feas) { double s = 0; for(std::size_t j = 0; j < n; ++j) s += w[j] * p[j]; best = std::max(best, s); } return best; };
    small S;
    auto rng = std::make_shared<std::mt19937>(R.seed * 109);
    arm(m, S, [&, rng] {
        std::uniform_real_distribution<double> U(0.5, 1.5);
        std::vector<double> w(n);
        for(std::size_t j = 0; j < n; ++j) w[j] = std::round(c[j] * U(*rng));
        S.offer(cp::raw_cut{col, w, 'L', sup(w)});
        S.offer(cp::raw_cut{col, c, 'L', opt});
        // one-term: the largest value a column ever takes
        for(std::size_t j = 0; j < n; ++j) { std::vector<double> e(n, 0.0); e[j] = 1; S.offer(cp::raw_cut{{col[j]}, {1.0}, 'L', sup(e)}); }
    });
    const double secs = timed(m, R.lim);
    report("general_int", R, m, S, opt, false, secs);
}

// CONTROL for the scale failures: the very rows the scale family offers, as model rows from the
// start, with the solver's own scaling applied to them. Presolve on and off.
static void static_scale(const run_opts & R, const bool presolve_off) {
    M m;
    oracle o = build_oracle(m, R.seed, 18, 4);
    std::mt19937 rng(R.seed * 11);
    std::uniform_real_distribution<double> U(0.2, 1.8);
    auto & api = m.native_api();
    for(int k = 0; k < 20; ++k) {
        std::vector<double> w(static_cast<std::size_t>(o.n));
        for(std::size_t q = 0; q < w.size(); ++q) w[q] = std::round(o.c[q] * U(rng));
        const double r = o.support(w);
        std::vector<int> ind{0}; std::vector<double> val{0};
        for(std::size_t q = 0; q < w.size(); ++q) { ind.push_back(o.col[q]); val.push_back(w[q] * R.p); }
        const int i = api.add_rows(m.native_model(), 1);
        api.set_mat_row(m.native_model(), i, o.n, ind.data(), val.data());
        api.set_row_bnds(m.native_model(), i, v1::GLP_UP, r * R.p, r * R.p);
    }
    m.p.presolve_off = presolve_off;
    if(presolve_off) attach(m, [](const std::vector<double> &, const cp::add_fn &) {});
    const double secs = timed(m, R.lim);
    small S;
    report(presolve_off ? "static_scale_off" : "static_scale_on", R, m, S, o.opt, false, secs);
}

// A model whose columns need very different scale factors: every entry of column j is multiplied
// by 10^e_j, the cuts are plain unit covers in the user's units. p is the range of e_j.
static void colscale(const run_opts & R) {
    M m;
    oracle o; o.n = 18;
    std::mt19937 g(R.seed);
    std::uniform_int_distribution<int> A(1, 100), C(10, 100);
    std::uniform_real_distribution<double> E(-R.p, R.p);
    const std::size_t Ns = 18;
    std::vector<double> sc(Ns);
    for(auto & v : sc) v = std::pow(10.0, std::round(E(g)));
    o.a.assign(4, std::vector<double>(Ns));
    for(auto & r : o.a) for(std::size_t j = 0; j < Ns; ++j) r[j] = A(g) * sc[j];
    for(auto & r : o.a) { std::vector<double> sorted(r); std::ranges::sort(sorted); double s = 0; for(std::size_t j = 0; j < Ns / 2; ++j) s += sorted[j]; o.b.push_back(s); }
    o.c.resize(Ns);
    for(auto & v : o.c) v = C(g);
    auto x = m.add_binary_variables(Ns);
    const auto ids = std::views::iota(std::size_t{0}, Ns);
    m.set_maximization();
    m.set_objective(xsum(ids, [&](std::size_t j) { return o.c[j] * x(j); }));
    for(std::size_t i = 0; i < 4; ++i) m.add_constraint(xsum(ids, [&, i](std::size_t j) { return o.a[i][j] * x(j); }) <= o.b[i]);
    for(std::size_t j = 0; j < Ns; ++j) o.col.push_back(m.native_id(x(j)));
    o.enumerate();
    small S;
    arm(m, S, [&] {
        // unit direction on what the LP has high: tight by the oracle
        std::vector<double> w(Ns, 0.0);
        for(std::size_t j = 0; j < Ns; ++j) if(S.x[static_cast<std::size_t>(o.col[j])] > 0.3) w[j] = 1.0;
        const double r = o.support(w);
        std::vector<int> cols; std::vector<double> val;
        for(std::size_t j = 0; j < Ns; ++j) if(w[j] != 0) { cols.push_back(o.col[j]); val.push_back(1.0); }
        S.offer(cp::raw_cut{cols, val, 'L', r});
        S.offer(cp::raw_cut{[&] { std::vector<int> a; for(std::size_t j = 0; j < Ns; ++j) a.push_back(o.col[j]); return a; }(), o.c, 'L', o.opt});
    });
    const double secs = timed(m, R.lim);
    report("colscale", R, m, S, o.opt, false, secs);
}

// y_j in [0, u_j], y_j <= 10 u_j x_j in the model, u_j ~ p: y_j - u_j x_j <= 0 is the classic valid
// cut, coefficients 1 and p in one row; the objective is balanced so that both columns matter.
static void mixed_u(const run_opts & R) {
    M m;
    oracle o = build_oracle(m, R.seed, 18, 4);
    const auto Ns = static_cast<std::size_t>(o.n);
    std::vector<double> u(Ns), d(Ns);
    std::mt19937 g(R.seed + 5);
    std::uniform_real_distribution<double> U(0.5, 2), D(1, 30);
    for(std::size_t j = 0; j < Ns; ++j) { u[j] = std::round(U(g) * R.p); d[j] = D(g) / R.p; }
    auto y = m.add_variables(Ns);
    std::vector<int> cy;
    auto & api = m.native_api();
    for(std::size_t j = 0; j < Ns; ++j) {
        cy.push_back(m.native_id(y(j)));
        api.set_col_bnds(m.native_model(), cy[j], v1::GLP_DB, 0, u[j]);
        api.set_obj_coef(m.native_model(), cy[j], d[j]);
        const int i = api.add_rows(m.native_model(), 1);
        std::vector<int> ind{0, cy[j], o.col[j]}; std::vector<double> val{0, 1, -10 * u[j]};
        api.set_mat_row(m.native_model(), i, 2, ind.data(), val.data());
        api.set_row_bnds(m.native_model(), i, v1::GLP_UP, 0, 0);
    }
    std::vector<double> eff(Ns);
    for(std::size_t j = 0; j < Ns; ++j) eff[j] = o.c[j] + d[j] * u[j];
    const double opt = o.support(eff);
    small S;
    arm(m, S, [&] {
        for(std::size_t j = 0; j < Ns; ++j) S.offer(cp::raw_cut{{cy[j], o.col[j]}, {1.0, -u[j]}, 'L', 0.0});
    });
    const double secs = timed(m, R.lim);
    report("mixed_u", R, m, S, opt, false, secs);
}

// Parity gadgets: x_j = 1 forces 2 x_k + 2 x_l = 1, which holds for the LP and for no integer point.
// So x_j <= 0 is a valid one-term cut that neither presolve nor the root preprocessor derives, and
// at a node where branching fixed x_j to 1 the node's LP is feasible and its box holds no integer
// point: the cut empties the node about to be pruned. `late` offers it only there.
static void parity(const run_opts & R, const bool late) {
    M m;
    oracle o; o.n = 18;
    std::mt19937 g(R.seed);
    std::uniform_int_distribution<int> A(1, 100), C(10, 60);
    const std::size_t Ns = 18, gadgets = 3;
    std::vector<std::vector<double>> a;
    for(int r = 0; r < 3; ++r) { std::vector<double> row(Ns); double s = 0; for(auto & v : row) { v = A(g); s += v; } a.push_back(row); o.b.push_back(std::floor(s * 0.45)); }
    for(std::size_t k = 0; k < gadgets; ++k) {
        // 2 x_u + 2 x_v == x_j + x_n, x_j + x_n <= 1. One row at a time rounds nothing: the
        // implied bound of x_u is 1. Integer points need x_j = x_n = 0, the LP takes x_j = 1, x_u = .5.
        std::vector<double> up(Ns, 0.0), dn(Ns, 0.0), one(Ns, 0.0);
        const std::size_t j = 4 * k, n = 4 * k + 1, u = 4 * k + 2, v = 4 * k + 3;
        up[u] = up[v] = 2; up[j] = up[n] = -1; dn[u] = dn[v] = -2; dn[j] = dn[n] = 1; one[j] = one[n] = 1;
        a.push_back(up); o.b.push_back(0); a.push_back(dn); o.b.push_back(0); a.push_back(one); o.b.push_back(1);
    }
    o.a = a;
    o.c.resize(Ns);
    for(auto & v : o.c) v = C(g);
    for(std::size_t k = 0; k < gadgets; ++k) { o.c[4 * k] = 200; o.c[4 * k + 1] = 150; }  // the LP wants the cursed columns
    auto x = m.add_binary_variables(Ns);
    const auto ids = std::views::iota(std::size_t{0}, Ns);
    m.set_maximization();
    m.set_objective(xsum(ids, [&](std::size_t j) { return o.c[j] * x(j); }));
    for(std::size_t i = 0; i < a.size(); ++i) m.add_constraint(xsum(ids, [&, i](std::size_t j) { return a[i][j] * x(j); }) <= o.b[i]);
    for(std::size_t j = 0; j < Ns; ++j) o.col.push_back(m.native_id(x(j)));
    o.enumerate();
    if(late) { generators_off(m); heuristics_off(m); m.model_params.br_tech = v1::GLP_BR_FFV; }  // branch on the first columns: the cursed ones
    small S;
    auto & api = m.native_api();
    long at_fixed = 0;
    arm(m, S, [&] {
        for(std::size_t k = 0; k < gadgets; ++k) {
            for(const std::size_t q : {4 * k, 4 * k + 1}) {
                const int j = o.col[q];
                const bool fixed_up = api.get_col_lb(S.P, j) > 0.5;
                if(late && !fixed_up) continue;
                if(fixed_up) ++at_fixed;
                S.offer(cp::raw_cut{{j}, {1.0}, 'L', 0.0});
            }
        }
    });
    const double secs = timed(m, R.lim);
    report(late ? "parity_late" : "parity", R, m, S, o.opt, false, secs);
    std::println(stderr, "   offered at a node with the column fixed at 1: {}", at_fixed);
}
// c'x <= opt below the root only: the cut sends the node's bound to the incumbent, so that it is pruned.
static void prune(const run_opts & R) {
    M m;
    oracle o = build_oracle(m, R.seed, 18, 4);
    small S;
    auto & nat = natives::of(m);
    std::vector<int> cols; for(int q = 0; q < o.n; ++q) cols.push_back(o.col[static_cast<std::size_t>(q)]);
    arm(m, S, [&] {
        if(nat.node_level(S.T, nat.curr_node(S.T)) == 0) return;
        S.offer(cp::raw_cut{cols, o.c, 'L', o.opt});
    });
    const double secs = timed(m, R.lim);
    report("prune", R, m, S, o.opt, false, secs);
}
int main(int argc, char ** argv) {
    run_opts R;
    R.fam = argv[1];
    if(argc > 2) R.seed = static_cast<unsigned>(std::atoi(argv[2]));
    if(argc > 3) R.p = std::atof(argv[3]);
    if(argc > 4) R.lim = std::atof(argv[4]);
    if(argc > 5) { R.normalize = std::atoi(argv[5]); g_normalize = R.normalize; }
    if(argc > 6) { R.tol = std::atof(argv[6]); g_tol = R.tol; }
    if(R.fam == "removed") removed(R);
    else if(R.fam == "mixed") mixed(R);
    else if(R.fam == "hidden_eq") hidden_eq(R, true);
    else if(R.fam == "hidden_eq_nosplit") hidden_eq(R, false);
    else if(R.fam == "infeasible_model") infeasible_model(R, static_cast<int>(R.p));
    else if(R.fam == "min_cover") min_cover(R);
    else if(R.fam == "general_int") general_int(R);
    else if(R.fam == "parity") parity(R, false);
    else if(R.fam == "parity_late") parity(R, true);
    else if(R.fam == "prune") prune(R);
    else if(R.fam == "static_scale_on") static_scale(R, false);
    else if(R.fam == "static_scale_off") static_scale(R, true);
    else if(R.fam == "colscale") colscale(R);
    else if(R.fam == "mixed_u") mixed_u(R);
    else run_case(R);
    return 0;
}
