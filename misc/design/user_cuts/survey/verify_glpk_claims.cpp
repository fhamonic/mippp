// Own experiments on the survey's claims, one sub-command per claim.
#include "verify_glpk_inst.hpp"
#include <csetjmp>
#include <sys/wait.h>
using namespace vg;
static const cp::separator noop = [](const std::vector<double> &, const cp::add_fn &) {};
static double lp_value(const oracle & o, const std::vector<double> & w, glp_prob * P, const v1::glpk_api & api) {
    double s = 0;
    for(int k = 0; k < o.n; ++k) s += w[static_cast<std::size_t>(k)] * api.get_col_prim(P, o.col[static_cast<std::size_t>(k)]);
    return s;
}

// ---------------------------------------------------------------------- L1 ---
// At level >= 1 add c'x <= best integer value inside the node's own box: valid in the
// subtree, globally invalid when the box misses the optimum. Named rows are then looked
// for at every node, on and off the path of the node that added them. At the root,
// c'x <= opt, globally valid and active, must then reach every node.
static void L1(unsigned seed, bool gens, bool heur, bool root_cut) {
    M m;
    auto o = build_oracle(m, seed, 20, 5);
    if(!gens) generators_off(m);
    if(!heur) heuristics_off(m);
    m.p.presolve_off = true;
    long added = 0, invalid_added = 0, on_path = 0, off_path = 0, own = 0, desc = 0, events = 0, root_seen = 0, root_on = 0, serial = 0;
    int orig = -1, maxlevel = 0;
    auto & c = attach(m, noop);
    auto & api = m.native_api();
    auto & n = natives::of(m);
    c.rules.drop_satisfied = false;
    c.also = [&](glp_tree * t, int why) {
        if(why != R_CUTGEN) return;
        glp_prob * P = api.ios_get_prob(t);
        if(orig < 0) orig = api.get_num_rows(P);
        ++events;
        const int node = n.curr_node(t), level = n.node_level(t, node);
        maxlevel = std::max(maxlevel, level);
        std::vector<int> chain(static_cast<std::size_t>(level) + 1);
        for(int q = node, l = level; q; q = n.up_node(t, q), --l) chain[static_cast<std::size_t>(l)] = q;
        bool saw_root_cut = false;
        for(int i = orig + 1; i <= api.get_num_rows(P); ++i) {
            const char * name = n.get_row_name(P, i);
            if(!name) continue;
            if(name[0] == 'R') { saw_root_cut = true; continue; }
            int by = 0, at = 0;
            if(std::sscanf(name, "U%d_%d", &by, &at) != 2) continue;
            const bool mine = at <= level && chain[static_cast<std::size_t>(at)] == by;
            (mine ? on_path : off_path)++;
            if(mine) (by == node ? own : desc)++;
        }
        if(level > 0) { ++root_seen; root_on += saw_root_cut; }
        std::uint32_t one = 0, zero = 0;
        for(int k = 0; k < o.n; ++k) {
            const int j = o.col[static_cast<std::size_t>(k)];
            if(api.get_col_lb(P, j) > 0.5) one |= 1u << k;
            if(api.get_col_ub(P, j) < 0.5) zero |= 1u << k;
        }
        const double lp = lp_value(o, o.c, P, api);
        std::vector<int> ind{0}; std::vector<double> val{0};
        for(int k = 0; k < o.n; ++k) { ind.push_back(o.col[static_cast<std::size_t>(k)]); val.push_back(o.c[static_cast<std::size_t>(k)]); }
        if(level == 0) {
            if(root_cut && lp > o.opt + 1e-3) n.ios_add_row(t, "R", 0, 0, o.n, ind.data(), val.data(), v1::GLP_UP, o.opt);
            return;
        }
        const double local = o.support_in_box(o.c, one, zero);
        if(local < -1e200 || lp <= local + 1e-3) return;
        const std::string name = std::format("U{}_{}_{}", node, level, ++serial);
        n.ios_add_row(t, name.c_str(), 0, 0, o.n, ind.data(), val.data(), v1::GLP_UP, local);
        ++added;
        if(local < o.opt - 1e-6) ++invalid_added;
    };
    m.set_time_limit(std::chrono::duration<double>(30));
    m.solve();
    std::println(stderr, "L1 seed={} gens={} heur={} root_cut={} {} value={} opt={} {} | nodes={} max level={} events={} | subtree-valid cuts offered={} globally invalid among them={} | named rows seen on the adder's path={} (own node {}, descendants {}) off the path={} | root cut seen at {}/{} deeper events",
        seed, gens, heur, root_cut, st(m), m.get_solution_value(), o.opt, std::fabs(m.get_solution_value() - o.opt) < 1e-6 ? "KEPT" : "LOST", m.closed, maxlevel, events, added, invalid_added, on_path, own, desc, off_path, root_on, root_seen);
}

// ---------------------------------------------------------------------- L2 ---
// Presolve on, valid cover cuts over the original columns, separated the way a user
// would: read x_j from the tree problem and trust j. k columns fixed by a row come
// first, so that presolve shifts every index down by k. Three rows sit on the first
// 12 arcs and two on the last 8: restricted to the first three, cuts stay in range.
static void L2(unsigned seed, int k, bool heur, bool gens, bool off, bool restricted, bool rng_c) {
    M m;
    auto f = m.add_binary_variables(static_cast<std::size_t>(k));
    for(std::size_t i = 0; i < static_cast<std::size_t>(k); ++i) m.add_constraint(1.0 * f(i) == 1.0);
    oracle o; o.n = 20;
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> A(1, 100), N(0, 30), C(1, 100);
    const std::size_t Ns = 20;
    o.a.assign(5, std::vector<double>(Ns, 0.0));
    for(std::size_t i = 0; i < 5; ++i) for(std::size_t j = 0; j < Ns; ++j) if((i < 3) == (j < 12)) o.a[i][j] = A(rng);
    o.c.resize(Ns);
    for(std::size_t j = 0; j < Ns; ++j) { double s = 0; for(auto & r : o.a) s += r[j]; o.c[j] = rng_c ? C(rng) : std::floor(s / (j < 12 ? 3 : 2)) + N(rng); }
    for(auto & r : o.a) { double s = 0; for(double v : r) s += v; o.b.push_back(std::floor(s / 2)); }
    auto x = m.add_binary_variables(Ns);
    const auto ids = std::views::iota(std::size_t{0}, Ns);
    m.set_maximization();
    m.set_objective(xsum(ids, [&](std::size_t j) { return o.c[j] * x(j); }));
    for(std::size_t i = 0; i < 5; ++i) m.add_constraint(xsum(ids, [&, i](std::size_t j) { return o.a[i][j] * x(j); }) <= o.b[i]);
    for(std::size_t j = 0; j < Ns; ++j) o.col.push_back(m.native_id(x(j)));
    o.enumerate();
    if(!heur) heuristics_off(m);
    if(!gens) generators_off(m);
    m.p.presolve_off = off;
    long cuts = 0, fired = 0; int tree_cols = -1;
    auto & c = attach(m, noop);
    auto & api = m.native_api(); auto & n = natives::of(m);
    c.also = [&](glp_tree * t, int why) {
        if(why != R_CUTGEN) return;
        glp_prob * P = api.ios_get_prob(t);
        tree_cols = api.get_num_cols(P);
        ++fired;
        const auto read = [&](std::size_t q) { const int j = o.col[q]; return j <= tree_cols ? api.get_col_prim(P, j) : 0.0; };
        for(std::size_t i = 0; i < (restricted ? 3u : 5u); ++i) {
            std::vector<std::size_t> order;
            for(std::size_t q = 0; q < Ns; ++q) if(o.a[i][q] > 0) order.push_back(q);
            std::ranges::sort(order, [&](std::size_t l, std::size_t r) { return read(l) > read(r); });
            double w = 0, act = 0; std::vector<int> ind{0}; std::vector<double> val{0};
            for(std::size_t q : order) {
                w += o.a[i][q]; act += read(q); ind.push_back(o.col[q]); val.push_back(1.0);
                if(w > o.b[i]) {
                    const int s = static_cast<int>(ind.size()) - 1;
                    if(act > s - 1 + 1e-4) { n.ios_add_row(t, nullptr, 0, 0, s, ind.data(), val.data(), v1::GLP_UP, s - 1.0); ++cuts; }
                    break;
                }
            }
        }
    };
    m.set_time_limit(std::chrono::duration<double>(20));
    m.solve();
    const double v = m.get_solution_value();
    std::println(stderr, "L2 seed={} k={} presolve={} heur={} gens={} restricted={}: {} value={} opt={} => {} | tree cols={} (model {}) fired={} cuts={} it={}",
        seed, k, off ? "off" : "on", heur, gens, restricted, st(m), v, o.opt, std::fabs(v - o.opt) < 1e-6 ? "KEPT" : "LOST", tree_cols, o.n + k, fired, cuts, m.iters);
}

// The survey's gadget: the hull cut of gadget 3, on the columns the presolved problem
// shows, at default settings and with heuristics off.
static void L2g(bool flip, bool heur, bool off, int which, bool gens) {
    M m;
    auto g = cp::build_gadgets(m, 10, flip);
    if(!heur) heuristics_off(m);
    if(!gens) generators_off(m);
    m.p.presolve_off = off;
    bool once = false;
    long fired = 0; int tree_cols = -1;
    auto & c = attach(m, noop);
    auto & api = m.native_api(); auto & n = natives::of(m);
    c.also = [&](glp_tree * t, int why) {
        if(why != R_CUTGEN) return;
        ++fired;
        tree_cols = api.get_num_cols(api.ios_get_prob(t));
        if(once) return;
        once = true;
        const cp::raw_cut cut = which ? cp::raw_cut{{g.za[12], g.dummy + 3}, {1.0, 6.0}, 'L', 9.0} : g.hull_cut(3);
        std::vector<int> ind{0}; std::vector<double> val{0};
        for(std::size_t k = 0; k < cut.ind.size(); ++k) if(cut.val[k] != 0) { ind.push_back(cut.ind[k]); val.push_back(cut.val[k]); }
        n.ios_add_row(t, nullptr, 0, 0, static_cast<int>(ind.size()) - 1, ind.data(), val.data(), v1::GLP_UP, cut.rhs);
    };
    m.set_time_limit(std::chrono::duration<double>(20));
    m.solve();
    std::println(stderr, "L2g cut={} gens={} flip={} heur={} presolve={}: {} value={} opt={} => {} | tree cols={} fired={}", which ? "survey" : "hull3", gens, flip, heur, off ? "off" : "on", st(m), m.get_solution_value(), g.opt, std::fabs(m.get_solution_value() - g.opt) < 1e-6 ? "KEPT" : "LOST", tree_cols, fired);
}

// ---------------------------------------------------------------------- L3 ---
static void L3(int gmi, int mir, int cov, int clq, int minlevel) {
    M m;
    auto o = build_oracle(m, 3, 20, 5);
    m.model_params.gmi_cuts = gmi; m.model_params.mir_cuts = mir; m.model_params.cov_cuts = cov; m.model_params.clq_cuts = clq;
    m.p.presolve_off = true;
    long added = 0, fired = 0; int level_added = -1;
    auto & c = attach(m, noop);
    auto & api = m.native_api(); auto & n = natives::of(m);
    std::mt19937 rng(9);
    std::vector<double> w(static_cast<std::size_t>(o.n));
    c.also = [&](glp_tree * t, int why) {
        if(why != R_CUTGEN) return;
        ++fired;
        glp_prob * P = api.ios_get_prob(t);
        const int level = n.node_level(t, n.curr_node(t));
        if(level < minlevel) return;
        std::vector<int> ind{0}; std::vector<double> val{0};
        for(int q = 0; q < o.n; ++q) { ind.push_back(o.col[static_cast<std::size_t>(q)]); val.push_back(o.c[static_cast<std::size_t>(q)]); }
        const double lp = lp_value(o, o.c, P, api);
        if(lp <= o.opt + 1e-6) return;
        const int i = api.add_rows(P, 1);
        api.set_mat_row(P, i, o.n, ind.data(), val.data());
        api.set_row_bnds(P, i, v1::GLP_UP, o.opt, o.opt);
        ++added; level_added = level;
        std::println(stderr, "  added a direct row at level {} after {} fires", level, fired);
    };
    m.set_time_limit(std::chrono::duration<double>(20));
    m.solve();
    std::println(stderr, "L3 gmi={} mir={} cov={} clq={} minlevel={} survived: {} value={} opt={} fired={} added={} last level {}", gmi, mir, cov, clq, minlevel, st(m), m.get_solution_value(), o.opt, fired, added, level_added);
}

// ---------------------------------------------------------------------- L4 ---
// An odd cycle: every x at 0.5. Test cuts need no validity here: only what lands counts.
struct cyc { std::vector<int> col; };
static cyc build_cycle(M & m, int n, double objscale = 1.0) {
    auto x = m.add_binary_variables(static_cast<std::size_t>(n));
    m.set_maximization();
    const auto ids = std::views::iota(std::size_t{0}, static_cast<std::size_t>(n));
    m.set_objective(xsum(ids, [&](std::size_t j) { return objscale * (1.0 + 0.001 * static_cast<double>(j)) * x(j); }));
    for(std::size_t j : ids) m.add_constraint(x(j) + x((j + 1) % static_cast<std::size_t>(n)) <= 1);
    cyc c;
    for(std::size_t j : ids) c.col.push_back(m.native_id(x(j)));
    return c;
}
// Counts the named rows landed, at the next IROWGEN of the same node (fires on every LP,
// fractional or not). Disjoint odd cycles: branching on one leaves all the others at 0.5.
static int landed_rows(M & m, glp_prob * P, int orig) {
    int count = 0;
    auto & n = natives::of(m);
    for(int i = orig + 1; i <= m.native_api().get_num_rows(P); ++i) { const char * nm = n.get_row_name(P, i); if(nm && nm[0] == 'V') ++count; }
    return count;
}
static void L4_count(int offer, int at_level) {
    M m;
    const int len = 9, cycles = 60;
    auto x = m.add_binary_variables(static_cast<std::size_t>(len * cycles));
    m.set_maximization();
    const auto ids = std::views::iota(std::size_t{0}, static_cast<std::size_t>(len * cycles));
    m.set_objective(xsum(ids, [&](std::size_t j) { return (1.0 + 0.001 * static_cast<double>(j % 37)) * x(j); }));
    for(int c0 = 0; c0 < cycles; ++c0) for(int k = 0; k < len; ++k) {
        const auto a = static_cast<std::size_t>(c0 * len + k), b = static_cast<std::size_t>(c0 * len + (k + 1) % len);
        m.add_constraint(x(a) + x(b) <= 1);
    }
    std::vector<int> col;
    for(std::size_t j : ids) col.push_back(m.native_id(x(j)));
    generators_off(m); heuristics_off(m);
    m.p.presolve_off = true;
    auto & c = attach(m, noop);
    auto & api = m.native_api(); auto & n = natives::of(m);
    bool done = false; int offered_node = -1, orig = -1, landed = -1, pool_after = -1, count = 0;
    c.also = [&](glp_tree * t, int why) {
        glp_prob * P = api.ios_get_prob(t);
        if(why == R_ROWGEN) {
            if(orig < 0) orig = api.get_num_rows(P);
            if(done && landed < 0 && n.curr_node(t) == offered_node) landed = landed_rows(m, P, orig);
            return;
        }
        if(why != R_CUTGEN || done) return;
        const int node = n.curr_node(t), level = n.node_level(t, node);
        if(level != at_level) return;
        // non-adjacent pairs of each cycle: disjoint terms, so no parallel filter
        for(int c0 = 0; c0 < cycles && count < offer; ++c0) for(int k = 0; k + 2 < len && count < offer; k += 2) {
            const auto a = static_cast<std::size_t>(c0 * len + k), b = static_cast<std::size_t>(c0 * len + k + 2);
            std::vector<int> ind{0, col[a], col[b]}; std::vector<double> val{0, 1.0, 1.0};
            if(api.get_col_prim(P, ind[1]) + api.get_col_prim(P, ind[2]) <= 0.9) continue;
            n.ios_add_row(t, "V", 0, 0, 2, ind.data(), val.data(), v1::GLP_UP, 0.9);
            ++count;
        }
        pool_after = n.pool_size(t);
        done = true; offered_node = node;
    };
    m.set_time_limit(std::chrono::duration<double>(1));
    m.solve();
    std::println(stderr, "L4 count offered={} (pool size {}) at level {}: landed={} {} GLPK {}", count, pool_after, at_level, landed, st(m), vers(m));
}
// One 3-term cut with violation v (per unit coefficient) at the root.
static void L4_thresh(double v, double coef, double objscale) {
    M m;
    auto cy = build_cycle(m, 9, objscale);
    generators_off(m); heuristics_off(m);
    m.p.presolve_off = true;
    auto & c = attach(m, noop);
    auto & api = m.native_api(); auto & n = natives::of(m);
    bool done = false; int orig = -1, landed = -1; double viol = 0;
    c.also = [&](glp_tree * t, int why) {
        glp_prob * P = api.ios_get_prob(t);
        if(why == R_ROWGEN) {
            if(orig < 0) orig = api.get_num_rows(P);
            if(done && landed < 0) landed = landed_rows(m, P, orig);
            return;
        }
        if(why != R_CUTGEN || done) return;
        done = true;
        std::vector<int> ind{0, cy.col[0], cy.col[2], cy.col[4]}; std::vector<double> val{0, coef, coef, coef};
        const double act = coef * (api.get_col_prim(P, ind[1]) + api.get_col_prim(P, ind[2]) + api.get_col_prim(P, ind[3]));
        viol = v * coef;
        n.ios_add_row(t, "V", 0, 0, 3, ind.data(), val.data(), v1::GLP_UP, act - viol);
    };
    m.set_time_limit(std::chrono::duration<double>(5));
    m.solve();
    std::println(stderr, "L4 thresh unit violation={} coef={} objscale={}: raw violation {} normalized {:.5f} landed={} {}", v, coef, objscale, viol, viol / (coef * std::sqrt(3.0)), landed, st(m));
}
// Two cuts at a given cosine, both violated by 0.2 before any is applied.
static void L4_parallel(double eps) {
    M m;
    auto cy = build_cycle(m, 9);
    generators_off(m); heuristics_off(m);
    m.p.presolve_off = true;
    auto & c = attach(m, noop);
    auto & api = m.native_api(); auto & n = natives::of(m);
    bool done = false; int orig = -1, landed = -1;
    c.also = [&](glp_tree * t, int why) {
        glp_prob * P = api.ios_get_prob(t);
        if(why == R_ROWGEN) {
            if(orig < 0) orig = api.get_num_rows(P);
            if(done && landed < 0) landed = landed_rows(m, P, orig);
            return;
        }
        if(why != R_CUTGEN || done) return;
        done = true;
        std::vector<int> ind{0, cy.col[0], cy.col[2], cy.col[4]};
        std::vector<double> v1_{0, 1, 1, 1}, v2_{0, 1, 1, eps};
        const double a1 = api.get_col_prim(P, ind[1]) + api.get_col_prim(P, ind[2]) + api.get_col_prim(P, ind[3]);
        const double a2 = api.get_col_prim(P, ind[1]) + api.get_col_prim(P, ind[2]) + eps * api.get_col_prim(P, ind[3]);
        n.ios_add_row(t, "V", 0, 0, 3, ind.data(), v1_.data(), v1::GLP_UP, a1 - 0.2);
        n.ios_add_row(t, "V", 0, 0, 3, ind.data(), v2_.data(), v1::GLP_UP, a2 - 0.2);
    };
    m.set_time_limit(std::chrono::duration<double>(5));
    m.solve();
    const double cos = (2 + eps) / (std::sqrt(3.0) * std::sqrt(2 + eps * eps));
    std::println(stderr, "L4 parallel third coefficient={} cosine={:.4f}: landed {} of 2 {}", eps, cos, landed, st(m));
}

// ---------------------------------------------------------------------- L6 ---
static void L6(const std::string & kase) {
    M m;
    auto o = build_oracle(m, 3, 16, 3);
    generators_off(m);
    m.p.presolve_off = true;
    auto & c = attach(m, noop);
    auto & api = m.native_api(); auto & n = natives::of(m);
    bool done = false;
    c.also = [&](glp_tree * t, int why) {
        if(why != R_CUTGEN || done) return;
        done = true;
        glp_prob * P = api.ios_get_prob(t);
        std::vector<int> ind{0, o.col[0], o.col[1], o.col[2]}; std::vector<double> val{0, 1, 1, 1};
        std::println(stderr, "  hook reached with case {}", kase);
        if(kase == "dup_pool") { ind[2] = ind[1]; n.ios_add_row(t, nullptr, 0, 0, 3, ind.data(), val.data(), v1::GLP_UP, 1.0); }
        else if(kase == "oor_pool") { ind[3] = api.get_num_cols(P) + 1; n.ios_add_row(t, nullptr, 0, 0, 3, ind.data(), val.data(), v1::GLP_UP, 1.0); }
        else if(kase == "zero_ind_pool") { ind[3] = 0; n.ios_add_row(t, nullptr, 0, 0, 3, ind.data(), val.data(), v1::GLP_UP, 1.0); }
        else if(kase == "eq_pool") { n.ios_add_row(t, nullptr, 0, 0, 3, ind.data(), val.data(), v1::GLP_FX, 1.0); }
        else if(kase == "dbl_pool") { n.ios_add_row(t, nullptr, 0, 0, 3, ind.data(), val.data(), v1::GLP_DB, 1.0); }
        else if(kase == "free_pool") { n.ios_add_row(t, nullptr, 0, 0, 3, ind.data(), val.data(), v1::GLP_FR, 1.0); }
        else if(kase == "zero_coef_pool") { val[2] = 0.0; n.ios_add_row(t, nullptr, 0, 0, 3, ind.data(), val.data(), v1::GLP_UP, 1.0); }
        else if(kase == "zero_coef_only_pool") { val[1] = val[2] = val[3] = 0.0; n.ios_add_row(t, nullptr, 0, 0, 3, ind.data(), val.data(), v1::GLP_UP, 1.0); }
        else if(kase == "empty_pool") { n.ios_add_row(t, nullptr, 0, 0, 0, ind.data(), val.data(), v1::GLP_UP, 1.0); }
        else if(kase == "neg_len_pool") { n.ios_add_row(t, nullptr, 0, 0, -1, ind.data(), val.data(), v1::GLP_UP, 1.0); }
        else if(kase == "dup_direct") { ind[2] = ind[1]; const int i = api.add_rows(P, 1); api.set_mat_row(P, i, 3, ind.data(), val.data()); api.set_row_bnds(P, i, v1::GLP_UP, 1.0, 1.0); }
        else if(kase == "oor_direct") { ind[3] = api.get_num_cols(P) + 1; const int i = api.add_rows(P, 1); api.set_mat_row(P, i, 3, ind.data(), val.data()); api.set_row_bnds(P, i, v1::GLP_UP, 1.0, 1.0); }
        else if(kase == "nan_rhs_pool") { n.ios_add_row(t, nullptr, 0, 0, 3, ind.data(), val.data(), v1::GLP_UP, std::nan("")); }
        else if(kase == "inf_coef_pool") { val[1] = std::numeric_limits<double>::infinity(); n.ios_add_row(t, nullptr, 0, 0, 3, ind.data(), val.data(), v1::GLP_UP, 1.0); }
        else { std::println(stderr, "unknown case"); std::exit(3); }
        std::println(stderr, "  call returned, no error code in sight");
    };
    m.set_time_limit(std::chrono::duration<double>(5));
    m.solve();
    std::println(stderr, "L6 {} survived: {} value={} opt={} GLPK {}", kase, st(m), m.get_solution_value(), o.opt, vers(m));
}
// error hook + longjmp: what is left afterwards.
static std::jmp_buf g_env;
static void on_error(void *) { std::longjmp(g_env, 1); }
static void L6_hook() {
    using hook_t = void (*)(void (*)(void *), void *);
    M m;
    auto o = build_oracle(m, 3, 16, 3);
    generators_off(m);
    m.p.presolve_off = true;
    auto & c = attach(m, noop);
    auto & api = m.native_api(); auto & n = natives::of(m);
    bool done = false;
    c.also = [&](glp_tree * t, int why) {
        if(why != R_CUTGEN || done) return;
        done = true;
        std::vector<int> ind{0, o.col[0], o.col[0]}; std::vector<double> val{0, 1, 1};
        n.ios_add_row(t, nullptr, 0, 0, 2, ind.data(), val.data(), v1::GLP_UP, 1.0);
    };
    const auto set_hook = cp::symf<hook_t>(m.native_api().library_path(), "glp_error_hook");
    const auto free_env = cp::symf<int (*)()>(m.native_api().library_path(), "glp_free_env");
    set_hook(on_error, nullptr);
    if(setjmp(g_env) == 0) {
        m.solve();
        std::println(stderr, "L6_hook no error?");
    } else {
        std::println(stderr, "L6_hook longjmp reached; the tree is still attached to P, glp_intopt would refuse it");
        std::println(stderr, "L6_hook: glp_free_env returns {}, and it frees every problem object of the environment, so the wrapper's model pointer now dangles", free_env());
        std::_Exit(0);
    }
}

// ---------------------------------------------------------------------- L7 ---
static void L7(bool fix, const std::string & how) {
    M m;
    auto o = build_oracle(m, 3, 16, 3);
    m.p.presolve_off = true;
    m.p.clear_stale = fix;
    attach(m, noop);
    m.solve();
    std::println(stderr, "L7 first solve: {} value={} mip_status={} (5 = GLP_OPT) x0={}", st(m), m.get_solution_value(), m.last_mip_status, m.native_api().mip_col_val(m.native_model(), o.col[0]));
    auto & api = m.native_api();
    glp_prob * P = m.native_model();
    if(how == "infeasible") {
        // a contradiction on an LP level: sum of columns <= -1
        std::vector<int> ind{0}; std::vector<double> val{0};
        for(int q = 0; q < o.n; ++q) { ind.push_back(o.col[static_cast<std::size_t>(q)]); val.push_back(1); }
        const int i = api.add_rows(P, 1);
        api.set_mat_row(P, i, o.n, ind.data(), val.data());
        api.set_row_bnds(P, i, v1::GLP_UP, -1, -1);
    } else if(how == "unbounded") {
        const int j = api.add_cols(P, 1);
        api.set_col_bnds(P, j, v1::GLP_LO, 0, 0);
        api.set_obj_coef(P, j, 1.0);
    } else if(how == "time") {
        // a root LP that cannot end within 2 ms: thousands of new rows and columns
        std::mt19937 rng(1);
        std::uniform_real_distribution<double> U(0.1, 10);
        const int J = 3000, I = 3000;
        const int j0 = api.add_cols(P, J);
        for(int j = 0; j < J; ++j) { api.set_col_bnds(P, j0 + j, v1::GLP_DB, 0, 1); api.set_obj_coef(P, j0 + j, U(rng)); }
        const int i0 = api.add_rows(P, I);
        std::uniform_int_distribution<int> pick(0, J - 1);
        for(int i = 0; i < I; ++i) {
            std::vector<int> ind{0}; std::vector<double> val{0};
            std::set<int> seen;
            for(int k = 0; k < 40; ++k) seen.insert(pick(rng));
            for(int j : seen) { ind.push_back(j0 + j); val.push_back(U(rng)); }
            api.set_mat_row(P, i0 + i, static_cast<int>(seen.size()), ind.data(), val.data());
            api.set_row_bnds(P, i0 + i, v1::GLP_UP, 20, 20);
        }
        m.set_time_limit(std::chrono::duration<double>(0.0));
    }
    m.solve();
    std::println(stderr, "L7 {} after: native ret={:#x} stopped_at_root={} glp_mip_status={} status={} get_solution_value()={} root_seconds={:.4f}", how, m.native_ret, m.stopped_at_root, m.last_mip_status, st(m), m.get_solution_value(), m.root_seconds);
}

// ---------------------------------------------------------------------- L8 ---
static void L8(unsigned seed, bool off) {
    M m;
    auto o = build_oracle(m, seed, 22, 3);
    generators_off(m); heuristics_off(m);
    m.model_params.bt_tech = v1::GLP_BT_BLB;
    m.p.presolve_off = off;
    auto & c = attach(m, noop);
    auto & api = m.native_api(); auto & n = natives::of(m);
    struct seen_t { int level, parent; };
    std::map<int, seen_t> first;
    long fires = 0, conflicts = 0;
    std::string sample;
    c.also = [&](glp_tree * t, int why) {
        if(why != R_SELECT && why != R_CUTGEN) return;
        const int node = n.curr_node(t);
        if(node == 0) return;
        ++fires;
        const seen_t now{n.node_level(t, node), n.up_node(t, node)};
        auto [it, fresh] = first.try_emplace(node, now);
        if(!fresh && (it->second.level != now.level || it->second.parent != now.parent)) {
            if(!conflicts) sample = std::format("node {} first level {} parent {}, later level {} parent {}", node, it->second.level, it->second.parent, now.level, now.parent);
            ++conflicts;
        }
    };
    m.set_time_limit(std::chrono::duration<double>(20));
    m.solve();
    std::println(stderr, "L8 seed={} presolve_off={}: {} nodes={} fires={} distinct ids={} id seen with another level or parent {} times; {}", seed, off, st(m), m.closed, fires, first.size(), conflicts, sample);
}

// ---------------------------------------------------------------------- L10 --
// The same model, same presolve setting: no cb_func, a hook that does nothing, a hook that
// reads everything at every reason. Fingerprint: iterations, closed nodes, value.
static void L10(const std::string & fam, unsigned seed) {
    const auto run = [&](int kind, bool off) {
        M m;
        cp::atsp a;
        if(fam == "atsp") a = cp::build_atsp(m, 14, false, seed);
        else if(fam == "mkp") cp::build_mkp(m, 5, 30, seed);
        else if(fam == "gadgets") { auto g = cp::build_gadgets(m, 8, false); }
        else { // weak facility location
            std::mt19937 rng(seed);
            std::uniform_real_distribution<double> U(1, 100);
            const std::size_t F = 8, C = 20;
            auto y = m.add_binary_variables(F);
            auto x = m.add_variables(F * C, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 1});
            std::vector<double> fc(F), cc(F * C);
            for(auto & v : fc) v = 100 + 3 * U(rng);
            for(auto & v : cc) v = U(rng);
            const auto Fs = std::views::iota(std::size_t{0}, F); const auto Cs = std::views::iota(std::size_t{0}, C);
            m.set_minimization();
            m.set_objective(xsum(Fs, [&](std::size_t i) { return fc[i] * y(i); }) + xsum(Fs, [&](std::size_t i) { return xsum(Cs, [&, i](std::size_t j) { return cc[i * C + j] * x(i * C + j); }); }));
            for(std::size_t j : Cs) m.add_constraint(xsum(Fs, [&, j](std::size_t i) { return 1.0 * x(i * C + j); }) == 1);
            for(std::size_t i : Fs) m.add_constraint(xsum(Cs, [&, i](std::size_t j) { return 1.0 * x(i * C + j); }) <= static_cast<double>(C) * y(i));
        }
        m.p.presolve_off = off;
        double sink = 0;
        if(kind >= 1) {
            auto & c = attach(m, noop);
            auto & api = m.native_api(); auto & n = natives::of(m);
            if(kind == 2) c.also = [&sink, &api, &n](glp_tree * t, int why) {
                glp_prob * P = api.ios_get_prob(t);
                int ac, nc, tc;
                api.ios_tree_size(t, &ac, &nc, &tc);
                sink += ac + nc + tc + api.ios_best_node(t) + (why == R_CUTGEN ? n.pool_size(t) : 0);
                const int node = n.curr_node(t);
                if(node) sink += n.node_level(t, node) + n.up_node(t, node);
                if(why == R_ROWGEN || why == R_CUTGEN || why == R_BRANCH || why == R_HEUR) {
                    for(int j = 1; j <= api.get_num_cols(P); ++j) sink += api.get_col_prim(P, j) + api.get_col_lb(P, j) + api.get_col_ub(P, j);
                    sink += api.get_obj_val(P) + api.get_num_rows(P);
                }
            };
        }
        m.set_time_limit(std::chrono::duration<double>(30));
        m.solve();
        return std::format("it={} closed={} {} value={}", m.iters, m.closed, st(m), m.get_solution_value());
    };
    for(const bool off : {false, true}) {
        const auto a0 = run(0, off), a1 = run(1, off), a2 = run(2, off);
        std::println(stderr, "L10 {} seed={} presolve_off={}: none [{}] noop [{}] reader [{}] => {}", fam, seed, off, a0, a1, a2, a0 == a1 && a1 == a2 ? "IDENTICAL" : "DIFFER");
    }
}

// ---------------------------------------------------------------------- main -
int main(int argc, char ** argv) {
    const std::string w = argc > 1 ? argv[1] : "";
    const auto I = [&](int k, int d) { return argc > k ? std::atoi(argv[k]) : d; };
    if(w == "L1") L1(static_cast<unsigned>(I(2, 1)), I(3, 1), I(4, 1), I(5, 0));
    else if(w == "L2") L2(static_cast<unsigned>(I(2, 1)), I(3, 3), I(4, 1), I(5, 1), I(6, 0), I(7, 1), I(8, 0));
    else if(w == "L2g") L2g(I(2, 0), I(3, 1), I(4, 0), I(5, 0), I(6, 1));
    else if(w == "L3") L3(I(2, 1), I(3, 1), I(4, 1), I(5, 1), I(6, 0));
    else if(w == "L4c") L4_count(I(2, 200), I(3, 0));
    else if(w == "L4t") L4_thresh(std::atof(argv[2]), std::atof(argv[3]), std::atof(argv[4]));
    else if(w == "L4p") L4_parallel(std::atof(argv[2]));
    else if(w == "L6") L6(argv[2]);
    else if(w == "L6h") L6_hook();
    else if(w == "L7") L7(I(2, 0), argv[3]);
    else if(w == "L8") L8(static_cast<unsigned>(I(2, 1)), I(3, 1));
    else if(w == "L10") L10(argv[2], static_cast<unsigned>(I(3, 1)));
    else { std::println(stderr, "unknown"); return 2; }
    return 0;
}
