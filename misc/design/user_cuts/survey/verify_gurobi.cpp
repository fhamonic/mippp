// Skeptic's targeted experiments on Gurobi, selected by argv[1].
#include "verify_gurobi_common.hpp"
using std::chrono::duration;

static const char * arg(int argc, char ** argv, int i, const char * d = "") { return argc > i ? argv[i] : d; }

// ======================================================================= G1
// A cut valid only in one subtree of a column, added at a tree node, before the
// optimum is an incumbent. c'x <= z_alt, where z_alt is the best value with x_j flipped.
struct g1_out { int lost = 0, kept = 0, nopower = 0; long sib_checks = 0, sib_viol = 0, same_checks = 0, same_viol = 0; };
static void g1_own(unsigned seeds, bool scope_settings, g1_out & o) {
    for(unsigned seed = 1; seed <= seeds; ++seed) {
        const cgm G = make_cg(seed, 45, 15);
        M base; seti(base, "Threads", 1); seti(base, "OutputFlag", 0); setd(base, "MIPGap", 0.0); setd(base, "TimeLimit", 30); G.r.load(base);
        const sol s0 = solve_native(base);
        if(s0.status != 2) continue;
        bool done = false;
        std::vector<std::pair<double, size_t>> cand;
        for(size_t j = 0; j < G.x.size(); ++j) {
            M alt; seti(alt, "Threads", 1); seti(alt, "OutputFlag", 0); setd(alt, "MIPGap", 0.0); setd(alt, "TimeLimit", 30); G.r.load(alt);
            auto [env, model] = alt.native_model();
            const double fix = std::round(s0.x[j]) > 0.5 ? 0.0 : 1.0;
            alt.native_api().setdblattrelement(model, "LB", G.x[j], fix);
            alt.native_api().setdblattrelement(model, "UB", G.x[j], fix);
            alt.native_api().updatemodel(model);
            const sol a = solve_native(alt, false);
            if(a.status == 2 && a.value < s0.value - 0.5) cand.emplace_back(a.value, j);
        }
        std::ranges::sort(cand, std::greater<>{});
        for(auto [zalt, j] : cand | std::views::take(6)) {
            M m; seti(m, "OutputFlag", 0); setd(m, "MIPGap", 0.0); setd(m, "TimeLimit", 30);
            if(scope_settings) scope(m); else seti(m, "Threads", 1);
            recipe(m); G.r.load(m);
            const int col = G.x[j];
            const double worse = std::round(s0.x[j]) > 0.5 ? 0.0 : 1.0;
            raw_cut cut{G.x, G.c, 'L', zalt};
            hctx c; bool added = false; long sc = 0, sv = 0, oc = 0, ov = 0; double added_at = -1, best_at = NAN;
            c.on_node = [&](hctx & h, const std::vector<double> & x) {
                const double v = x[static_cast<size_t>(col)];
                const bool in_worse = std::fabs(v - worse) < 1e-9, in_other = std::fabs(v - (1 - worse)) < 1e-9;
                if(!added) {
                    const bool no_opt_yet = !(h.best >= s0.value - 0.5);
                    if(h.nod > 0 && in_worse && no_opt_yet && cut.violation(x) > 1e-4) { added = h.add(cut) == 0 && h.cuts > 0; added_at = h.nod; best_at = h.best; }
                    return;
                }
                const bool vi = cut.violation(x) > 1e-6;
                if(in_worse) { ++sc; sv += vi; } else if(in_other) { ++oc; ov += vi; }
            };
            attach(m, c);
            const sol s = solve_native(m);
            if(!added) continue;
            const bool lost = !(std::fabs(s.value - s0.value) < 0.5);
            ++(lost ? o.lost : o.kept);
            o.sib_checks += oc; o.sib_viol += ov; o.same_checks += sc; o.same_viol += sv;
            std::println(stderr, "   seed={} col={} opt={} zalt={} worse_state={} added_at_nodcnt={} incumbent_then={} -> {} {} | subtree viol/checks {}/{} sibling viol/checks {}/{}", seed, col, s0.value, zalt, worse, added_at, best_at, sname(s.status), s.value, sv, sc, ov, oc);
            done = true;
            break;
        }
        if(!done) ++o.nopower;
    }
}
static void x_g1() {
    for(const bool flip : {false, true}) {
        { M m; seti(m, "OutputFlag", 0); scope(m); auto g = cp::build_gadgets(m, 10, flip); setd(m, "TimeLimit", 30);
          const sol s = solve_native(m); std::println(stderr, "S0 no hook flip={}: {} known opt {}", flip, line(s), g.opt); }
        { M m; seti(m, "OutputFlag", 0); scope(m); recipe(m); auto g = cp::build_gadgets(m, 10, flip); setd(m, "TimeLimit", 30);
          auto tr = std::make_shared<cp::scope_tracker>(); tr->cut = g.invalid_global_cut(); tr->trigger = g.worse_state(); tr->state = [&g](const auto & x) { return g.state(x); };
          hctx c; c.on_node = [&](hctx & h, const std::vector<double> & x) { if(tr->wants_add(x)) tr->add_error = h.add(tr->cut); };
          attach(m, c); const sol s = solve_native(m);
          std::println(stderr, "S1 subtree-only cut flip={}: {} known opt {} => {}\n   {}", flip, line(s), g.opt, std::fabs(s.value - g.opt) < 1e-4 ? "KEPT" : "LOST", tr->line()); }
        { M m; seti(m, "OutputFlag", 0); scope(m); recipe(m); auto g = cp::build_gadgets(m, 10, flip); setd(m, "TimeLimit", 30);
          auto tr = std::make_shared<cp::scope_tracker>(); tr->cut = g.hull_cut(3); tr->trigger = g.worse_state(); tr->state = [&g](const auto & x) { return g.state(x); };
          hctx c; c.on_node = [&](hctx & h, const std::vector<double> & x) { if(tr->wants_add(x)) tr->add_error = h.add(tr->cut); };
          attach(m, c); const sol s = solve_native(m);
          std::println(stderr, "S2 valid hull cut flip={}: {} known opt {}\n   {}", flip, line(s), g.opt, tr->line()); }
    }
    g1_out a; std::println(stderr, "own instance, scope settings:"); g1_own(12, true, a);
    std::println(stderr, "own/scope: LOST={} KEPT={} NO_POWER={} sibling viol/checks {}/{} subtree viol/checks {}/{}", a.lost, a.kept, a.nopower, a.sib_viol, a.sib_checks, a.same_viol, a.same_checks);
    g1_out b; std::println(stderr, "own instance, default settings + PreCrush=1, 1 thread:"); g1_own(12, false, b);
    std::println(stderr, "own/default: LOST={} KEPT={} NO_POWER={} sibling viol/checks {}/{} subtree viol/checks {}/{}", b.lost, b.kept, b.nopower, b.sib_viol, b.sib_checks, b.same_viol, b.same_checks);
}

// ======================================================================= G2
// Doubled stable set: each vertex is two parallel binary columns. Odd-hole cuts on the
// doubled columns are valid for every integer point. Separator: 5-holes by enumeration.
struct dss { rawm r; int nv; std::vector<std::array<int, 2>> col; std::vector<std::array<int, 5>> holes; };
static dss make_dss(unsigned seed, int groups, bool doubled = true) {
    dss D; D.nv = groups * 5;
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> W(5, 20), E(0, 3);
    for(int v = 0; v < D.nv; ++v) {
        const double w = W(rng);
        const int a = D.r.bin(w);
        const int b = doubled ? D.r.bin(w) : a;
        D.col.push_back({a, b});
    }
    const auto edge = [&](int u, int v) {
        std::vector<int> i; std::vector<double> c;
        for(int t : {u, v}) { i.push_back(D.col[static_cast<size_t>(t)][0]); c.push_back(1); if(doubled) { i.push_back(D.col[static_cast<size_t>(t)][1]); c.push_back(1); } }
        D.r.add(i, c, '<', 1);
    };
    for(int k = 0; k < groups; ++k) {
        std::array<int, 5> h{};
        for(int t = 0; t < 5; ++t) h[static_cast<size_t>(t)] = k * 5 + t;
        for(int t = 0; t < 5; ++t) edge(h[static_cast<size_t>(t)], h[static_cast<size_t>((t + 1) % 5)]);
        D.holes.push_back(h);
        // links between holes keep presolve and the components split from solving it
        if(k + 1 < groups) for(int t = 0; t < 2; ++t) edge(k * 5 + E(rng), (k + 1) * 5 + E(rng));
    }
    return D;
}
static raw_cut hole_cut(const dss & D, const std::array<int, 5> & h, bool doubled = true) {
    raw_cut c{{}, {}, 'L', 2};
    for(int v : h) { c.ind.push_back(D.col[static_cast<size_t>(v)][0]); c.val.push_back(1); if(doubled) { c.ind.push_back(D.col[static_cast<size_t>(v)][1]); c.val.push_back(1); } }
    return c;
}
static void x_g2() {
    for(const int precrush : {0, 1}) for(const int cuts : {-1, 0}) for(unsigned seed : {1u, 2u, 3u}) {
        const dss D = make_dss(seed, 12);
        M base; seti(base, "OutputFlag", 0); seti(base, "Threads", 1); if(cuts == 0) seti(base, "Cuts", 0); D.r.load(base);
        const sol s0 = solve_native(base, false);
        M m; seti(m, "OutputFlag", 1); seti(m, "LogToConsole", 0); seti(m, "Threads", 1); setd(m, "TimeLimit", 30); seti(m, "PreCrush", precrush); if(cuts == 0) seti(m, "Cuts", 0);
        D.r.load(m);
        std::vector<raw_cut> added; long checks = 0, viol = 0, first_checks = 0, first_viol = 0;
        hctx c; std::string dims;
        c.on_msg = [&](hctx &, const char * s) { std::string t(s); if(t.starts_with("Presolved:")) { dims = t; if(!dims.empty() && dims.back() == '\n') dims.pop_back(); } };
        c.on_node = [&](hctx & h, const std::vector<double> & x) {
            for(const auto & a : added) { ++checks; viol += a.violation(x) > 1e-6; }
            if(!added.empty()) { ++first_checks; first_viol += added.front().violation(x) > 1e-6; }
            for(const auto & hl : D.holes) { const raw_cut k = hole_cut(D, hl); if(k.violation(x) > 1e-4 && std::ranges::none_of(added, [&](const raw_cut & e) { return e.ind == k.ind; })) { if(h.add(k) == 0) added.push_back(k); } }
        };
        attach(m, c);
        const sol s = solve_native(m, false);
        std::println(stderr, "dss12 PreCrush={} Cuts={} seed={} baseline {} | {} | {} | cuts_added={} first_cut later viol/checks={}/{} all later viol/checks={}/{} native_err={}", precrush, cuts, seed, s0.value, line(s), dims, added.size(), first_viol, first_checks, viol, checks, c.native_err);
    }
}

// ======================================================================= G3
static void g3_report(const std::string & name, M & m, hctx & c, const sol & s) {
    (void)m;
    std::println(stderr, "{}: {}\n   {} threads={} off_main={} overlaps={} tree_fires_distinct_nodcnt={}", name, line(s), c.counters(), c.threads.size(), c.off_main.load(), c.overlaps.load(), c.tree_fires_per_nod.size());
}
static void x_g3() {
    const auto run_cover = [](const std::string & name, unsigned seed, int threads, std::function<void(M &)> params, size_t cols = 60, size_t rows = 10) {
        const cgm G = make_cg(seed, cols, rows);
        M m; seti(m, "OutputFlag", 0); if(threads) seti(m, "Threads", threads); setd(m, "TimeLimit", 20); recipe(m); params(m); G.r.load(m);
        hctx c; c.on_node = [&](hctx & h, const std::vector<double> & x) { for(const auto & k : covers(G, x)) h.add(k); };
        attach(m, c); const sol s = solve_native(m, false);
        g3_report(std::format("{} seed={} threads={}", name, seed, threads), m, c, s);
    };
    const auto run_atsp = [](const std::string & name, int n, unsigned seed, int threads, std::function<void(M &)> params) {
        M m; seti(m, "OutputFlag", 0); if(threads) seti(m, "Threads", threads); setd(m, "TimeLimit", 20); recipe(m); params(m);
        auto a = cp::build_atsp(m, n, false, seed);
        hctx c; c.on_node = [&](hctx & h, const std::vector<double> & x) { for(const auto & k : a.separate(x)) h.add(k); };
        attach(m, c); const sol s = solve_native(m, false);
        g3_report(std::format("{} atsp n={} seed={} threads={}", name, n, seed, threads), m, c, s);
    };
    const auto nop = [](M &) {};
    for(unsigned seed : {1u, 2u}) run_cover("covers default", seed, 1, nop);
    run_cover("covers CutPasses=5", 1, 1, [](M & m) { seti(m, "CutPasses", 5); });
    run_cover("covers Cuts=3", 1, 1, [](M & m) { seti(m, "Cuts", 3); });
    run_cover("covers Cuts=0", 1, 1, [](M & m) { seti(m, "Cuts", 0); });
    run_cover("covers CutAggPasses=5", 1, 1, [](M & m) { seti(m, "CutAggPasses", 5); });
    run_cover("covers NoRelHeurWork=0 MIPFocus=3", 1, 1, [](M & m) { seti(m, "MIPFocus", 3); });
    run_cover("covers default threads", 1, 0, nop);
    run_cover("covers 8 threads", 2, 8, nop);
    run_atsp("dfj default", 44, 9, 1, nop);
    run_atsp("dfj MIPFocus=3 Cuts=3", 44, 9, 1, [](M & m) { seti(m, "MIPFocus", 3); seti(m, "Cuts", 3); });
    run_atsp("dfj default threads", 44, 9, 0, nop);
    // never fires
    { M m; seti(m, "OutputFlag", 0); seti(m, "Threads", 1); auto gg = cp::build_gadgets(m, 10, false);
      hctx c; attach(m, c); const sol s = solve_native(m, false); g3_report("gadgets default settings (presolve solves it)", m, c, s); }
    { M m; seti(m, "OutputFlag", 0); seti(m, "Threads", 1); rawm r; r.sense = -1; int a = r.bin(3), b = r.bin(2); r.add({a, b}, {1, 1}, '<', 1); r.load(m);
      hctx c; attach(m, c); const sol s = solve_native(m, false); g3_report("two binaries", m, c, s); }
}

// ======================================================================= G4
// A core knapsack; two parallel integers b1, b2 in [0,3], worth a lot, in two rows with four
// items of the core optimum. k = b1 + b2 at the no-cut optimum; b1 <= k-3 keeps (k-3, 3).
struct dom { rawm r; int b1, b2; };
static dom make_dom(unsigned seed, double bobj, double cap, const std::vector<int> * order) {
    dom D; const size_t cols = 40, rows = 5;
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> A(1, 1000), R(0, 500);
    std::vector<std::vector<double>> a(rows, std::vector<double>(cols));
    for(auto & r : a) for(auto & v : r) v = A(rng);
    std::vector<int> xs;
    for(size_t j = 0; j < cols; ++j) { double t = 0; for(auto & r : a) t += r[j]; xs.push_back(D.r.bin(std::floor(t / rows + R(rng)))); }
    for(size_t i = 0; i < rows; ++i) { double t = 0; for(double v : a[i]) t += v; D.r.add(xs, a[i], '<', std::floor(t / 2)); }
    D.b1 = D.b2 = -1;
    if(order) {
        D.b1 = D.r.var(bobj, 0, 3, 'I'); D.b2 = D.r.var(bobj, 0, 3, 'I');
        const auto & o = *order;
        D.r.add({D.b1, D.b2, o[0], o[1]}, {50, 50, 500, 700}, '<', cap);
        D.r.add({D.b1, D.b2, o[2], o[3]}, {70, 70, 300, 900}, '<', cap);
    }
    return D;
}
static void x_g4(unsigned seeds) {
    int used = 0, wrong_pc1_def = 0, wrong_pc1_safe = 0, wrong_pc0 = 0, runs_pc1_def = 0, runs_pc1_safe = 0, runs_pc0 = 0;
    for(unsigned seed = 1; seed <= seeds; ++seed) for(double bobj : {45.0, 150.0}) for(double cap : {1500.0, 1460.0}) {
        std::vector<int> order;
        { const dom C = make_dom(seed, 0, 0, nullptr); M b; seti(b, "OutputFlag", 0); seti(b, "Threads", 1); setd(b, "TimeLimit", 10); C.r.load(b); const sol sc = solve_native(b); for(size_t j = 0; j < sc.x.size(); ++j) if(sc.x[j] > 0.5) order.push_back(static_cast<int>(j)); if(order.size() < 4) continue; }
        const dom D = make_dom(seed, bobj, cap, &order);
        M base; seti(base, "OutputFlag", 0); seti(base, "Threads", 1); setd(base, "MIPGap", 0.0); setd(base, "TimeLimit", 30); D.r.load(base);
        const sol s0 = solve_native(base);
        if(s0.status != 2) continue;
        const int k = static_cast<int>(std::round(s0.x[static_cast<size_t>(D.b1)] + s0.x[static_cast<size_t>(D.b2)]));
        if(k < 3 || k > 5) continue;
        const double rhs = k - 3;
        M ctl; seti(ctl, "OutputFlag", 0); seti(ctl, "Threads", 1); setd(ctl, "MIPGap", 0.0); setd(ctl, "TimeLimit", 30); rawm hard = D.r; hard.add({D.b1}, {1}, '<', rhs); hard.load(ctl);
        const sol sc = solve_native(ctl);
        const bool keeps = sc.status == 2 && std::fabs(sc.value - s0.value) < 1e-6;
        std::println(stderr, "seed={} bobj={} cap={} k={} baseline {} nodes={} | hard b1<={} control: {} {} keeps optimum={}", seed, bobj, cap, k, s0.value, s0.nodes, rhs, sname(sc.status), sc.value, keeps);
        if(!keeps) continue;
        ++used;
        for(int pc : {1, 0}) for(int mode : {0, 1, 2}) for(int heur : {-1, 0}) {
            M m; seti(m, "OutputFlag", 0); seti(m, "Threads", 1); setd(m, "MIPGap", 0.0); setd(m, "TimeLimit", 30);
            seti(m, "PreCrush", pc);
            if(mode == 1) seti(m, "DualReductions", 0);
            if(mode == 2) seti(m, "LazyConstraints", 1);
            if(heur == 0) setd(m, "Heuristics", 0.0);
            D.r.load(m);
            raw_cut cut{{D.b1}, {1.0}, 'L', rhs};
            long later_v = 0, later_c = 0; bool first = true; double viol_at = NAN;
            hctx c; c.on_node = [&](hctx & h, const std::vector<double> & x) {
                if(!first) { ++later_c; later_v += cut.violation(x) > 1e-6; }
                if(cut.violation(x) > 1e-6) { if(first) viol_at = cut.violation(x); if(h.add(cut) == 0) first = false; }
            };
            attach(m, c);
            const sol s = solve_native(m);
            const char * modes[] = {"default", "DualReductions=0", "LazyConstraints=1"};
            const bool right = s.status == 2 && std::fabs(s.value - s0.value) < 1e-6;
            if(c.cuts > 0) { if(pc == 1 && mode == 0) { ++runs_pc1_def; wrong_pc1_def += !right; } else if(pc == 1) { ++runs_pc1_safe; wrong_pc1_safe += !right; } else if(mode == 0) { ++runs_pc0; wrong_pc0 += !right; } }
            std::println(stderr, "   PreCrush={} {:18} Heuristics={:2}: {} {} {} cuts={} viol_at_add={} later viol/checks={}/{} b1_final={} nodes={} fires={} native_err={}", pc, modes[mode], heur, right ? "RIGHT" : "WRONG", sname(s.status), s.value, c.cuts, viol_at, later_v, later_c, s.x.empty() ? NAN : s.x[static_cast<size_t>(D.b1)], s.nodes, c.fires, c.native_err);
        }
    }
    std::println(stderr, "instances with a keeping cut: {}; with cuts added: PreCrush=1 default wrong {}/{}, PreCrush=1 DualReductions=0 or Lazy wrong {}/{}, PreCrush=0 default wrong {}/{}", used, wrong_pc1_def, runs_pc1_def, wrong_pc1_safe, runs_pc1_safe, wrong_pc0, runs_pc0);
}

// ======================================================================= G5
static void x_g5() {
    const auto run = [](const std::string & name, int threads, int concurrent, bool with_cuts, double spin_us) {
        const cgm G = make_cg(7, 100, 30);
        M m; seti(m, "OutputFlag", 0); if(threads) seti(m, "Threads", threads); if(concurrent) seti(m, "ConcurrentMIP", concurrent); setd(m, "TimeLimit", 4); recipe(m); G.r.load(m);
        hctx c; c.spin = std::chrono::microseconds(static_cast<long>(spin_us));
        if(with_cuts) c.on_node = [&](hctx & h, const std::vector<double> & x) { for(const auto & k : covers(G, x)) h.add(k); };
        attach(m, c); const sol s = solve_native(m, false);
        std::println(stderr, "{} threads={} concurrent={} cuts={} spin={}us: {} | fires={} opt_calls={} cuts={} hook_threads={} off_optimize_thread={} overlaps={} max_inflight={} swallowed={} native_err={}", name, threads, concurrent, with_cuts, spin_us, line(s), c.fires, c.opt_calls, c.cuts, c.threads.size(), c.off_main.load(), c.overlaps.load(), c.max_inflight.load(), c.swallowed, c.native_err);
    };
    for(int t : {2, 8, 16, 0}) run("spin", t, 0, false, 200);
    for(int t : {8, 0}) run("cuts+spin", t, 0, true, 200);
    run("cuts+spin", 8, 2, true, 200);
    run("cuts+spin", 16, 4, true, 200);
    run("cuts,no spin", 0, 0, true, 0);
}

// ======================================================================= G6
// The invalid cut c'x <= zalt, stop by node limit, then resume in several ways.
static void x_g6() {
    for(const bool flip : {false, true}) {
        const auto first = [&](M & m, cp::gadgets<M> & g, hctx & c, raw_cut & cut, bool & added) {
            seti(m, "OutputFlag", 0); scope(m); recipe(m); setd(m, "TimeLimit", 20); setd(m, "NodeLimit", 30);
            cut = g.invalid_global_cut();
            c.on_node = [&](hctx & h, const std::vector<double> & x) { if(!added && g.state(x) == g.worse_state() && cut.violation(x) > 1e-4) { h.add(cut); added = true; } };
            attach(m, c);
            return solve_native(m, false);
        };
        const auto checker = [](hctx & c, const raw_cut & cut, long & ck, long & vi) {
            c.on_node = [&, cut](hctx &, const std::vector<double> & x) { ++ck; vi += cut.violation(x) > 1e-6; };
        };
        for(const std::string variant : {"control_no_cut", "keep_hook", "detach_reattach", "no_hook", "reset", "param_precrush0", "param_lazy", "param_dualred", "param_threads", "param_cuts", "rhs_change_other", "update_noop", "obj_same_value"}) {
            M m; auto g = cp::build_gadgets(m, 10, flip);
            hctx c; raw_cut cut; bool added = false;
            sol s1;
            if(variant == "control_no_cut") { seti(m, "OutputFlag", 0); scope(m); recipe(m); setd(m, "TimeLimit", 20); setd(m, "NodeLimit", 30); cut = g.invalid_global_cut(); attach(m, c); s1 = solve_native(m, false); }
            else s1 = first(m, g, c, cut, added);
            long ck = 0, vi = 0;
            auto [env, model] = m.native_model();
            auto & api = m.native_api();
            setd(m, "NodeLimit", 1e100);
            hctx c2; checker(c2, cut, ck, vi);
            std::string extra;
            if(variant == "keep_hook") { c2.cols = c.cols; attach(m, c2); }
            else if(variant == "control_no_cut") { attach(m, c2); }
            else if(variant == "detach_reattach") { detach(m); attach(m, c2); }
            else if(variant == "no_hook") { detach(m); }
            else if(variant == "reset") { api.reset(model, 0); attach(m, c2); }
            else if(variant == "param_precrush0") { seti(m, "PreCrush", 0); attach(m, c2); }
            else if(variant == "param_lazy") { seti(m, "LazyConstraints", 1); attach(m, c2); }
            else if(variant == "param_dualred") { seti(m, "DualReductions", 0); attach(m, c2); }
            else if(variant == "param_threads") { seti(m, "Threads", 2); attach(m, c2); }
            else if(variant == "param_cuts") { seti(m, "Cuts", 2); attach(m, c2); }
            else if(variant == "rhs_change_other") { // a real model change: relax a gadget row that is slack at the optimum
                double r = 0; api.getdblattrelement(model, "RHS", 0, &r); api.setdblattrelement(model, "RHS", 0, r + 1e-9); api.updatemodel(model); attach(m, c2); }
            else if(variant == "update_noop") { api.updatemodel(model); attach(m, c2); }
            else if(variant == "obj_same_value") { double o = 0; api.getdblattrelement(model, "Obj", 0, &o); api.setdblattrelement(model, "Obj", 0, o); api.updatemodel(model); attach(m, c2); }
            const sol s2 = solve_native(m, false);
            std::println(stderr, "flip={} {:20} first: added={} {} incumbent={} | resume: {} known opt {} => {} | later viol/checks={}/{}", flip, variant, added, sname(s1.status), s1.value, line(s2), g.opt, std::fabs(s2.value - g.opt) < 1e-4 ? "optimum KEPT" : "optimum LOST", vi, ck);
        }
    }
    // A finished unchanged model.
    { M m; seti(m, "OutputFlag", 0); seti(m, "Threads", 1); const cgm G = make_cg(3); G.r.load(m); hctx c; attach(m, c); const sol s = solve_native(m, false); const long f = c.fires; const sol s2 = solve_native(m, false); std::println(stderr, "finished model: first fires={} {}; second solve fires={} {}", f, line(s), c.fires - f, line(s2)); }
}

// ======================================================================= G7
static std::string path_of(const sol & s) { return std::format("nodes={} iters={} work={:.6f} value={:.10g} bound={:.10g} status={}", s.nodes, s.iters, s.work, s.value, s.bound, sname(s.status)); }
static void x_g7() {
    using builder = std::function<void(M &)>;
    std::vector<std::pair<std::string, builder>> insts = {
        {"atsp26s1", [](M & m) { cp::build_atsp(m, 26, false, 1); }},
        {"atsp20s2", [](M & m) { cp::build_atsp(m, 20, false, 2); }},
        {"mkp5x60", [](M & m) { cp::build_mkp(m, 5, 60, 3); setd(m, "NodeLimit", 20000); }},
        {"cg45s1", [](M & m) { make_cg(1, 45, 15).r.load(m); }},
        {"dss12s1", [](M & m) { make_dss(1, 12).r.load(m); }},
    };
    for(const auto & [name, build] : insts) for(int threads : {1, 8}) {
        const auto run = [&, build = build, threads](const char * label, int precrush, int lazy, int hookkind) {
            M m; seti(m, "OutputFlag", 0); seti(m, "Threads", threads); setd(m, "TimeLimit", 30); seti(m, "PreCrush", precrush); seti(m, "LazyConstraints", lazy); build(m);
            hctx c;
            auto [env, model] = m.native_model();
            if(hookkind == 1) m.native_api().setcallbackfunc(model, noop_hook, nullptr);
            if(hookkind == 2) { c.on_node = [](hctx &, const std::vector<double> &) {}; attach(m, c); }
            const sol s = solve_native(m, false);
            std::println(stderr, "{} threads={} {:30} {} fires={}", name, threads, label, path_of(s), c.fires);
        };
        run("none", 0, 0, 0);
        run("noop hook", 0, 0, 1);
        run("hook reads status+REL", 0, 0, 2);
        run("PreCrush=1 none", 1, 0, 0);
        run("PreCrush=1 reads REL", 1, 0, 2);
        run("LazyConstraints=1 none", 0, 1, 0);
    }
}

// ======================================================================= G8
// Max z + stuff; z in a row, so that presolve keeps it. First call: duplicate-term cut on z.
static void x_g8() {
    struct t { const char * name; std::vector<double> v; double rhs; double sum, last, first; };
    const std::vector<t> tests = {
        {"(0.25,0.75)<=2", {0.25, 0.75}, 2, 2, 2.6667, 8},
        {"(0.75,0.25)<=2", {0.75, 0.25}, 2, 2, 8, 2.6667},
        {"(0.25,0.5,0.75)<=2", {0.25, 0.5, 0.75}, 2, 1.3333, 2.6667, 8},
        {"(0.75,0.5,0.25)<=2", {0.75, 0.5, 0.25}, 2, 1.3333, 8, 2.6667},
        {"(0.5,-0.25,0.4)<=1", {0.5, -0.25, 0.4}, 1, 1.5385, 2.5, 2},
    };
    for(int presolve : {0, -1}) for(bool nonadjacent : {false, true}) for(const auto & T : tests) {
        rawm r; r.sense = -1;
        const cgm G = make_cg(3, 35, 15);
        r = G.r;
        const int z = r.var(0.01, 0, 10, 'C');
        const int w = r.bin(0);
        // z joined to the knapsack so that it is not a singleton
        { auto i = G.x; i.push_back(z); std::vector<double> v(G.x.size(), 1.0); v.push_back(1.0); r.add(i, v, '<', 40); }
        M m; seti(m, "OutputFlag", 0); seti(m, "Threads", 1); seti(m, "Presolve", presolve); setd(m, "Heuristics", 0.0); setd(m, "TimeLimit", 20); recipe(m); r.load(m);
        hctx c; c.san = false; bool done = false; double zmax_later = -1, z_at_add = NAN;
        c.on_node = [&](hctx & h, const std::vector<double> & x) {
            const double zv = x[static_cast<size_t>(z)];
            if(done) { zmax_later = std::max(zmax_later, zv); return; }
            raw_cut k{{}, {}, 'L', T.rhs};
            for(size_t a = 0; a < T.v.size(); ++a) { if(nonadjacent && a == 1) { k.ind.push_back(w); k.val.push_back(1e-3); } k.ind.push_back(z); k.val.push_back(T.v[a]); }
            z_at_add = zv; done = h.add(k) == 0;
        };
        attach(m, c);
        const sol s = solve_native(m);
        const double zf = s.x.empty() ? NAN : s.x[static_cast<size_t>(z)];
        std::println(stderr, "Presolve={} nonadjacent={} {:22} sum z<={:.4f} last z<={:.4f} first z<={:.4f} | z at add={} max later REL z={:.4f} final z={:.4f} status={} calls={} native_err={}", presolve, nonadjacent, T.name, T.sum, T.last, T.first, z_at_add, zmax_later, zf, sname(s.status), c.opt_calls, c.native_err);
    }
}

// ======================================================================= G9
// Gadgets (scope settings), N spare unused columns, the valid hull cut of gadget 3 with
// variations. "Applied" is read from later relaxations and the node count (S0 vs hull).
static void x_g9() {
    const int spare = 60000;
    struct var { std::string name; std::function<raw_cut(const cp::gadgets<M> &, int)> make; };
    std::vector<var> vs;
    const auto scale = [](raw_cut c, double k) { for(auto & v : c.val) v *= k; c.rhs *= k; return c; };
    const auto pad = [](raw_cut c, int first, int count, double coef) { for(int j = 0; j < count; ++j) { c.ind.push_back(first + j); c.val.push_back(coef); } return c; };
    vs.push_back({"no cut", {}});
    vs.push_back({"plain hull", [=](const auto & g, int) { return g.hull_cut(3); }});
    for(double k : {1e-9, 1e-7, 1e-5, 1e-3, 1e3, 1e5, 1e7, 1e9, 1e13}) vs.push_back({std::format("scaled x{:g}", k), [=](const auto & g, int) { return scale(g.hull_cut(3), k); }});
    for(double k : {1e3, 1e4, 1e5, 1e6, 1e7, 1e9}) vs.push_back({std::format("+{:g} on 1 unused col", k), [=](const auto & g, int f) { return pad(g.hull_cut(3), f, 1, k); }});
    for(int n : {100, 1000, 5000, 20000, 60000}) vs.push_back({std::format("+1 on {} unused cols", n), [=](const auto & g, int f) { return pad(g.hull_cut(3), f, n, 1.0); }});
    for(int n : {100, 1000, 5000, 20000, 60000}) vs.push_back({std::format("+1e-4 on {} unused cols", n), [=](const auto & g, int f) { return pad(g.hull_cut(3), f, n, 1e-4); }});
    for(int n : {1000, 20000}) vs.push_back({std::format("+1e-9 on {} unused cols", n), [=](const auto & g, int f) { return pad(g.hull_cut(3), f, n, 1e-9); }});
    for(const auto & v : vs) {
        M m; seti(m, "OutputFlag", 0); scope(m); recipe(m); setd(m, "TimeLimit", 20);
        auto g = cp::build_gadgets(m, 10, false);
        auto u = m.add_variables(static_cast<size_t>(spare), {.obj_coef = 0, .lower_bound = 0, .upper_bound = 1});
        const int first = m.native_id(u(std::size_t{0}));
        hctx c; c.san = false; raw_cut plain = g.hull_cut(3); raw_cut k; bool added = false; long ck = 0, vi = 0; int err = 0; double viol_at = NAN;
        if(v.make) k = v.make(g, first);
        c.on_node = [&](hctx & h, const std::vector<double> & x) {
            if(added) { ++ck; vi += plain.violation(x) > 1e-6; return; }
            if(v.make && plain.violation(x) > 1e-4) { viol_at = plain.violation(x); err = h.add(k); added = true; }
        };
        attach(m, c);
        const sol s = solve_native(m, false);
        std::println(stderr, "{:30} err={} violation_at_add={:.4g} later hull viol/checks={}/{} {} calls={}", v.name, err, viol_at, vi, ck, line(s), c.opt_calls);
    }
    // A cut violated by a lot, 100% dense on a model with many real columns: covers all columns.
    for(int n : {200, 2000, 10000, 40000}) {
        rawm r; r.sense = -1;
        std::mt19937 rng(11);
        std::uniform_int_distribution<int> A(1, 100);
        std::vector<int> cols; for(int j = 0; j < n; ++j) cols.push_back(r.bin(A(rng)));
        for(int i = 0; i < 3; ++i) { std::vector<double> v; double s = 0; for(int j = 0; j < n; ++j) { v.push_back(A(rng)); s += v.back(); } r.add(cols, v, '<', std::floor(s / 2)); }
        M m; seti(m, "OutputFlag", 0); seti(m, "Threads", 1); seti(m, "Presolve", 0); seti(m, "Cuts", 0); setd(m, "Heuristics", 0.0); setd(m, "TimeLimit", 15); recipe(m); r.load(m);
        hctx c; c.san = false; raw_cut k{cols, std::vector<double>(cols.size(), 1.0), 'L', 0}; bool added = false; long ck = 0, vi = 0; double viol = NAN;
        c.on_node = [&](hctx & h, const std::vector<double> & x) {
            double a = 0; for(int j : cols) a += x[static_cast<size_t>(j)];
            if(!added) { k.rhs = std::floor(a * 0.97); viol = a - k.rhs; h.add(k); added = true; return; }
            ++ck; vi += k.violation(x) > 1e-6;
        };
        attach(m, c);
        const sol s = solve_native(m, false);
        std::println(stderr, "100% dense, n={} columns, strongly violated by {:.1f}: later viol/checks={}/{} {} native_err={}", n, viol, vi, ck, line(s), c.native_err);
        std::fflush(stderr);
    }
}


// G9 thresholds: nonzeros and coefficient range, including a natural big-M cut.
static void x_g9b() {
    const auto padded = [](int spare, int nnz, double coef) {
        M m; seti(m, "OutputFlag", 0); scope(m); recipe(m); setd(m, "TimeLimit", 20);
        auto g = cp::build_gadgets(m, 10, false);
        auto u = m.add_variables(static_cast<size_t>(spare), {.obj_coef = 0, .lower_bound = 0, .upper_bound = 1});
        const int first = m.native_id(u(std::size_t{0}));
        hctx c; c.san = false; raw_cut plain = g.hull_cut(3), k = plain; for(int j = 0; j < nnz; ++j) { k.ind.push_back(first + j); k.val.push_back(coef); }
        bool added = false; long ck = 0, vi = 0;
        c.on_node = [&](hctx & h, const std::vector<double> & x) { if(added) { ++ck; vi += plain.violation(x) > 1e-6; return; } if(plain.violation(x) > 1e-4) { h.add(k); added = true; } };
        attach(m, c); const sol s = solve_native(m, false);
        std::println(stderr, "padded spare={} nnz={} coef={:g}: later hull viol/checks={}/{} nodes={} {}", spare, nnz, coef, vi, ck, s.nodes, vi == 0 ? "APPLIED" : "NOT APPLIED");
    };
    for(int n : {5000, 7000, 10000, 12000, 15000, 18000}) padded(60000, n, 1.0);
    for(int n : {5000, 7000, 10000, 15000}) padded(20000, n, 1.0);
    for(int n : {100, 2000, 5000}) padded(6000, n, 1.0);
    const auto range = [](double K, bool on_unused) {
        M m; seti(m, "OutputFlag", 0); scope(m); recipe(m); setd(m, "TimeLimit", 20);
        auto g = cp::build_gadgets(m, 10, false);
        auto u = m.add_variables(1, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 1});
        const int col = on_unused ? m.native_id(u(std::size_t{0})) : g.dummy;
        hctx c; c.san = false; raw_cut plain = g.hull_cut(3), k = plain; k.ind.push_back(col); k.val.push_back(K);
        bool added = false; long ck = 0, vi = 0;
        c.on_node = [&](hctx & h, const std::vector<double> & x) { if(added) { ++ck; vi += plain.violation(x) > 1e-6; return; } if(plain.violation(x) > 1e-4) { h.add(k); added = true; } };
        attach(m, c); const sol s = solve_native(m, false);
        std::println(stderr, "range K={:g} on {}: later hull viol/checks={}/{} nodes={} {}", K, on_unused ? "unused" : "dummy", vi, ck, s.nodes, vi == 0 ? "APPLIED" : "NOT APPLIED");
    };
    for(double K : {1e3, 2e3, 3e3, 5e3, 7e3, 1e4}) range(K, true);
    // Natural big-M: one row Sum_j f_ij <= 3 M y_i, f_ij <= M y_i valid since each client needs at most M.
    for(double Mv : {1e2, 1e3, 1e4, 1e5, 1e6, 1e7}) for(int presolve : {-1, 0}) {
        rawm r; r.sense = 1; const size_t fac = 8, cli = 12;
        std::mt19937 rng(3); std::uniform_int_distribution<int> C(1, 40), F(100, 300);
        std::uniform_real_distribution<double> DD(0.3, 1.0);
        std::vector<double> d(cli); for(auto & v : d) v = Mv * DD(rng);
        std::vector<int> y; for(size_t i = 0; i < fac; ++i) y.push_back(r.bin(F(rng) * Mv / 10.0));
        std::vector<std::vector<int>> f(fac, std::vector<int>(cli));
        for(auto & row : f) for(auto & v : row) v = r.var(C(rng), 0, 1e20, 'C');
        for(size_t j = 0; j < cli; ++j) { std::vector<int> i; for(size_t k = 0; k < fac; ++k) i.push_back(f[k][j]); r.add(i, std::vector<double>(i.size(), 1.0), '=', d[j]); }
        for(size_t k = 0; k < fac; ++k) { std::vector<int> i; std::vector<double> v; for(size_t j = 0; j < cli; ++j) { i.push_back(f[k][j]); v.push_back(1); } i.push_back(y[k]); v.push_back(-3 * Mv); r.add(i, v, '<', 0); }
        M m; seti(m, "OutputFlag", 0); seti(m, "Threads", 1); seti(m, "Presolve", presolve); seti(m, "Cuts", 0); setd(m, "Heuristics", 0.0); setd(m, "TimeLimit", 15); recipe(m); r.load(m);
        hctx c; c.san = false; std::vector<raw_cut> added; std::set<std::pair<size_t, size_t>> have; long ck = 0, vi = 0;
        c.on_node = [&](hctx & h, const std::vector<double> & x) {
            for(const auto & a : added) { ++ck; vi += a.violation(x) > 1e-5 * Mv; }
            for(size_t k = 0; k < fac; ++k) for(size_t j = 0; j < cli; ++j) {
                raw_cut q{{f[k][j], y[k]}, {1.0, -d[j]}, 'L', 0.0};
                if(q.violation(x) > 1e-4 * Mv && have.insert({k, j}).second) { h.add(q); added.push_back(q); }
            }
        };
        attach(m, c); const sol s = solve_native(m, false);
        std::println(stderr, "natural big-M M={:g} Presolve={}: {} calls={} cuts={} earlier cuts later viol/checks={}/{} {}", Mv, presolve, line(s), c.opt_calls, c.cuts, vi, ck, ck == 0 ? "NO POWER" : vi * 2 < ck ? "APPLIED (mostly)" : "NOT APPLIED (mostly)");
    }
    // 100% dense thresholds
    for(int n : {15000, 20000, 30000}) {
        rawm r; r.sense = -1; std::mt19937 rng(11); std::uniform_int_distribution<int> A(1, 100);
        std::vector<int> cols; for(int j = 0; j < n; ++j) cols.push_back(r.bin(A(rng)));
        for(int i = 0; i < 3; ++i) { std::vector<double> v; double t = 0; for(int j = 0; j < n; ++j) { v.push_back(A(rng)); t += v.back(); } r.add(cols, v, '<', std::floor(t / 2)); }
        M m; seti(m, "OutputFlag", 0); seti(m, "Threads", 1); seti(m, "Presolve", 0); seti(m, "Cuts", 0); setd(m, "Heuristics", 0.0); setd(m, "TimeLimit", 10); recipe(m); r.load(m);
        hctx c; c.san = false; raw_cut k{cols, std::vector<double>(cols.size(), 1.0), 'L', 0}; bool added = false; long ck = 0, vi = 0;
        c.on_node = [&](hctx & h, const std::vector<double> & x) { double a = 0; for(int j : cols) a += x[static_cast<size_t>(j)]; if(!added) { k.rhs = std::floor(a * 0.97); h.add(k); added = true; return; } ++ck; vi += k.violation(x) > 1e-6; };
        attach(m, c); const sol s = solve_native(m, false);
        std::println(stderr, "100% dense n={}: later viol/checks={}/{} nodes={}", n, vi, ck, s.nodes);
    }
}


// Is the range filter an efficacy test? Same K, a larger violation: the objective-level cut of the gadgets.
static void x_g9c() {
    for(double K : {1e3, 1e4, 3e4, 1e5, 3e5, 1e6}) for(int kind : {0, 1}) {
        M m; seti(m, "OutputFlag", 0); scope(m); recipe(m); setd(m, "TimeLimit", 20);
        auto g = cp::build_gadgets(m, 10, false);
        auto u = m.add_variables(1, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 1});
        const int col = m.native_id(u(std::size_t{0}));
        raw_cut plain = kind == 0 ? g.hull_cut(3) : raw_cut{g.z, std::vector<double>(g.z.size(), 1.0), 'L', g.opt}, k = plain; k.ind.push_back(col); k.val.push_back(K);
        hctx c; c.san = false; bool added = false; long ck = 0, vi = 0; double v0 = NAN;
        c.on_node = [&](hctx & h, const std::vector<double> & x) { if(added) { ++ck; vi += plain.violation(x) > 1e-6; return; } if(plain.violation(x) > 1e-4) { v0 = plain.violation(x); h.add(k); added = true; } };
        attach(m, c); const sol s = solve_native(m, false);
        double norm = 0; for(double v : k.val) norm += v * v; norm = std::sqrt(norm);
        std::println(stderr, "K={:g} {}: violation={:.3f} efficacy={:.2e} later viol/checks={}/{} nodes={} {}", K, kind == 0 ? "hull cut" : "objective-level cut", v0, v0 / norm, vi, ck, s.nodes, vi == 0 ? "APPLIED" : "NOT APPLIED");
    }
}

// ======================================================================= Q2
static void x_q2() {
    const auto check = [](const std::string & name, const rawm & r, int threads, std::function<void(M &)> params, bool with_cuts_covers, const cgm * G) {
        M m; seti(m, "OutputFlag", 1); seti(m, "LogToConsole", 0); seti(m, "Threads", threads); setd(m, "TimeLimit", 20); recipe(m); params(m); r.load(m);
        auto [env, um] = m.native_model();
        hctx c; std::vector<double> obj(r.obj.size());
        long errs = 0, calls = 0, mismatch_own = 0, viol_bound = 0, sol_calls = 0, sol_mismatch = 0, model_diff = 0; double worst = 0, worst_sol = 0, first_val = NAN, first_bnd = NAN, last_root = NAN; double spent = 0;
        std::string root_line;
        c.on_msg = [&](hctx &, const char * s) { std::string t(s); if(t.starts_with("Root relaxation:")) { root_line = t; if(root_line.back() == '\n') root_line.pop_back(); } };
        g::GRBmodel * cbm_seen = nullptr;
        const auto comp = [&](const std::vector<double> & x, double & out) {
            double con = NAN; int e1 = c.api->getdblattrarray(um, "Obj", 0, static_cast<int>(obj.size()), obj.data()); int e2 = c.api->getdblattr(um, "ObjCon", &con);
            if(e1 || e2) { ++errs; return false; }
            double v = con; for(size_t j = 0; j < obj.size(); ++j) v += obj[j] * x[j];
            out = v; return true;
        };
        c.on_node = [&](hctx & h, const std::vector<double> & x) {
            ++calls; double v;
            const auto t0 = clk::now();
            if(!comp(x, v)) return;
            spent += duration<double>(clk::now() - t0).count();
            const double own = r.eval(x);
            worst = std::max(worst, std::fabs(v - own)); if(std::fabs(v - own) > 1e-9) ++mismatch_own;
            if(calls == 1) { first_val = v; first_bnd = h.bnd; }
            if(h.nod == 0) last_root = v;
            // a node relaxation can never beat the global bound
            if(r.sense < 0 ? v > h.bnd + 1e-6 * (1 + std::fabs(h.bnd)) : v < h.bnd - 1e-6 * (1 + std::fabs(h.bnd))) ++viol_bound;
            if(with_cuts_covers && G) for(const auto & k : covers(*G, x)) h.add(k);
        };
        c.on_sol = [&](hctx &, const std::vector<double> & x, double o) { ++sol_calls; double v; if(!comp(x, v)) return; worst_sol = std::max(worst_sol, std::fabs(v - o)); if(std::fabs(v - o) > 1e-6) ++sol_mismatch; };
        attach(m, c);
        const sol s = solve_native(m, false);
        std::println(stderr, "{} sense={} offset={} threads={}: {} | calls={} read_errs={} |own-formula|max={:.3g} mismatches={} bound_violations={} MIPSOL calls={} |Obj'x+ObjCon-MIPSOL_OBJ|max={:.3g} mismatches={} first value={:.8g} first OBJBND={:.8g} last root value={:.8g} | {} | read cost {:.1f} us/call swallowed={}", name, r.sense, r.offset, threads, line(s), calls, errs, worst, mismatch_own, viol_bound, sol_calls, worst_sol, sol_mismatch, first_val, first_bnd, last_root, root_line, calls ? 1e6 * spent / static_cast<double>(calls) : 0.0, c.swallowed);
        (void)cbm_seen; (void)model_diff;
    };
    const auto nop = [](M &) {};
    for(int sense : {-1, 1}) for(double off : {0.0, 100.0, -12345.5}) {
        cgm G = make_cg(4, 60, 10);
        G.r.offset = off;
        if(sense > 0) { // minimize -c'x, same model, opposite sign
            G.r.sense = 1; for(auto & v : G.r.obj) v = -v; }
        check("cg60x10 covers", G.r, 1, nop, true, &G);
    }
    { cgm G = make_cg(4, 60, 10); G.r.offset = 100; check("cg60x10 covers default threads", G.r, 8, nop, true, &G); }
    // Default presolve removing columns with costs: fixed, dual-fixed, aggregated, parallel.
    { cgm G = make_cg(5, 60, 10); rawm r = G.r; r.offset = 77;
      int f = r.var(3, 1, 1, 'B'); (void)f;                       // fixed with cost
      int w = r.var(7, 0, 4, 'C'); (void)w;                       // dual-fixed at 4 with cost
      int a = r.bin(11), ab = r.bin(0); r.add({a, ab}, {1, 1}, '=', 1); // aggregated
      int p1 = r.var(9, 0, 3, 'I'), p2 = r.var(9, 0, 3, 'I'); { auto i = G.x; i.push_back(p1); i.push_back(p2); std::vector<double> v(G.x.size(), 1.0); v.push_back(5); v.push_back(5); r.add(i, v, '<', 30); }
      check("cg60x10 + removed columns with costs", r, 1, nop, true, &G);
      check("same, CutPasses=0 Cuts=0 (root relaxation vs log)", r, 1, [](M & m) { seti(m, "CutPasses", 0); seti(m, "Cuts", 0); }, false, &G); }
    // The value on a user model with a pending change: Obj reads the updated attribute.
    { M m; const cgm G = make_cg(4, 60, 10); seti(m, "OutputFlag", 0); seti(m, "Threads", 1); G.r.load(m); auto [env, um] = m.native_model(); m.native_api().setdblattrelement(um, "Obj", 0, 999.0); double o = 0; m.native_api().getdblattrelement(um, "Obj", 0, &o); std::println(stderr, "pending change: Obj[0] read back {} before update, true {}", o, G.r.obj[0]); }
}

// ======================================================================= Q3
// Preloading: cuts satisfied at the first call and violated later. Offered through the
// recipe with the drop, and without it.
static void x_q3() {
    for(unsigned seed : {1u, 3u, 9u}) for(int mode : {0, 1, 2}) {
        const int n = 26;
        M m; seti(m, "OutputFlag", 0); seti(m, "Threads", 1); setd(m, "TimeLimit", 30); recipe(m);
        auto a = cp::build_atsp(m, n, false, seed);
        std::vector<raw_cut> pre;
        for(int i = 0; i < n; ++i) for(int j = i + 1; j < n; ++j) pre.push_back(raw_cut{{a.col[static_cast<size_t>(i)][static_cast<size_t>(j)], a.col[static_cast<size_t>(j)][static_cast<size_t>(i)]}, {1.0, 1.0}, 'L', 1.0});
        hctx c; c.drop_satisfied = mode != 1; bool first = true; long ck = 0, vi = 0, sat_first = 0;
        c.on_node = [&](hctx & h, const std::vector<double> & x) {
            if(first) { first = false; for(const auto & k : pre) { sat_first += k.violation(x) <= 1e-9; if(mode != 2) h.add(k); } return; }
            for(const auto & k : pre) { ++ck; vi += k.violation(x) > 1e-6; }
        };
        attach(m, c);
        const sol s = solve_native(m, false);
        const char * nm[] = {"offered, drop satisfied", "offered, no drop", "not offered"};
        std::println(stderr, "preload seed={} {:24}: {} first-call satisfied={}/{} cuts_passed={} later 2-cycle viol/checks={}/{}", seed, nm[mode], line(s), sat_first, pre.size(), c.cuts, vi, ck);
    }
}

// ======================================================================= controls
static void x_ctl() {
    // Filter removed: what REL returns at status != 2, and how many fires it is.
    for(auto [n, seed] : {std::pair{44, 9u}, std::pair{30, 1u}}) {
        M m; seti(m, "OutputFlag", 0); seti(m, "Threads", 1); setd(m, "TimeLimit", 30); recipe(m);
        auto a = cp::build_atsp(m, n, false, seed);
        hctx c; c.filter = false; std::map<int, long> garbage, clean; double sample = NAN; int sample_status = 0;
        c.on_node = [&](hctx & h, const std::vector<double> & x) {
            const bool bad = std::ranges::any_of(x, [](double v) { return v > 1e20 || v == -7.0; });
            if(bad) { sample = x[0]; sample_status = h.status; }
            ++(bad ? garbage : clean)[h.status];
            for(const auto & k : a.separate(x)) h.add(k);
        };
        attach(m, c);
        const sol s = solve_native(m, false);
        std::string gs, cs; for(auto [k, v] : garbage) gs += std::format(" {}:{}", k, v); for(auto [k, v] : clean) cs += std::format(" {}:{}", k, v);
        std::println(stderr, "no filter atsp n={} seed={}: {} | REL garbage by status{{{} }} clean by status{{{} }} rel_err={} sample value x[0]={:g} at status {} cuts={} native_err={}", n, seed, line(s), gs, cs, c.rel_err, sample, sample_status, c.cuts, c.native_err);
    }
    // cbcut outside MIPNODE: the recipe's where.
    { M m; seti(m, "OutputFlag", 0); seti(m, "Threads", 1); const cgm G = make_cg(3); recipe(m); G.r.load(m); hctx c; long errs = 0, calls = 0; int code = 0;
      c.on_sol = [&](hctx & h, const std::vector<double> &, double) { ++calls; int i = G.x[0]; double v = 1; const int e = h.api->cbcut(h.cbdata, 1, &i, &v, '<', 1.0); if(e) { ++errs; code = e; } };
      attach(m, c); const sol s = solve_native(m, false); std::println(stderr, "cbcut from MIPSOL: {} calls={} errors={} code={}", line(s), calls, errs, code); }
    // Separator that trusts x: rounds up fractional columns of the point into a "fix" cut. Garbage at status 6
    // reaches the user's code without the filter: bad point count.
}

int main(int argc, char ** argv) {
    const std::string w = arg(argc, argv, 1, "");
    { M p; print_release(p); }
    if(w == "g1") x_g1();
    else if(w == "g2") x_g2();
    else if(w == "g3") x_g3();
    else if(w == "g4") x_g4(static_cast<unsigned>(std::atoi(arg(argc, argv, 2, "6"))));
    else if(w == "g5") x_g5();
    else if(w == "g6") x_g6();
    else if(w == "g7") x_g7();
    else if(w == "g8") x_g8();
    else if(w == "g9") x_g9();
    else if(w == "g9b") x_g9b();
    else if(w == "g9c") x_g9c();
    else if(w == "q2") x_q2();
    else if(w == "q3") x_q3();
    else if(w == "ctl") x_ctl();
    else { std::println(stderr, "unknown {}", w); return 2; }
    return 0;
}
