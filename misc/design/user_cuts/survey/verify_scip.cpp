// Skeptic pass, SCIP: one subcommand per claim. Own hook (verify_scip_native.hpp),
// own instrumentation. Results on stderr.
#include "verify_scip_native.hpp"
#include <unistd.h>
#include <sys/wait.h>
using namespace vs;
using cp::raw_cut;
using G_t = cp::gadgets<V>;

static void scope(V & m, const bool allow_restarts = false) {
    natives & n = m.N(); void * s = m.sp();
    n.setPresolving(s, 3, 1); n.setHeuristics(s, 3, 1); n.setSeparating(s, 3, 1);
    m.seti("presolving/maxrestarts", allow_restarts ? -1 : 0);
    m.seti("misc/usesymmetry", 0);
    n.setBoolParam(s, "conflict/enable", 0);
}
static void select_node(V & m, const std::string & sel) {
    if(sel == "default") return;
    if(sel == "dfsup" || sel == "dfsdown") {
        m.seti("nodeselection/dfs/stdpriority", 1000000);
        m.N().setCharParam(m.sp(), "nodeselection/childsel", sel == "dfsup" ? 'u' : 'd');
        return;
    }
    if(sel == "defaultup" || sel == "defaultdown") {
        m.N().setCharParam(m.sp(), "nodeselection/childsel", sel == "defaultup" ? 'u' : 'd');
        return;
    }
    if(sel == "bfsnoplunge") {
        m.seti("nodeselection/bfs/maxplungedepth", 0);
        m.seti("nodeselection/bfs/minplungedepth", 0);
        return;
    }
    m.seti(std::format("nodeselection/{}/stdpriority", sel).c_str(), 1000000);
}
static std::string st(V & m) { return cp::status_name(m.get_status()); }
static double val(V & m) { return cf::value_of(m); }
static double solved_val(V & m, double limit = 60) {
    m.set_time_limit(std::chrono::duration<double>(limit));
    m.solve();
    return val(m);
}

// Reads a captured row's age at the end of every node.
struct track {
    natives * n = nullptr;
    ROW * row = nullptr, * row2 = nullptr;
    int age_end = -99, age2_end = -99; bool inlp_end = false, inlp2_end = false;
    long events = 0;
    int add_depth = -1;
    long sub_checks = 0, sub_viol = 0, sib_checks = 0, sib_viol = 0, sib_inlp = 0;
    bool fs_done = false; int fs_age = -99, fs_age2 = -99; bool fs_inlp_end = false, fs_inlp = false, fs_inlp2 = false; double fs_viol = 0;
    bool row_local = false;
    static int exec(void *, EVENTHDLR *, EVENT *, void * data) {
        auto & t = *static_cast<track *>(data);
        ++t.events;
        if(t.row) { t.age_end = t.n->rowGetAge(t.row); t.inlp_end = t.n->rowIsInLP(t.row); }
        if(t.row2) { t.age2_end = t.n->rowGetAge(t.row2); t.inlp2_end = t.n->rowIsInLP(t.row2); }
        return RC_OKAY;
    }
    void arm(V & m) {
        n = &m.N();
        EVENTHDLR * h = nullptr;
        void * s = m.sp();
        if(n->includeEventhdlrBasic(s, &h, "vtrack", "tracks a row", exec, nullptr) != RC_OKAY) std::exit(4);
        if(n->transformProb(s) != RC_OKAY) std::exit(4);
        const unsigned long long mask = n->majorVersion() < 10 ? (0x80000ull | 0x100000ull | 0x200000ull) : (0x100000ull | 0x200000ull | 0x400000ull);
        if(n->catchEvent(s, mask, h, this, nullptr) != RC_OKAY) std::exit(4);
    }
    void drop(V & m) {
        if(row) n->releaseRow(m.sp(), &row);
        if(row2) n->releaseRow(m.sp(), &row2);
    }
};

// ------------------------------------------------------------------ S1, S2 ---
struct s12_out { double value; long long nodes; int fs_age; bool fs_inlp; bool have_fs; bool kept; };
static s12_out s12_run(const std::string & sel, const bool local, const bool flip, const int k, const bool relaxed_scope = false, const bool quiet = false) {
    G() = recipe{}; V m;
    // presolve and separators off, or the gadgets close at the root; heuristics, conflict
    // analysis, restarts and symmetry stay at their defaults
    if(relaxed_scope) { m.N().setSeparating(m.sp(), 3, 1); m.N().setPresolving(m.sp(), 3, 1); }
    else scope(m);
    select_node(m, sel);
    auto g = cp::build_gadgets(m, k, flip);
    track t; t.arm(m);
    const raw_cut cut = g.invalid_global_cut();
    m.attach_custom([&](plugin & p, const std::vector<double> & x, int depth) {
        const int state = g.state(x);
        if(!t.row) {
            if(state == g.worse_state() && depth >= 1 && cut.violation(x) > 1e-4) {
                recipe rr; rr.local = local; rr.add_pool = false;
                p.add(cut, x, rr, &t.row);
                t.add_depth = depth;
                t.row_local = t.row && p.n->rowIsLocal(t.row);
            }
            return;
        }
        const bool v = cut.violation(x) > 1e-6;
        if(state == g.worse_state()) { ++t.sub_checks; t.sub_viol += v; }
        else if(state == 1 - g.worse_state()) {
            ++t.sib_checks; t.sib_viol += v; t.sib_inlp += p.n->rowIsInLP(t.row);
            if(!t.fs_done) { t.fs_done = true; t.fs_age = t.age_end; t.fs_inlp_end = t.inlp_end; t.fs_inlp = p.n->rowIsInLP(t.row); t.fs_viol = cut.violation(x); }
        }
    });
    const double v = solved_val(m, 30);
    const bool kept = std::fabs(v - g.opt) < 1e-4;
    if(!quiet) std::println(stderr, "S1/2 v{} sel={} local={} flip={} k={} scope={}: status={} value={:.6g} opt={:.6g} {} nodes={} add_depth={} row_local={} sub viol/checks={}/{} sib viol/checks={}/{} sib_inLP={} first_sib(age_at_jump={} inLP_at_jump={} inLP={} viol={:.4g}) events={}",
        m.version(), sel, local, flip, k, relaxed_scope ? "relaxed" : "off", st(m), v, g.opt, kept ? "KEPT" : "LOST", m.N().getNTotalNodes(m.sp()), t.add_depth, t.row_local, t.sub_viol, t.sub_checks, t.sib_viol, t.sib_checks, t.sib_inlp, t.fs_age, t.fs_inlp_end, t.fs_inlp, t.fs_viol, t.events);
    s12_out o{v, m.N().getNTotalNodes(m.sp()), t.fs_age, t.fs_inlp, t.fs_done, kept};
    t.drop(m);
    return o;
}
static void cmd_s1() {
    for(const char * sel : {"default", "breadthfirst", "bfsnoplunge", "dfs", "dfsup", "dfsdown", "estimate", "hybridestim", "restartdfs", "uct"})
        for(bool local : {true, false}) for(bool flip : {false, true}) s12_run(sel, local, flip, 10);
    for(const char * sel : {"default", "breadthfirst", "dfsup", "dfsdown", "uct"}) for(bool local : {true, false}) for(bool flip : {false, true}) s12_run(sel, local, flip, 10, true);
}
static void cmd_s2_table() {
    long tab[2][2] = {{0, 0}, {0, 0}};  // [age==0][inLP]
    long lost[2] = {0, 0}, kept[2] = {0, 0};
    long none = 0;
    for(const char * sel : {"default", "defaultup", "defaultdown", "breadthfirst", "bfsnoplunge", "dfsup", "dfsdown", "estimate", "hybridestim", "restartdfs", "uct"})
        for(int k : {6, 7, 8, 9, 10, 11, 12}) for(bool flip : {false, true}) {
            const auto o = s12_run(sel, false, flip, k, false, true);
            if(!o.have_fs) { ++none; continue; }
            const bool a0 = o.fs_age == 0;
            tab[a0][o.fs_inlp]++;
            (o.kept ? kept : lost)[o.fs_inlp]++;
            std::println(stderr, "   table sel={} k={} flip={} age_at_jump={} inLP_first_sib={} {} value={:.6g} nodes={}", sel, k, flip, o.fs_age, o.fs_inlp, o.kept ? "KEPT" : "LOST", o.value, o.nodes);
        }
    std::println(stderr, "S2 table: age=0&inLP={} age=0&absent={} age>0&inLP={} age>0&absent={} (no sibling seen={})", tab[1][1], tab[1][0], tab[0][1], tab[0][0], none);
    std::println(stderr, "S2 outcome: inLP KEPT={} LOST={}; absent KEPT={} LOST={}", kept[1], lost[1], kept[0], lost[0]);
}
// A (Sum z <= opt-.5), then in the next round something dominating it, so that A ends
// the node slack: age >= 1. A later row B is either global or local.
static void s2ab(const std::string & sel, const bool b_local, const bool flip) {
    G() = recipe{}; V m; scope(m); select_node(m, sel);
    auto g = cp::build_gadgets(m, 10, flip);
    track t; t.arm(m);
    raw_cut A = g.invalid_global_cut(); A.rhs = g.opt - 0.5;
    raw_cut B = g.invalid_global_cut(); B.rhs = g.opt - 1.0;
    long long at = -1;
    bool done = false;
    long sibs = 0, sib_A = 0, sib_B = 0;
    m.attach_custom([&](plugin & p, const std::vector<double> & x, int depth) {
        const int state = g.state(x);
        if(!t.row && state == g.worse_state() && depth >= 1 && A.violation(x) > 1e-4) {
            recipe rr; rr.add_pool = false;
            p.add(A, x, rr, &t.row);
            at = p.n->nodeGetNumber(p.n->getCurrentNode(p.s));
            return;
        }
        if(t.row && !t.row2 && p.n->nodeGetNumber(p.n->getCurrentNode(p.s)) == at && B.violation(x) > 1e-4) {
            recipe rr; rr.local = b_local; rr.add_pool = false;
            p.add(B, x, rr, &t.row2);
            return;
        }
        if(t.row2 && state == 1 - g.worse_state()) {
            ++sibs; sib_A += p.n->rowIsInLP(t.row); sib_B += p.n->rowIsInLP(t.row2);
            if(!done) { done = true; t.fs_age = t.age_end; t.fs_age2 = t.age2_end; t.fs_inlp = p.n->rowIsInLP(t.row); t.fs_inlp2 = p.n->rowIsInLP(t.row2); t.fs_viol = A.violation(x); }
        }
    });
    const double v = solved_val(m, 30);
    std::println(stderr, "S2ab v{} sel={} B_local={} flip={}: value={:.6g} opt={:.6g} {} nodes={} | first sibling: A(age_at_jump={} inLP={} viol={:.4g}) B(age_at_jump={} inLP={}) | sib calls={} A_inLP={} B_inLP={}",
        m.version(), sel, b_local, flip, v, g.opt, std::fabs(v - g.opt) < 1e-4 ? "KEPT" : "LOST", m.N().getNTotalNodes(m.sp()), t.fs_age, t.fs_inlp, t.fs_viol, t.fs_age2, t.fs_inlp2, sibs, sib_A, sib_B);
    t.drop(m);
}
static void cmd_s2ab() {
    for(const char * sel : {"breadthfirst", "default"}) for(bool bl : {false, true}) for(bool flip : {false, true}) s2ab(sel, bl, flip);
}

// A row flagged local, added at the root.
static void cmd_s1root() {
    for(bool flip : {false, true}) for(const char * sel : {"default", "breadthfirst"}) {
        G() = recipe{}; V m; scope(m); select_node(m, sel);
        auto g = cp::build_gadgets(m, 10, flip);
        const raw_cut cut = g.invalid_global_cut();
        ROW * row = nullptr; bool before = false, after = false; long sib_viol = 0, sib = 0;
        m.attach_custom([&](plugin & p, const std::vector<double> & x, int depth) {
            if(!row) {
                if(depth == 0 && cut.violation(x) > 1e-4) {
                    recipe rr; rr.local = true; rr.add_pool = false;
                    p.add(cut, x, rr, &row);
                    before = true; after = p.n->rowIsLocal(row);
                }
                return;
            }
            ++sib; sib_viol += cut.violation(x) > 1e-6;
        });
        const double v = solved_val(m, 30);
        std::println(stderr, "S1root v{} sel={} flip={}: value={:.6g} opt={:.6g} {} nodes={} row_is_local_after_SCIPaddRow={} viol/calls_after={}/{}", m.version(), sel, flip, v, g.opt, std::fabs(v - g.opt) < 1e-4 ? "KEPT" : "LOST", m.N().getNTotalNodes(m.sp()), after, sib_viol, sib);
        if(row) m.N().releaseRow(m.sp(), &row);
    }
}

// A local row that is infeasible for the local bounds runs conflict analysis
// (scip_cut.c:266). Does the conflict leak out of the subtree the row is valid in?
static void cmd_s1c_run(const bool local, const bool two_term, const bool conflict) {
    G() = recipe{}; V m;
    natives & n = m.N(); void * s = m.sp();
    n.setPresolving(s, 3, 1); n.setHeuristics(s, 3, 1); n.setSeparating(s, 3, 1);
    m.seti("presolving/maxrestarts", 0); m.seti("misc/usesymmetry", 0);
    n.setBoolParam(s, "conflict/enable", conflict);
    // gadget 0 (big): y0 = 0 is better by 10, gadget 1: y1 = 1 is better by 2, the optimum needs it
    const std::size_t K = 8;
    auto y = m.add_binary_variables(K), za = m.add_variables(K), zb = m.add_variables(K);
    auto yb = m.add_binary_variables(3), zab = m.add_variables(3), zbb = m.add_variables(3);
    const auto Is = std::views::iota(std::size_t{0}, K);
    const auto Js = std::views::iota(std::size_t{0}, std::size_t{3});
    m.set_maximization();
    m.set_objective(xsum(Is, [&](std::size_t i) { return za(i) + zb(i); }) + xsum(Js, [&](std::size_t j) { return zab(j) + zbb(j); }));
    double opt = 0;
    for(std::size_t i : Is) {
        const double c = i == 0 ? 5.0 : 1.0 + 0.03 * static_cast<double>(i);
        const double va0 = i == 0 ? 5 : 3;
        const double vb1 = i == 1 ? 5 : 3;
        m.add_constraint(za(i) - (c * 30) * y(i) <= c * va0);
        m.add_constraint(za(i) + (c * (va0 + 3)) * y(i) <= c * (va0 + 4));
        m.add_constraint(zb(i) + (c * 30) * y(i) <= c * (vb1 + 30));
        m.add_constraint(zb(i) - (c * (vb1 + 3)) * y(i) <= c * 1);
        opt += c * std::max(va0 + 1, 1 + vb1);
    }
    for(std::size_t j : Js) {
        const double c = 3;
        m.add_constraint(zab(j) - (c * 30) * yb(j) <= c * 3);
        m.add_constraint(zab(j) + (c * 6) * yb(j) <= c * 7);
        m.add_constraint(zbb(j) + (c * 30) * yb(j) <= c * 33);
        m.add_constraint(zbb(j) - (c * 6) * yb(j) <= c * 1);
        m.add_constraint(yb(j) - y(std::size_t{0}) <= 0);
        opt += c * 4;
    }
    const int y0 = m.native_id(y(std::size_t{0})), y1 = m.native_id(y(std::size_t{1})), y2 = m.native_id(y(std::size_t{2}));
    bool added = false; long flags_at_add = 0; int add_depth = -1;
    double ub_global_after = NAN; SCIP_VAR * tv = nullptr;
    long y1_up_calls_after = 0, y0_zero_y1_up_after = 0;
    m.attach_custom([&](plugin & p, const std::vector<double> & x, int depth) {
        SCIP_VAR * t0 = nullptr, * t1 = nullptr;
        p.n->getTransformedVar(p.s, m.var_of(y0), &t0);
        p.n->getTransformedVar(p.s, m.var_of(y1), &t1);
        const bool in_worse = p.n->varGetLbLocal(t0) > 0.5, y1_up = p.n->varGetLbLocal(t1) > 0.5;
        if(!added) {
            if(in_worse && y1_up) {
                recipe rr; rr.local = local; rr.add_pool = false; rr.drop_satisfied = false; rr.force = false;
                const raw_cut cut = two_term ? raw_cut{{y1, y2}, {1.0, 1.0}, 'L', 0.0} : raw_cut{{y1}, {1.0}, 'L', 0.0};
                p.add(cut, x, rr);
                added = true; flags_at_add = p.infeasible_flags; add_depth = depth; tv = t1;
            }
            return;
        }
        if(y1_up) { ++y1_up_calls_after; y0_zero_y1_up_after += !in_worse; }
        ub_global_after = p.n->varGetUbGlobal(t1);
    });
    const double v = solved_val(m, 30);
    std::println(stderr, "S1c v{} local={} terms={} conflict={}: status={} value={:.6g} opt={:.6g} {} nodes={} added={} add_depth={} infeasible_flag={} conflicts_found={} applied={} after: y1_ub_global={} calls_with_y1=1:{} of_which_in_sibling:{}",
        m.version(), local, two_term ? 2 : 1, conflict, st(m), v, opt, std::fabs(v - opt) < 1e-4 ? "KEPT" : "LOST", n.getNTotalNodes(s), added, add_depth, flags_at_add, n.getNConflictConssFound(s), n.getNConflictConssApplied(s), ub_global_after, y1_up_calls_after, y0_zero_y1_up_after);
}
static void cmd_s1c() {
    for(bool conflict : {true, false}) for(bool two : {false, true}) for(bool local : {true, false}) cmd_s1c_run(local, two, conflict);
}

// ---------------------------------------------------------------------- S3 ---
static void s3_run(const std::string & label, recipe r, const bool atsp) {
    G() = r; V m;
    long long nodes;
    if(atsp) {
        auto a = cp::build_atsp(m, 16, false, 2);
        m.set_time_limit(std::chrono::duration<double>(60));
        m.attach([](const std::vector<double> &, const cp::add_fn &) {});
        m.solve();
    } else {
        scope(m);
        auto g = cp::build_gadgets(m, 10, false);
        m.set_time_limit(std::chrono::duration<double>(30));
        m.attach([](const std::vector<double> &, const cp::add_fn &) {});
        m.solve();
    }
    nodes = m.N().getNTotalNodes(m.sp());
    std::string d;
    for(auto [k, c] : m.P->calls_by_depth) if(k <= 18) d += std::format(" {}:{}", k, c);
    std::println(stderr, "S3 v{} {} {}: nodes={} hook_calls={} distinct_nodes={} sepa_ncalls={} by_depth{}", m.version(), atsp ? "atsp16" : "gadgets10", label, nodes, m.P->entries, m.P->nodes_called.size(), m.N().sepaGetNCalls(m.P->sepa), d);
}
static void cmd_s3() {
    for(bool atsp : {false, true}) {
        recipe r;
        s3_run("recipe(freq=1,eb=1,mbd=1)", r, atsp);
        { recipe q = r; q.expbackoff = 0; s3_run("eb=default", q, atsp); }
        { recipe q = r; q.maxbounddist = 0; s3_run("mbd=0", q, atsp); }
        { recipe q = r; q.expbackoff = 0; q.maxbounddist = 0; s3_run("eb=default,mbd=0", q, atsp); }
        { recipe q = r; q.freq = 0; s3_run("freq=0", q, atsp); }
        { recipe q = r; q.freq = 3; s3_run("freq=3", q, atsp); }
        { recipe q = r; q.priority = -1000000; s3_run("prio=-1e6", q, atsp); }
    }
}

// ---------------------------------------------------------------------- S4 ---
static void s4_run(const std::string & where, const int poolfreq, const bool invalid) {
    G() = recipe{}; V m; scope(m);
    if(poolfreq >= 0) m.seti("separating/poolfreq", poolfreq);
    auto g = cp::build_gadgets(m, 12, false);
    const raw_cut cut = invalid ? g.invalid_global_cut() : g.hull_cut(3);
    ROW * row = nullptr; bool added = false; int add_depth = -1;
    struct per { long calls = 0, inlp = 0, viol = 0; };
    std::map<int, per> D;
    int first_lp_depth = -1;
    m.attach_custom([&](plugin & p, const std::vector<double> & x, int depth) {
        if(!added) {
            const bool ok = where == "root" ? depth == 0 : depth >= 1 && g.state(x) == g.worse_state();
            if(ok && cut.violation(x) > 1e-4) {
                recipe rr; rr.add_row = false; rr.add_pool = true; rr.pool_guard = false;
                p.add(cut, x, rr, &row);
                added = true; add_depth = depth;
            }
            return;
        }
        auto & e = D[depth];
        ++e.calls;
        const bool in = p.n->rowIsInLP(row);
        e.inlp += in; e.viol += cut.violation(x) > 1e-6;
        if(in && first_lp_depth < 0) first_lp_depth = depth;
    });
    const double v = solved_val(m, 40);
    std::string d;
    for(auto & [k, e] : D) d += std::format(" d{}:{}/{}/{}", k, e.calls, e.inlp, e.viol);
    std::println(stderr, "S4 v{} add_at={} poolfreq={} cut={}: value={:.6g} opt={:.6g} nodes={} add_depth={} first_depth_in_LP={} applied_total={} (depth:calls/inLP/violated){}",
        m.version(), where, poolfreq, invalid ? "invalid" : "hull", v, g.opt, m.N().getNTotalNodes(m.sp()), add_depth, first_lp_depth, m.N().sepaGetNCutsApplied(m.P->sepa), d);
    if(row) m.N().releaseRow(m.sp(), &row);
}
static void cmd_s4() {
    { G() = recipe{}; V m; scope(m); auto g = cp::build_gadgets(m, 12, false); const double v = solved_val(m, 40); std::println(stderr, "S4 v{} no hook: value={:.6g} nodes={}", m.version(), v, m.N().getNTotalNodes(m.sp())); }
    for(const char * w : {"depth", "root"}) for(int pf : {-1, 1}) for(bool inv : {false, true}) s4_run(w, pf, inv);
}

// ---------------------------------------------------------------------- S5 ---
static void s5_run(const bool local, const bool flip, const std::string & sel) {
    G() = recipe{}; V m; scope(m); select_node(m, sel);
    auto g = cp::build_gadgets(m, 10, flip);
    const int col = g.za[3];
    const raw_cut cut{{col}, {1.0}, 'L', 1.0};
    bool added = false;
    double ub_before = 0, ub_after = NAN, ubl_after = NAN; int add_depth = -1; long long add_node = -1;
    long sib = 0, sib_at = 0, sub = 0, sub_at = 0; bool read_after = false;
    SCIP_VAR * tv = nullptr;
    m.attach_custom([&](plugin & p, const std::vector<double> & x, int depth) {
        const int state = g.state(x);
        if(!added) {
            if(state == g.worse_state() && depth >= 1 && cut.violation(x) > 1e-4) {
                p.n->getTransformedVar(p.s, m.var_of(col), &tv);
                ub_before = p.n->varGetUbGlobal(tv);
                recipe rr; rr.local = local;
                p.add(cut, x, rr);
                added = true; add_depth = depth; add_node = p.n->nodeGetNumber(p.n->getCurrentNode(p.s));
            }
            return;
        }
        if(!read_after) { read_after = true; ub_after = p.n->varGetUbGlobal(tv); ubl_after = p.n->varGetUbLocal(tv); }
        const bool at = cut.violation(x) <= 1e-6;
        if(state == g.worse_state()) { ++sub; sub_at += at; }
        else if(state == 1 - g.worse_state()) { ++sib; sib_at += at; }
    });
    const double v = solved_val(m, 30);
    std::println(stderr, "S5 v{} sel={} local={} flip={}: value={:.6g} opt={:.6g} {} nodes={} add_depth={} node={} global_ub before={} next_call={} (local {}) subtree holds {}/{} sibling holds {}/{}",
        m.version(), sel, local, flip, v, g.opt, std::fabs(v - g.opt) < 1e-4 ? "KEPT" : "LOST", m.N().getNTotalNodes(m.sp()), add_depth, add_node, ub_before, ub_after, ubl_after, sub_at, sub, sib_at, sib);
}
static void cmd_s5() {
    for(const char * sel : {"default", "breadthfirst"}) for(bool local : {false, true}) for(bool flip : {false, true}) s5_run(local, flip, sel);
}

// ---------------------------------------------------------------------- S6 ---
struct s6_spec { std::string name; bool add_row, add_pool, local; bool at_root; bool invalid; int wait_age = -1; int poolfreq = -1; };
static void s6_run(const s6_spec & sp, const bool flip) {
    G() = recipe{}; V m; scope(m, true);
    if(sp.poolfreq >= 0) m.seti("separating/poolfreq", sp.poolfreq);
    auto g = cp::build_gadgets(m, 10, flip);
    const raw_cut cut = sp.invalid ? g.invalid_global_cut() : g.hull_cut(3);
    ROW * row = nullptr; bool added = false, restarted = false; int conss0 = -1, conss1 = -1, row_age = -9, cut_age = -9; bool row_inlp = false; double first_viol = NAN; bool first_run2 = false; int add_depth = -1;
    long run2_inlp = 0;
    m.attach_custom([&](plugin & p, const std::vector<double> & x, int depth) {
        natives & n = *p.n;
        const int run = n.getNRuns(p.s);
        if(run >= 2) { if(!first_run2) { first_run2 = true; conss1 = n.getNConss(p.s); first_viol = cut.violation(x); } return; }
        if(!added) {
            const bool ok = sp.at_root ? depth == 0 : depth >= 1 && g.state(x) == g.worse_state();
            if(ok && cut.violation(x) > 1e-4) {
                recipe rr; rr.add_row = sp.add_row; rr.add_pool = sp.add_pool; rr.pool_guard = false; rr.local = sp.local; 
                p.add(cut, x, rr, &row);
                added = true; add_depth = depth;
                conss0 = n.getNConss(p.s);
            }
            return;
        }
        if(!restarted) {
            const int age = n.rowGetAge(row);
            if(sp.wait_age >= 0 && age < sp.wait_age) return;
            row_age = age; row_inlp = n.rowIsInLP(row);
            for(int i = 0, c = n.getNPoolCuts(p.s); i < c; ++i) if(n.cutGetRow(n.getPoolCuts(p.s)[i]) == row) cut_age = n.cutGetAge(n.getPoolCuts(p.s)[i]);
            restarted = true;
            n.restartSolve(p.s);
        }
    });
    const double v = solved_val(m, 30);
    std::println(stderr, "S6 v{} {} flip={}: status={} value={:.6g} opt={:.6g} {} runs={} nodes={} add_depth={} at_restart(row_inLP={} row_age={} pool_age={}) conss {}->{} first_run2_violation={:.4g}",
        m.version(), sp.name, flip, st(m), v, g.opt, std::fabs(v - g.opt) < 1e-4 ? "KEPT" : "LOST", m.N().getNRuns(m.sp()), m.N().getNTotalNodes(m.sp()), add_depth, row_inlp, row_age, cut_age, conss0, conss1, first_viol);
    if(row) m.N().releaseRow(m.sp(), &row);
    (void)run2_inlp;
}
static void cmd_s6() {
    const std::vector<s6_spec> specs = {
        {"hull root row", true, false, false, true, false},
        {"hull root row+pool", true, true, false, true, false},
        {"hull root pool", false, true, false, true, false},
        {"hull depth row", true, false, false, false, false},
        {"hull depth local", true, false, true, false, false},
        {"hull depth row+pool", true, true, false, false, false},
        {"hull depth pool", false, true, false, false, false},
        {"hull depth pool freq1", false, true, false, false, false, -1, 1},
        {"invalid depth row", true, false, false, false, true},
        {"invalid depth local", true, false, true, false, true},
        {"invalid depth row+pool", true, true, false, false, true},
        {"invalid depth pool", false, true, false, false, true},
        {"invalid depth row+pool age>=2", true, true, false, false, true, 2},
        {"hull depth row+pool age>=2", true, true, false, false, false, 2},
    };
    for(const auto & sp : specs) for(bool flip : {false, true}) s6_run(sp, flip);
}
// takeCut reads the POOL age (SCIPcutGetAge), not the LP age: A sits in the LP slack, with
// row age >= 1 and pool age 0, because a later row B dominates it.
static void cmd_s6age_run(const bool flip) {
    G() = recipe{}; V m; scope(m, true);
    auto g = cp::build_gadgets(m, 10, flip);
    raw_cut A = g.invalid_global_cut(); A.rhs = g.opt - 0.5;
    raw_cut B = g.invalid_global_cut(); B.rhs = g.opt - 1.0;
    ROW * ra = nullptr, * rb = nullptr; bool restarted = false; int conss0 = -1, conss1 = -1, age = -9, pool_age = -9, b_age = -9; bool inlp = false, b_inlp = false; long long at = -1;
    m.attach_custom([&](plugin & p, const std::vector<double> & x, int depth) {
        natives & n = *p.n;
        if(n.getNRuns(p.s) >= 2) { if(conss1 < 0) conss1 = n.getNConss(p.s); return; }
        const long long node = n.nodeGetNumber(n.getCurrentNode(p.s));
        if(!ra) {
            if(depth >= 1 && g.state(x) == g.worse_state() && A.violation(x) > 1e-4) { recipe rr; p.add(A, x, rr, &ra); at = node; conss0 = n.getNConss(p.s); }
            return;
        }
        if(!rb) {
            if(node == at && B.violation(x) > 1e-4) { recipe rr; rr.add_pool = false; p.add(B, x, rr, &rb); }
            return;
        }
        if(!restarted && n.rowIsInLP(ra) && n.rowGetAge(ra) >= 1) {
            restarted = true;
            age = n.rowGetAge(ra); inlp = true; b_age = n.rowGetAge(rb); b_inlp = n.rowIsInLP(rb);
            for(int i = 0, c = n.getNPoolCuts(p.s); i < c; ++i) if(n.cutGetRow(n.getPoolCuts(p.s)[i]) == ra) pool_age = n.cutGetAge(n.getPoolCuts(p.s)[i]);
            n.restartSolve(p.s);
        }
    });
    const double v = solved_val(m, 30);
    std::println(stderr, "S6age v{} flip={}: status={} value={:.6g} opt={:.6g} runs={} restart asked={} A(inLP={} row_age={} pool_age={}) B(no pool, inLP={} row_age={}) conss {}->{}",
        m.version(), flip, st(m), v, g.opt, m.N().getNRuns(m.sp()), restarted, inlp, age, pool_age, b_inlp, b_age, conss0, conss1);
    if(ra) m.N().releaseRow(m.sp(), &ra);
    if(rb) m.N().releaseRow(m.sp(), &rb);
}
static void cmd_s6age() { for(bool flip : {false, true}) cmd_s6age_run(flip); }

// P across a restart: the converted constraint must die with the transform.
static void cmd_s6p() {
    G() = recipe{}; V m; scope(m, true);
    auto g = cp::build_gadgets(m, 10, false);
    const raw_cut cut = g.hull_cut(3);
    bool added = false, restarted = false; int c1 = -1, c2 = -1;
    m.attach_custom([&](plugin & p, const std::vector<double> & x, int depth) {
        natives & n = *p.n;
        if(n.getNRuns(p.s) >= 2) { if(c2 < 0) c2 = n.getNConss(p.s); return; }
        if(!added) { if(depth == 0 && cut.violation(x) > 1e-4) { recipe rr; p.add(cut, x, rr); added = true; c1 = n.getNConss(p.s); } return; }
        if(!restarted) { restarted = true; n.restartSolve(p.s); }
    });
    const double v = solved_val(m, 30);
    std::println(stderr, "S6P v{} first solve: value={:.6g} opt={:.6g} runs={} conss {}->{} wrapper num_constraints={}", m.version(), v, g.opt, m.N().getNRuns(m.sp()), c1, c2, m.num_constraints());
    for(std::size_t r = 12; r < 16; ++r) m.set_constraint_rhs(g.rows[r], m.get_constraint_rhs(g.rows[r]) + 50);
    m.P->attached = false; m.P->custom = {};
    const double w = solved_val(m, 30);
    std::println(stderr, "S6P re-solve after raising gadget 3 by 50: value={:.6g} want={:.6g} {} conss_now={}", w, g.opt + 100, std::fabs(w - (g.opt + 100)) < 1e-4 ? "cut did NOT persist" : "PERSISTED", m.N().getNConss(m.sp()));
}

// ---------------------------------------------------------------------- S7 ---
static int noop_exec(void *, SEPA *, SCIP_RESULT * r, unsigned, int) { *r = sc::SCIP_DIDNOTRUN; return RC_OKAY; }
static void cmd_s7() {
    recipe r; r.no_include = true; G() = r;
    {
        V m; scope(m); auto g = cp::build_gadgets(m, 6, false);
        solved_val(m);
        SEPA * late = nullptr;
        const int stage = m.N().getStage(m.sp());
        const int rc = m.N().includeSepaBasic(m.sp(), &late, "late", "x", 0, -1, 1.0, 0, 0, noop_exec, nullptr, nullptr);
        const int rc2 = m.N().freeTransform(m.sp());
        std::println(stderr, "S7 v{} include at stage {}: retcode {} late={} then SCIPfreeTransform retcode {}", m.version(), stage, rc, static_cast<void *>(late), rc2);
        // the wrapper's route: a mutator frees the transform and throws
        try { m.add_variables(1, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 1}); std::println(stderr, "S7 mutator after: no throw"); }
        catch(const std::exception & e) { std::println(stderr, "S7 mutator after: threw {}", e.what()); }
        std::fflush(stderr);
        _exit(0);  // what SCIPfree does afterwards is another question
    }
}
static void cmd_s7b() {
    recipe r; r.no_include = true; G() = r;
    V m; scope(m); auto g = cp::build_gadgets(m, 6, false);
    SEPA * ok = nullptr;
    const int rc0 = m.N().includeSepaBasic(m.sp(), &ok, "ok", "x", 0, -1, 1.0, 0, 0, noop_exec, nullptr, nullptr);
    std::println(stderr, "S7b v{} include at stage {}: retcode {}", m.version(), m.N().getStage(m.sp()), rc0);
    solved_val(m);
    std::println(stderr, "S7b v{} after solve, stage {}: set freq -> retcode {}", m.version(), m.N().getStage(m.sp()), m.seti("separating/ok/freq", 1));
    const int rc2 = m.N().freeTransform(m.sp());
    std::println(stderr, "S7b v{} SCIPfreeTransform retcode {} stage {}", m.version(), rc2, m.N().getStage(m.sp()));
    SEPA * late = nullptr;
    const int rc3 = m.N().includeSepaBasic(m.sp(), &late, "late2", "x", 0, -1, 1.0, 0, 0, noop_exec, nullptr, nullptr);
    std::println(stderr, "S7b v{} include back at stage PROBLEM ({}): retcode {}", m.version(), m.N().getStage(m.sp()), rc3);
    m.P->r.no_include = false;
    const double v = solved_val(m);
    std::println(stderr, "S7b v{} re-solve value={:.6g} stage {} freeTransform {}", m.version(), v, m.N().getStage(m.sp()), m.N().freeTransform(m.sp()));
}

// ---------------------------------------------------------------------- S8 ---
static void s8_run(const std::string & label, const bool maximize, const double offset, const double scale, const bool presolve) {
    G() = recipe{}; V m;
    if(!presolve) scope(m);
    std::mt19937 rng(7);
    std::uniform_int_distribution<int> A(1, 100), R(1, 50);
    const std::size_t C = 30, Rn = 5;
    std::vector<std::vector<double>> a(Rn, std::vector<double>(C));
    for(auto & r : a) for(auto & v : r) v = A(rng);
    std::vector<double> c(C); for(auto & v : c) v = R(rng) * scale;
    auto x = m.add_binary_variables(C);
    const auto ids = std::views::iota(std::size_t{0}, C);
    if(maximize) m.set_maximization(); else m.set_minimization();
    m.set_objective(xsum(ids, [&](std::size_t j) { return c[j] * x(j); }) + offset);
    if(maximize) for(auto & r : a) { double t = 0; for(double v : r) t += v; m.add_constraint(xsum(ids, [&r, &x](std::size_t j) { return r[j] * x(j); }) <= std::floor(t / 2)); }
    else for(auto & r : a) { double t = 0; for(double v : r) t += v; m.add_constraint(xsum(ids, [&r, &x](std::size_t j) { return r[j] * x(j); }) >= std::floor(t / 2)); }
    std::vector<int> col; for(std::size_t j = 0; j < C; ++j) col.push_back(m.native_id(x(j)));
    natives & n = m.N();
    long calls = 0; double max_orig = 0, max_lp = 0, max_lp_flip = 0; double f_lp = 0, f_orig = 0, f_own = 0;
    m.attach_custom([&](plugin & p, const std::vector<double> & pt, int) {
        double own = offset; for(std::size_t j = 0; j < C; ++j) own += c[j] * pt[static_cast<std::size_t>(col[j])];
        const double lp = n.getLPObjval(p.s), orig = n.getSolOrigObj(p.s, nullptr);
        if(calls == 0) { f_lp = lp; f_orig = orig; f_own = own; }
        ++calls;
        max_orig = std::max(max_orig, std::fabs(orig - own));
        max_lp = std::max(max_lp, std::fabs(lp - own));
        max_lp_flip = std::max(max_lp_flip, std::fabs(-lp + offset - own));
    });
    const double v = solved_val(m, 20);
    std::println(stderr, "S8 v{} {} max={} offset={} scale={} presolve={}: value={:.6g} runs={} calls={} first(LPObjval={:.6g} SolOrigObj={:.6g} own={:.6g}) max|orig-own|={:.3g} max|lp-own|={:.3g} max|(-lp+offset)-own|={:.3g}",
        m.version(), label, maximize, offset, scale, presolve, v, n.getNRuns(m.sp()), calls, f_lp, f_orig, f_own, max_orig, max_lp, max_lp_flip);
}
static void cmd_s8() {
    for(bool presolve : {false, true}) for(bool mx : {true, false}) {
        s8_run("base", mx, 0, 1, presolve);
        s8_run("offset", mx, 100, 1, presolve);
        s8_run("offset+scale", mx, 100, 0.25, presolve);
    }
}

// ---------------------------------------------------------------------- S9 ---
static void s9_run(const std::string & label, const bool use_scope, const bool two_term, const char * bparam, const bool no_dualfix) {
    G() = recipe{}; V m;
    if(use_scope) scope(m);
    auto g = cp::build_gadgets(m, 10, false);
    auto d2 = m.add_variables(1, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 1});
    const int dc2 = m.native_id(d2(std::size_t{0}));
    if(bparam) m.N().setBoolParam(m.sp(), bparam, 0);
    if(no_dualfix) { m.seti("propagating/dualfix/freq", -1); m.seti("propagating/dualfix/maxprerounds", 0); }
    const raw_cut cut = two_term ? raw_cut{{g.dummy, dc2}, {1.0, 1.0}, 'G', 1.0} : raw_cut{{g.dummy}, {1.0}, 'G', 1.0};
    bool once = false; double glb = NAN, gub = NAN; long inf0 = 0;
    m.attach_custom([&](plugin & p, const std::vector<double> & x, int) {
        if(once) return;
        once = true;
        SCIP_VAR * tv = nullptr; p.n->getTransformedVar(p.s, m.var_of(g.dummy), &tv);
        glb = p.n->varGetLbGlobal(tv); gub = p.n->varGetUbGlobal(tv);
        p.add(cut, x, p.r);
        inf0 = p.infeasible_flags;
    });
    const double v = solved_val(m, 20);
    std::println(stderr, "S9 v{} {} scope={} terms={} {}{}: status={} value={:.6g} (model opt {:.6g}) global_bounds_of_dummy_at_hook=[{},{}] infeasible_flag={} entries={}",
        m.version(), label, use_scope, two_term ? 2 : 1, bparam ? bparam : "defaults", no_dualfix ? " no-dualfix" : "", st(m), v, g.opt, glb, gub, inf0, m.P->entries);
}
static void s9b_run(const char * bparam, const bool dominance) {
    G() = recipe{}; V m;
    auto a = cp::build_atsp(m, 14, false, 2);
    auto d = m.add_variables(1, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 1});
    const int dc = m.native_id(d(std::size_t{0}));
    if(bparam) m.N().setBoolParam(m.sp(), bparam, 0);
    bool once = false;
    m.attach_custom([&](plugin & p, const std::vector<double> & x, int) {
        if(!once) { once = true; if(dominance) p.add(raw_cut{{dc}, {1.0}, 'G', 1.0}, x, p.r); }
        for(const auto & c : a.separate(x)) p.add(c, x, p.r);
    });
    const double v = solved_val(m, 30);
    std::println(stderr, "S9b v{} atsp14 at default settings, dummy>=1={} {}: status={} value={:.6g} nodes={} entries={} rows={} infeas={}", m.version(), dominance, bparam ? bparam : "defaults", st(m), v, m.N().getNTotalNodes(m.sp()), m.P->entries, m.P->rows, m.P->infeasible_flags);
}
static void cmd_s9() {
    s9b_run(nullptr, false); s9b_run(nullptr, true); s9b_run("misc/allowstrongdualreds", true); s9b_run("misc/allowweakdualreds", true);
    for(bool sc : {true, false}) for(bool two : {false, true}) {
        s9_run("dominance", sc, two, nullptr, false);
        s9_run("dominance", sc, two, "misc/allowstrongdualreds", false);
        s9_run("dominance", sc, two, "misc/allowweakdualreds", false);
        s9_run("dominance", sc, two, nullptr, true);
    }
}

// --------------------------------------------------------------------- S10 ---
struct built { cp::separator sep; std::function<bool(const std::vector<double> &)> ok; };
static built mkp_covers(V & m, const std::size_t cols, const std::size_t rows, const unsigned seed) {
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
    std::vector<int> col; for(std::size_t j = 0; j < cols; ++j) col.push_back(m.native_id(x(j)));
    built b;
    b.sep = [a, cap, col](const std::vector<double> & p, const cp::add_fn & add) {
        for(std::size_t i = 0; i < a.size(); ++i) {
            std::vector<std::size_t> order(col.size());
            for(std::size_t j = 0; j < order.size(); ++j) order[j] = j;
            std::ranges::sort(order, [&](std::size_t l, std::size_t r) { return p[static_cast<std::size_t>(col[l])] > p[static_cast<std::size_t>(col[r])]; });
            double w = 0; raw_cut cut{{}, {}, 'L', 0};
            for(std::size_t j : order) {
                w += a[i][j]; cut.ind.push_back(col[j]); cut.val.push_back(1);
                if(w > cap[i]) { cut.rhs = static_cast<double>(cut.ind.size()) - 1; if(cut.violation(p) > 1e-4) add(cut); break; }
            }
        }
    };
    b.ok = [a, cap, col](const std::vector<double> & p) {
        for(std::size_t i = 0; i < a.size(); ++i) {
            double act = 0;
            for(std::size_t j = 0; j < col.size(); ++j) { const double v = p[static_cast<std::size_t>(col[j])]; if(v < -1e-6 || v > 1 + 1e-6) return false; act += a[i][j] * v; }
            if(act > cap[i] + 1e-5 * (1 + cap[i])) return false;
        }
        return true;
    };
    return b;
}
static built cg_fuzz(V & m, const unsigned seed) {
    const std::size_t cols = 35, rows = 15;
    std::mt19937 rng(seed);
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
    std::vector<int> col; for(std::size_t j = 0; j < cols; ++j) col.push_back(m.native_id(x(j)));
    auto gen = std::make_shared<std::mt19937>(seed * 977u);
    built out;
    out.sep = [a, b, col, gen](const std::vector<double> & p, const cp::add_fn & add) {
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
            if(cut.violation(p) > 1e-4) { add(cut); ++added; }
        }
    };
    out.ok = [a, b, col](const std::vector<double> & p) {
        for(std::size_t i = 0; i < a.size(); ++i) {
            double act = 0;
            for(std::size_t j = 0; j < col.size(); ++j) { const double v = p[static_cast<std::size_t>(col[j])]; if(v < -1e-6 || v > 1 + 1e-6) return false; act += a[i][j] * v; }
            if(act > b[i] + 1e-5 * (1 + std::fabs(b[i]))) return false;
        }
        return true;
    };
    return out;
}
static built dfj_deep(V & m, const int n, const unsigned seed) {
    auto a = cp::build_atsp(m, n, false, seed);
    built b;
    b.sep = [a](const std::vector<double> & x, const cp::add_fn & add) { for(const auto & c : a.separate(x, 2)) add(c); };
    b.ok = [a](const std::vector<double> & x) { return a.point_ok(x); };
    return b;
}
static built hull_gadgets(V & m, const int k) {
    auto g = cp::build_gadgets(m, k, false);
    built b;
    b.sep = [g](const std::vector<double> & x, const cp::add_fn & add) {
        for(int i = 0; i < g.k; ++i) { const auto c = g.hull_cut(i); if(c.violation(x) > 1e-4) add(c); }
    };
    b.ok = [g](const std::vector<double> & x) { return g.point_ok(x); };
    return b;
}
struct inst { std::string name; std::function<built(V &)> make; bool deep_only = false; };
static std::vector<inst> instances() {
    return {
        {"atsp16s2_deepDFJ", [](V & m) { return dfj_deep(m, 16, 2); }, true},
        {"mkp40x5s2_covers", [](V & m) { return mkp_covers(m, 40, 5, 2); }},
        {"mkp40x5s3_covers(restarts)", [](V & m) { return mkp_covers(m, 40, 5, 3); }},
        {"cg35x15s1(restarts)", [](V & m) { return cg_fuzz(m, 1); }},
        {"gadgets8_hull", [](V & m) { return hull_gadgets(m, 8); }},
        {"mkp60x5s1_covers", [](V & m) { return mkp_covers(m, 60, 5, 1); }},
    };
}
static void cmd_s10() {
    for(const auto & in : instances()) {
        const auto run = [&](const std::string & lab, const recipe & r, const int active) {
            G() = r; V m;
            auto b = in.make(m);
            m.set_time_limit(std::chrono::duration<double>(120));
            if(active == 1) m.attach([](const std::vector<double> &, const cp::add_fn &) {});
            const auto t0 = clk::now();
            m.solve();
            const double t = std::chrono::duration<double>(clk::now() - t0).count();
            std::println(stderr, "S10 v{} {} {}: status={} value={:.10g} nodes={} lpiter={} runs={} entries={} t={:.2f}", m.version(), in.name, lab, st(m), val(m), m.N().getNTotalNodes(m.sp()), m.N().getNLPIterations(m.sp()), m.N().getNRuns(m.sp()), m.P->entries, t);
        };
        recipe none; none.no_include = true;
        run("no plugin      ", none, 0);
        run("no plugin again", none, 0);
        run("dormant        ", recipe{}, 0);
        run("noop hook      ", recipe{}, 1);
    }
}

// --------------------------------------------------------------------- S11 ---
static std::vector<SCIP_VAR *> * s11_copy = nullptr;
static V * s11_model = nullptr;
static natives * s11_n = nullptr;
static int s11_lock(void * s, CONSHDLR *, SCIP_CONS *, int locktype, int pos, int neg) {
    const int cnt = static_cast<int>(s11_model->num_variables());
    for(int i = 0; i < cnt; ++i) {
        SCIP_VAR * v = s11_copy ? (*s11_copy)[static_cast<std::size_t>(i)] : s11_model->var_of(i);
        const int rc = s11_n->addVarLocksType(s, v, locktype, pos, neg);
        if(rc != RC_OKAY) return rc;
    }
    return RC_OKAY;
}
static int s11_enfo(void *, CONSHDLR *, SCIP_CONS **, int, int, unsigned, SCIP_RESULT * r) { *r = sc::SCIP_FEASIBLE; return RC_OKAY; }
static int s11_enfops(void *, CONSHDLR *, SCIP_CONS **, int, int, unsigned, unsigned, SCIP_RESULT * r) { *r = sc::SCIP_FEASIBLE; return RC_OKAY; }
static int s11_check(void *, CONSHDLR *, SCIP_CONS **, int, void *, unsigned, unsigned, unsigned, unsigned, SCIP_RESULT * r) { *r = sc::SCIP_FEASIBLE; return RC_OKAY; }
static void cmd_s11(const std::string & how) {
    recipe r; r.no_include = true; G() = r;
    V * m = new V;  // deleted by hand: the destructor is the experiment
    s11_model = m; s11_n = &m->N();
    scope(*m);
    auto g = cp::build_gadgets(*m, 6, false);
    std::vector<SCIP_VAR *> copy = m->copy_vars();
    if(how == "copy") s11_copy = &copy;
    const int rc = m->N().includeConshdlrBasic(m->sp(), nullptr, "vlock", "lock probe", -2000000, -2000000, -1, 0,
        reinterpret_cast<void *>(s11_enfo), reinterpret_cast<void *>(s11_enfops), reinterpret_cast<void *>(s11_check), reinterpret_cast<void *>(s11_lock), nullptr);
    const double v = how == "nosolve" ? NAN : solved_val(*m, 20);
    std::println(stderr, "S11 v{} {} include={} value={:.6g} stage={} about to destroy", m->version(), how, rc, v, m->N().getStage(m->sp()));
    std::fflush(stderr);
    delete m;
    std::println(stderr, "S11 {} destroyed cleanly", how);
}
// The analogue for the recipe: a separator exit callback that reads the wrapper's vector.
static V * s11s_model = nullptr;
static int s11s_exitsol(void *, SEPA *) {
    SCIP_VAR * v = s11s_model->var_of(0);
    std::println(stderr, "   SEPAEXITSOL at stage {}: _scip_vars[0]={}", s11s_model->N().getStage(s11s_model->sp()), static_cast<void *>(v));
    return RC_OKAY;
}
static void cmd_s11sepa() {
    G() = recipe{}; V * m = new V; s11s_model = m;
    scope(*m);
    auto g = cp::build_gadgets(*m, 6, false);
    m->N().sepaSetExitsol(m->sp(), m->P->sepa, s11s_exitsol);
    m->attach([](const std::vector<double> &, const cp::add_fn &) {});
    solved_val(*m, 20);
    std::println(stderr, "S11sepa v{} about to destroy; plugin alive during SCIPfree", m->version());
    delete m;
    std::println(stderr, "S11sepa destroyed cleanly");
}

int main(int argc, char ** argv) {
    const std::string c = argc > 1 ? argv[1] : "";
    { G() = recipe{}; V m; std::println(stderr, "library {} SCIP {} cmd={}", m.native_api().library_path().string(), m.version(), c); }
    if(c == "s1") cmd_s1();
    else if(c == "s1c") cmd_s1c();
    else if(c == "s1root") cmd_s1root();
    else if(c == "s2table") cmd_s2_table();
    else if(c == "s2ab") cmd_s2ab();
    else if(c == "s3") cmd_s3();
    else if(c == "s4") cmd_s4();
    else if(c == "s5") cmd_s5();
    else if(c == "s6") cmd_s6();
    else if(c == "s6p") cmd_s6p();
    else if(c == "s6age") cmd_s6age();
    else if(c == "s7") cmd_s7();
    else if(c == "s7b") cmd_s7b();
    else if(c == "s8") cmd_s8();
    else if(c == "s9") cmd_s9();
    else if(c == "s10") cmd_s10();
    else if(c == "s11") cmd_s11(argc > 2 ? argv[2] : "plain");
    else if(c == "s11sepa") cmd_s11sepa();
    else { std::println(stderr, "unknown"); return 2; }
    return 0;
}
