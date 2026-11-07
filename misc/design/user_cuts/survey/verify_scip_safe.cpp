// Skeptic pass, SCIP: safe direction (D), measurements (E), odd paths. Every attack runs
// in a forked child with an alarm, so a crash is a result and not the end of the run.
#include "verify_scip_native.hpp"
#include <csignal>
#include <unistd.h>
#include <sys/wait.h>
using namespace vs;
using cp::raw_cut;
using std::chrono::duration;
static std::string st(V & m) { return cp::status_name(m.get_status()); }
static double val(V & m) { return cf::value_of(m); }
static void scope(V & m, const bool allow_restarts = false) {
    natives & n = m.N(); void * s = m.sp();
    n.setPresolving(s, 3, 1); n.setHeuristics(s, 3, 1); n.setSeparating(s, 3, 1);
    m.seti("presolving/maxrestarts", allow_restarts ? -1 : 0);
    m.seti("misc/usesymmetry", 0);
    n.setBoolParam(s, "conflict/enable", 0);
}
static double t_solve(V & m, double limit) {
    m.set_time_limit(duration<double>(limit));
    const auto t0 = clk::now();
    try { m.solve(); }
    catch(const std::exception & e) { std::println(stderr, "   solve() threw: {}", e.what()); }
    return duration<double>(clk::now() - t0).count();
}
template <class F>
static void guarded(const std::string & name, const int seconds, F f) {
    std::fflush(stderr);
    const pid_t pid = fork();
    if(pid == 0) {
        alarm(static_cast<unsigned>(seconds));
        f();
        std::fflush(stderr);
        _exit(0);
    }
    int ws = 0;
    waitpid(pid, &ws, 0);
    if(WIFSIGNALED(ws)) std::println(stderr, "   CHILD {} killed by signal {} ({})", name, WTERMSIG(ws), WTERMSIG(ws) == SIGALRM ? "timeout" : "crash");
    else if(WEXITSTATUS(ws)) std::println(stderr, "   CHILD {} exit {}", name, WEXITSTATUS(ws));
}

// ----------------------------------------------------------------- instances ---
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

// --------------------------------------------------- tiny coefficient targeted ---
// a + b + c - e w <= 1 is valid: a triple needs z = 1, which forces w = U, and e U = 2.
// Below SCIP's epsilon (1e-9) SCIPvarAddToRow drops the term without moving the side.
static void tiny_run(const double e, const bool presolve, const bool repair, const bool hook) {
    G() = recipe{}; V m;
    if(!presolve) scope(m);
    const double U = 2.0 / e;
    auto bx = m.add_binary_variables(3);
    auto z = m.add_binary_variables(1);
    auto w = m.add_variables(1, {.obj_coef = 0, .lower_bound = 0, .upper_bound = U});
    const auto a = bx(std::size_t{0}), b = bx(std::size_t{1}), c = bx(std::size_t{2}), Z = z(std::size_t{0}), W = w(std::size_t{0});
    m.set_maximization();
    m.set_objective(10.0 * a + 10.0 * b + 10.0 * c - 14.0 * Z - (4.0 / U) * W);
    m.add_constraint(a + b - 1.0 * Z <= 1);
    m.add_constraint(a + c - 1.0 * Z <= 1);
    m.add_constraint(b + c - 1.0 * Z <= 1);
    m.add_constraint(W - U * Z >= 0);
    const int ia = m.native_id(a), ib = m.native_id(b), ic = m.native_id(c), iw = m.native_id(W);
    const double want = 12;
    long offers = 0;
    if(hook) m.attach([&](const std::vector<double> & x, const cp::add_fn & add) {
        raw_cut cut{{ia, ib, ic, iw}, {1.0, 1.0, 1.0, -e}, 'L', 1.0};
        if(repair && e < 1e-9) { cut.ind.pop_back(); cut.val.pop_back(); cut.rhs += e * U; }  // what a wrapper could do
        if(cut.violation(x) > 1e-6) { ++offers; add(cut); }
    });
    const double t = t_solve(m, 30);
    const double v = val(m);
    std::println(stderr, "TINY v{} e={} U={} presolve={} repair={} hook={}: status={} value={:.8g} want={} {} nodes={} offers={} rows={} t={:.2f}",
        m.version(), e, U, presolve, repair, hook, st(m), v, want, std::fabs(v - want) < 1e-5 ? "KEPT" : "LOST", m.N().getNTotalNodes(m.sp()), offers, m.P->rows, t);
}
static void cmd_tiny() {
    for(bool pres : {true, false}) {
        guarded("tiny", 60, [&] { tiny_run(5e-10, pres, false, false); });
        for(double e : {5e-10, 9e-10, 1.0e-9, 2e-9, 1e-8, 1e-7}) guarded("tiny", 60, [&] { tiny_run(e, pres, false, true); });
        guarded("tiny", 60, [&] { tiny_run(5e-10, pres, true, true); });
    }
}

// ----------------------------------------------------------- scaled valid cuts ---
static void scaled_run(const std::string & iname, const double S, const double base, std::function<built(V &)> make, const bool nosan) {
    G() = recipe{}; if(nosan) G().sanitize = false; V m;
    auto b = make(m);
    auto sep = b.sep;
    m.attach([sep, S](const std::vector<double> & x, const cp::add_fn & add) {
        sep(x, [&](const raw_cut & c) { raw_cut d = c; for(double & v : d.val) v *= S; d.rhs *= S; return add(d); });
    });
    const double t = t_solve(m, 40);
    const double v = val(m);
    std::println(stderr, "SCALE v{} {} S={:g} sanitize={}: status={} value={:.10g} base={:.10g} {} nodes={} rows={} applied={} add_errors={} sanitized={} infeas={} t={:.2f}",
        m.version(), iname, S, !nosan, st(m), v, base, cf::same(v, base) && cp::status_name(m.get_status()).starts_with("optimal") ? "same" : "DIFFERENT", m.N().getNTotalNodes(m.sp()), m.P->rows, m.N().sepaGetNCutsApplied(m.P->sepa), m.P->add_errors, m.P->sanitized, m.P->infeasible_flags, t);
}
static void cmd_scaled() {
    const std::vector<std::pair<std::string, std::function<built(V &)>>> ins = {
        {"mkp40x5s2", [](V & m) { return mkp_covers(m, 40, 5, 2); }},
        {"cg35x15s2", [](V & m) { return cg_fuzz(m, 2); }},
    };
    for(auto & [name, make] : ins) {
        double base;
        { G() = recipe{}; V m; auto b = make(m); t_solve(m, 60); base = val(m); std::println(stderr, "SCALE v{} {} baseline {} nodes={}", m.version(), name, base, m.N().getNTotalNodes(m.sp())); }
        for(double S : {1.0, 1e3, 1e6, 1e8, 1e10, 1e12, 1e15, 1e18, 1e19, 1e20, 1e21, 1e25, 1e-3, 1e-6, 1e-8, 1e-9, 1e-10, 1e-12})
            guarded("scaled", 60, [&] { scaled_run(name, S, base, make, false); });
    }
}

// Which feature turns a scaled valid cut into a lost optimum?
static void big_run(const unsigned seed, const double S, const std::string & tog, const double base) {
    G() = recipe{};
    if(tog == "rowonly") G().add_pool = false;
    if(tog == "poolonly") G().add_row = false;
    if(tog == "force") G().force = true;
    if(tog == "keep") G().removable = false;
    V m;
    natives & n = m.N(); void * s = m.sp();
    if(tog == "noconflict") n.setBoolParam(s, "conflict/enable", 0);
    if(tog == "norestart") m.seti("presolving/maxrestarts", 0);
    if(tog == "noredcost") m.seti("propagating/redcost/freq", -1);
    if(tog == "noprop") m.seti("propagating/maxrounds", 0), m.seti("propagating/maxroundsroot", 0);
    if(tog == "nosepa") n.setSeparating(s, 3, 1);
    if(tog == "noheur") n.setHeuristics(s, 3, 1);
    if(tog == "nopresolve") n.setPresolving(s, 3, 1);
    if(tog == "scope") scope(m);
    if(tog == "harden") m.P->r.harden = true;
    auto b = mkp_covers(m, 40, 5, seed);
    const auto sep = b.sep;
    const bool normalize = tog == "normalize";
    m.attach([sep, S, normalize](const std::vector<double> & x, const cp::add_fn & add) {
        sep(x, [&](const raw_cut & c) {
            raw_cut d = c;
            for(double & v : d.val) v *= S;
            d.rhs *= S;
            if(normalize) { double mx = 0; for(double v : d.val) mx = std::max(mx, std::fabs(v)); if(mx > 0) { for(double & v : d.val) v /= mx; d.rhs /= mx; } }
            return add(d);
        });
    });
    const double t = t_solve(m, 30);
    const double v = val(m);
    std::println(stderr, "BIG v{} seed={} S={:g} {}: status={} value={:.8g} base={:.8g} {} nodes={} runs={} rows={} applied={} conflicts={} t={:.2f}", m.version(), seed, S, tog, st(m), v, base, cf::same(v, base) ? "same" : "DIFFERENT", n.getNTotalNodes(s), n.getNRuns(s), m.P->rows, n.sepaGetNCutsApplied(m.P->sepa), n.getNConflictConssFound(s), t);
}
static void cmd_big(const std::string & what) {
    if(what == "sweep2") {
        long tot[4] = {0, 0, 0, 0}, diff[4] = {0, 0, 0, 0};
        const double Ss[] = {1e6, 1e7, 1e8, 1e9};
        for(unsigned seed = 1; seed <= 40; ++seed) {
            double base; { G() = recipe{}; V m; auto b = mkp_covers(m, 40, 5, seed); t_solve(m, 30); base = val(m); }
            for(int k = 0; k < 4; ++k) {
                G() = recipe{}; V m; auto b = mkp_covers(m, 40, 5, seed);
                const auto sep = b.sep; const double S = Ss[k];
                m.attach([sep, S](const std::vector<double> & x, const cp::add_fn & add) { sep(x, [&](const raw_cut & c) { raw_cut d = c; for(double & v : d.val) v *= S; d.rhs *= S; return add(d); }); });
                t_solve(m, 30);
                ++tot[k]; if(!cf::same(val(m), base)) { ++diff[k]; std::println(stderr, "   lost seed={} S={:g} value={} base={}", seed, S, val(m), base); }
            }
        }
        G() = recipe{}; V v;
        for(int k = 0; k < 4; ++k) std::println(stderr, "BIGSWEEP v{} S={:g}: runs={} lost={}", v.version(), Ss[k], tot[k], diff[k]);
        return;
    }
    if(what == "sweep") {
        for(unsigned seed : {1u, 2u, 3u, 4u, 5u, 6u}) {
            double base; { G() = recipe{}; V m; auto b = mkp_covers(m, 40, 5, seed); t_solve(m, 30); base = val(m); }
            for(double S : {1e6, 1e7, 1e8, 1e9, 3e9, 1e10, 1e11, 1e12, 1e14}) guarded("big", 50, [&] { big_run(seed, S, "plain", base); });
        }
    } else {
        for(unsigned seed : {2u}) {
            double base; { G() = recipe{}; V m; auto b = mkp_covers(m, 40, 5, seed); t_solve(m, 30); base = val(m); }
            for(const char * tog : {"plain", "rowonly", "poolonly", "force", "keep", "noconflict", "norestart", "noredcost", "noprop", "nosepa", "noheur", "nopresolve", "scope", "normalize", "harden"})
                for(double S : {1e10, 1e12}) guarded("big", 50, [&] { big_run(seed, S, tog, base); });
        }
    }
}

// When does the optimum leave the search? Knows x* from a run without the hook.
struct btrace {
    natives * n = nullptr; void * s = nullptr;
    std::vector<SCIP_VAR *> tv;
    std::vector<double> xs;
    std::map<SCIP_VAR *, std::size_t> idx;
    bool reported = false;
    long pruned_with_xs = 0, solved_with_xs = 0;
    double xs_val = 0;
    bool xs_in_box() const {
        for(std::size_t j = 0; j < tv.size(); ++j) if(xs[j] < n->varGetLbLocal(tv[j]) - 1e-6 || xs[j] > n->varGetUbLocal(tv[j]) + 1e-6) return false;
        return true;
    }
    static int exec(void *, EVENTHDLR *, EVENT * e, void * data) {
        auto & T = *static_cast<btrace *>(data);
        natives & n = *T.n;
        if(T.tv.empty() || !T.xs_in_box()) return RC_OKAY;
        ++T.solved_with_xs;
        const auto ty = n.eventGetType(e);
        const bool infeasible_event = n.majorVersion() < 10 ? ty == 0x100000ull : ty == 0x200000ull;
        if(!infeasible_event) return RC_OKAY;
        ++T.pruned_with_xs;
        if(T.reported) return RC_OKAY;
        T.reported = true;
        std::println(stderr, "  x* in the box of a pruned node: node={} depth={} LP solstat={} (1 opt 2 infeas 4 objlimit) LPObjval(trans)={:.8g} LocalLower(trans)={:.8g} cutoffbound={:.8g} x*(trans)={:.8g}",
            n.nodeGetNumber(n.getCurrentNode(T.s)), n.getDepth(T.s), n.getLPSolstat(T.s), n.getLPObjval(T.s), n.getLocalLowerbound(T.s), n.getCutoffbound(T.s), n.transformObj(T.s, T.xs_val));
        ROW ** rows = nullptr; int nr = 0;
        n.getLPRowsData(T.s, &rows, &nr);
        int violated = 0; double worst = 0; std::string worst_name; long u = 0; double worst_rel = 0;
        for(int i = 0; i < nr; ++i) {
            ROW * r = rows[i];
            double act = n.rowGetConstant(r);
            void ** cols = n.rowGetCols(r); double * vals = n.rowGetVals(r);
            for(int k = 0, c = n.rowGetNLPNonz(r); k < c; ++k) {
                SCIP_VAR * v = n.colGetVar(cols[k]);
                auto it = T.idx.find(v);
                if(it == T.idx.end()) { std::println(stderr, "  unknown column in a row"); continue; }
                act += vals[k] * T.xs[it->second];
            }
            const double viol = std::max({act - n.rowGetRhs(r), n.rowGetLhs(r) - act, 0.0});
            const double rel = viol / std::max({1.0, std::fabs(act), std::fabs(n.rowGetRhs(r) < 1e19 ? n.rowGetRhs(r) : 0.0)});
            if(n.rowGetName(r)[0] == 'u') ++u;
            if(viol > 1e-9) { ++violated; if(rel > worst_rel) { worst_rel = rel; worst = viol; worst_name = n.rowGetName(r); } }
        }
        std::println(stderr, "  LP rows={} of which hook rows={} ; rows x* violates: {} worst abs={:.3g} rel={:.3g} in <{}>", nr, u, violated, worst, worst_rel, worst_name);
        return RC_OKAY;
    }
};
static void cmd_bigtrace(const unsigned seed, const double S, const bool use_scope) {
    std::vector<double> xs; double base;
    {
        G() = recipe{}; V m; if(use_scope) scope(m); auto b = mkp_covers(m, 40, 5, seed); t_solve(m, 30); base = val(m);
        auto sol = m.get_solution();
        xs.resize(m.num_variables());
        // read through native: original pointers
        m.N().getSolVals(m.sp(), m.native_api().getBestSol(m.native_model()), static_cast<int>(xs.size()), m.vars(), xs.data());
    }
    G() = recipe{}; V m; natives & n = m.N(); void * s = m.sp();
    if(use_scope) scope(m);
    auto b = mkp_covers(m, 40, 5, seed);
    btrace T; T.n = &n; T.s = s; T.xs = xs; T.xs_val = base;
    EVENTHDLR * h = nullptr;
    n.includeEventhdlrBasic(s, &h, "btrace", "x", btrace::exec, nullptr);
    n.transformProb(s);
    const unsigned long long mask = n.majorVersion() < 10 ? (0x80000ull | 0x100000ull | 0x200000ull) : (0x100000ull | 0x200000ull | 0x400000ull);
    n.catchEvent(s, mask, h, &T, nullptr);
    for(std::size_t j = 0; j < xs.size(); ++j) { SCIP_VAR * t = nullptr; n.getTransformedVar(s, m.var_of(static_cast<int>(j)), &t); T.tv.push_back(t); T.idx[t] = j; }
    const auto sep = b.sep;
    m.attach([sep, S](const std::vector<double> & x, const cp::add_fn & add) {
        sep(x, [&](const raw_cut & c) { raw_cut d = c; for(double & v : d.val) v *= S; d.rhs *= S; return add(d); });
    });
    t_solve(m, 30);
    std::println(stderr, "BIGTRACE v{} seed={} S={:g} scope={}: status={} value={:.8g} base={:.8g} nodes={} rows={} nodes_with_x*={} pruned_with_x*={}", m.version(), seed, S, use_scope, st(m), val(m), base, n.getNTotalNodes(s), m.P->rows, T.solved_with_xs, T.pruned_with_xs);
}

// a sum over an odd cycle, tiny term on a column tied to the relaxing binary: presolve
// cannot close this one, the rest of the model is a real stable set problem
static void tiny2_run(const double e, const bool repair, const bool hook, const unsigned seed, const bool normalize_big = false) {
    G() = recipe{}; V m;
    std::mt19937 rng(seed);
    const std::size_t N = 40;
    auto x = m.add_binary_variables(N);
    auto zv = m.add_binary_variables(1);
    const double U = 3.0 / e;
    auto wv = m.add_variables(1, {.obj_coef = 0, .lower_bound = 0, .upper_bound = U});
    const auto z = zv(std::size_t{0}), w = wv(std::size_t{0});
    std::uniform_int_distribution<int> W(1, 9);
    std::vector<double> c(N);
    for(auto & v : c) v = W(rng);
    for(std::size_t i = 0; i < 5; ++i) { c[i] = 12; c[5 + i] = 0.1; }
    m.set_maximization();
    m.set_objective(xsum(std::views::iota(std::size_t{0}, N), [&](std::size_t i) { return c[i] * x(i); }) - 33.0 * z);
    // the 5-cycle, relaxed by z
    for(std::size_t i = 0; i < 5; ++i) m.add_constraint(x(i) + x((i + 1) % 5) - 1.0 * z <= 1);
    m.add_constraint(w - U * z == 0);
    // a random stable set rest, sparse, with odd holes
    std::uniform_int_distribution<std::size_t> P(5, N - 1);
    for(int k = 0; k < 90; ++k) { const auto i = P(rng), j = P(rng); if(i != j) m.add_constraint(x(i) + x(j) <= 1); }
    // keeps the two parts together for the presolver
    for(std::size_t i = 0; i < 5; ++i) m.add_constraint(x(i) + x(5 + i) <= 1);
    std::vector<int> cyc; for(std::size_t i = 0; i < 5; ++i) cyc.push_back(m.native_id(x(i)));
    const int iw = m.native_id(w);
    long offers = 0;
    if(hook) m.attach([&, e, U, repair](const std::vector<double> & p, const cp::add_fn & add) {
        raw_cut cut{cyc, {1, 1, 1, 1, 1}, 'L', 2.0};
        cut.ind.push_back(iw); cut.val.push_back(-e);
        if(repair && e <= 1e-9) { m.P->r.harden = true; }
        if(cut.violation(p) > 1e-6) { ++offers; add(cut); }
    });
    if(std::getenv("TINY_LOG")) m.seti("display/verblevel", 4);
    t_solve(m, 30);
    std::println(stderr, "TINY2 v{} seed={} e={:g} U={:g} repair={} hook={}: status={} value={:.8g} nodes={} runs={} entries={} offers={} rows={} add_errors={}", m.version(), seed, e, U, repair, hook, st(m), val(m), m.N().getNTotalNodes(m.sp()), m.N().getNRuns(m.sp()), m.P->entries, offers, m.P->rows, m.P->add_errors);
}
static void cmd_tiny2() {
    if(std::getenv("TINY_LOG")) { tiny2_run(1e-9, false, true, 1); return; }
    for(unsigned seed : {1u, 2u, 3u}) {
        for(double e : {5e-10, 1e-9, 2e-9, 1e-8, 1e-6}) {
            guarded("tiny2", 40, [&] { tiny2_run(e, false, false, seed); });
            guarded("tiny2", 40, [&] { tiny2_run(e, false, true, seed); });
            if(e <= 1e-9) guarded("tiny2", 40, [&] { tiny2_run(e, true, true, seed); });
        }
    }
}

// C8 under load crashed once: a stop at any moment, then a resume, with and without hook.
static void cmd_stop(const int n_size, const int hook_mode, const int reps) {
    long crashes = 0;
    for(int rep = 0; rep < reps; ++rep) {
        const double first = 0.001 * (1 + rep);
        std::fflush(stderr);
        const pid_t pid = fork();
        if(pid == 0) {
            alarm(60);
            G() = recipe{}; if(hook_mode == 2) G().expbackoff = 0;
            V m; auto a = cp::build_atsp(m, n_size, false, 9);
            const auto sep = [a](const std::vector<double> & x, const cp::add_fn & add) { for(const auto & c : a.separate(x)) add(c); };
            if(hook_mode) m.attach(sep);
            m.set_time_limit(duration<double>(first));
            std::string first_status = "threw";
            int stage1 = -1;
            try { m.solve(); first_status = st(m); stage1 = m.N().getStage(m.sp()); } catch(const std::exception & e) { first_status = e.what(); }
            if(hook_mode) m.attach(sep);
            m.set_time_limit(duration<double>(30));
            m.solve();
            std::println(stderr, "STOP n={} hook={} first={:.3f} {} stage={} entries={} then {} value={:.8g} nodes={}", n_size, hook_mode, first, first_status, stage1, m.P->entries, st(m), val(m), m.N().getNTotalNodes(m.sp()));
            std::fflush(stderr);
            _exit(0);
        }
        int ws = 0; waitpid(pid, &ws, 0);
        if(WIFSIGNALED(ws)) { ++crashes; std::println(stderr, "   CRASH first={:.3f} signal {}", first, WTERMSIG(ws)); }
        else if(WEXITSTATUS(ws)) std::println(stderr, "   exit {} first={:.3f}", WEXITSTATUS(ws), first);
    }
    std::println(stderr, "STOP total n={} hook={} reps={} crashes={}", n_size, hook_mode, reps, crashes);
}

// The same rows at the same scale as model constraints: is it about cuts?
static void cmd_bigmodel() {
    for(unsigned seed : {2u, 6u}) {
        std::vector<raw_cut> cuts; double base;
        {
            G() = recipe{}; V m; auto b = mkp_covers(m, 40, 5, seed);
            const auto sep = b.sep;
            m.attach([&, sep](const std::vector<double> & x, const cp::add_fn & add) { sep(x, [&](const raw_cut & c) { cuts.push_back(c); return add(c); }); });
            t_solve(m, 30); base = val(m);
        }
        std::ranges::sort(cuts, [](const raw_cut & a, const raw_cut & b) { return a.ind < b.ind; });
        cuts.erase(std::unique(cuts.begin(), cuts.end(), [](const raw_cut & a, const raw_cut & b) { return a.ind == b.ind; }), cuts.end());
        if(cuts.size() > 300) cuts.resize(300);
        for(double S : {1.0, 1e10, 1e12}) for(bool pres : {true, false}) {
            G() = recipe{}; V m;
            if(!pres) scope(m);
            auto b = mkp_covers(m, 40, 5, seed);
            std::vector<V::var_t> cols; for(auto v : m.variables()) cols.push_back(v);
            for(const auto & c : cuts) {
                std::vector<std::size_t> K(c.ind.size()); for(std::size_t i = 0; i < K.size(); ++i) K[i] = i;
                m.add_constraint(xsum(K, [&, S](std::size_t i) { return (c.val[i] * S) * auto(cols[static_cast<std::size_t>(c.ind[i])]); }) <= c.rhs * S);
            }
            t_solve(m, 40);
            std::println(stderr, "BIGMODEL v{} seed={} S={:g} presolve={} rows_in_model={}: status={} value={:.8g} base={:.8g} {} nodes={}", m.version(), seed, S, pres, cuts.size(), st(m), val(m), base, cf::same(val(m), base) ? "same" : "DIFFERENT", m.N().getNTotalNodes(m.sp()));
        }
    }
}

// ------------------------------------------------------- columns presolve fixes ---
static void cmd_fixed() {
    // all terms on fixed columns: the row ends up empty with a constant
    const auto run = [&](const std::string & lab, const bool sanitize_on, const bool presolve, const unsigned kind) {
        G() = recipe{}; G().sanitize = sanitize_on; V m;
        if(!presolve) scope(m);
        auto f = m.add_binary_variables(6);
        for(std::size_t i = 0; i < 6; ++i) m.add_constraint(1.0 * f(i) == 1.0);
        auto b = mkp_covers(m, 40, 5, 2);
        // mkp_covers set the objective; it leaves f at zero
        std::vector<int> fc; for(std::size_t i = 0; i < 6; ++i) fc.push_back(m.native_id(f(i)));
        const auto sep = b.sep;
        long offered = 0;
        m.attach([&, sep, fc, kind](const std::vector<double> & x, const cp::add_fn & add) {
            sep(x, add);
            ++offered;
            if(kind == 0) add(raw_cut{{fc[0], fc[1]}, {1.0, 1.0}, 'L', 2.0});
            if(kind == 1) add(raw_cut{{fc[0], fc[1], fc[2]}, {0.1, 0.2, 0.3}, 'L', 0.6});  // the sum rounds above 0.6 or not
            if(kind == 2) add(raw_cut{{fc[0], fc[1]}, {1.0, -1.0}, 'E', 0.0});
            if(kind == 3) add(raw_cut{{fc[0]}, {1.0}, 'G', 1.0});
            if(kind == 4) add(raw_cut{{fc[0], fc[1], fc[2], fc[3], fc[4], fc[5]}, {1, 1, 1, 1, 1, 1}, 'G', 6.0});
        });
        t_solve(m, 30);
        std::println(stderr, "FIXED v{} {} sanitize={} presolve={} kind={}: status={} value={:.8g} nodes={} rows={} pool={} sanitized={} infeas={} add_errors={}", m.version(), lab, sanitize_on, presolve, kind, st(m), val(m), m.N().getNTotalNodes(m.sp()), m.P->rows, m.P->pool_adds, m.P->sanitized, m.P->infeasible_flags, m.P->add_errors);
    };
    { G() = recipe{}; V m; auto f = m.add_binary_variables(6); for(std::size_t i = 0; i < 6; ++i) m.add_constraint(1.0 * f(i) == 1.0); auto b = mkp_covers(m, 40, 5, 2); t_solve(m, 30); std::println(stderr, "FIXED v{} baseline value={:.8g} nodes={}", m.version(), val(m), m.N().getNTotalNodes(m.sp())); }
    for(bool pres : {true, false}) for(bool san : {true, false}) for(unsigned k : {0u, 1u, 2u, 3u, 4u}) guarded("fixed", 60, [&] { run("all terms fixed", san, pres, k); });
}

// ---------------------------------------------------------------- near duplicates ---
static void neardup_run(const unsigned seed, const double delta, const int form) {
    G() = recipe{}; V m;
    auto b = mkp_covers(m, 40, 5, seed);
    const auto sep = b.sep;
    auto rng = std::make_shared<std::mt19937>(seed);
    m.attach([&, sep, delta, form, rng](const std::vector<double> & x, const cp::add_fn & add) {
        sep(x, [&](const raw_cut & c) {
            add(c);
            raw_cut d = c;
            if(form == 0) {  // coefficients moved by up to delta, side moved up to keep it valid (binaries, nonnegative)
                std::uniform_real_distribution<double> U(0, delta);
                for(double & v : d.val) v *= 1 + U(*rng);
                d.rhs *= 1 + delta;
            } else if(form == 1) {  // side only
                d.rhs += delta;
            } else {  // a scaled copy, moved by delta
                for(double & v : d.val) v *= 3.0;
                d.rhs = d.rhs * 3.0 * (1 + delta);
            }
            return add(d);
        });
    });
    const double t = t_solve(m, 30);
    std::println(stderr, "NEARDUP v{} seed={} delta={:g} form={}: status={} value={:.8g} nodes={} rows={} applied={} pool_end={} t={:.2f}", m.version(), seed, delta, form, st(m), val(m), m.N().getNTotalNodes(m.sp()), m.P->rows, m.N().sepaGetNCutsApplied(m.P->sepa), m.N().getNPoolCuts(m.sp()), t);
}
static void cmd_neardup() {
    for(unsigned seed : {1u, 2u, 3u}) {
        { G() = recipe{}; V m; auto b = mkp_covers(m, 40, 5, seed); t_solve(m, 30); std::println(stderr, "NEARDUP v{} seed={} baseline value={:.8g} nodes={}", m.version(), seed, val(m), m.N().getNTotalNodes(m.sp())); }
        for(int form : {0, 1, 2}) for(double d : {1e-4, 1e-5, 1e-6, 1e-7, 1e-8, 1e-9, 1e-10, 1e-12, 0.0}) guarded("neardup", 60, [&] { neardup_run(seed, d, form); });
    }
}

// ----------------------------------------------------------- one-term valid cuts ---
// A 7-cycle, integer stable sets have at most 3 of them, the LP 3.5.
//   y <= Sum x, y continuous: y <= 3 is valid and violated by the LP.
//   Sum x >= 3.25 q: q <= 0 is valid, a node with q fixed up by branching has a feasible LP.
//   q <= .6 + .4 r, r paid for: q stays fractional at the root, so that it gets branched.
static void oneterm_run(const std::string & lab, const std::vector<double> & coef, const std::vector<int> & which, const char sense, const double rhs, const int when, const bool presolve, const bool cutoff_result, const int childsel = 0) {
    G() = recipe{}; G().cutoff_result = cutoff_result; V m;
    natives & n = m.N(); void * s = m.sp();
    if(!presolve) scope(m);
    const std::size_t C = 7;
    auto x = m.add_binary_variables(C);
    auto qv = m.add_binary_variables(1); auto rv = m.add_binary_variables(1);
    auto yv = m.add_variables(1, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 10});
    auto yc = m.add_variables(1, {.obj_coef = 0, .lower_bound = -10, .upper_bound = 20});
    auto xb = m.add_binary_variables(1);
    const auto q = qv(std::size_t{0}), r = rv(std::size_t{0}), y = yv(std::size_t{0}), yy = yc(std::size_t{0}), xbar = xb(std::size_t{0});
    const auto Cs = std::views::iota(std::size_t{0}, C);
    m.set_maximization();
    m.set_objective(xsum(Cs, [&](std::size_t i) { return 1.0 * x(i); }) + 10.0 * q - 6.0 * r + 1.0 * y);
    for(std::size_t i : Cs) m.add_constraint(x(i) + x((i + 1) % C) <= 1);
    m.add_constraint(y - xsum(Cs, [&](std::size_t i) { return 1.0 * x(i); }) <= 0);
    m.add_constraint(xsum(Cs, [&](std::size_t i) { return 1.0 * x(i); }) - 3.25 * q >= 0);
    m.add_constraint(q - 0.4 * r <= 0.6);
    m.add_constraint(yy - y == -1);       // yy aggregated on y
    m.add_constraint(xbar + q == 1);      // xbar the complement of q
    const int ids[] = {m.native_id(y), m.native_id(yy), m.native_id(q), m.native_id(xbar), m.native_id(r)};
    SCIP_VAR * tq = nullptr; const double want = 6;  // q = 0, Sum x = 3, y = 3
    long added = 0;
    m.attach_custom([&](plugin & p, const std::vector<double> & pt, int depth) {
        if(!tq) p.n->getTransformedVar(p.s, m.var_of(ids[2]), &tq);
        const bool q_up = p.n->varGetLbLocal(tq) > 0.5;
        if(when == 1 && depth == 0) return;
        if(when == 2 && !q_up) return;
        raw_cut c{{}, {}, sense, rhs};
        for(std::size_t k = 0; k < which.size(); ++k) { c.ind.push_back(ids[which[k]]); c.val.push_back(coef[k]); }
        if(p.add(c, pt, p.r) == 0 && p.found) ++added;
    });
    if(childsel) n.setCharParam(s, "nodeselection/childsel", childsel == 1 ? 'u' : 'd');
    t_solve(m, 20);
    const double v = val(m);
    std::println(stderr, "ONETERM v{} {} when={} presolve={} cutoff_result={}: status={} value={:.8g} want={} {} nodes={} entries={} rows={} one_term={} pool={} sanitized={} infeas={} add_errors={}", m.version(), lab, when == 0 ? "any" : when == 1 ? "deep" : "q-up", presolve, cutoff_result, st(m), v, want, std::fabs(v - want) < 1e-5 ? "KEPT" : "LOST", n.getNTotalNodes(s), m.P->entries, m.P->rows, m.P->one_term_rows, m.P->pool_adds, m.P->sanitized, m.P->infeasible_flags, m.P->add_errors);
}
static void cmd_oneterm() {
    guarded("base", 30, [&] { G() = recipe{}; V m; std::println(stderr, "ONETERM runs next"); });
    for(bool pres : {true, false}) for(int when : {0, 1}) {
        guarded("o", 30, [&] { oneterm_run("y <= 3", {1.0}, {0}, 'L', 3.0, when, pres, true); });
        guarded("o", 30, [&] { oneterm_run("2y <= 6", {2.0}, {0}, 'L', 6.0, when, pres, true); });
        guarded("o", 30, [&] { oneterm_run("1e8 y <= 3e8", {1e8}, {0}, 'L', 3e8, when, pres, true); });
        guarded("o", 30, [&] { oneterm_run("1e12 y <= 3e12", {1e12}, {0}, 'L', 3e12, when, pres, true); });
        guarded("o", 30, [&] { oneterm_run("1e-8 y <= 3e-8", {1e-8}, {0}, 'L', 3e-8, when, pres, true); });
        guarded("o", 30, [&] { oneterm_run("1e-10 y <= 3e-10", {1e-10}, {0}, 'L', 3e-10, when, pres, true); });
        guarded("o", 30, [&] { oneterm_run("yy <= 2 (aggregated)", {1.0}, {1}, 'L', 2.0, when, pres, true); });
        guarded("o", 30, [&] { oneterm_run("q <= 0", {1.0}, {2}, 'L', 0.0, when, pres, true); });
        guarded("o", 30, [&] { oneterm_run("xbar >= 1 (complement)", {1.0}, {3}, 'G', 1.0, when, pres, true); });
    }
    // a node about to be pruned: the row is infeasible for the local bounds
    for(bool pres : {false, true}) for(bool cr : {true, false}) for(int cs : {1, 2}) {
        guarded("p", 30, [&] { oneterm_run("q <= 0 at q=1", {1.0}, {2}, 'L', 0.0, 2, pres, cr, cs); });
        guarded("p", 30, [&] { oneterm_run("q + r <= 1 at q=1", {1.0, 1.0}, {2, 4}, 'L', 1.0, 2, pres, cr, cs); });
    }
}

// --------------------------------------------------------- columns not in the LP ---
static void cmd_loose() {
    const auto run = [&](const bool dualfix_off, const int kind) {
        G() = recipe{}; G().drop_satisfied = false; V m; scope(m);
        if(dualfix_off) { m.seti("propagating/dualfix/freq", -1); m.seti("propagating/dualfix/maxprerounds", 0); }
        auto g = cp::build_gadgets(m, 8, false);
        const int d = g.dummy;
        bool once = false;
        m.attach([&, d, kind](const std::vector<double> & x, const cp::add_fn & add) {
            if(once) return;
            once = true;
            if(kind == 0) add(raw_cut{{d, g.y[3]}, {1.0, 1.0}, 'L', 2.0});
            if(kind == 1) add(raw_cut{{d}, {1.0}, 'L', 1.0});
            if(kind == 2) add(raw_cut{{d, g.y[3]}, {1.0, -1.0}, 'L', 1.0});
        });
        t_solve(m, 20);
        std::println(stderr, "LOOSE v{} dualfix_off={} kind={}: status={} value={:.6g} opt={:.6g} nodes={} rows={} add_errors={} infeas={}", m.version(), dualfix_off, kind, st(m), val(m), g.opt, m.N().getNTotalNodes(m.sp()), m.P->rows, m.P->add_errors, m.P->infeasible_flags);
    };
    for(bool off : {false, true}) for(int k : {0, 1, 2}) guarded("loose", 40, [&] { run(off, k); });
}

// ------------------------------------------------------------ LP unbounded filter ---
static void cmd_unbounded() {
    const auto run = [&](const std::string & lab, const bool presolve, const bool filter, const int shape) {
        G() = recipe{}; G().lp_filter = filter; V m;
        if(!presolve) { scope(m); m.seti("propagating/dualfix/freq", -1); m.seti("propagating/dualfix/maxprerounds", 0); }
        auto x = m.add_variables(2, {.obj_coef = 0, .lower_bound = 0, .upper_bound = m.infinity()});
        auto y = m.add_binary_variables(2);
        m.set_maximization();
        const auto X0 = x(std::size_t{0}), X1 = x(std::size_t{1}), Y0 = y(std::size_t{0}), Y1 = y(std::size_t{1});
        if(shape == 0) { m.set_objective(1.0 * X0 + Y0); m.add_constraint(X0 - X1 + Y0 + Y1 <= 1.5); }
        else {  // LP unbounded, MIP infeasible: x0 - x1 == .5 with integral x
            m.set_integer(X0); m.set_integer(X1);
            m.set_objective(1.0 * X0 + Y0);
            m.add_constraint(2.0 * X0 - 2.0 * X1 == 1.0);
        }
        long calls = 0; std::vector<double> first;
        m.attach([&](const std::vector<double> & p, const cp::add_fn &) { if(!calls) first = p; ++calls; });
        t_solve(m, 10);
        std::println(stderr, "UNBD v{} {} presolve={} filter={} shape={}: status={} entries={} not_optimal={} sep_calls={} first_x0={}", m.version(), lab, presolve, filter, shape, st(m), m.P->entries, m.P->not_optimal, calls, first.empty() ? NAN : first[0]);
    };
    for(int sh : {0, 1}) for(bool pres : {true, false}) for(bool filter : {true, false}) guarded("unbd", 20, [&] { run("LP ray", pres, filter, sh); });
}

// ------------------------------------------------------------- exception crossing ---
static void cmd_exc() {
    G() = recipe{}; V m; scope(m);
    auto g = cp::build_gadgets(m, 6, false);
    long calls = 0;
    m.attach([&](const std::vector<double> &, const cp::add_fn &) { if(++calls == 5) throw std::runtime_error("boom"); });
    bool threw = false; std::string what;
    m.set_time_limit(duration<double>(20));
    try { m.solve(); } catch(const std::exception & e) { threw = true; what = e.what(); }
    std::println(stderr, "EXC v{} throw at call 5: solve threw={} ({}) exceptions={} stage={} status={}", m.version(), threw, what, m.P->exceptions, m.N().getStage(m.sp()), st(m));
    // what is left of the model?
    bool threw2 = false; std::string what2;
    try { m.solve(); } catch(const std::exception & e) { threw2 = true; what2 = e.what(); }
    std::println(stderr, "EXC re-solve same state: threw={} ({}) calls={} stage={} status={}", threw2, what2, calls, m.N().getStage(m.sp()), st(m));
    bool threw3 = false; std::string what3;
    try { m.set_time_limit(duration<double>(20)); m.add_variables(1, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 1}); } catch(const std::exception & e) { threw3 = true; what3 = e.what(); }
    std::println(stderr, "EXC mutator: threw={} ({}) stage={}", threw3, what3, m.N().getStage(m.sp()));
    m.P->attached = false; m.P->sep = {};
    bool threw4 = false; std::string what4;
    try { m.solve(); } catch(const std::exception & e) { threw4 = true; what4 = e.what(); }
    double v = NAN; try { v = m.get_solution_value(); } catch(const std::exception &) {}
    std::println(stderr, "EXC solve after mutator, no hook: threw={} ({}) value={:.6g} opt={:.6g} stage={} status={}", threw4, what4, v, g.opt, m.N().getStage(m.sp()), st(m));
}

// ------------------------------------------------- nested solve, worker thread, two libs ---
static void cmd_nested(const std::string & other_lib) {
    G() = recipe{}; V outer; scope(outer);
    auto g = cp::build_gadgets(outer, 8, false);
    bool nested_done = false; double inner_value = NAN, inner_opt = NAN; long inner_calls = 0; std::string inner_ver; plugin * inner_p = nullptr;
    const auto outer_plugin = outer.P.get();
    long after_ok = 0, after_bad = 0, offers = 0;
    outer.attach([&](const std::vector<double> & x, const cp::add_fn & add) {
        if(!nested_done) {
            nested_done = true;
            auto run_inner = [&](V & in) {
                scope(in);
                auto h = cp::build_gadgets(in, 6, true);
                in.attach([&](const std::vector<double> & y, const cp::add_fn & a) {
                    ++inner_calls;
                    for(int i = 0; i < h.k; ++i) { const auto c = h.hull_cut(i); if(c.violation(y) > 1e-4) a(c); }
                });
                in.set_time_limit(duration<double>(20));
                in.solve();
                inner_value = in.get_solution_value(); inner_opt = h.opt; inner_ver = in.version(); inner_p = in.P.get();
            };
            if(other_lib.empty()) { V in; run_inner(in); }
            else { V in(sc::scip_api::load(other_lib)); run_inner(in); }
        }
        (tl_cur == outer_plugin ? after_ok : after_bad)++;
        for(int i = 0; i < g.k; ++i) { const auto c = g.hull_cut(i); if(c.violation(x) > 1e-4) { ++offers; add(c); } }
    });
    outer.set_time_limit(duration<double>(30));
    std::thread t([&] { outer.solve(); });
    t.join();
    std::println(stderr, "NESTED outer v{} on worker thread: status={} value={:.6g} opt={:.6g} nodes={} rows={} add_errors={} | inner v{} value={:.6g} opt={:.6g} calls={} | tl restored ok={} bad={} tl_cur_now={}",
        outer.version(), st(outer), val(outer), g.opt, outer.N().getNTotalNodes(outer.sp()), outer.P->rows, outer.P->add_errors, inner_ver, inner_value, inner_opt, inner_calls, after_ok, after_bad, static_cast<void *>(tl_cur));
    (void)inner_p;
}

// ------------------------------------------------------------------ brute-force fuzz ---
struct fz_opts { bool deep_only = false; bool scale = false; bool big_scale = false; double prob = 1.0; bool equalities = true; bool off = false; bool norm = false; };
struct fz_total { long models = 0, feasible = 0, fails = 0, calls = 0, rows = 0, deep_rows = 0, one_term = 0, pool = 0, sanitized = 0, infeas = 0, add_err = 0, multirun = 0, nodes = 0, on_derived = 0, tight_rows = 0, base_mismatch = 0, timeouts = 0; };
struct fz_model {
    int ncols = 0;
    std::vector<std::vector<double>> pts;
    std::vector<double> obj;
    double offset = 0, opt = 0;
    bool maximize = true, feasible = false;
    std::vector<char> derived;
};
template <class VV>
static fz_model fz_build(VV & m, const unsigned seed) {
    fz_model F;
    std::mt19937 rng(seed);
    const auto R = [&](int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); };
    const std::size_t nb = static_cast<std::size_t>(R(14, 16));
    auto x = m.add_binary_variables(nb);
    std::vector<V::var_t> cols;
    for(std::size_t j = 0; j < nb; ++j) cols.push_back(x(j));
    // two general integers
    auto gi = m.add_variables(2, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 2});
    m.set_integer(gi(std::size_t{0})); m.set_integer(gi(std::size_t{1}));
    const std::size_t prim = nb + 2;
    cols.push_back(gi(std::size_t{0})); cols.push_back(gi(std::size_t{1}));
    // rows on the primary columns
    const int nrows = R(5, 9);
    struct row_t { std::vector<double> a; char sense; double rhs; };
    std::vector<row_t> rows;
    for(int i = 0; i < nrows; ++i) {
        row_t r; r.a.assign(prim, 0.0);
        double pos = 0;
        const bool knap = R(0, 9) < 8;
        for(std::size_t j = 0; j < prim; ++j) { if(R(0, 9) < 7) { r.a[j] = knap ? R(5, 40) : R(-6, 20); pos += std::max(r.a[j], 0.0) * (j >= nb ? 2 : 1); } }
        const int k = R(0, 19);
        r.sense = k < 17 ? 'L' : k < 19 ? 'G' : 'E';
        r.rhs = std::floor(pos * (0.3 + 0.25 * R(0, 10) / 10.0));
        if(r.sense == 'G') r.rhs = std::floor(pos * (0.1 + 0.15 * R(0, 10) / 10.0));
        rows.push_back(r);
    }
    // parallel pair: column 1 is a copy of column 0
    for(auto & r : rows) r.a[1] = r.a[0];
    // correlated with the weights, or the search closes at once
    std::vector<double> c(prim);
    for(std::size_t j = 0; j < prim; ++j) { double t = 0; for(const auto & r : rows) t += std::max(r.a[j], 0.0); c[j] = std::floor(t / static_cast<double>(rows.size())) + R(-3, 3); }
    c[1] = c[0];
    F.maximize = true;
    if(R(0, 3) == 0) { F.maximize = false; for(auto & r : rows) { r.sense = r.sense == 'L' ? 'G' : r.sense == 'G' ? 'L' : 'E'; double pos2 = 0; for(double v : r.a) pos2 += std::max(v, 0.0); if(r.sense == 'G') r.rhs = std::floor(pos2 * (0.4 + 0.2 * R(0, 10) / 10.0)); } }
    F.offset = R(0, 1) ? R(-50, 50) : 0;
    for(const auto & r : rows) {
        auto e = xsum(std::views::iota(std::size_t{0}, prim), [&](std::size_t j) { return r.a[j] * auto(cols[j]); });
        if(r.sense == 'L') m.add_constraint(e <= r.rhs); else if(r.sense == 'G') m.add_constraint(e >= r.rhs); else m.add_constraint(e == r.rhs);
    }
    // derived columns, determined by the primary ones, which presolve removes.
    // complements
    const int ncomp = 3;
    std::vector<std::pair<int, int>> comp;
    for(int i = 0; i < ncomp; ++i) {
        const auto j = static_cast<std::size_t>(R(0, static_cast<int>(nb) - 1));
        auto b = m.add_binary_variables(1);
        m.add_constraint(auto(cols[j]) + b(std::size_t{0}) == 1);
        comp.emplace_back(static_cast<int>(j), static_cast<int>(cols.size()));
        cols.push_back(b(std::size_t{0}));
    }
    // fixed by an equality row, and by its bounds
    auto f1 = m.add_binary_variables(1);
    m.add_constraint(1.0 * f1(std::size_t{0}) == 1.0);
    const int fc = static_cast<int>(cols.size()); cols.push_back(f1(std::size_t{0}));
    auto f2 = m.add_variables(1, {.obj_coef = 0, .lower_bound = 2.5, .upper_bound = 2.5});
    const int fd = static_cast<int>(cols.size()); cols.push_back(f2(std::size_t{0}));
    // continuous w = sum a x + k: multi-aggregated
    std::vector<double> wa(prim, 0.0);
    for(std::size_t j = 0; j < prim; ++j) if(R(0, 9) < 4) wa[j] = R(-4, 4);
    const double wk = R(-3, 3);
    auto w = m.add_variables(1, {.obj_coef = 0, .lower_bound = -1e4, .upper_bound = 1e4});
    const int wc = static_cast<int>(cols.size()); cols.push_back(w(std::size_t{0}));
    m.add_constraint(w(std::size_t{0}) - xsum(std::views::iota(std::size_t{0}, prim), [&](std::size_t j) { return wa[j] * auto(cols[j]); }) == wk);
    // a column in no row, zero objective: dual fixing removes it, both its values are feasible
    auto d = m.add_binary_variables(1);
    const int dc = static_cast<int>(cols.size()); cols.push_back(d(std::size_t{0}));
    const std::vector<std::size_t> objcols = [&] { std::vector<std::size_t> v(prim); for(std::size_t j = 0; j < prim; ++j) v[j] = j; return v; }();
    if(F.maximize) m.set_maximization(); else m.set_minimization();
    m.set_objective(xsum(objcols, [&](std::size_t j) { return c[j] * auto(cols[j]); }) + F.offset);
    F.ncols = static_cast<int>(cols.size());
    F.obj.assign(cols.size(), 0.0);
    for(std::size_t j = 0; j < prim; ++j) F.obj[j] = c[j];
    F.derived.assign(cols.size(), 0);
    for(std::size_t j = prim; j < cols.size(); ++j) F.derived[j] = 1;
    // enumerate
    std::vector<double> p(cols.size(), 0.0);
    const long nbin = 1L << nb;
    double best = F.maximize ? -1e300 : 1e300;
    for(long mask = 0; mask < nbin; ++mask) for(int g0 = 0; g0 <= 2; ++g0) for(int g1 = 0; g1 <= 2; ++g1) {
        for(std::size_t j = 0; j < nb; ++j) p[j] = (mask >> j) & 1;
        p[nb] = g0; p[nb + 1] = g1;
        bool ok = true;
        for(const auto & r : rows) {
            double act = 0;
            for(std::size_t j = 0; j < prim; ++j) act += r.a[j] * p[j];
            if((r.sense == 'L' && act > r.rhs + 1e-9) || (r.sense == 'G' && act < r.rhs - 1e-9) || (r.sense == 'E' && std::fabs(act - r.rhs) > 1e-9)) { ok = false; break; }
        }
        if(!ok) continue;
        for(auto [j, b] : comp) p[static_cast<std::size_t>(b)] = 1 - p[static_cast<std::size_t>(j)];
        p[static_cast<std::size_t>(fc)] = 1; p[static_cast<std::size_t>(fd)] = 2.5;
        double wv = wk; for(std::size_t j = 0; j < prim; ++j) wv += wa[j] * p[j];
        p[static_cast<std::size_t>(wc)] = wv;
        double v = F.offset; for(std::size_t j = 0; j < prim; ++j) v += c[j] * p[j];
        best = F.maximize ? std::max(best, v) : std::min(best, v);
        F.feasible = true;
        for(int dv = 0; dv <= 1; ++dv) { p[static_cast<std::size_t>(dc)] = dv; F.pts.push_back(p); }
    }
    F.opt = best;
    return F;
}
static cp::separator fz_sep(const fz_model & F, const unsigned seed, const fz_opts & o, plugin * pl, long & tight, long & on_derived) {
    auto rng = std::make_shared<std::mt19937>(seed * 31u + 5u);
    return [&F, rng, o, pl, &tight, &on_derived](const std::vector<double> & x, const cp::add_fn & add) {
        if(!F.feasible) return;
        if(o.deep_only && pl->n->getDepth(pl->s) == 0) return;
        auto & g = *rng;
        const auto U = [&](double a, double b) { return std::uniform_real_distribution<double>(a, b)(g); };
        const auto I = [&](int a, int b) { return std::uniform_int_distribution<int>(a, b)(g); };
        const auto max_of = [&](const std::vector<int> & ind, const std::vector<double> & val, double & mx, double & mn) {
            mx = -1e300; mn = 1e300;
            for(const auto & p : F.pts) {
                double s = 0;
                for(std::size_t k = 0; k < ind.size(); ++k) s += val[k] * p[static_cast<std::size_t>(ind[k])];
                mx = std::max(mx, s); mn = std::min(mn, s);
            }
        };
        int added = 0;
        const auto offer = [&](raw_cut c) {
            if(o.prob < 1 && U(0, 1) > o.prob) return;
            for(int j : c.ind) if(F.derived[static_cast<std::size_t>(j)]) { ++on_derived; break; }
            double sc = 1;
            if(o.scale && I(0, 3) == 0) sc = std::pow(10.0, I(-3, 3));
            if(o.big_scale && I(0, 1) == 0) sc = std::pow(10.0, I(-7, 9));
            if(sc != 1) { for(double & v : c.val) v *= sc; c.rhs *= sc; }
            ++added;
            add(c);
        };
        // one-term bounds of the integer hull
        for(int tries = 0; tries < 6 && added < 2; ++tries) {
            const int j = I(0, F.ncols - 1);
            double mx, mn;
            max_of({j}, {1.0}, mx, mn);
            if(x[static_cast<std::size_t>(j)] > mx + 1e-4) offer(raw_cut{{j}, {1.0}, 'L', mx});
            else if(x[static_cast<std::size_t>(j)] < mn - 1e-4) offer(raw_cut{{j}, {1.0}, 'G', mn});
        }
        // the objective, tight at every optimum
        if(I(0, 3) == 0) {
            raw_cut c{{}, {}, F.maximize ? 'L' : 'G', F.opt - F.offset};
            for(int j = 0; j < F.ncols; ++j) if(F.obj[static_cast<std::size_t>(j)] != 0) { c.ind.push_back(j); c.val.push_back(F.obj[static_cast<std::size_t>(j)]); }
            if(c.violation(x) > 1e-4) { ++tight; offer(c); }
        }
        // directions that follow the point, sided on the feasible set
        for(int tries = 0; tries < 12 && added < 6; ++tries) {
            std::vector<int> ind; std::vector<double> val;
            for(int j = 0; j < F.ncols; ++j) {
                if(U(0, 1) > 0.6) continue;
                const double pj = x[static_cast<std::size_t>(j)];
                double a = pj > 0.5 ? 1.0 : -1.0;
                if(pj > 1e-6 && pj < 1 - 1e-6) a *= I(1, 3);
                if(I(0, 7) == 0) a = I(-3, 3);
                if(a != 0) { ind.push_back(j); val.push_back(a); }
            }
            if(ind.empty()) continue;
            double mx, mn;
            max_of(ind, val, mx, mn);
            raw_cut c{ind, val, 'L', mx};
            if(c.violation(x) > 1e-4) {
                offer(c);
                if(I(0, 4) == 0) {  // a near duplicate, still valid: recomputed on the feasible set
                    raw_cut d = c;
                    for(double & v : d.val) v *= 1 + U(-1e-7, 1e-7);
                    double m2, n2; max_of(d.ind, d.val, m2, n2);
                    d.rhs = m2;
                    offer(d);
                }
            } else {
                raw_cut e{ind, val, 'G', mn};
                if(e.violation(x) > 1e-4) offer(e);
                else if(mx == mn && std::fabs(c.activity(x) - mx) > 1e-6) offer(raw_cut{ind, val, 'E', mx});
            }
        }
    };
}
static void fz_one(const unsigned seed, const fz_opts & o, fz_total & T, const recipe & r, const char * label) {
    G() = recipe{};
    double base = NAN; std::string bstatus; long long bnodes = 0;
    {
        V b; auto F = fz_build(b, seed); t_solve(b, 30);
        base = val(b); bstatus = st(b); bnodes = b.N().getNTotalNodes(b.sp());
        const bool okb = F.feasible ? (bstatus.starts_with("optimal") && std::fabs(base - F.opt) < 1e-6) : bstatus.starts_with("infeasible");
        if(!okb) { ++T.base_mismatch; std::println(stderr, "BASE MISMATCH seed={} status={} value={} brute={} feasible={}", seed, bstatus, base, F.opt, F.feasible); }
    }
    G() = r;
    V m; if(o.off) scope(m);
    auto F = fz_build(m, seed);
    long tight = 0, on_derived = 0;
    m.attach(fz_sep(F, seed, o, m.P.get(), tight, on_derived));
    const double t = t_solve(m, 30);
    const std::string s = st(m);
    const double v = val(m);
    ++T.models; T.feasible += F.feasible;
    T.calls += m.P->entries; T.rows += m.P->rows; T.deep_rows += m.P->deep_rows; T.one_term += m.P->one_term_rows; T.pool += m.P->pool_adds; T.sanitized += m.P->sanitized; T.infeas += m.P->infeasible_flags; T.add_err += m.P->add_errors; T.nodes += m.N().getNTotalNodes(m.sp()); T.on_derived += on_derived; T.tight_rows += tight;
    T.multirun += m.N().getNRuns(m.sp()) > 1;
    const bool ok = F.feasible ? (s.starts_with("optimal") && std::fabs(v - F.opt) < 1e-5 * (1 + std::fabs(F.opt))) : s.starts_with("infeasible");
    if(!ok) { ++T.fails; std::println(stderr, "FUZZ FAIL {} v{} seed={} status={} value={:.8g} brute={:.8g} feasible={} nodes={} base_nodes={} {} t={:.2f}", label, m.version(), seed, s, v, F.opt, F.feasible, m.N().getNTotalNodes(m.sp()), bnodes, m.line(), t); }
}
static void cmd_fuzzone(const unsigned seed) {
    fz_opts o; o.off = true; o.prob = 0.3; o.big_scale = true;
    for(int k = 0; k < 2; ++k) {
        G() = recipe{}; G().harden = k == 1;
        V m; scope(m);
        auto F = fz_build(m, seed);
        long tight = 0, on_derived = 0;
        m.attach(fz_sep(F, seed, o, m.P.get(), tight, on_derived));
        t_solve(m, 30);
        std::println(stderr, "FUZZONE v{} seed={} harden={} maximize={} points={} status={} value={:.10g} brute={:.10g} nodes={} rows={} applied={}", m.version(), seed, k == 1, F.maximize, F.pts.size(), st(m), val(m), F.opt, m.N().getNTotalNodes(m.sp()), m.P->rows, m.N().sepaGetNCutsApplied(m.P->sepa));
    }
}
static void cmd_fuzz(const unsigned from, const unsigned to) {
    struct variant { const char * name; fz_opts o; recipe r; };
    std::vector<variant> vars;
    vars.push_back({"any-depth", {}, recipe{}});
    { fz_opts o; o.deep_only = true; vars.push_back({"deep-only", o, recipe{}}); }
    { fz_opts o; o.scale = true; vars.push_back({"scaled1e3", o, recipe{}}); }
    { fz_opts o; o.big_scale = true; vars.push_back({"scaled1e-7..1e9", o, recipe{}}); }
    { fz_opts o; o.prob = 0.3; vars.push_back({"p=.3", o, recipe{}}); }
    { fz_opts o; o.off = true; vars.push_back({"solver-features-off", o, recipe{}}); }
    { fz_opts o; o.off = true; o.deep_only = true; vars.push_back({"off,deep-only", o, recipe{}}); }
    { fz_opts o; o.off = true; o.prob = 0.3; o.big_scale = true; recipe r; r.harden = true; vars.push_back({"off,p=.3,scaled,HARDENED", o, r}); }
    { fz_opts o; o.off = true; o.prob = 0.3; o.big_scale = true; vars.push_back({"off,p=.3,scaled1e-7..1e9", o, recipe{}}); }
    { recipe r; r.force = true; vars.push_back({"force", {}, r}); }
    { recipe r; r.add_pool = false; vars.push_back({"row-only", {}, r}); }
    { recipe r; r.pool_guard = false; vars.push_back({"no-oneterm-guard", {}, r}); }
    { recipe r; r.add_row = false; vars.push_back({"pool-only", {}, r}); }
    { recipe r; r.sanitize = false; vars.push_back({"no-sanitize", {}, r}); }
    { fz_opts o; o.big_scale = true; recipe r; r.harden = true; vars.push_back({"scaled1e-7..1e9,HARDENED", o, r}); }
    for(auto & va : vars) {
        if(const char * only = std::getenv("FUZZ_ONLY")) if(!std::string(va.name).contains(only)) continue;
        fz_total T;
        const auto t0 = clk::now();
        for(unsigned seed = from; seed <= to; ++seed) fz_one(seed, va.o, T, va.r, va.name);
        { G() = recipe{}; V m; std::println(stderr, "FUZZ v{} {} seeds {}..{}: models={} feasible={} FAILS={} base_mismatch={} | calls={} rows={} deep_rows={} one_term={} pool={} sanitized={} infeas_flags={} add_errors={} multirun_models={} nodes={} rows_on_presolved_columns={} objective_cuts={} t={:.0f}s",
            m.version(), va.name, from, to, T.models, T.feasible, T.fails, T.base_mismatch, T.calls, T.rows, T.deep_rows, T.one_term, T.pool, T.sanitized, T.infeas, T.add_err, T.multirun, T.nodes, T.on_derived, T.tight_rows, duration<double>(clk::now() - t0).count()); }
    }
}

// ---------------------------------------------------------------------- measures ---
struct inst { std::string name; std::function<built(V &)> make; bool deep_only; bool use_scope; };
static std::vector<inst> instances() {
    return {
        {"atsp16s2_deepDFJ", [](V & m) { auto a = cp::build_atsp(m, 16, false, 2); built b; b.sep = [a](const std::vector<double> & x, const cp::add_fn & add) { for(const auto & c : a.separate(x, 2)) add(c); }; b.ok = [a](const std::vector<double> & x) { return a.point_ok(x); }; return b; }, true, false},
        {"mkp40x5s2_covers", [](V & m) { return mkp_covers(m, 40, 5, 2); }, false, false},
        {"mkp40x5s3_covers", [](V & m) { return mkp_covers(m, 40, 5, 3); }, false, false},
        {"cg35x15s1", [](V & m) { return cg_fuzz(m, 1); }, false, false},
        {"gadgets12_hull", [](V & m) { auto g = cp::build_gadgets(m, 12, false); built b; b.sep = [g](const std::vector<double> & x, const cp::add_fn & add) { for(int i = 0; i < g.k; ++i) { const auto c = g.hull_cut(i); if(c.violation(x) > 1e-4) add(c); } }; b.ok = [g](const std::vector<double> & x) { return g.point_ok(x); }; return b; }, true, true},
        {"mkp60x5s1_covers", [](V & m) { return mkp_covers(m, 60, 5, 1); }, false, false},
    };
}
static void cmd_measure() {
    struct mode { const char * name; bool hook, row, pool, force; };
    const mode modes[] = {{"nohook", false, false, false, false}, {"row", true, true, false, false}, {"row+pool", true, true, true, false}, {"row F", true, true, false, true}, {"row+pool F", true, true, true, true}, {"pool", true, false, true, false}};
    for(const auto & in : instances()) {
        for(int shift : {0, 1, 2}) {
            double base = NAN;
            for(const auto & mo : modes) {
                G() = recipe{}; G().add_row = mo.row; G().add_pool = mo.pool; G().force = mo.force;
                V m;
                if(in.use_scope) scope(m);
                m.seti("randomization/randomseedshift", shift);
                auto b = in.make(m);
                if(mo.hook) {
                    const auto sep = b.sep; const bool deep = in.deep_only; plugin * pl = m.P.get();
                    m.attach([sep, deep, pl](const std::vector<double> & x, const cp::add_fn & add) { if(deep && pl->n->getDepth(pl->s) == 0) return; sep(x, add); });
                }
                const double t = t_solve(m, 60);
                const double v = val(m);
                if(!mo.hook) base = v;
                const bool opt = st(m).starts_with("optimal");
                std::println(stderr, "MEASURE v{} {} shift={} {:11}: {} value_same={} nodes={} lpiter={} t={:.2f} runs={} rows={} applied={} deep_rows={} conss_end={} pool_end={}",
                    m.version(), in.name, shift, mo.name, opt ? "optimal" : st(m), cf::same(v, base), m.N().getNTotalNodes(m.sp()), m.N().getNLPIterations(m.sp()), t, m.N().getNRuns(m.sp()), m.P->rows, mo.hook ? m.N().sepaGetNCutsApplied(m.P->sepa) : 0, m.P->deep_rows, m.N().getNConss(m.sp()), m.N().getNPoolCuts(m.sp()));
            }
        }
    }
}

// Real automatic restarts: count, what moved across, whether the hook keeps working.
static void cmd_auto(const unsigned seeds, const bool pool) {
    struct agg { long models = 0, multi = 0, runs2_calls = 0, runs2_rows = 0, converted = 0, bad_points_r2 = 0, bad_points_r1 = 0, fails = 0, base_multi = 0, cases_conv = 0; };
    const auto one = [&](const char * name, const std::function<built(V &)> & make, agg & A) {
        double base; bool basemulti;
        { G() = recipe{}; V b; auto x = make(b); t_solve(b, 40); base = val(b); basemulti = b.N().getNRuns(b.sp()) > 1; }
        G() = recipe{}; G().add_pool = pool;
        V m; auto b = make(m);
        const auto sep = b.sep; const auto ok = b.ok;
        natives & n = m.N();
        long r2calls = 0, r2rows0 = -1, bad2 = 0, bad1 = 0, conv = 0; int lastrun = 1;
        m.P->custom = nullptr;
        m.attach_custom([&](plugin & p, const std::vector<double> & x, int) {
            const int run = n.getNRuns(p.s);
            if(run != lastrun) {
                lastrun = run;
                if(r2rows0 < 0) r2rows0 = p.rows;
                const int nc = n.getNConss(p.s);
                void ** cons = n.getConss(p.s);
                for(int i = 0; i < nc; ++i) { const char * nm = n.consGetName(cons[i]); if(nm[0] == 'u' && nm[1] == '_') ++conv; }
            }
            if(run >= 2) { ++r2calls; if(!ok(x)) ++bad2; } else if(!ok(x)) ++bad1;
            sep(x, [&](const raw_cut & c) { return p.add(c, x, p.r); });
        });
        t_solve(m, 40);
        const double v = val(m);
        ++A.models; A.base_multi += basemulti;
        const bool multi = n.getNRuns(m.sp()) > 1;
        A.multi += multi; A.runs2_calls += r2calls; if(multi && r2rows0 >= 0) A.runs2_rows += m.P->rows - r2rows0; A.converted += conv; A.cases_conv += conv > 0; A.bad_points_r2 += bad2; A.bad_points_r1 += bad1;
        const bool good = st(m).starts_with("optimal") && cf::same(v, base);
        if(!good) { ++A.fails; std::println(stderr, "AUTO FAIL {} status={} value={} base={}", name, st(m), v, base); }
    };
    agg M, C;
    for(unsigned s = 1; s <= seeds; ++s) {
        one("mkp", [s](V & m) { return mkp_covers(m, 40, 5, s); }, M);
        one("cg", [s](V & m) { return cg_fuzz(m, s); }, C);
    }
    for(auto [name, A] : {std::pair{"mkp40x5", M}, std::pair{"cg35x15", C}}) {
        G() = recipe{}; V m;
        std::println(stderr, "AUTO v{} {} pool={}: models={} with_restart={} (baseline {}) hook_calls_after_restart={} rows_after_restart={} converted_user_cuts={} in_models={} bad_points run1={} run2+={} FAILS={}", m.version(), name, pool, A.models, A.multi, A.base_multi, A.runs2_calls, A.runs2_rows, A.converted, A.cases_conv, A.bad_points_r1, A.bad_points_r2, A.fails);
    }
}

// Real restarts with user cuts in the LP at the restart: DFJ at the root, and the same with
// the restart thresholds lowered (still SCIP's own decision, labelled as induced).
static void cmd_auto2(const bool induced, const bool pool) {
    long models = 0, multi = 0, conv = 0, conv_models = 0, fails = 0, r2calls = 0, bad = 0, rows2 = 0, base_fail = 0;
    const auto one = [&](std::function<built(V &)> make) {
        const auto tune = [&](V & m) { if(induced) { m.native_api().setRealParam(m.native_model(), "presolving/immrestartfac", 0.0); m.native_api().setRealParam(m.native_model(), "presolving/restartfac", 0.0); m.native_api().setRealParam(m.native_model(), "presolving/subrestartfac", 0.0); m.native_api().setRealParam(m.native_model(), "presolving/restartminred", 0.0); } };
        double base; std::string bs;
        { G() = recipe{}; V b; tune(b); auto x = make(b); t_solve(b, 30); base = val(b); bs = st(b); }
        if(!bs.starts_with("optimal")) { ++base_fail; return; }
        G() = recipe{}; G().add_pool = pool;
        V m; tune(m); auto b = make(m);
        const auto sep = b.sep; const auto ok = b.ok;
        natives & n = m.N();
        long c2 = 0, cv = 0, r0 = -1; int last = 1;
        m.attach_custom([&](plugin & p, const std::vector<double> & x, int) {
            const int run = n.getNRuns(p.s);
            if(run != last) {
                last = run;
                if(r0 < 0) r0 = p.rows;
                const int nc = n.getNConss(p.s);
                void ** cons = n.getConss(p.s);
                for(int i = 0; i < nc; ++i) { const char * nm = n.consGetName(cons[i]); if(nm[0] == 'u' && nm[1] == '_') ++cv; }
            }
            if(run >= 2) { ++c2; if(!ok(x)) ++bad; }
            sep(x, [&](const raw_cut & c) { return p.add(c, x, p.r); });
        });
        t_solve(m, 30);
        ++models;
        const bool mr = n.getNRuns(m.sp()) > 1;
        multi += mr; conv += cv; conv_models += cv > 0; r2calls += c2; if(mr && r0 >= 0) rows2 += m.P->rows - r0;
        if(!(st(m).starts_with("optimal") && cf::same(val(m), base))) { ++fails; std::println(stderr, "AUTO2 FAIL status={} value={} base={} runs={} conv={}", st(m), val(m), base, n.getNRuns(m.sp()), cv); }
    };
    const auto dfj_i = [](int nn, unsigned seed) { return [nn, seed](V & m) { auto a = cp::build_atsp(m, nn, false, seed); built b; b.sep = [a](const std::vector<double> & x, const cp::add_fn & add) { for(const auto & c : a.separate(x, 5)) add(c); }; b.ok = [a](const std::vector<double> & x) { return a.point_ok(x); }; return b; }; };
    for(int nn : {12, 14, 16}) for(unsigned s = 1; s <= 8; ++s) one(dfj_i(nn, s));
    for(unsigned s = 1; s <= 12; ++s) { one([s](V & m) { return mkp_covers(m, 40, 5, s); }); one([s](V & m) { return mkp_covers(m, 25, 3, s); }); }
    G() = recipe{}; V v;
    std::println(stderr, "AUTO2 v{} induced={} pool={}: models={} with_restart={} baseline_timeouts={} hook_calls_after_restart={} rows_after_restart={} user_cuts_converted={} in_models={} bad_points_after_restart={} FAILS={}", v.version(), induced, pool, models, multi, base_fail, r2calls, rows2, conv, conv_models, bad, fails);
}

// A hook registered after a solve: the dormant separator against a late include.
static void cmd_lateattach() {
    const auto run = [&](const char * name, recipe r) {
        G() = r; V m; scope(m);
        auto g = cp::build_gadgets(m, 8, false);
        t_solve(m, 20);
        const double v1 = val(m); const int st1 = m.N().getStage(m.sp());
        long calls = 0;
        m.attach([&](const std::vector<double> & x, const cp::add_fn & add) { ++calls; for(int i = 0; i < g.k; ++i) { const auto c = g.hull_cut(i); if(c.violation(x) > 1e-4) add(c); } });
        t_solve(m, 20);
        const long calls2 = calls; const double v2 = val(m);
        // the first mutator frees the transform
        bool threw = false; std::string what;
        try { m.set_objective_coefficient(V::var_t{static_cast<int>(g.dummy)}, 0.0); } catch(const std::exception & e) { threw = true; what = e.what(); }
        double v3 = NAN; long calls3 = -1; std::string s3 = "-";
        if(!threw) { t_solve(m, 20); v3 = val(m); calls3 = calls; s3 = st(m); }
        std::println(stderr, "LATE v{} {}: first solve value={:.6g} stage={} | attach, solve: value={:.6g} hook_calls={} | mutator threw={} ({}) | then status={} value={:.6g} hook_calls={} nodes={} opt={:.6g}", m.version(), name, v1, st1, v2, calls2, threw, what, s3, v3, calls3, threw ? -1 : m.N().getNTotalNodes(m.sp()), g.opt);
        std::fflush(stderr);
        if(threw) _exit(0);
    };
    guarded("late", 60, [&] { run("recipe (dormant in ctor)", recipe{}); });
    guarded("late", 60, [&] { recipe r; r.include_late = true; run("include in attach", r); });
}
// Point read: does the separator see the point of the original model?
static void cmd_pointcheck() {
    const auto run = [&](const char * name, recipe r, const bool presolve_shift) {
        G() = r; V m;
        if(presolve_shift) { auto f = m.add_binary_variables(10); for(std::size_t i = 0; i < 10; ++i) m.add_constraint(1.0 * f(i) == 1.0); }
        auto b = mkp_covers(m, 40, 5, 2);
        const auto sep = b.sep; const auto ok = b.ok;
        long calls = 0, bad = 0;
        m.attach([&, sep, ok](const std::vector<double> & x, const cp::add_fn & add) { ++calls; if(!ok(x)) ++bad; sep(x, add); });
        t_solve(m, 30);
        std::println(stderr, "POINT v{} {} columns_shifted={}: status={} value={:.8g} calls={} bad_points={} rows={}", m.version(), name, presolve_shift, st(m), val(m), calls, bad, m.P->rows);
    };
    for(bool sh : {false, true}) {
        run("recipe (original pointers)", recipe{}, sh);
        recipe r; r.read_by_getvars = true; run("control: read SCIPgetVars by position", r, sh);
    }
}
// SCIPsetSeparating(OFF) on the model silences the plugin unless solve() writes freq again.
static void cmd_setsep() {
    for(bool write : {true, false}) {
        G() = recipe{}; G().freq = 1; V m; scope(m);
        m.seti("separating/mippp/freq", 1);
        m.no_freq_write = !write;
        auto g = cp::build_gadgets(m, 8, false);
        if(!write) m.N().setSeparating(m.sp(), 3, 1);
        m.attach([](const std::vector<double> &, const cp::add_fn &) {});
        t_solve(m, 20);
        std::println(stderr, "SETSEP v{} solve() writes freq={}: value={:.6g} entries={}", m.version(), write, val(m), m.P->entries);
    }
}
// Nested solve with the thread-local not restored: the outer hook then runs on the wrong plugin.
static void cmd_tlcontrol() {
    guarded("tl", 60, [&] {
        tl_no_restore = true;
        G() = recipe{}; V outer; scope(outer);
        auto g = cp::build_gadgets(outer, 8, false);
        bool done = false; long hits = 0;
        outer.attach([&](const std::vector<double> & x, const cp::add_fn & add) {
            ++hits;
            if(!done) { done = true; G() = recipe{}; V in; scope(in); auto h = cp::build_gadgets(in, 6, true); in.attach([](const std::vector<double> &, const cp::add_fn &) {}); in.set_time_limit(duration<double>(20)); in.solve(); }
            for(int i = 0; i < g.k; ++i) { const auto c = g.hull_cut(i); if(c.violation(x) > 1e-4) add(c); }
        });
        outer.set_time_limit(duration<double>(20));
        outer.solve();
        std::println(stderr, "TLCONTROL no restore: status={} value={:.6g} opt={:.6g} hook_hits={}", st(outer), val(outer), g.opt, hits);
    });
}
// Two SCIP versions in one process with no hook at all: sequential plain solves.
static void cmd_coexist(const std::string & a, const std::string & b) {
    for(const std::string & order : {std::string("ab"), std::string("ba")}) {
        guarded("coexist", 60, [&] {
            for(char c : order) {
                const std::string & lib = c == 'a' ? a : b;
                G() = recipe{}; G().no_include = true;
                bool threw = false; std::string what; double v = NAN; std::string ver;
                try {
                    V m(sc::scip_api::load(lib));
                    ver = m.version();
                    auto g = cp::build_gadgets(m, 6, false);
                    m.set_time_limit(duration<double>(20));
                    m.solve();
                    v = val(m);
                } catch(const std::exception & e) { threw = true; what = e.what(); }
                std::println(stderr, "COEXIST order={} now={} v{}: threw={} ({}) value={:.6g}", order, c, ver, threw, what, v);
            }
        });
    }
}

// A valid equality the LP violates: a + b >= .5 with binaries means a + b == 1.
static void cmd_eqcut() {
    for(unsigned seed : {1u, 2u, 3u}) for(bool pres : {true, false}) for(int mode : {0, 1, 2}) {
        double base;
        const auto build = [&](V & m) {
            if(!pres) scope(m);
            auto ab = m.add_binary_variables(2);
            auto b = mkp_covers(m, 40, 5, seed);
            m.set_objective_coefficient(ab(std::size_t{0}), -1.0);
            m.set_objective_coefficient(ab(std::size_t{1}), -1.5);
            m.add_constraint(2.0 * ab(std::size_t{0}) + 2.0 * ab(std::size_t{1}) >= 1.0);
            b.ok = [ia = m.native_id(ab(std::size_t{0})), ib = m.native_id(ab(std::size_t{1}))](const std::vector<double> & x) { return x[static_cast<std::size_t>(ia)] + x[static_cast<std::size_t>(ib)] < 1 - 1e-6; };
            return std::tuple(b, m.native_id(ab(std::size_t{0})), m.native_id(ab(std::size_t{1})));
        };
        { G() = recipe{}; V m; auto t = build(m); t_solve(m, 30); base = val(m); }
        G() = recipe{}; V m; auto [b, ia, ib] = build(m);
        long offered = 0;
        const auto sep = b.sep; const auto low = b.ok;
        m.attach([&, sep, low, ia, ib, mode](const std::vector<double> & x, const cp::add_fn & add) {
            sep(x, add);
            if(low(x)) { ++offered; add(raw_cut{{ia, ib}, {1.0, 1.0}, mode == 0 ? 'E' : 'G', 1.0}); if(mode == 2) add(raw_cut{{ia, ib}, {1e6, 1e6}, 'E', 1e6}); }
        });
        t_solve(m, 30);
        std::println(stderr, "EQCUT v{} seed={} presolve={} mode={}: status={} value={:.8g} base={:.8g} {} nodes={} offered={} rows={} add_errors={}", m.version(), seed, pres, mode == 0 ? "E" : mode == 1 ? "G" : "G+E*1e6", st(m), val(m), base, cf::same(val(m), base) ? "same" : "DIFFERENT", m.N().getNTotalNodes(m.sp()), offered, m.P->rows, m.P->add_errors);
    }
}
// A cut offered once, never again: what the pool is for.
static void cmd_once() {
    const char * sels[] = {"default", "breadthfirst"};
    for(const char * sel : sels) for(int k : {10, 11, 12}) {
        G() = recipe{}; long long base_nodes;
        { V m; scope(m); if(std::string(sel) != "default") m.seti("nodeselection/breadthfirst/stdpriority", 1000000); auto g = cp::build_gadgets(m, k, false); t_solve(m, 30); base_nodes = m.N().getNTotalNodes(m.sp()); }
        for(int mode : {0, 1, 2}) {
            G() = recipe{}; G().add_pool = mode != 0; G().add_row = mode != 2;
            V m; scope(m);
            if(std::string(sel) != "default") m.seti("nodeselection/breadthfirst/stdpriority", 1000000);
            auto g = cp::build_gadgets(m, k, false);
            const raw_cut cut = g.hull_cut(3);
            bool once = false; int d = -1;
            m.attach_custom([&](plugin & p, const std::vector<double> & x, int depth) {
                if(!once && depth >= 1 && g.state(x) == g.worse_state() && cut.violation(x) > 1e-4) { once = true; d = depth; p.add(cut, x, p.r); }
            });
            t_solve(m, 30);
            std::println(stderr, "ONCE v{} sel={} k={} {}: value={:.6g} opt={:.6g} nodes={} (no cut: {}) added_at_depth={} pool_end={}", m.version(), sel, k, mode == 0 ? "row" : mode == 1 ? "row+pool" : "pool", val(m), g.opt, m.N().getNTotalNodes(m.sp()), base_nodes, d, m.N().getNPoolCuts(m.sp()));
        }
    }
}

// Dynamic range inside one valid row: a cover plus e * (every other column), side raised by
// e * count, so valid on binaries; hardened or not.
static void cmd_range() {
    for(unsigned seed : {2u, 6u}) {
        double base; { G() = recipe{}; V m; auto b = mkp_covers(m, 40, 5, seed); t_solve(m, 30); base = val(m); }
        for(bool hard : {false, true}) for(double e : {1e-3, 1e-5, 1e-6, 1e-7, 1e-8, 3e-9, 1e-9, 1e-10, 1e-12}) {
            guarded("range", 50, [&] {
                G() = recipe{}; G().harden = hard; V m; auto b = mkp_covers(m, 40, 5, seed);
                const auto sep = b.sep; const int cols = static_cast<int>(m.num_variables());
                m.attach([sep, e, cols](const std::vector<double> & x, const cp::add_fn & add) {
                    sep(x, [&](const raw_cut & c) {
                        raw_cut d = c; std::set<int> in(c.ind.begin(), c.ind.end());
                        for(int j = 0; j < cols; ++j) if(!in.contains(j)) { d.ind.push_back(j); d.val.push_back(e); d.rhs += e; }
                        return add(d);
                    });
                });
                t_solve(m, 30);
                std::println(stderr, "RANGE v{} seed={} e={:g} harden={}: status={} value={:.8g} base={:.8g} {} nodes={} rows={} applied={} sanitized={}", m.version(), seed, e, hard, st(m), val(m), base, cf::same(val(m), base) ? "same" : "DIFFERENT", m.N().getNTotalNodes(m.sp()), m.P->rows, m.N().sepaGetNCutsApplied(m.P->sepa), m.P->sanitized);
            });
        }
    }
}

// Every coefficient of a valid cut below epsilon: e w >= e 1e5, valid because integrality forces
// t = 1 and w >= 1e5 t. SCIP keeps "0 >= 1e-5", which is more than its tolerance.
static void tinyinf_run(const double e, const bool hook, const bool presolve, const bool harden, const unsigned seed) {
    G() = recipe{}; G().harden = harden; V m;
    if(!presolve) scope(m);
    std::mt19937 rng(seed);
    const std::size_t N = 40;
    auto x = m.add_binary_variables(N);
    auto tv = m.add_binary_variables(1), sv = m.add_binary_variables(1);
    auto wv = m.add_variables(1, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 1e6});
    const auto t = tv(std::size_t{0}), sl = sv(std::size_t{0}), w = wv(std::size_t{0});
    std::uniform_int_distribution<int> W(1, 9);
    std::vector<double> c(N); for(auto & v : c) v = W(rng);
    m.set_maximization();
    m.set_objective(xsum(std::views::iota(std::size_t{0}, N), [&](std::size_t i) { return c[i] * x(i); }) - 40.0 * t - 1e-5 * w);
    for(std::size_t i = 0; i < 5; ++i) m.add_constraint(x(i) + x((i + 1) % 5) <= 1);
    m.add_constraint(t + sl >= 1);
    m.add_constraint(sl - xsum(std::views::iota(std::size_t{0}, std::size_t{5}), [&](std::size_t i) { return 1.0 * x(i); }) <= -2);
    m.add_constraint(w - 1e5 * t >= 0);
    std::uniform_int_distribution<std::size_t> P(5, N - 1);
    for(int k = 0; k < 90; ++k) { const auto i = P(rng), j = P(rng); if(i != j) m.add_constraint(x(i) + x(j) <= 1); }
    for(std::size_t i = 0; i < 5; ++i) m.add_constraint(x(i) + x(5 + i) <= 1);
    const int iw = m.native_id(w);
    long offers = 0;
    if(hook) m.attach([&, e, iw](const std::vector<double> & p, const cp::add_fn & add) {
        raw_cut cut{{iw}, {e}, 'G', e * 1e5};
        if(cut.violation(p) > 1e-9) { ++offers; add(cut); }
    });
    t_solve(m, 30);
    std::println(stderr, "TINYINF v{} seed={} e={:g} hook={} presolve={} harden={}: status={} value={:.8g} nodes={} entries={} offers={} rows={} infeas={}", m.version(), seed, e, hook, presolve, harden, st(m), val(m), m.N().getNTotalNodes(m.sp()), m.P->entries, offers, m.P->rows, m.P->infeasible_flags);
}
static void cmd_tinyinf() {
    for(unsigned seed : {1u, 2u}) for(bool pres : {true, false}) {
        guarded("ti", 40, [&] { tinyinf_run(1e-10, false, pres, false, seed); });
        for(double e : {1e-10, 1e-9, 2e-9, 1e-8}) guarded("ti", 40, [&] { tinyinf_run(e, true, pres, false, seed); });
        guarded("ti", 40, [&] { tinyinf_run(1e-10, true, pres, true, seed); });
    }
}

int main(int argc, char ** argv) {
    const std::string c = argc > 1 ? argv[1] : "";
    { G() = recipe{}; V m; std::println(stderr, "library {} SCIP {} cmd={}", m.native_api().library_path().string(), m.version(), c); }
    if(c == "tiny") cmd_tiny();
    else if(c == "scaled") cmd_scaled();
    else if(c == "fixed") cmd_fixed();
    else if(c == "big") cmd_big(argv[2]);
    else if(c == "bigtrace") cmd_bigtrace(static_cast<unsigned>(std::stoi(argv[2])), std::stod(argv[3]), std::string(argv[4]) == "scope");
    else if(c == "tiny2") cmd_tiny2();
    else if(c == "bigmodel") cmd_bigmodel();
    else if(c == "tinyinf") cmd_tinyinf();
    else if(c == "range") cmd_range();
    else if(c == "eqcut") cmd_eqcut();
    else if(c == "once") cmd_once();
    else if(c == "lateattach") cmd_lateattach();
    else if(c == "pointcheck") cmd_pointcheck();
    else if(c == "setsep") cmd_setsep();
    else if(c == "tlcontrol") cmd_tlcontrol();
    else if(c == "coexist") cmd_coexist(argv[2], argv[3]);
    else if(c == "fuzzone") cmd_fuzzone(static_cast<unsigned>(std::stoi(argv[2])));
    else if(c == "auto2") cmd_auto2(std::string(argv[2]) == "induced", std::string(argv[3]) == "pool");
    else if(c == "stop") cmd_stop(std::stoi(argv[2]), std::stoi(argv[3]), std::stoi(argv[4]));
    else if(c == "neardup") cmd_neardup();
    else if(c == "oneterm") cmd_oneterm();
    else if(c == "loose") cmd_loose();
    else if(c == "unbounded") cmd_unbounded();
    else if(c == "exc") cmd_exc();
    else if(c == "nested") cmd_nested(argc > 2 ? argv[2] : "");
    else if(c == "fuzz") cmd_fuzz(static_cast<unsigned>(std::stoi(argv[2])), static_cast<unsigned>(std::stoi(argv[3])));
    else if(c == "measure") cmd_measure();
    else if(c == "auto") cmd_auto(static_cast<unsigned>(std::stoi(argv[2])), std::string(argv[3]) == "pool");
    else { std::println(stderr, "unknown"); return 2; }
    return 0;
}
