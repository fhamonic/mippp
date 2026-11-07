// User-cut survey, SCIP. A separator plugin (SCIPincludeSepaBasic), cut modes as
// listed in `modes`. Native code never throws through SCIP: errors are retcodes.
#include "mippp/solvers/scip/all.hpp"
#include "cut_common.hpp"

namespace sc = mippp::scip::impl::v1;
using sc::SCIP_VAR;
using sc::SCIP_CONS;
using sc::SCIP_RESULT;
using sc::SCIP_RETCODE;
struct SCIP_Sepa; struct SCIP_Row; struct SCIP_Node;
using SEPA = SCIP_Sepa;
using ROW = SCIP_Row;
using NODE = SCIP_Node;
using Scip = sc::SCIP;

// The wrapper's native_id(variable) is SCIP_VAR *, the harness wants an int column,
// and its P experiment is gated on set_constraint_rhs, which scip_milp does not have.
struct M : mippp::scip_milp {
    using mippp::scip_milp::scip_milp;
    int native_id(variable v) const noexcept { return static_cast<int>(v.uid()); }
    SCIP_VAR * var_of(int i) const noexcept { return _scip_vars[static_cast<std::size_t>(i)]; }
    SCIP_VAR ** vars() noexcept { return _scip_vars.data(); }
    void set_constraint_rhs(constraint c, double r) { set_constraint_upper_bound(c, r); }
    double get_constraint_rhs(constraint c) { return get_constraint_upper_bound(c); }
};

enum { RC_OKAY = 1, RC_ERROR = 0 };
enum { LP_OPTIMAL = 1 };
enum { PS_OFF = 3, PS_DEFAULT = 0 };
using execlp_t = int (*)(void *, SEPA *, SCIP_RESULT *, unsigned, int);
using sepa_cb_t = int (*)(void *, SEPA *);

// Entry points scip_api does not load, resolved in the loaded library.
struct natives {
    std::filesystem::path lib;
    int (*includeSepaBasic)(void *, SEPA **, const char *, const char *, int, int, double, unsigned, unsigned, execlp_t, void *, void *);
    void * (*sepaGetData)(SEPA *);
    int (*setSepaExitsol)(void *, SEPA *, sepa_cb_t);
    long long (*sepaGetNCalls)(SEPA *);
    long long (*sepaGetNCutsFound)(SEPA *);
    long long (*sepaGetNCutsApplied)(SEPA *);
    int (*getLPSolstat)(void *);
    int (*createEmptyRowSepa)(void *, ROW **, SEPA *, const char *, double, double, unsigned, unsigned, unsigned);
    int (*cacheRow)(void *, ROW *);
    int (*flushRow)(void *, ROW *);
    int (*addVarToRow)(void *, ROW *, SCIP_VAR *, double);
    int (*addRow)(void *, ROW *, unsigned, unsigned *);
    int (*addPoolCut)(void *, ROW *);
    int (*releaseRow)(void *, ROW **);
    int (*addConsLocal)(void *, SCIP_CONS *, void *);
    int (*setPresolving)(void *, int, unsigned);
    int (*setHeuristics)(void *, int, unsigned);
    int (*setSeparating)(void *, int, unsigned);
    int (*setBoolParam)(void *, const char *, unsigned);
    int (*getTransformedVar)(void *, SCIP_VAR *, SCIP_VAR **);
    double (*getVarSol)(void *, SCIP_VAR *);
    int (*getDepth)(void *);
    NODE * (*getCurrentNode)(void *);
    long long (*nodeGetNumber)(NODE *);
    double (*varGetLbLocal)(SCIP_VAR *);
    double (*varGetUbLocal)(SCIP_VAR *);
    double (*varGetLbGlobal)(SCIP_VAR *);
    double (*varGetUbGlobal)(SCIP_VAR *);
    int (*varGetStatus)(SCIP_VAR *);
    unsigned (*inProbing)(void *);
    int (*getNRuns)(void *);
    int (*getNPoolCuts)(void *);
    int (*getNLPRows)(void *);
    int (*restartSolve)(void *);
    int (*getNSepaRounds)(void *);
    int (*getNCutsApplied)(void *);
    int (*getProbvarSum)(void *, SCIP_VAR **, double *, double *);
    int (*getNConss)(void *);
    int (*addVarsToRow)(void *, ROW *, int, SCIP_VAR **, double *);
};
static natives N;
template <class F> static void get(F & f, const char * n) { f = cp::symf<F>(N.lib, n); }
static void load_natives(M & m) {
    const auto l = m.native_api().library_path();
    if(N.lib == l) return;
    N.lib = l;
    get(N.includeSepaBasic, "SCIPincludeSepaBasic"); get(N.sepaGetData, "SCIPsepaGetData"); get(N.setSepaExitsol, "SCIPsetSepaExitsol");
    get(N.sepaGetNCalls, "SCIPsepaGetNCalls"); get(N.sepaGetNCutsFound, "SCIPsepaGetNCutsFound"); get(N.sepaGetNCutsApplied, "SCIPsepaGetNCutsApplied");
    get(N.getLPSolstat, "SCIPgetLPSolstat"); get(N.createEmptyRowSepa, "SCIPcreateEmptyRowSepa");
    get(N.cacheRow, "SCIPcacheRowExtensions"); get(N.flushRow, "SCIPflushRowExtensions"); get(N.addVarToRow, "SCIPaddVarToRow");
    get(N.addRow, "SCIPaddRow"); get(N.addPoolCut, "SCIPaddPoolCut"); get(N.releaseRow, "SCIPreleaseRow"); get(N.addConsLocal, "SCIPaddConsLocal");
    get(N.setPresolving, "SCIPsetPresolving"); get(N.setHeuristics, "SCIPsetHeuristics"); get(N.setSeparating, "SCIPsetSeparating");
    get(N.setBoolParam, "SCIPsetBoolParam"); get(N.getTransformedVar, "SCIPgetTransformedVar"); get(N.getVarSol, "SCIPgetVarSol");
    get(N.getDepth, "SCIPgetDepth"); get(N.getCurrentNode, "SCIPgetCurrentNode"); get(N.nodeGetNumber, "SCIPnodeGetNumber");
    get(N.varGetLbLocal, "SCIPvarGetLbLocal"); get(N.varGetUbLocal, "SCIPvarGetUbLocal");
    get(N.varGetLbGlobal, "SCIPvarGetLbGlobal"); get(N.varGetUbGlobal, "SCIPvarGetUbGlobal"); get(N.varGetStatus, "SCIPvarGetStatus");
    get(N.inProbing, "SCIPinProbing"); get(N.getNRuns, "SCIPgetNRuns"); get(N.getNPoolCuts, "SCIPgetNPoolCuts"); get(N.getNLPRows, "SCIPgetNLPRows");
    get(N.restartSolve, "SCIPrestartSolve"); get(N.getNSepaRounds, "SCIPgetNSepaRounds"); get(N.getNCutsApplied, "SCIPgetNCutsApplied");
    get(N.getProbvarSum, "SCIPgetProbvarSum"); get(N.getNConss, "SCIPgetNConss"); get(N.addVarsToRow, "SCIPaddVarsToRow");
}
static int setint(void * s, const char * p, int v, const sc::scip_api & api) {
    const int rc = api.setIntParam(static_cast<Scip *>(s), p, v);
    if(rc != RC_OKAY) std::println(stderr, "   setIntParam {} = {} -> retcode {}", p, v, rc);
    return rc;
}

// How one cut goes in.
struct mode_spec {
    enum kind_t { row, pool, row_and_pool, cons, cons_local } kind = row;
    bool local = false, removable = true, force = false;
    bool model_cons = false;  // cons modes: check and enforce, a model constraint
    int poolfreq = -1;        // separating/poolfreq; -1 keeps the default
};
static const std::map<std::string, mode_spec> mode_table = {
    {"row_g",            {mode_spec::row, false, true, false}},
    {"row_g_force",      {mode_spec::row, false, true, true}},
    {"row_g_keep",       {mode_spec::row, false, false, false}},
    {"row_g_keep_force", {mode_spec::row, false, false, true}},
    {"row_l",            {mode_spec::row, true, true, false}},
    {"row_l_force",      {mode_spec::row, true, true, true}},
    {"row_l_keep",       {mode_spec::row, true, false, false}},
    {"row_l_keep_force", {mode_spec::row, true, false, true}},
    {"pool_g",           {mode_spec::pool, false, true, false}},
    {"pool_g_keep",      {mode_spec::pool, false, false, false}},
    {"pool_g_freq1",     {mode_spec::pool, false, true, false, false, 1}},
    {"row+pool",         {mode_spec::row_and_pool, false, true, false}},
    {"row_force+pool",   {mode_spec::row_and_pool, false, true, true}},
    {"row_force+pool_freq1", {mode_spec::row_and_pool, false, true, true, false, 1}},
    {"pool_l",           {mode_spec::pool, true, true, false}},
    {"cons_g",           {mode_spec::cons, false, true, false, false}},
    {"cons_g_model",     {mode_spec::cons, false, false, false, true}},
    {"cons_l",           {mode_spec::cons_local, true, true, false, false}},
    {"cons_l_model",     {mode_spec::cons_local, true, false, false, true}},
};

struct ctx {
    cp::separator sep;
    const sc::scip_api * api;
    int cols;
    SCIP_VAR ** vars;
    mode_spec spec;
    std::string mode;
    SEPA * sepa = nullptr;
    // extra counters, printed at the end of each run
    long hook_runs = 0, not_optimal = 0, in_probing = 0, allowlocal_false = 0, allowlocal_false_at_add = 0, infeasible_flag = 0, depth_mismatch = 0;
    std::map<long long, int> calls_at_node;
    std::map<int, long> calls_at_depth;
    int last_add_depth = -1;
    unsigned last_allowlocal = 0;
    // Set by the extras: runs inside the hook, after the separator.
    std::function<void(void *, SEPA *, unsigned, int)> inspect;
    std::function<void(void *)> on_exit;
    // How a column reaches the row: 0 the original SCIP_VAR*, 1 its transformed
    // copy, 2 the active variable from SCIPgetProbvarSum with scalar and constant,
    // 3 the original pointers through SCIPaddVarsToRow, which caches and flushes itself.
    int var_map = 0;
};
static std::vector<std::unique_ptr<ctx>> keep;

static int add_cut(void * s, ctx & c, const cp::raw_cut & r, bool & found, bool & infeasible) {
    // SCIP_OKAY is 1 and SCIP_ERROR 0: map to the harness's zero-is-fine.
    const auto err = [](int rc) { return rc == RC_OKAY ? 0 : rc == RC_ERROR ? 1000 : rc; };
    auto * sp = static_cast<Scip *>(s);
    const double inf = c.api->infinity(sp);
    const double lhs = r.sense == 'L' ? -inf : r.rhs;
    const double rhs = r.sense == 'G' ? inf : r.rhs;
    const auto & spec = c.spec;
    if(spec.kind == mode_spec::cons || spec.kind == mode_spec::cons_local) {
        std::vector<SCIP_VAR *> v; std::vector<double> a;
        for(std::size_t k = 0; k < r.ind.size(); ++k) { v.push_back(c.vars[static_cast<std::size_t>(r.ind[k])]); a.push_back(r.val[k]); }
        SCIP_CONS * cons = nullptr;
        // initial, separate, enforce, check, propagate, local, modifiable, dynamic, removable, stickingatnode
        int rc = c.api->createConsLinear(sp, &cons, "probe_cons", static_cast<int>(v.size()), v.data(), a.data(), lhs, rhs,
            1, 1, spec.model_cons, spec.model_cons, 1, spec.local, 0, !spec.model_cons, spec.removable, 0);
        if(rc != RC_OKAY) return err(rc);
        rc = spec.kind == mode_spec::cons ? c.api->addCons(sp, cons) : N.addConsLocal(s, cons, nullptr);
        const int rc2 = c.api->releaseCons(sp, &cons);
        if(rc != RC_OKAY) return err(rc);
        if(rc2 != RC_OKAY) return err(rc2);
        found = true;
        return 0;
    }
    ROW * row = nullptr;
    int rc = RC_OKAY;
    if(c.var_map == 2) {
        // Resolve to active variables by hand: the sides move by the constants.
        std::vector<SCIP_VAR *> v; std::vector<double> a;
        double shift = 0;
        for(std::size_t k = 0; k < r.ind.size(); ++k) {
            SCIP_VAR * x = c.vars[static_cast<std::size_t>(r.ind[k])];
            double scalar = 1.0, constant = 0.0;
            if((rc = N.getProbvarSum(s, &x, &scalar, &constant)) != RC_OKAY) return err(rc);
            shift += r.val[k] * constant;
            if(x) { v.push_back(x); a.push_back(r.val[k] * scalar); }
        }
        rc = N.createEmptyRowSepa(s, &row, c.sepa, "probe", lhs <= -inf ? lhs : lhs - shift, rhs >= inf ? rhs : rhs - shift, spec.local, 0, spec.removable);
        if(rc != RC_OKAY) return err(rc);
        if((rc = N.cacheRow(s, row)) != RC_OKAY) return err(rc);
        for(std::size_t k = 0; k < v.size(); ++k)
            if((rc = N.addVarToRow(s, row, v[k], a[k])) != RC_OKAY) { N.releaseRow(s, &row); return err(rc); }
    } else if(c.var_map == 3) {
        std::vector<SCIP_VAR *> v; std::vector<double> a(r.val);
        for(int j : r.ind) v.push_back(c.vars[static_cast<std::size_t>(j)]);
        rc = N.createEmptyRowSepa(s, &row, c.sepa, "probe", lhs, rhs, spec.local, 0, spec.removable);
        if(rc != RC_OKAY) return err(rc);
        if((rc = N.addVarsToRow(s, row, static_cast<int>(v.size()), v.data(), a.data())) != RC_OKAY) { N.releaseRow(s, &row); return err(rc); }
        if((rc = N.cacheRow(s, row)) != RC_OKAY) return err(rc);  // pairs with the flush below
    } else {
    rc = N.createEmptyRowSepa(s, &row, c.sepa, "probe", lhs, rhs, spec.local, 0, spec.removable);
    if(rc != RC_OKAY) return err(rc);
    if((rc = N.cacheRow(s, row)) != RC_OKAY) return err(rc);
    for(std::size_t k = 0; k < r.ind.size(); ++k) {
        SCIP_VAR * x = c.vars[static_cast<std::size_t>(r.ind[k])];
        if(c.var_map == 1 && (rc = N.getTransformedVar(s, x, &x)) != RC_OKAY) { N.releaseRow(s, &row); return err(rc); }
        if((rc = N.addVarToRow(s, row, x, r.val[k])) != RC_OKAY) { N.releaseRow(s, &row); return err(rc); }
    }
    }
    if((rc = N.flushRow(s, row)) != RC_OKAY) return err(rc);
    if(spec.kind == mode_spec::row || spec.kind == mode_spec::row_and_pool) {
        unsigned inf_flag = 0;
        rc = N.addRow(s, row, spec.force, &inf_flag);
        if(rc != RC_OKAY) { N.releaseRow(s, &row); return err(rc); }
        if(inf_flag) { infeasible = true; ++c.infeasible_flag; }
    }
    if(spec.kind == mode_spec::pool || spec.kind == mode_spec::row_and_pool) {
        rc = N.addPoolCut(s, row);
        if(rc != RC_OKAY) { N.releaseRow(s, &row); return err(rc); }
    }
    rc = N.releaseRow(s, &row);
    if(rc != RC_OKAY) return err(rc);
    found = true;
    return 0;
}

static int execlp(void * s, SEPA * sepa, SCIP_RESULT * result, unsigned allowlocal, int depth) {
    auto & c = *static_cast<ctx *>(N.sepaGetData(sepa));
    *result = sc::SCIP_DIDNOTRUN;
    ++c.hook_runs;
    if(N.getLPSolstat(s) != LP_OPTIMAL) { ++c.not_optimal; return RC_OKAY; }
    if(N.inProbing(s)) ++c.in_probing;
    if(!allowlocal) ++c.allowlocal_false;
    if(depth != N.getDepth(s)) ++c.depth_mismatch;
    ++c.calls_at_node[N.nodeGetNumber(N.getCurrentNode(s))];
    ++c.calls_at_depth[N.getDepth(s)];
    auto * sp = static_cast<Scip *>(s);
    bool found = false, infeasible = false;
    try {
        std::vector<double> x(static_cast<std::size_t>(c.cols));
        const int rc = c.api->getSolVals(sp, nullptr, c.cols, c.vars, x.data());
        if(rc != RC_OKAY) return rc;
        c.sep(x, [&](const cp::raw_cut & r) {
            if(!allowlocal) ++c.allowlocal_false_at_add;
            c.last_add_depth = N.getDepth(s);
            c.last_allowlocal = allowlocal;
            return add_cut(s, c, r, found, infeasible);
        });
        if(c.inspect) c.inspect(s, sepa, allowlocal, depth);
    } catch(...) {
        return RC_ERROR;
    }
    *result = infeasible ? sc::SCIP_CUTOFF : found ? sc::SCIP_SEPARATED : sc::SCIP_DIDNOTFIND;
    return RC_OKAY;
}

static int exitsol(void * s, SEPA * sepa) {
    auto & c = *static_cast<ctx *>(N.sepaGetData(sepa));
    if(c.on_exit) c.on_exit(s);
    int maxat = 0, rootcalls = 0;
    for(auto [n, k] : c.calls_at_node) { if(n == 1) rootcalls = k; else maxat = std::max(maxat, k); }
    long multi = 0;
    for(auto [n, k] : c.calls_at_node) multi += k > 1;
    std::string depths;
    for(auto [d, k] : c.calls_at_depth) { if(d > 12) break; depths += std::format(" {}:{}", d, k); }
    std::println(stderr, "   [{}] run={} hook_runs={} lp_not_optimal={} probing={} allowlocal_false={} (at add {}) last_add_depth={} last_allowlocal={} infeasible_flag={} sepa: ncalls={} found={} applied={} | nodes_called={} root_calls={} max_calls_per_nonroot_node={} nodes_called_twice_or_more={} | calls_by_depth{} | pool={} lprows={} total_applied={}",
        c.mode, N.getNRuns(s), c.hook_runs, c.not_optimal, c.in_probing, c.allowlocal_false, c.allowlocal_false_at_add, c.last_add_depth, c.last_allowlocal, c.infeasible_flag,
        N.sepaGetNCalls(sepa), N.sepaGetNCutsFound(sepa), N.sepaGetNCutsApplied(sepa),
        c.calls_at_node.size(), rootcalls, maxat, multi, depths, N.getNPoolCuts(s), N.getNLPRows(s), N.getNCutsApplied(s));
    return RC_OKAY;
}

// Firing as asked by the survey; the extras override expbackoff and the rest.
struct sepa_params { int priority = 1000000, freq = 1, expbackoff = 1; double maxbounddist = 1.0; bool delay = false; };
static ctx * attach_with(M & m, cp::separator sep, const std::string & mode, const sepa_params & p = {}) {
    load_natives(m);
    void * s = m.native_model();
    auto & api = m.native_api();
    auto c = std::make_unique<ctx>();
    c->sep = std::move(sep);
    c->api = &api;
    c->cols = static_cast<int>(m.num_variables());
    c->vars = m.vars();
    c->spec = mode_table.at(mode);
    c->mode = mode;
    int rc = N.includeSepaBasic(s, &c->sepa, "probe", "user cut probe", p.priority, p.freq, p.maxbounddist, 0, p.delay, execlp, nullptr, c.get());
    if(rc != RC_OKAY) { std::println(stderr, "include sepa: retcode {}", rc); std::exit(3); }
    rc = N.setSepaExitsol(s, c->sepa, exitsol);
    if(rc != RC_OKAY) { std::println(stderr, "set exitsol: retcode {}", rc); std::exit(3); }
    if(p.expbackoff > 0) setint(s, "separating/probe/expbackoff", p.expbackoff, api);
    if(c->spec.poolfreq >= 0) setint(s, "separating/poolfreq", c->spec.poolfreq, api);
    keep.push_back(std::move(c));
    return keep.back().get();
}
static void attach(M & m, cp::separator sep, const std::string & mode) { attach_with(m, std::move(sep), mode); }
static void one_thread(M &) {}  // SCIPsolve is single threaded: E5 shows it
static void scope_settings(M & m) {
    load_natives(m);
    void * s = m.native_model();
    auto & api = m.native_api();
    N.setPresolving(s, PS_OFF, 1);
    N.setHeuristics(s, PS_OFF, 1);
    // SCOPE_SEPA / SCOPE_CONFLICT: leave the solver's separators, conflict analysis on.
    if(!std::getenv("SCOPE_SEPA")) N.setSeparating(s, PS_OFF, 1);
    setint(s, "presolving/maxrestarts", 0, api);
    setint(s, "misc/usesymmetry", 0, api);
    if(!std::getenv("SCOPE_CONFLICT")) N.setBoolParam(s, "conflict/enable", 0);
    // SCOPE_INT="a/b=1;c/d=2": integer parameters on top, for the node selection variants.
    if(const char * e = std::getenv("SCOPE_INT")) {
        std::string t(e); std::size_t p = 0;
        while(p < t.size()) {
            auto q = t.find(';', p); if(q == std::string::npos) q = t.size();
            const std::string kv = t.substr(p, q - p); p = q + 1;
            const auto eq = kv.find('=');
            setint(s, kv.substr(0, eq).c_str(), std::stoi(kv.substr(eq + 1)), api);
        }
    }
}

#ifndef PROBE_SCIP_NO_MAIN
int main() {
    { M m; std::println(stderr, "library {} SCIP {}.{}.{}", m.native_api().library_path().string(), m.native_api().majorVersion(), m.native_api().minorVersion(), m.native_api().techVersion()); }
    cp::backend<M> B;
    B.attach = attach; B.scope_settings = scope_settings; B.one_thread = one_thread;
    B.cut_modes = {"row_g", "row_g_force", "row_g_keep", "row_g_keep_force", "row_l", "row_l_force", "row_l_keep", "row_l_keep_force",
                   "pool_g", "pool_g_keep", "pool_g_freq1", "row+pool", "row_force+pool", "row_force+pool_freq1", "pool_l",
                   "cons_g", "cons_g_model", "cons_l", "cons_l_model"};
    if(const char * e = std::getenv("CUT_MODES")) {
        B.cut_modes.clear();
        std::string t(e); std::size_t p = 0;
        while(p <= t.size()) { auto q = t.find(',', p); if(q == std::string::npos) q = t.size(); B.cut_modes.push_back(t.substr(p, q - p)); p = q + 1; }
    }
    cp::run_matrix(B);
}
#endif
