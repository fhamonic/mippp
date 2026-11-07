#pragma once
// Independent native layer for the SCIP skeptic pass. Nothing here is shared with
// probe_scip*.cpp: own symbol table, own hook, own recipe with one toggle per piece.
#include "mippp/solvers/scip/all.hpp"
#include "cut_conformance.hpp"

namespace vs {
namespace sc = mippp::scip::impl::v1;
using sc::SCIP_VAR;
using sc::SCIP_CONS;
using sc::SCIP_RESULT;
struct SEPA; struct ROW; struct NODE; struct CUT; struct EVENTHDLR; struct EVENT; struct CONSHDLR;
enum { RC_OKAY = 1, RC_ERROR = 0 };
enum { LP_OPTIMAL = 1, LP_UNBOUNDEDRAY = 3 };
using sepa_exec_t = int (*)(void *, SEPA *, SCIP_RESULT *, unsigned, int);

struct natives {
    std::filesystem::path lib;
    int (*includeSepaBasic)(void *, SEPA **, const char *, const char *, int, int, double, unsigned, unsigned, sepa_exec_t, void *, void *);
    int (*sepaSetExitsol)(void *, SEPA *, int (*)(void *, SEPA *));
    void * (*sepaGetData)(SEPA *);
    long long (*sepaGetNCalls)(SEPA *);
    long long (*sepaGetNCutsFound)(SEPA *);
    long long (*sepaGetNCutsApplied)(SEPA *);
    int (*getLPSolstat)(void *);
    int (*createEmptyRowSepa)(void *, ROW **, SEPA *, const char *, double, double, unsigned, unsigned, unsigned);
    int (*addVarsToRow)(void *, ROW *, int, SCIP_VAR **, double *);
    int (*addRow)(void *, ROW *, unsigned, unsigned *);
    int (*addPoolCut)(void *, ROW *);
    int (*releaseRow)(void *, ROW **);
    int (*captureRow)(void *, ROW *);
    int (*rowGetAge)(ROW *);
    unsigned (*rowIsInLP)(ROW *);
    unsigned (*rowIsLocal)(ROW *);
    int (*rowGetNNonz)(ROW *);
    int (*getDepth)(void *);
    NODE * (*getCurrentNode)(void *);
    long long (*nodeGetNumber)(NODE *);
    NODE * (*nodeGetParent)(NODE *);
    int (*nodeGetDepth)(NODE *);
    int (*getNRuns)(void *);
    int (*getNConss)(void *);
    int (*getNPoolCuts)(void *);
    CUT ** (*getPoolCuts)(void *);
    int (*cutGetAge)(CUT *);
    ROW * (*cutGetRow)(CUT *);
    int (*getNLPRows)(void *);
    long long (*getNTotalNodes)(void *);
    long long (*getNLPIterations)(void *);
    double (*getLPObjval)(void *);
    double (*getSolOrigObj)(void *, void *);
    int (*setPresolving)(void *, int, unsigned);
    int (*setHeuristics)(void *, int, unsigned);
    int (*setSeparating)(void *, int, unsigned);
    int (*setBoolParam)(void *, const char *, unsigned);
    int (*setCharParam)(void *, const char *, char);
    int (*getLPRowsData)(void *, ROW ***, int *);
    void ** (*rowGetCols)(ROW *);
    double * (*rowGetVals)(ROW *);
    int (*rowGetNLPNonz)(ROW *);
    double (*rowGetConstant)(ROW *);
    double (*rowGetLhs)(ROW *);
    double (*rowGetRhs)(ROW *);
    const char * (*rowGetName)(ROW *);
    SCIP_VAR * (*colGetVar)(void *);
    unsigned long long (*eventGetType)(EVENT *);
    double (*getCutoffbound)(void *);
    double (*getLocalLowerbound)(void *);
    double (*getLowerbound)(void *);
    double (*transformObj)(void *, double);
    double (*retransformObj)(void *, double);
    double (*getLPColumnObjval)(void *);
    long long (*getNConflictConssFound)(void *);
    long long (*getNConflictConssApplied)(void *);
    void ** (*getConss)(void *);
    const char * (*consGetName)(void *);
    int (*restartSolve)(void *);
    double (*varGetLbGlobal)(SCIP_VAR *);
    double (*varGetUbGlobal)(SCIP_VAR *);
    double (*varGetLbLocal)(SCIP_VAR *);
    double (*varGetUbLocal)(SCIP_VAR *);
    int (*getTransformedVar)(void *, SCIP_VAR *, SCIP_VAR **);
    int (*freeTransform)(void *);
    SCIP_VAR ** (*getVars)(void *);
    int (*getNVars)(void *);
    double (*getVarSol)(void *, SCIP_VAR *);
    int (*getSolVals)(void *, void *, int, SCIP_VAR **, double *);
    double (*infinity)(void *);
    int (*getStage)(void *);
    int (*includeConshdlrBasic)(void *, CONSHDLR **, const char *, const char *, int, int, int, unsigned, void *, void *, void *, void *, void *);
    int (*addVarLocksType)(void *, SCIP_VAR *, int, int, int);
    int (*includeEventhdlrBasic)(void *, EVENTHDLR **, const char *, const char *, int (*)(void *, EVENTHDLR *, EVENT *, void *), void *);
    int (*catchEvent)(void *, unsigned long long, EVENTHDLR *, void *, int *);
    int (*transformProb)(void *);
    int (*majorVersion)();
    int (*minorVersion)();
    int (*techVersion)();
};
inline natives & natives_for(const std::filesystem::path & lib) {
    static std::map<std::filesystem::path, natives> all;
    auto [it, fresh] = all.try_emplace(lib);
    natives & N = it->second;
    if(!fresh) return N;
    N.lib = lib;
    const auto S = [&](auto & f, const char * name) { f = cp::symf<std::remove_reference_t<decltype(f)>>(lib, name); };
    S(N.includeSepaBasic, "SCIPincludeSepaBasic"); S(N.sepaSetExitsol, "SCIPsetSepaExitsol"); S(N.sepaGetData, "SCIPsepaGetData");
    S(N.sepaGetNCalls, "SCIPsepaGetNCalls"); S(N.sepaGetNCutsFound, "SCIPsepaGetNCutsFound"); S(N.sepaGetNCutsApplied, "SCIPsepaGetNCutsApplied");
    S(N.getLPSolstat, "SCIPgetLPSolstat"); S(N.createEmptyRowSepa, "SCIPcreateEmptyRowSepa"); S(N.addVarsToRow, "SCIPaddVarsToRow");
    S(N.addRow, "SCIPaddRow"); S(N.addPoolCut, "SCIPaddPoolCut"); S(N.releaseRow, "SCIPreleaseRow"); S(N.captureRow, "SCIPcaptureRow");
    S(N.rowGetAge, "SCIProwGetAge"); S(N.rowIsInLP, "SCIProwIsInLP"); S(N.rowIsLocal, "SCIProwIsLocal"); S(N.rowGetNNonz, "SCIProwGetNNonz");
    S(N.getDepth, "SCIPgetDepth"); S(N.getCurrentNode, "SCIPgetCurrentNode"); S(N.nodeGetNumber, "SCIPnodeGetNumber"); S(N.nodeGetParent, "SCIPnodeGetParent"); S(N.nodeGetDepth, "SCIPnodeGetDepth");
    S(N.getNRuns, "SCIPgetNRuns"); S(N.getNConss, "SCIPgetNConss"); S(N.getNPoolCuts, "SCIPgetNPoolCuts"); S(N.getPoolCuts, "SCIPgetPoolCuts");
    S(N.cutGetAge, "SCIPcutGetAge"); S(N.cutGetRow, "SCIPcutGetRow"); S(N.getNLPRows, "SCIPgetNLPRows");
    S(N.getNTotalNodes, "SCIPgetNTotalNodes"); S(N.getNLPIterations, "SCIPgetNLPIterations"); S(N.getLPObjval, "SCIPgetLPObjval"); S(N.getSolOrigObj, "SCIPgetSolOrigObj");
    S(N.setPresolving, "SCIPsetPresolving"); S(N.setHeuristics, "SCIPsetHeuristics"); S(N.setSeparating, "SCIPsetSeparating"); S(N.setBoolParam, "SCIPsetBoolParam"); S(N.setCharParam, "SCIPsetCharParam"); S(N.getLPRowsData, "SCIPgetLPRowsData"); S(N.rowGetCols, "SCIProwGetCols"); S(N.rowGetVals, "SCIProwGetVals"); S(N.rowGetNLPNonz, "SCIProwGetNLPNonz"); S(N.rowGetConstant, "SCIProwGetConstant"); S(N.rowGetLhs, "SCIProwGetLhs"); S(N.rowGetRhs, "SCIProwGetRhs"); S(N.rowGetName, "SCIProwGetName"); S(N.colGetVar, "SCIPcolGetVar"); S(N.eventGetType, "SCIPeventGetType"); S(N.getCutoffbound, "SCIPgetCutoffbound"); S(N.getLocalLowerbound, "SCIPgetLocalLowerbound"); S(N.getLowerbound, "SCIPgetLowerbound"); S(N.retransformObj, "SCIPretransformObj"); S(N.transformObj, "SCIPtransformObj"); S(N.getLPColumnObjval, "SCIPgetLPColumnObjval"); S(N.getNConflictConssFound, "SCIPgetNConflictConssFound"); S(N.getNConflictConssApplied, "SCIPgetNConflictConssApplied"); S(N.getConss, "SCIPgetConss"); S(N.consGetName, "SCIPconsGetName");
    S(N.restartSolve, "SCIPrestartSolve"); S(N.varGetLbGlobal, "SCIPvarGetLbGlobal"); S(N.varGetUbGlobal, "SCIPvarGetUbGlobal");
    S(N.varGetLbLocal, "SCIPvarGetLbLocal"); S(N.varGetUbLocal, "SCIPvarGetUbLocal"); S(N.getTransformedVar, "SCIPgetTransformedVar");
    S(N.freeTransform, "SCIPfreeTransform"); S(N.getVars, "SCIPgetVars"); S(N.getNVars, "SCIPgetNVars"); S(N.getVarSol, "SCIPgetVarSol"); S(N.getSolVals, "SCIPgetSolVals");
    S(N.infinity, "SCIPinfinity"); S(N.getStage, "SCIPgetStage"); S(N.includeConshdlrBasic, "SCIPincludeConshdlrBasic"); S(N.addVarLocksType, "SCIPaddVarLocksType");
    S(N.includeEventhdlrBasic, "SCIPincludeEventhdlrBasic"); S(N.catchEvent, "SCIPcatchEvent"); S(N.transformProb, "SCIPtransformProb");
    S(N.majorVersion, "SCIPmajorVersion"); S(N.minorVersion, "SCIPminorVersion"); S(N.techVersion, "SCIPtechVersion");
    return N;
}

// The recipe under test, one field per piece, so a negative control flips one thing.
struct recipe {
    int priority = 1000000, freq = 1, expbackoff = 1;
    double maxbounddist = 1.0;
    bool local = false, modifiable = false, removable = true, force = false;
    bool add_row = true, add_pool = true, pool_guard = true;
    bool lp_filter = true, sanitize = true, drop_satisfied = true, cutoff_result = true, release = true;
    bool result_garbage = false, include_late = false, no_include = false;
    bool read_by_getvars = false, write_by_getvars = false;
    bool harden = false;  // candidate amendment: scale to max|a| = 1, move terms SCIP would drop to the side
};
inline recipe from_env() {
    recipe r;
    const char * e = std::getenv("VERIFY_DROP");
    const std::string d = e ? e : "";
    if(d == "expbackoff") r.expbackoff = 0;
    else if(d == "maxbounddist") r.maxbounddist = 0.0;
    else if(d == "priority") r.priority = -1000000;
    else if(d == "freq") r.freq = 0;
    else if(d == "filter") r.lp_filter = false;
    else if(d == "sanitize") r.sanitize = false;
    else if(d == "drop_satisfied") r.drop_satisfied = false;
    else if(d == "pool") r.add_pool = false;
    else if(d == "row") r.add_row = false;
    else if(d == "oneterm_guard") r.pool_guard = false;
    else if(d == "local") r.local = true;
    else if(d == "local_nopool") { r.local = true; r.add_pool = false; }
    else if(d == "force") r.force = true;
    else if(d == "removable") r.removable = false;
    else if(d == "modifiable") r.modifiable = true;
    else if(d == "cutoff") r.cutoff_result = false;
    else if(d == "result") r.result_garbage = true;
    else if(d == "release") r.release = false;
    else if(d == "include_late") r.include_late = true;
    else if(d == "columns_read") r.read_by_getvars = true;
    else if(d == "columns_write") r.write_by_getvars = true;
    else if(d == "harden") r.harden = true;
    else if(!d.empty()) { std::println(stderr, "unknown VERIFY_DROP {}", d); std::exit(2); }
    return r;
}
inline recipe & G() { static recipe r = from_env(); return r; }

struct plugin {
    natives * n = nullptr;
    const sc::scip_api * api = nullptr;
    recipe r;
    cp::separator sep;
    int cols = 0;
    SCIP_VAR ** vars = nullptr;
    void * s = nullptr;
    SEPA * sepa = nullptr;
    bool attached = false;
    // per call
    bool found = false, infeasible = false;
    // counters
    long entries = 0, not_optimal = 0, rows = 0, pool_adds = 0, sanitized = 0, infeasible_flags = 0, exceptions = 0, add_errors = 0, deep_calls = 0, deep_rows = 0, one_term_rows = 0;
    std::map<int, long> calls_by_depth;
    std::set<long long> nodes_called;
    std::map<int, std::array<int, 3>> run_info;  // run -> conss, pool, lp rows at its first hook call
    // A probe can take over from sep.
    std::function<void(plugin &, const std::vector<double> &, int)> custom;
    void reset_counters() { entries = not_optimal = rows = pool_adds = sanitized = infeasible_flags = exceptions = add_errors = deep_calls = deep_rows = one_term_rows = 0; calls_by_depth.clear(); nodes_called.clear(); run_info.clear(); }

    // Row max|a| = 1, then every term SCIP would ignore (|a| <= 1e-9) moves to the side at its
    // worst bound; nullopt when that bound is infinite. A cut stays valid this way.
    std::optional<cp::raw_cut> harden(const cp::raw_cut & in) {
        double mx = 0;
        for(double v : in.val) mx = std::max(mx, std::fabs(v));
        if(mx == 0) return std::nullopt;
        cp::raw_cut o{{}, {}, in.sense, in.rhs / mx};
        const double inf = n->infinity(s);
        for(std::size_t k = 0; k < in.ind.size(); ++k) {
            const double a = in.val[k] / mx;
            if(std::fabs(a) > 1e-9) { o.ind.push_back(in.ind[k]); o.val.push_back(a); continue; }
            SCIP_VAR * t = nullptr;
            n->getTransformedVar(s, vars[static_cast<std::size_t>(in.ind[k])], &t);
            const double lb = n->varGetLbGlobal(t), ub = n->varGetUbGlobal(t);
            if(in.sense == 'E') return std::nullopt;
            const double worst = in.sense == 'L' ? (a > 0 ? a * lb : a * ub) : (a > 0 ? a * ub : a * lb);
            if(std::fabs(lb) >= inf || std::fabs(ub) >= inf) return std::nullopt;
            o.rhs -= worst;
        }
        return o;
    }
    // zero is fine, as the harness reads it
    static int err(int rc) { return rc == RC_OKAY ? 0 : rc == RC_ERROR ? 1000 : rc; }
    int add(const cp::raw_cut & raw, const std::vector<double> & x, const recipe & rc_, ROW ** keep = nullptr) {
        const cp::raw_cut * c = &raw;
        std::optional<cp::raw_cut> san;
        if(rc_.sanitize) {
            san = cf::sanitize(raw, x, rc_.drop_satisfied);
            if(!san) { ++sanitized; return 0; }
            c = &*san;
        }
        std::optional<cp::raw_cut> hard;
        if(rc_.harden) {
            hard = harden(*c);
            if(!hard) { ++sanitized; return 0; }
            c = &*hard;
        }
        const double inf = n->infinity(s);
        const double lhs = c->sense == 'L' ? -inf : c->rhs;
        const double rhs = c->sense == 'G' ? inf : c->rhs;
        std::vector<SCIP_VAR *> v;
        std::vector<double> a(c->val);
        if(rc_.write_by_getvars) {
            SCIP_VAR ** act = n->getVars(s);
            const int nact = n->getNVars(s);
            for(int j : c->ind) v.push_back(act[j % std::max(nact, 1)]);
        } else {
            for(int j : c->ind) v.push_back(vars[static_cast<std::size_t>(j)]);
        }
        ROW * row = nullptr;
        int rc = n->createEmptyRowSepa(s, &row, sepa, "u", lhs, rhs, rc_.local, rc_.modifiable, rc_.removable);
        if(rc != RC_OKAY) { ++add_errors; return err(rc); }
        rc = n->addVarsToRow(s, row, static_cast<int>(v.size()), v.data(), a.data());
        if(rc != RC_OKAY) { ++add_errors; n->releaseRow(s, &row); return err(rc); }
        unsigned inf_flag = 0;
        if(rc_.add_row) {
            rc = n->addRow(s, row, rc_.force, &inf_flag);
            if(rc != RC_OKAY) { ++add_errors; n->releaseRow(s, &row); return err(rc); }
        }
        if(inf_flag) { infeasible = true; ++infeasible_flags; }
        else if(rc_.add_pool && (!rc_.pool_guard || c->ind.size() > 1)) {
            rc = n->addPoolCut(s, row);
            if(rc != RC_OKAY) { ++add_errors; n->releaseRow(s, &row); return err(rc); }
            ++pool_adds;
        }
        ++rows;
        if(c->ind.size() == 1) ++one_term_rows;
        if(n->getDepth(s) > 0) ++deep_rows;
        found = true;
        if(keep) { n->captureRow(s, row); *keep = row; }
        if(rc_.release) {
            rc = n->releaseRow(s, &row);
            if(rc != RC_OKAY) { ++add_errors; return err(rc); }
        }
        return 0;
    }
};

inline thread_local plugin * tl_cur = nullptr;
inline bool tl_no_restore = false;  // negative control
struct tl_scope {
    plugin * prev;
    explicit tl_scope(plugin * p) : prev(tl_cur) { tl_cur = p; }
    ~tl_scope() { if(!tl_no_restore) tl_cur = prev; }
};

inline int hook(void * s, SEPA * sepa, SCIP_RESULT * result, unsigned, int depth) {
    plugin & p = *tl_cur;
    natives & n = *p.n;
    p.s = s;
    p.sepa = sepa;
    *result = sc::SCIP_DIDNOTRUN;
    ++p.entries;
    if(n.getLPSolstat(s) != LP_OPTIMAL) {
        ++p.not_optimal;
        if(p.r.lp_filter) return RC_OKAY;
    }
    ++p.calls_by_depth[depth];
    if(depth > 0) ++p.deep_calls;
    p.nodes_called.insert(n.nodeGetNumber(n.getCurrentNode(s)));
    const int run = n.getNRuns(s);
    if(!p.run_info.contains(run)) p.run_info[run] = {n.getNConss(s), n.getNPoolCuts(s), n.getNLPRows(s)};
    p.found = p.infeasible = false;
    try {
        std::vector<double> x(static_cast<std::size_t>(p.cols));
        if(p.r.read_by_getvars) {
            SCIP_VAR ** act = n.getVars(s);
            const int nact = n.getNVars(s);
            for(int j = 0; j < p.cols; ++j) x[static_cast<std::size_t>(j)] = j < nact ? n.getVarSol(s, act[j]) : 0.0;
        } else {
            const int rc = n.getSolVals(s, nullptr, p.cols, p.vars, x.data());
            if(rc != RC_OKAY) return rc;
        }
        if(p.custom) p.custom(p, x, depth);
        else if(p.sep) p.sep(x, [&](const cp::raw_cut & c) { return p.add(c, x, p.r); });
    } catch(...) {
        ++p.exceptions;
        return RC_ERROR;
    }
    if(p.r.result_garbage) *result = static_cast<SCIP_RESULT>(0);
    else *result = (p.infeasible && p.r.cutoff_result) ? sc::SCIP_CUTOFF : p.found ? sc::SCIP_SEPARATED : sc::SCIP_DIDNOTFIND;
    return RC_OKAY;
}

// Destroyed after scip_milp, as a member of scip_milp would be: SCIPfree runs first.
struct holder { std::unique_ptr<plugin> P = std::make_unique<plugin>(); };

struct V : holder, mippp::scip_milp {
    using var_t = mippp::scip_milp::variable;
    bool no_freq_write = false;  // negative control
    V() { init(); }
    explicit V(const sc::scip_api & a) : mippp::scip_milp(a) { init(); }
    void init() {
        P->n = &natives_for(native_api().library_path());
        P->api = &native_api();
        P->r = G();
        if(!P->r.include_late && !P->r.no_include) include();
    }
    natives & N() const { return *P->n; }
    void * sp() const { return native_model(); }
    int native_id(variable v) const noexcept { return static_cast<int>(v.uid()); }
    SCIP_VAR * var_of(int i) const noexcept { return _scip_vars[static_cast<std::size_t>(i)]; }
    SCIP_VAR ** vars() noexcept { return _scip_vars.data(); }
    std::vector<SCIP_VAR *> copy_vars() const { return _scip_vars; }
    void include() {
        const int rc = N().includeSepaBasic(sp(), &P->sepa, "mippp", "user cuts", P->r.priority, -1, P->r.maxbounddist, 0, 0, hook, nullptr, P.get());
        if(rc != RC_OKAY) { std::println(stderr, "include: retcode {}", rc); std::exit(3); }
        if(P->r.expbackoff > 0) seti("separating/mippp/expbackoff", P->r.expbackoff);
    }
    int seti(const char * p, int v) {
        const int rc = native_api().setIntParam(native_model(), p, v);
        if(rc != RC_OKAY) std::println(stderr, "   setIntParam {} = {} -> retcode {} at stage {}", p, v, rc, N().getStage(sp()));
        return rc;
    }
    void attach(cp::separator sep) {
        P->sep = std::move(sep);
        P->attached = true;
        if(P->r.include_late && !P->sepa) {
            const int rc = N().includeSepaBasic(sp(), &P->sepa, "mippp", "user cuts", P->r.priority, -1, P->r.maxbounddist, 0, 0, hook, nullptr, P.get());
            std::println(stderr, "   late include at stage {}: retcode {}", N().getStage(sp()), rc);
            if(P->r.expbackoff > 0) seti("separating/mippp/expbackoff", P->r.expbackoff);
        }
    }
    void attach_custom(std::function<void(plugin &, const std::vector<double> &, int)> f) { P->custom = std::move(f); P->attached = true; }
    void solve() {
        tl_scope scope(P.get());
        P->cols = static_cast<int>(num_variables());
        P->vars = _scip_vars.data();
        if(P->sepa && !no_freq_write) seti("separating/mippp/freq", P->attached ? P->r.freq : -1);
        mippp::scip_milp::solve();
        if(std::getenv("VERIFY_V")) std::println(stderr, "   {}", line());
    }
    std::string line() {
        std::string runs;
        for(auto & [r, i] : P->run_info) runs += std::format(" r{}:conss={},pool={},lprows={}", r, i[0], i[1], i[2]);
        long long ncalls = -1, found = -1, applied = -1;
        // SCIP_STAGE_SOLVING = 9: the pools do not exist before it, a stop in presolve leaves them null
        if(N().getStage(sp()) < 9) return std::format("[V] stopped at stage {} before solving", N().getStage(sp()));
        if(P->sepa) { ncalls = N().sepaGetNCalls(P->sepa); found = N().sepaGetNCutsFound(P->sepa); applied = N().sepaGetNCutsApplied(P->sepa); }
        return std::format("[V] runs={} conss_end={} nodes={} lpiter={} entries={} not_optimal={} deep_calls={} rows={} deep_rows={} one_term={} pool_adds={} sanitized={} infeasible_flags={} add_errors={} exceptions={} sepa(calls={} found={} applied={}) pool_end={} |{}",
            N().getNRuns(sp()), N().getNConss(sp()), N().getNTotalNodes(sp()), N().getNLPIterations(sp()), P->entries, P->not_optimal, P->deep_calls, P->rows, P->deep_rows, P->one_term_rows, P->pool_adds, P->sanitized, P->infeasible_flags, P->add_errors, P->exceptions, ncalls, found, applied, N().getNPoolCuts(sp()), runs);
    }
    void set_constraint_rhs(constraint c, double r) { set_constraint_upper_bound(c, r); }
    double get_constraint_rhs(constraint c) { return get_constraint_upper_bound(c); }
    std::string version() { return std::format("{}.{}.{}", N().majorVersion(), N().minorVersion(), N().techVersion()); }
};
}  // namespace vs
