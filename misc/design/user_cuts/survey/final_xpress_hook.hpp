#pragma once
// final: verify_xpress_hook.hpp with cf::sanitize and the split replaced by fp::prepare.
// The skeptic's own Xpress hook. One recipe struct; each negative control is one flag.
// Native codes come back as return values: nothing throws through the C frames.
#include "mippp/solvers/xpress/all.hpp"
#include "cut_conformance.hpp"
#include "final_prepare.hpp"
#include <sstream>
using M = mippp::xpress_milp;
namespace xn = mippp::xpress::impl::v1;
using prob_t = xn::XPRSprob;
namespace vx {
// xprs.h 47.01.01
enum : int {
    C_PRESOLVE = 8011, C_MIPPRESOLVE = 8078, C_COVERCUTS = 8134, C_GOMCUTS = 8135, C_TREECOVERCUTS = 8140, C_TREEGOMCUTS = 8141,
    C_CUTSTRATEGY = 8138, C_CUTFREQ = 8116, C_HEUREMPHASIS = 8427, C_HEURBEFORELP = 8280, C_FEASIBILITYPUMP = 8193,
    C_HEURDIVESTRATEGY = 8177, C_HEURSEARCHFREQ = 8196, C_HEURFREQ = 8155, C_PRECOMPONENTS = 8339, C_SYMMETRY = 8118, C_MIPRESTART = 8290,
    C_MIPDUALREDUCTIONS = 8392, C_THREADS = 8278, C_MIPTHREADS = 8079, C_MUTEXCALLBACKS = 8210, C_OUTPUTLOG = 8035,
    A_LPSTATUS = 1010, A_CURRENTNODE = 1248, A_NODEDEPTH = 1014, A_CUTROUNDS = 1121, A_COLS = 1018, A_ROWS = 1001, A_INPUTCOLS = 1409,
    A_MIPTHREADID = 1037, A_RESTARTS = 1381, A_CUTS = 1012, A_NODES = 1013, A_MIPSTATUS = 1011, D_LPOBJVAL = 2001, LP_OPTIMAL = 1, STOP_USER = 9
};
using addcbcutround_t = int(prob_t, void (*)(prob_t, void *, int, int *), void *, int);
using removecbcutround_t = int(prob_t, void (*)(prob_t, void *, int, int *), void *);
using addmanaged_t = int(prob_t, int, int, const char *, const double *, const int *, const int *, const double *);
using presolverow_t = int(prob_t, char, int, const int *, const double *, double, int, int *, int *, double *, double *, int *);
using getlpsol_t = int(prob_t, double *, double *, double *, double *);
using getsol_t = int(prob_t, int *, double *, int, int);
using versionnumbers_t = int(int *, int *, int *);
struct fns {
    addcbcutround_t * addcbcutround; removecbcutround_t * removecbcutround; addmanaged_t * addmanaged; presolverow_t * presolverow; getlpsol_t * getlpsol; getsol_t * getpresolved;
    explicit fns(const std::filesystem::path & l)
        : addcbcutround(cp::symf<addcbcutround_t *>(l, "XPRSaddcbcutround")), removecbcutround(cp::symf<removecbcutround_t *>(l, "XPRSremovecbcutround"))
        , addmanaged(cp::symf<addmanaged_t *>(l, "XPRSaddmanagedcuts")), presolverow(cp::symf<presolverow_t *>(l, "XPRSpresolverow"))
        , getlpsol(cp::symf<getlpsol_t *>(l, "XPRSgetlpsol")), getpresolved(cp::symf<getsol_t *>(l, "XPRSgetcallbackpresolvesolution")) {}
};

struct recipe {
    std::string name;
    bool cutround_hook = true;                  // false: optnode
    enum { managed, presolved_addcuts, naive_addcuts } cut = managed;
    int globalvalid = 1;
    bool folds = true;                          // cf::sanitize at all
    bool drop_satisfied = true;
    bool split_equality = true;                 // 'E' becomes L + G
    bool use_main_prob = false;
    int point = 0;                              // 0 getcallbacksolution, 1 getlpsol, 2 getcallbackpresolvesolution
    bool point_size_cols = false;               // XPRS_COLS instead of XPRS_INPUTCOLS
    bool ignore_presolve_status = false;
    bool lp_status_check = true;
    bool use_mutex = true;
    int action_on_add = -100;                   // -100 leaves p_action alone
    int action_first = -100;                    // on the first firing at each node (CUTROUNDS==0), when nothing was added
    double scale = 1.0;                         // every cut multiplied by this just before the native call
    double tol = 1e-9;                          // cf::sanitize's tolerance on the violation
    bool drop_relaxed = false;                  // presolverow status 1 (the row was relaxed): do not add it
    bool rel_tol = false;                       // tol times max(1, largest |coefficient|)
};
inline recipe parse(std::string s) {
    recipe r; r.name = s;
    std::stringstream ss(s); std::string f;
    while(std::getline(ss, f, ',')) {
        if(f == "R1") { r.cutround_hook = true; r.cut = recipe::managed; }
        else if(f == "R2") { r.cutround_hook = false; r.cut = recipe::presolved_addcuts; }
        else if(f == "R2cr") { r.cutround_hook = true; r.cut = recipe::presolved_addcuts; }  // fallback: same cutround hook
        else if(f == "local") r.globalvalid = 0;
        else if(f == "no_presolverow") r.cut = recipe::naive_addcuts;
        else if(f == "no_sanitize") { r.folds = false; r.drop_satisfied = false; }
        else if(f == "no_folding") r.folds = false;
        else if(f == "keep_satisfied") r.drop_satisfied = false;
        else if(f == "no_split_eq") r.split_equality = false;
        else if(f == "main_prob") r.use_main_prob = true;
        else if(f == "point_lpsol") r.point = 1;
        else if(f == "point_presolved") r.point = 2;
        else if(f == "point_cols") r.point_size_cols = true;
        else if(f == "ignore_status") r.ignore_presolve_status = true;
        else if(f == "no_lpstatus") r.lp_status_check = false;
        else if(f == "no_mutex") r.use_mutex = false;
        else if(f == "drop_relaxed") r.drop_relaxed = true;
        else if(f == "add2") r.action_on_add = 2;
        else if(f == "first2") r.action_first = 2;
        else if(f == "add0") r.action_on_add = 0;
        else if(f == "managed_optnode") { r.cutround_hook = false; r.cut = recipe::managed; }
        else if(f.starts_with("scale=")) r.scale = std::stod(f.substr(6));
        else if(f.starts_with("tol=")) r.tol = std::stod(f.substr(4));
        else if(f.starts_with("reltol=")) { r.tol = std::stod(f.substr(7)); r.rel_tol = true; }
        else { std::println(stderr, "unknown recipe flag {}", f); std::exit(2); }
    }
    return r;
}

struct ctx {
    cp::separator sep;
    const xn::xpress_api * api = nullptr;
    prob_t main_prob = nullptr;
    recipe r;
    std::shared_ptr<fns> fn;
    std::mutex mx;
    std::atomic<long> fired{0}, not_opt{0}, not_avail{0}, dropped{0}, cb_is_main{0}, cb_differs{0}, presolve_empty{0}, read_err{0}, exceptions{0}, added_native{0};
    std::map<int, long> presolve_status, errors;
    std::set<int> nodes;
    int max_rounds = 0, max_depth = 0, cols_seen_lo = 1 << 30, cols_seen_hi = 0, input_cols = 0, restarts = 0, rows_first = -1, rows_max = 0, cuts_max = 0;
    std::string first_error;
    std::string line() {
        std::string s = std::format("xp[{}]: fired={} nodes={} max_depth={} max_rounds={} added_native={} dropped_by_sanitize={} not_optimal={} not_available={} read_err={} main/worker={}/{} COLS=[{},{}] input={} restarts={} presolve_empty={} exceptions={} rows_first={} rows_max={} CUTS_max={} status{{", r.name, fired.load(), nodes.size(), max_depth, max_rounds, added_native.load(), dropped.load(), not_opt.load(), not_avail.load(), read_err.load(), cb_is_main.load(), cb_differs.load(), cols_seen_lo, cols_seen_hi, input_cols, restarts, presolve_empty.load(), exceptions.load(), rows_first, rows_max, cuts_max);
        for(auto [k, v] : presolve_status) s += std::format("{}:{} ", k, v);
        s += "} errors{";
        for(auto [k, v] : errors) s += std::format("{}:{} ", k, v);
        s += "}";
        if(!first_error.empty()) s += " first_error=[" + first_error + "]";
        return s;
    }
};
inline std::vector<std::unique_ptr<ctx>> & contexts() { static std::vector<std::unique_ptr<ctx>> v; return v; }
inline bool quiet_flush = false;
inline thread_local int current_depth = 0;
inline void flush_stats() {
    static std::size_t printed = 0;
    for(; printed < contexts().size(); ++printed) if(!quiet_flush) std::println(stderr, "   {}", contexts()[printed]->line());
}

inline int native_add(ctx & c, prob_t cb, const cp::raw_cut & cut) {
    const auto & api = *c.api;
    const int start[2] = {0, static_cast<int>(cut.ind.size())};
    const int type = 1;
    std::vector<double> sv = cut.val;
    double srhs = cut.rhs;
    if(c.r.scale != 1.0) { for(auto & v : sv) v *= c.r.scale; srhs *= c.r.scale; }
    int e = 0;
    switch(c.r.cut) {
        case recipe::managed:
            e = c.fn->addmanaged(cb, c.r.globalvalid, 1, &cut.sense, &srhs, start, cut.ind.data(), sv.data());
            break;
        case recipe::naive_addcuts:
            e = api.addcuts(cb, 1, &type, &cut.sense, &srhs, start, cut.ind.data(), sv.data());
            break;
        case recipe::presolved_addcuts: {
            int cols = 0, n = 0, status = -99;
            api.getintattrib(cb, A_COLS, &cols);
            std::vector<int> pi(static_cast<std::size_t>(cols) + 1);
            std::vector<double> pv(static_cast<std::size_t>(cols) + 1);
            double prhs = 0;
            e = c.fn->presolverow(cb, cut.sense, start[1], cut.ind.data(), sv.data(), srhs, cols, &n, pi.data(), pv.data(), &prhs, &status);
            { std::lock_guard l(c.mx); c.presolve_status[status]++; }
            if(e) break;
            if(status < 0 && !c.r.ignore_presolve_status) return 0;  // the cut may be dropped: the contract allows it
            if(status == 1 && c.r.drop_relaxed) return 0;
            if(n == 0) c.presolve_empty.fetch_add(1);
            const int s2[2] = {0, n};
            e = api.addcuts(cb, 1, &type, &cut.sense, &prhs, s2, pi.data(), pv.data());
            break;
        }
    }
    if(!e) c.added_native.fetch_add(1);
    return e;
}
inline int add_cut(ctx & c, prob_t cb, const cp::raw_cut & raw, const std::vector<double> & x, bool & added) {
    fp::options o = fp::opts();
    o.split_eq = c.r.split_equality;
    const auto prepared = fp::prepare(raw, x, o);
    if(prepared.empty()) { c.dropped.fetch_add(1); return 0; }
    int e = 0;
    for(const auto & r : prepared) { e = native_add(c, cb, r); if(e) break; }
    if(e) {
        std::lock_guard l(c.mx);
        c.errors[e]++;
        if(c.first_error.empty()) { char msg[512] = {}; c.api->getlasterror(cb, msg); c.first_error = std::format("{}: {}", e, msg); }
    } else added = true;
    return e;
}
inline void body(ctx & c, prob_t cbprob, int * p_action) {
    const auto & api = *c.api;
    const prob_t cb = c.r.use_main_prob ? c.main_prob : cbprob;
    (cbprob == c.main_prob ? c.cb_is_main : c.cb_differs).fetch_add(1);
    int lp = 0, node = 0, depth = 0, rounds = 0, cols = 0, input = 0, restarts = 0, rows = 0, ncuts = 0;
    api.getintattrib(cb, A_ROWS, &rows);
    api.getintattrib(cb, A_CUTS, &ncuts);
    api.getintattrib(cb, A_LPSTATUS, &lp);
    api.getintattrib(cb, A_CURRENTNODE, &node);
    api.getintattrib(cb, A_NODEDEPTH, &depth);
    api.getintattrib(cb, A_CUTROUNDS, &rounds);
    api.getintattrib(cb, A_COLS, &cols);
    api.getintattrib(cb, A_INPUTCOLS, &input);
    api.getintattrib(cb, A_RESTARTS, &restarts);
    {
        std::lock_guard l(c.mx);
        c.nodes.insert(node);
        c.max_rounds = std::max(c.max_rounds, rounds);
        c.max_depth = std::max(c.max_depth, depth);
        c.cols_seen_lo = std::min(c.cols_seen_lo, cols);
        c.cols_seen_hi = std::max(c.cols_seen_hi, cols);
        c.input_cols = input;
        c.restarts = std::max(c.restarts, restarts);
        if(c.rows_first < 0) c.rows_first = rows;
        c.rows_max = std::max(c.rows_max, rows);
        c.cuts_max = std::max(c.cuts_max, ncuts);
    }
    c.fired.fetch_add(1);
    current_depth = depth;
    if(c.r.lp_status_check && lp != LP_OPTIMAL) { c.not_opt.fetch_add(1); return; }
    const int size = c.r.point_size_cols ? cols : input;
    std::vector<double> x(static_cast<std::size_t>(size));
    int avail = 1, e = 0;
    if(c.r.point == 0) e = api.getcallbacksolution(cb, &avail, x.data(), 0, size - 1);
    else if(c.r.point == 1) e = c.fn->getlpsol(cb, x.data(), nullptr, nullptr, nullptr);
    else e = c.fn->getpresolved(cb, &avail, x.data(), 0, size - 1);
    if(e) { c.read_err.fetch_add(1); std::lock_guard l(c.mx); c.errors[e]++; if(c.first_error.empty()) { char msg[512] = {}; api.getlasterror(cb, msg); c.first_error = std::format("{}: {}", e, msg); } return; }
    if(!avail) { c.not_avail.fetch_add(1); return; }
    if(x.size() < static_cast<std::size_t>(input)) x.resize(static_cast<std::size_t>(input), 0.0);  // so a too short read is a wrong point, not a crash in the harness
    bool added = false;
    c.sep(x, [&](const cp::raw_cut & raw) { return add_cut(c, cb, raw, x, added); });
    if(p_action) {
        if(added && c.r.action_on_add != -100) *p_action = c.r.action_on_add;
        else if(!added && c.r.action_first != -100 && rounds == 0) *p_action = c.r.action_first;
    }
}
inline void guarded(ctx & c, prob_t cb, int * p_action) {
    try {
        std::unique_lock<std::mutex> l;
        static std::mutex serial;
        if(c.r.use_mutex) l = std::unique_lock<std::mutex>(serial);
        body(c, cb, p_action);
    } catch(...) {
        c.exceptions.fetch_add(1);
        c.api->interrupt(cb, STOP_USER);
    }
}
inline void hook_cutround(prob_t cb, void * d, int, int * p_action) { guarded(*static_cast<ctx *>(d), cb, p_action); }
inline void hook_optnode(prob_t cb, void * d, int *) { guarded(*static_cast<ctx *>(d), cb, nullptr); }

// Re-points: every addcb is additive and C8 attaches twice on one model.
inline ctx & attach(M & m, cp::separator sep, const recipe & r) {
    flush_stats();
    const prob_t prob = m.native_model();
    const auto & api = m.native_api();
    auto fn = std::make_shared<fns>(api.library_path());
    fn->removecbcutround(prob, hook_cutround, nullptr);
    api.removecboptnode(prob, hook_optnode, nullptr);
    contexts().push_back(std::make_unique<ctx>());
    ctx & c = *contexts().back();
    c.sep = std::move(sep); c.api = &api; c.main_prob = prob; c.r = r; c.fn = fn;
    const int e = r.cutround_hook ? fn->addcbcutround(prob, hook_cutround, &c, 0) : api.addcboptnode(prob, hook_optnode, &c, 0);
    if(e) { std::println(stderr, "add callback failed {}", e); std::exit(2); }
    return c;
}
inline void setint(M & m, int control, int v) { const int e = m.native_api().setintcontrol(m.native_model(), control, v); if(e) { std::println(stderr, "setintcontrol {} failed {}", control, e); std::exit(2); } }
inline std::string version(M & m) { char t[64] = {}; m.native_api().getversion(t); return t; }
}  // namespace vx
