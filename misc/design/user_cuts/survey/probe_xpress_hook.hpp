#pragma once
// The native side of the Xpress probes: hook, cut modes, settings. See REPORT_xpress.md.
#include "mippp/solvers/xpress/all.hpp"
#include "cut_common.hpp"
using M = mippp::xpress_milp;
namespace x = mippp::xpress::impl::v1;
using prob_t = x::XPRSprob;

// xprs.h 47.01; same numbers in 45.01 (checked with grep, see the report)
enum : int {
    C_PRESOLVE = 8011, C_MIPPRESOLVE = 8078, C_PREPROBING = 8238, C_COVERCUTS = 8134, C_GOMCUTS = 8135,
    C_TREECOVERCUTS = 8140, C_TREEGOMCUTS = 8141, C_CUTSTRATEGY = 8138, C_CUTFREQ = 8116, C_AUTOCUTTING = 8446,
    C_HEUREMPHASIS = 8427, C_HEURBEFORELP = 8280, C_FEASIBILITYPUMP = 8193, C_HEURDIVESTRATEGY = 8177,
    C_HEURSEARCHFREQ = 8196, C_PRECOMPONENTS = 8339, C_SYMMETRY = 8118, C_MIPRESTART = 8290,
    C_MIPDUALREDUCTIONS = 8392, C_THREADS = 8278, C_MIPTHREADS = 8079, C_NODESELECTION = 8026, C_BACKTRACK = 8027,
    C_BRANCHCHOICE = 8162, C_MUTEXCALLBACKS = 8210, C_CALLBACKFROMMAINTHREAD = 8090, C_PRESOLVEOPS = 8077,
    C_LOCALCHOICE = 8170, C_SBBEST = 8147, C_OUTPUTLOG = 8035,
    A_LPSTATUS = 1010, A_CURRENTNODE = 1248, A_NODEDEPTH = 1014, A_PARENTNODE = 1027, A_CUTROUNDS = 1121,
    A_COLS = 1018, A_ROWS = 1001, A_INPUTCOLS = 1409, A_ORIGINALCOLS = 1214, A_MIPTHREADID = 1037, A_RESTARTS = 1381,
    A_CUTS = 1012, A_PRESOLVESTATE = 1026, D_PREROOTEFFORT = 7173, D_LPOBJVAL = 2001, LP_OPTIMAL = 1
};
using addcbcutround_t = int(prob_t, void (*)(prob_t, void *, int, int *), void *, int);
using addmanaged_t = int(prob_t, int, int, const char *, const double *, const int *, const int *, const double *);
using presolverow_t = int(prob_t, char, int, const int *, const double *, double, int, int *, int *, double *, double *, int *);
using storecuts_t = int(prob_t, int, int, const int *, const char *, const double *, const int *, void **, const int *, const double *);
using loadcuts_t = int(prob_t, int, int, int, void * const *);
using getcpcutlist_t = int(prob_t, int, int, double, int *, int, void **, double *);
using getcutlist_t = int(prob_t, int, int, int *, int, void **);
using getsol_t = int(prob_t, int *, double *, int, int);
using getlpsol_t = int(prob_t, double *, double *, double *, double *);
using getmap_t = int(prob_t, int *, int *);

struct xp_fns {
    addcbcutround_t * addcbcutround;
    addmanaged_t * addmanaged;
    presolverow_t * presolverow;
    storecuts_t * storecuts;
    loadcuts_t * loadcuts;
    getcpcutlist_t * getcpcutlist;
    getcutlist_t * getcutlist;
    getsol_t * getpresolved;
    getlpsol_t * getlpsol;
    getmap_t * getmap;
    explicit xp_fns(const std::filesystem::path & l)
        : addcbcutround(cp::symf<addcbcutround_t *>(l, "XPRSaddcbcutround"))
        , addmanaged(cp::symf<addmanaged_t *>(l, "XPRSaddmanagedcuts"))
        , presolverow(cp::symf<presolverow_t *>(l, "XPRSpresolverow"))
        , storecuts(cp::symf<storecuts_t *>(l, "XPRSstorecuts"))
        , loadcuts(cp::symf<loadcuts_t *>(l, "XPRSloadcuts"))
        , getcpcutlist(cp::symf<getcpcutlist_t *>(l, "XPRSgetcpcutlist"))
        , getcutlist(cp::symf<getcutlist_t *>(l, "XPRSgetcutlist"))
        , getpresolved(cp::symf<getsol_t *>(l, "XPRSgetcallbackpresolvesolution"))
        , getlpsol(cp::symf<getlpsol_t *>(l, "XPRSgetlpsol"))
        , getmap(cp::symf<getmap_t *>(l, "XPRSgetpresolvemap")) {}
};

enum class hook_kind { optnode, cutround };
enum class cut_kind { addcuts, naive, managed_global, managed_local, pool, pool_global };
enum class point_kind { callback, presolved, lpsol };
struct mode_spec {
    hook_kind hook;
    cut_kind cut;
    point_kind point = point_kind::callback;
    int action_on_cut = -100;  // what *p_action gets when a cut was added, -100 leaves it alone
    int action_first = -100;   // what *p_action gets on the first firing at a node (CUTROUNDS 0) when no cut was added
};
inline mode_spec spec_of(const std::string & mode) {
    using enum hook_kind;
    if(mode == "addcuts") return {optnode, cut_kind::addcuts};
    if(mode == "naive") return {optnode, cut_kind::naive};
    if(mode == "pool") return {optnode, cut_kind::pool};
    if(mode == "pool_global") return {optnode, cut_kind::pool_global};
    if(mode == "managed_global") return {cutround, cut_kind::managed_global, point_kind::callback, 2};
    if(mode == "managed_local") return {cutround, cut_kind::managed_local, point_kind::callback, 2};
    if(mode == "managed_global_noaction") return {cutround, cut_kind::managed_global};
    if(mode == "managed_local_noaction") return {cutround, cut_kind::managed_local};
    if(mode == "managed_global_a1") return {cutround, cut_kind::managed_global, point_kind::callback, 2, 1};
    if(mode == "managed_local_a1") return {cutround, cut_kind::managed_local, point_kind::callback, 2, 1};
    if(mode == "managed_global_a2") return {cutround, cut_kind::managed_global, point_kind::callback, 2, 2};
    if(mode == "managed_local_a2") return {cutround, cut_kind::managed_local, point_kind::callback, 2, 2};
    if(mode == "managed_optnode") return {optnode, cut_kind::managed_global};
    if(mode == "cutround_addcuts") return {cutround, cut_kind::addcuts};
    if(mode == "cutround_addcuts_a2") return {cutround, cut_kind::addcuts, point_kind::callback, 2};
    if(mode == "cutround_naive") return {cutround, cut_kind::naive};
    if(mode == "point_presolved") return {optnode, cut_kind::addcuts, point_kind::presolved};
    if(mode == "point_lpsol") return {optnode, cut_kind::addcuts, point_kind::lpsol};
    std::println(stderr, "unknown mode {}", mode);
    std::exit(2);
}

struct ctx {
    cp::separator sep;
    const x::xpress_api * api;
    prob_t main_prob;
    std::string mode;
    mode_spec spec;
    int cols;
    std::shared_ptr<xp_fns> fn;
    std::mutex m;
    long fired = 0, not_opt = 0, not_avail = 0, cb_is_main = 0, cb_differs = 0, xcuts1 = 0, xcuts0 = 0, loaded_from_pool = 0;
    int max_depth = 0, max_rounds = 0, first_action = -999, cols_in_cb = -1, restarts = 0;
    std::set<int> nodes, worker_ids;
    std::map<int, long> presolve_status, native_errors;
    std::string first_error, first_presolved;
    std::set<void *> stored;
    void error(prob_t cb, int e) {
        native_errors[e]++;
        if(first_error.empty()) {
            char msg[512] = {};
            api->getlasterror(cb, msg);
            first_error = std::format("{}: {}", e, msg);
        }
    }
    std::string line() {
        std::string s = std::format("xp[{}]: fired={} distinct_nodes={} max_depth={} max_cutrounds={} xcuts(0/1)={}/{} first_action={} not_optimal={} not_available={} cbprob_is_main={} cbprob_differs={} worker_ids={} COLS_in_cb={} of input {} restarts={}",
            mode, fired, nodes.size(), max_depth, max_rounds, xcuts0, xcuts1, first_action, not_opt, not_avail, cb_is_main, cb_differs, worker_ids.size(), cols_in_cb, cols, restarts);
        s += " presolverow_status{";
        for(auto [k, v] : presolve_status) s += std::format("{}:{} ", k, v);
        s += "} native_errors{";
        for(auto [k, v] : native_errors) s += std::format("{}:{} ", k, v);
        s += "}";
        if(spec.cut == cut_kind::pool_global) s += std::format(" stored={} loaded_from_pool={}", stored.size(), loaded_from_pool);
        if(!first_error.empty()) s += " first_error=[" + first_error + "]";
        if(!first_presolved.empty()) s += " first_presolved=[" + first_presolved + "]";
        return s;
    }
};
inline std::vector<std::unique_ptr<ctx>> keep;
inline void flush_previous() {
    if(!keep.empty() && keep.back()) { std::println(stderr, "   {}", keep.back()->line()); keep.back().reset(); }
    // kept alive: a re-solve in the harness (P) still calls it
}
// the P experiment re-solves after the next line of output, so keep every context alive
inline std::vector<std::unique_ptr<ctx>> graveyard;

// presolve status s < 0 comes back as 9000 - s, to tell it from a native error
inline int add_cut(ctx & c, prob_t cb, const cp::raw_cut & r) {
    const auto & fn = *c.fn;
    const auto & api = *c.api;
    const int start[2] = {0, static_cast<int>(r.ind.size())};
    const int type = 1;
    int e = 0;
    const auto presolve = [&](std::vector<int> & pi, std::vector<double> & pv, double & prhs, int & status) {
        int cols = 0, n = 0;
        api.getintattrib(cb, A_COLS, &cols);
        pi.resize(static_cast<std::size_t>(cols));
        pv.resize(static_cast<std::size_t>(cols));
        status = -1;
        int err = fn.presolverow(cb, r.sense, start[1], r.ind.data(), r.val.data(), r.rhs, cols, &n, pi.data(), pv.data(), &prhs, &status);
        pi.resize(static_cast<std::size_t>(n));
        pv.resize(static_cast<std::size_t>(n));
        std::lock_guard l(c.m);
        c.presolve_status[status]++;
        if(c.first_presolved.empty()) {
            c.first_presolved = std::format("{} terms in, status {}, {} terms out, rhs {} -> {}:", start[1], status, n, r.rhs, prhs);
            for(std::size_t q = 0; q < pi.size() && q < 6; ++q) c.first_presolved += std::format(" {}*c{}", pv[q], pi[q]);
        }
        return err;
    };
    switch(c.spec.cut) {
        case cut_kind::managed_global:
        case cut_kind::managed_local:
            e = fn.addmanaged(cb, c.spec.cut == cut_kind::managed_global, 1, &r.sense, &r.rhs, start, r.ind.data(), r.val.data());
            break;
        case cut_kind::naive:
            e = api.addcuts(cb, 1, &type, &r.sense, &r.rhs, start, r.ind.data(), r.val.data());
            break;
        case cut_kind::addcuts: {
            std::vector<int> pi; std::vector<double> pv; double prhs = 0; int status = 0;
            e = presolve(pi, pv, prhs, status);
            if(e) break;
            if(status < 0) return 9000 - status;
            const int s2[2] = {0, static_cast<int>(pi.size())};
            e = api.addcuts(cb, 1, &type, &r.sense, &prhs, s2, pi.data(), pv.data());
            break;
        }
        case cut_kind::pool:
        case cut_kind::pool_global: {
            std::vector<int> pi; std::vector<double> pv; double prhs = 0; int status = 0;
            e = presolve(pi, pv, prhs, status);
            if(e) break;
            if(status < 0) return 9000 - status;
            const int s2[2] = {0, static_cast<int>(pi.size())};
            void * h = nullptr;
            e = fn.storecuts(cb, 1, 0, &type, &r.sense, &prhs, s2, &h, pi.data(), pv.data());
            if(e) break;
            if(h) e = fn.loadcuts(cb, -1, -1, 1, &h);
            std::lock_guard l(c.m);
            c.stored.insert(h);
            break;
        }
    }
    if(e) { std::lock_guard l(c.m); c.error(cb, e); }
    return e;
}

inline void hook_body(ctx & c, prob_t cb, int * p_action, const int ifxpresscuts) {
    const auto & api = *c.api;
    int lp = 0, node = 0, depth = 0, rounds = 0, tid = 0, cols = 0, restarts = 0;
    api.getintattrib(cb, A_LPSTATUS, &lp);
    api.getintattrib(cb, A_CURRENTNODE, &node);
    api.getintattrib(cb, A_NODEDEPTH, &depth);
    api.getintattrib(cb, A_CUTROUNDS, &rounds);
    api.getintattrib(cb, A_MIPTHREADID, &tid);
    api.getintattrib(cb, A_COLS, &cols);
    api.getintattrib(cb, A_RESTARTS, &restarts);
    {
        std::lock_guard l(c.m);
        ++c.fired;
        c.nodes.insert(node);
        c.max_depth = std::max(c.max_depth, depth);
        c.max_rounds = std::max(c.max_rounds, rounds);
        c.worker_ids.insert(tid);
        c.cols_in_cb = cols;
        c.restarts = std::max(c.restarts, restarts);
        (cb == c.main_prob ? c.cb_is_main : c.cb_differs)++;
        if(p_action) {
            if(c.first_action == -999) c.first_action = *p_action;
            (ifxpresscuts ? c.xcuts1 : c.xcuts0)++;
        }
        if(lp != LP_OPTIMAL) ++c.not_opt;
    }
    if(lp != LP_OPTIMAL) return;
    std::vector<double> xv(static_cast<std::size_t>(c.cols), 0.0);
    int avail = 1, e = 0;
    switch(c.spec.point) {
        case point_kind::callback: e = api.getcallbacksolution(cb, &avail, xv.data(), 0, c.cols - 1); break;
        case point_kind::presolved: e = c.fn->getpresolved(cb, &avail, xv.data(), 0, std::min(cols, c.cols) - 1); break;
        case point_kind::lpsol: e = c.fn->getlpsol(cb, xv.data(), nullptr, nullptr, nullptr); break;
    }
    if(e || !avail) {
        std::lock_guard l(c.m);
        ++c.not_avail;
        if(e) c.error(cb, e);
        return;
    }
    if(c.spec.cut == cut_kind::pool_global) {
        // emulates a global cut from optnode: load whatever the pool holds and this point violates
        std::vector<void *> viol(4096);
        int n = 0;
        e = c.fn->getcpcutlist(cb, -1, -1, 1e-6, &n, 4096, viol.data(), nullptr);
        if(e) { std::lock_guard l(c.m); c.error(cb, e); }
        else if(n > 0) {
            e = c.fn->loadcuts(cb, -1, -1, std::min(n, 4096), viol.data());
            std::lock_guard l(c.m);
            if(e) c.error(cb, e); else c.loaded_from_pool += n;
        }
    }
    bool added = false;
    try {
        c.sep(xv, [&](const cp::raw_cut & r) {
            const int err = add_cut(c, cb, r);
            if(!err) added = true;
            return err;
        });
    } catch(...) {
        std::lock_guard l(c.m);
        c.first_error += " [exception in separator]";
    }
    if(p_action && added && c.spec.action_on_cut != -100) *p_action = c.spec.action_on_cut;
    if(p_action && !added && rounds == 0 && c.spec.action_first != -100) *p_action = c.spec.action_first;
}
inline void optnode_hook(prob_t cb, void * data, int *) { hook_body(*static_cast<ctx *>(data), cb, nullptr, 0); }
inline void cutround_hook(prob_t cb, void * data, int ifxpresscuts, int * p_action) { hook_body(*static_cast<ctx *>(data), cb, p_action, ifxpresscuts); }

inline std::shared_ptr<xp_fns> fns_of(M & m) {
    static std::shared_ptr<xp_fns> f;
    if(!f) f = std::make_shared<xp_fns>(m.native_api().library_path());
    return f;
}
inline void attach(M & m, cp::separator sep, const std::string & mode) {
    if(!keep.empty() && keep.back()) { std::println(stderr, "   {}", keep.back()->line()); graveyard.push_back(std::move(keep.back())); keep.pop_back(); }
    auto & api = m.native_api();
    auto prob = m.native_model();
    int cols = 0;
    api._check(prob, api.getintattrib(prob, A_INPUTCOLS, &cols));
    if(cols == 0) api._check(prob, api.getintattrib(prob, A_COLS, &cols));
    auto c = std::make_unique<ctx>();
    c->sep = std::move(sep); c->api = &api; c->main_prob = prob; c->mode = mode; c->spec = spec_of(mode);
    c->cols = cols; c->fn = fns_of(m);
    if(c->spec.hook == hook_kind::optnode)
        api._check(prob, api.addcboptnode(prob, optnode_hook, c.get(), 0));
    else
        api._check(prob, c->fn->addcbcutround(prob, cutround_hook, c.get(), 0));
    keep.push_back(std::move(c));
}
inline void flush_last() {
    if(!keep.empty() && keep.back()) { std::println(stderr, "   {}", keep.back()->line()); graveyard.push_back(std::move(keep.back())); keep.pop_back(); }
}
inline void set_int(M & m, const int ctl, const int v) {
    m.native_api()._check(m.native_model(), m.native_api().setintcontrol(m.native_model(), ctl, v));
}
inline void apply_xp_set(M & m) {
    // XP_SET="8078:-1,8011:1": integer controls applied last, to undo one scope setting at a time
    if(const char * s = std::getenv("XP_SET")) {
        std::string t(s);
        std::size_t pos = 0;
        while(pos < t.size()) {
            const auto colon = t.find(':', pos), comma = t.find(',', pos);
            const int ctl = std::atoi(t.substr(pos, colon - pos).c_str());
            const int val = std::atoi(t.substr(colon + 1, comma == std::string::npos ? std::string::npos : comma - colon - 1).c_str());
            set_int(m, ctl, val);
            if(comma == std::string::npos) break;
            pos = comma + 1;
        }
    }
}
inline void one_thread(M & m) { m.set_thread_limit(1); }
// CUTSTRATEGY stays non-zero: XPRSaddmanagedcuts note 3 says managed cuts are not loaded at 0.
// XP_CUTSTRATEGY0=1 sets it to 0 to see that; XP_CUTFREQ1=1 sets CUTFREQ to 1 (tree cutround firing).
inline void scope_settings(M & m) {
    for(auto [c, v] : {std::pair{C_PRESOLVE, 0}, {C_MIPPRESOLVE, 0}, {C_PREPROBING, 0}, {C_COVERCUTS, 0}, {C_GOMCUTS, 0},
                       {C_TREECOVERCUTS, 0}, {C_TREEGOMCUTS, 0}, {C_HEUREMPHASIS, 0}, {C_HEURBEFORELP, 0},
                       {C_FEASIBILITYPUMP, 0}, {C_HEURDIVESTRATEGY, 0}, {C_HEURSEARCHFREQ, 0}, {C_PRECOMPONENTS, 0},
                       {C_SYMMETRY, 0}, {C_MIPRESTART, 0}, {C_MIPDUALREDUCTIONS, 0}, {C_THREADS, 1}, {C_MIPTHREADS, 1}})
        set_int(m, c, v);
    m.native_api()._check(m.native_model(), m.native_api().setdblcontrol(m.native_model(), D_PREROOTEFFORT, 0.0));
    if(std::getenv("XP_CUTSELECT0")) { set_int(m, 8142, 0); set_int(m, 8143, 0); }  // CUTSELECT, TREECUTSELECT
    if(std::getenv("XP_TREECUTS")) { set_int(m, C_TREECOVERCUTS, -1); set_int(m, C_TREEGOMCUTS, -1); }
    if(std::getenv("XP_CUTSTRATEGY0")) set_int(m, C_CUTSTRATEGY, 0);
    if(const char * s = std::getenv("XP_CUTSTRATEGY")) set_int(m, C_CUTSTRATEGY, std::atoi(s));
    if(const char * s = std::getenv("XP_THREADS")) { set_int(m, C_THREADS, std::atoi(s)); set_int(m, C_MIPTHREADS, std::atoi(s)); }
    if(std::getenv("XP_CUTFREQ1")) set_int(m, C_CUTFREQ, 1);
    if(const char * s = std::getenv("XP_NODESEL")) set_int(m, C_NODESELECTION, std::atoi(s));
    if(const char * s = std::getenv("XP_BACKTRACK")) set_int(m, C_BACKTRACK, std::atoi(s));
    if(const char * s = std::getenv("XP_BRANCHCHOICE")) set_int(m, C_BRANCHCHOICE, std::atoi(s));
    apply_xp_set(m);
}
