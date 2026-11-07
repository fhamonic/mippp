// Included by every probe_cplex*.cpp. The native hook and the native settings.
#include "mippp/solvers/cplex/all.hpp"
#include "cut_common.hpp"
using M = mippp::cplex_milp;
namespace c = mippp::cplex::impl::v1;
using ctxptr = c::CPXCALLBACKCONTEXTptr;
using cpxlong = long long;

// cpxconst.h 22.1.2. The wrapper's own CPXCALLBACKINFO stops at BEST_BND, so the
// info queries below are loaded with an int parameter instead of its enum.
enum {
    INFO_THREADID = 0, INFO_NODECOUNT = 1, INFO_ITCOUNT = 2, INFO_BEST_SOL = 3, INFO_BEST_BND = 4,
    INFO_THREADS = 5, INFO_FEASIBLE = 6, INFO_TIME = 7, INFO_DETTIME = 8, INFO_NODEUID = 9,
    INFO_NODEDEPTH = 10, INFO_CANDIDATE_SOURCE = 11, INFO_RESTARTS = 12, INFO_AFTERCUTLOOP = 13, INFO_NODESLEFT = 14
};
enum { USECUT_FORCE = 0, USECUT_PURGE = 1, USECUT_FILTER = 2 };
enum { RELAX_NOSOLVE = 1, STAT_OPTIMAL = 1 };
enum { CUT_USER = 15 };
enum {
    P_PREIND = 1030, P_AGGIND = 1003, P_PREPASS = 1052, P_REDUCE = 1057, P_PRELINEAR = 1058, P_REFORM = 1167,
    P_DEPIND = 1008, P_SCRIND = 1035, P_THREADS = 1067, P_PARALLEL = 1109,
    P_CLIQUES = 2003, P_COEREDIND = 2004, P_COVERS = 2005, P_MIPDISPLAY = 2012, P_NODESEL = 2018, P_BRDIR = 2001,
    P_BNDSTREN = 2029, P_HEURFREQ = 2031, P_CUTSFACTOR = 2033, P_RELAXPREIND = 2034, P_PRESLVND = 2037,
    P_FLOWCOVERS = 2040, P_IMPLBD = 2041, P_PROBE = 2042, P_GUBCOVERS = 2044, P_FRACCUTS = 2049, P_FLOWPATHS = 2051,
    P_MIRCUTS = 2052, P_DISJCUTS = 2053, P_MIPCBREDLP = 2055, P_CUTPASS = 2056, P_SYMMETRY = 2059, P_RINS = 2061,
    P_LBHEUR = 2063, P_REPEATPRESOLVE = 2064, P_FPHEUR = 2098, P_EACHCUTLIM = 2102, P_MIPSEARCH = 2109,
    P_ZEROHALF = 2111, P_HEUREFFORT = 2120, P_MCF = 2134, P_LANDP = 2152, P_NODECUTS = 2157, P_LOCALIMPLBD = 2181,
    P_BQP = 2195, P_RLT = 2196
};

struct syms {
    int (*point)(ctxptr, double *, int, int, double *);
    int (*status)(ctxptr, int *, cpxlong);
    int (*infoint)(ctxptr, int, int *);
    int (*infolong)(ctxptr, int, cpxlong *);
    int (*infodbl)(ctxptr, int, double *);
    int (*locallb)(ctxptr, double *, int, int);
    int (*localub)(ctxptr, double *, int, int);
    int (*globallb)(ctxptr, double *, int, int);
    int (*globalub)(ctxptr, double *, int, int);
    int (*exitcutloop)(ctxptr);
    int (*numcuts)(c::CPXCENVptr, c::CPXCLPptr, int, int *);
    int (*objval)(c::CPXCENVptr, c::CPXCLPptr, double *);
    static const syms & get(const M & m) {
        static const syms s = [&] {
            const auto & p = m.native_api().library_path();
            return syms{
                cp::symf<decltype(syms::point)>(p, "CPXcallbackgetrelaxationpoint"),
                cp::symf<decltype(syms::status)>(p, "CPXcallbackgetrelaxationstatus"),
                cp::symf<decltype(syms::infoint)>(p, "CPXcallbackgetinfoint"),
                cp::symf<decltype(syms::infolong)>(p, "CPXcallbackgetinfolong"),
                cp::symf<decltype(syms::infodbl)>(p, "CPXcallbackgetinfodbl"),
                cp::symf<decltype(syms::locallb)>(p, "CPXcallbackgetlocallb"),
                cp::symf<decltype(syms::localub)>(p, "CPXcallbackgetlocalub"),
                cp::symf<decltype(syms::globallb)>(p, "CPXcallbackgetgloballb"),
                cp::symf<decltype(syms::globalub)>(p, "CPXcallbackgetglobalub"),
                cp::symf<decltype(syms::exitcutloop)>(p, "CPXcallbackexitcutloop"),
                cp::symf<decltype(syms::numcuts)>(p, "CPXgetnumcuts"),
                cp::symf<decltype(syms::objval)>(p, "CPXgetobjval")};
        }();
        return s;
    }
};

inline void param(M & m, int which, int v) {
    auto [env, lp] = m.native_model();
    m.native_api()._check(env, m.native_api().setintparam(env, which, v));
}
inline int get_param(M & m, int which) {
    auto [env, lp] = m.native_model();
    int v = 0;
    m.native_api()._check(env, m.native_api().getintparam(env, which, &v));
    return v;
}
inline void log_if_asked(M & m) {
    if(std::getenv("CUT_LOG")) { param(m, P_SCRIND, 1); param(m, P_MIPDISPLAY, 2); }
}
inline int env_int(const char * name, int fallback) {
    const char * v = std::getenv(name);
    return v ? std::atoi(v) : fallback;
}

// {purgeable, local} of a mode name such as "global-purge"
inline std::pair<int, int> mode_flags(const std::string & mode) {
    const int local = mode.starts_with("local") ? 1 : 0;
    const int purgeable = mode.ends_with("force") ? USECUT_FORCE : mode.ends_with("filter") ? USECUT_FILTER : USECUT_PURGE;
    return {purgeable, local};
}

struct diag {
    std::mutex m;
    std::map<int, long> status_hist;
    long read_errors = 0, after_loop = 0, before_loop = 0, bad_status_err = 0;
    std::set<cpxlong> nodes;
    cpxlong max_depth = 0;
    int restarts = 0;
    std::string line() {
        std::string s = "status_hist:";
        for(auto [k, v] : status_hist) s += std::format(" {}={}", k, v);
        return std::format("{} status_query_errors={} read_errors={} calls_after_cutloop={} calls_in_cutloop={} distinct_nodes={} max_depth={} restarts={}",
                           s, bad_status_err, read_errors, after_loop, before_loop, nodes.size(), max_depth, restarts);
    }
};
struct hook_ctx {
    cp::separator sep;
    const c::cplex_api * api;
    const syms * s;
    int cols, purgeable, local;
    diag d;
};
static std::vector<std::unique_ptr<hook_ctx>> keep;

static int hook(ctxptr context, cpxlong id, void * usr) {
    auto & h = *static_cast<hook_ctx *>(usr);
    if(id != c::CPX_CALLBACKCONTEXT_RELAXATION) return 0;
    int st = -1, after = 0, restarts = 0;
    cpxlong uid = -1, depth = -1;
    const int e = h.s->status(context, &st, RELAX_NOSOLVE);
    h.s->infoint(context, INFO_AFTERCUTLOOP, &after);
    h.s->infoint(context, INFO_RESTARTS, &restarts);
    h.s->infolong(context, INFO_NODEUID, &uid);
    h.s->infolong(context, INFO_NODEDEPTH, &depth);
    {
        std::lock_guard l(h.d.m);
        h.d.status_hist[st]++;
        if(e) h.d.bad_status_err++;
        (after ? h.d.after_loop : h.d.before_loop)++;
        h.d.nodes.insert(uid);
        h.d.max_depth = std::max(h.d.max_depth, depth);
        h.d.restarts = std::max(h.d.restarts, restarts);
    }
    if(e || st != STAT_OPTIMAL) return 0;
    std::vector<double> x(static_cast<std::size_t>(h.cols));
    if(h.s->point(context, x.data(), 0, h.cols - 1, nullptr)) {
        std::lock_guard l(h.d.m);
        h.d.read_errors++;
        return 0;
    }
    h.sep(x, [&](const cp::raw_cut & r) {
        const int beg = 0;
        return h.api->callbackaddusercuts(context, 1, static_cast<int>(r.ind.size()), &r.rhs, &r.sense, &beg,
                                          r.ind.data(), r.val.data(), &h.purgeable, &h.local);
    });
    return 0;
}
inline void flush_diag() {
    if(!keep.empty() && keep.back()->sep) {
        std::println(stderr, "   [diag] {}", keep.back()->d.line());
        keep.back()->sep = {};
    }
}
static void attach(M & m, cp::separator sep, const std::string & mode) {
    flush_diag();
    auto [env, lp] = m.native_model();
    auto & api = m.native_api();
    const auto [purgeable, local] = mode_flags(mode);
    keep.push_back(std::make_unique<hook_ctx>());
    auto & h = *keep.back();
    h.sep = std::move(sep); h.api = &api; h.s = &syms::get(m);
    h.cols = static_cast<int>(m.num_variables());
    h.purgeable = purgeable; h.local = local;
    // no MIP++ callback set: solve() leaves this registration alone
    api._check(env, api.callbacksetfunc(env, lp, c::CPX_CALLBACKCONTEXT_RELAXATION, hook, &h));
    log_if_asked(m);
}
static void one_thread(M & m) { m.set_thread_limit(1); log_if_asked(m); }
// CUT_SEARCH: 0 auto, 1 traditional, 2 dynamic (default 1)
static void scope_settings(M & m) {
    for(auto [p, v] : std::initializer_list<std::pair<int, int>>{
            {P_PREIND, 0}, {P_AGGIND, 0}, {P_PREPASS, 0}, {P_REDUCE, 0}, {P_PRELINEAR, 0}, {P_REFORM, 0}, {P_DEPIND, 0},
            {P_RELAXPREIND, 0}, {P_REPEATPRESOLVE, 0}, {P_PRESLVND, -1}, {P_PROBE, -1}, {P_COEREDIND, 0}, {P_BNDSTREN, 0},
            {P_SYMMETRY, 0}, {P_CLIQUES, -1}, {P_COVERS, -1}, {P_DISJCUTS, -1}, {P_FLOWCOVERS, -1}, {P_FLOWPATHS, -1},
            {P_FRACCUTS, -1}, {P_GUBCOVERS, -1}, {P_IMPLBD, -1}, {P_LANDP, -1}, {P_MCF, -1}, {P_MIRCUTS, -1}, {P_ZEROHALF, -1},
            {P_LOCALIMPLBD, -1}, {P_BQP, -1}, {P_RLT, -1}, {P_HEURFREQ, -1}, {P_RINS, -1}, {P_FPHEUR, -1},
            {P_LBHEUR, 0}, {P_THREADS, 1}, {P_MIPSEARCH, env_int("CUT_SEARCH", 1)}})
        param(m, p, v);
    {   // double-valued
        auto [env, lp] = m.native_model();
        m.native_api()._check(env, m.native_api().setdblparam(env, P_HEUREFFORT, 0.0));
    }
    if(std::getenv("CUT_PRESOLVE")) {  // presolve as it comes by default; still no cuts, no heuristics
        for(auto [p, v] : std::initializer_list<std::pair<int, int>>{{P_PREIND, 1}, {P_AGGIND, -1}, {P_PREPASS, -1}, {P_REDUCE, 3}, {P_REFORM, 3}, {P_DEPIND, -1},
                {P_RELAXPREIND, -1}, {P_REPEATPRESOLVE, -1}, {P_PRESLVND, 0}, {P_PROBE, 0}, {P_COEREDIND, -1}, {P_BNDSTREN, -1}, {P_SYMMETRY, -1}})
            param(m, p, v);
        if(std::atoi(std::getenv("CUT_PRESOLVE")) == 2) {  // the instance's coefficients survive
            param(m, P_PROBE, -1); param(m, P_COEREDIND, 0); param(m, P_BNDSTREN, 0);
        }
    }
    if(const char * extra = std::getenv("CUT_PARAMS")) {  // "1057=3,1167=3"
        std::string e = extra; std::size_t at = 0;
        while(at < e.size()) {
            auto next = e.find(',', at); if(next == std::string::npos) next = e.size();
            const auto item = e.substr(at, next - at); const auto eq = item.find('=');
            param(m, std::atoi(item.substr(0, eq).c_str()), std::atoi(item.substr(eq + 1).c_str()));
            at = next + 1;
        }
    }
    log_if_asked(m);
}
