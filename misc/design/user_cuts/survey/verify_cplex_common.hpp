#pragma once
// Fresh probes for the CPLEX user-cut recipe (independent of probe_cplex_*).
#include "mippp/solvers/cplex/all.hpp"
#include "cut_conformance.hpp"
#include "ilcplex/cpxconst.h"   // after the wrapper: macros would clash with its own names
#include <cstring>
#include <deque>
#include <unordered_map>

using M = mippp::cplex_milp;
namespace cx = mippp::cplex::impl::v1;
using ctxptr = cx::CPXCALLBACKCONTEXTptr;
using cxlong = cx::CPXLONG;

struct vsyms {
    int (*point)(ctxptr, double *, int, int, double *);
    int (*infoint)(ctxptr, int, int *);
    int (*infolong)(ctxptr, int, cxlong *);
    int (*infodbl)(ctxptr, int, double *);
    int (*locallb)(ctxptr, double *, int, int);
    int (*localub)(ctxptr, double *, int, int);
    int (*numcuts)(cx::CPXCENVptr, cx::CPXCLPptr, int, int *);
    int (*rejectcand)(ctxptr, int, int, const double *, const char *, const int *, const int *, const double *);
    int (*candpoint)(ctxptr, double *, int, int, double *);
    int (*abort)(ctxptr);
    int (*getnumcols)(cx::CPXCENVptr, cx::CPXCLPptr);
    static const vsyms & get(const M & m) {
        static const vsyms s = [&] {
            const auto & p = m.native_api().library_path();
            return vsyms{
                cp::symf<decltype(vsyms::point)>(p, "CPXcallbackgetrelaxationpoint"),
                cp::symf<decltype(vsyms::infoint)>(p, "CPXcallbackgetinfoint"),
                cp::symf<decltype(vsyms::infolong)>(p, "CPXcallbackgetinfolong"),
                cp::symf<decltype(vsyms::infodbl)>(p, "CPXcallbackgetinfodbl"),
                cp::symf<decltype(vsyms::locallb)>(p, "CPXcallbackgetlocallb"),
                cp::symf<decltype(vsyms::localub)>(p, "CPXcallbackgetlocalub"),
                cp::symf<decltype(vsyms::numcuts)>(p, "CPXgetnumcuts"),
                cp::symf<decltype(vsyms::rejectcand)>(p, "CPXcallbackrejectcandidate"),
                cp::symf<decltype(vsyms::candpoint)>(p, "CPXcallbackgetcandidatepoint"),
                cp::symf<decltype(vsyms::abort)>(p, "CPXcallbackabort"),
                cp::symf<decltype(vsyms::getnumcols)>(p, "CPXgetnumcols")};
        }();
        return s;
    }
};

// The recipe, each piece a switch so that a negative control can remove it.
struct recipe {
    int purgeable = CPX_USECUT_PURGE;
    int local = 0;
    bool fold = true;              // sanitize: duplicates, zeros, empties
    bool drop_satisfied = true;    // sanitize: ignore a cut the point satisfies
    double satisfied_tol = 1e-9;   // the tolerance cf::sanitize uses by default
    double rel_tol = 0;            // tried: also drop when violation <= rel_tol * ||a||_2
    double inf_tol = 0;            // proposed: also drop when violation <= inf_tol * max|a|
    bool same_point_guard = false; // tried: no separator call on a point identical to this thread's previous one at this node
    bool use_mutex = true;         // around the user separator
    bool abort_on_1121 = false;    // the shipped cbcheck behaviour on any nonzero
    bool skip_post_loop = false;   // ignore CPXCALLBACKINFO_AFTERCUTLOOP=1 calls
    int reform = -1;               // -1: leave Reformulations alone
};
struct vctx {
    cp::separator sep;
    const cx::cplex_api * api;
    const vsyms * s;
    int cols;
    recipe r;
    std::mutex mx;
    std::atomic<long> guard_skips{0}, hook_calls{0}, post_loop_calls{0}, err_1121{0}, err_other{0}, added{0}, dropped_noop{0}, read_err{0}, exceptions{0};
    std::string line() const {
        return std::format("guard_skips={} hook_calls={} post_loop={} added={} noop_dropped={} err1121={} err_other={} read_err={} exceptions={}",
            guard_skips.load(), hook_calls.load(), post_loop_calls.load(), added.load(), dropped_noop.load(), err_1121.load(), err_other.load(), read_err.load(), exceptions.load());
    }
};
inline std::vector<std::unique_ptr<vctx>> & vkeep() { static std::vector<std::unique_ptr<vctx>> k; return k; }

inline int vhook(ctxptr context, cxlong id, void * usr) {
    auto & h = *static_cast<vctx *>(usr);
    if(id != CPX_CALLBACKCONTEXT_RELAXATION) return 0;
    try {
        h.hook_calls.fetch_add(1, std::memory_order_relaxed);
        int after = 0;
        h.s->infoint(context, CPXCALLBACKINFO_AFTERCUTLOOP, &after);
        if(after) h.post_loop_calls.fetch_add(1, std::memory_order_relaxed);
        if(after && h.r.skip_post_loop) return 0;
        std::vector<double> x(static_cast<std::size_t>(h.cols));
        if(h.s->point(context, x.data(), 0, h.cols - 1, nullptr)) { h.read_err.fetch_add(1); return 0; }
        if(h.r.same_point_guard) {
            thread_local std::unordered_map<const vctx *, std::pair<cxlong, std::vector<double>>> last;
            cxlong uid = -1; h.s->infolong(context, CPXCALLBACKINFO_NODEUID, &uid);
            auto & e = last[&h];
            if(e.first == uid && e.second == x) { h.guard_skips.fetch_add(1, std::memory_order_relaxed); return 0; }
            e.first = uid; e.second = x;
        }
        const cp::add_fn add = [&](const cp::raw_cut & raw) -> int {
            cp::raw_cut folded = raw;
            if(h.r.fold) {
                auto r = cf::sanitize(raw, x, h.r.drop_satisfied, h.r.satisfied_tol);
                if(!r) { h.dropped_noop.fetch_add(1, std::memory_order_relaxed); return 0; }
                folded = std::move(*r);
                if(h.r.inf_tol > 0) {
                    double ni = 0; for(double w : folded.val) ni = std::max(ni, std::fabs(w));
                    if(folded.violation(x) <= h.r.inf_tol * ni) { h.dropped_noop.fetch_add(1, std::memory_order_relaxed); return 0; }
                }
                if(h.r.rel_tol > 0) {
                    double n2 = 0; for(double w : folded.val) n2 += w * w;
                    if(folded.violation(x) <= h.r.rel_tol * std::sqrt(n2)) { h.dropped_noop.fetch_add(1, std::memory_order_relaxed); return 0; }
                }
            } else if(h.r.drop_satisfied && raw.violation(x) <= h.r.satisfied_tol) {
                h.dropped_noop.fetch_add(1, std::memory_order_relaxed); return 0;
            }
            const int beg = 0;
            const int e = h.api->callbackaddusercuts(context, 1, static_cast<int>(folded.ind.size()), &folded.rhs, &folded.sense, &beg,
                                                     folded.ind.data(), folded.val.data(), &h.r.purgeable, &h.r.local);
            if(e == 1121) {
                h.err_1121.fetch_add(1, std::memory_order_relaxed);
                if(h.r.abort_on_1121) h.s->abort(context);
            } else if(e) {
                h.err_other.fetch_add(1, std::memory_order_relaxed);
                h.s->abort(context);
            } else h.added.fetch_add(1, std::memory_order_relaxed);
            return e;
        };
        if(h.r.use_mutex) {
            std::lock_guard l(h.mx);
            h.sep(x, add);
        } else h.sep(x, add);
    } catch(...) {
        h.exceptions.fetch_add(1);
        h.s->abort(context);
    }
    return 0;
}

inline void vparam(M & m, int which, int v) {
    auto [env, lp] = m.native_model();
    m.native_api()._check(env, m.native_api().setintparam(env, which, v));
}
inline void vparamd(M & m, int which, double v) {
    auto [env, lp] = m.native_model();
    m.native_api()._check(env, m.native_api().setdblparam(env, which, v));
}
inline vctx & vattach(M & m, cp::separator sep, const recipe & r = {}, const long mask = CPX_CALLBACKCONTEXT_RELAXATION) {
    auto [env, lp] = m.native_model();
    auto & api = m.native_api();
    vkeep().push_back(std::make_unique<vctx>());
    auto & h = *vkeep().back();
    h.sep = std::move(sep); h.api = &api; h.s = &vsyms::get(m);
    h.cols = static_cast<int>(m.num_variables());
    h.r = r;
    if(r.reform >= 0) vparam(m, CPXPARAM_Preprocessing_Reformulations, r.reform);
    api._check(env, api.callbacksetfunc(env, lp, mask, vhook, &h));
    return h;
}
inline int vnumcuts(M & m, int type = CPX_CUT_USER) {
    auto [env, lp] = m.native_model();
    int n = -1;
    vsyms::get(m).numcuts(env, lp, type, &n);
    return n;
}
inline void print_version(M & m) {
    auto [env, lp] = m.native_model();
    std::println(stderr, "CPXversion {} library {}", m.native_api().version(env), m.native_api().library_path().string());
}
// Every cut family off.
inline void all_families_off(M & m) {
    for(int p : {CPX_PARAM_CLIQUES, CPX_PARAM_COVERS, CPX_PARAM_DISJCUTS, CPX_PARAM_FLOWCOVERS, CPX_PARAM_FLOWPATHS, CPX_PARAM_FRACCUTS,
                 CPX_PARAM_GUBCOVERS, CPX_PARAM_IMPLBD, CPX_PARAM_LANDPCUTS, CPX_PARAM_MCFCUTS, CPX_PARAM_MIRCUTS, CPX_PARAM_ZEROHALFCUTS,
                 CPX_PARAM_LOCALIMPLBD, CPX_PARAM_BQPCUTS, CPX_PARAM_RLTCUTS})
        vparam(m, p, -1);
}
inline void scope_settings(M & m, const int search = 1) {
    for(auto [p, v] : std::initializer_list<std::pair<int, int>>{
            {CPX_PARAM_PREIND, 0}, {CPX_PARAM_AGGIND, 0}, {CPX_PARAM_PREPASS, 0}, {CPX_PARAM_REDUCE, 0}, {CPX_PARAM_PRELINEAR, 0},
            {CPXPARAM_Preprocessing_Reformulations, 0}, {CPX_PARAM_DEPIND, 0}, {CPX_PARAM_RELAXPREIND, 0}, {CPX_PARAM_REPEATPRESOLVE, 0},
            {CPX_PARAM_PRESLVND, -1}, {CPX_PARAM_PROBE, -1}, {CPX_PARAM_COEREDIND, 0}, {CPX_PARAM_BNDSTRENIND, 0}, {CPX_PARAM_SYMMETRY, 0},
            {CPX_PARAM_HEURFREQ, -1}, {CPX_PARAM_RINSHEUR, -1}, {CPX_PARAM_FPHEUR, -1}, {CPX_PARAM_LBHEUR, 0}, {CPX_PARAM_MIPSEARCH, search}})
        vparam(m, p, v);
    all_families_off(m);
    vparamd(m, CPXPARAM_MIP_Strategy_HeuristicEffort, 0.0);
}
inline void set_threads(M & m, int t) { vparam(m, CPXPARAM_Threads, t); }
inline std::string info_str(const char * name) { const char * v = std::getenv(name); return v ? v : ""; }
