#pragma once
// Gurobi side of the user-cut survey: the native hook, its modes, and the counters
// the shared header does not keep. Included by probe_gurobi_matrix.cpp and
// probe_gurobi_extra.cpp. Constants are from gurobi_c.h 11.0.3, 12.0.1, 13.0.2,
// checked equal by grep.
#include "mippp/solvers/gurobi/all.hpp"
#include "cut_common.hpp"
using M = mippp::gurobi_milp;
namespace grb = mippp::gurobi::impl::v1;

enum : int {
    W_POLLING = 0, W_MIP = 3, W_MIPSOL = 4, W_MIPNODE = 5,
    SOL_SOL = 4001, SOL_OBJ = 4002, SOL_NODCNT = 4005,
    NODE_STATUS = 5001, NODE_REL = 5002, NODE_OBJBST = 5003, NODE_OBJBND = 5004,
    NODE_NODCNT = 5005, NODE_SOLCNT = 5006, NODE_BRVAR = 5007, NODE_PHASE = 5009,
    GRB_OPTIMAL_ = 2
};

// How a cut reaches the solver, and what goes on the env first.
struct mode_spec {
    bool lazy = false;   // GRBcblazy instead of GRBcbcut
    int precrush = 0;    // PreCrush
    int lazyparam = 0;   // LazyConstraints
};
inline mode_spec spec_of(const std::string & mode) {
    if(mode == "cbcut_precrush") return {false, 1, 0};
    if(mode == "cbcut_noprecrush") return {false, 0, 0};
    if(mode == "cblazy_mipnode") return {true, 0, 1};
    if(mode == "cbcut_precrush_lazyparam") return {false, 1, 1};
    if(mode == "cbcut_noprecrush_lazyparam") return {false, 0, 1};
    if(mode == "cblazy_noparam") return {true, 0, 0};
    std::println(stderr, "unknown mode {}", mode);
    std::exit(2);
}
inline char sense_of(char s) { return s == 'E' ? '=' : s == 'L' ? '<' : '>'; }

// The status filter, where in the tree the calls fall, and whether a call repeats
// the node (NODCNT unchanged) after a cut or after no cut.
struct hook_ctx {
    cp::separator sep;
    const grb::gurobi_api * api = nullptr;
    int cols = 0;
    bool lazy = false;
    std::string label;
    std::shared_ptr<double> nodcnt_out;
    std::map<int, long> status_hist, phase_hist, calls_per_nodcnt_hist;
    long raw_fires = 0, root_calls = 0, tree_calls = 0, repeat_after_cut = 0, repeat_after_nocut = 0,
         next_after_cut = 0, next_after_nocut = 0, root_cuts = 0, tree_cuts = 0, cut_errors = 0, run_at_nodcnt = 0;
    double last_nodcnt = -1, max_nodcnt = 0;
    std::set<double> distinct_nodcnt;
    bool last_added = false, printed = false;
    int first_error = 0;

    void close_run() {
        if(run_at_nodcnt > 0) calls_per_nodcnt_hist[static_cast<int>(std::min(run_at_nodcnt, 50L))]++;
        run_at_nodcnt = 0;
    }
    std::string line() {
        close_run();
        std::string st, ph, hist;
        for(auto [k, v] : status_hist) st += std::format(" {}:{}", k, v);
        for(auto [k, v] : phase_hist) ph += std::format(" {}:{}", k, v);
        for(auto [k, v] : calls_per_nodcnt_hist) hist += std::format(" {}x{}", k, v);
        return std::format(
            "[{}] fires={} status{{{} }} phase{{{} }} root_calls={} tree_calls={} root_cuts={} tree_cuts={} distinct_nodcnt={} max_nodcnt={} calls_per_nodcnt(calls x runs){{{} }} after_cut: same_nodcnt={} next={}; after_no_cut: same_nodcnt={} next={}; cut_errors={} first_native_error={}",
            label, raw_fires, st, ph, root_calls, tree_calls, root_cuts, tree_cuts, distinct_nodcnt.size(), max_nodcnt, hist,
            repeat_after_cut, next_after_cut, repeat_after_nocut, next_after_nocut, cut_errors, first_error);
    }
};

inline int cut_hook(grb::GRBmodel *, void * cbdata, int where, void * usr) {
    auto & c = *static_cast<hook_ctx *>(usr);
    if(where != W_MIPNODE) return 0;
    ++c.raw_fires;
    int status = 0, phase = -1;
    double nodcnt = 0;
    c.api->cbget(cbdata, where, NODE_STATUS, &status);
    c.api->cbget(cbdata, where, NODE_NODCNT, &nodcnt);
    c.api->cbget(cbdata, where, NODE_PHASE, &phase);
    c.status_hist[status]++;
    c.phase_hist[phase]++;
    if(status != GRB_OPTIMAL_) return 0;
    std::vector<double> x(static_cast<std::size_t>(c.cols));
    const int rel_err = c.api->cbget(cbdata, where, NODE_REL, x.data());
    if(rel_err) { if(!c.first_error) c.first_error = rel_err; return 0; }
    (nodcnt == 0 ? c.root_calls : c.tree_calls)++;
    if(c.last_nodcnt >= 0) {
        const bool same = nodcnt == c.last_nodcnt;
        if(c.last_added) (same ? c.repeat_after_cut : c.next_after_cut)++;
        else (same ? c.repeat_after_nocut : c.next_after_nocut)++;
    }
    if(nodcnt != c.last_nodcnt) c.close_run();
    ++c.run_at_nodcnt;
    c.last_nodcnt = nodcnt;
    c.distinct_nodcnt.insert(nodcnt);
    c.max_nodcnt = std::max(c.max_nodcnt, nodcnt);
    if(c.nodcnt_out) *c.nodcnt_out = nodcnt;
    bool added = false;
    c.sep(x, [&](const cp::raw_cut & r) {
        const int n = static_cast<int>(r.ind.size());
        const char s = sense_of(r.sense);
        const int e = c.lazy ? c.api->cblazy(cbdata, n, r.ind.data(), r.val.data(), s, r.rhs)
                             : c.api->cbcut(cbdata, n, r.ind.data(), r.val.data(), s, r.rhs);
        if(e) { ++c.cut_errors; if(!c.first_error) c.first_error = e; }
        else { added = true; (nodcnt == 0 ? c.root_cuts : c.tree_cuts)++; }
        return e;
    });
    c.last_added = added;
    return 0;
}

inline std::vector<std::unique_ptr<hook_ctx>> keep_ctx;
inline void flush_ctx() {
    if(!keep_ctx.empty() && !keep_ctx.back()->printed) {
        std::println(stderr, "   {}", keep_ctx.back()->line());
        keep_ctx.back()->printed = true;
    }
}
inline void set_int(M & m, const char * name, int v) {
    auto [env, model] = m.native_model();
    m.native_api()._check(env, m.native_api().setintparam(env, name, v));
}
inline void set_dbl(M & m, const char * name, double v) {
    auto [env, model] = m.native_model();
    m.native_api()._check(env, m.native_api().setdblparam(env, name, v));
}
inline hook_ctx * attach_hook(M & m, cp::separator sep, const std::string & mode) {
    flush_ctx();
    const mode_spec s = spec_of(mode);
    auto [env, model] = m.native_model();
    auto & api = m.native_api();
    set_int(m, "PreCrush", s.precrush);
    set_int(m, "LazyConstraints", s.lazyparam);
    auto c = std::make_unique<hook_ctx>();
    c->sep = std::move(sep);
    c->api = &api;
    c->cols = static_cast<int>(m.num_variables());
    c->lazy = s.lazy;
    c->label = mode;
    keep_ctx.push_back(std::move(c));
    api._check(env, api.setcallbackfunc(model, cut_hook, keep_ctx.back().get()));
    return keep_ctx.back().get();
}
inline void one_thread(M & m) { m.set_thread_limit(1); }
inline void scope_settings(M & m) {
    set_int(m, "Presolve", 0);
    set_int(m, "Cuts", 0);
    set_dbl(m, "Heuristics", 0.0);
    set_int(m, "Symmetry", 0);
    set_int(m, "Disconnected", 0);
    set_int(m, "Threads", 1);
}
