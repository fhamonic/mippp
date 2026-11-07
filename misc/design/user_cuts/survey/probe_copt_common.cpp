// Shared by probe_copt.cpp (the matrix) and probe_copt_x.cpp (the extras).
// copt_milp::solve() always takes COPT's one callback slot, so a hook set on the
// model would be overwritten. M hides solve() with a copy of the wrapper's body
// (hence -fno-access-control) that registers probe_hook instead. Without a hook it
// forwards to the wrapper, so baselines are what MIP++ does today (MIPNODE on).
#include "mippp/solvers/copt/all.hpp"
#include "cut_common.hpp"
namespace c = mippp::copt::impl::v1;

using addcuts_fn = int (*)(void *, int, const int *, const int *, const int *, const double *, const char *, const double *);
using addcut_row_fn = int (*)(c::copt_prob *, int, const int *, const double *, char, double, double, const char *);

struct hook_ctx;
struct M : c::copt_milp {
    std::unique_ptr<hook_ctx> hook;
    bool no_callback = false;  // the native solve, with the slot empty
    void solve();
    c::copt_prob * P() { return prob; }
};

struct hook_ctx {
    cp::separator sep;
    M * model = nullptr;
    const c::copt_api * api = nullptr;
    int cols = 0;
    std::string mode;
    int mask = c::COPT_CBCONTEXT_MIPRELAX | c::COPT_CBCONTEXT_MIPNODE;
    addcuts_fn addcuts = nullptr;
    int native_rc = 0;  // the return code of COPT_Solve
    std::function<int(c::copt_prob *, void *, int)> raw;  // takes over the whole callback
    std::atomic<long> node_calls{0}, relax_calls{0}, sol_calls{0}, inc_calls{0}, read_errors{0}, exceptions{0};
    std::atomic<int> last_read_error{0};
};

static int probe_hook(c::copt_prob * prob, void * cbdata, int cbctx, void * usr) {
    auto & h = *static_cast<hook_ctx *>(usr);
    if(h.raw) { try { return h.raw(prob, cbdata, cbctx); } catch(...) { ++h.exceptions; return 0; } }
    if(cbctx == c::COPT_CBCONTEXT_MIPNODE) {
        ++h.node_calls;
        double b;
        if(h.api->GetCallbackInfo(cbdata, c::COPT_CBINFO_BESTBND, &b) == 0) h.model->_search_bound = b;
        return 0;
    }
    if(cbctx == c::COPT_CBCONTEXT_MIPSOL) { ++h.sol_calls; return 0; }
    if(cbctx != c::COPT_CBCONTEXT_MIPRELAX) { ++h.inc_calls; return 0; }
    ++h.relax_calls;
    std::vector<double> x(static_cast<std::size_t>(h.cols));
    const int e = h.api->GetCallbackInfo(cbdata, c::COPT_CBINFO_RELAXSOLUTION, x.data());
    if(e) { ++h.read_errors; h.last_read_error = e; return 0; }
    try {
        h.sep(x, [&](const cp::raw_cut & r) -> int {
            const int n = static_cast<int>(r.ind.size());
            if(h.mode == "lazy_relax")
                return h.api->AddCallbackLazyConstr(cbdata, n, r.ind.data(), r.val.data(), r.sense, r.rhs);
            if(h.mode == "usercuts_batch") {
                const int beg = 0;
                return h.addcuts(cbdata, 1, &beg, &n, r.ind.data(), r.val.data(), &r.sense, &r.rhs);
            }
            return h.api->AddCallbackUserCut(cbdata, n, r.ind.data(), r.val.data(), r.sense, r.rhs);
        });
    } catch(...) { ++h.exceptions; }
    return 0;
}

void M::solve() {
    if(no_callback) {
        reset_status();
        check(COPT->GetIntAttr(prob, c::COPT_INTATTR_ISMIP, &_is_mip));
        check(COPT->Solve(prob));
        _status = _get_status_milp();
        _mark_results_valid();
        return;
    }
    if(!hook) { c::copt_milp::solve(); return; }
    reset_status();
    check(COPT->GetIntAttr(prob, c::COPT_INTATTR_ISMIP, &_is_mip));
    int sense;
    check(COPT->GetIntAttr(prob, c::COPT_INTATTR_OBJSENSE, &sense));
    _search_bound = sense == c::COPT_MINIMIZE ? -c::COPT_INFINITY : c::COPT_INFINITY;
    check(COPT->SetCallback(prob, probe_hook, hook->mask, hook.get()));
    hook->native_rc = COPT->Solve(prob);
    COPT->SetCallback(prob, nullptr, 0, nullptr);
    check(static_cast<c::ret_code>(hook->native_rc));
    _status = _get_status_milp();
    _mark_results_valid();
}

static int mask_of_mode(const std::string & mode) {
    constexpr int R = c::COPT_CBCONTEXT_MIPRELAX, N = c::COPT_CBCONTEXT_MIPNODE, S = c::COPT_CBCONTEXT_MIPSOL;
    if(mode == "usercut_nonode") return R;
    if(mode == "lazy_relax") return R | N | S;
    return R | N;
}
static void attach(M & m, cp::separator sep, const std::string & mode) {
    auto h = std::make_unique<hook_ctx>();
    h->sep = std::move(sep);
    h->model = &m;
    h->api = &m.native_api();
    h->cols = static_cast<int>(m.num_variables());
    h->mode = mode;
    h->mask = mask_of_mode(mode);
    h->addcuts = cp::symf<addcuts_fn>(m.native_api().library_path(), "COPT_AddCallbackUserCuts");
    m.hook = std::move(h);
}
static void log_if_asked(M & m) {
    if(std::getenv("CUT_LOG")) m.native_api()._check(nullptr, m.native_api().SetIntParam(m.native_model().second, "Logging", 1));
}
static void one_thread(M & m) { m.set_thread_limit(1); log_if_asked(m); }
// CutLevel=0 silences CBCONTEXT_MIPRELAX (out_copt_params.txt), so the harness's
// "no solver cuts" cannot be honoured literally: a tree-cut level must stay on.
// CUT_SCOPE picks the variant, default noroot (no root cuts, tree cuts on auto).
static void scope_settings(M & m) {
    auto & api = m.native_api();
    auto [env, prob] = m.native_model();
    for(const char * name : {"Presolve", "HeurLevel", "PreRootHeurLevel", "RoundingHeurLevel",
                             "DivingHeurLevel", "SubMipHeurLevel", "FAPHeurLevel", "ConflictAnalysis"})
        api._check(env, api.SetIntParam(prob, name, 0));
    api._check(env, api.SetIntParam(prob, "Threads", 1));
    api._check(env, api.SetIntParam(prob, "MipTasks", 1));
    const char * e = std::getenv("CUT_SCOPE");
    const std::string v = e ? e : "noroot";
    std::vector<std::pair<const char *, int>> extra;
    if(v == "tree1") extra = {{"CutLevel", 0}, {"TreeCutLevel", 1}};
    else if(v == "tree1r1") extra = {{"CutLevel", 0}, {"TreeCutLevel", 1}, {"NodeCutRounds", 1}};
    else if(v == "tree3") extra = {{"CutLevel", 0}, {"TreeCutLevel", 3}};
    else if(v == "noroot") extra = {{"RootCutLevel", 0}};
    else if(v == "noroot_r1") extra = {{"RootCutLevel", 0}, {"NodeCutRounds", 1}};
    else if(v == "root_r1") extra = {{"RootCutRounds", 1}};
    else if(v == "root_r1_tree1") extra = {{"RootCutRounds", 1}, {"TreeCutLevel", 1}};
    else if(v == "root1_tree1") extra = {{"CutLevel", 0}, {"RootCutLevel", 1}, {"TreeCutLevel", 1}, {"RootCutRounds", 1}};
    else if(v == "silent") extra = {{"CutLevel", 0}};
    for(auto [name, val] : extra) api._check(env, api.SetIntParam(prob, name, val));
    log_if_asked(m);
}
