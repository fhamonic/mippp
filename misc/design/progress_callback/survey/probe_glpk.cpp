// built with -fno-access-control: glpk_milp::model_params holds mip.cb_func
#include "mippp/solvers/glpk/all.hpp"
#include "probe_common.hpp"
using M = mippp::glpk_milp;
using L = mippp::glpk_lp;
namespace v1 = mippp::glpk::impl::v1;
using T = v1::glp_tree; using Pb = v1::glp_prob;
static int (*reason)(T *);
static Pb * (*getprob)(T *);
static void (*tsize)(T *, int *, int *, int *);
static int (*bestnode)(T *);
static double (*nodebound)(T *, int);
static double (*mipobj)(Pb *);
static int (*mipstat)(Pb *);
static void (*terminate_)(T *);
static double (*mipgap)(T *);
template <class F> static void get(const std::filesystem::path & l, F & f, const char * n) { f = reinterpret_cast<F>(sym(l, n)); }
constexpr int R_ROWGEN = 1, R_BINGO = 2, R_HEUR = 3, R_CUTGEN = 4, R_BRANCH = 5, R_SELECT = 6, R_PREPRO = 7;
static void cb(T * t, void * u) {
    auto & r = *static_cast<recorder *>(u);
    int why = reason(t);
    if(why != R_SELECT && why != R_BINGO) { r.count("other_reason"); return; }
    Pb * p = getprob(t);
    int a, n, tot; tsize(t, &a, &n, &tot);
    int b = bestnode(t);
    double d = b ? nodebound(t, b) : NAN;
    double o = mipstat(p) == 1 /*GLP_UNDEF*/ ? NAN : mipobj(p);
    if(r.record(why == R_BINGO ? "IBINGO" : "ISELECT", o, d, tot)) terminate_(t);
}
static void attach(M & m, recorder & r) {
    auto l = m.native_api().library_path();
    get(l, reason, "glp_ios_reason"); get(l, getprob, "glp_ios_get_prob"); get(l, tsize, "glp_ios_tree_size");
    get(l, bestnode, "glp_ios_best_node"); get(l, nodebound, "glp_ios_node_bound"); get(l, mipobj, "glp_mip_obj_val");
    get(l, mipstat, "glp_mip_status"); get(l, terminate_, "glp_ios_terminate"); get(l, mipgap, "glp_ios_mip_gap");
    m.model_params.cb_func = cb; m.model_params.cb_info = &r;
}
static std::atomic<long> lines{0};
static int term_cb(void *, const char *) { ++lines; return 1; }
using hook_t = void (*)(int (*)(void *, const char *), void *);
int main(int argc, char ** argv) {
    using std::chrono::duration;
    backend<M> b{.name = "glpk", .attach = attach, .one_thread = [](M &) {}};
    if(argc > 1) { run_lp_slices<L>(800, 1200, 0.2); run_slices<M>(30, 100); return 0; }
    run_milp_matrix(b);
    if(std::getenv("PROBE_QUICK")) return 0;
    const auto large = make_mkp(30, 100);
    for(bool verbose : {false, true}) {
        M m; build_mkp(m, large, +1, 100.0); m.set_time_limit(duration<double>(3));
        if(verbose) m.set_verbose(true);
        lines = 0;
        auto hook = symf<hook_t>(m.native_api().library_path(), "glp_term_hook");
        hook(term_cb, nullptr);
        auto o = timed_solve(m);
        hook(nullptr, nullptr);
        std::println(stderr, "== X1 glp_term_hook verbose={}: lines={} in {:.2f}s", verbose, lines.load(), o.solve_s);
    }
    {
        L m; build_lp(m, 800, 1200); m.set_time_limit(duration<double>(30));
        m.set_verbose(true);
        lines = 0;
        auto hook = symf<hook_t>(m.native_api().library_path(), "glp_term_hook");
        hook(term_cb, nullptr);
        auto o = timed_solve(m);
        hook(nullptr, nullptr);
        final_line(std::format("L glpk_lp verbose, glp_term_hook lines={}", lines.load()), m, o.solve_s);
    }
    {
        L m; build_lp(m, 800, 1200); m.set_time_limit(duration<double>(30));
        lines = 0;
        auto hook = symf<hook_t>(m.native_api().library_path(), "glp_term_hook");
        hook(term_cb, nullptr);
        auto o = timed_solve(m);
        hook(nullptr, nullptr);
        final_line(std::format("L glpk_lp quiet, glp_term_hook lines={}", lines.load()), m, o.solve_s);
    }
    run_lp_slices<L>(800, 1200, 0.2);
    run_slices<M>(30, 100);
}
