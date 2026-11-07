#include "mippp/solvers/mosek/all.hpp"
#include "probe_common.hpp"
using M = mippp::mosek_milp;
using L = mippp::mosek_lp;
namespace v1 = mippp::mosek::impl::v1;
using v1::MSKtask_t;
static int I_BOUND = -1, I_INT = -1, I_NODES = -1, I_TIME = -1;
static int I_SIM_OBJ = -1, I_IPM_OBJ = -1, I_NUMSOL = -1, I_BOUND_DEF = -1;
static long nosol_samples = 0, nobound_samples = 0;
using infindex_t = int (*)(MSKtask_t, int, const char *, int *);
template <class X>
static void bind(X & m) {
    auto task = m.native_model().second;
    auto f = symf<infindex_t>(m.native_api().library_path(), "MSK_getinfindex");
    // MSK_INF_DOU_TYPE 0, MSK_INF_INT_TYPE 1
    f(task, 0, "MSK_DINF_MIO_OBJ_BOUND", &I_BOUND);
    f(task, 0, "MSK_DINF_MIO_OBJ_INT", &I_INT);
    f(task, 1, "MSK_IINF_MIO_NUM_SOLVED_NODES", &I_NODES);
    f(task, 0, "MSK_DINF_OPTIMIZER_TIME", &I_TIME);
    f(task, 0, "MSK_DINF_SIM_OBJ", &I_SIM_OBJ);
    f(task, 0, "MSK_DINF_INTPNT_PRIMAL_OBJ", &I_IPM_OBJ);
    f(task, 1, "MSK_IINF_MIO_NUM_INT_SOLUTIONS", &I_NUMSOL);
    f(task, 1, "MSK_IINF_MIO_OBJ_BOUND_DEFINED", &I_BOUND_DEF);
    nosol_samples = nobound_samples = 0;
}
static const char * code_name(int c) {
    switch(c) {
        case v1::MSK_CALLBACK_BEGIN_MIO: return "BEGIN_MIO";
        case v1::MSK_CALLBACK_END_MIO: return "END_MIO";
        case v1::MSK_CALLBACK_IM_MIO: return "IM_MIO";
        case v1::MSK_CALLBACK_NEW_INT_MIO: return "NEW_INT_MIO";
        case v1::MSK_CALLBACK_RESTART_MIO: return "RESTART_MIO";
        case v1::MSK_CALLBACK_DECOMP_MIO: return "DECOMP_MIO";
        case v1::MSK_CALLBACK_IM_MIO_DUAL_SIMPLEX: return "IM_MIO_DUAL_SIMPLEX";
        case v1::MSK_CALLBACK_IM_MIO_PRIMAL_SIMPLEX: return "IM_MIO_PRIMAL_SIMPLEX";
        case v1::MSK_CALLBACK_IM_MIO_INTPNT: return "IM_MIO_INTPNT";
        case v1::MSK_CALLBACK_INTPNT: return "INTPNT";
        case v1::MSK_CALLBACK_UPDATE_DUAL_SIMPLEX: return "UPDATE_DUAL_SIMPLEX";
        case v1::MSK_CALLBACK_UPDATE_PRIMAL_SIMPLEX: return "UPDATE_PRIMAL_SIMPLEX";
        case v1::MSK_CALLBACK_IM_SIMPLEX: return "IM_SIMPLEX";
        case v1::MSK_CALLBACK_UPDATE_SIMPLEX: return "UPDATE_SIMPLEX";
        default: return nullptr;
    }
}
static int cb(MSKtask_t, void * u, v1::MSKcallbackcodee caller, const v1::MSKrealt * d, const v1::MSKint32t * i, const v1::MSKint64t *) {
    auto & r = *static_cast<recorder *>(u);
    const char * n = code_name(caller);
    if(caller == v1::MSK_CALLBACK_IM_MIO || caller == v1::MSK_CALLBACK_NEW_INT_MIO || caller == v1::MSK_CALLBACK_END_MIO) {
        if(i[I_NUMSOL] == 0) { ++nosol_samples; r.note = std::format("samples with NUM_INT_SOLUTIONS=0: {}; OBJ_BOUND_DEFINED=0: {}", nosol_samples, nobound_samples); }
        if(i[I_BOUND_DEF] == 0) { ++nobound_samples; r.note = std::format("samples with NUM_INT_SOLUTIONS=0: {}; OBJ_BOUND_DEFINED=0: {}", nosol_samples, nobound_samples); }
        if(r.record(n, d[I_INT], d[I_BOUND], i[I_NODES])) return 1;
    } else if(n) r.count(n);
    else r.count("other");
    return 0;
}
static int lp_cb(MSKtask_t, void * u, v1::MSKcallbackcodee caller, const v1::MSKrealt * d, const v1::MSKint32t *, const v1::MSKint64t *) {
    auto & r = *static_cast<recorder *>(u);
    const char * n = code_name(caller);
    if(caller == v1::MSK_CALLBACK_INTPNT) r.record(n, d[I_IPM_OBJ], NAN, 0);
    else if(caller == v1::MSK_CALLBACK_UPDATE_SIMPLEX || caller == v1::MSK_CALLBACK_UPDATE_DUAL_SIMPLEX || caller == v1::MSK_CALLBACK_UPDATE_PRIMAL_SIMPLEX) r.record(n, d[I_SIM_OBJ], NAN, 0);
    else if(n) r.count(n); else r.count("other");
    return 0;
}
// declared in mosek_api.hpp, not loaded by it
using put_t = v1::MSKrescodee (*)(MSKtask_t, v1::MSKcallbackfunc *, void *);
template <class X> static put_t put(X & m) { return symf<put_t>(m.native_api().library_path(), "MSK_putcallbackfunc"); }
static std::atomic<long> lines{0};
static void stream_cb(void *, const char *) { ++lines; }

int main() {
    using std::chrono::duration;
    backend<M> b{.name = "mosek",
        .attach = [](M & m, recorder & r) {
            bind(m); auto task = m.native_model().second;
            m.native_api()._check(put(m)(task, cb, &r)); },
        .one_thread = [](M & m) { m.set_thread_limit(1); }};
    run_milp_matrix(b);
    if(std::getenv("PROBE_QUICK")) return 0;
    const auto large = make_mkp(30, 100);
    for(bool verbose : {false, true}) {
        M m; build_mkp(m, large, +1, 100.0); m.set_time_limit(duration<double>(3));
        if(verbose) m.set_verbose(true);
        auto task = m.native_model().second; lines = 0;
        int e = m.native_api().linkfunctotaskstream(task, v1::MSK_STREAM_LOG, nullptr, stream_cb);
        auto o = timed_solve(m);
        std::println(stderr, "== X1 log stream verbose={}: link={} lines={} in {:.2f}s", verbose, e, lines.load(), o.solve_s);
    }
    {
        L m; build_lp(m, 12000, 18000); m.set_time_limit(duration<double>(40));
        bind(m); recorder r; auto task = m.native_model().second;
        m.native_api()._check(put(m)(task, lp_cb, &r));
        r.start(); auto o = timed_solve(m);
        final_line("L LP on mosek_lp", m, o.solve_s); hook_report(r, -1, o.solve_s);
    }
    run_slices<M>(30, 100);
}
