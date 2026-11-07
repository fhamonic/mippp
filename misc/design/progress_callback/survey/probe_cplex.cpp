#include "mippp/solvers/cplex/all.hpp"
#include "probe_common.hpp"
using M = mippp::cplex_milp;
using L = mippp::cplex_lp;
namespace v1 = mippp::cplex::impl::v1;
using ctx_t = v1::CPXCALLBACKCONTEXTptr;
using dbl_fn = int (*)(ctx_t, int, double *);
using lng_fn = int (*)(ctx_t, int, long long *);
enum { I_THREADID, I_NODECOUNT, I_ITCOUNT, I_BEST_SOL, I_BEST_BND, I_THREADS, I_FEASIBLE, I_TIME };
static dbl_fn getdbl;
static lng_fn getlng;
static const v1::cplex_api * API;

static int cb(ctx_t c, v1::CPXLONG id, void * usr) {
    auto & r = *static_cast<recorder *>(usr);
    const char * name = id == v1::CPX_CALLBACKCONTEXT_GLOBAL_PROGRESS ? "GLOBAL"
                      : id == v1::CPX_CALLBACKCONTEXT_LOCAL_PROGRESS  ? "LOCAL"
                      : id == v1::CPX_CALLBACKCONTEXT_CANDIDATE       ? "CANDIDATE" : "other";
    double p = NAN, d = NAN; long long n = -1;
    int e1 = getdbl(c, I_BEST_SOL, &p), e2 = getdbl(c, I_BEST_BND, &d), e3 = getlng(c, I_NODECOUNT, &n);
    if(e1 || e2 || e3) ++r.quiet_read_errors;
    if(id == v1::CPX_CALLBACKCONTEXT_GLOBAL_PROGRESS) {
        if(r.record(name, p, d, n)) API->callbackabort(c);
    } else r.count(name);
    return 0;
}
template <class X>
static void bind(X & m) {
    API = &m.native_api();
    getdbl = symf<dbl_fn>(m.native_api().library_path(), "CPXcallbackgetinfodbl");
    getlng = symf<lng_fn>(m.native_api().library_path(), "CPXcallbackgetinfolong");
}
static void attach_mask(M & m, recorder & r, v1::CPXLONG mask) {
    bind(m);
    auto [env, lp] = m.native_model();
    m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, mask, cb, &r));
}
static std::atomic<long> log_lines{0};
static void log_fn(void *, const char *) { ++log_lines; }
using lpcb_t = int (*)(v1::CPXCENVptr, void *, int, void *);
using setlp_t = int (*)(v1::CPXENVptr, lpcb_t, void *);
using getinfo_t = int (*)(v1::CPXCENVptr, void *, int, int, void *);
static getinfo_t getinfo;
static int lp_cb(v1::CPXCENVptr env, void * cbdata, int wherefrom, void * usr) {
    auto & r = *static_cast<recorder *>(usr);
    int it = -1; double obj = NAN;
    getinfo(env, cbdata, wherefrom, 7 /*ITCOUNT*/, &it);
    getinfo(env, cbdata, wherefrom, 1 /*PRIMAL_OBJ*/, &obj);
    r.record(std::format("LPCB_where{}", wherefrom), obj, NAN, it);
    return 0;
}

int main() {
    using std::chrono::duration;
    backend<M> b{.name = "cplex",
                 .attach = [](M & m, recorder & r) { attach_mask(m, r, v1::CPX_CALLBACKCONTEXT_GLOBAL_PROGRESS); },
                 .one_thread = [](M & m) { m.set_thread_limit(1); }};
    run_milp_matrix(b);
    if(std::getenv("PROBE_QUICK")) return 0;
    const auto large = make_mkp(30, 100);
    {   // X1 local progress, for its rate
        M m; build_mkp(m, large, +1, 100.0); m.set_time_limit(duration<double>(5));
        recorder r; attach_mask(m, r, v1::CPX_CALLBACKCONTEXT_GLOBAL_PROGRESS | v1::CPX_CALLBACKCONTEXT_LOCAL_PROGRESS);
        r.start(); auto o = timed_solve(m);
        final_line("X1 GLOBAL|LOCAL 5s", m, o.solve_s); hook_report(r, +1, o.solve_s);
        std::println(stderr, "   read errors {}", r.quiet_read_errors);
    }
    {   // X2 one function, two contexts: candidate and progress side by side
        M m; build_mkp(m, large, +1, 100.0); m.set_time_limit(duration<double>(5));
        recorder r; attach_mask(m, r, v1::CPX_CALLBACKCONTEXT_GLOBAL_PROGRESS | v1::CPX_CALLBACKCONTEXT_CANDIDATE);
        r.start(); auto o = timed_solve(m);
        final_line("X2 GLOBAL|CANDIDATE 5s", m, o.solve_s); hook_report(r, +1, o.solve_s);
    }
    {   // X3 the refiner with a native progress hook: the wrapper detaches only its own candidate callback
        M m; auto x = m.add_variables(1, {.lower_bound = 0, .upper_bound = 1});
        m.add_constraint(x(0) >= 2); 
        m.set_minimization(); m.set_objective(1.0 * x(0));
        recorder r; attach_mask(m, r, v1::CPX_CALLBACKCONTEXT_GLOBAL_PROGRESS);
        try { auto iis = m.compute_iis(); std::println(stderr, "== X3 compute_iis with a progress hook attached: returned"); }
        catch(const std::exception & e) { std::println(stderr, "== X3 compute_iis with a progress hook attached: threw {}", e.what()); }
    }
    {   // X4 the log channel under the quiet default and when verbose
        for(bool verbose : {false, true}) {
            M m; build_mkp(m, large, +1, 100.0); m.set_time_limit(duration<double>(3));
            if(verbose) m.set_verbose(true);
            bind(m);
            auto [env, lp] = m.native_model();
            using chan_t = int (*)(v1::CPXCENVptr, void **, void **, void **, void **);
            using dest_t = int (*)(v1::CPXCENVptr, void *, void *, void (*)(void *, const char *));
            auto getch = symf<chan_t>(m.native_api().library_path(), "CPXgetchannels");
            auto adddest = symf<dest_t>(m.native_api().library_path(), "CPXaddfuncdest");
            void * res, * warn, * err, * log;
            getch(env, &res, &warn, &err, &log);
            log_lines = 0;
            int e = adddest(env, res, nullptr, log_fn);
            auto o = timed_solve(m);
            std::println(stderr, "== X4 log channel verbose={}: addfuncdest={} lines={} in {:.2f}s", verbose, e, log_lines.load(), o.solve_s);
            using deldest_t = int (*)(v1::CPXCENVptr, void *, void *, void (*)(void *, const char *));
            symf<deldest_t>(m.native_api().library_path(), "CPXdelfuncdest")(env, res, nullptr, log_fn);
        }
    }
    {   // X5 legacy LP callback next to a generic callback on a MIP
        M m; build_mkp(m, large, +1, 100.0); m.set_time_limit(duration<double>(3));
        recorder r; attach_mask(m, r, v1::CPX_CALLBACKCONTEXT_GLOBAL_PROGRESS);
        auto [env, lp] = m.native_model();
        getinfo = symf<getinfo_t>(m.native_api().library_path(), "CPXgetcallbackinfo");
        auto setlp = symf<setlp_t>(m.native_api().library_path(), "CPXsetlpcallbackfunc");
        int e = setlp(env, lp_cb, &r);
        r.start();
        try {
            auto o = timed_solve(m);
            final_line(std::format("X5 legacy LP cb (set={}) + GLOBAL, MIP 3s", e), m, o.solve_s);
            hook_report(r, +1, o.solve_s);
        } catch(const std::exception & ex) { std::println(stderr, "== X5 legacy + generic: threw {}", ex.what()); }
    }
    {   // L LP on cplex_lp through the legacy callback
        L m; build_lp(m, 900, 1000); // Community Edition: at most 1000 rows and columns
        m.set_time_limit(duration<double>(40));
        recorder r; auto [env, lp] = m.native_model();
        getinfo = symf<getinfo_t>(m.native_api().library_path(), "CPXgetcallbackinfo");
        auto setlp = symf<setlp_t>(m.native_api().library_path(), "CPXsetlpcallbackfunc");
        int e = setlp(env, lp_cb, &r);
        r.start(); auto o = timed_solve(m);
        final_line(std::format("L LP on cplex_lp, legacy cb (set={})", e), m, o.solve_s);
        hook_report(r, -1, o.solve_s);
    }
    run_slices<M>(30, 100);
}
