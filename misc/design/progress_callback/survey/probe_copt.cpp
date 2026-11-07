#include "mippp/solvers/copt/all.hpp"
#include "probe_common.hpp"
using M = mippp::copt_milp;
using L = mippp::copt_lp;
namespace v1 = mippp::copt::impl::v1;
static const v1::copt_api * API;
using v1::copt_prob;
struct ctx { recorder * r; int errs = 0; int node_err = 0; };
static std::vector<std::unique_ptr<ctx>> keep;
static int cb(copt_prob * prob, void * cbdata, int cbctx, void * u) {
    auto * c = static_cast<ctx *>(u);
    double o = NAN, b = NAN; int has = -1, n = -1;
    int e1 = API->GetCallbackInfo(cbdata, "BestObj", &o);
    int e2 = API->GetCallbackInfo(cbdata, "BestBnd", &b);
    int e3 = API->GetCallbackInfo(cbdata, "HasIncumbent", &has);
    int e4 = API->GetIntAttr(prob, "NodeCnt", &n);
    if(e1 || e2 || e3) ++c->errs;
    if(e4) ++c->node_err;
    const char * name = cbctx == v1::COPT_CBCONTEXT_MIPNODE ? "MIPNODE" : cbctx == v1::COPT_CBCONTEXT_INCUMBENT ? "INCUMBENT"
                      : cbctx == v1::COPT_CBCONTEXT_MIPSOL ? "MIPSOL" : cbctx == v1::COPT_CBCONTEXT_MIPRELAX ? "MIPRELAX" : "other";
    if(has == 0) c->r->count("has_incumbent_0");
    if(c->r->record(name, o, b, n)) API->Interrupt(prob);
    c->r->note = std::format("read errors {} NodeCnt read errors {}", c->errs, c->node_err);
    return 0;
}
template <class X>
static void attach_mask(X & m, recorder & r, int mask) {
    API = &m.native_api();
    keep.push_back(std::make_unique<ctx>(ctx{&r}));
    auto [env, prob] = m.native_model();
    if(int e = m.native_api().SetCallback(prob, cb, mask, keep.back().get())) std::println(stderr, "SetCallback mask {} -> {}", mask, e);
}
static std::atomic<long> lines{0};
static void log_cb(char *, void *) { ++lines; }
using setlog_t = int (*)(copt_prob *, void (*)(char *, void *), void *);

int main() {
    using std::chrono::duration;
    const int mask = v1::COPT_CBCONTEXT_MIPNODE | v1::COPT_CBCONTEXT_INCUMBENT;
    backend<M> b{.name = "copt",
        .attach = [=](M & m, recorder & r) { attach_mask(m, r, mask); },
        .one_thread = [](M & m) { m.set_thread_limit(1); }};
    run_milp_matrix(b);
    if(std::getenv("PROBE_QUICK")) return 0;
    const auto large = make_mkp(30, 100);
    const auto small = make_mkp(10, 60);
    const auto single = [&](const char * label, const mkp & I, int threads, int msk) {
        M m; build_mkp(m, I, +1, 100.0); m.set_time_limit(duration<double>(20));
        if(threads) m.set_thread_limit(threads);
        recorder r; attach_mask(m, r, msk); r.start(); auto o = timed_solve(m);
        final_line(label, m, o.solve_s); hook_report(r, +1, o.solve_s);
    };
    single("X1 INCUMBENT only, small, 1 thread", small, 1, v1::COPT_CBCONTEXT_INCUMBENT);
    single("X2 MIPNODE only, small, 1 thread", small, 1, v1::COPT_CBCONTEXT_MIPNODE);
    single("X3 MIPSOL only, small, 1 thread (documented: dual reductions off)", small, 1, v1::COPT_CBCONTEXT_MIPSOL);
    single("X4 MIPRELAX only, small, 1 thread", small, 1, v1::COPT_CBCONTEXT_MIPRELAX);
    {   // X5 quiet log callback, and verbose
        for(bool verbose : {false, true}) {
            M m; build_mkp(m, large, +1, 100.0); m.set_time_limit(duration<double>(3));
            if(verbose) m.set_verbose(true);
            auto [env, prob] = m.native_model(); lines = 0;
            int e = symf<setlog_t>(m.native_api().library_path(), "COPT_SetLogCallback")(prob, log_cb, nullptr);
            auto o = timed_solve(m);
            std::println(stderr, "== X5 log callback verbose={}: set={} lines={} in {:.2f}s", verbose, e, lines.load(), o.solve_s);
        }
    }
    {   // L LP: the MIP contexts, then the log callback
        L m; build_lp(m, 12000, 18000); m.set_time_limit(duration<double>(40));
        recorder r; attach_mask(m, r, mask | v1::COPT_CBCONTEXT_MIPSOL | v1::COPT_CBCONTEXT_MIPRELAX);
        auto [env, prob] = m.native_model(); lines = 0;
        int e = symf<setlog_t>(m.native_api().library_path(), "COPT_SetLogCallback")(prob, log_cb, nullptr);
        r.start(); auto o = timed_solve(m);
        final_line("L LP on copt_lp, every context", m, o.solve_s); hook_report(r, -1, o.solve_s);
        std::println(stderr, "   log callback quiet: set={} lines={}", e, lines.load());
    }
    run_slices<M>(30, 100);
}
