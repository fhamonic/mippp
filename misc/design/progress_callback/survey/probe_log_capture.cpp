// Can the log reach a hook with nothing on stdout? Run with stdout to a file.
#if defined(B_XPRESS)
#include "mippp/solvers/xpress/all.hpp"
using M = mippp::xpress_milp;
#elif defined(B_MOSEK)
#include "mippp/solvers/mosek/all.hpp"
using M = mippp::mosek_milp;
#elif defined(B_SCIP)
#include "mippp/solvers/scip/all.hpp"
using M = mippp::scip_milp;
#elif defined(B_COPT)
#include "mippp/solvers/copt/all.hpp"
using M = mippp::copt_milp;
#elif defined(B_GLPK)
#include "mippp/solvers/glpk/all.hpp"
using M = mippp::glpk_milp;
#endif
#include "probe_common.hpp"
static std::atomic<long> lines{0};
#if defined(B_XPRESS)
namespace v1 = mippp::xpress::impl::v1;
static void cb(v1::XPRSprob, void *, const char *, int, int) { ++lines; }
#elif defined(B_MOSEK)
namespace v1 = mippp::mosek::impl::v1;
static void cb(void *, const char *) { ++lines; }
#elif defined(B_SCIP)
static void cb(void *, FILE *, const char *) { ++lines; }
#elif defined(B_COPT)
namespace v1 = mippp::copt::impl::v1;
static void cb(char *, void *) { ++lines; }
#elif defined(B_GLPK)
static int cb(void *, const char *) { ++lines; return 1; }
#endif
int main() {
    M m; build_mkp(m, make_mkp(30, 100), +1, 100.0);
    m.set_time_limit(std::chrono::duration<double>(2));
    m.set_verbose(true);
    auto l = m.native_api().library_path();
#if defined(B_XPRESS)
    auto p = m.native_model();
    int e1 = symf<int (*)(v1::XPRSprob, void *, void *)>(l, "XPRSremovecbmessage")(p, nullptr, nullptr);
    int e2 = m.native_api().addcbmessage(p, cb, nullptr, 0);
    std::println(stderr, "remove={} add={}", e1, e2);
#elif defined(B_MOSEK)
    auto task = m.native_model().second;
    int e = m.native_api().linkfunctotaskstream(task, v1::MSK_STREAM_LOG, nullptr, cb);
    std::println(stderr, "link={}", e);
#elif defined(B_SCIP)
    using create_t = int (*)(void **, unsigned, const char *, unsigned, void (*)(void *, FILE *, const char *), void (*)(void *, FILE *, const char *), void (*)(void *, FILE *, const char *), int (*)(void *), void *);
    void * h = nullptr;
    int e1 = symf<create_t>(l, "SCIPmessagehdlrCreate")(&h, 0u, nullptr, 0u, cb, cb, cb, nullptr, nullptr);
    int e2 = symf<int (*)(void *, void *)>(l, "SCIPsetMessagehdlr")(m.native_model(), h);
    std::println(stderr, "create={} set={}", e1, e2);
#elif defined(B_COPT)
    auto [env, prob] = m.native_model();
    int e1 = m.native_api().SetIntParam(prob, "LogToConsole", 0);
    int e2 = symf<int (*)(v1::copt_prob *, void (*)(char *, void *), void *)>(l, "COPT_SetLogCallback")(prob, cb, nullptr);
    std::println(stderr, "LogToConsole={} set={}", e1, e2);
#elif defined(B_GLPK)
    symf<void (*)(int (*)(void *, const char *), void *)>(l, "glp_term_hook")(cb, nullptr);
#endif
    auto o = timed_solve(m);
    std::fflush(stdout);
    std::println(stderr, "hook lines={} status={} in {:.2f}s", lines.load(), status_name(m.get_status()), o.solve_s);
#if defined(B_GLPK)
    symf<void (*)(int (*)(void *, const char *), void *)>(l, "glp_term_hook")(nullptr, nullptr);
#endif
}
