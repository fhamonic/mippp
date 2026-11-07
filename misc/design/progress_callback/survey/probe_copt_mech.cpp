// What moves COPT's search when a MIPSOL callback is installed?
#include "mippp/solvers/copt/all.hpp"
#include "probe_common.hpp"
using M = mippp::copt_milp;
namespace v1 = mippp::copt::impl::v1;
static int noop(v1::copt_prob *, void *, int, void *) { return 0; }
template <class F> static void run(const char * label, F setup) {
    M m; build_mkp(m, make_mkp(10, 60), +1, 100.0); m.set_thread_limit(1);
    m.set_time_limit(std::chrono::duration<double>(40));
    setup(m);
    auto o = timed_solve(m);
    final_line(label, m, o.solve_s);
}
int main() {
    auto lazy = [](M & m, int v) { auto [e, p] = m.native_model(); m.native_api().SetIntParam(p, "LazyConstraints", v); };
    auto cb = [](M & m, int mask) { auto [e, p] = m.native_model(); m.native_api().SetCallback(p, noop, mask, nullptr); };
    run("1 baseline", [](M &) {});
    run("2 baseline again", [](M &) {});
    run("3 MIPSOL callback", [&](M & m) { cb(m, v1::COPT_CBCONTEXT_MIPSOL); });
    run("4 MIPSOL callback, LazyConstraints=0", [&](M & m) { cb(m, v1::COPT_CBCONTEXT_MIPSOL); lazy(m, 0); });
    run("5 no callback, LazyConstraints=1", [&](M & m) { lazy(m, 1); });
    run("6 INCUMBENT|MIPNODE callback", [&](M & m) { cb(m, v1::COPT_CBCONTEXT_INCUMBENT | v1::COPT_CBCONTEXT_MIPNODE); });
}
