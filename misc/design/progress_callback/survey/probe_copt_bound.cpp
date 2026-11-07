// What does COPT hold as a bound after a stop, with and without an incumbent?
#include "mippp/solvers/copt/all.hpp"
#include "probe_common.hpp"
using M = mippp::copt_milp;
static double attr(M & m, const char * n) { double v = NAN; int e = m.native_api().GetDblAttr(m.native_model().second, n, &v); return e ? -999999 : v; }
static int iattr(M & m, const char * n) { int v = -1; m.native_api().GetIntAttr(m.native_model().second, n, &v); return v; }
static void show(M & m, const char * label, double hook, double hook_obj) {
    std::println(stderr, "{}: status={} MipStatus={} HasMipSol={} NodeCnt={} | BestBnd={:g} ObjBound={:g} BestGap={:g} BestObj={:g} ObjVal={:g} LpObjval={:g} | hook last BestBnd={:g} BestObj={:g}", label, status_name(m.get_status()), iattr(m,"MipStatus"), iattr(m,"HasMipSol"), iattr(m,"NodeCnt"), attr(m,"BestBnd"), attr(m,"ObjBound"), attr(m,"BestGap"), attr(m,"BestObj"), attr(m,"ObjVal"), attr(m,"LpObjval"), hook, hook_obj);
}
template <class B> static void run(const char * label, B build, double limit, int stop_after = 0) {
    M m; build(m); m.set_time_limit(std::chrono::duration<double>(limit));
    double hook = NAN, hook_obj = NAN; int calls = 0;
    m.set_progress_callback([&](auto & h) { hook = h.get_dual_bound(); hook_obj = h.get_solution_value(); if(stop_after && ++calls == stop_after) h.interrupt(); });
    m.solve(); show(m, label, hook, hook_obj);
}
int main() {
    run("no incumbent min, limit 2s", [](M & m) { build_no_incumbent(m, 7); }, 2);
    run("no incumbent max, limit 2s", [](M & m) { build_no_incumbent(m, 7); m.set_maximization(); }, 2);
    run("no incumbent min, interrupted", [](M & m) { build_no_incumbent(m, 7); }, 30, 200);
    run("incumbent min, limit 2s", [](M & m) { build_mkp(m, make_mkp(30, 100), +1, 100.0); }, 2);
    run("incumbent max, limit 2s", [](M & m) { build_mkp(m, make_mkp(30, 100), -1, 100.0); }, 2);
    run("solved min", [](M & m) { build_mkp(m, make_mkp(5, 25), +1, 100.0); }, 30);
    run("no incumbent min, limit 10ms", [](M & m) { build_no_incumbent(m, 7); }, 0.01);
    run("infeasible", [](M & m) { auto x = m.add_binary_variables(2); m.set_objective(1.0 * x(0)); m.add_constraint(x(0) + x(1) >= 3); }, 5);
}
