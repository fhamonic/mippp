// What does a do-nothing MIPNODE|INCUMBENT callback cost COPT? Same search, same instance, repeated.
#include "mippp/solvers/copt/all.hpp"
#include "probe_common.hpp"
using M = mippp::copt_milp;
namespace v1 = mippp::copt::impl::v1;
static const v1::copt_api * API; static double last;
static int cb(v1::copt_prob *, void * d, int, void *) { API->GetCallbackInfo(d, "BestBnd", &last); return 0; }
int main() {
    const mkp big = make_mkp(30, 100), small = make_mkp(10, 60);
    for(int threads : {1, 0}) for(int rep = 0; rep < 3; ++rep) for(bool with : {false, true}) {
        M m; if(threads) m.set_thread_limit(threads); build_mkp(m, threads ? small : big, +1, 100.0);
        m.set_time_limit(std::chrono::duration<double>(threads ? 60 : 8));
        API = &m.native_api();
        if(with) m.native_api().SetCallback(m.native_model().second, cb, v1::COPT_CBCONTEXT_MIPNODE | v1::COPT_CBCONTEXT_INCUMBENT, nullptr);
        auto o = timed_solve(m);
        std::println(stderr, "threads={} rep={} callback={}: {:.2f}s nodes={} dual={:.6g}", threads, rep, with ? "yes" : "no ", o.solve_s, m.get_node_count(), o.dual);
    }
}
