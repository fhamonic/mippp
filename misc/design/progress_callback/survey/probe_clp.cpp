#include "mippp/solvers/clp/all.hpp"
#include "probe_common.hpp"
using L = mippp::clp_lp;
namespace v1 = mippp::clp::impl::v1;
using S = v1::Clp_Simplex;
static recorder * R; static int dumped = 0;
using cb_t = void (*)(S *, int, int, const double *, int, const int *, int, char **);
static void msg_cb(S *, int msgno, int nd, const double * dv, int ni, const int * iv, int, char **) {
    if(dumped < 10) {
        std::string d, i;
        for(int k = 0; k < nd; ++k) d += std::format("{:.6g} ", dv[k]);
        for(int k = 0; k < ni; ++k) i += std::format("{} ", iv[k]);
        std::println(stderr, "   msg {}: dvec=[{}] ivec=[{}]", msgno, d, i);
        ++dumped;
    }
    if(nd >= 1 && ni >= 1) R->record(std::format("msg{}", msgno), dv[0], NAN, iv[0]);
    else R->count(std::format("msg{}", msgno));
}
int main() {
    using std::chrono::duration;
    { L m; std::println(stderr, "library {}", m.native_api().library_path().string()); }
    for(bool verbose : {false, true}) {
        L m; build_lp(m, 3000, 4500); m.set_time_limit(duration<double>(30));
        if(verbose) m.set_verbose(true);
        recorder r; R = &r; dumped = 0;
        symf<void (*)(S *, cb_t)>(m.native_api().library_path(), "Clp_registerCallBack")(m.native_model(), msg_cb);
        r.start(); auto o = timed_solve(m);
        final_line(std::format("L clp_lp message callback verbose={}", verbose), m, o.solve_s);
        hook_report(r, -1, o.solve_s);
    }
    {   // long enough for the status lines to come
        L m; build_lp(m, 12000, 18000); m.set_time_limit(duration<double>(30)); m.set_verbose(true);
        recorder r; R = &r; dumped = 0;
        symf<void (*)(S *, cb_t)>(m.native_api().library_path(), "Clp_registerCallBack")(m.native_model(), msg_cb);
        r.start(); auto o = timed_solve(m);
        final_line("L2 clp_lp big LP, message callback verbose", m, o.solve_s);
        hook_report(r, -1, o.solve_s);
    }
    run_lp_slices<L>(3000, 4500, 0.5);
}
