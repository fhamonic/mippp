#include "mippp/solvers/highs/all.hpp"
#include "probe_common.hpp"
using M = mippp::highs_milp;
namespace v1 = mippp::highs::impl::v1;
using Out = v1::HighsCallbackDataOut; using In = v1::HighsCallbackDataIn;
static int calls; static int stop_on; static bool rearm;
static void cb(int, const char *, const Out *, In * in, void *) {
    ++calls;
    if(calls == stop_on) in->user_interrupt = 1;
    else if(rearm) in->user_interrupt = 0;
}
int main() {
    using std::chrono::duration;
    M probe; std::println(stderr, "library {}", probe.native_api().library_path().string());
    for(bool r : {false, true}) {
        rearm = r; calls = 0; stop_on = 3;
        M m; build_mkp(m, make_mkp(30, 100), +1, 100.0); m.set_time_limit(duration<double>(5));
        auto l = m.native_api().library_path();
        symf<v1::HighsInt (*)(void *, v1::HighsCCallbackType *, void *)>(l, "Highs_setCallback")(m.native_model(), cb, nullptr);
        symf<v1::HighsInt (*)(void *, int)>(l, "Highs_startCallback")(m.native_model(), 6);
        auto o = timed_solve(m);
        final_line(std::format("rearm={} first solve (stop at call 3)", r), m, o.solve_s);
        int after = calls;
        m.set_time_limit(duration<double>(2));
        auto o2 = timed_solve(m);
        final_line(std::format("rearm={} second solve, hook calls in it={}", r, calls - after), m, o2.solve_s);
    }
}
