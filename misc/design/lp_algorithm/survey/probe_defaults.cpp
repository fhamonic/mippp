// HiGHS solver=simplex with simplex_strategy 0 (choose), and the native
// defaults the decoders would read on fresh Xpress and HiGHS models
#include "mippp/solvers/highs/all.hpp"
#include "mippp/solvers/xpress/all.hpp"
#include "probe_common.hpp"

int main(int, char ** argv) {
    if(std::string(argv[1]) == "xpress") {
        mippp::xpress_lp m;
        int f = -9, c = -9;
        m.native_api().getintcontrol(m.native_model(), 8385, &f);
        m.native_api().getintcontrol(m.native_model(), 8044, &c);
        std::println("xpress fresh: LPFLAGS={} CROSSOVER={}", f, c);
        return 0;
    }
    using M = mippp::highs_lp;
    {
        M m;
        int s = -9;
        m.native_api().getIntOptionValue(m.native_model(), "simplex_strategy", &s);
        std::println("highs fresh: simplex_strategy={}", s);
    }
    for(int strategy : {0, 1, 4}) {
        probe_config<M> cfg{"", [=](M & m) {
            m.native_api().setStringOptionValue(m.native_model(), "solver", "simplex");
            m.native_api().setIntOptionValue(m.native_model(), "simplex_strategy", strategy);
        }};
        for(int which : {0, 1, 2}) {
            M m;
            if(which == 0) build_feasible(m);
            else if(which == 1) build_infeasible(m, max_of_extra_row<M>());
            else build_unbounded(m);
            cfg.apply(m);
            m.solve();
            int it = -1;
            m.native_api().getIntInfoValue(m.native_model(), "simplex_iteration_count", &it);
            std::println("highs simplex strategy {} LP {}: status={} simplex={}", strategy, which,
                         status_name(m.get_status()), it);
        }
        std::print("  lp_status suite:");
        for(int w = 0; w < 5; ++w) std::print(" {}", tiny_status(cfg, w));
        std::println("");
    }
}
