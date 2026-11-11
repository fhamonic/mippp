#include "mippp/solvers/cplex/all.hpp"

#include "benders_common.hpp"
#include "recheck_common.hpp"

using M = mippp::cplex_milp;
using post_fn = int (*)(void *, int, const int *, const double *, double, int);

int main(int argc, char ** argv) {
    post_fn post = nullptr;
    {
        M probe;
        auto [env, lp] = probe.native_model();
        std::println(stderr, "CPLEX {} {}", probe.native_api().version(env),
                     probe.native_api().library_path().string());
        post = bp::symf<post_fn>(probe.native_api().library_path(),
                                 "CPXcallbackpostheursoln");
    }
    // STRAT: -1 nocheck, 0 checkfeas, 1 propagate, 2 solve
    const char * e = std::getenv("STRAT");
    const int full_strat = e ? std::atoi(e) : 0;
    const char * pe = std::getenv("PSTRAT");
    const int partial_strat = pe ? std::atoi(pe) : 2;
    auto inject = [&](auto & handle, const std::vector<double> & x,
                      bool partial, double obj) {
        std::vector<int> ind;
        std::vector<double> val;
        for(int k = 0; k < static_cast<int>(x.size()); ++k)
            if(!std::isnan(x[k])) {
                ind.push_back(k);
                val.push_back(x[k]);
            }
        return post(handle.context, static_cast<int>(ind.size()), ind.data(),
                    val.data(), obj, partial ? partial_strat : full_strat);
    };
    auto setup = [](M & m, int threads) {
        if(threads > 0) m.set_thread_limit(static_cast<std::size_t>(threads));
    };
    if(argc > 1 && std::string(argv[1]) == "recheck") {
        satisfied<M>("cplex");
        recheck<M>("cplex", inject, -1);
        recheck<M>("cplex", inject, 8);
        recheck<M>("cplex", inject, 5);
        return 0;
    }
    return bp::main_loop<M>("cplex", argc, argv, inject, setup);
}
