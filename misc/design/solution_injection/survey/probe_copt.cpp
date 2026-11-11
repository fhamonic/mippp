#include "mippp/solvers/copt/all.hpp"

#include "benders_common.hpp"

using M = mippp::copt_milp;

int main(int argc, char ** argv) {
    {
        M probe;
        std::println(stderr, "COPT {}",
                     probe.native_api().library_path().string());
    }
    auto inject = [](auto & handle, std::vector<double> x, bool, double) {
        for(auto & v : x)
            if(std::isnan(v)) v = 1e40;  // COPT_UNDEFINED
        double obj = 0;
        const int rc = static_cast<int>(
            handle.COPT->AddCallbackSolution(handle.cbdata, x.data(), &obj));
        bp::last_obj = obj;
        return rc;
    };
    auto setup = [](M & m, int threads) {
        if(threads > 0) m.set_thread_limit(static_cast<std::size_t>(threads));
    };
    return bp::main_loop<M>("copt", argc, argv, inject, setup);
}
