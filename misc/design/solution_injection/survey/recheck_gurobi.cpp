#include "mippp/solvers/gurobi/all.hpp"

#include "benders_common.hpp"
#include "recheck_common.hpp"

using M = mippp::gurobi_milp;

int main(int argc, char ** argv) {
    {
        M probe;
        int a, b, c;
        bp::symf<void (*)(int *, int *, int *)>(
            probe.native_api().library_path(), "GRBversion")(&a, &b, &c);
        std::println(stderr, "Gurobi {}.{}.{} {}", a, b, c,
                     probe.native_api().library_path().string());
    }
    auto inject = [](auto & handle, std::vector<double> x, bool, double) {
        for(auto & v : x)
            if(std::isnan(v)) v = 1e101;  // GRB_UNDEFINED
        double obj = 0;
        const int rc =
            handle.parent.GRB->cbsolution(handle.cbdata, x.data(), &obj);
        bp::last_obj = obj;
        return rc;
    };
    auto setup = [](M & m, int threads) {
        if(threads > 0) m.set_thread_limit(static_cast<std::size_t>(threads));
    };
    if(argc > 1 && std::string(argv[1]) == "recheck") {
        satisfied<M>("gurobi");
        recheck<M>("gurobi", inject, -1);
        recheck<M>("gurobi", inject, 8);
        recheck<M>("gurobi", inject, 5);
        return 0;
    }
    return bp::main_loop<M>("gurobi", argc, argv, inject, setup);
}
