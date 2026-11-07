// one lp_status instance per process: which one crashes COPT's PDLP
#include "mippp/solvers/copt/all.hpp"
#include "probe_common.hpp"

using M = mippp::copt_lp;
int main(int argc, char ** argv) {
    const int which = std::atoi(argv[1]), method = std::atoi(argv[2]);
    probe_config<M> cfg{"", [=](M & m) {
        m.native_api()._check(m.native_model().first,
            m.native_api().SetIntParam(m.native_model().second, "LpMethod", method));
    }};
    std::println("instance {} LpMethod {}: {}", which, method, tiny_status(cfg, which));
}
