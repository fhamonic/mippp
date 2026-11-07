#include "mippp/solvers/mosek/all.hpp"
#include "probe_common.hpp"

using M = mippp::mosek_lp;
namespace v1 = mippp::mosek::impl::v1;

static auto cfg(int optimizer, int basis = -1) {
    return [=](M & m) {
        auto task = m.native_model().second;
        m.native_api()._check(m.native_api().putintparam(task, v1::MSK_IPAR_OPTIMIZER, optimizer));
        if(basis != -1)
            m.native_api()._check(m.native_api().putintparam(task, static_cast<v1::MSKiparame>(17), basis));
    };
}

int main() {
    std::vector<probe_config<M>> configs = {
        {"default (FREE 2)", [](M &) {}},
        {"FREE_SIMPLEX 3", cfg(3)},
        {"PRIMAL_SIMPLEX 8", cfg(8)},
        {"DUAL_SIMPLEX 1", cfg(1)},
        {"NEW_PRIMAL_SIMPLEX 7", cfg(7)},
        {"NEW_DUAL_SIMPLEX 6", cfg(6)},
        {"INTPNT 4", cfg(4)},
        {"INTPNT 4, INTPNT_BASIS NEVER", cfg(4, 0)},
        {"FREE 2, INTPNT_BASIS NEVER", cfg(2, 0)},
    };
    run_all<M>(configs, [](M & m) {
        auto & api = m.native_api();
        auto task = m.native_model().second;
        int ip = -1, sp = -1, sd = -1;
        api.getintinf(task, static_cast<v1::MSKiinfiteme>(17), &ip);
        api.getintinf(task, static_cast<v1::MSKiinfiteme>(128), &sp);
        api.getintinf(task, static_cast<v1::MSKiinfiteme>(121), &sd);
        return std::format("[intpnt={} primal_simplex={} dual_simplex={}]", ip, sp, sd);
    });
}
