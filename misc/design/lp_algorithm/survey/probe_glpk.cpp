#include "mippp/solvers/glpk/all.hpp"
#include "probe_common.hpp"

using M = mippp::glpk_lp;

static auto meth(int v) { return [=](M & m) { m.model_params.meth = v; }; }

int main() {
    std::vector<probe_config<M>> configs = {
        {"today: GLP_PRIMAL", meth(1)},
        {"GLP_DUALP", meth(2)},
        {"GLP_DUAL", meth(3)},
    };
    run_all<M>(configs, [](M & m) {
        static auto ds = reinterpret_cast<int (*)(void *)>(sym(m.native_api().library_path(), "glp_get_dual_stat"));
        static auto ps = reinterpret_cast<int (*)(void *)>(sym(m.native_api().library_path(), "glp_get_prim_stat"));
        static auto gs = reinterpret_cast<int (*)(void *)>(sym(m.native_api().library_path(), "glp_get_status"));
        void * p = m.native_model();
        return std::format("[iterations={} status={} prim={} dual={}]", m.native_api().get_it_cnt(m.native_model()), gs(p), ps(p), ds(p));
    });
}
