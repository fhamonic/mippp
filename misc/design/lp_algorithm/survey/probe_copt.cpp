#include "mippp/solvers/copt/all.hpp"
#include "probe_common.hpp"

using M = mippp::copt_lp;
namespace v1 = mippp::copt::impl::v1;
using one_fn = int (*)(v1::copt_prob *, const char *, int *);

static auto cfg(int method, int crossover = -99) {
    return [=](M & m) {
        auto prob = m.native_model().second;
        m.native_api()._check(m.native_model().first, m.native_api().SetIntParam(prob, "LpMethod", method));
        if(crossover != -99)
            m.native_api()._check(m.native_model().first, m.native_api().SetIntParam(prob, "Crossover", crossover));
    };
}

int main() {
    {
        M m;
        auto lib = m.native_api().library_path();
        auto fd = reinterpret_cast<one_fn>(sym(lib, "COPT_GetIntParamDef"));
        auto fl = reinterpret_cast<one_fn>(sym(lib, "COPT_GetIntParamMin"));
        auto fh = reinterpret_cast<one_fn>(sym(lib, "COPT_GetIntParamMax"));
        for(const char * p : {"LpMethod", "Crossover", "DualPrice"}) {
            int def = 0, lo = 0, hi = 0;
            fd(m.native_model().second, p, &def);
            fl(m.native_model().second, p, &lo);
            fh(m.native_model().second, p, &hi);
            std::println("{}: default {} range [{}, {}]", p, def, lo, hi);
        }
    }
    std::vector<probe_config<M>> configs = {
        {"default", [](M &) {}},
        {"LpMethod 1 dual simplex", cfg(1)},
        {"LpMethod 2 barrier", cfg(2)},
        {"LpMethod 2 barrier, Crossover 0", cfg(2, 0)},
        {"LpMethod 3 crossover", cfg(3)},
        {"LpMethod 4 concurrent", cfg(4)},
        {"LpMethod 5 heuristic", cfg(5)},
        {"LpMethod 6 PDLP", cfg(6)},
        {"LpMethod 6 PDLP, Crossover 0", cfg(6, 0)},
    };
    run_all<M>(configs, [](M & m) {
        auto & api = m.native_api();
        auto prob = m.native_model().second;
        int s = -1, b = -1, p = -1, st = -1;
        api.GetIntAttr(prob, "SimplexIter", &s);
        api.GetIntAttr(prob, "BarrierIter", &b);
        api.GetIntAttr(prob, "PDLPIter", &p);
        api.GetIntAttr(prob, "LpStatus", &st);
        return std::format("[simplex={} barrier={} pdlp={} LpStatus={}]", s, b, p, st);
    });
}
