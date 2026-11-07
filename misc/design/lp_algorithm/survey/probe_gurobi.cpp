#include "mippp/solvers/gurobi/all.hpp"
#include "probe_common.hpp"

using M = mippp::gurobi_lp;

static void set_int(M & m, const char * p, int v) {
    m.native_api()._check(m.native_model().first,
                          m.native_api().setintparam(m.native_model().first,
                                                     p, v));
}
static auto method(int v, int crossover = -1) {
    return [=](M & m) {
        set_int(m, "Method", v);
        if(crossover != -1) set_int(m, "Crossover", crossover);
    };
}

int main() {
    std::vector<probe_config<M>> configs = {
        {"default (Method -1)", [](M &) {}},
        {"Method 0 primal", method(0)},
        {"Method 1 dual", method(1)},
        {"Method 2 barrier", method(2)},
        {"Method 2 barrier, Crossover 0", method(2, 0)},
        {"Method 3 concurrent", method(3)},
        {"Method 4 det. concurrent", method(4)},
        {"Method 5 det. concurrent simplex", method(5)},
        {"Method 6 PDHG", method(6)},
        {"Method 6 PDHG, Crossover 0", method(6, 0)},
    };
    run_all<M>(configs, [](M & m) {
        auto & api = m.native_api();
        auto * model = m.native_model().second;
        double it = -1;
        int bar = -1, win = -99;
        api.getdblattr(model, "IterCount", &it);
        api.getintattr(model, "BarIterCount", &bar);
        int e = api.getintattr(model, "ConcurrentWinMethod", &win);
        return std::format("[simplex={} barrier={} winner={}]", it, bar,
                           e ? std::string("n/a") : std::to_string(win));
    });
}
