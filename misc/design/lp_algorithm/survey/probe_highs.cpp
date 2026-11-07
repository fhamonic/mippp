#include "mippp/solvers/highs/all.hpp"
#include "probe_common.hpp"

using M = mippp::highs_lp;

static auto opts(std::vector<std::pair<const char *, std::string>> s,
                 int strategy = -1) {
    return [=](M & m) {
        for(auto & [k, v] : s)
            if(m.native_api().setStringOptionValue(m.native_model(), k,
                                                   v.c_str()) != 0)
                throw std::runtime_error(std::string("rejected ") + k + "=" + v);
        if(strategy >= 0 &&
           m.native_api().setIntOptionValue(m.native_model(),
                                            "simplex_strategy", strategy) != 0)
            throw std::runtime_error("rejected simplex_strategy");
    };
}

int main() {
    std::vector<probe_config<M>> configs = {
        {"default (solver=choose)", [](M &) {}},
        {"simplex", opts({{"solver", "simplex"}})},
        {"simplex, strategy 1 dual", opts({{"solver", "simplex"}}, 1)},
        {"simplex, strategy 4 primal", opts({{"solver", "simplex"}}, 4)},
        {"choose, strategy 4 primal", opts({}, 4)},
        {"ipm", opts({{"solver", "ipm"}})},
        {"ipm, run_crossover off", opts({{"solver", "ipm"}, {"run_crossover", "off"}})},
        {"ipx, run_crossover off", opts({{"solver", "ipx"}, {"run_crossover", "off"}})},
        {"hipo", opts({{"solver", "hipo"}})},
        {"pdlp", opts({{"solver", "pdlp"}})},
        {"pdlp, run_crossover on", opts({{"solver", "pdlp"}, {"run_crossover", "on"}})},
    };
    run_all<M>(configs, [](M & m) {
        auto & api = m.native_api();
        int s = -1, i = -1, c = -1, p = -1;
        api.getIntInfoValue(m.native_model(), "simplex_iteration_count", &s);
        api.getIntInfoValue(m.native_model(), "ipm_iteration_count", &i);
        api.getIntInfoValue(m.native_model(), "crossover_iteration_count", &c);
        api.getIntInfoValue(m.native_model(), "pdlp_iteration_count", &p);
        int ps = -1, ds = -1;
        api.getIntInfoValue(m.native_model(), "primal_solution_status", &ps);
        api.getIntInfoValue(m.native_model(), "dual_solution_status", &ds);
        return std::format("[simplex={} ipm={} crossover={} pdlp={} model_status={} primal_sol={} dual_sol={}]", s, i, c, p,
                           api.getModelStatus(m.native_model()), ps, ds);
    });
}
