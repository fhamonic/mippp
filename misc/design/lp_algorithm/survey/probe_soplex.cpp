#include "mippp/solvers/soplex/all.hpp"
#include "probe_common.hpp"

using M = mippp::soplex_lp;

static auto alg(int v) {
    return [=](M & m) { m.native_api().setIntParam(m.native_model(), 2, v); };
}

int main() {
    std::vector<probe_config<M>> configs = {
        {"default (ALGORITHM 1 dual)", [](M &) {}},
        {"ALGORITHM 0 primal", alg(0)},
        {"ALGORITHM 1 dual", alg(1)},
        {"REPRESENTATION column, ALGORITHM 0", [](M & m) { m.native_api().setIntParam(m.native_model(), 1, 1); alg(0)(m); }},
        {"REPRESENTATION column, ALGORITHM 1", [](M & m) { m.native_api().setIntParam(m.native_model(), 1, 1); alg(1)(m); }},
        {"REPRESENTATION row, ALGORITHM 0", [](M & m) { m.native_api().setIntParam(m.native_model(), 1, 2); alg(0)(m); }},
        {"REPRESENTATION row, ALGORITHM 1", [](M & m) { m.native_api().setIntParam(m.native_model(), 1, 2); alg(1)(m); }},
    };
    run_all<M>(configs, [](M & m) {
        static auto it = reinterpret_cast<int (*)(void *)>(
            sym(m.native_api().library_path(), "SoPlex_getNumIterations"));
        static auto gp = reinterpret_cast<int (*)(void *, int)>(
            sym(m.native_api().library_path(), "SoPlex_getIntParam"));
        return std::format("[iterations={} ALGORITHM={} REPRESENTATION={}]", it(m.native_model()),
                           gp(m.native_model(), 2), gp(m.native_model(), 1));
    });
}
