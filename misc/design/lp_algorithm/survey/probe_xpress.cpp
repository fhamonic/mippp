#include "mippp/solvers/xpress/all.hpp"
#include "probe_common.hpp"

using M = mippp::xpress_lp;
namespace v1 = mippp::xpress::impl::v1;
constexpr int DEFAULTALG = 8023, CROSSOVER = 8044, LPFLAGS = 8385;

static auto cfg(const char * flags, std::vector<std::pair<int, int>> controls) {
    return [=](M & m) {
        for(auto [c, v] : controls)
            m.native_api()._check(m.native_model(),
                                  m.native_api().setintcontrol(m.native_model(), c, v));
        v1::probe_flags = flags;
    };
}

int main() {
    reset_hooks = [] { v1::probe_flags = nullptr; };
    std::vector<probe_config<M>> configs = {
        {"today: flags nullptr", cfg(nullptr, {})},
        {"flags \"p\"", cfg("p", {})},
        {"flags \"d\"", cfg("d", {})},
        {"flags \"b\"", cfg("b", {})},
        {"flags \"b\", CROSSOVER 0", cfg("b", {{CROSSOVER, 0}})},
        {"flags \"n\"", cfg("n", {})},
        {"LPFLAGS 1 dual", cfg(nullptr, {{LPFLAGS, 1}})},
        {"LPFLAGS 2 primal", cfg(nullptr, {{LPFLAGS, 2}})},
        {"LPFLAGS 4 barrier", cfg(nullptr, {{LPFLAGS, 4}})},
        {"LPFLAGS 4 barrier, CROSSOVER 0", cfg(nullptr, {{LPFLAGS, 4}, {CROSSOVER, 0}})},
        {"DEFAULTALG 3 primal", cfg(nullptr, {{DEFAULTALG, 3}})},
        {"DEFAULTALG 4 barrier", cfg(nullptr, {{DEFAULTALG, 4}})},
        {"flags \"\" + DEFAULTALG 3", cfg("", {{DEFAULTALG, 3}})},
    };
    run_all<M>(configs, [](M & m) {
        auto & api = m.native_api();
        int s = -1, b = -1, c = -1, a = -1;
        api.getintattrib(m.native_model(), 1009, &s);
        api.getintattrib(m.native_model(), 5001, &b);
        api.getintattrib(m.native_model(), 1051, &c);
        api.getintattrib(m.native_model(), 1049, &a);
        int lps = -1, sol = -1, stop = -1, solve = -1;
        api.getintattrib(m.native_model(), 1010, &lps);
        api.getintattrib(m.native_model(), 1053, &sol);
        api.getintattrib(m.native_model(), 1179, &stop);
        api.getintattrib(m.native_model(), 1394, &solve);
        return std::format("[alg={} simplex={} barrier={} crossover={} LPSTATUS={} SOLSTATUS={} STOPSTATUS={} SOLVESTATUS={}]", a, s, b, c, lps, sol, stop, solve);
    });
}
