#include "mippp/solvers/cplex/all.hpp"
#include "probe_common.hpp"

using M = mippp::cplex_lp;
namespace v1 = mippp::cplex::impl::v1;
using opt_fn = int (*)(v1::CPXCENVptr, v1::CPXLPptr);
using cnt_fn = int (*)(v1::CPXCENVptr, v1::CPXCLPptr);
using info_fn = int (*)(v1::CPXCENVptr, v1::CPXCLPptr, int *, int *, int *, int *);

static std::filesystem::path lib() { return M().native_api().library_path(); }
static opt_fn opt(const char * name) { return reinterpret_cast<opt_fn>(sym(lib(), name)); }

static auto cfg(const char * routine, std::vector<std::pair<int, int>> params) {
    return [=](M & m) {
        auto [env, lp] = m.native_model();
        for(auto [p, v] : params)
            m.native_api()._check(env, m.native_api().setintparam(env, p, v));
        v1::probe_hook = routine ? opt(routine) : nullptr;
    };
}
constexpr int LPMethod = 1062, Crossover = 3018, SolutionType = 1147;

int main() {
    reset_hooks = [] { v1::probe_hook = nullptr; };
    std::vector<probe_config<M>> configs = {
        {"today: CPXprimopt", cfg(nullptr, {})},
        {"CPXprimopt with LPMethod 4 (ignored?)", cfg(nullptr, {{LPMethod, 4}})},
        {"CPXlpopt LPMethod 0 auto", cfg("CPXlpopt", {})},
        {"CPXlpopt LPMethod 1 primal", cfg("CPXlpopt", {{LPMethod, 1}})},
        {"CPXlpopt LPMethod 2 dual", cfg("CPXlpopt", {{LPMethod, 2}})},
        {"CPXlpopt LPMethod 3 network", cfg("CPXlpopt", {{LPMethod, 3}})},
        {"CPXlpopt LPMethod 4 barrier", cfg("CPXlpopt", {{LPMethod, 4}})},
        {"CPXlpopt LPMethod 4, Crossover -1", cfg("CPXlpopt", {{LPMethod, 4}, {Crossover, -1}})},
        {"CPXlpopt LPMethod 4, SolutionType 2", cfg("CPXlpopt", {{LPMethod, 4}, {SolutionType, 2}})},
        {"CPXlpopt LPMethod 5 sifting", cfg("CPXlpopt", {{LPMethod, 5}})},
        {"CPXlpopt LPMethod 6 concurrent", cfg("CPXlpopt", {{LPMethod, 6}})},
        {"CPXdualopt", cfg("CPXdualopt", {})},
        {"CPXbaropt", cfg("CPXbaropt", {})},
        {"CPXbaropt, Crossover -1", cfg("CPXbaropt", {{Crossover, -1}})},
    };
    run_all<M>(configs, [](M & m) {
        auto [env, lp] = m.native_model();
        static auto getmethod = reinterpret_cast<cnt_fn>(sym(lib(), "CPXgetmethod"));
        static auto itcnt = reinterpret_cast<cnt_fn>(sym(lib(), "CPXgetitcnt"));
        static auto baritcnt = reinterpret_cast<cnt_fn>(sym(lib(), "CPXgetbaritcnt"));
        static auto solninfo = reinterpret_cast<info_fn>(sym(lib(), "CPXsolninfo"));
        int meth = 0, type = 0, pf = 0, df = 0;
        solninfo(env, lp, &meth, &type, &pf, &df);
        return std::format("[method={} simplex={} barrier={} solntype={}]",
                           getmethod(env, lp), itcnt(env, lp), baritcnt(env, lp), type);
    });
}
