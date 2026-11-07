#include "mippp/solvers/clp/all.hpp"
#include "probe_common.hpp"

using M = mippp::clp_lp;
namespace v1 = mippp::clp::impl::v1;
using S = v1::Clp_Simplex;
using one = int (*)(S *);
using two = int (*)(S *, int);

static std::filesystem::path lib() { return M().native_api().library_path(); }
static two primal, dual;
static one entry;

static auto cfg_two(two * f) {
    return [=](M &) {
        static two * g; g = f;
        v1::probe_hook = [](S * s) { return (*g)(s, 0); };
    };
}
static auto cfg_one(const char * name) {
    return [=](M &) {
        entry = reinterpret_cast<one>(sym(lib(), name));
        v1::probe_hook = [](S * s) { return entry(s); };
    };
}

int main() {
    primal = reinterpret_cast<two>(sym(lib(), "Clp_primal"));
    dual = reinterpret_cast<two>(sym(lib(), "Clp_dual"));
    reset_hooks = [] { v1::probe_hook = nullptr; };
    std::vector<probe_config<M>> configs = {
        {"today: Clp_primal", [](M &) { v1::probe_hook = nullptr; }},
        {"Clp_dual", cfg_two(&dual)},
        {"Clp_initialSolve", cfg_one("Clp_initialSolve")},
        {"Clp_initialPrimalSolve", cfg_one("Clp_initialPrimalSolve")},
        {"Clp_initialDualSolve", cfg_one("Clp_initialDualSolve")},
        {"Clp_initialBarrierSolve", cfg_one("Clp_initialBarrierSolve")},
        {"Clp_initialBarrierNoCrossSolve", cfg_one("Clp_initialBarrierNoCrossSolve")},
    };
    run_all<M>(configs, [](M & m) {
        static auto iters = reinterpret_cast<one>(sym(lib(), "Clp_numberIterations"));
        static auto status = reinterpret_cast<one>(sym(lib(), "Clp_status"));
        static auto sec = reinterpret_cast<one>(sym(lib(), "Clp_secondaryStatus"));
        return std::format("[iterations={} status={} secondary={}]", iters(m.native_model()),
                           status(m.native_model()), sec(m.native_model()));
    });
}
