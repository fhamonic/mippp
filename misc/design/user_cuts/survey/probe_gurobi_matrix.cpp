// The shared matrix on Gurobi. No argument: the three modes, plain global first.
// One argument: that mode only, so that E2, E4, E5, S0, P and W also run on it.
#include "probe_gurobi_common.hpp"

int main(int argc, char ** argv) {
    cp::backend<M> B;
    B.attach = [](M & m, cp::separator sep, const std::string & mode) { attach_hook(m, std::move(sep), mode); };
    B.scope_settings = scope_settings;
    B.one_thread = one_thread;
    B.cut_modes = {"cbcut_precrush", "cbcut_noprecrush", "cblazy_mipnode"};
    if(argc > 1) B.cut_modes = {argv[1]};
    int major, minor, tech;
    {
        M probe;
        probe.native_api().version(&major, &minor, &tech);
        std::println(stderr, "== release {}.{}.{} from {}", major, minor, tech, probe.native_api().library_path().string());
    }
    cp::run_matrix(B);
    flush_ctx();
}
