// The shared matrix on Xpress. Arguments: the cut modes, the first one is the "front" mode
// which E2, E4, E5, S0, P and W use. Default: addcuts managed_global managed_local pool
#include "probe_xpress_hook.hpp"
int main(int argc, char ** argv) {
    cp::backend<M> B;
    B.attach = attach; B.scope_settings = scope_settings; B.one_thread = one_thread;
    for(int i = 1; i < argc; ++i) B.cut_modes.emplace_back(argv[i]);
    if(B.cut_modes.empty()) B.cut_modes = {"addcuts", "managed_global", "managed_local", "pool"};
    if(const char * s = std::getenv("XP_ATSP_LIMIT")) B.atsp_limit = std::atof(s);
    cp::run_matrix(B);
    flush_last();
}
