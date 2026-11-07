// The shared matrix on cplex_milp.
#include "probe_cplex_common.cpp"
int main() {
    cp::backend<M> B;
    B.attach = attach; B.scope_settings = scope_settings; B.one_thread = one_thread;
    B.cut_modes = {"global-purge", "global-filter", "global-force", "local-purge", "local-filter", "local-force"};
    B.atsp_n = env_int("CUT_ATSP_N", 26);
    B.atsp_complement_n = 20;
    B.atsp_limit = 60;
    cp::run_matrix(B);
    flush_diag();
}
