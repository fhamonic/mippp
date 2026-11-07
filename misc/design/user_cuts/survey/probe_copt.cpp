#include "probe_copt_common.cpp"
int main() {
    cp::backend<M> B;
    B.attach = attach; B.scope_settings = scope_settings; B.one_thread = one_thread;
    B.cut_modes = {"usercut", "usercuts_batch", "usercut_nonode", "lazy_relax"};
    cp::run_matrix(B);
}
