// The GLPK recipe under the shared conformance set. VG_REMOVE names the one piece
// removed, for the negative controls; VG_ATSP, VG_FUZZ etc. set the sizes.
// final: fp::prepare + stall guard k=5. argv[1]: conf (default), sweep, sweep_ns, bigm, tiny, harm.
#include "final_glpk_model.hpp"
#include "final_tests.hpp"
using vg::M;
static bool env(const char * n) { return std::getenv(n) != nullptr; }
static int envi(const char * n, int d) { const char * s = std::getenv(n); return s ? std::atoi(s) : d; }
static double envd(const char * n, double d) { const char * s = std::getenv(n); return s ? std::atof(s) : d; }
int main(int argc, char ** argv) {
    const std::string mode = argc > 1 ? argv[1] : "conf";
    if(mode != "conf") {
        std::println(stderr, "GLPK {}", M{}.native_api().version());
        ft::bed<M> E;
        E.name = "glpk";
        E.attach = [](M & m, cp::separator sep) { m.p.presolve_off = true; vg::attach(m, std::move(sep), vg::cut_rules{}); };
        E.one_thread = [](M &) {};
        E.presolve_off = [](M & m) { m.p.presolve_off = true; };
        E.recipe_line = [] { auto & c = *vg::kept().back(); return std::format("glpk(offered={} passed={} stall_stops={} threw={})", c.offered, c.passed, c.st_stops, c.threw); };
        if(!ft::run_mode(E, mode)) { std::println(stderr, "unknown mode {}", mode); return 2; }
        return 0;
    }
    const std::string rm = std::getenv("VG_REMOVE") ? std::getenv("VG_REMOVE") : "";
    cf::setup<M> B;
    B.attach = [rm](M & m, cp::separator sep) {
        vg::cut_rules r;
        if(std::getenv("VG_V2")) { r.normalize = true; r.tol = 1e-6; }
        if(std::getenv("VG_STALL")) r.stall_k = std::atoi(std::getenv("VG_STALL"));
        m.p.presolve_off = true;
        if(rm == "presolve") m.p.presolve_off = false;
        if(rm == "root_lp") m.p.root_lp = false;
        if(rm == "scale") m.p.scale = false;
        if(rm == "adv_basis") m.p.adv_basis = false;
        if(rm == "fold") r.fold = false;
        if(rm == "strip") r.strip_zeros = false;
        if(rm == "skip_empty") r.skip_empty = false;
        if(rm == "drop_satisfied") r.drop_satisfied = false;
        if(rm == "split_eq") r.split_eq = false;
        if(rm == "pool") r.direct_rows = true;
        if(rm == "pool_irowgen") { r.direct_rows = true; r.irowgen = true; }
        vg::attach(m, std::move(sep), r);
        m.fork_solve = std::getenv("VG_FORK") != nullptr;
    };
    B.one_thread = [](M &) {};  // glp_iocp has no thread field
    std::vector<int> sizes;
    { std::stringstream ss(std::getenv("VG_ATSP") ? std::getenv("VG_ATSP") : "12,14,16"); std::string t; while(std::getline(ss, t, ',')) sizes.push_back(std::atoi(t.c_str())); }
    B.atsp_sizes = sizes;
    B.limit = envd("VG_LIMIT", 60);
    B.stop_size = envi("VG_STOP", 26);
    B.first_limit = envd("VG_FIRST", 0.02);
    B.fuzz_seeds = envi("VG_FUZZ", 20);
    std::println(stderr, "GLPK {} remove='{}' sizes {} limit {} stop_size {} first_limit {} fuzz {}", M{}.native_api().version(), rm, sizes, B.limit, B.stop_size, B.first_limit, B.fuzz_seeds);
    cf::run(B);
    return cf::finish();
}
