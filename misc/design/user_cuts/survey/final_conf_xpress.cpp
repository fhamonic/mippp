// Conformance: argv[1] recipe spec, sizes from env (printed).
// final: R1 with fp::prepare. argv[1]: conf (default), sweep, sweep_ns, bigm, tiny, harm; XP_RECIPE overrides R1.
#include "final_xpress_hook.hpp"
#include "final_tests.hpp"
static int env_i(const char * n, int d) { const char * v = std::getenv(n); return v ? std::atoi(v) : d; }
static double env_d(const char * n, double d) { const char * v = std::getenv(n); return v ? std::atof(v) : d; }
int main(int argc, char ** argv) {
    const std::string mode = argc > 1 ? argv[1] : "conf";
    const vx::recipe r = vx::parse(std::getenv("XP_RECIPE") ? std::getenv("XP_RECIPE") : "R1");
    if(mode != "conf") {
        vx::quiet_flush = true;
        ft::bed<M> E;
        E.name = "xpress";
        E.attach = [r](M & m, cp::separator sep) { vx::attach(m, std::move(sep), r); };
        E.one_thread = [](M & m) { m.set_thread_limit(1); };
        E.presolve_off = [](M & m) { vx::setint(m, vx::C_PRESOLVE, 0); };
        E.recipe_line = [] { return vx::contexts().back()->line(); };
        { M m; std::println(stderr, "release {} recipe {}", vx::version(m), r.name); }
        if(!ft::run_mode(E, mode)) { std::println(stderr, "unknown mode {}", mode); return 2; }
        return 0;
    }
    cf::setup<M> B;
    B.attach = [r](M & m, cp::separator sep) { vx::attach(m, std::move(sep), r); };
    B.one_thread = [](M & m) { m.set_thread_limit(1); };
    if(const char * s = std::getenv("SIZES")) { B.atsp_sizes.clear(); std::stringstream ss(s); int v; while(ss >> v) B.atsp_sizes.push_back(v); }
    if(const char * s = std::getenv("SEEDS")) { B.seeds.clear(); std::stringstream ss(s); unsigned v; while(ss >> v) B.seeds.push_back(v); }
    B.limit = env_d("LIMIT", 60);
    B.stop_size = env_i("STOP_SIZE", 30);
    B.first_limit = env_d("FIRST_LIMIT", 0.02);
    B.fuzz_seeds = env_i("FUZZ", 20);
    std::string sizes; for(int v : B.atsp_sizes) sizes += std::format("{} ", v);
    { M m; std::println(stderr, "release {} recipe {} sizes [{}] limit {} stop_size {} first_limit {} fuzz {}", vx::version(m), r.name, sizes, B.limit, B.stop_size, B.first_limit, B.fuzz_seeds); }
    cf::run(B);
    vx::flush_stats();
    return cf::finish();
}
