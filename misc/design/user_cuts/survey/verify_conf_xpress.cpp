// Conformance: argv[1] recipe spec, sizes from env (printed).
#include "verify_xpress_hook.hpp"
static int env_i(const char * n, int d) { const char * v = std::getenv(n); return v ? std::atoi(v) : d; }
static double env_d(const char * n, double d) { const char * v = std::getenv(n); return v ? std::atof(v) : d; }
int main(int argc, char ** argv) {
    const vx::recipe r = vx::parse(argc > 1 ? argv[1] : "R1");
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
