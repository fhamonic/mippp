// The Cbc recipe under the shared conformance set. Sizes via env, see below.
#include "verify_cbc_recipe.hpp"
using vc::M;
static std::vector<int> env_list(const char * name, std::vector<int> d) {
    if(const char * e = std::getenv(name)) { d.clear(); std::stringstream ss(e); std::string t; while(std::getline(ss, t, ',')) d.push_back(std::atoi(t.c_str())); }
    return d;
}
int main() {
    vc::banner();
    cf::setup<M> B;
    B.attach = [](M & m, cp::separator sep) { m.attach(std::move(sep)); };
    B.one_thread = [](M &) {};  // threads defaults to 0, the serial path
    B.atsp_sizes = env_list("CONF_SIZES", {18, 20, 22});
    B.seeds = {1, 2, 3};
    B.stop_size = vc::env_int("CONF_STOP", 26);
    B.first_limit = 0.02;
    B.fuzz_seeds = vc::env_int("CONF_FUZZ", 20);
    B.limit = vc::env_int("CONF_LIMIT", 120);
    std::println(stderr, "sizes [{}] stop_size={} first_limit={} fuzz_seeds={} limit={}", env_list("CONF_SIZES", {18, 20, 22}).front(), B.stop_size, B.first_limit, B.fuzz_seeds, B.limit);
    cf::run(B);
    // Extra cases with power below the root: DFJ closes most of the instances above at the root.
    const int skip = vc::env_int("CONF_SKIP", 25);
    for(int n : {20, 22}) for(unsigned seed : {1u, 2u, 3u}) cf::compare(B, std::format("X1 DFJ only after {} calls n={} seed={}", skip, n, seed), [=](M & m) {
        auto a = cp::build_atsp(m, n, false, seed);
        auto count = std::make_shared<std::atomic<long>>(0);
        return cp::separator([a, count, skip](const std::vector<double> & x, const cp::add_fn & add) { if(count->fetch_add(1) < skip) return; for(const auto & c : a.separate(x, 1)) add(c); });
    });
    for(unsigned seed : {31u, 35u, 9u}) cf::compare(B, std::format("X2 budgeted ATSP that restarts, valid cuts by original index, seed={}", seed), [=](M & m) {
        auto a = cp::build_atsp(m, 24, false, seed);
        vc::add_budget(m, a, 0.72, seed);
        auto full = std::make_shared<std::atomic<int>>(0);
        return cp::separator([a, full](const std::vector<double> & x, const cp::add_fn & add) { if(full->fetch_add(1) >= 3) return; for(const auto & c : a.separate(x, 1)) add(c); });
    });
    return cf::finish();
}
