// No hook, no callback, one instance: does merely perturbing COPT's path reach a false optimum?
// usage: verify_copt_falseopt_perturb seed truth
#include "verify_copt_common.hpp"
using namespace std::chrono;
int main(int argc, char ** argv) {
    const unsigned seed = std::atoi(argv[1]);
    const double truth = std::atof(argv[2]);
    const std::vector<std::pair<std::string, std::vector<std::pair<const char *, int>>>> sts{
        {"defaults", {}}, {"Threads=2", {{"Threads", 2}}}, {"Threads=4", {{"Threads", 4}}}, {"Threads=8", {{"Threads", 8}}},
        {"CutLevel=0", {{"CutLevel", 0}}}, {"CutLevel=1", {{"CutLevel", 1}}}, {"CutLevel=2", {{"CutLevel", 2}}}, {"CutLevel=3", {{"CutLevel", 3}}},
        {"RootCutLevel=0", {{"RootCutLevel", 0}}}, {"TreeCutLevel=0", {{"TreeCutLevel", 0}}}, {"TreeCutLevel=3", {{"TreeCutLevel", 3}}},
        {"HeurLevel=0", {{"HeurLevel", 0}}}, {"HeurLevel=1", {{"HeurLevel", 1}}}, {"HeurLevel=3", {{"HeurLevel", 3}}},
        {"DivingHeurLevel=0", {{"DivingHeurLevel", 0}}}, {"SubMipHeurLevel=0", {{"SubMipHeurLevel", 0}}}, {"RoundingHeurLevel=0", {{"RoundingHeurLevel", 0}}},
        {"StrongBranching=0", {{"StrongBranching", 0}}}, {"StrongBranching=3", {{"StrongBranching", 3}}}, {"ConflictAnalysis=0", {{"ConflictAnalysis", 0}}},
        {"Presolve=0", {{"Presolve", 0}}}, {"Presolve=1", {{"Presolve", 1}}}, {"Presolve=2", {{"Presolve", 2}}}, {"Presolve=3", {{"Presolve", 3}}},
    };
    for(const auto & [name, ps] : sts) {
        V m; bool threads_set = false; for(auto [k, v] : ps) if(std::string(k) == "Threads") threads_set = true;
        if(!threads_set) m.iparam("Threads", 1);
        for(auto [k, v] : ps) m.iparam(k, v);
        m.set_time_limit(duration<double>(20.0));
        cp::build_mkp(m, 5, 30, seed);
        m.empty_slot = true;
        const double t = cp::timed_solve(m);
        const double v = cf::value_of(m);
        std::println(stderr, "seed={} {:22} {} {}", seed, name, cp::summary(m, t), std::fabs(v - truth) <= 1.1e-4 * truth ? "right" : std::format("FALSE, truth {}", truth));
    }
}
