// Public API only: a candidate callback that adds a lazy constraint the candidate
// already satisfies. Does the shipped backend accept the candidate, loop, or lose
// the optimum? Build with -DBACKEND_COPT, -DBACKEND_CPLEX or -DBACKEND_GUROBI.
#if defined(BACKEND_COPT)
#include "mippp/solvers/copt/all.hpp"
using M = mippp::copt_milp;
#elif defined(BACKEND_CPLEX)
#include "mippp/solvers/cplex/all.hpp"
using M = mippp::cplex_milp;
#else
#include "mippp/solvers/gurobi/all.hpp"
using M = mippp::gurobi_milp;
#endif
#include "cut_common.hpp"

static void report(const char * name, M & m, double t, long calls, long adds, double truth) {
    std::string val = "none";
    try { val = std::format("{:.6g}", m.get_solution_value()); } catch(...) {}
    std::println("{} | {} | value={} truth={} calls={} lazy_adds={} solve={:.2f}s", name, cp::status_name(m.get_status()), val, truth, calls, adds, t);
}

int main(int argc, char ** argv) {
    const int threads = argc > 1 ? std::atoi(argv[1]) : 1;
    // A: max x + y, integers in [0,5], x + y <= 7. Every candidate gets x + y <= 100.
    // B: same, the satisfied row only at a candidate of value 7 (the optimum).
    for(int mode : {0, 1}) {
        M m;
        m.set_thread_limit(static_cast<std::size_t>(threads));
        m.set_time_limit(std::chrono::duration<double>(5.0));
        auto x = m.add_integer_variable({.obj_coef = 0, .lower_bound = 0, .upper_bound = 5});
        auto y = m.add_integer_variable({.obj_coef = 0, .lower_bound = 0, .upper_bound = 5});
        m.set_maximization();
        m.set_objective(x + y);
        m.add_constraint(x + y <= 7);
        long calls = 0, adds = 0;
        m.set_candidate_solution_callback([&](auto & h) {
            ++calls;
            auto s = h.get_solution();
            const double v = s[x] + s[y];
            if(mode == 0 || v >= 7 - 1e-6) { h.add_lazy_constraint(x + y <= 100); ++adds; }
        });
        const double t = cp::timed_solve(m);
        report(mode == 0 ? "A every candidate gets a satisfied lazy" : "B only the optimum gets a satisfied lazy", m, t, calls, adds, 7);
    }
    // C: Chu-Beasley 5x30, every candidate gets one of the model's own rows (satisfied).
    for(unsigned seed : {1u, 2u, 3u}) {
        double truth = 0;
        {
            M b; b.set_thread_limit(static_cast<std::size_t>(threads)); b.set_time_limit(std::chrono::duration<double>(5.0));
            cp::build_mkp(b, 5, 30, seed); cp::timed_solve(b);
            try { truth = b.get_solution_value(); } catch(...) {}
        }
        M m;
        m.set_thread_limit(static_cast<std::size_t>(threads));
        m.set_time_limit(std::chrono::duration<double>(5.0));
        cp::build_mkp(m, 5, 30, seed);
        const auto vars = m.variables();  // read before the solve, not from the callback
        long calls = 0, adds = 0;
        m.set_candidate_solution_callback([&](auto & h) {
            ++calls;
            h.add_lazy_constraint(xsum(vars) <= 30.0);
            ++adds;
        });
        const double t = cp::timed_solve(m);
        report(std::format("C mkp5x30 seed={} every candidate gets sum x <= 30", seed).c_str(), m, t, calls, adds, truth);
    }
    return 0;
}
