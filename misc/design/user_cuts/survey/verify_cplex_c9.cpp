// C9: with RELAXATION in the mask, the shipped callback_fun sends that context to the else branch.
#include "mippp/solvers/cplex/all.hpp"
#include "cut_common.hpp"
using namespace cp;
using M = mippp::cplex_milp;
int main() {
    M m; m.set_thread_limit(1); build_mkp(m, 5, 60, 3); m.set_time_limit(std::chrono::duration<double>(5));
    long cand = 0;
    m.set_candidate_solution_callback([&](auto &) { ++cand; });
    const double t = timed_solve(m);
    std::println(stderr, "C9 {} | candidate lambda calls={} | else branch with an empty progress_callback: {} std::bad_function_call, contextid 0x{:x}", summary(m, t), cand, mippp::cplex::impl::v1::verify_else_empty, mippp::cplex::impl::v1::verify_else_ctx);
}
