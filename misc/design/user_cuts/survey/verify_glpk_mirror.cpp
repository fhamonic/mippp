// Harness check: M (no hook, presolve on) must walk the same path as the real glpk_milp.
#include "verify_glpk_model.hpp"
template <class T> void build(T & m, int f, unsigned s) { if(f == 0) cp::build_atsp(m, 14, false, s); else if(f == 1) cp::build_mkp(m, 5, 30, s); else cp::build_gadgets(m, 8, false); }
using vg::M;
int main() {
    for(int f = 0; f < 3; ++f) for(unsigned s : {1u, 2u, 3u}) {
        Base a; build(a, f, s); a.set_time_limit(std::chrono::duration<double>(60)); a.solve();
        M b; build(b, f, s); b.set_time_limit(std::chrono::duration<double>(60)); b.solve();
        const int ita = a.native_api().get_it_cnt(a.native_model());
        std::println(stderr, "family {} seed {}: glpk_milp it={} value={} {} | M it={} value={} {} => {}", f, s, ita, a.get_solution_value(), cp::status_name(a.get_status()), b.iters, b.get_solution_value(), cp::status_name(b.get_status()), ita == b.iters ? "SAME" : "DIFFERENT");
    }
}
