#include "verify_cplex_common.hpp"
using namespace cp;
using std::chrono::duration;
int main() {
    M m; m.set_thread_limit(1);
    const std::size_t C = 30;
    std::mt19937 rng(2);
    std::uniform_int_distribution<int> A(1, 40), P(5, 60);
    auto x = m.add_binary_variables(C);
    auto zz = m.add_integer_variables(1, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 10});
    auto ww = m.add_binary_variables(1);
    std::vector<double> c(C); for(auto & v : c) v = P(rng);
    const auto ids = std::views::iota(std::size_t{0}, C);
    m.set_maximization();
    m.set_objective(xsum(ids, [&](std::size_t j) { return c[j] * x(j); }) + 400.0 * zz(std::size_t{0}) + 400.0 * ww(std::size_t{0}));
    for(int r = 0; r < 3; ++r) {
        std::vector<double> a(C); double t = 0; for(auto & v : a) { v = A(rng); t += v; }
        m.add_constraint(xsum(ids, [&](std::size_t j) { return a[j] * x(j); }) + 3.0 * zz(std::size_t{0}) <= std::floor(t * 0.4));
    }
    m.add_constraint(x(std::size_t{0}) + x(std::size_t{1}) + 2.0 * ww(std::size_t{0}) == 1);
    m.add_constraint(2.0 * zz(std::size_t{0}) <= 5);
    const int w = m.native_id(ww(std::size_t{0})), z = m.native_id(zz(std::size_t{0}));
    scope_settings(m); vparam(m, CPX_PARAM_MIRCUTS, 2); vparam(m, CPX_PARAM_FRACCUTS, 2);
    vparam(m, CPX_PARAM_SCRIND, 1);
    m.set_time_limit(duration<double>(5));
    int n = 0;
    auto & h = vattach(m, [&](const std::vector<double> & p, const add_fn &) { if(n++ < 5) std::println(stderr, "call {} w={} z={} x0={} x1={}", n, p[static_cast<std::size_t>(w)], p[static_cast<std::size_t>(z)], p[0], p[1]); });
    timed_solve(m);
    std::println(stderr, "{} | {}", summary(m, 0), h.line());
}
