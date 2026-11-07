// Market split with slacks (test/test_suites/node_count.hpp), and capacitated
// facility location with tight capacity: no-op candidate callback vs none.
#include SOLVER_HEADER
#include "probe_common.hpp"
using M = SOLVER_MODEL;
template <class X>
static void market(X & model, std::size_t m) {
    const std::size_t n = 10 * (m - 1);
    std::mt19937 rng(42);
    std::uniform_int_distribution<int> coef(0, 99);
    auto x = model.add_binary_variables(n);
    auto plus = model.add_variables(m);
    auto minus = model.add_variables(m);
    model.set_minimization();
    model.set_objective(xsum(std::views::iota(std::size_t{0}, m), [&](auto i) { return plus(i) + minus(i); }));
    std::vector<int> row(n);
    for(std::size_t i = 0; i < m; ++i) {
        int total = 0;
        for(int & a : row) total += (a = coef(rng));
        model.add_constraint(xsum(std::views::iota(std::size_t{0}, n), [&](auto j) { return row[j] * x(j); }) + plus(i) - minus(i) == total / 2);
    }
}
template <class X>
static void cfl(X & m, int fac, int cust) {
    std::mt19937 rng(3);
    std::uniform_real_distribution<double> F(200, 400), D(5, 30), P(0, 100);
    std::vector<std::pair<double,double>> pf(fac), pc(cust);
    for(auto & p : pf) p = {P(rng), P(rng)};
    for(auto & p : pc) p = {P(rng), P(rng)};
    auto y = m.add_binary_variables(static_cast<std::size_t>(fac));
    auto x = m.add_variables(static_cast<std::size_t>(fac * cust));
    auto id = [&](int i, int j) { return static_cast<std::size_t>(i * cust + j); };
    std::vector<double> fc(fac), dem(cust), cap(fac);
    double total = 0;
    for(auto & v : fc) v = F(rng);
    for(auto & v : dem) { v = D(rng); total += v; }
    for(auto & v : cap) v = total * 1.5 / fac;
    auto fs = std::views::iota(0, fac); auto cs = std::views::iota(0, cust);
    m.set_minimization();
    m.set_objective(xsum(fs, [&](int i) { return fc[i] * y(static_cast<std::size_t>(i)); }) +
                    xsum(fs, [&](int i) { return xsum(cs, [&, i](int j) {
                        double dx = pf[i].first - pc[j].first, dy = pf[i].second - pc[j].second;
                        return std::sqrt(dx*dx + dy*dy) * x(id(i, j)); }); }));
    for(int j = 0; j < cust; ++j)
        m.add_constraint(xsum(fs, [&, j](int i) { return 1.0 * x(id(i, j)); }) == dem[j]);
    for(int i = 0; i < fac; ++i)
        m.add_constraint(xsum(cs, [&, i](int j) { return 1.0 * x(id(i, j)); }) - cap[i] * y(static_cast<std::size_t>(i)) <= 0.0);
}
template <class B>
static void compare(const char * name, B build) {
    using std::chrono::duration;
    { M m; m.set_thread_limit(1); build(m); m.set_time_limit(duration<double>(40)); auto o = timed_solve(m); final_line(std::format("{} no callback", name), m, o.solve_s); }
    { M m; m.set_thread_limit(1); build(m); m.set_time_limit(duration<double>(40));
      long calls = 0; m.set_candidate_solution_callback([&](auto & h) { ++calls; (void)h.get_solution_value(); });
      auto o = timed_solve(m); final_line(std::format("{} no-op candidate, calls={}", name, calls), m, o.solve_s); }
}
int main() {
    compare("market split m=5", [](M & m) { market(m, 5); });
    compare("tight CFL 15x30", [](M & m) { cfl(m, 15, 30); });
}
