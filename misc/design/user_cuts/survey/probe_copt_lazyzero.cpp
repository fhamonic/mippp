// The existing wrapper path on a continuous column: real copt_milp, real add_lazy_constraint.
// Facility location; on a candidate with few facilities open, a lazy floor on their number, with or
// without an expression that coalesces to an explicit 0.0 on a flow column. Both must give one value.
#include "probe_copt_common.cpp"
using namespace std::chrono;
int main() {
    for(const bool zero : {false, true}) {
        c::copt_milp m; m.set_thread_limit(1); m.set_time_limit(duration<double>(20.0));
        const std::size_t F = 12, C = 40;
        std::mt19937 rng(3);
        std::uniform_real_distribution<double> P(0, 100), dem(5, 35), fix(100, 300);
        std::vector<std::pair<double, double>> fp(F), cp_(C);
        for(auto & p : fp) p = {P(rng), P(rng)};
        for(auto & p : cp_) p = {P(rng), P(rng)};
        std::vector<double> d(C), f(F), cap(F, 0);
        for(auto & v : d) v = std::round(dem(rng));
        for(auto & v : f) v = std::round(fix(rng));
        double total = 0; for(double v : d) total += v;
        for(auto & v : cap) v = std::round(total * 3.0 / static_cast<double>(F));
        auto y = m.add_binary_variables(F);
        auto x = m.add_variables(F * C);
        const auto id = [&](std::size_t i, std::size_t j) { return i * F + j; };
        const auto Cs = std::views::iota(std::size_t{0}, C);
        const auto Fs = std::views::iota(std::size_t{0}, F);
        m.set_minimization();
        m.set_objective(xsum(Fs, [&](std::size_t j) { return f[j] * y(j); }) + xsum(Cs, [&](std::size_t i) { return xsum(Fs, [&, i](std::size_t j) { return (0.2 * std::hypot(cp_[i].first - fp[j].first, cp_[i].second - fp[j].second)) * x(id(i, j)); }); }));
        for(std::size_t i : Cs) m.add_constraint(xsum(Fs, [&, i](std::size_t j) { return 1.0 * x(id(i, j)); }) == d[i]);
        for(std::size_t j : Fs) m.add_constraint(xsum(Cs, [&, j](std::size_t i) { return 1.0 * x(id(i, j)); }) - cap[j] * y(j) <= 0);
        long lazies = 0;
        const double floor_open = 7;
        m.set_candidate_solution_callback([&](auto & h) {
            auto sol = h.get_solution();
            double open = 0; for(std::size_t j : Fs) open += sol[y(j)];
            if(open >= floor_open - 1e-6) return;
            ++lazies;
            auto s = xsum(Fs, [&](std::size_t j) { return 1.0 * y(j); });
            if(zero) h.add_lazy_constraint(s + 1.0 * x(id(1, 1)) - 1.0 * x(id(1, 1)) >= floor_open);
            else h.add_lazy_constraint(s >= floor_open);
        });
        const auto t0 = clk::now();
        try { m.solve(); } catch(const std::exception & e) { std::println(stderr, "solve threw {}", e.what()); }
        std::string val = "throws"; try { val = std::format("{:.6g}", m.get_solution_value()); } catch(...) {}
        std::println(stderr, "lazy floor, real wrapper, explicit zero on a continuous column={}: status={} value={} nodes={} lazies={} in {:.2f}s", zero, cp::status_name(m.get_status()), val, m.get_node_count(), lazies, duration<double>(clk::now() - t0).count());
    }
}
