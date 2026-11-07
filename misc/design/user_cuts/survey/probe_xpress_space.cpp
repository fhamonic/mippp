// Column space, per mode, under DEFAULT presolve. cut_common's E3 and E4 run on instances where the
// x columns come first, so presolve removing later columns leaves their indices alone and a naive
// wrapper passes by luck. Here K fixed columns come first, so every presolved index is shifted.
// Instances: plain (E3), complements (E4), shifted. argv: modes
#include "probe_xpress_hook.hpp"
static double baseline(int n) {
    M m; one_thread(m); cp::build_atsp(m, n); m.set_time_limit(std::chrono::duration<double>(60.0));
    cp::timed_solve(m);
    return m.get_solution_value();
}
int main(int argc, char ** argv) {
    std::vector<std::string> modes;
    for(int i = 1; i < argc; ++i) modes.emplace_back(argv[i]);
    if(modes.empty()) modes = {"addcuts", "naive", "cutround_addcuts", "cutround_naive", "managed_global", "managed_local", "pool", "pool_global", "point_lpsol", "point_presolved", "managed_optnode"};
    const int n = 26;
    const double base = baseline(n);
    std::println(stderr, "baseline n={} value={}", n, base);
    const auto run = [&](const std::string & label, const std::string & mode, const int fixed_first, const bool complements) {
        M m; one_thread(m);
        if(const char * s = std::getenv("XP_DUALRED")) set_int(m, C_MIPDUALREDUCTIONS, std::atoi(s));
        if(fixed_first) m.add_variables(static_cast<std::size_t>(fixed_first), {.obj_coef = 0, .lower_bound = 0, .upper_bound = 0});
        auto a = cp::build_atsp(m, n, complements);
        m.set_time_limit(std::chrono::duration<double>(30.0));
        auto s = std::make_shared<cp::stats>();
        attach(m, cp::watched([&a](const auto & x, const cp::add_fn & add) { for(const auto & c : a.separate(x)) add(c); }, s,
            [&a](const auto & x) { return a.objective(x); }, [&a](const auto & x) { return a.point_ok(x); }, [&a](const auto & x) { return a.integral(x); }), mode);
        const double t = cp::timed_solve(m);
        double v = NAN; try { v = m.get_solution_value(); } catch(const std::exception &) {}
        std::println(stderr, "== {} mode {}: {} => {}\n   {}", label, mode, cp::summary(m, t), std::fabs(v - base) < 1e-6 ? "value right" : "VALUE WRONG", s->line());
        flush_last();
    };
    for(const auto & mode : modes) {
        run("E3 plain", mode, 0, false);
        run("E4 complements", mode, 0, true);
        run("E3s 40 fixed columns first", mode, 40, false);
    }
}
