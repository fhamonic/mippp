// The real glpk_milp, nothing derived: does set_time_limit hold when the root LP eats most of it?
#include "verify_glpk_model.hpp"
int main(int argc, char ** argv) {
    const int n = std::atoi(argv[1]);
    const double limit = std::atof(argv[2]);
    Base m;
    cp::build_atsp(m, n, false, 1);
    m.set_time_limit(std::chrono::duration<double>(limit));
    const auto t0 = clk::now();
    m.solve();
    const double t = std::chrono::duration<double>(clk::now() - t0).count();
    std::println(stderr, "glpk_milp atsp n={} rows {} limit {}s: {} total {:.3f}s = {:.2f} x limit GLPK {}", n, m.native_api().get_num_rows(m.native_model()), limit, cp::status_name(m.get_status()), t, t / limit, m.native_api().version());
    return 0;
}
