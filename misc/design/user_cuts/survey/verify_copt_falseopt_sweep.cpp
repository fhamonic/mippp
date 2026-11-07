// How often does COPT, with no hook at all (empty callback slot), disagree with Gurobi beyond the default gap?
// usage: verify_copt_falseopt_sweep <family> from to [noise] ; dump=1 writes MPS of each mismatch.
#include "verify_copt_common.hpp"
#include "mippp/solvers/gurobi/all.hpp"
using namespace std::chrono;
template <class Mdl>
void build_family(Mdl & m, const std::string & fam, unsigned seed) {
    if(fam == "nd") { setenv("CAPF", "0.2", 1); build_mcnd(m, 8, 10, 6, seed); return; }
    if(fam == "mkp") { cp::build_mkp(m, 5, 30, seed); return; }
    if(fam == "mkp40") { cp::build_mkp(m, 5, 40, seed); return; }
    std::mt19937 rng(seed);
    const std::size_t cols = 35, rows = 15;
    std::uniform_int_distribution<int> A(-5, 20), C(1, 40);
    std::vector<std::vector<double>> a(rows, std::vector<double>(cols));
    for(auto & r : a) for(auto & v : r) v = A(rng);
    std::vector<double> b(rows), c(cols);
    for(std::size_t i = 0; i < rows; ++i) { double t = 0; for(double v : a[i]) t += std::max(v, 0.0); b[i] = std::floor(t * 0.4); }
    for(auto & v : c) v = C(rng);
    const auto ids = std::views::iota(std::size_t{0}, cols);
    if(fam == "cg") {
        auto x = m.add_binary_variables(cols);
        m.set_maximization();
        m.set_objective(xsum(ids, [&](std::size_t j) { return c[j] * x(j); }));
        for(std::size_t i = 0; i < rows; ++i) m.add_constraint(xsum(ids, [&, i](std::size_t j) { return a[i][j] * x(j); }) <= b[i]);
    } else {
        auto x = m.add_integer_variables(cols, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 3});
        auto s = m.add_variables(cols, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 10});
        m.set_maximization();
        m.set_objective(xsum(ids, [&](std::size_t j) { return c[j] * x(j) + 0.3 * c[j] * s(j); }));
        for(std::size_t i = 0; i < rows; ++i) m.add_constraint(xsum(ids, [&, i](std::size_t j) { return a[i][j] * x(j); }) <= 3.0 * b[i]);
        for(std::size_t j = 0; j < cols; ++j) m.add_constraint(s(j) - 2.5 * x(j) <= 0.0);
        m.add_constraint(xsum(ids, [&](std::size_t j) { return 1.0 * s(j); }) <= 60.0);
    }
}
int main(int argc, char ** argv) {
    const std::string fam = argv[1];
    const unsigned from = std::atoi(argv[2]), to = std::atoi(argv[3]);
    const bool noisy = argc > 4 && std::string(argv[4]) == "noise";
    using wm_fn = int (*)(c::copt_prob *, const char *);
    int cases = 0, mism = 0, nofinish = 0;
    for(unsigned seed = from; seed <= to; ++seed) {
        V m; if(!std::getenv("COPT_THREADS_DEFAULT")) m.iparam("Threads", 1); if(const char * e = std::getenv("COPT_INT")) { std::string s(e); auto p = s.find('='); m.iparam(s.substr(0, p).c_str(), std::atoi(s.substr(p + 1).c_str())); } m.set_time_limit(duration<double>(20.0)); build_family(m, fam, seed); m.empty_slot = true;
        mippp::gurobi_milp g; g.set_thread_limit(1); g.set_time_limit(duration<double>(20.0)); build_family(g, fam, seed); g.set_optimality_tolerance(0.0);
        if(noisy) {
            using getcol_fn = int (*)(c::copt_prob *, const char *, int, const int *, double *);
            auto getcol = cp::symf<getcol_fn>(m.native_api().library_path(), "COPT_GetColInfo");
            std::vector<int> ids; for(auto v : m.variables()) ids.push_back(m.native_id(v));
            std::vector<double> o(ids.size()); getcol(m.native_model().second, "Obj", (int)ids.size(), ids.data(), o.data());
            std::mt19937 rng(seed * 977u); std::uniform_real_distribution<double> noise(0.8, 1.2);
            for(auto & w : o) w = std::round(w * noise(rng) * 100) / 100;
            { auto ff = *m.variables().begin(); std::vector<decltype(ff)> fv; for(auto v : m.variables()) fv.push_back(v);
              m.set_objective(xsum(std::views::iota(std::size_t{0}, ids.size()), [&](std::size_t j) { return o[j] * fv[j]; })); }
            { auto ff = *g.variables().begin(); std::vector<decltype(ff)> fv; for(auto v : g.variables()) fv.push_back(v);
              g.set_objective(xsum(std::views::iota(std::size_t{0}, ids.size()), [&](std::size_t j) { return o[j] * fv[j]; })); }
        }
        const double t1 = cp::timed_solve(m), t2 = cp::timed_solve(g);
        if(!is_a<status::optimal>(m.get_status()) || !is_a<status::optimal>(g.get_status())) { ++nofinish; std::println(stderr, "{} seed={} unfinished: COPT {} | GUROBI {}", fam, seed, cp::summary(m, t1), cp::summary(g, t2)); continue; }
        ++cases;
        const double vc = m.get_solution_value(), vg = g.get_solution_value();
        const bool bad = std::fabs(vc - vg) > 1.1e-4 * std::fabs(vg);
        mism += bad;
        if(bad) {
            std::println(stderr, "{} seed={}{} MISMATCH COPT {} | GUROBI {} | rel diff {:.2e}", fam, seed, noisy ? " noise" : "", cp::summary(m, t1), cp::summary(g, t2), std::fabs(vc - vg) / std::fabs(vg));
            if(std::getenv("DUMP")) { auto wm = cp::symf<wm_fn>(m.native_api().library_path(), "COPT_WriteMps"); wm(m.native_model().second, std::format("vout_copt_falseopt_{}_{}{}.mps", fam, seed, noisy ? "n" : "").c_str()); }
        }
    }
    std::println(stderr, "== {}{} seeds {}..{}: compared={} mismatches={} unfinished={}", fam, noisy ? " noise" : "", from, to, cases, mism, nofinish);
}
