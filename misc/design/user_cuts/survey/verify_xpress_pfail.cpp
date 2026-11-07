// What does XPRSpresolverow / XPRSaddmanagedcuts do on every failure path? One process per call. Native codes only; nothing is added after.
#include "verify_xpress_hook.hpp"
#include <sys/wait.h>
#include <unistd.h>
using namespace mippp; using namespace mippp::operators;
struct K { std::string name; std::function<cp::raw_cut(int, int, int, int)> make; };
struct D { const xn::xpress_api * api; vx::fns fn; cp::raw_cut cut; bool managed; bool done = false; };
static void hook(prob_t cb, void * u, int, int * p_action) {
    auto & c = *static_cast<D *>(u);
    if(c.done) return;
    c.done = true;
    int cols = 0, input = 0; c.api->getintattrib(cb, vx::A_COLS, &cols); c.api->getintattrib(cb, vx::A_INPUTCOLS, &input);
    const int n0 = static_cast<int>(c.cut.ind.size());
    if(c.managed) {
        const int start[2] = {0, n0};
        const int e = c.fn.addmanaged(cb, 1, 1, &c.cut.sense, &c.cut.rhs, start, c.cut.ind.data(), c.cut.val.data());
        char msg[256] = {}; if(e) c.api->getlasterror(cb, msg);
        std::println(stderr, "   addmanaged e={} [{}] cols={} input={}", e, std::string(msg).substr(0, 90), cols, input);
    } else {
        const auto big = static_cast<std::size_t>(cols) * 4 + 1000;
        std::vector<int> pi(big); std::vector<double> pv(big);
        int n = -1, st = -99; double prhs = 0;
        const int e = c.fn.presolverow(cb, c.cut.sense, n0, c.cut.ind.data(), c.cut.val.data(), c.cut.rhs, cols, &n, pi.data(), pv.data(), &prhs, &st);
        char msg[256] = {}; if(e) c.api->getlasterror(cb, msg);
        std::println(stderr, "   presolverow e={} status={} n={} rhs={} [{}] cols={} input={}", e, st, n, prhs, std::string(msg).substr(0, 90), cols, input);
    }
    std::fflush(stderr);
    (void)p_action;
}
int main() {
    { M m; std::println(stderr, "release {}", vx::version(m)); }
    const std::vector<K> ks = {
        {"E t - s == 0 on merged duplicates", [](int t, int s, int, int) { return cp::raw_cut{{t, s}, {1, -1}, 'E', 0}; }},
        {"E t == y on merged duplicates", [](int t, int, int y, int) { return cp::raw_cut{{t, y}, {1, -1}, 'E', 0}; }},
        {"E t + s - 2y == 0 equal coefficients", [](int t, int s, int y, int) { return cp::raw_cut{{t, s, y}, {1, 1, -2}, 'E', 0}; }},
        {"L t - s <= 0 on merged duplicates", [](int t, int s, int, int) { return cp::raw_cut{{t, s}, {1, -1}, 'L', 0}; }},
        {"G t - s >= 0 on merged duplicates", [](int t, int s, int, int) { return cp::raw_cut{{t, s}, {1, -1}, 'G', 0}; }},
        {"L repeated column", [](int t, int, int, int) { return cp::raw_cut{{t, t}, {1, 1}, 'L', 1}; }},
        {"L column out of range (input cols)", [](int, int, int, int nn) { return cp::raw_cut{{nn}, {1}, 'L', 1}; }},
        {"L column -1", [](int, int, int, int) { return cp::raw_cut{{-1}, {1}, 'L', 1}; }},
        {"bad rowtype X", [](int t, int, int, int) { return cp::raw_cut{{t}, {1}, 'X', 1}; }},
        {"L no terms", [](int, int, int, int) { return cp::raw_cut{{}, {}, 'L', 1}; }},
        {"L NaN coefficient", [](int t, int, int, int) { return cp::raw_cut{{t}, {std::numeric_limits<double>::quiet_NaN()}, 'L', 1}; }},
        {"L infinite rhs", [](int t, int, int, int) { return cp::raw_cut{{t}, {1}, 'L', std::numeric_limits<double>::infinity()}; }},
    };
    for(const auto & k : ks) for(const bool managed : {false, true}) {
        std::println(stderr, "{} via {}", k.name, managed ? "XPRSaddmanagedcuts" : "XPRSpresolverow");
        std::fflush(stderr);
        const pid_t p = fork();
        if(p == 0) {
            alarm(30);
            M m; m.set_thread_limit(1); m.set_time_limit(std::chrono::duration<double>(10));
            // keeps a search alive: a covers-resistant knapsack. Then t + s == 2 y with duplicates t, s.
            std::mt19937 rng(3); std::uniform_int_distribution<int> A(1, 100), R(0, 50);
            const std::size_t cols = 40, rows = 5;
            std::vector<std::vector<double>> a(rows, std::vector<double>(cols));
            for(auto & r : a) for(auto & v : r) v = A(rng);
            std::vector<double> c(cols), cap(rows);
            for(std::size_t j = 0; j < cols; ++j) { double s = 0; for(auto & r : a) s += r[j]; c[j] = std::floor(s / static_cast<double>(rows) + R(rng)); }
            for(std::size_t i = 0; i < rows; ++i) { double s = 0; for(double v : a[i]) s += v; cap[i] = std::floor(s / 2); }
            auto x = m.add_binary_variables(cols);
            const auto ids = std::views::iota(std::size_t{0}, cols);
            m.set_maximization();
            m.set_objective(xsum(ids, [&](std::size_t j) { return c[j] * x(j); }));
            for(std::size_t i = 0; i < rows; ++i) m.add_constraint(xsum(ids, [&, i](std::size_t j) { return a[i][j] * x(j); }) <= cap[i]);
            auto b = m.add_binary_variables(3);
            const std::size_t z = 0;
            m.add_constraint(b(z) + b(z + 1) - 2.0 * b(z + 2) == 0);
            m.set_objective_coefficient(b(z), 10); m.set_objective_coefficient(b(z + 1), 10); m.set_objective_coefficient(b(z + 2), -3);
            D d{&m.native_api(), vx::fns(m.native_api().library_path()), k.make(m.native_id(b(z)), m.native_id(b(z + 1)), m.native_id(b(z + 2)), static_cast<int>(m.num_variables())), managed};
            d.fn.addcbcutround(m.native_model(), hook, &d, 0);
            const double t = cp::timed_solve(m);
            std::println(stderr, "   solve: {}", cp::summary(m, t));
            std::fflush(stderr);
            _exit(0);
        }
        int st = 0; waitpid(p, &st, 0);
        if(WIFSIGNALED(st)) std::println(stderr, "   CRASH signal {}", WTERMSIG(st));
    }
}
