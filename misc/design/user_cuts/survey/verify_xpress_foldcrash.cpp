// Is the R2 segfault on a repeated column in XPRSpresolverow, with buffers big enough? Prints the call, in the child only.
#include "verify_xpress_hook.hpp"
using namespace mippp; using namespace mippp::operators;
int main(int argc, char ** argv) {
    M m; m.set_thread_limit(1); m.set_time_limit(std::chrono::duration<double>(10));
    auto a = cp::build_atsp(m, 26, false, 5);
    const double slack = argc > 1 ? std::atof(argv[1]) : 1;  // buffers are this many times COLS, plus 1000
    struct D { const xn::xpress_api * api; vx::fns fn; cp::atsp * a; double slack; } d{&m.native_api(), vx::fns(m.native_api().library_path()), &a, slack};
    const auto hook = +[](prob_t cb, void * u, int *) {
        auto & c = *static_cast<D *>(u);
        int input = 0, cols = 0; c.api->getintattrib(cb, vx::A_INPUTCOLS, &input); c.api->getintattrib(cb, vx::A_COLS, &cols);
        std::vector<double> x(static_cast<std::size_t>(input)); int av = 0; c.api->getcallbacksolution(cb, &av, x.data(), 0, input - 1);
        auto cuts = c.a->separate(x);
        if(cuts.empty()) return;
        cp::raw_cut r = cuts.front();
        r.ind.push_back(r.ind[0]); r.val.push_back(0.0);
        const auto big = static_cast<std::size_t>(c.slack * cols) + 1000;
        std::vector<int> pi(big); std::vector<double> pv(big);
        int n = -1, st = -99; double prhs = 0;
        std::println(stderr, "calling presolverow: {} terms, last repeats col {} with value 0, cols={}, buffers {}", r.ind.size(), r.ind.back(), cols, big);
        std::fflush(stderr);
        const int e = c.fn.presolverow(cb, r.sense, static_cast<int>(r.ind.size()), r.ind.data(), r.val.data(), r.rhs, cols, &n, pi.data(), pv.data(), &prhs, &st);
        std::println(stderr, "  returned e={} n={} status={} rhs={}", e, n, st, prhs);
        if(e) return;
        const int s2[2] = {0, n}; const int type = 1;
        std::println(stderr, "calling addcuts with {} terms", n); std::fflush(stderr);
        const int e2 = c.api->addcuts(cb, 1, &type, &r.sense, &prhs, s2, pi.data(), pv.data());
        std::println(stderr, "  returned {}", e2);
    };
    m.native_api().addcboptnode(m.native_model(), hook, &d, 0);
    const double t = cp::timed_solve(m);
    std::println(stderr, "{}", cp::summary(m, t));
}
