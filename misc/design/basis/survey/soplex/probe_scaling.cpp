// Cost of set_basis through SoPlex_readBasisFile, whose reader names entity j
// "x0x1...xj" (quadratic in the model size). Usage: ./probe_scaling N
#include <dlfcn.h>
#include <sys/resource.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include "mippp/solvers/soplex/all.hpp"

using namespace mippp;
using namespace mippp::operators;
using var_t = mippp::model_variable<int, double>;

static long rss_kb() {
    std::ifstream f("/proc/self/status");
    for(std::string line; std::getline(f, line);)
        if(line.rfind("VmRSS:", 0) == 0) return std::atol(line.c_str() + 6);
    return -1;
}
static long peak_kb() {
    rusage u;
    getrusage(RUSAGE_SELF, &u);
    return u.ru_maxrss;
}
using clk = std::chrono::steady_clock;
static double secs(clk::time_point a) {
    return std::chrono::duration<double>(clk::now() - a).count();
}

int main(int argc, char ** argv) {
    const int n = argc > 1 ? std::atoi(argv[1]) : 1000;
    const soplex_api & api = soplex_api::load();
    void * h = dlopen(api.library_path().c_str(), RTLD_NOW | RTLD_NOLOAD);
    auto colStatus = (int (*)(void *, int))dlsym(h, "SoPlex_basisColStatus");
    auto readBasisFile =
        (int (*)(void *, const char *))dlsym(h, "SoPlex_readBasisFile");
    auto iters = (int (*)(void *))dlsym(h, "SoPlex_getNumIterations");

    auto build = [&](soplex_lp & m) {
        m.set_time_limit(std::chrono::duration<double>(20.0));
        std::vector<var_t> x;
        for(int j = 0; j < n; ++j)
            x.push_back(
                m.add_variable({.lower_bound = 0.0, .upper_bound = 10.0}));
        for(int i = 0; i < n; ++i)
            m.add_constraint(x[(std::size_t)i] + 2 * x[(std::size_t)((i + 1) % n)] +
                                 x[(std::size_t)((i + 7) % n)] <=
                             15.0 + (i % 13));
        runtime_linear_expression<var_t, double> o;
        for(int j = 0; j < n; ++j) o += (1.0 + (j * 7 % 11)) * x[(std::size_t)j];
        m.set_maximization();
        m.set_objective(o);
    };
    soplex_lp a(api);
    build(a);
    auto t = clk::now();
    a.solve();
    std::printf("n=%d rows=%d: cold solve %.3f s, %d iterations, obj %.6f\n", n,
                n, secs(t), iters(a.native_model()), a.get_solution_value());
    t = clk::now();
    std::vector<int> cols(n), rows(n);
    for(int j = 0; j < n; ++j) cols[(std::size_t)j] = colStatus(a.native_model(), j);
    for(int i = 0; i < n; ++i)
        rows[(std::size_t)i] = api.basisRowStatus(a.native_model(), i);
    std::printf("  read %d codes per entity: %.4f s\n", 2 * n, secs(t));

    // file with cumulative names, built incrementally
    t = clk::now();
    std::vector<std::string> cn((std::size_t)n), rn((std::size_t)n);
    std::string acc;
    for(int j = 0; j < n; ++j) {
        acc += "x" + std::to_string(j);
        cn[(std::size_t)j] = acc;
    }
    acc.clear();
    for(int i = 0; i < n; ++i) {
        acc += "C" + std::to_string(i);
        rn[(std::size_t)i] = acc;
    }
    const std::string path =
        "/tmp/claude-1000/-home-plaiseek-Projects-mippp/"
        "66409ca5-709c-4c59-b3c1-e7b746a2b680/scratchpad/survey/soplex/"
        "scaling_" +
        std::to_string(n) + ".bas";
    std::size_t lines = 0;
    {
        std::ofstream f(path);
        f << "NAME  soplex.bas\n";
        std::vector<int> nb;
        for(int i = 0; i < n; ++i)
            if(rows[(std::size_t)i] != 4) nb.push_back(i);
        std::size_t k = 0;
        for(int j = 0; j < n; ++j) {
            const int c = cols[(std::size_t)j];
            if(c == 4 && k < nb.size()) {
                const int r = nb[k++];
                f << (rows[(std::size_t)r] == 0 ? " XU " : " XL ")
                  << cn[(std::size_t)j] << "       " << rn[(std::size_t)r]
                  << "\n";
                ++lines;
            } else if(c == 0) {
                f << " UL " << cn[(std::size_t)j] << "\n";
                ++lines;
            }
        }
        f << "ENDATA\n";
    }
    std::ifstream sz(path, std::ios::ate | std::ios::binary);
    const double mb = (double)sz.tellg() / 1e6;
    cn.clear();
    cn.shrink_to_fit();
    rn.clear();
    rn.shrink_to_fit();
    std::printf("  wrote %zu lines, %.1f MB in %.3f s\n", lines, mb, secs(t));

    soplex_lp b(api);
    build(b);
    const long rss0 = rss_kb(), peak0 = peak_kb();
    t = clk::now();
    if(argc > 2) {  // "empty": isolate the reader's own name building
        std::ofstream f(path);
        f << "NAME  soplex.bas\nENDATA\n";
    }
    const int ret = readBasisFile(b.native_model(), path.c_str());
    std::printf(
        "  readBasisFile = %d in %.3f s; RSS %ld -> %ld kB, peak %ld -> %ld kB\n",
        ret, secs(t), rss0, rss_kb(), peak0, peak_kb());
    api.setIntParam(b.native_model(), 5, 0);
    t = clk::now();
    const int s = api.optimize(b.native_model());
    std::printf("  warm solve ITERLIMIT=0: status %d, %d iterations, %.3f s, "
                "obj %.6f\n",
                s, iters(b.native_model()), secs(t),
                api.objValueReal(b.native_model()));
    std::remove(path.c_str());
    return 0;
}
