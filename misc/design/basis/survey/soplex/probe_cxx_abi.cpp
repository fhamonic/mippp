// Feasibility check only: set a basis through the C++ symbols the 7.1.3 .so
// exports (SoPlexBase<double>::readBasisFile with caller-built NameSets),
// bypassing the C wrapper's quadratic default names. Itanium ABI, 7.1.x
// mangling (NameSet(int, int, double, double)); 8.x mangles it Eimdd.
#include <dlfcn.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

#include "mippp/solvers/soplex/all.hpp"

using namespace mippp;
using namespace mippp::operators;
using var_t = mippp::model_variable<int, double>;
using clk = std::chrono::steady_clock;
static double secs(clk::time_point a) {
    return std::chrono::duration<double>(clk::now() - a).count();
}

int main(int argc, char ** argv) {
    const int n = argc > 1 ? std::atoi(argv[1]) : 8000;
    const soplex_api & api = soplex_api::load();
    void * h = dlopen(api.library_path().c_str(), RTLD_NOW | RTLD_NOLOAD);
    auto colStatus = (int (*)(void *, int))dlsym(h, "SoPlex_basisColStatus");
    auto iters = (int (*)(void *))dlsym(h, "SoPlex_getNumIterations");
    auto ns_ctor = (void (*)(void *, int, int, double, double))dlsym(
        h, "_ZN6soplex7NameSetC1Eiidd");
    auto ns_add = (void (*)(void *, const char *))dlsym(
        h, "_ZN6soplex7NameSet3addEPKc");
    auto ns_dtor = (void (*)(void *))dlsym(h, "_ZN6soplex7NameSetD1Ev");
    auto read_bas = (bool (*)(void *, const char *, const void *,
                              const void *))dlsym(
        h, "_ZN6soplex10SoPlexBaseIdE13readBasisFileEPKcPKNS_7NameSetES6_");
    std::printf("symbols: ctor %p add %p dtor %p readBasisFile %p\n",
                (void *)ns_ctor, (void *)ns_add, (void *)ns_dtor,
                (void *)read_bas);
    if(!ns_ctor || !ns_add || !ns_dtor || !read_bas) return 0;

    auto build = [&](soplex_lp & m) {
        m.set_time_limit(std::chrono::duration<double>(20.0));
        std::vector<var_t> x;
        for(int j = 0; j < n; ++j)
            x.push_back(
                m.add_variable({.lower_bound = 0.0, .upper_bound = 10.0}));
        for(int i = 0; i < n; ++i)
            m.add_constraint(x[(std::size_t)i] +
                                 2 * x[(std::size_t)((i + 1) % n)] +
                                 x[(std::size_t)((i + 7) % n)] <=
                             15.0 + (i % 13));
        runtime_linear_expression<var_t, double> o;
        for(int j = 0; j < n; ++j)
            o += (1.0 + (j * 7 % 11)) * x[(std::size_t)j];
        m.set_maximization();
        m.set_objective(o);
    };
    soplex_lp a(api);
    build(a);
    auto t = clk::now();
    a.solve();
    std::printf("n=%d: cold solve %.3f s, %d iterations, obj %.6f\n", n,
                secs(t), iters(a.native_model()), a.get_solution_value());
    std::vector<int> cols(n), rows(n);
    for(int j = 0; j < n; ++j)
        cols[(std::size_t)j] = colStatus(a.native_model(), j);
    for(int i = 0; i < n; ++i)
        rows[(std::size_t)i] = api.basisRowStatus(a.native_model(), i);
    const std::string path =
        "/tmp/claude-1000/-home-plaiseek-Projects-mippp/"
        "66409ca5-709c-4c59-b3c1-e7b746a2b680/scratchpad/survey/soplex/"
        "cxx_abi.bas";
    t = clk::now();
    {
        std::ofstream f(path);
        f.setf(std::ios::left);
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
                  << std::setw(8) << ("c" + std::to_string(j)) << "       "
                  << ("r" + std::to_string(r)) << "\n";
            } else if(c == 0) {
                f << " UL c" << j << "\n";
            }
        }
        f << "ENDATA\n";
    }
    std::printf("  file written in %.3f s\n", secs(t));
    soplex_lp b(api);
    build(b);
    alignas(64) static unsigned char rbuf[1 << 14], cbuf[1 << 14];
    t = clk::now();
    ns_ctor(rbuf, n, -1, 2.0, 2.0);
    ns_ctor(cbuf, n, -1, 2.0, 2.0);
    for(int i = 0; i < n; ++i) ns_add(rbuf, ("r" + std::to_string(i)).c_str());
    for(int j = 0; j < n; ++j) ns_add(cbuf, ("c" + std::to_string(j)).c_str());
    const bool ok = read_bas(b.native_model(), path.c_str(), rbuf, cbuf);
    ns_dtor(rbuf);
    ns_dtor(cbuf);
    std::printf("  C++ readBasisFile with own NameSets = %d in %.3f s\n", ok,
                secs(t));
    int same = 0;
    for(int j = 0; j < n; ++j)
        same += colStatus(b.native_model(), j) == cols[(std::size_t)j];
    for(int i = 0; i < n; ++i)
        same += api.basisRowStatus(b.native_model(), i) == rows[(std::size_t)i];
    std::printf("  codes equal to A's: %d of %d\n", same, 2 * n);
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
