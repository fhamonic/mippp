// Follow-up probe: iteration limit 1 vs 0, barrier through CPXlpopt,
// refine_lp_status, codes of new columns in the resident basis, stale codes
// after bound / sense changes, empty-array edge cases, problem type switch,
// a real time-limit stop, FREE_SUPER on a column whose bounds exclude 0.
#include <dlfcn.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>

#include "mippp/solvers/cplex/all.hpp"

using namespace mippp;
using namespace mippp::operators;
namespace cx = mippp::cplex::impl::v1;
using cx::CPXCENVptr;
using cx::CPXCLPptr;
using cx::CPXENVptr;
using cx::CPXLPptr;

constexpr int AT_LOWER = 0, BASIC = 1, AT_UPPER = 2, FREE_SUPER = 3;
constexpr int P_PREIND = 1030, P_LPMETHOD = 1062, P_SOLUTIONTYPE = 1147,
              P_BARCROSSALG = 3018, P_THREADS = 1067;

struct Ext {
    int (*getbase)(CPXCENVptr, CPXCLPptr, int *, int *);
    int (*copybase)(CPXCENVptr, CPXLPptr, const int *, const int *);
    int (*copystart)(CPXCENVptr, CPXLPptr, const int *, const int *,
                     const double *, const double *, const double *,
                     const double *);
    int (*getitcnt)(CPXCENVptr, CPXCLPptr);
    int (*getmethod)(CPXCENVptr, CPXCLPptr);
} ext;

template <class F>
void load_sym(F & f, void * h, const char * n) {
    f = reinterpret_cast<F>(dlsym(h, n));
    if(!f) {
        std::printf("missing symbol %s\n", n);
        std::exit(1);
    }
}
std::string errstr(cplex_lp & m, int code) {
    if(code == 0) return "ok";
    auto [env, lp] = m.native_model();
    char buf[1024];
    const char * s = m.native_api().geterrorstring(env, code, buf);
    std::string r = s ? std::string(buf) : std::string("?");
    while(!r.empty() && (r.back() == '\n' || r.back() == ' ')) r.pop_back();
    return std::to_string(code) + " " + r;
}
struct Basis {
    int ret = -1;
    std::vector<int> c, r;
};
Basis get_basis(cplex_lp & m) {
    auto [env, lp] = m.native_model();
    const auto & api = m.native_api();
    Basis b;
    b.c.assign(static_cast<std::size_t>(api.getnumcols(env, lp)), -7);
    b.r.assign(static_cast<std::size_t>(api.getnumrows(env, lp)), -7);
    b.ret = ext.getbase(env, lp, b.c.data(), b.r.data());
    return b;
}
void print_basis(const char * label, cplex_lp & m) {
    Basis b = get_basis(m);
    auto [env, lp] = m.native_model();
    const auto & api = m.native_api();
    int sm, st, pf, df;
    api.solninfo(env, lp, &sm, &st, &pf, &df);
    double obj = NAN;
    int ro = api.getobjval(env, lp, &obj);
    std::printf("[%s] getstat=%d method=%d soltype=%d pfeas=%d dfeas=%d itcnt=%d "
                "obj=%.10g(%s) getbase=%s\n  cstat:",
                label, api.getstat(env, lp), sm, st, pf, df,
                ext.getitcnt(env, lp), obj, errstr(m, ro).c_str(),
                errstr(m, b.ret).c_str());
    for(int s : b.c) std::printf(" %d", s);
    std::printf("\n  rstat:");
    for(int s : b.r) std::printf(" %d", s);
    int nb = 0;
    for(int s : b.c) nb += s == BASIC;
    for(int s : b.r) nb += s == BASIC;
    std::printf("\n  #basic=%d nrows=%zu\n", nb, b.r.size());
}
int set_int(cplex_lp & m, int p, int v) {
    auto [env, lp] = m.native_model();
    return m.native_api().setintparam(env, p, v);
}
void build_R(cplex_lp & m, std::size_t n = 80, int k = 60, unsigned seed = 7,
             int density = 4) {
    m.set_time_limit(std::chrono::seconds(10));
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> coef(1, 20), dens(0, 9), obj(1, 30);
    auto items = std::views::iota(std::size_t{0}, n);
    auto x = m.add_variables(n, {.lower_bound = 0., .upper_bound = 10.});
    std::vector<int> row(n);
    for(int & c : row) c = obj(rng);
    m.set_maximization();
    m.set_objective(xsum(items, [&](auto j) { return row[j] * x(j); }));
    for(int i = 0; i < k; ++i) {
        int total = 0;
        for(int & a : row) total += (a = dens(rng) < density ? coef(rng) : 0);
        if(i % 5 == 4)
            m.add_constraint(xsum(items, [&](auto j) { return row[j] * x(j); }) >=
                             total / 10);
        else
            m.add_constraint(xsum(items, [&](auto j) { return row[j] * x(j); }) <=
                             total / 3);
    }
}

int main() {
    {
        cplex_lp pm;
        const auto & api = pm.native_api();
        std::printf("library: %s version: %s\n",
                    api.library_path().string().c_str(),
                    to_string(*api.library_version()).c_str());
        void * h = dlopen(api.library_path().c_str(), RTLD_NOW | RTLD_NOLOAD);
        load_sym(ext.getbase, h, "CPXgetbase");
        load_sym(ext.copybase, h, "CPXcopybase");
        load_sym(ext.copystart, h, "CPXcopystart");
        load_sym(ext.getitcnt, h, "CPXgetitcnt");
        load_sym(ext.getmethod, h, "CPXgetmethod");
    }
    std::printf("\n===== P8: real time-limit stop mid-simplex\n");
    for(double t : {0.1, 0.2, 0.3}) {
        cplex_lp m;
        build_R(m, 900, 900, 11, 5);
        set_int(m, P_THREADS, 1);
        m.set_time_limit(std::chrono::duration<double>(t));
        auto t0 = std::chrono::steady_clock::now();
        m.solve();
        double el = std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - t0)
                        .count();
        std::printf("time limit %g: wrapper status index=%zu elapsed=%.3f\n", t,
                    m.get_status().index(), el);
        Basis b = get_basis(m);
        auto [env, lp] = m.native_model();
        int sm, st, pf, df;
        m.native_api().solninfo(env, lp, &sm, &st, &pf, &df);
        int nb = 0;
        for(int s : b.c) nb += s == BASIC;
        for(int s : b.r) nb += s == BASIC;
        std::printf("  getstat=%d itcnt=%d soltype=%d pfeas=%d dfeas=%d "
                    "getbase=%s #basic=%d nrows=%zu\n",
                    m.native_api().getstat(env, lp), ext.getitcnt(env, lp), st,
                    pf, df, errstr(m, b.ret).c_str(), nb, b.r.size());
    }
    {
        cplex_lp m;
        build_R(m, 900, 900, 11, 5);
        set_int(m, P_THREADS, 1);
        auto t0 = std::chrono::steady_clock::now();
        m.solve();
        double el = std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - t0)
                        .count();
        auto [env, lp] = m.native_model();
        std::printf("no time limit: status index=%zu elapsed=%.3f itcnt=%d\n",
                    m.get_status().index(), el, ext.getitcnt(env, lp));
    }
    return 0;
}
