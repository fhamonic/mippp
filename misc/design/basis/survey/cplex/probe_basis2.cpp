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
    std::printf("\n===== P1: iteration limit 1 vs 0 with an optimal basis; "
                "itlim 1 cold\n");
    {
        cplex_lp a;
        build_R(a);
        a.solve();
        Basis bA = get_basis(a);
        for(int lim : {0, 1, 2}) {
            cplex_lp b;
            build_R(b);
            b.set_iteration_limit(static_cast<std::size_t>(lim));
            auto [env, lp] = b.native_model();
            ext.copybase(env, lp, bA.c.data(), bA.r.data());
            b.solve();
            char l[64];
            std::snprintf(l, sizeof l, "warm itlim=%d", lim);
            std::printf("wrapper status index=%zu\n", b.get_status().index());
            print_basis(l, b);
        }
        for(int lim : {1}) {
            cplex_lp b;
            build_R(b);
            b.set_iteration_limit(static_cast<std::size_t>(lim));
            b.solve();
            std::printf("cold itlim=%d wrapper status index=%zu\n", lim,
                        b.get_status().index());
        }
    }
    std::printf("\n===== P2: CPXlpopt with LPMETHOD=barrier(4) / "
                "concurrent(6), SolutionType 0 / 2\n");
    for(int meth : {4, 6}) {
        for(int soltype : {0, 2}) {
            cplex_lp m;
            build_R(m);
            set_int(m, P_LPMETHOD, meth);
            set_int(m, P_SOLUTIONTYPE, soltype);
            auto [env, lp] = m.native_model();
            int r = m.native_api().lpopt(env, lp);
            char l[96];
            std::snprintf(l, sizeof l, "lpopt LPMETHOD=%d SOLUTIONTYPE=%d ret=%d",
                          meth, soltype, r);
            print_basis(l, m);
        }
    }
    std::printf("\n===== P3: refine_lp_status on INForUNBD (unbounded, "
                "presolve on)\n");
    {
        cplex_lp m;
        m.set_time_limit(std::chrono::seconds(10));
        auto x = m.add_variable();
        auto y = m.add_variable();
        m.set_maximization();
        m.set_objective(x + y);
        m.add_constraint(x - y <= 1);
        m.add_constraint(x + 2 * y >= 1);
        m.solve();
        print_basis("before refine", m);
        m.refine_lp_status();
        std::printf("status index after refine=%zu\n", m.get_status().index());
        print_basis("after refine", m);
    }
    std::printf("\n===== P4: codes of new columns / rows in the resident "
                "basis (solved model, no re-solve)\n");
    {
        cplex_lp m;
        m.set_time_limit(std::chrono::seconds(10));
        auto x = m.add_variable({.lower_bound = 0., .upper_bound = 4.});
        auto y = m.add_variable({.lower_bound = 0., .upper_bound = 4.});
        m.set_maximization();
        m.set_objective(x + y);
        m.add_constraint(x + y <= 5);
        m.solve();
        print_basis("solved", m);
        m.add_variable({.lower_bound = 0., .upper_bound = 3.});
        m.add_variable({.lower_bound = std::nullopt, .upper_bound = 3.});
        m.add_variable({.lower_bound = std::nullopt});
        m.add_variable({.lower_bound = 1., .upper_bound = 1.});
        m.add_variable({.lower_bound = -2., .upper_bound = 3.});
        auto r = m.add_constraint(x - y >= -100);
        auto e = m.add_constraint(x + 2 * y == 7);
        auto g = m.add_constraint(x <= 3);
        m.set_constraint_lower_bound(g, 1.);
        (void)r;
        (void)e;
        print_basis("after adding cols [0,3] (-inf,3] free [1,1] [-2,3] and "
                    "rows G, E, R",
                    m);
    }
    std::printf("\n===== P5: stale codes after bound/sense change (no "
                "re-solve)\n");
    {
        cplex_lp m;
        m.set_time_limit(std::chrono::seconds(10));
        auto x = m.add_variable({.lower_bound = 0., .upper_bound = 2.});
        auto y = m.add_variable();
        m.set_maximization();
        m.set_objective(2 * x + y);
        auto c1 = m.add_constraint(x + y <= 3);
        m.solve();
        print_basis("solved (x at upper 2, c1 tight)", m);
        m.set_variable_upper_bound(x, m.infinity());
        print_basis("after ub(x)=inf", m);
        m.set_constraint_sense(c1, constraint_sense::greater_equal);
        print_basis("after sense(c1) L->G", m);
        m.solve();
        std::printf("status index after re-solve=%zu\n", m.get_status().index());
        print_basis("re-solved", m);
    }
    std::printf("\n===== P6: empty arrays (model without rows / without "
                "columns)\n");
    {
        cplex_lp m;
        m.set_time_limit(std::chrono::seconds(10));
        auto x = m.add_variable({.lower_bound = 0., .upper_bound = 2.});
        m.set_maximization();
        m.set_objective(x);
        m.solve();
        auto [env, lp] = m.native_model();
        int cs[1] = {-7}, dummy[1] = {-7};
        int r1 = ext.getbase(env, lp, cs, nullptr);
        std::printf("no rows: getbase(cstat,NULL)=%s cstat=%d\n",
                    errstr(m, r1).c_str(), cs[0]);
        int r2 = ext.getbase(env, lp, cs, dummy);
        std::printf("no rows: getbase(cstat,dummy)=%s\n", errstr(m, r2).c_str());
        int w[1] = {AT_LOWER};
        int r3 = ext.copybase(env, lp, w, nullptr);
        std::printf("no rows: copybase(cstat,NULL)=%s\n", errstr(m, r3).c_str());
        int r4 = ext.copybase(env, lp, w, dummy);
        std::printf("no rows: copybase(cstat,dummy)=%s\n", errstr(m, r4).c_str());
        m.solve();
        print_basis("no rows re-solved after copybase", m);

        cplex_lp e;
        e.set_time_limit(std::chrono::seconds(10));
        e.solve();
        std::printf("empty model status index=%zu\n", e.get_status().index());
        print_basis("empty model", e);
        auto [env2, lp2] = e.native_model();
        int r5 = ext.copybase(env2, lp2, nullptr, nullptr);
        std::printf("empty model copybase(NULL,NULL)=%s\n",
                    errstr(e, r5).c_str());
        int r6 = ext.copybase(env2, lp2, dummy, dummy);
        std::printf("empty model copybase(dummy,dummy)=%s\n",
                    errstr(e, r6).c_str());
    }
    std::printf("\n===== P7: problem type switch LP -> MILP -> LP (native)\n");
    {
        cplex_lp m;
        build_R(m);
        m.solve();
        auto [env, lp] = m.native_model();
        const auto & api = m.native_api();
        print_basis("LP solved", m);
        int r = api.chgprobtype(env, lp, cx::CPXPROB_MILP);
        std::printf("chgprobtype MILP=%s\n", errstr(m, r).c_str());
        Basis b = get_basis(m);
        std::printf("getbase on MILP-typed problem=%s\n",
                    errstr(m, b.ret).c_str());
        r = api.chgprobtype(env, lp, cx::CPXPROB_LP);
        std::printf("chgprobtype LP=%s\n", errstr(m, r).c_str());
        b = get_basis(m);
        std::printf("getbase back on LP=%s\n", errstr(m, b.ret).c_str());
        m.solve();
        print_basis("re-solved", m);
    }
    std::printf("\n===== P8: real time-limit stop mid-simplex\n");
    for(double t : {0.002, 0.02}) {
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
    std::printf("\n===== P9: FREE_SUPER on a column with bounds [1,5] (itlim "
                "0 and default)\n");
    for(int lim : {0, -1}) {
        cplex_lp m;
        m.set_time_limit(std::chrono::seconds(10));
        auto x = m.add_variable({.lower_bound = 1., .upper_bound = 5.});
        auto y = m.add_variable();
        m.set_maximization();
        m.set_objective(x + y);
        m.add_constraint(x + y <= 8);
        if(lim >= 0) m.set_iteration_limit(0);
        auto [env, lp] = m.native_model();
        int cs[2] = {FREE_SUPER, BASIC}, rs[1] = {AT_LOWER};
        ext.copybase(env, lp, cs, rs);
        m.solve();
        auto sol = m.get_solution();
        std::printf("itlim=%s x=%g y=%g\n", lim >= 0 ? "0" : "default", sol[x],
                    sol[y]);
        print_basis("P9", m);
    }
    std::printf("\n===== P10: copystart with basis only, itlim 1\n");
    {
        cplex_lp a;
        build_R(a);
        a.solve();
        Basis bA = get_basis(a);
        cplex_lp b;
        build_R(b);
        b.set_iteration_limit(1);
        auto [env, lp] = b.native_model();
        int r = ext.copystart(env, lp, bA.c.data(), bA.r.data(), nullptr,
                              nullptr, nullptr, nullptr);
        std::printf("copystart ret=%s\n", errstr(b, r).c_str());
        b.solve();
        print_basis("copystart warm itlim 1", b);
    }
    return 0;
}
