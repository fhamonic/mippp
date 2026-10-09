// LP basis survey probe for the CPLEX backend (cplex_lp).
// Builds models with MIP++ and reaches CPXgetbase / CPXcopybase / ... through
// dlsym on the very library file the api loaded.
#include <dlfcn.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
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
using var_t = model_variable_t<cplex_lp>;
using con_t = model_constraint_t<cplex_lp>;

constexpr int AT_LOWER = 0, BASIC = 1, AT_UPPER = 2, FREE_SUPER = 3;
constexpr int P_ADVIND = 1001, P_PREIND = 1030, P_LPMETHOD = 1062,
              P_BARCROSSALG = 3018, P_ITLIM = 1020;

struct Ext {
    int (*getbase)(CPXCENVptr, CPXCLPptr, int *, int *);
    int (*copybase)(CPXCENVptr, CPXLPptr, const int *, const int *);
    int (*copystart)(CPXCENVptr, CPXLPptr, const int *, const int *,
                     const double *, const double *, const double *,
                     const double *);
    int (*getitcnt)(CPXCENVptr, CPXCLPptr);
    int (*getphase1cnt)(CPXCENVptr, CPXCLPptr);
    int (*getmethod)(CPXCENVptr, CPXCLPptr);
    int (*baropt)(CPXCENVptr, CPXLPptr);
    int (*getslack)(CPXCENVptr, CPXCLPptr, double *, int, int);
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

const char * code_name(int c) {
    switch(c) {
        case AT_LOWER: return "AT_LOWER";
        case BASIC: return "BASIC";
        case AT_UPPER: return "AT_UPPER";
        case FREE_SUPER: return "FREE_SUPER";
        case -7: return "(untouched)";
        default: return "??";
    }
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

void info(const char * label, cplex_lp & m) {
    auto [env, lp] = m.native_model();
    const auto & api = m.native_api();
    int sm = -9, st = -9, pf = -9, df = -9;
    int si = api.solninfo(env, lp, &sm, &st, &pf, &df);
    std::printf(
        "[%s] getstat=%d solninfo(ret=%d method=%d type=%d pfeas=%d dfeas=%d) "
        "getmethod=%d itcnt=%d phase1=%d\n",
        label, api.getstat(env, lp), si, sm, st, pf, df, ext.getmethod(env, lp),
        ext.getitcnt(env, lp), ext.getphase1cnt(env, lp));
}

void dump(const char * label, cplex_lp & m,
          const std::vector<std::string> * cn = nullptr,
          const std::vector<std::string> * rn = nullptr, bool full = true) {
    auto [env, lp] = m.native_model();
    const auto & api = m.native_api();
    info(label, m);
    const int nc = api.getnumcols(env, lp), nr = api.getnumrows(env, lp);
    Basis b = get_basis(m);
    std::printf("  getbase ret=%s  (ncols=%d nrows=%d)\n",
                errstr(m, b.ret).c_str(), nc, nr);
    int nbasic = 0;
    for(int s : b.c) nbasic += s == BASIC;
    for(int s : b.r) nbasic += s == BASIC;
    std::printf("  #basic=%d (cols %d + rows %d)\n", nbasic,
                (int)std::count(b.c.begin(), b.c.end(), BASIC),
                (int)std::count(b.r.begin(), b.r.end(), BASIC));
    if(!full) return;
    std::vector<double> x(nc, NAN), lb(nc), ub(nc), sl(nr, NAN), rhs(nr),
        rng(nr, 0.);
    std::vector<char> sense(nr);
    int gx = nc ? api.getx(env, lp, x.data(), 0, nc - 1) : 0;
    if(nc) {
        api.getlb(env, lp, lb.data(), 0, nc - 1);
        api.getub(env, lp, ub.data(), 0, nc - 1);
    }
    int gs = 0;
    if(nr) {
        api.getsense(env, lp, sense.data(), 0, nr - 1);
        api.getrhs(env, lp, rhs.data(), 0, nr - 1);
        api.getrngval(env, lp, rng.data(), 0, nr - 1);
        gs = ext.getslack(env, lp, sl.data(), 0, nr - 1);
    }
    std::printf("  getx ret=%s getslack ret=%s\n", errstr(m, gx).c_str(),
                errstr(m, gs).c_str());
    for(int j = 0; j < nc; ++j)
        std::printf("   col %-3d %-4s lb=%-7g ub=%-7g x=%-9g cstat=%d %s\n", j,
                    cn && j < (int)cn->size() ? (*cn)[j].c_str() : "",
                    lb[j], ub[j], x[j], b.c[j], code_name(b.c[j]));
    for(int i = 0; i < nr; ++i) {
        // activity from slack: L/G/E: rhs - slack ; R: rhs + slack
        double act = sense[i] == 'R' ? rhs[i] + sl[i] : rhs[i] - sl[i];
        std::printf(
            "   row %-3d %-26s sense=%c rhs=%-6g rng=%-6g slack=%-9g act=%-9g "
            "rstat=%d %s\n",
            i, rn && i < (int)rn->size() ? (*rn)[i].c_str() : "", sense[i],
            rhs[i], rng[i], sl[i], act, b.r[i], code_name(b.r[i]));
    }
}

int set_int(cplex_lp & m, int p, int v) {
    auto [env, lp] = m.native_model();
    return m.native_api().setintparam(env, p, v);
}

///////////////////////////////////////////////////////////////////////////////
// Model C: the row/column side convention model
///////////////////////////////////////////////////////////////////////////////
const std::vector<std::string> C_cols = {
    "y1", "y2", "y3", "y4", "u[0,2]", "f1=3", "f2=2", "y5", "y6",
    "z free", "w(-inf,5]", "v>=0"};
const std::vector<std::string> C_rows = {
    "R1 L y1+y2<=4 tight",  "R2 G y1-y2>=2 tight",
    "R3 E y3+f1==7 (pi<0)", "R4 R 1<=y4+u<=6 @up",
    "R5 L <=100 loose",     "R6 R 1<=y5<=8 @low",
    "R7 E y6-f2==-1 (pi>0)", "R8 free row (L,1e20)"};

void build_C(cplex_lp & m) {
    m.set_time_limit(std::chrono::seconds(10));
    auto y1 = m.add_variable();
    auto y2 = m.add_variable();
    auto y3 = m.add_variable();
    auto y4 = m.add_variable();
    auto u = m.add_variable({.lower_bound = 0., .upper_bound = 2.});
    auto f1 = m.add_variable({.lower_bound = 3., .upper_bound = 3.});
    auto f2 = m.add_variable({.lower_bound = 2., .upper_bound = 2.});
    auto y5 = m.add_variable();
    auto y6 = m.add_variable();
    auto z = m.add_variable({.lower_bound = std::nullopt});
    auto w = m.add_variable({.lower_bound = std::nullopt, .upper_bound = 5.});
    auto v = m.add_variable();
    (void)z;
    m.set_maximization();
    m.set_objective(y2 - y3 + y4 + 2 * u + 5 * f1 - 5 * f2 - y5 + y6 + w - v);
    m.add_constraint(y1 + y2 <= 4);
    m.add_constraint(y1 - y2 >= 2);
    m.add_constraint(y3 + f1 == 7);
    auto r4 = m.add_constraint(y4 + u <= 6);
    m.set_constraint_lower_bound(r4, 1.);
    m.add_constraint(y1 + y3 + y4 <= 100);
    auto r6 = m.add_constraint(y5 <= 8);
    m.set_constraint_lower_bound(r6, 1.);
    m.add_constraint(y6 - f2 == -1);
    auto r8 = m.add_constraint(y1 - y2 <= 3);
    m.set_constraint_upper_bound(r8, m.infinity());
}

///////////////////////////////////////////////////////////////////////////////
// Model R: a random LP large enough for iteration limits
///////////////////////////////////////////////////////////////////////////////
void build_R(cplex_lp & m, std::size_t n = 80, int k = 60,
             unsigned seed = 7) {
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
        for(int & a : row) total += (a = dens(rng) < 4 ? coef(rng) : 0);
        if(i % 5 == 4)
            m.add_constraint(xsum(items, [&](auto j) { return row[j] * x(j); }) >=
                             total / 10);
        else
            m.add_constraint(xsum(items, [&](auto j) { return row[j] * x(j); }) <=
                             total / 3);
    }
}

void solve_report(const char * label, cplex_lp & m, bool full = false,
                  const std::vector<std::string> * cn = nullptr,
                  const std::vector<std::string> * rn = nullptr) {
    try {
        m.solve();
    } catch(const std::exception & e) {
        std::printf("[%s] solve threw: %s\n", label, e.what());
    }
    auto [env, lp] = m.native_model();
    double obj = NAN;
    int r = m.native_api().getobjval(env, lp, &obj);
    std::printf("[%s] wrapper status index=%zu objval=%.10g (ret %s)\n", label,
                m.get_status().index(), obj, errstr(m, r).c_str());
    dump(label, m, cn, rn, full);
}

int main(int argc, char ** argv) {
    const std::string only = argc > 1 ? argv[1] : "";
    auto want = [&](const char * s) { return only.empty() || only == s; };
    {
        cplex_lp probe_model;
        const auto & api = probe_model.native_api();
        std::printf("library: %s version: %s\n",
                    api.library_path().string().c_str(),
                    api.library_version()
                        ? to_string(*api.library_version()).c_str()
                        : "none");
        void * h = dlopen(api.library_path().c_str(), RTLD_NOW | RTLD_NOLOAD);
        if(!h) {
            std::printf("dlopen NOLOAD failed: %s\n", dlerror());
            return 1;
        }
        load_sym(ext.getbase, h, "CPXgetbase");
        load_sym(ext.copybase, h, "CPXcopybase");
        load_sym(ext.copystart, h, "CPXcopystart");
        load_sym(ext.getitcnt, h, "CPXgetitcnt");
        load_sym(ext.getphase1cnt, h, "CPXgetphase1cnt");
        load_sym(ext.getmethod, h, "CPXgetmethod");
        load_sym(ext.baropt, h, "CPXbaropt");
        load_sym(ext.getslack, h, "CPXgetslack");
        auto [env, lp] = probe_model.native_model();
        int v;
        api.getintparam(env, P_ADVIND, &v);
        std::printf("defaults: ADVIND=%d", v);
        api.getintparam(env, P_PREIND, &v);
        std::printf(" PREIND=%d", v);
        api.getintparam(env, P_LPMETHOD, &v);
        std::printf(" LPMETHOD=%d", v);
        api.getintparam(env, P_BARCROSSALG, &v);
        std::printf(" BARCROSSALG=%d", v);
        api.getintparam(env, P_ITLIM, &v);
        std::printf(" ITLIM=%d\n", v);
    }

    ///////////////////////////////////////////////////////////////////////////
    // C. side convention
    ///////////////////////////////////////////////////////////////////////////
    if(want("C")) {
        std::printf("\n===== C1: model C, wrapper defaults (primopt, presolve on)\n");
        cplex_lp m;
        build_C(m);
        {
            Basis b = get_basis(m);
            std::printf("[C1 before solve] getbase ret=%s\n",
                        errstr(m, b.ret).c_str());
            info("C1 before solve", m);
        }
        solve_report("C1", m, true, &C_cols, &C_rows);

        std::printf("\n===== C2: model C, presolve off\n");
        cplex_lp m2;
        build_C(m2);
        set_int(m2, P_PREIND, 0);
        solve_report("C2", m2, true, &C_cols, &C_rows);

        std::printf("\n===== C3: model C, ranged rows rewritten with a NEGATIVE "
                    "range value (native), presolve off\n");
        cplex_lp m3;
        build_C(m3);
        {
            auto [env, lp] = m3.native_model();
            const auto & api = m3.native_api();
            int idx[2] = {3, 5};
            double rhs[2] = {6., 8.}, rng[2] = {-5., -7.};
            api._check(env, api.chgrhs(env, lp, 2, idx, rhs));
            api._check(env, api.chgrngval(env, lp, 2, idx, rng));
        }
        set_int(m3, P_PREIND, 0);
        solve_report("C3", m3, true, &C_cols, &C_rows);

        std::printf("\n===== C4: model C, dual simplex via CPXdualopt "
                    "(native), presolve off\n");
        cplex_lp m4;
        build_C(m4);
        set_int(m4, P_PREIND, 0);
        {
            auto [env, lp] = m4.native_model();
            int r = m4.native_api().dualopt(env, lp);
            std::printf("dualopt ret=%s\n", errstr(m4, r).c_str());
        }
        dump("C4", m4, &C_cols, &C_rows);
    }

    ///////////////////////////////////////////////////////////////////////////
    // D. availability
    ///////////////////////////////////////////////////////////////////////////
    if(want("D")) {
        std::printf("\n===== D1: infeasible LP (presolve on / off)\n");
        for(int pre : {1, 0}) {
            cplex_lp m;
            m.set_time_limit(std::chrono::seconds(10));
            auto x = m.add_variable();
            auto y = m.add_variable();
            m.set_objective(x + y);
            m.add_constraint(x + y <= 4);
            m.add_constraint(x - y >= 6);
            m.add_constraint(y - x >= 1);
            set_int(m, P_PREIND, pre);
            solve_report(pre ? "D1 infeasible presolve on"
                             : "D1 infeasible presolve off",
                         m, true);
        }
        std::printf("\n===== D2: unbounded LP (presolve on / off)\n");
        for(int pre : {1, 0}) {
            cplex_lp m;
            m.set_time_limit(std::chrono::seconds(10));
            auto x = m.add_variable();
            auto y = m.add_variable();
            m.set_maximization();
            m.set_objective(x + y);
            m.add_constraint(x - y <= 1);
            m.add_constraint(x + 2 * y >= 1);
            set_int(m, P_PREIND, pre);
            solve_report(pre ? "D2 unbounded presolve on"
                             : "D2 unbounded presolve off",
                         m, true);
        }
        std::printf("\n===== D3: iteration limit stop (random LP, itlim 3)\n");
        for(int pre : {1, 0}) {
            cplex_lp m;
            build_R(m);
            set_int(m, P_PREIND, pre);
            m.set_iteration_limit(3);
            solve_report(pre ? "D3 itlim presolve on" : "D3 itlim presolve off",
                         m);
        }
        std::printf("\n===== D4: time limit stop (random LP, limit 0 s)\n");
        {
            cplex_lp m;
            build_R(m);
            m.set_time_limit(std::chrono::duration<double>(0.));
            solve_report("D4 timelim 0", m);
        }
        std::printf("\n===== D5: modifications after an optimal solve, no "
                    "re-solve (model C)\n");
        {
            cplex_lp m;
            build_C(m);
            m.solve();
            dump("D5 optimal", m, nullptr, nullptr, false);
            auto vars = m.variables();
            // change a bound of a basic column, which leaves the basis optimal
            m.set_variable_upper_bound(vars[0], 50.);
            dump("D5 after set_variable_upper_bound(y1,50)", m, nullptr,
                 nullptr, false);
            m.add_constraint(vars[0] + vars[1] <= 1000);
            dump("D5 after add_constraint", m, nullptr, nullptr, true);
            m.add_variable({.obj_coef = -1., .lower_bound = 0., .upper_bound = 4.});
            dump("D5 after add_variable", m, nullptr, nullptr, true);
            m.set_objective_coefficient(vars[1], 1.5);
            dump("D5 after set_objective_coefficient", m, nullptr, nullptr,
                 false);
            m.solve();
            dump("D5 re-solved", m, nullptr, nullptr, false);
            m.remove_variable(vars[11]);
            dump("D5 after remove_variable(v)", m, nullptr, nullptr, true);
            m.solve();
            dump("D5 re-solved after removal", m, nullptr, nullptr, false);
        }
        std::printf("\n===== D5b: bound change that makes the basis "
                    "infeasible, no re-solve\n");
        {
            cplex_lp m;
            build_C(m);
            m.solve();
            auto vars = m.variables();
            m.set_variable_upper_bound(vars[0], 1.);  // y1=3 basic -> infeasible
            dump("D5b after ub(y1)=1", m, nullptr, nullptr, true);
            m.solve();
            dump("D5b re-solved", m, nullptr, nullptr, false);
        }
        std::printf("\n===== D6: barrier (native CPXbaropt) with / without "
                    "crossover, random LP\n");
        for(int cross : {0, -1}) {
            cplex_lp m;
            build_R(m);
            set_int(m, P_BARCROSSALG, cross);
            auto [env, lp] = m.native_model();
            int r = ext.baropt(env, lp);
            std::printf("baropt (BARCROSSALG=%d) ret=%s\n", cross,
                        errstr(m, r).c_str());
            dump(cross == -1 ? "D6 barrier no crossover"
                             : "D6 barrier auto crossover",
                 m, nullptr, nullptr, false);
        }
        std::printf("\n===== D7: CPXlpopt with LPMETHOD auto (random LP and "
                    "model C)\n");
        {
            cplex_lp m;
            build_R(m);
            auto [env, lp] = m.native_model();
            int r = m.native_api().lpopt(env, lp);
            std::printf("lpopt ret=%s\n", errstr(m, r).c_str());
            dump("D7 lpopt auto random", m, nullptr, nullptr, false);
            cplex_lp c;
            build_C(c);
            auto [env2, lp2] = c.native_model();
            r = c.native_api().lpopt(env2, lp2);
            std::printf("lpopt ret=%s\n", errstr(c, r).c_str());
            dump("D7 lpopt auto C", c, nullptr, nullptr, false);
        }
        std::printf("\n===== D8: LP solved entirely by presolve\n");
        {
            cplex_lp m;
            m.set_time_limit(std::chrono::seconds(10));
            auto x = m.add_variable({.lower_bound = 0., .upper_bound = 5.});
            auto y = m.add_variable({.lower_bound = 0., .upper_bound = 3.});
            m.set_maximization();
            m.set_objective(x + 2 * y);
            m.add_constraint(x <= 4);  // singleton row
            m.add_constraint(x + y <= 100);
            solve_report("D8 presolve solved", m, true);
        }
        std::printf("\n===== D9: compute_iis / refine_lp_status side effects\n");
        {
            cplex_lp m;
            build_C(m);
            m.solve();
            dump("D9 optimal", m, nullptr, nullptr, false);
            auto iis = m.compute_iis();
            std::printf("compute_iis outcome index=%zu\n",
                        iis.get_outcome().index());
            dump("D9 after compute_iis on a feasible model", m, nullptr,
                 nullptr, false);
            m.solve();
            dump("D9 re-solved", m, nullptr, nullptr, false);
        }
        {
            cplex_lp m;
            m.set_time_limit(std::chrono::seconds(10));
            auto x = m.add_variable();
            auto y = m.add_variable();
            m.set_objective(x + y);
            m.add_constraint(x + y <= 4);
            m.add_constraint(x - y >= 6);
            m.add_constraint(y - x >= 1);
            set_int(m, P_PREIND, 0);
            m.solve();
            dump("D9b infeasible", m, nullptr, nullptr, false);
            auto iis = m.compute_iis();
            std::printf("compute_iis outcome index=%zu\n",
                        iis.get_outcome().index());
            dump("D9b after compute_iis", m, nullptr, nullptr, false);
        }
        {
            cplex_lp m;
            m.set_time_limit(std::chrono::seconds(10));
            auto x = m.add_variable();
            auto y = m.add_variable();
            m.set_objective(x + y);
            m.add_constraint(x + y <= 4);
            m.add_constraint(x - y >= 6);
            m.add_constraint(y - x >= 1);
            m.solve();
            dump("D9c infeasible presolve on", m, nullptr, nullptr, false);
            try {
                m.refine_lp_status();
            } catch(const std::exception & e) {
                std::printf("refine threw %s\n", e.what());
            }
            std::printf("status index after refine=%zu\n",
                        m.get_status().index());
            dump("D9c after refine_lp_status", m, nullptr, nullptr, false);
        }
    }

    ///////////////////////////////////////////////////////////////////////////
    // E. warm start
    ///////////////////////////////////////////////////////////////////////////
    if(want("E")) {
        std::printf("\n===== E1: random LP A -> basis -> fresh B, copybase, "
                    "itlim 0\n");
        cplex_lp a;
        build_R(a);
        a.solve();
        double objA;
        {
            auto [env, lp] = a.native_model();
            a.native_api().getobjval(env, lp, &objA);
        }
        dump("E1 A", a, nullptr, nullptr, false);
        Basis bA = get_basis(a);
        for(int adv : {1, 0, 2}) {
            for(int pre : {1, 0}) {
                for(int lim : {0, -1}) {
                    cplex_lp b;
                    build_R(b);
                    set_int(b, P_ADVIND, adv);
                    set_int(b, P_PREIND, pre);
                    if(lim >= 0) b.set_iteration_limit(lim);
                    auto [env, lp] = b.native_model();
                    int r = ext.copybase(env, lp, bA.c.data(), bA.r.data());
                    char label[200];
                    std::snprintf(label, sizeof label,
                                  "E1 B ADVIND=%d PREIND=%d itlim=%s copybase=%s",
                                  adv, pre, lim >= 0 ? "0" : "default",
                                  errstr(b, r).c_str());
                    solve_report(label, b);
                    Basis bB = get_basis(b);
                    std::printf("  B basis == A basis: %s ; objA=%.10g\n",
                                (bB.c == bA.c && bB.r == bA.r) ? "yes" : "no",
                                objA);
                }
            }
        }
        std::printf("\n===== E2: copybase then getbase without solving\n");
        {
            cplex_lp b;
            build_R(b);
            auto [env, lp] = b.native_model();
            int r = ext.copybase(env, lp, bA.c.data(), bA.r.data());
            std::printf("copybase ret=%s\n", errstr(b, r).c_str());
            Basis bB = get_basis(b);
            std::printf("getbase after copybase ret=%s equal=%s\n",
                        errstr(b, bB.ret).c_str(),
                        (bB.c == bA.c && bB.r == bA.r) ? "yes" : "no");
            info("E2", b);
        }
        std::printf("\n===== E3: model C basis into fresh C', itlim 0\n");
        {
            cplex_lp c;
            build_C(c);
            c.solve();
            Basis bc = get_basis(c);
            cplex_lp c2;
            build_C(c2);
            c2.set_iteration_limit(0);
            auto [env, lp] = c2.native_model();
            int r = ext.copybase(env, lp, bc.c.data(), bc.r.data());
            std::printf("copybase ret=%s\n", errstr(c2, r).c_str());
            solve_report("E3 C' itlim 0", c2, true, &C_cols, &C_rows);
        }
        std::printf("\n===== E4: bound change and copybase on A\n");
        {
            // (a) change bound after solve, then copybase old basis, itlim 0
            cplex_lp c;
            build_C(c);
            c.solve();
            Basis bc = get_basis(c);
            auto vars = c.variables();
            c.set_variable_upper_bound(vars[4], 1.5);  // u at upper: 2 -> 1.5
            auto [env, lp] = c.native_model();
            int r = ext.copybase(env, lp, bc.c.data(), bc.r.data());
            std::printf("E4a copybase after ub(u)=1.5: ret=%s\n",
                        errstr(c, r).c_str());
            c.set_iteration_limit(0);
            solve_report("E4a solve itlim 0", c, true, &C_cols, &C_rows);
            // (b) copybase, then change bound, then solve
            cplex_lp d;
            build_C(d);
            auto [env2, lp2] = d.native_model();
            r = ext.copybase(env2, lp2, bc.c.data(), bc.r.data());
            std::printf("E4b copybase on fresh C: ret=%s\n",
                        errstr(d, r).c_str());
            auto dv = d.variables();
            d.set_variable_upper_bound(dv[4], 1.5);
            Basis bd = get_basis(d);
            std::printf("E4b getbase after bound change: ret=%s equal=%s\n",
                        errstr(d, bd.ret).c_str(),
                        (bd.c == bc.c && bd.r == bc.r) ? "yes" : "no");
            d.set_iteration_limit(0);
            solve_report("E4b solve itlim 0", d, false);
            // (c) copybase, then add a row, then solve
            cplex_lp e;
            build_C(e);
            auto [env3, lp3] = e.native_model();
            r = ext.copybase(env3, lp3, bc.c.data(), bc.r.data());
            auto ev = e.variables();
            e.add_constraint(ev[0] + ev[3] <= 500);
            Basis be = get_basis(e);
            std::printf("E4c copybase then add_constraint: getbase ret=%s "
                        "nrows=%zu last rstat=%d\n",
                        errstr(e, be.ret).c_str(), be.r.size(),
                        be.r.empty() ? -1 : be.r.back());
            e.set_iteration_limit(0);
            solve_report("E4c solve itlim 0", e, false);
        }
        std::printf("\n===== E5: invalid bases written into fresh model C "
                    "(presolve default)\n");
        cplex_lp c;
        build_C(c);
        c.solve();
        Basis good = get_basis(c);
        struct Case {
            const char * name;
            std::function<void(Basis &)> edit;
            bool null_rstat = false;
        };
        std::vector<Case> cases = {
            {"too many basic: u(col4) BASIC", [](Basis & b) { b.c[4] = BASIC; }},
            {"too few basic: y1(col0) AT_LOWER",
             [](Basis & b) { b.c[0] = AT_LOWER; }},
            {"AT_UPPER on v (col11, ub=+inf)",
             [](Basis & b) { b.c[11] = AT_UPPER; }},
            {"AT_LOWER on free z (col9)", [](Basis & b) { b.c[9] = AT_LOWER; }},
            {"AT_UPPER on free z (col9)", [](Basis & b) { b.c[9] = AT_UPPER; }},
            {"FREE_SUPER on bounded u (col4)",
             [](Basis & b) { b.c[4] = FREE_SUPER; }},
            {"FREE_SUPER on v (col11, [0,inf))",
             [](Basis & b) { b.c[11] = FREE_SUPER; }},
            {"AT_LOWER on fixed f1 (col5, read AT_UPPER)", [](Basis & b) { b.c[5] = AT_LOWER; }},
            {"AT_UPPER on fixed f2 (col6, read AT_LOWER)", [](Basis & b) { b.c[6] = AT_UPPER; }},
            {"FREE_SUPER on fixed f2 (col6)", [](Basis & b) { b.c[6] = FREE_SUPER; }},
            {"AT_UPPER on R row R6 (wrong side, tight at lower)", [](Basis & b) { b.r[5] = AT_UPPER; }},
            {"AT_LOWER on w (col10, (-inf,5]) at upper",
             [](Basis & b) { b.c[10] = AT_LOWER; }},
            {"AT_UPPER on L row R1", [](Basis & b) { b.r[0] = AT_UPPER; }},
            {"AT_UPPER on G row R2", [](Basis & b) { b.r[1] = AT_UPPER; }},
            {"AT_UPPER on E row R3", [](Basis & b) { b.r[2] = AT_UPPER; }},
            {"AT_UPPER -> AT_LOWER on R row R4 (wrong side)",
             [](Basis & b) { b.r[3] = AT_LOWER; }},
            {"FREE_SUPER on L row R1", [](Basis & b) { b.r[0] = FREE_SUPER; }},
            {"AT_LOWER on free row R8 (swap with R1 basic)",
             [](Basis & b) {
                 b.r[7] = AT_LOWER;
                 b.r[0] = BASIC;
             }},
            {"code 7 on col0", [](Basis & b) { b.c[0] = 7; }},
            {"code -1 on row0", [](Basis & b) { b.r[0] = -1; }},
            {"rstat nullptr", [](Basis &) {}, true},
            {"singular: y1,y2 basic swapped for z,w basic",
             [](Basis & b) {
                 b.c[0] = AT_LOWER;
                 b.c[1] = AT_LOWER;
                 b.c[9] = BASIC;
                 b.c[10] = BASIC;
             }},
            {"all slack basis (cols nonbasic at lower/free, rows basic)",
             [](Basis & b) {
                 for(auto & s : b.c) s = AT_LOWER;
                 b.c[9] = FREE_SUPER;
                 b.c[10] = AT_UPPER;
                 for(auto & s : b.r) s = BASIC;
             }},
        };
        for(auto & cs : cases) {
            for(int lim : {0, -1}) {
                Basis b = good;
                cs.edit(b);
                cplex_lp t;
                build_C(t);
                if(lim >= 0) t.set_iteration_limit(lim);
                auto [env, lp] = t.native_model();
                int r = ext.copybase(env, lp, b.c.data(),
                                     cs.null_rstat ? nullptr : b.r.data());
                Basis back = get_basis(t);
                std::printf("\n-- E5 [%s] itlim=%s: copybase ret=%s ; getbase "
                            "before solve ret=%s equal-to-written=%s\n",
                            cs.name, lim >= 0 ? "0" : "default",
                            errstr(t, r).c_str(), errstr(t, back.ret).c_str(),
                            (back.c == b.c && back.r == b.r) ? "yes" : "no");
                solve_report(cs.name, t, false);
                Basis after = get_basis(t);
                std::printf("  after solve basis == good optimal basis: %s\n",
                            (after.c == good.c && after.r == good.r) ? "yes"
                                                                     : "no");
                if(lim >= 0) {
                    std::printf("  cstat:");
                    for(int s : after.c) std::printf(" %d", s);
                    std::printf("  rstat:");
                    for(int s : after.r) std::printf(" %d", s);
                    std::printf("\n");
                }
            }
        }
    }

    ///////////////////////////////////////////////////////////////////////////
    // F. extended bases
    ///////////////////////////////////////////////////////////////////////////
    if(want("F")) {
        std::printf("\n===== F1: basis of C extended with a new column "
                    "(AT_LOWER) and a new row (BASIC)\n");
        cplex_lp c;
        build_C(c);
        c.solve();
        Basis bc = get_basis(c);
        for(int newcol : {AT_LOWER, BASIC}) {
            for(int newrow : {BASIC, AT_LOWER}) {
                cplex_lp t;
                build_C(t);
                auto tv = t.variables();
                auto nv = t.add_variable({.obj_coef = -1., .lower_bound = 0., .upper_bound = 4.});
                t.add_constraint(tv[0] + tv[3] + nv <= 500);
                Basis b = bc;
                b.c.push_back(newcol);
                b.r.push_back(newrow);
                t.set_iteration_limit(0);
                auto [env, lp] = t.native_model();
                int r = ext.copybase(env, lp, b.c.data(), b.r.data());
                char label[200];
                std::snprintf(label, sizeof label,
                              "F1 new col=%s new row=%s copybase=%s itlim 0",
                              code_name(newcol), code_name(newrow),
                              errstr(t, r).c_str());
                solve_report(label, t, false);
            }
        }
        std::printf("\n===== F2: resident basis after add_constraint / "
                    "add_column on a solved model, then re-solve\n");
        {
            cplex_lp t;
            build_R(t);
            t.solve();
            dump("F2 solved", t, nullptr, nullptr, false);
            auto vars = t.variables();
            t.add_constraint(vars[0] + vars[1] + vars[2] <= 5);
            t.add_variable({.obj_coef = 3., .lower_bound = 0., .upper_bound = 1.});
            Basis b = get_basis(t);
            std::printf("F2 getbase after mods ret=%s sizes=%zu/%zu\n",
                        errstr(t, b.ret).c_str(), b.c.size(), b.r.size());
            t.solve();
            dump("F2 re-solved (warm from resident basis?)", t, nullptr,
                 nullptr, false);
            cplex_lp cold;
            build_R(cold);
            auto cv = cold.variables();
            cold.add_constraint(cv[0] + cv[1] + cv[2] <= 5);
            cold.add_variable({.obj_coef = 3., .lower_bound = 0., .upper_bound = 1.});
            cold.solve();
            dump("F2 same model solved cold", cold, nullptr, nullptr, false);
        }
    }
    return 0;
}
