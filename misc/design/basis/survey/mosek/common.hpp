#pragma once
// MOSEK basis survey probe: builds models with MIP++ (mosek_lp) and reaches
// the native status-key functions through native_api()/native_model() plus
// dlsym on api.library_path() for the symbols the api struct lacks.
#include <dlfcn.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include "mippp/solvers/mosek/all.hpp"

using namespace mippp;
using namespace mippp::operators;
using namespace std::chrono_literals;

struct probe_lp : mosek_lp {
    using mosek_base::_pick_continuous_solution;
};
using var = model_variable_t<mosek_lp>;
using con = model_constraint_t<mosek_lp>;

// symbols the api struct does not declare
struct ext_api {
    decltype(&MSK_getskc) getskc;
    decltype(&MSK_getskx) getskx;
    decltype(&MSK_putskc) putskc;
    decltype(&MSK_putskx) putskx;
    decltype(&MSK_putskcslice) putskcslice;
    decltype(&MSK_putskxslice) putskxslice;
    decltype(&MSK_getintinf) getintinf;
    decltype(&MSK_getdouinf) getdouinf;
    decltype(&MSK_getskcslice) getskcslice;
    explicit ext_api(const char * path) {
        void * h = dlopen(path, RTLD_NOW | RTLD_NOLOAD);
        if(!h) h = dlopen(path, RTLD_NOW);
        if(!h) {
            std::printf("dlopen failed: %s\n", dlerror());
            std::exit(2);
        }
#define SYM(n) n = reinterpret_cast<decltype(n)>(dlsym(h, "MSK_" #n));      \
    std::printf("dlsym MSK_%s -> %s\n", #n, n ? "found" : "MISSING");
        SYM(getskc) SYM(getskx) SYM(putskc) SYM(putskx) SYM(putskcslice)
            SYM(putskxslice) SYM(getintinf) SYM(getdouinf) SYM(getskcslice)
#undef SYM
    }
};

const mosek_api * API = nullptr;
ext_api * EXT = nullptr;

const char * skname(int k) {
    static const char * n[] = {"UNK", "BAS", "SUPBAS", "LOW",
                               "UPR", "FIX", "INF"};
    return (k >= 0 && k <= 6) ? n[k] : "???";
}
const char * bkname(int k) {
    static const char * n[] = {"LO", "UP", "FX", "FR", "RA"};
    return (k >= 0 && k <= 4) ? n[k] : "??";
}
std::string rcs(int rc) {
    if(rc == 0) return "0(OK)";
    char sym[MSK_MAX_STR_LEN] = {0}, str[MSK_MAX_STR_LEN] = {0};
    API->getcodedesc(static_cast<MSKrescodee>(rc), sym, str);
    return std::to_string(rc) + "(" + sym + ")";
}
const char * solstaname(int s) {
    static const char * n[] = {"UNKNOWN",        "OPTIMAL",
                               "PRIM_FEAS",      "DUAL_FEAS",
                               "PRIM_AND_DUAL_FEAS", "PRIM_INFEAS_CER",
                               "DUAL_INFEAS_CER", "PRIM_ILLPOSED_CER",
                               "DUAL_ILLPOSED_CER", "INTEGER_OPTIMAL"};
    return (s >= 0 && s <= 9) ? n[s] : "?";
}

MSKtask_t T(probe_lp & m) { return m.native_model().second; }

void iparam(probe_lp & m, MSKiparame p, int v) {
    int rc = API->putintparam(T(m), p, v);
    if(rc) std::printf("  putintparam(%d,%d) -> %s\n", (int)p, v, rcs(rc).c_str());
}

struct iters {
    int prim = -1, dual = -1, ipm = -1, phot = -1, dhot = -1, photlu = -1,
        dhotlu = -1;
};
iters get_iters(probe_lp & m) {
    iters it;
    EXT->getintinf(T(m), MSK_IINF_SIM_PRIMAL_ITER, &it.prim);
    EXT->getintinf(T(m), MSK_IINF_SIM_DUAL_ITER, &it.dual);
    EXT->getintinf(T(m), MSK_IINF_INTPNT_ITER, &it.ipm);
    EXT->getintinf(T(m), MSK_IINF_SIM_PRIMAL_HOTSTART, &it.phot);
    EXT->getintinf(T(m), MSK_IINF_SIM_DUAL_HOTSTART, &it.dhot);
    EXT->getintinf(T(m), MSK_IINF_SIM_PRIMAL_HOTSTART_LU, &it.photlu);
    EXT->getintinf(T(m), MSK_IINF_SIM_DUAL_HOTSTART_LU, &it.dhotlu);
    return it;
}

struct solve_out {
    int rc;
    int trm;
};
solve_out raw_solve(probe_lp & m, const char * label) {
    MSKrescodee trm = MSK_RES_OK;
    int rc = API->optimizetrm(T(m), &trm);
    auto it = get_iters(m);
    std::printf(
        "  [%s] optimizetrm rc=%s trm=%s | iters primal=%d dual=%d ipm=%d | "
        "hotstart primal=%d dual=%d | hotstartLU primal=%d dual=%d\n",
        label, rcs(rc).c_str(), rcs(trm).c_str(), it.prim, it.dual, it.ipm,
        it.phot, it.dhot, it.photlu, it.dhotlu);
    return {rc, trm};
}

void slot_line(probe_lp & m, MSKsoltypee slot, const char * name) {
    MSKbooleant def = 0;
    int rc = API->solutiondef(T(m), slot, &def);
    std::printf("  slot %s: solutiondef rc=%s defined=%d", name, rcs(rc).c_str(),
                def);
    if(def) {
        MSKsolstae s;
        MSKprostae p;
        API->getsolsta(T(m), slot, &s);
        API->getprosta(T(m), slot, &p);
        double obj = 0;
        API->getprimalobj(T(m), slot, &obj);
        std::printf(" solsta=%s prosta=%d pobj=%.9g", solstaname(s), (int)p,
                    obj);
    }
    std::printf("\n");
}

struct basis_arrays {
    std::vector<MSKstakeye> skc, skx;
};

// reads and prints: per row bound key, bounds, activity, code; per column
basis_arrays dump(probe_lp & m, const char * label,
                  MSKsoltypee slot = MSK_SOL_BAS, bool print_entries = true,
                  const std::vector<std::string> * rnames = nullptr,
                  const std::vector<std::string> * cnames = nullptr) {
    std::printf("--- %s ---\n", label);
    const int nc = (int)m.num_constraints(), nv = (int)m.num_variables();
    std::printf("  numcon=%d numvar=%d\n", nc, nv);
    slot_line(m, MSK_SOL_BAS, "BAS");
    slot_line(m, MSK_SOL_ITR, "ITR");
    auto pick = m._pick_continuous_solution();
    std::printf("  wrapper _pick_continuous_solution -> %s\n",
                !pick                 ? "nullopt"
                : *pick == MSK_SOL_BAS ? "BAS"
                                       : "ITR");
    basis_arrays b;
    b.skc.assign(nc, (MSKstakeye)99);
    b.skx.assign(nv, (MSKstakeye)99);
    int rc1 = EXT->getskc(T(m), slot, b.skc.data());
    int rc2 = EXT->getskx(T(m), slot, b.skx.data());
    std::printf("  getskc(%s) rc=%s getskx rc=%s\n",
                slot == MSK_SOL_BAS ? "BAS" : "ITR", rcs(rc1).c_str(),
                rcs(rc2).c_str());
    if(rc1 || rc2) return b;
    std::vector<double> xc(nc, 0), xx(nv, 0);
    int rc3 = API->getsolution(T(m), slot, nullptr, nullptr, nullptr, nullptr,
                               nullptr, xc.data(), xx.data(), nullptr, nullptr,
                               nullptr, nullptr, nullptr, nullptr);
    if(rc3) std::printf("  getsolution rc=%s\n", rcs(rc3).c_str());
    int nbas = 0, nsup = 0;
    for(auto k : b.skc) nbas += (k == MSK_SK_BAS), nsup += (k == MSK_SK_SUPBAS);
    for(auto k : b.skx) nbas += (k == MSK_SK_BAS), nsup += (k == MSK_SK_SUPBAS);
    std::printf("  #BAS=%d (numcon=%d) #SUPBAS=%d\n", nbas, nc, nsup);
    if(!print_entries) return b;
    for(int i = 0; i < nc; ++i) {
        MSKboundkeye bk;
        double bl, bu;
        API->getconbound(T(m), i, &bk, &bl, &bu);
        std::printf("  row %2d %-14s bk=%s [%g, %g] act=%.6g  -> %s\n", i,
                    rnames && i < (int)rnames->size() ? (*rnames)[i].c_str()
                                                       : "",
                    bkname(bk), bl, bu, xc[i], skname(b.skc[i]));
    }
    for(int j = 0; j < nv; ++j) {
        MSKboundkeye bk;
        double bl, bu;
        API->getvarbound(T(m), j, &bk, &bl, &bu);
        std::printf("  col %2d %-14s bk=%s [%g, %g] val=%.6g -> %s\n", j,
                    cnames && j < (int)cnames->size() ? (*cnames)[j].c_str()
                                                       : "",
                    bkname(bk), bl, bu, xx[j], skname(b.skx[j]));
    }
    return b;
}

void put_basis(probe_lp & m, const basis_arrays & b, const char * label) {
    int rcx = EXT->putskx(T(m), MSK_SOL_BAS, b.skx.data());
    int rcc = EXT->putskc(T(m), MSK_SOL_BAS, b.skc.data());
    MSKbooleant def = 0;
    API->solutiondef(T(m), MSK_SOL_BAS, &def);
    std::printf("  [%s] putskx rc=%s putskc rc=%s -> BAS defined=%d", label,
                rcs(rcx).c_str(), rcs(rcc).c_str(), def);
    if(def) {
        MSKsolstae s;
        API->getsolsta(T(m), MSK_SOL_BAS, &s);
        std::printf(" solsta=%s", solstaname(s));
    }
    std::printf("\n");
}

///////////////////////////////////////////////////////////////////////////////
// Model C: side convention
///////////////////////////////////////////////////////////////////////////////
struct cmodel {
    std::vector<std::string> rn, cn;
};
cmodel build_c(probe_lp & m, bool with_free) {
    cmodel c;
    m.set_time_limit(10s);
    m.set_maximization();
    auto y1 = m.add_variable({.obj_coef = 1, .lower_bound = 0});
    auto y2 = m.add_variable({.obj_coef = -1, .lower_bound = 0});
    auto y3 = m.add_variable({.obj_coef = 1, .lower_bound = 0});
    auto y4 = m.add_variable({.obj_coef = 1, .lower_bound = 0});
    auto y5 = m.add_variable({.obj_coef = -1, .lower_bound = 0});
    auto y6 = m.add_variable({.obj_coef = -1, .lower_bound = 0});
    auto y7 = m.add_variable({.obj_coef = 1, .lower_bound = 0});
    auto x1 = m.add_variable({.obj_coef = 1, .lower_bound = 0, .upper_bound = 4});
    auto x2 = m.add_variable({.obj_coef = 1, .lower_bound = 2, .upper_bound = 2});
    auto x4 = m.add_variable({.obj_coef = -1, .lower_bound = 0});
    auto x5 = m.add_variable(
        {.obj_coef = 1, .lower_bound = std::nullopt, .upper_bound = 7});
    c.cn = {"y1", "y2", "y3", "y4", "y5", "y6", "y7",
            "x1[0,4]ub", "x2fixed2", "x4lo", "x5(-inf,7]"};
    if(with_free) {
        m.add_variable({.obj_coef = 0, .lower_bound = std::nullopt});
        c.cn.push_back("x3free_empty");
    }
    m.add_constraint(y1 <= 3);  // tight <=
    m.add_constraint(y2 >= 2);  // tight >=
    m.add_constraint(y3 == 5);  // ==
    auto r4 = m.add_constraint(y4 <= 6);
    m.set_constraint_lower_bound(r4, 1);  // ranged [1,6] tight at upper
    auto r5 = m.add_constraint(y5 <= 6);
    m.set_constraint_lower_bound(r5, 1);  // ranged [1,6] tight at lower
    m.add_constraint(-1.0 * y6 <= -4);    // tight <= with negative coef
    m.add_constraint(-1.0 * y7 >= -8);    // tight >= with negative coef
    m.add_constraint(y1 + y2 + x1 + x2 + x4 <= 100);  // slack, basic
    m.add_constraint(x5 + y3 >= -50);                 // slack >= row
    c.rn = {"y1<=3",      "y2>=2",       "y3==5",       "1<=y4<=6 up",
            "1<=y5<=6 lo", "-y6<=-4",     "-y7>=-8",     "slack<=100",
            "slack>=-50"};
    return c;
}

///////////////////////////////////////////////////////////////////////////////
// Model R: random dense LP needing simplex work
///////////////////////////////////////////////////////////////////////////////
struct rmodel {
    std::vector<var> x;
    var xf;
    std::vector<con> rows;
};
rmodel build_r(probe_lp & m, int nrows = 20, int ncols = 30,
               unsigned seed = 12345) {
    rmodel r{.x = {}, .xf = var(0), .rows = {}};
    m.set_time_limit(10s);
    m.set_maximization();
    std::mt19937 gen(seed);
    std::uniform_int_distribution<int> coef(1, 9), coin(0, 1), rhs(50, 100);
    for(int j = 0; j < ncols; ++j) {
        double c = coef(gen);
        if(j % 5 == 4)
            r.x.push_back(m.add_variable(
                {.obj_coef = c, .lower_bound = 0, .upper_bound = 3}));
        else
            r.x.push_back(m.add_variable({.obj_coef = c, .lower_bound = 0}));
    }
    r.xf = m.add_variable({.obj_coef = 0.5, .lower_bound = std::nullopt});
    for(int i = 0; i < nrows; ++i) {
        std::vector<std::pair<var, double>> terms;
        for(int j = 0; j < ncols; ++j)
            if(coin(gen)) terms.emplace_back(r.x[j], (double)coef(gen));
        auto e = linear_expression_view(std::move(terms));
        if(i % 7 == 3)
            r.rows.push_back(m.add_constraint(std::move(e) >= (double)(5 + i % 5)));
        else
            r.rows.push_back(m.add_constraint(std::move(e) <= (double)rhs(gen)));
    }
    r.rows.push_back(m.add_constraint(r.xf - r.x[0] - r.x[1] == -3));
    return r;
}

double ref_obj = 0;

probe_lp fresh_r(int nrows = 20, int ncols = 30) {
    probe_lp m;
    build_r(m, nrows, ncols);
    return m;
}

void obj_line(probe_lp & m, const char * label) {
    MSKbooleant def = 0;
    API->solutiondef(T(m), MSK_SOL_BAS, &def);
    double o = 0;
    MSKsolstae s = MSK_SOL_STA_UNKNOWN;
    if(def) {
        API->getprimalobj(T(m), MSK_SOL_BAS, &o);
        API->getsolsta(T(m), MSK_SOL_BAS, &s);
    }
    std::printf("  [%s] BAS def=%d solsta=%s pobj=%.9g (ref %.9g) %s\n", label,
                def, solstaname(s), o, ref_obj,
                def && std::abs(o - ref_obj) < 1e-6 ? "MATCH" : "differs");
}

void simplex_settings(probe_lp & m, MSKoptimizertypee opt, int presolve,
                      int maxit, int hotstart = -1) {
    iparam(m, MSK_IPAR_OPTIMIZER, opt);
    if(presolve >= 0) iparam(m, MSK_IPAR_PRESOLVE_USE, presolve);
    if(maxit >= 0) iparam(m, MSK_IPAR_SIM_MAX_ITERATIONS, maxit);
    if(hotstart >= 0) iparam(m, MSK_IPAR_SIM_HOTSTART, hotstart);
}

