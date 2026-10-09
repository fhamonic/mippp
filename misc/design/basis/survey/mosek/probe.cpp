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

int main() {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    API = &mosek_api::load();
    std::printf("library %s version %s\n", API->library_path().c_str(),
                API->library_version() ? to_string(*API->library_version()).c_str()
                                       : "?");
    ext_api ext(API->library_path().c_str());
    EXT = &ext;
    {
        probe_lp m;
        int v = -1;
        API->getintparam(T(m), MSK_IPAR_OPTIMIZER, &v);
        std::printf("default MSK_IPAR_OPTIMIZER=%d (FREE=%d)\n", v,
                    (int)MSK_OPTIMIZER_FREE);
        API->getintparam(T(m), MSK_IPAR_INTPNT_BASIS, &v);
        std::printf("default MSK_IPAR_INTPNT_BASIS=%d (ALWAYS=%d)\n", v,
                    (int)MSK_BI_ALWAYS);
        API->getintparam(T(m), MSK_IPAR_SIM_HOTSTART, &v);
        std::printf("default MSK_IPAR_SIM_HOTSTART=%d (FREE=%d)\n", v,
                    (int)MSK_SIM_HOTSTART_FREE);
        API->getintparam(T(m), MSK_IPAR_PRESOLVE_USE, &v);
        std::printf("default MSK_IPAR_PRESOLVE_USE=%d (FREE=%d)\n", v,
                    (int)MSK_PRESOLVE_MODE_FREE);
        API->getintparam(T(m), MSK_IPAR_SIM_MAX_ITERATIONS, &v);
        std::printf("default MSK_IPAR_SIM_MAX_ITERATIONS=%d\n", v);
    }

    std::printf("\n===== C. side convention =====\n");
    {
        probe_lp m;
        auto c = build_c(m, true);
        dump(m, "C before any solve", MSK_SOL_BAS, false);
        m.solve();  // wrapper path: default optimizer
        std::printf("  wrapper status index=%zu (optimal is 1)\n",
                    m.get_status().index());
        raw_solve(m, "C re-solve default, for counters");
        dump(m, "C after default solve (BAS)", MSK_SOL_BAS, true, &c.rn, &c.cn);
        dump(m, "C after default solve (ITR keys)", MSK_SOL_ITR, true, &c.rn,
             &c.cn);
    }
    for(auto opt : {MSK_OPTIMIZER_PRIMAL_SIMPLEX, MSK_OPTIMIZER_DUAL_SIMPLEX}) {
        for(int pre : {MSK_PRESOLVE_MODE_OFF, MSK_PRESOLVE_MODE_FREE}) {
            probe_lp m;
            auto c = build_c(m, true);
            simplex_settings(m, opt, pre, -1);
            char label[128];
            std::snprintf(label, sizeof label, "C %s presolve=%d",
                          opt == MSK_OPTIMIZER_PRIMAL_SIMPLEX ? "primal" : "dual",
                          pre);
            raw_solve(m, label);
            dump(m, label, MSK_SOL_BAS, true, &c.rn, &c.cn);
        }
    }
    {
        probe_lp m;
        auto c = build_c(m, true);
        iparam(m, MSK_IPAR_INTPNT_BASIS, MSK_BI_NEVER);
        raw_solve(m, "C intpnt BI=NEVER");
        dump(m, "C intpnt BI=NEVER (BAS)", MSK_SOL_BAS, false);
        dump(m, "C intpnt BI=NEVER (ITR)", MSK_SOL_ITR, true, &c.rn, &c.cn);
    }

    std::printf("\n===== D. availability =====\n");
    {
        probe_lp m;
        build_r(m);
        dump(m, "R before solve", MSK_SOL_BAS, false);
        raw_solve(m, "R default");
        dump(m, "R after default solve", MSK_SOL_BAS, false);
        API->getprimalobj(T(m), MSK_SOL_BAS, &ref_obj);
        std::printf("  ref_obj=%.9g\n", ref_obj);
    }
    // infeasible
    for(int opt : {(int)MSK_OPTIMIZER_FREE, (int)MSK_OPTIMIZER_PRIMAL_SIMPLEX,
                   (int)MSK_OPTIMIZER_DUAL_SIMPLEX}) {
        for(int pre : {MSK_PRESOLVE_MODE_FREE, MSK_PRESOLVE_MODE_OFF}) {
            probe_lp m;
            m.set_time_limit(10s);
            auto x = m.add_variable({.obj_coef = 1, .lower_bound = 0});
            auto y = m.add_variable({.obj_coef = 1, .lower_bound = 0});
            m.add_constraint(x + y <= 1);
            m.add_constraint(x + 2 * y >= 3);
            m.add_constraint(x >= 2);
            m.set_maximization();
            iparam(m, MSK_IPAR_OPTIMIZER, opt);
            iparam(m, MSK_IPAR_PRESOLVE_USE, pre);
            char label[96];
            std::snprintf(label, sizeof label, "infeasible opt=%d presolve=%d",
                          opt, pre);
            raw_solve(m, label);
            dump(m, label, MSK_SOL_BAS, true);
        }
    }
    // unbounded
    for(int opt : {(int)MSK_OPTIMIZER_FREE, (int)MSK_OPTIMIZER_PRIMAL_SIMPLEX,
                   (int)MSK_OPTIMIZER_DUAL_SIMPLEX}) {
        for(int pre : {MSK_PRESOLVE_MODE_FREE, MSK_PRESOLVE_MODE_OFF}) {
            probe_lp m;
            m.set_time_limit(10s);
            auto x = m.add_variable({.obj_coef = 1, .lower_bound = 0});
            auto y = m.add_variable({.obj_coef = 1, .lower_bound = 0});
            m.add_constraint(x - y <= 1);
            m.add_constraint(x + y >= 1);
            m.set_maximization();
            iparam(m, MSK_IPAR_OPTIMIZER, opt);
            iparam(m, MSK_IPAR_PRESOLVE_USE, pre);
            char label[96];
            std::snprintf(label, sizeof label, "unbounded opt=%d presolve=%d",
                          opt, pre);
            raw_solve(m, label);
            dump(m, label, MSK_SOL_BAS, true);
        }
    }
    // iteration limits
    {
        probe_lp m;
        build_r(m);
        simplex_settings(m, MSK_OPTIMIZER_PRIMAL_SIMPLEX, MSK_PRESOLVE_MODE_OFF,
                         3);
        raw_solve(m, "R primal simplex SIM_MAX_ITERATIONS=3");
        dump(m, "R primal simplex iter-limit", MSK_SOL_BAS, false);
    }
    {
        probe_lp m;
        build_r(m);
        simplex_settings(m, MSK_OPTIMIZER_DUAL_SIMPLEX, MSK_PRESOLVE_MODE_OFF,
                         3);
        raw_solve(m, "R dual simplex SIM_MAX_ITERATIONS=3");
        dump(m, "R dual simplex iter-limit", MSK_SOL_BAS, false);
    }
    {
        probe_lp m;
        build_r(m);
        iparam(m, MSK_IPAR_INTPNT_MAX_ITERATIONS, 2);
        raw_solve(m, "R intpnt INTPNT_MAX_ITERATIONS=2 BI=ALWAYS");
        dump(m, "R intpnt iter-limit", MSK_SOL_BAS, false);
    }
    {
        probe_lp m;
        build_r(m);
        iparam(m, MSK_IPAR_INTPNT_MAX_ITERATIONS, 2);
        iparam(m, MSK_IPAR_INTPNT_BASIS, MSK_BI_NO_ERROR);
        iparam(m, MSK_IPAR_BI_IGNORE_MAX_ITER, MSK_ON);
        raw_solve(m, "R intpnt iter-limit BI=NO_ERROR IGNORE_MAX_ITER=ON");
        dump(m, "R intpnt iter-limit BI ignore", MSK_SOL_BAS, false);
    }
    // time limit on a bigger LP
    for(int opt : {(int)MSK_OPTIMIZER_FREE, (int)MSK_OPTIMIZER_PRIMAL_SIMPLEX,
                   (int)MSK_OPTIMIZER_DUAL_SIMPLEX}) {
        probe_lp m;
        build_r(m, 400, 600, 777);
        API->putdouparam(T(m), MSK_DPAR_OPTIMIZER_MAX_TIME, 1e-4);
        iparam(m, MSK_IPAR_OPTIMIZER, opt);
        iparam(m, MSK_IPAR_PRESOLVE_USE, MSK_PRESOLVE_MODE_OFF);
        char label[96];
        std::snprintf(label, sizeof label, "big R time limit 1e-4 opt=%d", opt);
        raw_solve(m, label);
        dump(m, label, MSK_SOL_BAS, false);
    }
    // modifications after an optimal solve
    {
        probe_lp m;
        auto r = build_r(m);
        raw_solve(m, "R default for mods");
        auto b0 = dump(m, "R solved", MSK_SOL_BAS, false);
        m.set_variable_upper_bound(r.x[0], 1.5);
        dump(m, "R after set_variable_upper_bound (no re-solve)", MSK_SOL_BAS,
             false);
    }
    {
        probe_lp m;
        auto r = build_r(m);
        raw_solve(m, "R default for mods 2");
        m.set_constraint_upper_bound(r.rows[0], 10);
        dump(m, "R after set_constraint_upper_bound (no re-solve)", MSK_SOL_BAS,
             false);
    }
    {
        probe_lp m;
        auto r = build_r(m);
        raw_solve(m, "R default for mods 3");
        m.add_constraint(r.x[2] + r.x[3] <= 1000);
        auto b = dump(m, "R after add_constraint (no re-solve)", MSK_SOL_BAS,
                      false);
        if(!b.skc.empty())
            std::printf("  new row code=%s\n", skname(b.skc.back()));
    }
    {
        probe_lp m;
        auto r = build_r(m);
        raw_solve(m, "R default for mods 4");
        m.add_variable({.obj_coef = -1, .lower_bound = 0});
        auto b = dump(m, "R after add_variable (no re-solve)", MSK_SOL_BAS,
                      false);
        if(!b.skx.empty())
            std::printf("  new col code=%s\n", skname(b.skx.back()));
    }
    {
        probe_lp m;
        auto r = build_r(m);
        raw_solve(m, "R default for mods 5");
        m.set_objective_coefficient(r.x[0], 100);
        dump(m, "R after set_objective_coefficient (no re-solve)", MSK_SOL_BAS,
             false);
    }
    {
        probe_lp m;
        auto r = build_r(m);
        raw_solve(m, "R default for mods 6");
        m.reset_status();
        dump(m, "R after reset_status()", MSK_SOL_BAS, false);
    }

    std::printf("\n===== E. warm start =====\n");
    basis_arrays bA;
    {
        probe_lp A;
        build_r(A);
        raw_solve(A, "A default");
        bA = dump(A, "A optimal", MSK_SOL_BAS, false);
        API->getprimalobj(T(A), MSK_SOL_BAS, &ref_obj);
    }
    // cold controls
    for(auto opt : {MSK_OPTIMIZER_PRIMAL_SIMPLEX, MSK_OPTIMIZER_DUAL_SIMPLEX,
                    MSK_OPTIMIZER_FREE_SIMPLEX}) {
        auto B = fresh_r();
        simplex_settings(B, opt, MSK_PRESOLVE_MODE_OFF, 0);
        char label[96];
        std::snprintf(label, sizeof label, "COLD opt=%d presolve=OFF maxit=0",
                      (int)opt);
        raw_solve(B, label);
        obj_line(B, label);
    }
    // warm, various settings
    struct variant {
        const char * name;
        int opt, presolve, maxit, hotstart;
    };
    std::vector<variant> variants = {
        {"wrapper defaults (FREE opt, presolve FREE), SIM_MAX_IT=0",
         MSK_OPTIMIZER_FREE, -1, 0, -1},
        {"wrapper defaults, no iteration limit", MSK_OPTIMIZER_FREE, -1, -1, -1},
        {"INTPNT explicit, maxit=0", MSK_OPTIMIZER_INTPNT, -1, 0, -1},
        {"PRIMAL presolve FREE maxit=0", MSK_OPTIMIZER_PRIMAL_SIMPLEX,
         MSK_PRESOLVE_MODE_FREE, 0, -1},
        {"PRIMAL presolve OFF maxit=0", MSK_OPTIMIZER_PRIMAL_SIMPLEX,
         MSK_PRESOLVE_MODE_OFF, 0, -1},
        {"DUAL presolve FREE maxit=0", MSK_OPTIMIZER_DUAL_SIMPLEX,
         MSK_PRESOLVE_MODE_FREE, 0, -1},
        {"DUAL presolve OFF maxit=0", MSK_OPTIMIZER_DUAL_SIMPLEX,
         MSK_PRESOLVE_MODE_OFF, 0, -1},
        {"FREE_SIMPLEX presolve FREE maxit=0", MSK_OPTIMIZER_FREE_SIMPLEX,
         MSK_PRESOLVE_MODE_FREE, 0, -1},
        {"FREE_SIMPLEX presolve OFF maxit=0", MSK_OPTIMIZER_FREE_SIMPLEX,
         MSK_PRESOLVE_MODE_OFF, 0, -1},
        {"PRIMAL presolve OFF maxit=0 HOTSTART=STATUS_KEYS",
         MSK_OPTIMIZER_PRIMAL_SIMPLEX, MSK_PRESOLVE_MODE_OFF, 0,
         MSK_SIM_HOTSTART_STATUS_KEYS},
        {"DUAL presolve OFF maxit=0 HOTSTART=STATUS_KEYS",
         MSK_OPTIMIZER_DUAL_SIMPLEX, MSK_PRESOLVE_MODE_OFF, 0,
         MSK_SIM_HOTSTART_STATUS_KEYS},
        {"FREE_SIMPLEX presolve FREE maxit=0 HOTSTART=STATUS_KEYS",
         MSK_OPTIMIZER_FREE_SIMPLEX, MSK_PRESOLVE_MODE_FREE, 0,
         MSK_SIM_HOTSTART_STATUS_KEYS},
        {"PRIMAL presolve OFF maxit=0 HOTSTART=NONE",
         MSK_OPTIMIZER_PRIMAL_SIMPLEX, MSK_PRESOLVE_MODE_OFF, 0,
         MSK_SIM_HOTSTART_NONE},
        {"PRIMAL presolve FREE no limit", MSK_OPTIMIZER_PRIMAL_SIMPLEX,
         MSK_PRESOLVE_MODE_FREE, -1, -1},
        {"DUAL presolve FREE no limit", MSK_OPTIMIZER_DUAL_SIMPLEX,
         MSK_PRESOLVE_MODE_FREE, -1, -1},
    };
    for(auto & v : variants) {
        auto B = fresh_r();
        put_basis(B, bA, v.name);
        auto back = dump(B, "B after put (readback, no solve)", MSK_SOL_BAS,
                         false);
        bool same = back.skc == bA.skc && back.skx == bA.skx;
        std::printf("  readback equals written: %d\n", same);
        simplex_settings(B, (MSKoptimizertypee)v.opt, v.presolve, v.maxit,
                         v.hotstart);
        raw_solve(B, v.name);
        obj_line(B, v.name);
    }
    // wrapper solve() after put with simplex + maxit 0: status the wrapper reports
    {
        auto B = fresh_r();
        put_basis(B, bA, "wrapper solve after put");
        simplex_settings(B, MSK_OPTIMIZER_DUAL_SIMPLEX, MSK_PRESOLVE_MODE_OFF, 0);
        B.solve();
        std::printf("  wrapper solve(): status index=%zu (1 optimal, 9 "
                    "iteration_limit)\n",
                    B.get_status().index());
        obj_line(B, "wrapper solve after put");
    }
    // A re-solve: is the basis of the last solve reused automatically?
    {
        probe_lp A;
        build_r(A);
        raw_solve(A, "A2 default first");
        simplex_settings(A, MSK_OPTIMIZER_PRIMAL_SIMPLEX, MSK_PRESOLVE_MODE_OFF,
                         0);
        raw_solve(A, "A2 re-solve same model, primal maxit=0");
        obj_line(A, "A2 re-solve");
    }
    // bound change then write basis (kept or dropped?)
    {
        probe_lp A;
        auto r = build_r(A);
        raw_solve(A, "A3 default first");
        auto b = dump(A, "A3 optimal", MSK_SOL_BAS, false);
        // relax a bound that is not active: basis stays optimal
        A.set_variable_upper_bound(r.x[5], 1000);
        dump(A, "A3 after relaxing x5 ub (no re-solve)", MSK_SOL_BAS, false);
        put_basis(A, b, "A3 put after bound change");
        auto back = dump(A, "A3 readback", MSK_SOL_BAS, false);
        std::printf("  readback equals written: %d\n",
                    back.skc == b.skc && back.skx == b.skx);
        simplex_settings(A, MSK_OPTIMIZER_PRIMAL_SIMPLEX, MSK_PRESOLVE_MODE_OFF,
                         0);
        raw_solve(A, "A3 primal maxit=0 after put");
        obj_line(A, "A3");
    }
    {
        probe_lp A;
        auto r = build_r(A);
        raw_solve(A, "A4 default first");
        auto b = dump(A, "A4 optimal", MSK_SOL_BAS, false);
        put_basis(A, b, "A4 put BEFORE bound change");
        A.set_variable_upper_bound(r.x[5], 1000);
        auto back = dump(A, "A4 after put then bound change", MSK_SOL_BAS,
                         false);
        std::printf("  readback equals written: %d\n",
                    back.skc == b.skc && back.skx == b.skx);
        simplex_settings(A, MSK_OPTIMIZER_PRIMAL_SIMPLEX, MSK_PRESOLVE_MODE_OFF,
                         0);
        raw_solve(A, "A4 primal maxit=0");
        obj_line(A, "A4");
    }
    // tighten an active bound: basis no longer primal feasible
    {
        probe_lp A;
        auto r = build_r(A);
        raw_solve(A, "A5 default first");
        auto b = dump(A, "A5 optimal", MSK_SOL_BAS, false);
        A.set_constraint_upper_bound(r.rows[0], 10);
        put_basis(A, b, "A5 put after tightening row 0");
        simplex_settings(A, MSK_OPTIMIZER_DUAL_SIMPLEX, MSK_PRESOLVE_MODE_OFF,
                         -1);
        raw_solve(A, "A5 dual simplex after put (expect few iters, hotstart)");
    }

    std::printf("\n===== E. invalid bases =====\n");
    auto find_code = [](const std::vector<MSKstakeye> & v, MSKstakeye k,
                        int skip = 0) {
        for(int i = 0; i < (int)v.size(); ++i)
            if(v[i] == k && skip-- == 0) return i;
        return -1;
    };
    struct inval {
        const char * name;
        std::function<void(basis_arrays &, rmodel &)> f;
    };
    std::vector<inval> invals = {
        {"one basic too few (a basic column -> LOW)",
         [&](basis_arrays & b, rmodel &) {
             int j = find_code(b.skx, MSK_SK_BAS);
             if(j < 0) { std::printf("    no candidate\n"); return; }
             std::printf("    column %d BAS -> LOW\n", j);
             b.skx[j] = MSK_SK_LOW;
         }},
        {"one basic too many (a nonbasic row -> BAS)",
         [&](basis_arrays & b, rmodel &) {
             int i = find_code(b.skc, MSK_SK_UPR);
             if(i < 0) i = find_code(b.skc, MSK_SK_LOW);
             if(i < 0) { std::printf("    no candidate\n"); return; }
             std::printf("    row %d %s -> BAS\n", i, skname(b.skc[i]));
             b.skc[i] = MSK_SK_BAS;
         }},
        {"UPR on a column with infinite upper bound",
         [&](basis_arrays & b, rmodel & r) {
             int j = -1;
             for(int k = 0; k < (int)b.skx.size(); ++k)
                 if(b.skx[k] == MSK_SK_LOW && k % 5 != 4 &&
                    k < (int)r.x.size()) {
                     j = k;
                     break;
                 }
             if(j < 0) { std::printf("    no candidate\n"); return; }
             std::printf("    column %d LOW -> UPR (ub = +inf)\n", j);
             b.skx[j] = MSK_SK_UPR;
         }},
        {"LOW on the free column (basic at optimum, swap with a LOW column)",
         [&](basis_arrays & b, rmodel & r) {
             int jf = (int)r.x.size();
             int j = find_code(b.skx, MSK_SK_LOW);
             if(j < 0) { std::printf("    no candidate\n"); return; }
             std::printf("    free column %d %s -> LOW, column %d %s -> BAS\n",
                         jf, skname(b.skx[jf]), j, skname(b.skx[j]));
             b.skx[jf] = MSK_SK_LOW;
             b.skx[j] = MSK_SK_BAS;
         }},
        {"LOW on the free column only (count off by one)",
         [&](basis_arrays & b, rmodel & r) {
             int jf = (int)r.x.size();
             std::printf("    free column %d %s -> LOW\n", jf,
                         skname(b.skx[jf]));
             b.skx[jf] = MSK_SK_LOW;
         }},
        {"SUPBAS on a nonbasic bounded column",
         [&](basis_arrays & b, rmodel &) {
             int j = find_code(b.skx, MSK_SK_LOW);
             if(j < 0) { std::printf("    no candidate\n"); return; }
             std::printf("    column %d LOW -> SUPBAS\n", j);
             b.skx[j] = MSK_SK_SUPBAS;
         }},
        {"FIX on a non-fixed nonbasic column",
         [&](basis_arrays & b, rmodel &) {
             int j = find_code(b.skx, MSK_SK_LOW);
             if(j < 0) { std::printf("    no candidate\n"); return; }
             std::printf("    column %d LOW -> FIX\n", j);
             b.skx[j] = MSK_SK_FIX;
         }},
        {"INF code on a nonbasic column",
         [&](basis_arrays & b, rmodel &) {
             int j = find_code(b.skx, MSK_SK_LOW);
             if(j < 0) { std::printf("    no candidate\n"); return; }
             std::printf("    column %d LOW -> INF\n", j);
             b.skx[j] = MSK_SK_INF;
         }},
        {"UNK everywhere",
         [&](basis_arrays & b, rmodel &) {
             for(auto & k : b.skx) k = MSK_SK_UNK;
             for(auto & k : b.skc) k = MSK_SK_UNK;
         }},
        {"all BAS (way too many)",
         [&](basis_arrays & b, rmodel &) {
             for(auto & k : b.skx) k = MSK_SK_BAS;
             for(auto & k : b.skc) k = MSK_SK_BAS;
         }},
        {"out-of-range code 7",
         [&](basis_arrays & b, rmodel &) { b.skx[0] = (MSKstakeye)7; }},
        {"singular basis (rows all BAS->LOW swapped with zero-coef columns)",
         [&](basis_arrays & b, rmodel & r) {
             // make the free column and a column absent from row set basic
             // while removing basic rows: likely singular. Use: all rows
             // nonbasic LOW/UPR by sense, first m columns basic.
             int m = (int)b.skc.size();
             for(auto & k : b.skc) k = MSK_SK_UPR;
             b.skc.back() = MSK_SK_FIX;
             for(auto & k : b.skx) k = MSK_SK_LOW;
             for(int j = 0; j < m && j < (int)b.skx.size(); ++j)
                 b.skx[j] = MSK_SK_BAS;
             b.skx[0] = MSK_SK_LOW;
             b.skx[r.x.size()] = MSK_SK_BAS;  // free column basic
         }},
    };
    for(auto & iv : invals) {
        for(int maxit : {0, -1}) {
            probe_lp B;
            auto r = build_r(B);
            basis_arrays b = bA;
            std::printf("* invalid: %s, maxit=%d\n", iv.name, maxit);
            iv.f(b, r);
            put_basis(B, b, iv.name);
            auto back = dump(B, "readback", MSK_SOL_BAS, false);
            int diffs = 0;
            for(size_t k = 0; k < b.skx.size() && k < back.skx.size(); ++k)
                diffs += b.skx[k] != back.skx[k];
            for(size_t k = 0; k < b.skc.size() && k < back.skc.size(); ++k)
                diffs += b.skc[k] != back.skc[k];
            std::printf("  readback differs from written in %d entries\n",
                        diffs);
            simplex_settings(B, MSK_OPTIMIZER_PRIMAL_SIMPLEX,
                             MSK_PRESOLVE_MODE_OFF, maxit);
            raw_solve(B, "primal");
            obj_line(B, iv.name);
        }
    }

    std::printf("\n===== F. partial / extended bases =====\n");
    {
        probe_lp A;
        auto r = build_r(A);
        raw_solve(A, "F1 A default");
        auto b = dump(A, "F1 A optimal", MSK_SOL_BAS, false);
        // grow: a column whose cost makes it unattractive, a non-binding row
        A.add_variable({.obj_coef = -100, .lower_bound = 0});
        A.add_constraint(r.x[2] + r.x[3] <= 100000);
        auto g = dump(A, "F1 A after growth (no re-solve)", MSK_SOL_BAS, false);
        if(!g.skx.empty())
            std::printf("  new col=%s new row=%s\n", skname(g.skx.back()),
                        skname(g.skc.back()));
        b.skx.push_back(MSK_SK_LOW);
        b.skc.push_back(MSK_SK_BAS);
        put_basis(A, b, "F1 put extended (new col LOW, new row BAS)");
        simplex_settings(A, MSK_OPTIMIZER_PRIMAL_SIMPLEX, MSK_PRESOLVE_MODE_OFF,
                         0);
        raw_solve(A, "F1 primal maxit=0");
        MSKsolstae s;
        API->getsolsta(T(A), MSK_SOL_BAS, &s);
        double o;
        API->getprimalobj(T(A), MSK_SOL_BAS, &o);
        std::printf("  F1 solsta=%s obj=%.9g (ref %.9g)\n", solstaname(s), o,
                    ref_obj);
    }
    {
        probe_lp A;
        auto r = build_r(A);
        raw_solve(A, "F2 A default");
        A.add_variable({.obj_coef = -100, .lower_bound = 0});
        A.add_constraint(r.x[2] + r.x[3] <= 100000);
        simplex_settings(A, MSK_OPTIMIZER_PRIMAL_SIMPLEX, MSK_PRESOLVE_MODE_OFF,
                         0);
        raw_solve(A, "F2 grown, no put, primal maxit=0");
        obj_line(A, "F2");
    }
    {
        probe_lp A;
        auto r = build_r(A);
        raw_solve(A, "F2b A default");
        A.add_variable({.obj_coef = -100, .lower_bound = 0});
        A.add_constraint(r.x[2] + r.x[3] <= 100000);
        simplex_settings(A, MSK_OPTIMIZER_PRIMAL_SIMPLEX, MSK_PRESOLVE_MODE_OFF,
                         -1);
        raw_solve(A, "F2b grown, no put, primal no limit");
        obj_line(A, "F2b");
    }
    {
        // subset: only the columns via putskx, rows untouched on a fresh model
        probe_lp B;
        build_r(B);
        int rc = EXT->putskx(T(B), MSK_SOL_BAS, bA.skx.data());
        std::printf("  F3 putskx only rc=%s\n", rcs(rc).c_str());
        dump(B, "F3 fresh model after putskx only", MSK_SOL_BAS, false);
        basis_arrays rb;
        rb.skc.assign(B.num_constraints(), (MSKstakeye)99);
        EXT->getskc(T(B), MSK_SOL_BAS, rb.skc.data());
        int unk = 0;
        for(auto k : rb.skc) unk += k == MSK_SK_UNK;
        std::printf("  F3 rows read back UNK: %d of %zu\n", unk, rb.skc.size());
        simplex_settings(B, MSK_OPTIMIZER_PRIMAL_SIMPLEX, MSK_PRESOLVE_MODE_OFF,
                         0);
        raw_solve(B, "F3 primal maxit=0");
        obj_line(B, "F3");
    }
    {
        // slices: half the columns via putskxslice + all rows
        probe_lp B;
        build_r(B);
        int half = (int)bA.skx.size() / 2;
        int rc1 = EXT->putskxslice(T(B), MSK_SOL_BAS, 0, half, bA.skx.data());
        int rc2 = EXT->putskc(T(B), MSK_SOL_BAS, bA.skc.data());
        std::printf("  F4 putskxslice[0,%d) rc=%s putskc rc=%s\n", half,
                    rcs(rc1).c_str(), rcs(rc2).c_str());
        auto back = dump(B, "F4 readback", MSK_SOL_BAS, false);
        int unk = 0;
        for(int j = half; j < (int)back.skx.size(); ++j)
            unk += back.skx[j] == MSK_SK_UNK;
        std::printf("  F4 unset columns read back UNK: %d of %zu\n", unk,
                    back.skx.size() - half);
        simplex_settings(B, MSK_OPTIMIZER_PRIMAL_SIMPLEX, MSK_PRESOLVE_MODE_OFF,
                         -1);
        raw_solve(B, "F4 primal no limit");
        obj_line(B, "F4");
    }
    {
        // slice write on an already solved model: replace one entry only
        probe_lp A;
        build_r(A);
        raw_solve(A, "F5 A default");
        MSKstakeye k = MSK_SK_BAS;
        int rc = EXT->putskcslice(T(A), MSK_SOL_BAS, 0, 1, &k);
        std::printf("  F5 putskcslice row0 := BAS rc=%s\n", rcs(rc).c_str());
        dump(A, "F5 after single-entry put", MSK_SOL_BAS, false);
    }
    std::printf("\nDONE\n");
}
