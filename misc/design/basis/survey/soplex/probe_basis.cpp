// SoPlex basis survey probe: builds models with MIP++ and reaches the native
// basis functions through native_api() / native_model() and dlsym.
#include <dlfcn.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "mippp/solvers/soplex/all.hpp"

using namespace mippp;
using namespace mippp::operators;

using var_t = mippp::model_variable<int, double>;
using con_t = mippp::model_constraint<int>;

namespace {

using colstat_t = int (*)(void *, int);
using readbas_t = int (*)(void *, const char *);
using int_of_t = int (*)(void *);
using vec_t = void (*)(void *, double *, int);

struct natives_t {
    colstat_t colStatus = nullptr;
    readbas_t readBasisFile = nullptr;
    int_of_t getNumIterations = nullptr;
    int_of_t getStatus = nullptr;
    vec_t getRedCost = nullptr;
} N;

constexpr int ITERLIMIT = 5;       // IntParam
constexpr int REPRESENTATION = 1;  // IntParam: 0 auto, 1 column, 2 row
constexpr int SIMPLIFIER = 10;     // IntParam: 0 off, 1 auto
constexpr int ALGORITHM = 2;       // IntParam: 0 primal, 1 dual

const char * code_name(int c) {
    switch(c) {
        case 0: return "ON_UPPER";
        case 1: return "ON_LOWER";
        case 2: return "FIXED";
        case 3: return "ZERO";
        case 4: return "BASIC";
        case 5: return "UNDEFINED";
        default: return "?";
    }
}
const char * status_name(int s) {
    switch(s) {
        case -15: return "ERROR";
        case -11: return "NOT_INIT";
        case -8: return "ABORT_CYCLING";
        case -7: return "ABORT_TIME";
        case -6: return "ABORT_ITER";
        case -5: return "ABORT_VALUE";
        case -4: return "SINGULAR";
        case -3: return "NO_PROBLEM";
        case -2: return "REGULAR";
        case -1: return "RUNNING";
        case 0: return "UNKNOWN";
        case 1: return "OPTIMAL";
        case 2: return "UNBOUNDED";
        case 3: return "INFEASIBLE";
        case 4: return "INForUNBD";
        case 5: return "OPTIMAL_UNSCALED_VIOLATIONS";
        default: return "?";
    }
}
const char * wrapper_status_name(std::size_t i) {
    static const char * names[] = {"unknown",   "optimal",
                                   "optimal_infeasible_unscaled",
                                   "infeasible_or_unbounded",
                                   "infeasible", "unbounded",
                                   "time_limit", "failed"};
    return i < 8 ? names[i] : "?";
}

struct codes_t {
    std::vector<int> cols, rows;
};

codes_t read_codes(soplex_lp & m) {
    const auto & api = m.native_api();
    void * p = m.native_model();
    codes_t c;
    const int n = api.numCols(p), r = api.numRows(p);
    for(int j = 0; j < n; ++j) c.cols.push_back(N.colStatus(p, j));
    for(int i = 0; i < r; ++i) c.rows.push_back(api.basisRowStatus(p, i));
    return c;
}

std::string side_of(double v, double lb, double ub) {
    const double tol = 1e-7;
    bool at_lb = lb > -1e99 && std::fabs(v - lb) <= tol;
    bool at_ub = ub < 1e99 && std::fabs(v - ub) <= tol;
    if(at_lb && at_ub) return "at both";
    if(at_lb) return "at lower";
    if(at_ub) return "at upper";
    return "strictly inside";
}

void dump(soplex_lp & m, const std::string & label, bool with_solution,
          const std::vector<std::string> & cn = {},
          const std::vector<std::string> & rn = {}) {
    const auto & api = m.native_api();
    void * p = m.native_model();
    const int n = api.numCols(p), r = api.numRows(p);
    std::printf("---- %s\n", label.c_str());
    std::printf("  SoPlex_getStatus = %d (%s), iterations = %d\n",
                N.getStatus(p), status_name(N.getStatus(p)),
                N.getNumIterations(p));
    std::vector<double> x(static_cast<std::size_t>(n), 0.0);
    if(with_solution && n > 0) api.getPrimalReal(p, x.data(), n);
    int nbasic = 0;
    for(int j = 0; j < n; ++j) {
        const auto v = var_t(j);
        const double lb = m.get_variable_lower_bound(v);
        const double ub = m.get_variable_upper_bound(v);
        const int code = N.colStatus(p, j);
        nbasic += code == 4;
        std::printf("  col %-6s code %d %-9s bounds [%g, %g]",
                    j < (int)cn.size() ? cn[(std::size_t)j].c_str()
                                       : ("x" + std::to_string(j)).c_str(),
                    code, code_name(code), lb, ub);
        if(with_solution)
            std::printf(" value %g (%s)", x[(std::size_t)j],
                        side_of(x[(std::size_t)j], lb, ub).c_str());
        std::printf("\n");
    }
    std::vector<long> idx(static_cast<std::size_t>(std::max(n, 1)));
    std::vector<double> coef(static_cast<std::size_t>(std::max(n, 1)));
    for(int i = 0; i < r; ++i) {
        double lhs, rhs;
        api.getRowBoundsReal(p, i, &lhs, &rhs);
        int nnz = 0;
        api.getRowVectorReal(p, i, &nnz, idx.data(), coef.data());
        double act = 0;
        for(int k = 0; k < nnz; ++k)
            act += coef[(std::size_t)k] * x[(std::size_t)idx[(std::size_t)k]];
        const int code = api.basisRowStatus(p, i);
        nbasic += code == 4;
        std::printf("  row %-7s code %d %-9s sides [%g, %g]",
                    i < (int)rn.size() ? rn[(std::size_t)i].c_str()
                                       : ("C" + std::to_string(i)).c_str(),
                    code, code_name(code), lhs, rhs);
        if(with_solution)
            std::printf(" activity %g (%s)", act,
                        side_of(act, lhs, rhs).c_str());
        std::printf("\n");
    }
    std::printf("  #basic = %d, #rows = %d -> %s\n", nbasic, r,
                nbasic == r ? "equal" : "DIFFERENT");
}

// SPxBasisBase::readBasis names entity j with a stringstream it never
// resets: column j is "x0x1...xj", row i is "C0C1...Ci".
std::string cname(int j) {
    std::string s;
    for(int k = 0; k <= j; ++k) s += "x" + std::to_string(k);
    return s;
}
std::string rname(int i) {
    std::string s;
    for(int k = 0; k <= i; ++k) s += "C" + std::to_string(k);
    return s;
}
// The layout of SoPlex's own SPxBasisBase::writeBasis: the reader has a
// fixed-format MPS heuristic that patches blanks inside short lines.
std::string xline(const char * ind, int col, int row) {
    std::ostringstream os;
    os.setf(std::ios::left);
    os << " " << ind << " " << std::setw(8) << cname(col) << "       "
       << rname(row) << "\n";
    return os.str();
}
std::string bas_from_codes(const codes_t & c, bool * ok = nullptr) {
    std::ostringstream os;
    os << "NAME  soplex.bas\n";
    std::vector<int> basic_cols, nonbasic_rows;
    for(std::size_t j = 0; j < c.cols.size(); ++j)
        if(c.cols[j] == 4) basic_cols.push_back((int)j);
    for(std::size_t i = 0; i < c.rows.size(); ++i)
        if(c.rows[i] != 4) nonbasic_rows.push_back((int)i);
    if(ok) *ok = basic_cols.size() == nonbasic_rows.size();
    std::size_t k = 0;
    for(std::size_t j = 0; j < c.cols.size(); ++j) {
        const int code = c.cols[j];
        if(code == 4) {
            if(k >= nonbasic_rows.size()) continue;
            const int row = nonbasic_rows[k++];
            os << xline(c.rows[(std::size_t)row] == 0 ? "XU" : "XL", (int)j,
                        row);
        } else if(code == 0) {
            os << " UL " << cname((int)j) << "\n";
        } else if(code == 1) {
            os << " LL " << cname((int)j) << "\n";
        }  // FIXED and ZERO: the reader's defaults
    }
    os << "ENDATA\n";
    return os.str();
}
std::string body(const std::string & bas) {
    return bas.substr(0, bas.find("ENDATA"));
}

const std::string scratch =
    "/tmp/claude-1000/-home-plaiseek-Projects-mippp/"
    "66409ca5-709c-4c59-b3c1-e7b746a2b680/scratchpad/survey/soplex/";

int write_basis_file(soplex_lp & m, const std::string & text,
                     const std::string & name) {
    const std::string path = scratch + name;
    {
        std::ofstream f(path);
        f << text;
    }
    std::printf("  .bas file %s:\n", name.c_str());
    std::istringstream is(text);
    for(std::string line; std::getline(is, line);)
        std::printf("    |%s\n", line.c_str());
    const int ret = N.readBasisFile(m.native_model(), path.c_str());
    std::printf("  SoPlex_readBasisFile returned %d\n", ret);
    return ret;
}

int raw_optimize(soplex_lp & m) {
    const int s = m.native_api().optimize(m.native_model());
    std::printf("  raw SoPlex_optimize = %d (%s), iterations = %d, obj = %.9g\n",
                s, status_name(s), N.getNumIterations(m.native_model()),
                m.native_api().objValueReal(m.native_model()));
    return s;
}
void wrapper_solve(soplex_lp & m) {
    m.solve();
    std::printf("  wrapper solve(): status %s, SoPlex_getStatus %d (%s), "
                "iterations %d, obj %.9g\n",
                wrapper_status_name(m.get_status().index()),
                N.getStatus(m.native_model()),
                status_name(N.getStatus(m.native_model())),
                N.getNumIterations(m.native_model()),
                m.get_solution_value());
}
void set_int(soplex_lp & m, int param, int value) {
    m.native_api().setIntParam(m.native_model(), param, value);
}

const std::vector<std::string> main_cn = {"y1",  "y2",   "y3",   "y4",
                                          "xup", "xfix", "xlow", "xfree"};
const std::vector<std::string> main_rn = {"r_le", "r_ge", "r_eq", "r_rng",
                                          "r_slk"};

// optimum: y = (5,3,3,3), xup = 4 (at upper), xfix = 2, xlow = 1 (at lower),
// xfree = 0 (free, empty column); r_le at upper, r_ge at lower, r_eq,
// r_rng = [1,7] at upper, r_slk basic. Objective -21 (min).
struct main_lp {
    var_t y1, y2, y3, y4, xup, xfix, xlow, xfree;
    con_t r_le, r_ge, r_eq, r_rng, r_slk;
};
main_lp build_main(soplex_lp & m, bool maximize = false,
                   bool with_free = true) {
    m.set_time_limit(std::chrono::duration<double>(10.0));
    main_lp l{m.add_variable({.lower_bound = 0.0, .upper_bound = 10.0}),
              m.add_variable({.lower_bound = 0.0, .upper_bound = 10.0}),
              m.add_variable({.lower_bound = 0.0, .upper_bound = 10.0}),
              m.add_variable({.lower_bound = 0.0, .upper_bound = 10.0}),
              m.add_variable({.lower_bound = 0.0, .upper_bound = 4.0}),
              m.add_variable({.lower_bound = 2.0, .upper_bound = 2.0}),
              m.add_variable({.lower_bound = 1.0, .upper_bound = 5.0}),
              with_free ? m.add_variable({.lower_bound = std::nullopt,
                                          .upper_bound = std::nullopt})
                        : var_t(-1),
              con_t(0), con_t(0), con_t(0), con_t(0),
              con_t(0)};
    l.r_le = m.add_constraint(l.y1 + l.y2 <= 8.0);
    l.r_ge = m.add_constraint(l.y1 - l.y2 >= 2.0);
    l.r_eq = m.add_constraint(l.y3 + l.xfix == 5.0);
    l.r_rng = m.add_constraint(l.y4 + l.xup <= 7.0);
    m.set_constraint_lower_bound(l.r_rng, 1.0);
    l.r_slk = m.add_constraint(l.y1 + l.y2 + l.y3 + l.y4 + l.xlow <= 100.0);
    auto obj = -l.y1 - 2 * l.y2 + l.y3 - l.y4 - 3 * l.xup + 2 * l.xlow;
    if(maximize) {
        m.set_maximization();
        m.set_objective(-1 * obj);
    } else {
        m.set_minimization();
        m.set_objective(obj);
    }
    return l;
}

// a dense LP that needs a few dozen iterations
void build_dense(soplex_lp & m, int n, int r) {
    m.set_time_limit(std::chrono::duration<double>(10.0));
    std::vector<var_t> x;
    for(int j = 0; j < n; ++j)
        x.push_back(m.add_variable({.lower_bound = 0.0, .upper_bound = 10.0}));
    unsigned s = 12345u;
    auto rnd = [&] {
        s = s * 1103515245u + 12345u;
        return (double)((s >> 16) % 1000u) / 100.0;
    };
    for(int i = 0; i < r; ++i) {
        runtime_linear_expression<var_t, double> e;
        for(int j = 0; j < n; ++j) e += rnd() * x[(std::size_t)j];
        m.add_constraint(e <= 50.0 + rnd() * 10.0);
    }
    runtime_linear_expression<var_t, double> o;
    for(int j = 0; j < n; ++j) o += (1.0 + rnd()) * x[(std::size_t)j];
    m.set_maximization();
    m.set_objective(o);
}

}  // namespace

int main() {
    const soplex_api & api = soplex_api::load();
    std::printf("library: %s\n", api.library_path().c_str());
    void * h = dlopen(api.library_path().c_str(), RTLD_NOW | RTLD_NOLOAD);
    if(!h) h = dlopen(api.library_path().c_str(), RTLD_NOW);
    N.colStatus = (colstat_t)dlsym(h, "SoPlex_basisColStatus");
    N.readBasisFile = (readbas_t)dlsym(h, "SoPlex_readBasisFile");
    N.getNumIterations = (int_of_t)dlsym(h, "SoPlex_getNumIterations");
    N.getStatus = (int_of_t)dlsym(h, "SoPlex_getStatus");
    N.getRedCost = (vec_t)dlsym(h, "SoPlex_getRedCostReal");
    std::printf(
        "api.basisRowStatus=%p dlsym: basisColStatus=%p readBasisFile=%p "
        "getNumIterations=%p getStatus=%p getRedCostReal=%p "
        "getRowBoundsReal(api)=%p getRowVectorReal(api)=%p\n",
        (void *)api.basisRowStatus, (void *)N.colStatus,
        (void *)N.readBasisFile, (void *)N.getNumIterations,
        (void *)N.getStatus, (void *)N.getRedCost,
        (void *)api.getRowBoundsReal, (void *)api.getRowVectorReal);
    for(const char * sym :
        {"SoPlex_setBasis", "SoPlex_getBasis", "SoPlex_writeBasisFile",
         "SoPlex_clearBasis", "SoPlex_hasBasis", "SoPlex_getBasisInd",
         "_ZN6soplex10SoPlexBaseIdE8setBasisEPKNS_13SPxSolverBaseIdE9VarStatusES6_"})
        std::printf("  dlsym(%s) = %p\n", sym, dlsym(h, sym));
    if(!api.basisRowStatus || !N.colStatus || !N.readBasisFile ||
       !N.getNumIterations || !N.getStatus) {
        std::printf("basis functions unavailable in this release: stop\n");
        return 0;
    }

    std::printf("\n==================== C: row side convention\n");
    {
        soplex_lp m(api);
        build_main(m);
        std::printf("default IntParams: SIMPLIFIER=%d REPRESENTATION=%d "
                    "ALGORITHM=%d ITERLIMIT=%d\n",
                    api.getIntParam(m.native_model(), SIMPLIFIER),
                    api.getIntParam(m.native_model(), REPRESENTATION),
                    api.getIntParam(m.native_model(), ALGORITHM),
                    api.getIntParam(m.native_model(), ITERLIMIT));
        wrapper_solve(m);
        dump(m, "C1 main LP, minimize, wrapper defaults", true, main_cn,
             main_rn);
        std::vector<double> rc(8), du(5);
        N.getRedCost(m.native_model(), rc.data(), 8);
        api.getDualReal(m.native_model(), du.data(), 5);
        std::printf("  redcost:");
        for(double v : rc) std::printf(" %g", v);
        std::printf("\n  duals:");
        for(double v : du) std::printf(" %g", v);
        std::printf("\n");
    }
    {
        soplex_lp m(api);
        build_main(m, true);
        wrapper_solve(m);
        dump(m, "C2 main LP, maximize (negated objective)", true, main_cn,
             main_rn);
    }
    {
        soplex_lp m(api);
        build_main(m);
        set_int(m, SIMPLIFIER, 0);
        wrapper_solve(m);
        dump(m, "C3 main LP, minimize, SIMPLIFIER off", true, main_cn,
             main_rn);
    }
    for(int rep : {1, 2}) {
        for(int alg : {0, 1}) {
            soplex_lp m(api);
            build_main(m);
            set_int(m, SIMPLIFIER, 0);
            set_int(m, REPRESENTATION, rep);
            set_int(m, ALGORITHM, alg);
            wrapper_solve(m);
            dump(m,
                 std::string("C4 main LP, SIMPLIFIER off, REPRESENTATION=") +
                     (rep == 1 ? "COLUMN" : "ROW") +
                     ", ALGORITHM=" + (alg == 0 ? "PRIMAL" : "DUAL"),
                 true, main_cn, main_rn);
        }
    }
    {
        // tall: 15 extra slack rows, not redundant by bounds, not parallel
        soplex_lp m(api);
        auto l = build_main(m);
        for(int k = 0; k < 15; ++k)
            m.add_constraint(l.y1 + (2.0 + 0.1 * k) * l.y2 <= 12.0 + k);
        wrapper_solve(m);
        dump(m, "C5 tall LP (20 rows, 8 cols), wrapper defaults", true);
    }

    std::printf("\n==================== D: availability\n");
    {
        soplex_lp m(api);
        build_main(m);
        dump(m, "D1 before any solve", false, main_cn, main_rn);
    }
    {
        soplex_lp m(api);
        auto x = m.add_variable({.lower_bound = 0.0, .upper_bound = 10.0});
        auto y = m.add_variable({.lower_bound = 0.0, .upper_bound = 10.0});
        m.add_constraint(x + y >= 5.0);
        m.add_constraint(x + y <= 3.0);
        m.add_constraint(x - y <= 1.0);
        m.set_objective(x + 2 * y);
        m.set_time_limit(std::chrono::duration<double>(10.0));
        wrapper_solve(m);
        dump(m, "D2a infeasible LP, wrapper defaults (presolve on)", true);
        soplex_lp m2(api);
        auto x2 = m2.add_variable({.lower_bound = 0.0, .upper_bound = 10.0});
        auto y2 = m2.add_variable({.lower_bound = 0.0, .upper_bound = 10.0});
        m2.add_constraint(x2 + y2 >= 5.0);
        m2.add_constraint(x2 + y2 <= 3.0);
        m2.add_constraint(x2 - y2 <= 1.0);
        m2.set_objective(x2 + 2 * y2);
        m2.set_time_limit(std::chrono::duration<double>(10.0));
        set_int(m2, SIMPLIFIER, 0);
        wrapper_solve(m2);
        dump(m2, "D2b infeasible LP, SIMPLIFIER off", true);
    }
    for(int simp : {1, 0}) {
        soplex_lp m(api);
        auto x = m.add_variable();
        auto y = m.add_variable();
        auto z = m.add_variable({.lower_bound = 0.0, .upper_bound = 3.0});
        m.add_constraint(x - y <= 1.0);
        m.add_constraint(x + z >= 1.0);
        m.set_objective(-x - y + z);
        m.set_time_limit(std::chrono::duration<double>(10.0));
        set_int(m, SIMPLIFIER, simp);
        wrapper_solve(m);
        dump(m, std::string("D3 unbounded LP, SIMPLIFIER ") +
                    (simp ? "auto" : "off"),
             true);
    }
    {
        soplex_lp m(api);
        build_dense(m, 30, 20);
        wrapper_solve(m);
        std::printf("  (D4 reference: dense LP cold solve above)\n");
        soplex_lp m2(api);
        build_dense(m2, 30, 20);
        set_int(m2, ITERLIMIT, 3);
        wrapper_solve(m2);
        dump(m2, "D4a dense LP, ITERLIMIT=3, wrapper defaults", true);
        soplex_lp m3(api);
        build_dense(m3, 30, 20);
        set_int(m3, ITERLIMIT, 3);
        set_int(m3, SIMPLIFIER, 0);
        wrapper_solve(m3);
        dump(m3, "D4b dense LP, ITERLIMIT=3, SIMPLIFIER off", true);
        soplex_lp m4(api);
        build_dense(m4, 30, 20);
        m4.set_time_limit(std::chrono::duration<double>(0.0));
        wrapper_solve(m4);
        dump(m4, "D4c dense LP, time limit 0 s", true);
        // continue from the iteration-limit basis
        set_int(m2, ITERLIMIT, -1);
        wrapper_solve(m2);
        std::printf("  D4d: resolving D4a without limit continues from its "
                    "basis (iterations above)\n");
    }
    {
        soplex_lp m(api);
        auto l = build_main(m);
        wrapper_solve(m);
        auto before = read_codes(m);
        auto c_new = m.add_constraint(l.y1 + l.y3 <= 50.0);
        dump(m, "D5a after add_constraint (y1 + y3 <= 50), no re-solve", false,
             main_cn);
        (void)c_new;
        m.set_variable_upper_bound(l.xup, 3.5);
        dump(m, "D5b after xup ub 4 -> 3.5 (xup was ON_UPPER), no re-solve",
             false, main_cn);
        m.set_variable_upper_bound(l.xup, m.infinity());
        dump(m, "D5c after xup ub -> infinity (xup was ON_UPPER)", false,
             main_cn);
        m.set_variable_upper_bound(l.xup, 4.0);
        m.set_variable_lower_bound(l.xlow, -m.infinity());
        dump(m, "D5d after xup ub back to 4, xlow lb -> -inf (xlow was ON_LOWER)",
             false, main_cn);
        m.set_variable_lower_bound(l.xlow, 1.0);
        auto v_new = m.add_variable({.lower_bound = 0.0, .upper_bound = 10.0});
        auto v_new2 = m.add_column({{l.r_slk, 1.0}, {l.r_le, 1.0}},
                                   {.obj_coef = 1.0,
                                    .lower_bound = std::nullopt,
                                    .upper_bound = 6.0});
        (void)v_new;
        (void)v_new2;
        dump(m,
             "D5e after add_variable [0,10] and add_column (-inf,6] (no "
             "re-solve)",
             false, main_cn);
        m.set_constraint_upper_bound(l.r_le, m.infinity());
        dump(m, "D5f after freeing r_le's active upper side (no re-solve)",
             false, main_cn);
        m.set_constraint_upper_bound(l.r_le, 8.0);
        dump(m, "D5g after restoring r_le's upper side", false, main_cn);
        wrapper_solve(m);
        dump(m, "D5h after re-solve", true, main_cn);
        (void)before;
    }
    {
        soplex_lp m(api);
        m.set_time_limit(std::chrono::duration<double>(10.0));
        auto x = m.add_variable({.lower_bound = 1.0, .upper_bound = 3.0});
        auto y = m.add_variable({.lower_bound = 0.0, .upper_bound = 3.0});
        m.add_constraint(x + y <= 10.0);
        m.set_objective(x - y);
        wrapper_solve(m);
        dump(m, "D6 LP that presolve solves entirely (redundant row)", true);
    }

    std::printf("\n==================== E: warm start\n");
    codes_t optimal_codes;
    std::string optimal_bas;
    {
        soplex_lp a(api);
        build_main(a);
        wrapper_solve(a);
        optimal_codes = read_codes(a);
        bool ok = false;
        optimal_bas = bas_from_codes(optimal_codes, &ok);
        std::printf("  basis of A read; pairing ok = %d\n", ok);
    }
    {
        soplex_lp c(api);
        build_main(c);
        set_int(c, ITERLIMIT, 0);
        std::printf("-- E0 control: fresh B, no basis, ITERLIMIT=0\n");
        raw_optimize(c);
    }
    {
        soplex_lp b(api);
        build_main(b);
        std::printf("-- E1 fresh B, basis of A written via SoPlex_readBasisFile, "
                    "ITERLIMIT=0\n");
        write_basis_file(b, optimal_bas, "e1.bas");
        dump(b, "E1 codes read back after readBasisFile, before solve", false,
             main_cn, main_rn);
        set_int(b, ITERLIMIT, 0);
        raw_optimize(b);
        dump(b, "E1 after the 0-iteration solve", true, main_cn, main_rn);
        set_int(b, ITERLIMIT, -1);
        raw_optimize(b);
        std::printf("  (E1 re-solve without the limit)\n");
    }
    {
        soplex_lp b(api);
        build_main(b);
        write_basis_file(b, optimal_bas, "e1b.bas");
        std::printf("-- E1b fresh B, basis written, no iteration limit, "
                    "wrapper solve()\n");
        wrapper_solve(b);
        soplex_lp b2(api);
        build_main(b2);
        set_int(b2, SIMPLIFIER, 0);
        write_basis_file(b2, optimal_bas, "e1c.bas");
        std::printf("-- E1c same, SIMPLIFIER off\n");
        set_int(b2, ITERLIMIT, 0);
        raw_optimize(b2);
        soplex_lp b3(api);
        build_main(b3);
        set_int(b3, ALGORITHM, 0);
        write_basis_file(b3, optimal_bas, "e1d.bas");
        std::printf("-- E1d same, primal simplex\n");
        set_int(b3, ITERLIMIT, 0);
        raw_optimize(b3);
    }
    {
        soplex_lp a(api);
        auto l = build_main(a);
        wrapper_solve(a);
        std::printf("-- E2a A solved; xup ub 4 -> 3.5; write A's old basis "
                    "into A; ITERLIMIT=0\n");
        a.set_variable_upper_bound(l.xup, 3.5);
        write_basis_file(a, optimal_bas, "e2a.bas");
        dump(a, "E2a codes after readBasisFile", false, main_cn, main_rn);
        set_int(a, ITERLIMIT, 0);
        raw_optimize(a);
        std::printf("  expected optimum after the change: -21 + 3*0.5 - 0.5 "
                    "= -20\n");
    }
    {
        soplex_lp b(api);
        auto l = build_main(b);
        std::printf("-- E2b fresh B, basis written, THEN r_slk rhs 100 -> 200 "
                    "(basis stays optimal); ITERLIMIT=0\n");
        write_basis_file(b, optimal_bas, "e2b.bas");
        b.set_constraint_upper_bound(l.r_slk, 200.0);
        dump(b, "E2b codes after the change", false, main_cn, main_rn);
        set_int(b, ITERLIMIT, 0);
        raw_optimize(b);
        soplex_lp b2(api);
        auto l2 = build_main(b2);
        std::printf("-- E2c fresh B, basis written, THEN y1 ub 10 -> 4 (basis "
                    "primal infeasible); no limit\n");
        write_basis_file(b2, optimal_bas, "e2c.bas");
        b2.set_variable_upper_bound(l2.y1, 4.0);
        raw_optimize(b2);
        soplex_lp b3(api);
        auto l3 = build_main(b3);
        b3.set_variable_upper_bound(l3.y1, 4.0);
        std::printf("-- E2c control: cold solve of the changed LP\n");
        raw_optimize(b3);
    }
    {
        std::printf("-- E2d ranged row r_rng (C3) written XL (lower side) "
                    "instead of XU\n");
        soplex_lp b(api);
        build_main(b);
        codes_t c = optimal_codes;
        c.rows[3] = 1;
        write_basis_file(b, bas_from_codes(c), "e2d.bas");
        dump(b, "E2d codes after readBasisFile", false, main_cn, main_rn);
        raw_optimize(b);
        std::printf("-- E2e XL written on the <= row r_le (C0) and XU on the "
                    ">= row r_ge (C1): sides forced by row type?\n");
        soplex_lp b2(api);
        build_main(b2);
        codes_t c2 = optimal_codes;
        c2.rows[0] = 1;
        c2.rows[1] = 0;
        write_basis_file(b2, bas_from_codes(c2), "e2e.bas");
        dump(b2, "E2e codes after readBasisFile", false, main_cn, main_rn);
    }

    std::printf("\n==================== E3: invalid bases\n");
    auto invalid = [&](const std::string & label, const std::string & text,
                       const std::string & file) {
        std::printf("-- %s\n", label.c_str());
        soplex_lp b(api);
        build_main(b);
        write_basis_file(b, text, file);
        dump(b, label + ": codes after readBasisFile", false, main_cn,
             main_rn);
        raw_optimize(b);
        dump(b, label + ": after solve", true, main_cn, main_rn);
    };
    const std::string head = "NAME  soplex.bas\n";
    // optimal: y1..y4 basic paired with C0..C3; xup UL; xlow LL
    invalid("E3a too many basic (xlow basic, paired with C0 again)",
            body(optimal_bas) + xline("XU", 6, 0) + "ENDATA\n", "e3a.bas");
    invalid("E3b too few basic (y1 paired twice: C0 and C4 nonbasic)",
            body(optimal_bas) + xline("XU", 0, 4) + "ENDATA\n", "e3b.bas");
    invalid("E3c UL on a free column (infinite upper)",
            body(optimal_bas) + " UL " + cname(7) + "\nENDATA\n", "e3c.bas");
    invalid("E3d LL on a free column (infinite lower)",
            body(optimal_bas) + " LL " + cname(7) + "\nENDATA\n", "e3d.bas");
    invalid("E3e unknown column name x99", head + " UL x99\nENDATA\n",
            "e3e.bas");
    invalid("E3f singular: empty column xfree basic, paired with C4",
            body(optimal_bas) + xline("XU", 7, 4) + "ENDATA\n", "e3f.bas");
    invalid("E3g LL on the fixed column xfix (explicit lower)",
            body(optimal_bas) + " LL " + cname(5) + "\nENDATA\n", "e3g.bas");
    invalid("E3h short line ' XU x0 C0' (fixed-format heuristic)",
            head + " XU x0 C0\nENDATA\n", "e3h.bas");
    invalid("E3i plain names x1/C1 (what the C header suggests)",
            head + xline("XU", 0, 0) + " XL x1             C1\nENDATA\n",
            "e3i.bas");
    invalid("E3j empty basis file (NAME + ENDATA only)",
            head + "ENDATA\n", "e3j.bas");

    std::printf("\n==================== F: partial / extended bases\n");
    {
        soplex_lp b(api);
        auto l = build_main(b);
        b.add_column({{l.r_slk, 1.0}},
                     {.obj_coef = 1.0, .lower_bound = 0.0, .upper_bound = 10.0});
        b.add_constraint(l.y1 + l.y4 <= 50.0);
        std::printf("-- F1 model grew a column (x8, obj +1, lb 0) and a row "
                    "(C5); A's basis written (names only old entities); "
                    "ITERLIMIT=0\n");
        write_basis_file(b, optimal_bas, "f1.bas");
        dump(b, "F1 codes after readBasisFile", false);
        set_int(b, ITERLIMIT, 0);
        raw_optimize(b);
    }
    {
        soplex_lp b(api);
        build_main(b);
        codes_t c = optimal_codes;
        c.cols[4] = 1;  // drop the UL line of xup: written LL instead
        std::string t = bas_from_codes(c);
        auto pos = t.find(" LL " + cname(4) + "\n");
        t.erase(pos, 5 + cname(4).size());
        std::printf("-- F2 subset: A's basis without xup's UL line\n");
        write_basis_file(b, t, "f2.bas");
        dump(b, "F2 codes after readBasisFile", false, main_cn, main_rn);
        raw_optimize(b);
    }

    std::printf("\n==================== G: gotchas\n");
    for(int variant = 0; variant < 3; ++variant) {
        // the wrapper test freeing_an_active_row_keeps_the_resolve_right
        soplex_lp m(api);
        m.set_time_limit(std::chrono::duration<double>(10.0));
        auto x = m.add_variable({.lower_bound = 0.0, .upper_bound = 10.0});
        auto y = m.add_variable({.lower_bound = 0.0, .upper_bound = 10.0});
        auto z = m.add_variable({.lower_bound = 0.0, .upper_bound = 10.0});
        auto c1 = m.add_constraint(x + y <= 4.0);
        m.add_constraint(x - y >= -2.0);
        m.add_constraint(z == 1.0);
        m.set_maximization();
        m.set_objective(x + 2 * y + z);
        wrapper_solve(m);
        m.set_constraint_upper_bound(c1, m.infinity());
        const char * what[] = {
            "raw optimize (no reload)",
            "readBasisFile(empty NAME/ENDATA file) then raw optimize",
            "wrapper solve() (reload)"};
        std::printf("-- G1.%d freed active row, then %s; expect 31\n", variant,
                    what[variant]);
        dump(m, "G1 codes before re-solve", false);
        if(variant == 0) raw_optimize(m);
        if(variant == 1) {
            write_basis_file(m, head + "ENDATA\n", "g1.bas");
            dump(m, "G1.1 codes after readBasisFile(empty)", false);
            raw_optimize(m);
        }
        if(variant == 2) wrapper_solve(m);
        dump(m, "G1 after re-solve", true);
    }
    return 0;
}
