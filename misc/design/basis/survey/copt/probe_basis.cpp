// COPT basis survey probe: builds models with MIP++ (copt_lp) and reaches the
// native basis routines through native_api() / native_model(), plus dlsym for
// the routines the api struct does not declare.
#include <dlfcn.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <optional>
#include <random>
#include <ranges>
#include <string>
#include <vector>

#include "mippp/solvers/copt/all.hpp"

using namespace mippp;
using namespace mippp::copt::impl::v1;
using params_t = model_variable_params_t<copt_lp>;
using variable = model_variable_t<copt_lp>;
using constraint = model_constraint_t<copt_lp>;

constexpr int LOWER = 0, BASIC = 1, UPPER = 2, SUPERBASIC = 3, FIXED = 4;
constexpr int UNTOUCHED = -77;

static const char * code_name(int c) {
    switch(c) {
        case LOWER: return "LOWER";
        case BASIC: return "BASIC";
        case UPPER: return "UPPER";
        case SUPERBASIC: return "SUPERBASIC";
        case FIXED: return "FIXED";
        case UNTOUCHED: return "untouched";
        default: return "?";
    }
}

using set_slack_basis_t = int(copt_prob *);
using get_entity_basis_t = int(copt_prob *, int, const int *, int *);
set_slack_basis_t * SetSlackBasis = nullptr;
get_entity_basis_t * GetColBasis = nullptr;
get_entity_basis_t * GetRowBasis = nullptr;

struct nat {
    const copt_api & api;
    copt_prob * prob;
    explicit nat(copt_lp & m)
        : api(m.native_api()), prob(m.native_model().second) {}
    int iattr(const char * n) {
        int v = -999;
        int r = api.GetIntAttr(prob, n, &v);
        return r ? -1000 - r : v;
    }
    double dattr(const char * n) {
        double v = NAN;
        int r = api.GetDblAttr(prob, n, &v);
        return r ? -1000.0 - r : v;
    }
    int setint(const char * n, int v) { return api.SetIntParam(prob, n, v); }
    int cols() { return iattr("Cols"); }
    int rows() { return iattr("Rows"); }
    // returns ret code, fills arrays (prefilled UNTOUCHED)
    int get_basis(std::vector<int> & c, std::vector<int> & r) {
        c.assign(static_cast<std::size_t>(cols()), UNTOUCHED);
        r.assign(static_cast<std::size_t>(rows()), UNTOUCHED);
        return api.GetBasis(prob, c.data(), r.data());
    }
    int set_basis(const std::vector<int> & c, const std::vector<int> & r) {
        return api.SetBasis(prob, c.data(), r.data());
    }
    std::string state() {
        char buf[256];
        std::snprintf(buf, sizeof buf,
                      "LpStatus=%d HasBasis=%d HasLpSol=%d SimplexIter=%d "
                      "BarrierIter=%d PDLPIter=%d",
                      iattr("LpStatus"), iattr("HasBasis"), iattr("HasLpSol"),
                      iattr("SimplexIter"), iattr("BarrierIter"),
                      iattr("PDLPIter"));
        return buf;
    }
};

static std::string codes(const std::vector<int> & v) {
    std::string s;
    for(int c : v) {
        s += code_name(c);
        if(std::string(code_name(c)) == "?") s += "(" + std::to_string(c) + ")";
        s += " ";
    }
    return s;
}
static int count_basic(const std::vector<int> & v) {
    return static_cast<int>(std::ranges::count(v, BASIC));
}

///////////////////////////////////////////////////////////////////////////////
// Convention LP
///////////////////////////////////////////////////////////////////////////////
struct conv_handles {
    variable x1, x2, x3, x4, y, z, f, l;
    constraint r1, r2, r3, r4, r5;
};
static const char * conv_col_names[] = {"x1(basic)",  "x2(basic)",
                                        "x3(basic)",  "x4(basic)",
                                        "y[0,3]",     "z[2,2]fixed",
                                        "f free,empty", "l[0,inf)"};
static const char * conv_row_names[] = {
    "R1 <=4 tight", "R2 >=2 tight", "R3 ==1", "R4 ranged", "R5 <=100 slack"};

// ranged_at_upper: R4 = [1,3] tight at 3; else R4 = [3,7] tight at 3.
static conv_handles build_conv(copt_lp & m, bool ranged_at_upper,
                               bool maximize = false) {
    using namespace operators;
    m.set_time_limit(std::chrono::seconds(10));
    conv_handles h;
    h.x1 = m.add_variable();
    h.x2 = m.add_variable();
    h.x3 = m.add_variable();
    h.x4 = m.add_variable();
    h.y = m.add_variable(params_t{.obj_coef = 0, .lower_bound = 0, .upper_bound = 3});
    h.z = m.add_variable(params_t{.obj_coef = 0, .lower_bound = 2, .upper_bound = 2});
    h.f = m.add_variable(params_t{.obj_coef = 0, .lower_bound = std::nullopt,
                                  .upper_bound = std::nullopt});
    h.l = m.add_variable(params_t{.obj_coef = 0, .lower_bound = 0,
                                  .upper_bound = std::nullopt});
    h.r1 = m.add_constraint(h.x1 + h.x2 <= 4);
    h.r2 = m.add_constraint(h.x3 + h.x4 >= 2);
    h.r3 = m.add_constraint(h.x1 - h.x3 == 1);
    if(ranged_at_upper) {
        h.r4 = m.add_constraint(h.x2 + h.x4 <= 3);
        m.set_constraint_lower_bound(h.r4, 1);
    } else {
        h.r4 = m.add_constraint(h.x2 + h.x4 >= 3);
        m.set_constraint_upper_bound(h.r4, 7);
    }
    h.r5 = m.add_constraint(h.x1 + h.x2 + h.x3 + h.x4 + h.y + h.z + h.l <= 100);
    const double c2 = ranged_at_upper ? -2.0 : 0.0;
    const double c4 = ranged_at_upper ? 0.0 : 2.0;
    const double s = maximize ? -1.0 : 1.0;
    if(maximize)
        m.set_maximization();
    else
        m.set_minimization();
    m.set_objective(s * -0.5 * h.x1 + s * c2 * h.x2 + s * 0.5 * h.x3 +
                    s * c4 * h.x4 + s * -1.0 * h.y + s * 1.0 * h.z +
                    s * 1.0 * h.l);
    return h;
}

static void print_conv(copt_lp & m) {
    nat n(m);
    std::vector<int> cb, rb;
    int ret = n.get_basis(cb, rb);
    std::printf("    %s  GetBasis ret=%d obj=%g\n", n.state().c_str(), ret,
                n.dattr("LpObjval"));
    std::vector<double> val(cb.size()), red(cb.size()), act(rb.size()),
        dual(rb.size());
    int r2 = n.api.GetLpSolution(n.prob, val.data(), act.data(), dual.data(),
                                 red.data());
    std::vector<double> rlb(rb.size()), rub(rb.size());
    n.api.GetRowInfo(n.prob, "LB", static_cast<int>(rb.size()), nullptr,
                     rlb.data());
    n.api.GetRowInfo(n.prob, "UB", static_cast<int>(rb.size()), nullptr,
                     rub.data());
    for(std::size_t j = 0; j < cb.size(); ++j)
        std::printf("      col %-14s code=%-10s value=%g redcost=%g\n",
                    conv_col_names[j], code_name(cb[j]), r2 ? NAN : val[j],
                    r2 ? NAN : red[j]);
    for(std::size_t i = 0; i < rb.size(); ++i)
        std::printf(
            "      row %-15s code=%-10s activity(\"slack\")=%g dual=%g "
            "[LB=%g, UB=%g]\n",
            conv_row_names[i], code_name(rb[i]), r2 ? NAN : act[i],
            r2 ? NAN : dual[i], rlb[i], rub[i]);
    std::printf("      #basic = %d (cols %d + rows %d), rows = %zu\n",
                count_basic(cb) + count_basic(rb), count_basic(cb),
                count_basic(rb), rb.size());
}

struct config {
    const char * name;
    std::vector<std::pair<const char *, int>> int_params;
};

static void section_C() {
    std::printf("\n=== C. row side convention ===\n");
    const std::vector<config> configs = {
        {"wrapper defaults", {}},
        {"Presolve=0", {{"Presolve", 0}}},
        {"LpMethod=1 dual simplex, Presolve=0", {{"LpMethod", 1}, {"Presolve", 0}}},
        {"LpMethod=2 barrier, Crossover=1 (default)", {{"LpMethod", 2}}},
        {"LpMethod=2 barrier, Crossover=0", {{"LpMethod", 2}, {"Crossover", 0}}},
        {"LpMethod=2 barrier, Crossover=0, Presolve=0",
         {{"LpMethod", 2}, {"Crossover", 0}, {"Presolve", 0}}},
        {"LpMethod=4 concurrent", {{"LpMethod", 4}}},
        {"LpMethod=5 auto simplex/barrier", {{"LpMethod", 5}}},
        {"LpMethod=6 PDLP, Crossover=1", {{"LpMethod", 6}}},
        {"LpMethod=6 PDLP, Crossover=0", {{"LpMethod", 6}, {"Crossover", 0}}},
        {"LpMethod=3 crossover-only", {{"LpMethod", 3}}},
    };
    for(bool at_upper : {true, false}) {
        for(const auto & cfg : configs) {
            if(!at_upper && cfg.int_params.size() > 0 &&
               std::string(cfg.name) != "Presolve=0" &&
               std::string(cfg.name) != "LpMethod=2 barrier, Crossover=0")
                continue;
            try {
                copt_lp m;
                build_conv(m, at_upper);
                nat n(m);
                std::string pr;
                for(auto [k, v] : cfg.int_params)
                    pr += std::string(k) + "=" + std::to_string(v) + ":ret" +
                          std::to_string(n.setint(k, v)) + " ";
                m.solve();
                std::printf("  -- R4 %s, %s [%s]\n",
                            at_upper ? "[1,3] tight at UPPER"
                                     : "[3,7] tight at LOWER",
                            cfg.name, pr.c_str());
                print_conv(m);
            } catch(const std::exception & e) {
                std::printf("  -- %s: exception %s\n", cfg.name, e.what());
            }
        }
    }
    for(bool presolve : {true, false}) {
        try {
            copt_lp m;
            build_conv(m, true, /*maximize=*/true);
            nat n(m);
            if(!presolve) n.setint("Presolve", 0);
            m.solve();
            std::printf("  -- maximize(-obj), R4 at upper, Presolve %s\n",
                        presolve ? "default" : "0");
            print_conv(m);
        } catch(const std::exception & e) {
            std::printf("  -- maximize: exception %s\n", e.what());
        }
    }
    // slack basis
    try {
        copt_lp m;
        build_conv(m, true);
        nat n(m);
        int r = SetSlackBasis ? SetSlackBasis(n.prob) : -12345;
        std::printf("  -- COPT_SetSlackBasis before any solve: ret=%d\n", r);
        print_conv(m);
    } catch(const std::exception & e) {
        std::printf("  -- slack basis: exception %s\n", e.what());
    }
}

///////////////////////////////////////////////////////////////////////////////
// Dense LP for warm start (as test_suites/iteration_limit.hpp)
///////////////////////////////////////////////////////////////////////////////
static void build_dense(copt_lp & m, std::size_t size = 200) {
    using namespace operators;
    m.set_time_limit(std::chrono::seconds(20));
    std::mt19937 rng(42);
    std::uniform_int_distribution<int> coef(1, 1000);
    auto columns = std::views::iota(std::size_t{0}, size);
    auto x = m.add_variables(size);
    std::vector<int> row(size);
    for(int & a : row) a = coef(rng);
    m.set_maximization();
    m.set_objective(xsum(columns, [&](auto j) { return row[j] * x(j); }));
    for(std::size_t i = 0; i < size; ++i) {
        for(int & a : row) a = coef(rng);
        m.add_constraint(xsum(columns, [&](auto j) { return row[j] * x(j); }) <=
                         1000.0 * static_cast<double>(size));
    }
}

static void section_D() {
    std::printf("\n=== D. availability ===\n");
    // D1 before any solve
    try {
        copt_lp m;
        build_conv(m, true);
        nat n(m);
        std::vector<int> cb, rb;
        int ret = n.get_basis(cb, rb);
        std::printf("  D1 before solve: %s GetBasis ret=%d\n      cols: %s\n      rows: %s\n",
                    n.state().c_str(), ret, codes(cb).c_str(),
                    codes(rb).c_str());
        if(GetColBasis) {
            std::vector<int> one(1, UNTOUCHED);
            int idx = 4;
            int r = GetColBasis(n.prob, 1, &idx, one.data());
            std::printf("      GetColBasis(y) ret=%d code=%s\n", r,
                        code_name(one[0]));
        }
    } catch(const std::exception & e) {
        std::printf("  D1: exception %s\n", e.what());
    }
    // D0 empty model
    try {
        copt_lp m;
        nat n(m);
        std::vector<int> cb, rb;
        int dummy_c = UNTOUCHED, dummy_r = UNTOUCHED;
        int ret = n.api.GetBasis(n.prob, &dummy_c, &dummy_r);
        std::printf("  D0 empty model, no solve: %s GetBasis ret=%d\n",
                    n.state().c_str(), ret);
        m.solve();
        ret = n.api.GetBasis(n.prob, &dummy_c, &dummy_r);
        std::printf("  D0 empty model, solved: %s GetBasis ret=%d\n",
                    n.state().c_str(), ret);
    } catch(const std::exception & e) {
        std::printf("  D0: exception %s\n", e.what());
    }
    // D3 infeasible, D4 unbounded
    for(int presolve : {-1, 0}) {
        for(int kind : {0, 1}) {
            try {
                using namespace operators;
                copt_lp m;
                m.set_time_limit(std::chrono::seconds(10));
                nat n(m);
                n.setint("Presolve", presolve);
                auto x = m.add_variable();
                auto y = m.add_variable();
                if(kind == 0) {
                    m.add_constraint(x + y >= 5);
                    m.add_constraint(x + 2 * y <= 3);
                } else {
                    m.set_maximization();
                    m.set_objective(x + y);
                    m.add_constraint(x - y <= 1);
                    m.add_constraint(x + y >= 1);
                }
                m.solve();
                std::vector<int> cb, rb;
                int ret = n.get_basis(cb, rb);
                std::printf("  D%d %s Presolve=%d: %s GetBasis ret=%d cols: %s rows: %s\n",
                            kind == 0 ? 3 : 4,
                            kind == 0 ? "infeasible" : "unbounded", presolve,
                            n.state().c_str(), ret, codes(cb).c_str(),
                            codes(rb).c_str());
            } catch(const std::exception & e) {
                std::printf("  D3/4: exception %s\n", e.what());
            }
        }
    }
    // D5 iteration limit params (hidden?)
    try {
        copt_lp m;
        nat n(m);
        for(const char * p : {"IterLimit", "SimplexIterLimit", "LpIterLimit",
                              "BarIterLimit"}) {
            int r = n.setint(p, 0);
            int v = -999;
            int g = n.api.GetIntParam(n.prob, p, &v);
            std::printf("  D5 SetIntParam(\"%s\", 0) ret=%d; GetIntParam ret=%d value=%d\n",
                        p, r, g, v);
            double dv = NAN;
            int gd = n.api.GetDblParam(n.prob, p, &dv);
            std::printf("     GetDblParam(\"%s\") ret=%d value=%g\n", p, gd, dv);
        }
    } catch(const std::exception & e) {
        std::printf("  D5: exception %s\n", e.what());
    }
    // D5 time limit stop
    for(double tl : {0.0, 1e-4, 0.01}) {
        for(int presolve : {-1, 0}) {
            try {
                copt_lp m;
                build_dense(m, 300);
                nat n(m);
                n.setint("Presolve", presolve);
                m.set_time_limit(std::chrono::duration<double>(tl));
                m.solve();
                std::vector<int> cb, rb;
                int ret = n.get_basis(cb, rb);
                std::printf("  D5 time limit %g Presolve=%d: %s GetBasis ret=%d #basic=%d rows=%zu"
                            " untouched=%d\n",
                            tl, presolve, n.state().c_str(), ret,
                            count_basic(cb) + count_basic(rb), rb.size(),
                            static_cast<int>(std::ranges::count(cb, UNTOUCHED) +
                                             std::ranges::count(rb, UNTOUCHED)));
            } catch(const std::exception & e) {
                std::printf("  D5 tl: exception %s\n", e.what());
            }
        }
    }
    // D6 modifications after an optimal solve
    const std::vector<std::pair<const char *, std::function<void(copt_lp &, conv_handles &)>>>
        mods = {
            {"add a row (slack)",
             [](copt_lp & m, conv_handles & h) {
                 using namespace operators;
                 m.add_constraint(h.x1 + h.x4 <= 50);
             }},
            {"change a column bound (y ub 3 -> 2)",
             [](copt_lp & m, conv_handles & h) {
                 m.set_variable_upper_bound(h.y, 2);
             }},
            {"change a column bound (l ub inf -> 1000, irrelevant)",
             [](copt_lp & m, conv_handles & h) {
                 m.set_variable_upper_bound(h.l, 1000);
             }},
            {"add a column", [](copt_lp & m, conv_handles &) { m.add_variable(); }},
            {"add a column with entries (add_column)",
             [](copt_lp & m, conv_handles & h) {
                 m.add_column({{h.r5, 1.0}},
                              params_t{.obj_coef = 3, .lower_bound = 0,
                                       .upper_bound = std::nullopt});
             }},
            {"change an objective coefficient",
             [](copt_lp & m, conv_handles & h) {
                 m.set_objective_coefficient(h.l, 2);
             }},
            {"change a row bound (R5 ub 100 -> 90)",
             [](copt_lp & m, conv_handles & h) {
                 m.set_constraint_upper_bound(h.r5, 90);
             }},
            {"COPT_Reset(prob, 0)",
             [](copt_lp & m, conv_handles &) {
                 auto & api = m.native_api();
                 std::printf("      Reset ret=%d\n",
                             static_cast<int>(api.Reset(m.native_model().second, 0)));
             }},
            {"COPT_Reset(prob, 1)",
             [](copt_lp & m, conv_handles &) {
                 auto & api = m.native_api();
                 std::printf("      Reset ret=%d\n",
                             static_cast<int>(api.Reset(m.native_model().second, 1)));
             }},
            {"set_time_limit (a parameter)",
             [](copt_lp & m, conv_handles &) {
                 m.set_time_limit(std::chrono::seconds(9));
             }},
            {"compute_iis() on the feasible model",
             [](copt_lp & m, conv_handles &) {
                 auto snap = m.compute_iis();
                 (void)snap;
             }},
        };
    for(int presolve : {-1, 0}) {
        for(auto & [name, mod] : mods) {
            try {
                copt_lp m;
                auto h = build_conv(m, true);
                nat n(m);
                n.setint("Presolve", presolve);
                m.solve();
                std::vector<int> cb0, rb0;
                n.get_basis(cb0, rb0);
                mod(m, h);
                std::vector<int> cb, rb;
                int ret = n.get_basis(cb, rb);
                std::printf("  D6 Presolve=%d after \"%s\": %s GetBasis ret=%d\n"
                            "      cols: %s\n      rows: %s\n",
                            presolve, name, n.state().c_str(), ret,
                            codes(cb).c_str(), codes(rb).c_str());
                m.solve();
                std::printf("      re-solve: %s obj=%g\n", n.state().c_str(),
                            n.dattr("LpObjval"));
            } catch(const std::exception & e) {
                std::printf("  D6 %s: exception %s\n", name, e.what());
            }
        }
    }
    // re-solve of an unchanged model
    try {
        copt_lp m;
        build_dense(m);
        nat n(m);
        m.solve();
        std::printf("  D7 dense first solve: %s\n", n.state().c_str());
        m.solve();
        std::printf("  D7 dense re-solve unchanged: %s\n", n.state().c_str());
    } catch(const std::exception & e) {
        std::printf("  D7: exception %s\n", e.what());
    }
}

///////////////////////////////////////////////////////////////////////////////
// E. warm start
///////////////////////////////////////////////////////////////////////////////
static void section_E() {
    std::printf("\n=== E. warm start ===\n");
    std::vector<int> cb, rb;
    double objA = NAN;
    {
        copt_lp a;
        build_dense(a);
        nat n(a);
        a.solve();
        n.get_basis(cb, rb);
        objA = n.dattr("LpObjval");
        std::printf("  A cold (defaults): %s obj=%.10g #basic=%d rows=%zu\n",
                    n.state().c_str(), objA, count_basic(cb) + count_basic(rb),
                    rb.size());
        int ncol_at[5] = {0, 0, 0, 0, 0}, nrow_at[5] = {0, 0, 0, 0, 0};
        for(int c : cb) if(c >= 0 && c < 5) ++ncol_at[c];
        for(int c : rb) if(c >= 0 && c < 5) ++nrow_at[c];
        std::printf("    col codes L/B/U/S/F = %d/%d/%d/%d/%d ; row codes = %d/%d/%d/%d/%d\n",
                    ncol_at[0], ncol_at[1], ncol_at[2], ncol_at[3], ncol_at[4],
                    nrow_at[0], nrow_at[1], nrow_at[2], nrow_at[3], nrow_at[4]);
    }
    {
        copt_lp c;
        build_dense(c);
        nat n(c);
        n.setint("Presolve", 0);
        c.solve();
        std::printf("  C cold, Presolve=0: %s obj=%.10g\n", n.state().c_str(),
                    n.dattr("LpObjval"));
    }
    const std::vector<config> configs = {
        {"defaults", {}},
        {"Presolve=0", {{"Presolve", 0}}},
        {"LpMethod=1", {{"LpMethod", 1}}},
        {"LpMethod=1 Presolve=0", {{"LpMethod", 1}, {"Presolve", 0}}},
        {"LpMethod=2 barrier", {{"LpMethod", 2}}},
        {"LpMethod=4 concurrent", {{"LpMethod", 4}}},
        {"LpMethod=6 PDLP", {{"LpMethod", 6}}},
    };
    for(const auto & cfg : configs) {
        try {
            copt_lp b;
            build_dense(b);
            nat n(b);
            for(auto [k, v] : cfg.int_params) n.setint(k, v);
            int sret = n.set_basis(cb, rb);
            std::vector<int> cb2, rb2;
            int gret = n.get_basis(cb2, rb2);
            std::printf("  B [%s]: SetBasis ret=%d; before solve %s; GetBasis ret=%d round-trip equal=%d\n",
                        cfg.name, sret, n.state().c_str(), gret,
                        cb2 == cb && rb2 == rb);
            b.solve();
            std::printf("     after solve: %s obj=%.10g (A %.10g)\n",
                        n.state().c_str(), n.dattr("LpObjval"), objA);
        } catch(const std::exception & e) {
            std::printf("  B [%s]: exception %s\n", cfg.name, e.what());
        }
    }
    // E2: basis set, then a modification, then solve -> kept or dropped?
    const std::vector<std::pair<const char *, int>> mods = {
        {"none", 0},
        {"upper bound of a nonbasic column inf -> 1e6 (irrelevant)", 1},
        {"upper bound of a basic column inf -> 1e6 (irrelevant)", 2},
        {"add a slack row", 3},
        {"add an empty column", 4},
        {"objective coefficient of a nonbasic column made less attractive", 5},
        {"set_time_limit", 6},
        {"upper bound of a basic column tightened below its value", 7},
    };
    for(int presolve : {-1, 0}) {
        for(auto [name, which] : mods) {
            try {
                using namespace operators;
                copt_lp b;
                build_dense(b);
                nat n(b);
                n.setint("Presolve", presolve);
                int sret = n.set_basis(cb, rb);
                int nb = -1, bb = -1;
                for(std::size_t j = 0; j < cb.size(); ++j) {
                    if(cb[j] == LOWER && nb < 0) nb = static_cast<int>(j);
                    if(cb[j] == BASIC && bb < 0) bb = static_cast<int>(j);
                }
                double val_bb = NAN;
                if(which == 7) {
                    // value of the basic column at A's optimum
                    copt_lp a;
                    build_dense(a);
                    a.solve();
                    val_bb = a.get_solution()[variable(bb)];
                }
                switch(which) {
                    case 1: b.set_variable_upper_bound(variable(nb), 1e6); break;
                    case 2: b.set_variable_upper_bound(variable(bb), 1e6); break;
                    case 3: b.add_constraint(variable(0) + variable(1) <= 1e7); break;
                    case 4: b.add_variable(); break;
                    case 5: b.set_objective_coefficient(variable(nb), -5); break;
                    case 6: b.set_time_limit(std::chrono::seconds(15)); break;
                    case 7: b.set_variable_upper_bound(variable(bb), val_bb / 2); break;
                    default: break;
                }
                std::printf("  E2 Presolve=%d SetBasis(ret=%d) then \"%s\": before solve %s\n",
                            presolve, sret, name, n.state().c_str());
                b.solve();
                std::printf("     after solve: %s obj=%.10g\n", n.state().c_str(),
                            n.dattr("LpObjval"));
            } catch(const std::exception & e) {
                std::printf("  E2 %s: exception %s\n", name, e.what());
            }
        }
    }
    // E2b: modification then SetBasis (on the solved A itself)
    for(int presolve : {-1, 0}) {
        try {
            copt_lp a;
            build_dense(a);
            nat n(a);
            n.setint("Presolve", presolve);
            a.solve();
            std::vector<int> c0, r0;
            n.get_basis(c0, r0);
            int bb = -1;
            for(std::size_t j = 0; j < c0.size(); ++j)
                if(c0[j] == BASIC) { bb = static_cast<int>(j); break; }
            double v = a.get_solution()[variable(bb)];
            a.set_variable_upper_bound(variable(bb), v / 2);
            std::printf("  E2b Presolve=%d A solved, basic col ub -> value/2: %s\n",
                        presolve, n.state().c_str());
            a.solve();
            std::printf("     re-solve without SetBasis: %s obj=%.10g\n",
                        n.state().c_str(), n.dattr("LpObjval"));
            copt_lp a2;
            build_dense(a2);
            nat n2(a2);
            n2.setint("Presolve", presolve);
            a2.set_variable_upper_bound(variable(bb), v / 2);
            a2.solve();
            std::printf("     fresh cold model with that bound: %s obj=%.10g\n",
                        n2.state().c_str(), n2.dattr("LpObjval"));
            copt_lp a3;
            build_dense(a3);
            nat n3(a3);
            n3.setint("Presolve", presolve);
            a3.set_variable_upper_bound(variable(bb), v / 2);
            int sret = n3.set_basis(c0, r0);
            a3.solve();
            std::printf("     fresh model with that bound + SetBasis(A's, ret=%d): %s obj=%.10g\n",
                        sret, n3.state().c_str(), n3.dattr("LpObjval"));
        } catch(const std::exception & e) {
            std::printf("  E2b: exception %s\n", e.what());
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
// E3. invalid bases on the convention LP
///////////////////////////////////////////////////////////////////////////////
static void section_E3() {
    std::printf("\n=== E3. invalid bases (convention LP, R4 at upper) ===\n");
    std::vector<int> cb, rb;
    double obj_ref = NAN;
    {
        copt_lp m;
        build_conv(m, true);
        nat n(m);
        n.setint("Presolve", 0);
        m.solve();
        n.get_basis(cb, rb);
        obj_ref = n.dattr("LpObjval");
        std::printf("  reference (Presolve=0): obj=%g cols: %s rows: %s\n",
                    obj_ref, codes(cb).c_str(), codes(rb).c_str());
    }
    // col indices: x1 0, x2 1, x3 2, x4 3, y 4, z 5, f 6, l 7
    // row indices: R1 0, R2 1, R3 2, R4 3, R5 4
    struct edit {
        const char * name;
        std::vector<std::pair<int, int>> cols;  // (index, code)
        std::vector<std::pair<int, int>> rows;
    };
    const std::vector<edit> edits = {
        {"valid optimal basis (control)", {}, {}},
        {"l -> BASIC (m+1 basic)", {{7, BASIC}}, {}},
        {"x1 -> LOWER (m-1 basic)", {{0, LOWER}}, {}},
        {"l -> UPPER (infinite upper bound)", {{7, UPPER}}, {}},
        {"f (free) -> LOWER", {{6, LOWER}}, {}},
        {"f (free) -> UPPER", {{6, UPPER}}, {}},
        {"f (free) -> FIXED", {{6, FIXED}}, {}},
        {"l -> SUPERBASIC", {{7, SUPERBASIC}}, {}},
        {"l -> FIXED (not fixed)", {{7, FIXED}}, {}},
        {"z (fixed) -> LOWER", {{5, LOWER}}, {}},
        {"z (fixed) -> UPPER", {{5, UPPER}}, {}},
        {"y -> LOWER (suboptimal but valid)", {{4, LOWER}}, {}},
        {"l -> code 7 (out of range)", {{7, 7}}, {}},
        {"l -> code -1 (out of range)", {{7, -1}}, {}},
        {"R2 (>= row) -> UPPER (infinite activity upper)", {}, {{1, UPPER}}},
        {"R1 (<= row) -> LOWER (infinite activity lower)", {}, {{0, LOWER}}},
        {"R3 (== row) -> LOWER", {}, {{2, LOWER}}},
        {"R3 (== row) -> UPPER", {}, {{2, UPPER}}},
        {"R3 (== row) -> FIXED", {}, {{2, FIXED}}},
        {"R1 -> UPPER, R2 -> LOWER (activity convention)", {}, {{0, UPPER}, {1, LOWER}}},
        {"R1 -> LOWER, R2 -> UPPER (slack convention)", {}, {{0, LOWER}, {1, UPPER}}},
        {"R5 -> SUPERBASIC", {}, {{4, SUPERBASIC}}},
        {"every column and row BASIC", {{0, BASIC}, {1, BASIC}, {2, BASIC}, {3, BASIC}, {4, BASIC}, {5, BASIC}, {6, BASIC}, {7, BASIC}},
         {{0, BASIC}, {1, BASIC}, {2, BASIC}, {3, BASIC}, {4, BASIC}}},
        {"slack basis by hand (cols LOWER, f SUPERBASIC, z FIXED, y LOWER, rows BASIC)",
         {{0, LOWER}, {1, LOWER}, {2, LOWER}, {3, LOWER}, {4, LOWER}, {5, FIXED}, {6, SUPERBASIC}, {7, LOWER}},
         {{0, BASIC}, {1, BASIC}, {2, BASIC}, {3, BASIC}, {4, BASIC}}},
    };
    for(int presolve : {0, -1}) {
        for(const auto & e : edits) {
            try {
                copt_lp m;
                build_conv(m, true);
                nat n(m);
                n.setint("Presolve", presolve);
                auto c = cb;
                auto r = rb;
                for(auto [i, code] : e.cols) c[static_cast<std::size_t>(i)] = code;
                for(auto [i, code] : e.rows) r[static_cast<std::size_t>(i)] = code;
                int sret = n.set_basis(c, r);
                std::vector<int> c2, r2;
                int gret = n.get_basis(c2, r2);
                std::printf("  Presolve=%d [%s]: SetBasis ret=%d; GetBasis ret=%d HasBasis=%d echo=%s\n",
                            presolve, e.name, sret, gret, n.iattr("HasBasis"),
                            (c2 == c && r2 == r) ? "same" : "DIFFERENT");
                if(!(c2 == c && r2 == r))
                    std::printf("      echo cols: %s\n      echo rows: %s\n",
                                codes(c2).c_str(), codes(r2).c_str());
                int solve_ret = n.api.SolveLp(n.prob);
                std::vector<int> c3, r3;
                n.get_basis(c3, r3);
                double o = n.dattr("LpObjval");
                std::printf("      SolveLp ret=%d %s obj=%g (%s)\n", solve_ret,
                            n.state().c_str(), o,
                            std::abs(o - obj_ref) < 1e-7 ? "correct" : "WRONG");
            } catch(const std::exception & e2) {
                std::printf("  [%s]: exception %s\n", e.name, e2.what());
            }
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
// F. partial / extended bases
///////////////////////////////////////////////////////////////////////////////
static void section_F() {
    std::printf("\n=== F. extended / partial bases ===\n");
    std::vector<int> cb, rb;
    double objA;
    {
        copt_lp a;
        build_dense(a);
        nat n(a);
        a.solve();
        n.get_basis(cb, rb);
        objA = n.dattr("LpObjval");
    }
    const std::size_t size = 200;
    for(int presolve : {-1, 0}) {
        for(int variant : {0, 1, 2, 3}) {
            // 0: new col LOWER, new row BASIC
            // 1: new col BASIC, new row BASIC (m+1 basic)
            // 2: new col LOWER, new row UPPER (m-1 basic)
            // 3: new col SUPERBASIC, new row BASIC
            try {
                using namespace operators;
                copt_lp b;
                build_dense(b);
                nat n(b);
                n.setint("Presolve", presolve);
                // new column: unattractive (objective 0 in a max problem with
                // positive entries in the new row only)
                auto nc = b.add_variable();
                b.add_constraint(variable(0) + variable(1) + nc <= 1e7);
                auto c = cb;
                auto r = rb;
                c.push_back(variant == 1 ? BASIC : variant == 3 ? SUPERBASIC : LOWER);
                r.push_back(variant == 2 ? UPPER : BASIC);
                int sret = n.set_basis(c, r);
                b.solve();
                std::printf("  F Presolve=%d variant %d: SetBasis ret=%d after solve %s obj=%.10g (A %.10g)\n",
                            presolve, variant, sret, n.state().c_str(),
                            n.dattr("LpObjval"), objA);
            } catch(const std::exception & e) {
                std::printf("  F variant %d: exception %s\n", variant, e.what());
            }
        }
    }
    // on A itself: solved, then add col and row, then SetBasis(extended)
    for(int presolve : {-1, 0}) {
        try {
            using namespace operators;
            copt_lp a;
            build_dense(a);
            nat n(a);
            n.setint("Presolve", presolve);
            a.solve();
            std::vector<int> c0, r0;
            n.get_basis(c0, r0);
            auto nc = a.add_variable();
            a.add_constraint(variable(0) + variable(1) + nc <= 1e7);
            std::vector<int> c1, r1;
            int g = n.get_basis(c1, r1);
            std::printf("  F on A Presolve=%d after add col+row: %s GetBasis ret=%d new col=%s new row=%s\n",
                        presolve, n.state().c_str(), g, code_name(c1.back()),
                        code_name(r1.back()));
            c0.push_back(LOWER);
            r0.push_back(BASIC);
            int sret = n.set_basis(c0, r0);
            a.solve();
            std::printf("     SetBasis(extended) ret=%d, re-solve: %s obj=%.10g\n",
                        sret, n.state().c_str(), n.dattr("LpObjval"));
        } catch(const std::exception & e) {
            std::printf("  F on A: exception %s\n", e.what());
        }
    }
    // nullptr arrays
    try {
        copt_lp b;
        build_dense(b);
        nat n(b);
        int r1 = n.api.SetBasis(n.prob, cb.data(), nullptr);
        std::printf("  F SetBasis(cols, nullptr) ret=%d %s\n", r1, n.state().c_str());
        int r2 = n.api.SetBasis(n.prob, nullptr, rb.data());
        std::printf("  F SetBasis(nullptr, rows) ret=%d %s\n", r2, n.state().c_str());
        b.solve();
        std::vector<int> c(size, UNTOUCHED);
        int r3 = n.api.GetBasis(n.prob, c.data(), nullptr);
        std::printf("  F GetBasis(cols, nullptr) after solve ret=%d first=%s\n", r3,
                    code_name(c[0]));
        std::vector<int> r(size, UNTOUCHED);
        int r4 = n.api.GetBasis(n.prob, nullptr, r.data());
        std::printf("  F GetBasis(nullptr, rows) ret=%d first=%s\n", r4,
                    code_name(r[0]));
        if(GetColBasis && GetRowBasis) {
            int list[2] = {3, 1};
            int out[2] = {UNTOUCHED, UNTOUCHED};
            int r5 = GetColBasis(n.prob, 2, list, out);
            std::printf("  F GetColBasis(list {3,1}) ret=%d -> %s %s (bulk: %s %s)\n", r5,
                        code_name(out[0]), code_name(out[1]), code_name(c[3]),
                        code_name(c[1]));
            int r6 = GetRowBasis(n.prob, 2, list, out);
            std::printf("  F GetRowBasis(list {3,1}) ret=%d -> %s %s (bulk: %s %s)\n", r6,
                        code_name(out[0]), code_name(out[1]), code_name(r[3]),
                        code_name(r[1]));
        }
    } catch(const std::exception & e) {
        std::printf("  F nullptr: exception %s\n", e.what());
    }
}

int main() {
    const copt_api & api = copt_api::load();
    std::printf("library: %s version: %s\n", api.library_path().c_str(),
                api.library_version() ? to_string(*api.library_version()).c_str()
                                      : "none");
    void * h = dlopen(api.library_path().c_str(), RTLD_NOW | RTLD_NOLOAD);
    if(h) {
        SetSlackBasis = reinterpret_cast<set_slack_basis_t *>(
            dlsym(h, "COPT_SetSlackBasis"));
        GetColBasis = reinterpret_cast<get_entity_basis_t *>(
            dlsym(h, "COPT_GetColBasis"));
        GetRowBasis = reinterpret_cast<get_entity_basis_t *>(
            dlsym(h, "COPT_GetRowBasis"));
    }
    std::printf("dlsym SetSlackBasis=%p GetColBasis=%p GetRowBasis=%p\n",
                reinterpret_cast<void *>(SetSlackBasis),
                reinterpret_cast<void *>(GetColBasis),
                reinterpret_cast<void *>(GetRowBasis));
    try {
        copt_lp m;
        nat n(m);
        for(const char * p : {"Presolve", "LpMethod", "Crossover", "Dualize",
                              "DualPerturb", "Threads"}) {
            int v = -999;
            int r = n.api.GetIntParam(n.prob, p, &v);
            std::printf("default %s = %d (ret %d)\n", p, v, r);
        }
    } catch(const std::exception & e) {
        std::printf("defaults: exception %s\n", e.what());
    }
    section_C();
    section_D();
    section_E();
    section_E3();
    section_F();
    std::printf("\nDONE\n");
}
