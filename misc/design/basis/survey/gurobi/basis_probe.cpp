// Survey probe: Gurobi LP basis (VBasis / CBasis) through MIP++'s gurobi_lp.
// Usage: basis_probe <section>   section in {P, C, D, E, F, G, all}
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <tuple>
#include <optional>
#include <ranges>
#include <string>
#include <vector>

#include "mippp/solvers/gurobi/all.hpp"

using namespace mippp;
using namespace mippp::operators;
namespace g = mippp::gurobi::impl::v1;
using var_t = model_variable_t<gurobi_lp>;
using con_t = model_constraint_t<gurobi_lp>;

struct N {
    const gurobi_api & G;
    g::GRBenv * env;
    g::GRBmodel * m;
    explicit N(gurobi_lp & model)
        : G(model.native_api())
        , env(model.native_model().first)
        , m(model.native_model().second) {}
    const char * err() { return G.geterrormsg(env); }
    int iparam(const char * p, int v) {
        int e = G.setintparam(env, p, v);
        if(e) std::printf("    setintparam(%s,%d) -> %d (%s)\n", p, v, e, err());
        return e;
    }
    int dparam(const char * p, double v) {
        int e = G.setdblparam(env, p, v);
        if(e) std::printf("    setdblparam(%s,%g) -> %d (%s)\n", p, v, e, err());
        return e;
    }
    int nv() { int v = -1; G.getintattr(m, "NumVars", &v); return v; }
    int nc() { int v = -1; G.getintattr(m, "NumConstrs", &v); return v; }
    int update() {
        int e = G.updatemodel(m);
        if(e) std::printf("    updatemodel -> %d (%s)\n", e, err());
        return e;
    }
    int optimize() {
        int e = G.optimize(m);
        if(e) std::printf("    optimize -> %d (%s)\n", e, err());
        return e;
    }
};

static const char * status_name(int s) {
    switch(s) {
        case 1: return "LOADED"; case 2: return "OPTIMAL"; case 3: return "INFEASIBLE";
        case 4: return "INF_OR_UNBD"; case 5: return "UNBOUNDED"; case 7: return "ITERATION_LIMIT";
        case 9: return "TIME_LIMIT"; case 11: return "INTERRUPTED"; case 12: return "NUMERIC";
        case 13: return "SUBOPTIMAL"; default: return "?";
    }
}

static void show_status(N & n, const char * label) {
    int st = -1, e1 = n.G.getintattr(n.m, "Status", &st);
    double it = -1; int e2 = n.G.getdblattr(n.m, "IterCount", &it);
    int bit = -1, e3 = n.G.getintattr(n.m, "BarIterCount", &bit);
    int cw = -9, e4 = n.G.getintattr(n.m, "ConcurrentWinMethod", &cw);
    double obj = NAN; int e5 = n.G.getdblattr(n.m, "ObjVal", &obj);
    int sc = -1; n.G.getintattr(n.m, "SolCount", &sc);
    std::printf("  [%s] Status=%d %s(e%d) IterCount=%g(e%d) BarIter=%d(e%d) "
                "ConcWin=%d(e%d) ObjVal=%.9g(e%d) SolCount=%d\n",
                label, st, status_name(st), e1, it, e2, bit, e3, cw, e4, obj, e5, sc);
}

struct basis {
    int ev = 0, ec = 0;
    std::vector<int> v, c;
};

static basis read_basis(N & n, int nv = -1, int nc = -1) {
    if(nv < 0) nv = n.nv();
    if(nc < 0) nc = n.nc();
    basis b;
    b.v.assign(static_cast<std::size_t>(nv), 99);
    b.c.assign(static_cast<std::size_t>(nc), 99);
    if(nv) b.ev = n.G.getintattrarray(n.m, "VBasis", 0, nv, b.v.data());
    if(nc) b.ec = n.G.getintattrarray(n.m, "CBasis", 0, nc, b.c.data());
    return b;
}
static void print_basis(N & n, const basis & b, const char * label) {
    std::printf("  [%s] VBasis(e%d%s%s):", label, b.ev, b.ev ? " " : "",
                b.ev ? n.err() : "");
    if(!b.ev) for(int x : b.v) std::printf(" %d", x);
    std::printf(" | CBasis(e%d):", b.ec);
    if(!b.ec) for(int x : b.c) std::printf(" %d", x);
    if(!b.ev && !b.ec) {
        int basics = 0;
        for(int x : b.v) basics += (x == 0);
        for(int x : b.c) basics += (x == 0);
        std::printf("  #basic=%d #rows=%zu", basics, b.c.size());
    }
    std::printf("\n");
}
static int write_basis(N & n, const std::vector<int> & v, const std::vector<int> & c,
                       const char * label) {
    std::vector<int> vv = v, cc = c;
    int ev = vv.empty() ? 0 : n.G.setintattrarray(n.m, "VBasis", 0, (int)vv.size(), vv.data());
    if(ev) std::printf("    [%s] set VBasis -> %d (%s)\n", label, ev, n.err());
    int ec = cc.empty() ? 0 : n.G.setintattrarray(n.m, "CBasis", 0, (int)cc.size(), cc.data());
    if(ec) std::printf("    [%s] set CBasis -> %d (%s)\n", label, ec, n.err());
    return ev ? ev : ec;
}

static void dump(N & n, const char * label, const std::vector<std::string> & cn,
                 const std::vector<std::string> & rn) {
    show_status(n, label);
    const int nv = n.nv(), nc = n.nc();
    std::vector<double> lb(nv), ub(nv), x(nv), rc(nv), rhs(nc), sl(nc), pi(nc);
    std::vector<int> vb(nv), cb(nc);
    std::vector<char> sense(nc);
    int e_lb = n.G.getdblattrarray(n.m, "LB", 0, nv, lb.data());
    int e_ub = n.G.getdblattrarray(n.m, "UB", 0, nv, ub.data());
    int e_x = n.G.getdblattrarray(n.m, "X", 0, nv, x.data());
    int e_rc = n.G.getdblattrarray(n.m, "RC", 0, nv, rc.data());
    int e_vb = n.G.getintattrarray(n.m, "VBasis", 0, nv, vb.data());
    int e_se = nc ? n.G.getcharattrarray(n.m, "Sense", 0, nc, sense.data()) : 0;
    int e_rhs = nc ? n.G.getdblattrarray(n.m, "RHS", 0, nc, rhs.data()) : 0;
    int e_sl = nc ? n.G.getdblattrarray(n.m, "Slack", 0, nc, sl.data()) : 0;
    int e_pi = nc ? n.G.getdblattrarray(n.m, "Pi", 0, nc, pi.data()) : 0;
    int e_cb = nc ? n.G.getintattrarray(n.m, "CBasis", 0, nc, cb.data()) : 0;
    std::printf("    errs: LB%d UB%d X%d RC%d VBasis%d Sense%d RHS%d Slack%d Pi%d CBasis%d\n",
                e_lb, e_ub, e_x, e_rc, e_vb, e_se, e_rhs, e_sl, e_pi, e_cb);
    int basics = 0;
    for(int j = 0; j < nv; ++j) {
        std::printf("    col %-3d %-8s lb=%-8g ub=%-8g", j,
                    j < (int)cn.size() ? cn[j].c_str() : "?", lb[j], ub[j]);
        if(!e_x) std::printf(" x=%-10.6g", x[j]);
        if(!e_rc) std::printf(" rc=%-10.6g", rc[j]);
        if(!e_vb) { std::printf(" VBasis=%d", vb[j]); basics += vb[j] == 0; }
        std::printf("\n");
    }
    for(int i = 0; i < nc; ++i) {
        std::printf("    row %-3d %-10s sense=%c rhs=%-8g", i,
                    i < (int)rn.size() ? rn[i].c_str() : "?", sense[i], rhs[i]);
        if(!e_sl) std::printf(" slack=%-10.6g", sl[i]);
        if(!e_pi) std::printf(" pi=%-10.6g", pi[i]);
        if(!e_cb) { std::printf(" CBasis=%d", cb[i]); basics += cb[i] == 0; }
        std::printf("\n");
    }
    if(!e_vb && !e_cb) std::printf("    #basic=%d #rows=%d\n", basics, nc);
}

/////////////////////////////////// models ///////////////////////////////////

// max 2x + 3y + u - v + t; unique optimum x=2 y=4 w=4 u=1 f=3 v=1 t=3, obj 19
struct core_t {
    var_t x, y, w, u, f, v, t, z1, z2;
    con_t r_le, r_ge, r_eq, r_slack, r_geslack;
};
static const std::vector<std::string> core_cols = {"x[0,10]", "y[0,10]", "w[0,inf)", "u[0,1]",
                                                   "f[3,3]", "v[1,5]", "t(-inf,3]", "z1free",
                                                   "z2free0"};
static const std::vector<std::string> core_rows = {"le:x+y<=6", "ge:x-y>=-2", "eq:y-w=0",
                                                   "slack<=100", "geslack>=1"};
static core_t build_core(gurobi_lp & m) {
    core_t c{
        m.add_variable({.lower_bound = 0., .upper_bound = 10.}),
        m.add_variable({.lower_bound = 0., .upper_bound = 10.}),
        m.add_variable({.lower_bound = 0.}),
        m.add_variable({.lower_bound = 0., .upper_bound = 1.}),
        m.add_variable({.lower_bound = 3., .upper_bound = 3.}),
        m.add_variable({.lower_bound = 1., .upper_bound = 5.}),
        m.add_variable({.lower_bound = std::nullopt, .upper_bound = 3.}),
        m.add_variable({}),
        m.add_variable({}),
        con_t(0), con_t(0), con_t(0), con_t(0), con_t(0)};
    m.set_maximization();
    m.set_objective(2 * c.x + 3 * c.y + c.u - c.v + c.t);
    c.r_le = m.add_constraint(c.x + c.y <= 6.);
    c.r_ge = m.add_constraint(c.x - c.y >= -2.);
    c.r_eq = m.add_constraint(c.y - c.w == 0.);
    c.r_slack = m.add_constraint(c.x + c.y + c.u + c.f + c.v + c.t + c.z1 <= 100.);
    c.r_geslack = m.add_constraint(c.x + c.y >= 1.);
    m.set_time_limit(std::chrono::duration<double>(10.));
    return c;
}

struct lcg {
    std::uint64_t s;
    double next() {
        s = s * 6364136223846793005ULL + 1442695040888963407ULL;
        return static_cast<double>(s >> 11) * (1.0 / 9007199254740992.0);
    }
};
// 60 columns in [0,10], 40 rows of mixed senses, x = 1 strictly feasible
static std::vector<var_t> build_medium(gurobi_lp & m, int n = 60, int rows = 40) {
    lcg r{12345};
    std::vector<var_t> xs;
    for(int j = 0; j < n; ++j)
        xs.push_back(m.add_variable({.lower_bound = 0., .upper_bound = 10.}));
    std::vector<double> c(n);
    for(auto & cj : c) cj = 1. + std::floor(9. * r.next());
    m.set_maximization();
    m.set_objective(xsum(std::views::iota(0, n), [&](int j) { return c[j] * xs[j]; }));
    for(int i = 0; i < rows; ++i) {
        std::vector<double> a(n, 0.);
        double act = 0;
        for(int j = 0; j < n; ++j) {
            if(r.next() < 0.5) continue;
            a[j] = std::floor(15. * r.next()) - 5.;
            act += a[j];
        }
        auto e = xsum(std::views::iota(0, n), [&, i](int j) { return a[j] * xs[j]; });
        if(i % 5 == 4)
            m.add_constraint(e == act);
        else if(i % 2 == 0)
            m.add_constraint(e <= act + 20.);
        else
            m.add_constraint(e >= act - 20.);
    }
    m.set_time_limit(std::chrono::duration<double>(10.));
    return xs;
}

/////////////////////////////////// sections /////////////////////////////////

static void section_P() {
    std::printf("=== P: library and parameter defaults\n");
    gurobi_lp model;
    N n(model);
    auto v = model.native_api().library_version();
    std::printf("  library=%s version=%d.%d.%d\n",
                model.native_api().library_path().c_str(), v ? v->major : -1,
                v ? v->minor : -1, v ? v->patch : -1);
    for(const char * p : {"Method", "LPWarmStart", "Presolve", "Crossover", "Threads",
                          "UpdateMode", "DualReductions", "SimplexPricing"}) {
        int val = -999;
        int e = n.G.getintparam(n.env, p, &val);
        std::printf("  param %-15s = %d (e%d)\n", p, val, e);
    }
    double il = 0; n.G.getdblparam(n.env, "IterationLimit", &il);
    std::printf("  param IterationLimit = %g\n", il);
}

static void core_variant(const char * label, int method, int presolve) {
    gurobi_lp model;
    N n(model);
    build_core(model);
    if(method != -99) n.iparam("Method", method);
    if(presolve != -99) n.iparam("Presolve", presolve);
    n.optimize();
    dump(n, label, core_cols, core_rows);
}

static void section_C() {
    std::printf("=== C: row side convention on the core LP\n");
    core_variant("C.default", -99, -99);
    core_variant("C.primal_nopresolve", 0, 0);
    core_variant("C.dual_nopresolve", 1, 0);
    core_variant("C.dual_presolve", 1, -99);
    core_variant("C.barrier_crossover_nopresolve", 2, 0);
    // minimisation of the negated objective: same basis expected
    {
        gurobi_lp model;
        N n(model);
        auto c = build_core(model);
        model.set_minimization();
        model.set_objective(-2 * c.x - 3 * c.y - c.u + c.v - c.t);
        n.iparam("Presolve", 0);
        n.optimize();
        dump(n, "C.min_negated_nopresolve", core_cols, core_rows);
    }
}

static void section_D() {
    std::printf("=== D: availability\n");
    {
        gurobi_lp model;
        N n(model);
        build_core(model);
        std::printf("  D.before_solve, no update:\n");
        print_basis(n, read_basis(n, 9, 5), "D.before_solve_noupdate(len 9/5)");
        n.update();
        print_basis(n, read_basis(n), "D.before_solve_updated");
    }
    // infeasible, presolve-detectable
    for(int pre : {-99, 0}) {
        gurobi_lp model;
        N n(model);
        auto x = model.add_variable({.lower_bound = 0., .upper_bound = 10.});
        auto y = model.add_variable({.lower_bound = 0., .upper_bound = 10.});
        model.set_maximization();
        model.set_objective(x + y);
        model.add_constraint(x + y <= -1.);
        model.add_constraint(x - y <= 3.);
        if(pre != -99) n.iparam("Presolve", pre);
        n.dparam("TimeLimit", 10.);
        n.optimize();
        std::printf("  D.infeasible_trivial presolve=%d\n", pre);
        show_status(n, "status");
        print_basis(n, read_basis(n), "basis");
    }
    // infeasible, not presolve-detectable (the backend test's depot/store conflict)
    for(auto [pre, meth] : {std::pair{-99, -99}, std::pair{0, 1}, std::pair{0, 0}}) {
        gurobi_lp model;
        N n(model);
        const auto sites = std::views::iota(0, 3);
        auto ship = model.add_variables(9, [](int i, int j) { return 3 * i + j; });
        for(int i : sites)
            for(int j : sites)
                model.set_variable_upper_bound(ship(i, j), 3 + (i + 2 * j) % 4);
        for(int i : sites)
            model.add_constraint(xsum(sites, [&, i](int j) { return ship(i, j); }) <= 8.);
        for(int j : sites)
            model.add_constraint(xsum(sites, [&, j](int i) { return ship(i, j); }) >= 9.);
        if(pre != -99) n.iparam("Presolve", pre);
        if(meth != -99) n.iparam("Method", meth);
        n.dparam("TimeLimit", 10.);
        n.optimize();
        std::printf("  D.infeasible_depot presolve=%d method=%d\n", pre, meth);
        show_status(n, "status");
        print_basis(n, read_basis(n), "basis");
    }
    // unbounded
    for(auto [pre, meth] : {std::pair{-99, -99}, std::pair{0, 0}, std::pair{0, 1}}) {
        gurobi_lp model;
        N n(model);
        auto x = model.add_variable();
        auto y = model.add_variable();
        auto z = model.add_variable({.lower_bound = 0., .upper_bound = 4.});
        model.set_maximization();
        model.set_objective(x + y + z);
        model.add_constraint(x - y <= 1.);
        model.add_constraint(x + z <= 7.);
        model.add_constraint(y - 2 * z >= -3.);
        if(pre != -99) n.iparam("Presolve", pre);
        if(meth != -99) n.iparam("Method", meth);
        n.dparam("TimeLimit", 10.);
        n.optimize();
        std::printf("  D.unbounded presolve=%d method=%d\n", pre, meth);
        show_status(n, "status");
        print_basis(n, read_basis(n), "basis");
        n.iparam("DualReductions", 0);
        n.optimize();
        show_status(n, "status after DualReductions=0 re-solve");
        print_basis(n, read_basis(n), "basis");
    }
    // iteration limit stop
    for(auto [pre, meth, lim] : {std::tuple{-99, -99, 1.}, std::tuple{0, 1, 1.},
                                 std::tuple{0, 0, 5.}, std::tuple{-99, 2, 3.}}) {
        gurobi_lp model;
        N n(model);
        build_medium(model);
        if(pre != -99) n.iparam("Presolve", pre);
        if(meth != -99) n.iparam("Method", meth);
        n.dparam("IterationLimit", lim);
        n.optimize();
        std::printf("  D.iteration_limit presolve=%d method=%d limit=%g\n", pre, meth, lim);
        show_status(n, "status");
        print_basis(n, read_basis(n), "basis");
    }
    // barrier stopped by BarIterLimit
    {
        gurobi_lp model;
        N n(model);
        build_medium(model);
        n.iparam("Method", 2);
        n.iparam("BarIterLimit", 2);
        n.optimize();
        std::printf("  D.bar_iter_limit method=2 BarIterLimit=2\n");
        show_status(n, "status");
        print_basis(n, read_basis(n), "basis");
    }
    // time limit 0
    {
        gurobi_lp model;
        N n(model);
        build_medium(model);
        n.dparam("TimeLimit", 0.);
        n.optimize();
        std::printf("  D.time_limit_0\n");
        show_status(n, "status");
        print_basis(n, read_basis(n), "basis");
    }
    // barrier without crossover, barrier with crossover, PDHG (13+)
    for(auto [meth, cross] : {std::pair{2, 0}, std::pair{2, -99}, std::pair{6, 0},
                              std::pair{6, -99}}) {
        gurobi_lp model;
        N n(model);
        build_medium(model);
        int e = n.iparam("Method", meth);
        if(e) { std::printf("  D.method=%d unavailable on this release\n", meth); continue; }
        if(cross != -99) n.iparam("Crossover", cross);
        n.optimize();
        std::printf("  D.method=%d crossover=%d\n", meth, cross);
        show_status(n, "status");
        print_basis(n, read_basis(n), "basis");
    }
    // the wrapper's default on the medium LP: #basic == #rows ?
    {
        gurobi_lp model;
        N n(model);
        build_medium(model);
        model.solve();
        std::printf("  D.wrapper_solve_medium\n");
        show_status(n, "status");
        print_basis(n, read_basis(n), "basis");
    }
    // modifications after an optimal solve
    {
        gurobi_lp model;
        N n(model);
        auto c = build_core(model);
        n.optimize();
        show_status(n, "D.mod base solve");
        model.set_variable_upper_bound(c.x, 9.);
        print_basis(n, read_basis(n), "D.mod ub(x)=9 queued, no update");
        double xv = NAN; int ex = n.G.getdblattrelement(n.m, "X", 0, &xv);
        std::printf("    X[x] read -> e%d %g\n", ex, xv);
        n.update();
        print_basis(n, read_basis(n), "D.mod ub(x)=9 after GRBupdatemodel");
        ex = n.G.getdblattrelement(n.m, "X", 0, &xv);
        std::printf("    X[x] read -> e%d %g\n", ex, xv);

        n.optimize();
        show_status(n, "D.mod re-solve");
        (void)model.get_variable_upper_bound(c.x);  // wrapper getter: update, nothing pending
        print_basis(n, read_basis(n), "D.mod after wrapper getter (no pending change)");
        model.add_constraint(c.x + c.w <= 50.);
        print_basis(n, read_basis(n, 9, 5), "D.mod add row queued, len 9/5, no update");
        print_basis(n, read_basis(n, 9, 6), "D.mod add row queued, len 9/6, no update");
        n.update();
        print_basis(n, read_basis(n), "D.mod add row after update");

        n.optimize();
        show_status(n, "D.mod re-solve 2");
        model.add_variable({.lower_bound = 0.});
        print_basis(n, read_basis(n, 9, 6), "D.mod add column queued, len 9/6, no update");
        print_basis(n, read_basis(n, 10, 6), "D.mod add column queued, len 10/6, no update");
        n.update();
        print_basis(n, read_basis(n), "D.mod add column after update");

        n.optimize();
        show_status(n, "D.mod re-solve 3");
        model.set_objective_coefficient(c.x, 2.5);
        n.update();
        print_basis(n, read_basis(n), "D.mod obj coef change after update");
        n.optimize();
        model.set_constraint_rhs(c.r_le, 6.5);
        n.update();
        print_basis(n, read_basis(n), "D.mod rhs change after update");
        n.optimize();
        n.iparam("Method", 1);
        print_basis(n, read_basis(n), "D.mod param change (Method=1)");
    }
}

static basis solved_basis_core(int presolve, int method) {
    gurobi_lp model;
    N n(model);
    build_core(model);
    if(presolve != -99) n.iparam("Presolve", presolve);
    if(method != -99) n.iparam("Method", method);
    n.optimize();
    return read_basis(n);
}

// fresh medium model, basis b written, parameters, solve
static void warm_medium(const basis * b, const char * label, int method, int presolve,
                        int lpwarm, double iterlimit) {
    gurobi_lp model;
    N n(model);
    build_medium(model);
    if(method != -99) n.iparam("Method", method);
    if(presolve != -99) n.iparam("Presolve", presolve);
    if(lpwarm != -99) n.iparam("LPWarmStart", lpwarm);
    if(iterlimit >= 0) n.dparam("IterationLimit", iterlimit);
    if(b) write_basis(n, b->v, b->c, label);
    n.optimize();
    show_status(n, label);
}

static void section_E() {
    std::printf("=== E: warm start\n");
    basis bA;
    int first_basic = -1;
    {
        gurobi_lp model;
        N n(model);
        build_medium(model);
        model.solve();  // the wrapper's own solve
        show_status(n, "E.A cold solve (wrapper defaults)");
        bA = read_basis(n);
        print_basis(n, bA, "E.A basis");
        for(std::size_t j = 0; j < bA.v.size(); ++j)
            if(bA.v[j] == 0) { first_basic = (int)j; break; }
        double xv = NAN; n.G.getdblattrelement(n.m, "X", first_basic, &xv);
        std::printf("  first basic column %d, x=%g\n", first_basic, xv);
    }
    std::printf("  -- cold controls, IterationLimit=0\n");
    warm_medium(nullptr, "E.cold default lim0", -99, -99, -99, 0.);
    warm_medium(nullptr, "E.cold presolve0 lim0", -99, 0, -99, 0.);
    std::printf("  -- warm, IterationLimit=0\n");
    warm_medium(&bA, "E.warm default lim0", -99, -99, -99, 0.);
    warm_medium(&bA, "E.warm Method=0 lim0", 0, -99, -99, 0.);
    warm_medium(&bA, "E.warm Method=1 lim0", 1, -99, -99, 0.);
    warm_medium(&bA, "E.warm Method=2 lim0", 2, -99, -99, 0.);
    warm_medium(&bA, "E.warm Method=3 lim0", 3, -99, -99, 0.);
    warm_medium(&bA, "E.warm Presolve=0 lim0", -99, 0, -99, 0.);
    warm_medium(&bA, "E.warm LPWarmStart=0 lim0", -99, -99, 0, 0.);
    warm_medium(&bA, "E.warm LPWarmStart=1 lim0", -99, -99, 1, 0.);
    warm_medium(&bA, "E.warm LPWarmStart=2 lim0", -99, -99, 2, 0.);
    warm_medium(&bA, "E.warm LPWarmStart=2 Presolve=2 lim0", -99, 2, 2, 0.);
    warm_medium(&bA, "E.warm LPWarmStart=2 Presolve=0 lim0", -99, 0, 2, 0.);
    std::printf("  -- warm, no iteration limit\n");
    warm_medium(&bA, "E.warm default nolim", -99, -99, -99, -1.);
    warm_medium(&bA, "E.warm LPWarmStart=0 nolim", -99, -99, 0, -1.);
    warm_medium(&bA, "E.warm LPWarmStart=2 nolim", -99, -99, 2, -1.);

    std::printf("  -- read back of a written basis on a fresh model\n");
    {
        gurobi_lp model;
        N n(model);
        build_medium(model);
        n.update();
        write_basis(n, bA.v, bA.c, "E.readback");
        print_basis(n, read_basis(n), "E.readback before update");
        n.update();
        print_basis(n, read_basis(n), "E.readback after update");
        n.dparam("IterationLimit", 0.);
        n.optimize();
        show_status(n, "E.readback solve lim0");
        basis after = read_basis(n);
        std::printf("  E.readback basis after warm solve equals written: %s\n",
                    (after.v == bA.v && after.c == bA.c) ? "yes" : "no");
    }

    std::printf("  -- bound change interplay (column %d loosened to ub 20)\n", first_basic);
    {   // (i) write, then modify, then solve
        gurobi_lp model;
        N n(model);
        auto xs = build_medium(model);
        n.dparam("IterationLimit", 0.);
        write_basis(n, bA.v, bA.c, "E.i");
        model.set_variable_upper_bound(xs[first_basic], 20.);
        n.optimize();
        show_status(n, "E.i fresh: write basis, then loosen bound, solve lim0");
    }
    {   // (ii) modify, then write, then solve
        gurobi_lp model;
        N n(model);
        auto xs = build_medium(model);
        n.dparam("IterationLimit", 0.);
        model.set_variable_upper_bound(xs[first_basic], 20.);
        write_basis(n, bA.v, bA.c, "E.ii");
        n.optimize();
        show_status(n, "E.ii fresh: loosen bound, then write basis, solve lim0");
    }
    {   // (iii)-(v) on the solved model A itself
        gurobi_lp model;
        N n(model);
        auto xs = build_medium(model);
        n.optimize();
        show_status(n, "E.A solved");
        std::vector<int> slack_v(bA.v.size(), -1), slack_c(bA.c.size(), 0);
        n.dparam("IterationLimit", 0.);
        // (iv) slack basis written on the solved, unmodified model
        write_basis(n, slack_v, slack_c, "E.iv");
        n.optimize();
        show_status(n, "E.iv solved A: write all-slack basis, solve lim0");
        // restore optimum
        n.dparam("IterationLimit", 1e100);
        n.optimize();
        show_status(n, "E.A re-solved");
        n.dparam("IterationLimit", 0.);
        model.set_variable_upper_bound(xs[first_basic], 20.);
        write_basis(n, slack_v, slack_c, "E.iii");
        n.optimize();
        show_status(n, "E.iii solved A: loosen bound, write all-slack basis, solve lim0");
        n.dparam("IterationLimit", 1e100);
        n.optimize();
        show_status(n, "E.A re-solved");
        n.dparam("IterationLimit", 0.);
        model.set_variable_upper_bound(xs[first_basic], 25.);
        write_basis(n, bA.v, bA.c, "E.v");
        n.optimize();
        show_status(n, "E.v solved A: loosen bound, write A's optimal basis, solve lim0");
        n.dparam("IterationLimit", 1e100);
        n.optimize();
        model.set_variable_upper_bound(xs[first_basic], 30.);
        n.dparam("IterationLimit", 0.);
        n.optimize();
        show_status(n, "E.vi solved A: loosen bound, nothing written, solve lim0 (internal)");
    }

    std::printf("  -- invalid bases on the core LP (reference basis: Presolve=0 Method=1)\n");
    basis bc = solved_basis_core(0, 1);
    {
        gurobi_lp model;
        N n(model);
        build_core(model);
        n.update();
        print_basis(n, bc, "E.core reference basis");
    }
    for(auto [pre, lim, label] : {std::tuple{-99, 0., "cold core default lim0"},
                                  std::tuple{0, 0., "cold core presolve0 lim0"}}) {
        gurobi_lp model;
        N n(model);
        build_core(model);
        if(pre != -99) n.iparam("Presolve", pre);
        n.dparam("IterationLimit", lim);
        n.optimize();
        show_status(n, label);
    }
    struct tweak {
        const char * name;
        bool row;
        int index;
        int code;
    };
    // core columns: 0 x 1 y 2 w 3 u 4 f 5 v 6 t 7 z1 8 z2; rows 0 le 1 ge 2 eq 3 slack 4 geslack
    const std::vector<std::vector<tweak>> cases = {
        {},
        {{"too few basic: x basic -> -1", false, 0, -1}},
        {{"too many basic: u -> 0", false, 3, 0}},
        {{"-2 on w (ub = +inf)", false, 2, -2}},
        {{"-1 on z2 (free, empty column)", false, 8, -1}},
        {{"-2 on z2 (free, empty column)", false, 8, -2}},
        {{"-3 on u (bounded, at ub)", false, 3, -3}},
        {{"-3 on v (bounded, at lb)", false, 5, -3}},
        {{"-1 on f (fixed)", false, 4, -1}},
        {{"-2 on f (fixed)", false, 4, -2}},
        {{"-2 on t (lb -inf, at ub): already", false, 6, -2}},
        {{"-1 on t (lb = -inf)", false, 6, -1}},
        {{"CBasis -2 on le row", true, 0, -2}},
        {{"CBasis -3 on le row", true, 0, -3}},
        {{"CBasis -2 on ge row", true, 1, -2}},
        {{"CBasis -2 on eq row", true, 2, -2}},
        {{"VBasis 1 on x (out of range)", false, 0, 1}},
        {{"VBasis -4 on u (out of range)", false, 3, -4}},
        {{"CBasis 1 on le (out of range)", true, 0, 1}},
        {{"singular: z2 (empty col) basic", false, 8, 0}, {"", true, 4, -1}},
        {{"valid swap, suboptimal: le row basic", true, 0, 0}, {"", false, 3, -1}},
    };
    for(const auto & cs : cases) {
        std::vector<int> v = bc.v, c = bc.c;
        std::string name = cs.empty() ? "unmodified reference" : cs[0].name;
        for(const auto & t : cs) (t.row ? c : v)[(std::size_t)t.index] = t.code;
        std::printf("  E.invalid: %s\n", name.c_str());
        for(auto [pre, lim, label] : {std::tuple{-99, 0., "    default lim0"},
                                      std::tuple{0, 0., "    presolve0 lim0"},
                                      std::tuple{-99, -1., "    default nolim"}}) {
            gurobi_lp model;
            N n(model);
            build_core(model);
            n.update();
            if(pre != -99) n.iparam("Presolve", pre);
            if(lim >= 0) n.dparam("IterationLimit", lim);
            int ew = write_basis(n, v, c, label);
            int eu = n.update();
            std::printf("%s: write->%d update->%d\n", label, ew, eu);
            basis rb = read_basis(n);
            print_basis(n, rb, "    read back before solve");
            n.optimize();
            show_status(n, label);
            print_basis(n, read_basis(n), "    basis after solve");
        }
    }
}

static void section_F() {
    std::printf("=== F: partial / extended bases\n");
    basis bc = solved_basis_core(0, 1);
    auto extended = [](gurobi_lp & model) {
        auto c = build_core(model);
        auto n1 = model.add_column({{c.r_slack, 1.}},
                                   {.obj_coef = -5., .lower_bound = 0.});
        auto rn = model.add_constraint(c.x + c.w <= 50.);
        (void)n1;
        (void)rn;
    };
    struct fcase {
        const char * name;
        std::function<void(N &)> write;
    };
    std::vector<int> v_full = bc.v, c_full = bc.c;
    v_full.push_back(-1);
    c_full.push_back(0);
    std::vector<int> v_toomany = bc.v, c_toomany = bc.c;
    v_toomany.push_back(0);
    c_toomany.push_back(0);
    const std::vector<fcase> cases = {
        {"cold (nothing written)", [](N &) {}},
        {"old-length arrays (9/5), new entries unset",
         [&](N & n) { write_basis(n, bc.v, bc.c, "F.a"); }},
        {"full arrays, new column -1, new row 0",
         [&](N & n) { write_basis(n, v_full, c_full, "F.b"); }},
        {"full arrays, new column 0, new row 0 (too many basic)",
         [&](N & n) { write_basis(n, v_toomany, c_toomany, "F.c"); }},
        {"VBasis only (full)", [&](N & n) { write_basis(n, v_full, {}, "F.d"); }},
        {"CBasis only (full)", [&](N & n) { write_basis(n, {}, c_full, "F.e"); }},
        {"one element: VBasis[x]=0",
         [&](N & n) {
             int e = n.G.setintattrelement(n.m, "VBasis", 0, 0);
             std::printf("    setintattrelement -> %d\n", e);
         }},
        {"every element one by one (= full arrays)",
         [&](N & n) {
             int e = 0;
             for(std::size_t j = 0; j < v_full.size(); ++j)
                 e |= n.G.setintattrelement(n.m, "VBasis", (int)j, v_full[j]);
             for(std::size_t i = 0; i < c_full.size(); ++i)
                 e |= n.G.setintattrelement(n.m, "CBasis", (int)i, c_full[i]);
             std::printf("    setintattrelement (all) -> %d\n", e);
         }},
    };
    for(const auto & cs : cases) {
        std::printf("  F: %s\n", cs.name);
        for(auto [pre, lim, label] : {std::tuple{-99, 0., "    default lim0"},
                                      std::tuple{0, 0., "    presolve0 lim0"},
                                      std::tuple{0, -1., "    presolve0 nolim"}}) {
            gurobi_lp model;
            N n(model);
            extended(model);
            n.update();
            if(pre != -99) n.iparam("Presolve", pre);
            if(lim >= 0) n.dparam("IterationLimit", lim);
            cs.write(n);
            n.update();
            print_basis(n, read_basis(n), "    read back before solve");
            n.optimize();
            show_status(n, label);
            print_basis(n, read_basis(n), "    basis after solve");
        }
    }
    {   // the solved model itself gains a column and a row
        gurobi_lp model;
        N n(model);
        auto c = build_core(model);
        n.iparam("Presolve", 0);
        n.optimize();
        show_status(n, "F.h base solve");
        model.add_column({{c.r_slack, 1.}}, {.obj_coef = -5., .lower_bound = 0.});
        model.add_constraint(c.x + c.w <= 50.);
        n.dparam("IterationLimit", 0.);
        n.optimize();
        show_status(n, "F.h solved model + column + row, nothing written, lim0");
        print_basis(n, read_basis(n), "F.h basis");
    }
    {   // the solved model gains a binding row: internal warm start vs iterations
        gurobi_lp model;
        N n(model);
        auto c = build_core(model);
        n.iparam("Presolve", 0);
        n.optimize();
        model.add_constraint(c.x + c.w <= 5.);
        n.optimize();
        show_status(n, "F.i solved model + binding row, nothing written, nolim");
        print_basis(n, read_basis(n), "F.i basis");
    }
}

static void section_G() {
    std::printf("=== G: gotchas\n");
    {   // G1: writing a basis on a solved model discards the solution?
        gurobi_lp model;
        N n(model);
        build_core(model);
        n.iparam("Presolve", 0);
        n.optimize();
        basis b = read_basis(n);
        write_basis(n, b.v, b.c, "G1");
        double xv = NAN, pv = NAN;
        int ex = n.G.getdblattrelement(n.m, "X", 0, &xv);
        int ep = n.G.getdblattrelement(n.m, "Pi", 0, &pv);
        int st = -1; n.G.getintattr(n.m, "Status", &st);
        std::printf("  G1 after VBasis/CBasis write, no update: X e%d %g, Pi e%d %g, Status %d\n",
                    ex, xv, ep, pv, st);
        print_basis(n, read_basis(n), "G1 basis, no update");
        n.update();
        ex = n.G.getdblattrelement(n.m, "X", 0, &xv);
        ep = n.G.getdblattrelement(n.m, "Pi", 0, &pv);
        n.G.getintattr(n.m, "Status", &st);
        std::printf("  G1 after update: X e%d %g, Pi e%d %g, Status %d\n", ex, xv, ep, pv, st);
        print_basis(n, read_basis(n), "G1 basis after update");
        try {
            auto sol = model.get_solution();
            std::printf("  G1 wrapper get_solution OK\n");
        } catch(const std::exception & e) {
            std::printf("  G1 wrapper get_solution throws: %s\n", e.what());
        }
        try {
            std::printf("  G1 wrapper get_solution_value = %g\n", model.get_solution_value());
        } catch(const std::exception & e) {
            std::printf("  G1 wrapper get_solution_value throws: %s\n", e.what());
        }
    }
    {   // G2: native ranged row adds a column
        for(int sense : {-1, 1}) {
            gurobi_lp model;
            N n(model);
            auto x = model.add_variable({.lower_bound = 0., .upper_bound = 10.});
            auto y = model.add_variable({.lower_bound = 0., .upper_bound = 10.});
            if(sense < 0) model.set_maximization(); else model.set_minimization();
            model.set_objective(x + 2 * y);
            model.add_constraint(x - y <= 1.);
            int ind[2] = {0, 1};
            double val[2] = {1., 1.};
            int e = n.G.addrangeconstr(n.m, 2, ind, val, 2., 5., "rng");
            std::printf("  G2 addrangeconstr -> %d; NumVars before update %d", e, n.nv());
            n.update();
            std::printf(", after update %d\n", n.nv());
            n.iparam("Presolve", 0);
            n.optimize();
            dump(n, sense < 0 ? "G2 ranged max (activity at upper 5)"
                              : "G2 ranged min (activity at lower 2)",
                 {"x", "y", "RgRange"}, {"x-y<=1", "rng[2,5]"});
        }
    }
    {   // G3: queued column: array length vs NumVars
        gurobi_lp model;
        N n(model);
        build_core(model);
        n.iparam("Presolve", 0);
        n.optimize();
        basis b = read_basis(n);
        model.add_variable({.lower_bound = 0.});
        std::vector<int> v = b.v;
        v.push_back(-1);
        int e = n.G.setintattrarray(n.m, "VBasis", 0, (int)v.size(), v.data());
        std::printf("  G3 queued column, set VBasis len %zu before update -> %d (%s)\n",
                    v.size(), e, e ? n.err() : "");
        n.update();
        e = n.G.setintattrarray(n.m, "VBasis", 0, (int)v.size(), v.data());
        std::printf("  G3 after update -> %d\n", e);
    }
    {   // G4: removing a variable after a solve (wrapper remaps ids)
        gurobi_lp model;
        N n(model);
        auto c = build_core(model);
        n.iparam("Presolve", 0);
        n.optimize();
        model.remove_variable(c.z1);
        print_basis(n, read_basis(n), "G4 after remove_variable(z1) (wrapper updates)");
        n.dparam("IterationLimit", 0.);
        n.optimize();
        show_status(n, "G4 re-solve lim0");
        print_basis(n, read_basis(n), "G4 basis");
        std::printf("  G4 native_id(t)=%d native_id(z2)=%d\n", model.native_id(c.t),
                    model.native_id(c.z2));
    }
    {   // G5: VBasis on a MIP-typed column of an LP model is not reachable through gurobi_lp;
        // check the attribute after a solve of the same model typed as MIP natively
        gurobi_lp model;
        N n(model);
        build_core(model);
        n.G.setcharattrelement(n.m, "VType", 0, 'I');
        n.optimize();
        show_status(n, "G5 core with x integer (native VType write)");
        print_basis(n, read_basis(n), "G5 basis after MIP solve");
        int e = n.G.setintattrelement(n.m, "VBasis", 0, 0);
        std::printf("  G5 set VBasis on MIP -> %d\n", e);
    }
}

// core without the two free columns: max 2x + 3y + u - v + t
static void build_core_nofree(gurobi_lp & m) {
    auto x = m.add_variable({.lower_bound = 0., .upper_bound = 10.});
    auto y = m.add_variable({.lower_bound = 0., .upper_bound = 10.});
    auto w = m.add_variable({.lower_bound = 0.});
    auto u = m.add_variable({.lower_bound = 0., .upper_bound = 1.});
    auto f = m.add_variable({.lower_bound = 3., .upper_bound = 3.});
    auto v = m.add_variable({.lower_bound = 1., .upper_bound = 5.});
    auto t = m.add_variable({.lower_bound = std::nullopt, .upper_bound = 3.});
    m.set_maximization();
    m.set_objective(2 * x + 3 * y + u - v + t);
    m.add_constraint(x + y <= 6.);
    m.add_constraint(x - y >= -2.);
    m.add_constraint(y - w == 0.);
    m.add_constraint(x + y + u + f + v + t <= 100.);
    m.add_constraint(x + y >= 1.);
    m.set_time_limit(std::chrono::duration<double>(10.));
}

template <typename Build>
static void warm_case(const char * label, Build build, const basis & b, bool update_first,
                      int presolve, double lim, int method = -99, bool show = false) {
    gurobi_lp model;
    N n(model);
    build(model);
    if(update_first) n.update();
    if(presolve != -99) n.iparam("Presolve", presolve);
    if(method != -99) n.iparam("Method", method);
    if(lim >= 0) n.dparam("IterationLimit", lim);
    write_basis(n, b.v, b.c, label);
    n.update();
    basis rb = read_basis(n);
    std::printf("  %s: read back after update e%d/e%d%s\n", label, rb.ev, rb.ec,
                (!rb.ev && !rb.ec) ? ((rb.v == b.v && rb.c == b.c) ? " (== written)" : " (DIFFERS)") : "");
    n.optimize();
    if(show) {
        std::vector<std::string> none;
        dump(n, label, none, none);
    } else
        show_status(n, label);
}

static void section_H() {
    std::printf("=== H: warm start clarifications\n");
    auto core = [](gurobi_lp & m) { build_core(m); };
    auto nofree = [](gurobi_lp & m) { build_core_nofree(m); };
    auto medium = [](gurobi_lp & m) { build_medium(m); };
    basis bc = solved_basis_core(0, 1);
    basis bcx = solved_basis_core(0, 2);  // barrier + crossover: z1 superbasic
    basis bnf;
    {
        gurobi_lp model; N n(model); build_core_nofree(model); n.iparam("Presolve", 0);
        n.iparam("Method", 1); n.optimize(); bnf = read_basis(n);
        print_basis(n, bnf, "H nofree reference");
        print_basis(n, bc, "H core reference (dual)");
        print_basis(n, bcx, "H core reference (barrier+crossover)");
    }
    basis bm;
    {
        gurobi_lp model; N n(model); build_medium(model); n.optimize(); bm = read_basis(n);
    }
    std::printf("  -- cold controls, Presolve=0, no limit\n");
    for(int meth : {-99, 0, 1}) {
        gurobi_lp model; N n(model); build_core(model); n.iparam("Presolve", 0);
        if(meth != -99) n.iparam("Method", meth);
        n.optimize(); std::printf("  method %d ", meth); show_status(n, "cold core presolve0");
    }
    {
        gurobi_lp model; N n(model); build_core_nofree(model); n.iparam("Presolve", 0);
        n.optimize(); show_status(n, "cold nofree presolve0");
    }
    std::printf("  -- warm core / nofree / medium\n");
    warm_case("H1 core ref presolve0 lim0", core, bc, true, 0, 0., -99, true);
    warm_case("H1 core ref presolve0 lim1", core, bc, true, 0, 1.);
    warm_case("H1 core ref presolve0 nolim", core, bc, true, 0, -1.);
    warm_case("H1 core ref default nolim", core, bc, true, -99, -1.);
    warm_case("H1 core ref presolve0 nolim Method=0", core, bc, true, 0, -1., 0);
    warm_case("H1 core ref presolve0 nolim Method=1", core, bc, true, 0, -1., 1);
    warm_case("H2 core barrier-basis presolve0 lim0", core, bcx, true, 0, 0., -99, true);
    warm_case("H2 core barrier-basis presolve0 nolim", core, bcx, true, 0, -1.);
    warm_case("H3 nofree ref presolve0 lim0", nofree, bnf, true, 0, 0., -99, true);
    warm_case("H3 nofree ref default lim0", nofree, bnf, true, -99, 0.);
    warm_case("H3 nofree ref presolve0 lim1", nofree, bnf, true, 0, 1.);
    warm_case("H3 nofree ref presolve0 nolim", nofree, bnf, true, 0, -1.);
    warm_case("H4 medium, write before first update, nolim", medium, bm, false, -99, -1.);
    warm_case("H4 medium, write before first update, lim0", medium, bm, false, -99, 0.);
    warm_case("H5 medium, update first, nolim", medium, bm, true, -99, -1.);
    warm_case("H5 medium, update first, lim0", medium, bm, true, -99, 0.);
    warm_case("H5 medium, update first, lim1", medium, bm, true, -99, 1.);
    warm_case("H5 medium, update first, lim0 Method=0", medium, bm, true, -99, 0., 0);
    warm_case("H5 medium, update first, lim0 Method=1", medium, bm, true, -99, 0., 1);
    warm_case("H5 medium, update first, lim0 Method=2", medium, bm, true, -99, 0., 2);
    warm_case("H5 medium, update first, nolim Method=2", medium, bm, true, -99, -1., 2);
    warm_case("H5 medium, update first, nolim Method=3", medium, bm, true, -99, -1., 3);
    {
        gurobi_lp model; N n(model); build_medium(model); n.update();
        n.iparam("LPWarmStart", 0);
        write_basis(n, bm.v, bm.c, "H5 LPWarmStart=0");
        n.optimize(); show_status(n, "H5 medium, update first, LPWarmStart=0, nolim");
    }
    {
        gurobi_lp model; N n(model); build_medium(model); n.update();
        n.iparam("LPWarmStart", 2);
        write_basis(n, bm.v, bm.c, "H5 LPWarmStart=2");
        n.optimize(); show_status(n, "H5 medium, update first, LPWarmStart=2, nolim");
    }
    {   // X after an ITERATION_LIMIT stop that holds the optimal basis
        gurobi_lp model; N n(model); build_medium(model); n.update();
        n.dparam("IterationLimit", 0.);
        write_basis(n, bm.v, bm.c, "H6");
        model.solve(); show_status(n, "H6 warm lim0 (wrapper solve)");
        std::vector<double> x(60), pi(40);
        int ex = n.G.getdblattrarray(n.m, "X", 0, 60, x.data());
        int ep = n.G.getdblattrarray(n.m, "Pi", 0, 40, pi.data());
        std::printf("  H6 X e%d x[3]=%g, Pi e%d\n", ex, x[3], ep);
        auto st = model.get_status();
        std::printf("  H6 wrapper status index %zu (iteration_limit has_solution? %d)\n", st.index(),
                    std::holds_alternative<status::iteration_limit>(st)
                        ? (int)std::get<status::iteration_limit>(st).solution_available : -1);
    }
    std::printf("  -- write on a solved model\n");
    {
        gurobi_lp model; N n(model); auto xs = build_medium(model); n.optimize();
        show_status(n, "H7 A solved");
        std::vector<int> sv(bm.v.size(), -1), sc(bm.c.size(), 0);
        write_basis(n, sv, sc, "H7");
        print_basis(n, read_basis(n), "H7 slack basis written, no update");
        n.update();
        print_basis(n, read_basis(n), "H7 after update");
        double xv; int ex = n.G.getdblattrelement(n.m, "X", 3, &xv);
        int st; n.G.getintattr(n.m, "Status", &st);
        std::printf("  H7 X e%d, Status %d\n", ex, st);
        n.dparam("IterationLimit", 0.);
        n.optimize(); show_status(n, "H7 solve lim0");
        print_basis(n, read_basis(n), "H7 basis after");
        n.dparam("IterationLimit", 1e100);
        n.optimize(); show_status(n, "H7 solve nolim");
    }
    {
        gurobi_lp model; N n(model); auto xs = build_medium(model); n.optimize();
        std::vector<int> sv(bm.v.size(), -1), sc(bm.c.size(), 0);
        model.set_variable_upper_bound(xs[3], 20.);
        write_basis(n, sv, sc, "H8");
        n.optimize(); show_status(n, "H8 A solved, loosen bound, write slack basis, nolim");
    }
    {
        gurobi_lp model; N n(model); auto xs = build_medium(model); n.optimize();
        std::vector<int> sv(bm.v.size(), -1), sc(bm.c.size(), 0);
        write_basis(n, sv, sc, "H9");
        model.set_variable_upper_bound(xs[3], 20.);
        n.optimize(); show_status(n, "H9 A solved, write slack basis, loosen bound, nolim");
    }
    {
        gurobi_lp model; N n(model); auto xs = build_medium(model); n.optimize();
        model.set_variable_upper_bound(xs[3], 20.);
        n.optimize(); show_status(n, "H10 A solved, loosen bound, nothing written, nolim");
    }
    std::printf("  -- write then modify on a fresh model (update first)\n");
    {
        gurobi_lp model; N n(model); auto xs = build_medium(model); n.update();
        write_basis(n, bm.v, bm.c, "H11");
        model.set_variable_upper_bound(xs[3], 20.);
        n.optimize(); show_status(n, "H11 fresh: write, loosen bound (same update), nolim");
    }
    {
        gurobi_lp model; N n(model); auto xs = build_medium(model); n.update();
        write_basis(n, bm.v, bm.c, "H12");
        n.update();
        model.set_variable_upper_bound(xs[3], 20.);
        n.optimize(); show_status(n, "H12 fresh: write, update, loosen bound, nolim");
    }
    {
        gurobi_lp model; N n(model); auto xs = build_medium(model); n.update();
        model.set_variable_upper_bound(xs[3], 20.);
        write_basis(n, bm.v, bm.c, "H13");
        n.optimize(); show_status(n, "H13 fresh: loosen bound, write (same update), nolim");
    }
    {
        gurobi_lp model; N n(model); auto xs = build_medium(model); n.update();
        model.set_variable_upper_bound(xs[3], 20.);
        n.update();
        write_basis(n, bm.v, bm.c, "H14");
        n.optimize(); show_status(n, "H14 fresh: loosen bound, update, write, nolim");
    }
    {
        gurobi_lp model; N n(model); auto xs = build_medium(model); n.update();
        write_basis(n, bm.v, bm.c, "H15");
        n.update();
        model.add_constraint(xs[0] + xs[1] <= 1000.);
        n.update();
        print_basis(n, read_basis(n), "H15 fresh: write, update, add row, update: read");
        n.optimize(); show_status(n, "H15 nolim");
    }
}

#include <dlfcn.h>
using grbreset_t = int(g::GRBmodel *, int);
static grbreset_t * load_reset(const gurobi_api & api) {
    void * h = dlopen(api.library_path().c_str(), RTLD_NOW | RTLD_NOLOAD);
    if(!h) h = dlopen(api.library_path().c_str(), RTLD_NOW);
    return h ? reinterpret_cast<grbreset_t *>(dlsym(h, "GRBreset")) : nullptr;
}

static void section_I() {
    std::printf("=== I: written basis on a model that was solved before\n");
    basis bm, b5;
    {
        gurobi_lp model; N n(model); build_medium(model); n.optimize(); bm = read_basis(n);
    }
    {
        gurobi_lp model; N n(model); build_medium(model); n.iparam("Presolve", 0);
        n.iparam("Method", 0); n.dparam("IterationLimit", 5.); n.optimize(); b5 = read_basis(n);
        show_status(n, "I suboptimal basis source (primal, 5 iters)");
    }
    std::vector<int> sv(bm.v.size(), -1), sc(bm.c.size(), 0);
    {
        gurobi_lp model; N n(model); build_medium(model); n.update();
        write_basis(n, b5.v, b5.c, "I18");
        n.optimize(); show_status(n, "I18 fresh: suboptimal basis written, nolim (baseline)");
    }
    {
        gurobi_lp model; N n(model); build_medium(model); n.update();
        write_basis(n, sv, sc, "I18b");
        n.optimize(); show_status(n, "I18b fresh: slack basis written, nolim (baseline)");
    }
    {
        gurobi_lp model; N n(model); build_medium(model); n.optimize();
        write_basis(n, b5.v, b5.c, "I16");
        n.optimize(); show_status(n, "I16 solved A, unmodified: suboptimal basis written, nolim");
        print_basis(n, read_basis(n), "I16 basis after");
    }
    {
        gurobi_lp model; N n(model); auto xs = build_medium(model); n.optimize();
        model.set_objective_coefficient(xs[0], 1.5);
        write_basis(n, sv, sc, "I17");
        n.optimize(); show_status(n, "I17 solved A, obj coef change, slack basis written, nolim");
    }
    {
        gurobi_lp model; N n(model); auto xs = build_medium(model); n.optimize();
        model.set_objective_coefficient(xs[0], 1.5);
        n.optimize(); show_status(n, "I17b solved A, obj coef change, nothing written, nolim");
    }
    {
        gurobi_lp model; N n(model); auto xs = build_medium(model); n.optimize();
        model.set_objective_coefficient(xs[0], 1.5);
        n.update();
        write_basis(n, sv, sc, "I17c");
        n.optimize(); show_status(n, "I17c solved A, obj coef change, update, slack basis written, nolim");
    }
    {
        gurobi_lp model; N n(model); auto xs = build_medium(model); n.optimize();
        n.update();
        write_basis(n, sv, sc, "I23");
        n.update();
        model.set_variable_upper_bound(xs[3], 20.);
        n.optimize(); show_status(n, "I23 solved A, write slack, update, loosen bound, nolim");
    }
    {
        gurobi_lp model; N n(model); auto xs = build_medium(model); n.optimize();
        model.set_variable_upper_bound(xs[3], 20.);
        n.update();
        write_basis(n, sv, sc, "I24");
        n.optimize(); show_status(n, "I24 solved A, loosen bound, update, write slack, nolim");
    }
    {
        gurobi_lp model; N n(model); auto xs = build_medium(model); n.optimize();
        model.set_variable_upper_bound(xs[3], 20.);
        n.update();
        write_basis(n, sv, sc, "I24b");
        n.update();
        n.optimize(); show_status(n, "I24b solved A, loosen bound, update, write slack, update, nolim");
    }
    {
        gurobi_lp model; N n(model); build_medium(model); n.optimize();
        n.update();
        write_basis(n, sv, sc, "I26");
        n.update();
        n.optimize(); show_status(n, "I26 solved A unmodified, update, write slack, update, nolim");
        double xv; int ex = n.G.getdblattrelement(n.m, "X", 3, &xv);
        basis after = read_basis(n);
        std::printf("  I26 X e%d; basis after == slack written: %s\n", ex,
                    (after.v == sv && after.c == sc) ? "yes" : "no");
    }
    grbreset_t * reset = nullptr;
    {
        gurobi_lp model; N n(model); reset = load_reset(model.native_api());
        std::printf("  GRBreset symbol %s\n", reset ? "found" : "NOT found");
    }
    if(reset) {
        {
            gurobi_lp model; N n(model); build_medium(model); n.optimize();
            int e = reset(n.m, 0);
            std::printf("  I19 GRBreset(0) -> %d\n", e);
            print_basis(n, read_basis(n), "I19 basis after reset");
            write_basis(n, sv, sc, "I19");
            n.optimize(); show_status(n, "I19 solved A, GRBreset(0), slack basis written, nolim");
        }
        {
            gurobi_lp model; N n(model); build_medium(model); n.optimize();
            reset(n.m, 0);
            write_basis(n, b5.v, b5.c, "I20");
            n.optimize(); show_status(n, "I20 solved A, GRBreset(0), suboptimal basis written, nolim");
        }
        {
            gurobi_lp model; N n(model); build_medium(model); n.optimize();
            reset(n.m, 0);
            n.optimize(); show_status(n, "I21 solved A, GRBreset(0), nothing written, nolim");
        }
        {
            gurobi_lp model; N n(model); build_medium(model); n.optimize();
            write_basis(n, b5.v, b5.c, "I22");
            int e = reset(n.m, 0);
            std::printf("  I22 reset after write -> %d\n", e);
            print_basis(n, read_basis(n), "I22 basis after write+reset");
            n.optimize(); show_status(n, "I22 solved A, write suboptimal, GRBreset(0), nolim");
        }
        {
            gurobi_lp model; N n(model); build_medium(model); n.optimize();
            n.update(); reset(n.m, 0);
            write_basis(n, bm.v, bm.c, "I25");
            n.dparam("IterationLimit", 1.);
            n.optimize(); show_status(n, "I25 solved A, update, GRBreset(0), optimal basis written, lim1");
            basis after = read_basis(n);
            std::printf("  I25 basis after == written: %s\n", (after.v == bm.v && after.c == bm.c) ? "yes" : "no");
        }
    }
}

static void section_J() {
    std::printf("=== J: the update + GRBreset(0) + write recipe\n");
    basis bm;
    { gurobi_lp model; N n(model); build_medium(model); n.optimize(); bm = read_basis(n); }
    grbreset_t * reset = nullptr;
    { gurobi_lp model; reset = load_reset(model.native_api()); }
    if(!reset) { std::printf("  GRBreset missing\n"); return; }
    auto recipe = [&](N & n, const basis & b, const char * label) {
        n.update();
        int e = reset(n.m, 0);
        if(e) std::printf("    reset -> %d\n", e);
        write_basis(n, b.v, b.c, label);
    };
    {
        gurobi_lp model; N n(model); auto xs = build_medium(model); n.optimize();
        recipe(n, bm, "J1");
        auto c = model.add_column({{con_t(0), 1.}}, {.obj_coef = -100., .lower_bound = 0.});
        model.add_constraint(xs[0] + xs[1] <= 1000.);
        (void)c;
        n.optimize(); show_status(n, "J1 solved A, recipe(optimal), then add column + loose row, nolim");
    }
    {
        gurobi_lp model; N n(model); auto xs = build_medium(model); n.optimize();
        recipe(n, bm, "J2");
        model.set_variable_upper_bound(xs[3], 20.);
        n.optimize(); show_status(n, "J2 solved A, recipe(optimal), then loosen bound, nolim");
    }
    {
        gurobi_lp model; N n(model); auto xs = build_medium(model); n.optimize();
        std::vector<int> sv(bm.v.size(), -1), sc(bm.c.size(), 0);
        recipe(n, basis{0, 0, sv, sc}, "J3");
        model.set_variable_upper_bound(xs[3], 20.);
        n.optimize(); show_status(n, "J3 solved A, recipe(slack), then loosen bound, nolim");
    }
    {
        gurobi_lp model; N n(model); build_medium(model);
        recipe(n, bm, "J4");
        n.dparam("IterationLimit", 0.);
        model.solve(); show_status(n, "J4 fresh, recipe(optimal), wrapper solve lim0");
        n.dparam("IterationLimit", 1.);
        recipe(n, bm, "J4b");
        model.solve(); show_status(n, "J4b same model, recipe(optimal), wrapper solve lim1");
        double xv; int ex = n.G.getdblattrelement(n.m, "X", 3, &xv);
        std::printf("  J4b X e%d\n", ex);
    }
    {
        gurobi_lp model; N n(model); build_medium(model);
        model.add_variable({.lower_bound = 0.});  // queued
        int e = reset(n.m, 0);
        std::printf("  J5 reset with a queued column -> %d, NumVars after reset %d (expected 61 if it flushes)\n",
                    e, n.nv());
    }
    {
        gurobi_lp model; N n(model); build_medium(model); n.optimize();
        recipe(n, bm, "J6");
        double xv; int ex = n.G.getdblattrelement(n.m, "X", 3, &xv);
        int st = -1; n.G.getintattr(n.m, "Status", &st);
        std::printf("  J6 after recipe on a solved model: X e%d, Status %d\n", ex, st);
        print_basis(n, read_basis(n), "J6 read back before update");
        n.update();
        print_basis(n, read_basis(n), "J6 read back after update");
    }
}

int main(int argc, char ** argv) {
    const std::string s = argc > 1 ? argv[1] : "all";
    try {
        if(s == "P" || s == "all") section_P();
        if(s == "C" || s == "all") section_C();
        if(s == "D" || s == "all") section_D();
        if(s == "E" || s == "all") section_E();
        if(s == "F" || s == "all") section_F();
        if(s == "G" || s == "all") section_G();
        if(s == "H" || s == "all") section_H();
        if(s == "I" || s == "all") section_I();
        if(s == "J" || s == "all") section_J();
    } catch(const std::exception & e) {
        std::printf("EXCEPTION: %s\n", e.what());
        return 1;
    }
    return 0;
}
