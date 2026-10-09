// Xpress basis survey probe. Sections chosen by argv[1]: C D E F G.
// Builds every model through MIP++ (xpress_lp), reaches the native basis
// functions through dlsym on api.library_path() (never dlclose).
#include <dlfcn.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "mippp/solvers/xpress/all.hpp"

using namespace mippp;
using prob_t = xpress::impl::v1::XPRSprob;
constexpr double INF = std::numeric_limits<double>::infinity();

// ---------------------------------------------------------------- natives
namespace nat {
int (*getbasis)(prob_t, int *, int *);
int (*loadbasis)(prob_t, const int *, const int *);
int (*getbasisval)(prob_t, int, int, int *, int *);
int (*getpresolvebasis)(prob_t, int *, int *);
int (*loadpresolvebasis)(prob_t, const int *, const int *);
int (*getslacks)(prob_t, int *, double *, int, int);
int (*getlpsol)(prob_t, double *, double *, double *, double *);
int (*removecbmessage)(prob_t, void (*)(prob_t, void *, const char *, int, int),
                       void *);
}  // namespace nat

template <typename F>
void bind_one(void * h, F & f, const char * name) {
    f = reinterpret_cast<F>(dlsym(h, name));
    std::printf("dlsym %-24s %s\n", name, f ? "found" : "MISSING");
}
const xpress_api * API = nullptr;
void bind(const xpress_api & api) {
    API = &api;
    void * h = dlopen(api.library_path().c_str(), RTLD_NOW | RTLD_NOLOAD);
    std::printf("library %s version %s handle %p\n",
                api.library_path().c_str(),
                api.library_version() ? to_string(*api.library_version()).c_str()
                                      : "?",
                h);
    bind_one(h, nat::getbasis, "XPRSgetbasis");
    bind_one(h, nat::loadbasis, "XPRSloadbasis");
    bind_one(h, nat::getbasisval, "XPRSgetbasisval");
    bind_one(h, nat::getpresolvebasis, "XPRSgetpresolvebasis");
    bind_one(h, nat::loadpresolvebasis, "XPRSloadpresolvebasis");
    bind_one(h, nat::getslacks, "XPRSgetslacks");
    bind_one(h, nat::getlpsol, "XPRSgetlpsol");
    bind_one(h, nat::removecbmessage, "XPRSremovecbmessage");
}

// controls / attributes (xprs.h 47.01 lines 239-628, same ids in 45.01)
enum : int {
    LPITERLIMIT = 8007,
    PRESOLVE = 8011,
    CRASH = 8012,
    DEFAULTALG = 8023,
    OUTPUTLOG = 8035,
    CROSSOVER = 8044,
    KEEPBASIS = 8054,
    LPFLAGS = 8385,
    A_ROWS = 1001,
    A_COLS = 1018,
    A_SIMPLEXITER = 1009,
    A_LPSTATUS = 1010,
    A_PRESOLVESTATE = 1026,
    A_ALGORITHM = 1049,
    A_CROSSOVERITER = 1051,
    A_SOLSTATUS = 1053,
    A_ORIGINALROWS = 1124,
    A_STOPSTATUS = 1179,
    A_ORIGINALCOLS = 1214,
    A_OPTIMIZETYPEUSED = 1268,
    A_SOLVESTATUS = 1394,
    A_INPUTROWS = 1408,
    A_INPUTCOLS = 1409,
    A_BARITER = 5001,
};

// ---------------------------------------------------------------- messages
std::vector<std::pair<int, std::string>> g_msgs;
void msg_cb(prob_t, void *, const char * msg, int len, int type) {
    if(msg && len > 0) g_msgs.emplace_back(type, std::string(msg, len));
}
void print_msgs(const char * tag) {
    for(auto & [t, m] : g_msgs) {
        bool keep = t >= 2 || std::getenv("PROBE_ALLMSG");
        for(const char * k :
            {"asis", "rash", "warm", "Warm", "invalid", "Invalid", "lost"})
            if(m.find(k) != std::string::npos) keep = true;
        if(keep) std::printf("   [%s msg type %d] %s\n", tag, t, m.c_str());
    }
    g_msgs.clear();
}

// ---------------------------------------------------------------- helpers
struct ctx {
    xpress_lp & m;
    prob_t p;
    explicit ctx(xpress_lp & model) : m(model), p(model.native_model()) {
        model.set_time_limit(std::chrono::seconds(10));
        nat::removecbmessage(p, nullptr, nullptr);
        API->addcbmessage(p, msg_cb, nullptr, 0);
        API->setintcontrol(p, OUTPUTLOG, 1);
    }
    int geti(int a) const {
        int v = -12345;
        int rc = API->getintattrib(p, a, &v);
        return rc ? -99999 : v;
    }
    int getc(int c) const {
        int v = -12345;
        API->getintcontrol(p, c, &v);
        return v;
    }
    void setc(int c, int v) const {
        int rc = API->setintcontrol(p, c, v);
        if(rc) std::printf("   setintcontrol(%d,%d) rc=%d %s\n", c, v, rc,
                           err().c_str());
    }
    std::string err() const {
        char buf[512] = {0};
        API->getlasterror(p, buf);
        return buf;
    }
};

const char * status_name(std::size_t i) {
    static const char * n[] = {"unknown",           "optimal",
                               "infeasible",        "unbounded",
                               "limit_reached",     "time_limit",
                               "iteration_limit",   "failed",
                               "numerical_failure", "out_of_memory",
                               "interrupted"};
    return i < 11 ? n[i] : "?";
}

void print_state(const ctx & c, const char * tag) {
    std::printf(
        "  [%s] wrapper=%s LPSTATUS=%d STOPSTATUS=%d SOLSTATUS=%d "
        "SOLVESTATUS=%d PRESOLVESTATE=0x%x ALGORITHM=%d OPTTYPE=%d "
        "SIMPLEXITER=%d BARITER=%d XOVERITER=%d KEEPBASIS=%d ROWS=%d/%d/%d "
        "COLS=%d/%d/%d\n",
        tag, status_name(c.m.get_status().index()), c.geti(A_LPSTATUS),
        c.geti(A_STOPSTATUS), c.geti(A_SOLSTATUS), c.geti(A_SOLVESTATUS),
        c.geti(A_PRESOLVESTATE), c.geti(A_ALGORITHM),
        c.geti(A_OPTIMIZETYPEUSED), c.geti(A_SIMPLEXITER), c.geti(A_BARITER),
        c.geti(A_CROSSOVERITER), c.getc(KEEPBASIS), c.geti(A_ROWS),
        c.geti(A_ORIGINALROWS), c.geti(A_INPUTROWS), c.geti(A_COLS),
        c.geti(A_ORIGINALCOLS), c.geti(A_INPUTCOLS));
}

struct basis {
    int rc = 0;
    std::string err;
    std::vector<int> r, c;
    int num_basic() const {
        int k = 0;
        for(int s : r) k += s == 1;
        for(int s : c) k += s == 1;
        return k;
    }
};
basis get_basis(const ctx & c, bool presolved = false) {
    basis b;
    int nr = presolved ? c.geti(A_ROWS) : c.geti(A_ORIGINALROWS);
    int nc = presolved ? c.geti(A_COLS) : c.geti(A_ORIGINALCOLS);
    if(nr < 0) nr = 0;
    if(nc < 0) nc = 0;
    b.r.assign(static_cast<std::size_t>(nr), -9);
    b.c.assign(static_cast<std::size_t>(nc), -9);
    b.rc = presolved ? nat::getpresolvebasis(c.p, b.r.data(), b.c.data())
                     : nat::getbasis(c.p, b.r.data(), b.c.data());
    if(b.rc) b.err = c.err();
    return b;
}
std::string codes(const std::vector<int> & v, std::size_t max = 40) {
    std::string s;
    for(std::size_t i = 0; i < v.size() && i < max; ++i)
        s += (v[i] == -9 ? std::string("_") : std::to_string(v[i]));
    if(v.size() > max) s += "...";
    return s;
}
void hist(const std::vector<int> & v, int out[5]) {
    for(int i = 0; i < 5; ++i) out[i] = 0;
    for(int s : v) {
        if(s >= 0 && s <= 3)
            ++out[s];
        else
            ++out[4];
    }
}
void print_basis(const ctx & c, const char * tag, const basis & b) {
    int hr[5], hc[5];
    hist(b.r, hr);
    hist(b.c, hc);
    std::printf(
        "  [%s] getbasis rc=%d%s%s rows(%zu) 0:%d 1:%d 2:%d 3:%d other:%d | "
        "cols(%zu) 0:%d 1:%d 2:%d 3:%d other:%d | #basic=%d vs ROWS=%d\n",
        tag, b.rc, b.rc ? " err=" : "", b.err.c_str(), b.r.size(), hr[0],
        hr[1], hr[2], hr[3], hr[4], b.c.size(), hc[0], hc[1], hc[2], hc[3],
        hc[4], b.num_basic(), c.geti(A_ROWS));
    std::printf("     row codes %s\n     col codes %s\n", codes(b.r).c_str(),
                codes(b.c).c_str());
}
int load_basis(const ctx & c, const basis & b, const char * tag) {
    int rc = nat::loadbasis(c.p, b.r.data(), b.c.data());
    std::printf("  [%s] loadbasis rc=%d%s%s KEEPBASIS after=%d\n", tag, rc,
                rc ? " err=" : "", rc ? c.err().c_str() : "",
                c.getc(KEEPBASIS));
    return rc;
}
void solve(ctx & c, const char * tag) {
    try {
        c.m.solve();
    } catch(const std::exception & e) {
        std::printf("  [%s] solve threw: %s\n", tag, e.what());
    }
    print_state(c, tag);
    double obj = NAN;
    int rc = API->getdblattrib(c.p, 2001, &obj);
    std::printf("  [%s] LPOBJVAL=%.9g (rc %d)\n", tag, obj, rc);
    print_msgs(tag);
}

// ---------------------------------------------------------------- model data
struct col_data {
    const char * name;
    double lb, ub, cost;
};
struct row_data {
    const char * name;
    std::vector<std::pair<int, double>> terms;
    double lo, hi;
};
struct lp_data {
    std::vector<col_data> cols;
    std::vector<row_data> rows;
    bool maximize = false;
};

using var_t = model_variable_t<xpress_lp>;
using con_t = model_constraint_t<xpress_lp>;
using vparams_t = model_variable_params_t<xpress_lp>;

std::optional<double> opt(double v) {
    return std::isinf(v) ? std::nullopt : std::optional<double>(v);
}
var_t add_col(xpress_lp & m, const col_data & cd) {
    return m.add_variable(
        {.obj_coef = 0., .lower_bound = opt(cd.lb), .upper_bound = opt(cd.ub)});
}
con_t add_row(xpress_lp & m, const std::vector<var_t> & x, const row_data & r) {
    using namespace operators;
    auto e = xsum(r.terms, [&](const std::pair<int, double> & t) {
        return t.second * x[static_cast<std::size_t>(t.first)];
    });
    if(r.lo == r.hi) return m.add_constraint(e == r.lo);
    if(std::isinf(r.lo) && std::isinf(r.hi)) {
        auto cc = m.add_constraint(e <= 50.);
        m.set_constraint_upper_bound(cc, m.infinity());
        return cc;
    }
    if(std::isinf(r.lo)) return m.add_constraint(e <= r.hi);
    if(std::isinf(r.hi)) return m.add_constraint(e >= r.lo);
    auto cc = m.add_constraint(e <= r.hi);
    m.set_constraint_lower_bound(cc, r.lo);
    return cc;
}
struct built {
    std::vector<var_t> x;
    std::vector<con_t> c;
};
built build(xpress_lp & m, const lp_data & d) {
    using namespace operators;
    built b;
    for(auto & cd : d.cols) b.x.push_back(add_col(m, cd));
    for(auto & r : d.rows) b.c.push_back(add_row(m, b.x, r));
    m.set_objective(
        xsum(std::views::iota(std::size_t{0}, d.cols.size()),
             [&](std::size_t j) { return d.cols[j].cost * b.x[j]; }));
    if(d.maximize)
        m.set_maximization();
    else
        m.set_minimization();
    return b;
}

// The C model: unique nondegenerate optimum except for the free column x6.
// optimum: x1=5 x2=3 x3=4(U) x4=2(F,dj+3) x5=2 x6=0(free,dj0) x7=0(L)
// x8=3 x10=2 x11=4 x12=1(F,dj-3) x13=5(U, lb -inf) x14=-3(L)
lp_data small_lp() {
    lp_data d;
    d.cols = {
        {"x1", 0, 10, 1.5},   {"x2", 0, 10, -3},      {"x3", 0, 4, -2},
        {"x4", 2, 2, 3},      {"x5", -INF, INF, 1},   {"x6", -INF, INF, 0},
        {"x7", 0, 10, 1},     {"x8", 0, 10, -1},      {"x10", 0, 100, 1},
        {"x11", 0, 10, -1},   {"x12", 1, 1, -3},      {"x13", -INF, 5, -1},
        {"x14", -3, INF, 1},
    };
    // indices: x1 0, x2 1, x3 2, x4 3, x5 4, x6 5, x7 6, x8 7, x10 8, x11 9,
    // x12 10, x13 11, x14 12
    d.rows = {
        {"r1 L tight", {{0, 1}, {1, 1}}, -INF, 8},
        {"r2 G tight", {{0, 1}, {1, -1}}, 2, INF},
        {"r3 E y>0", {{4, 1}, {0, 1}}, 7, 7},
        {"r4 R@upper", {{1, 1}, {7, 1}}, 1, 6},
        {"r5 L basic", {{0, 1}, {1, 1}, {2, 1}, {6, 1}, {12, 1}}, -INF, 100},
        {"r6 L basic", {{3, 1}, {0, 1}, {11, 1}, {10, 1}}, -INF, 20},
        {"r7 R@lower", {{8, 1}, {7, 1}}, 5, 12},
        {"r8 E y<0", {{9, 1}, {6, 1}}, 4, 4},
        {"r9 N free", {{0, 1}, {2, 1}}, -INF, INF},
        {"r10 G basic", {{0, 1}, {1, 1}}, 1, INF},
    };
    return d;
}

// IterationLimitTest's model: dense positive rows, maximize; presolve keeps
// it whole and simplex needs many iterations.
lp_data big_lp(std::size_t size = 200, unsigned seed = 42) {
    lp_data d;
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> coef(1, 1000);
    std::vector<int> row(size);
    for(int & a : row) a = coef(rng);
    for(std::size_t j = 0; j < size; ++j)
        d.cols.push_back({"x", 0, INF, double(row[j])});
    for(std::size_t i = 0; i < size; ++i) {
        row_data r{"r", {}, -INF, 1000.0 * double(size)};
        for(std::size_t j = 0; j < size; ++j)
            r.terms.emplace_back(int(j), double(coef(rng)));
        d.rows.push_back(std::move(r));
    }
    d.maximize = true;
    return d;
}

void print_table(const ctx & c, const lp_data & d, const basis & b) {
    const int nc = int(d.cols.size()), nr = int(d.rows.size());
    std::vector<double> x(nc, NAN), dj(nc, NAN), sl(nr, NAN), y(nr, NAN);
    std::vector<char> type(nr, '?');
    std::vector<double> rhs(nr, NAN), rng(nr, NAN);
    int st;
    int rc1 = API->getsolution(c.p, &st, x.data(), 0, nc - 1);
    int rc2 = API->getredcosts(c.p, &st, dj.data(), 0, nc - 1);
    int rc3 = nat::getslacks(c.p, &st, sl.data(), 0, nr - 1);
    int rc4 = API->getduals(c.p, &st, y.data(), 0, nr - 1);
    API->getrowtype(c.p, type.data(), 0, nr - 1);
    API->getrhs(c.p, rhs.data(), 0, nr - 1);
    API->getrhsrange(c.p, rng.data(), 0, nr - 1);
    std::printf("  sol rc %d %d %d %d\n", rc1, rc2, rc3, rc4);
    std::printf("  %-6s %8s %8s %10s %10s  code  getbasisval\n", "col", "lb",
                "ub", "x", "dj");
    for(int j = 0; j < nc; ++j) {
        int rs = -9, cs = -9;
        int rc = nat::getbasisval(c.p, -1, j, nullptr, &cs);
        (void)rs;
        std::printf("  %-6s %8g %8g %10.5g %10.5g  %d     %d (rc %d)\n",
                    d.cols[j].name, d.cols[j].lb, d.cols[j].ub, x[j], dj[j],
                    j < int(b.c.size()) ? b.c[j] : -9, cs, rc);
    }
    std::printf("  %-12s %4s %8s %8s %10s %10s %10s  code  getbasisval\n",
                "row", "type", "rhs", "range", "activity", "slack", "dual");
    for(int i = 0; i < nr; ++i) {
        double act = 0;
        for(auto & [j, a] : d.rows[i].terms) act += a * x[j];
        int rs = -9;
        int rc = nat::getbasisval(c.p, i, -1, &rs, nullptr);
        std::printf(
            "  %-12s %4c %8g %8g %10.5g %10.5g %10.5g  %d     %d (rc %d)\n",
            d.rows[i].name, type[i], rhs[i], rng[i], act, sl[i], y[i],
            i < int(b.r.size()) ? b.r[i] : -9, rs, rc);
    }
}

// ---------------------------------------------------------------- section C
void section_C() {
    struct variant {
        const char * name;
        std::vector<std::pair<int, int>> controls;
    };
    std::vector<variant> vs = {
        {"default", {}},
        {"presolve=0", {{PRESOLVE, 0}}},
        {"presolve=0 LPFLAGS dual", {{PRESOLVE, 0}, {LPFLAGS, 1}}},
        {"presolve=0 LPFLAGS primal", {{PRESOLVE, 0}, {LPFLAGS, 2}}},
        {"LPFLAGS primal", {{LPFLAGS, 2}}},
        {"LPFLAGS barrier (crossover default)", {{LPFLAGS, 4}}},
        {"presolve=0 LPFLAGS barrier CROSSOVER=0", {{PRESOLVE, 0}, {LPFLAGS, 4}, {CROSSOVER, 0}}},
    };
    const auto d = small_lp();
    for(auto & v : vs) {
        std::printf("\n=== C: %s\n", v.name);
        xpress_lp m(*API);
        ctx c(m);
        build(m, d);
        for(auto [k, val] : v.controls) c.setc(k, val);
        std::printf("  DEFAULTALG=%d LPFLAGS=%d PRESOLVE=%d CROSSOVER=%d\n",
                    c.getc(DEFAULTALG), c.getc(LPFLAGS), c.getc(PRESOLVE),
                    c.getc(CROSSOVER));
        solve(c, "C");
        auto b = get_basis(c);
        print_basis(c, "C", b);
        print_table(c, d, b);
    }
}

// ---------------------------------------------------------------- section D
void section_D() {
    const auto d = small_lp();
    {
        std::printf("\n=== D1: before any solve (small model)\n");
        xpress_lp m(*API);
        ctx c(m);
        build(m, d);
        print_state(c, "D1");
        auto b = get_basis(c);
        print_basis(c, "D1", b);
        int rs = -9, cs = -9;
        int rc = nat::getbasisval(c.p, 0, 0, &rs, &cs);
        std::printf("  [D1] getbasisval(0,0) rc=%d rs=%d cs=%d %s\n", rc, rs,
                    cs, rc ? c.err().c_str() : "");
        print_msgs("D1");
    }
    {
        std::printf("\n=== D1b: empty model getbasis\n");
        xpress_lp m(*API);
        ctx c(m);
        auto b = get_basis(c);
        print_basis(c, "D1b", b);
        print_msgs("D1b");
    }
    for(int presolve : {1, 0}) {
        std::printf("\n=== D3: infeasible LP presolve=%d\n", presolve);
        lp_data inf;
        inf.cols = {{"x", 0, 10, 1}, {"y", 0, 10, 1}, {"z", 0, 10, 1}};
        inf.rows = {{"a", {{0, 1}, {1, 1}, {2, 1}}, 4, INF},
                    {"b", {{0, 1}, {1, 1}, {2, 2}}, -INF, 2},
                    {"c", {{0, 1}, {1, -1}}, -INF, 3}};
        xpress_lp m(*API);
        ctx c(m);
        build(m, inf);
        c.setc(PRESOLVE, presolve);
        solve(c, "D3");
        print_basis(c, "D3", get_basis(c));
    }
    for(int presolve : {1, 0}) {
        std::printf("\n=== D4: unbounded LP presolve=%d\n", presolve);
        lp_data ub;
        ub.cols = {{"x", 0, INF, 1}, {"y", 0, INF, 1}, {"z", 0, 5, 0}};
        ub.rows = {{"a", {{0, 1}, {1, -1}, {2, 1}}, -INF, 1},
                   {"b", {{0, 1}, {1, -2}}, -INF, 4}};
        ub.maximize = true;
        xpress_lp m(*API);
        ctx c(m);
        build(m, ub);
        c.setc(PRESOLVE, presolve);
        solve(c, "D4");
        print_basis(c, "D4", get_basis(c));
    }
    const auto big = big_lp();
    {
        std::printf("\n=== D0: big model default cold solve\n");
        xpress_lp m(*API);
        ctx c(m);
        build(m, big);
        solve(c, "D0");
        auto b = get_basis(c);
        print_basis(c, "D0", b);
    }
    for(int lim : {5, 0}) {
        std::printf("\n=== D5: big model LPITERLIMIT=%d via model.solve()\n",
                    lim);
        xpress_lp m(*API);
        ctx c(m);
        build(m, big);
        c.setc(LPITERLIMIT, lim);
        solve(c, "D5");
        print_basis(c, "D5", get_basis(c));
        std::printf("  -> resume with no limit (KEEPBASIS=%d)\n",
                    c.getc(KEEPBASIS));
        c.setc(LPITERLIMIT, 2147483647);
        solve(c, "D5 resume");
    }
    for(int lim : {5, 0}) {
        std::printf(
            "\n=== D5n: big model LPITERLIMIT=%d native lpoptimize, no "
            "postsolve\n",
            lim);
        xpress_lp m(*API);
        ctx c(m);
        build(m, big);
        c.setc(LPITERLIMIT, lim);
        int rc = API->lpoptimize(c.p, "");
        std::printf("  lpoptimize rc=%d\n", rc);
        print_state(c, "D5n");
        print_basis(c, "D5n getbasis", get_basis(c));
        print_basis(c, "D5n getpresolvebasis", get_basis(c, true));
        auto b = get_basis(c);
        rc = nat::loadbasis(c.p, b.r.data(), b.c.data());
        std::printf("  [D5n] loadbasis while (maybe) presolved rc=%d %s\n", rc,
                    rc ? c.err().c_str() : "");
        rc = API->postsolve(c.p);
        std::printf("  postsolve rc=%d\n", rc);
        print_state(c, "D5n post");
        print_basis(c, "D5n post getbasis", get_basis(c));
        print_msgs("D5n");
    }
    {
        std::printf("\n=== D6: time limit stop (n=400 dense, 1 ms)\n");
        auto big4 = big_lp(400, 7);
        xpress_lp m(*API);
        ctx c(m);
        build(m, big4);
        m.set_time_limit(std::chrono::milliseconds(1));
        solve(c, "D6");
        print_basis(c, "D6", get_basis(c));
    }
    for(int xo : {0, -1}) {
        std::printf("\n=== D8: barrier LPFLAGS=4 CROSSOVER=%d (big)\n", xo);
        xpress_lp m(*API);
        ctx c(m);
        build(m, big);
        c.setc(LPFLAGS, 4);
        c.setc(CROSSOVER, xo);
        solve(c, "D8");
        print_basis(c, "D8", get_basis(c));
    }
    {
        std::printf("\n=== D8s: barrier no crossover on the small model\n");
        xpress_lp m(*API);
        ctx c(m);
        build(m, d);
        c.setc(LPFLAGS, 4);
        c.setc(CROSSOVER, 0);
        c.setc(PRESOLVE, 0);
        solve(c, "D8s");
        auto b = get_basis(c);
        print_basis(c, "D8s", b);
        print_table(c, d, b);
    }
    // modifications after an optimal solve, without re-solving
    for(int kind = 0; kind < 4; ++kind) {
        static const char * names[] = {"add row", "change bound",
                                       "add column", "change rhs"};
        std::printf("\n=== D7: optimal small model then %s (no re-solve)\n",
                    names[kind]);
        xpress_lp m(*API);
        ctx c(m);
        auto bt = build(m, d);
        c.setc(PRESOLVE, 0);
        solve(c, "D7");
        print_basis(c, "D7 before", get_basis(c));
        using namespace operators;
        if(kind == 0) m.add_constraint(bt.x[0] + bt.x[2] <= 30.);
        if(kind == 1) m.set_variable_upper_bound(bt.x[2], 3.);
        if(kind == 2) m.add_variable({.obj_coef = 1., .lower_bound = 0.});
        if(kind == 3) m.set_constraint_upper_bound(bt.c[0], 9.);
        print_state(c, "D7 after");
        print_basis(c, "D7 after", get_basis(c));
        print_msgs("D7");
        solve(c, "D7 resolve");
        print_basis(c, "D7 resolve", get_basis(c));
    }
}

// ---------------------------------------------------------------- section E
void section_E() {
    const auto big = big_lp();
    basis bA;
    double objA = NAN;
    {
        std::printf("\n=== E1: model A cold\n");
        xpress_lp m(*API);
        ctx c(m);
        build(m, big);
        solve(c, "E1");
        API->getdblattrib(c.p, 2001, &objA);
        bA = get_basis(c);
        print_basis(c, "E1", bA);
    }
    struct variant {
        const char * name;
        std::vector<std::pair<int, int>> controls;
        bool load;
    };
    std::vector<variant> vs = {
        {"cold, LPITERLIMIT=0", {{LPITERLIMIT, 0}}, false},
        {"load, LPITERLIMIT=0", {{LPITERLIMIT, 0}}, true},
        {"load, LPITERLIMIT=1", {{LPITERLIMIT, 1}}, true},
        {"load, unlimited", {}, true},
        {"load, presolve=0 LPITERLIMIT=0", {{PRESOLVE, 0}, {LPITERLIMIT, 0}},
         true},
        {"load, primal LPITERLIMIT=0", {{LPFLAGS, 2}, {LPITERLIMIT, 0}},
         true},
        {"load, dual LPITERLIMIT=0", {{LPFLAGS, 1}, {LPITERLIMIT, 0}},
         true},
        {"load, barrier unlimited", {{LPFLAGS, 4}}, true},
        {"load, KEEPBASIS=0 after load, unlimited", {}, true},
    };
    for(auto & v : vs) {
        std::printf("\n=== E2: fresh B, %s\n", v.name);
        xpress_lp m(*API);
        ctx c(m);
        build(m, big);
        for(auto [k, val] : v.controls) c.setc(k, val);
        std::printf("  KEEPBASIS before load=%d\n", c.getc(KEEPBASIS));
        if(v.load) load_basis(c, bA, "E2");
        if(std::strstr(v.name, "KEEPBASIS=0")) c.setc(KEEPBASIS, 0);
        solve(c, "E2");
        std::printf("  objA=%.9g\n", objA);
        auto b = get_basis(c);
        bool same = b.rc == 0 && b.r == bA.r && b.c == bA.c;
        print_basis(c, "E2 after", b);
        std::printf("  basis after == A's: %s\n", same ? "yes" : "no");
    }
    // round trip on the small model with every status kind
    const auto d = small_lp();
    basis bS;
    {
        std::printf("\n=== E3: small model A (presolve=0)\n");
        xpress_lp m(*API);
        ctx c(m);
        build(m, d);
        c.setc(PRESOLVE, 0);
        solve(c, "E3");
        bS = get_basis(c);
        print_basis(c, "E3", bS);
    }
    for(int presolve : {1, 0}) {
        std::printf("\n=== E3b: small model B load, LPITERLIMIT=0 presolve=%d\n",
                    presolve);
        xpress_lp m(*API);
        ctx c(m);
        build(m, d);
        c.setc(PRESOLVE, presolve);
        c.setc(LPITERLIMIT, 0);
        load_basis(c, bS, "E3b");
        solve(c, "E3b");
        print_basis(c, "E3b after", get_basis(c));
    }
    // changed bound
    {
        std::printf("\n=== E4: A solved, bound changed, re-solve (no load)\n");
        xpress_lp m(*API);
        ctx c(m);
        auto bt = build(m, big);
        solve(c, "E4 first");
        auto b0 = get_basis(c);
        int basic_col = -1;
        for(std::size_t j = 0; j < b0.c.size(); ++j)
            if(b0.c[j] == 1) {
                basic_col = int(j);
                break;
            }
        std::vector<double> x(b0.c.size());
        API->getsolution(c.p, nullptr, x.data(), 0, int(x.size()) - 1);
        std::printf("  tightening ub of basic col %d (x=%g) to x/2\n",
                    basic_col, x[std::size_t(basic_col)]);
        m.set_variable_upper_bound(bt.x[std::size_t(basic_col)],
                                   x[std::size_t(basic_col)] / 2);
        print_basis(c, "E4 after chgbound", get_basis(c));
        solve(c, "E4 resolve");
    }
    {
        std::printf("\n=== E5: A solved, bound changed, then load A's basis\n");
        xpress_lp m(*API);
        ctx c(m);
        auto bt = build(m, big);
        solve(c, "E5 first");
        auto b0 = get_basis(c);
        int basic_col = -1;
        for(std::size_t j = 0; j < b0.c.size(); ++j)
            if(b0.c[j] == 1) {
                basic_col = int(j);
                break;
            }
        std::vector<double> x(b0.c.size());
        API->getsolution(c.p, nullptr, x.data(), 0, int(x.size()) - 1);
        m.set_variable_upper_bound(bt.x[std::size_t(basic_col)],
                                   x[std::size_t(basic_col)] / 2);
        load_basis(c, b0, "E5");
        print_basis(c, "E5 getbasis after load", get_basis(c));
        solve(c, "E5 resolve");
    }
    {
        std::printf(
            "\n=== E6: fresh B, load A's basis, then change a bound, solve\n");
        xpress_lp m(*API);
        ctx c(m);
        auto bt = build(m, big);
        load_basis(c, bA, "E6");
        print_basis(c, "E6 getbasis right after load", get_basis(c));
        int nb = -1;
        for(std::size_t j = 0; j < bA.c.size(); ++j)
            if(bA.c[j] == 0) {
                nb = int(j);
                break;
            }
        // nonbasic at lower 0: raise its lower bound to 0.001
        m.set_variable_lower_bound(bt.x[std::size_t(nb)], 1e-3);
        solve(c, "E6");
    }
    // invalid bases on the small model
    struct inval {
        const char * name;
        int kind;
        int alg;
    };
    std::vector<inval> iv = {
        {"one extra basic (x3 U->basic)", 0, 0},
        {"one missing basic (x1 basic->0)", 1, 0},
        {"code 2 on x14 (ub=+inf)", 2, 0},
        {"code 0 on free x6", 3, 0},
        {"code 2 on free x6", 4, 0},
        {"code 3 on bounded x7 (dual)", 5, 1},
        {"code 3 on bounded x7 (primal)", 5, 2},
        {"code 3 on bounded x7 (default)", 5, 0},
        {"code 7 on column x7", 6, 0},
        {"code 7 on row r1", 7, 0},
        {"code 2 on r1 (L row, slack ub inf?)", 8, 0},
        {"code 2 on r2 (G row)", 9, 0},
        {"all-slack basis", 10, 0},
        {"code -1 on column x7", 11, 0},
        {"code 3 on row r1", 12, 0},
        {"x13 (lb -inf) code 0", 13, 0},
    };
    for(auto & v : iv) {
        std::printf("\n=== E7: invalid basis: %s\n", v.name);
        xpress_lp m(*API);
        ctx c(m);
        build(m, d);
        c.setc(PRESOLVE, 0);
        if(v.alg) c.setc(LPFLAGS, v.alg);
        basis b = bS;
        switch(v.kind) {
            case 0: b.c[2] = 1; break;
            case 1: b.c[0] = 0; break;
            case 2: b.c[12] = 2; break;
            case 3: b.c[5] = 0; break;
            case 4: b.c[5] = 2; break;
            case 5: b.c[6] = 3; break;
            case 6: b.c[6] = 7; break;
            case 7: b.r[0] = 7; break;
            case 8: b.r[0] = 2; break;
            case 9: b.r[1] = 2; break;
            case 10:
                for(int & s : b.c) s = 0;
                for(int & s : b.r) s = 1;
                break;
            case 11: b.c[6] = -1; break;
            case 12: b.r[0] = 3; break;
            case 13: b.c[11] = 0; break;
        }
        std::printf("  loading row %s col %s (#basic %d, rows %zu)\n",
                    codes(b.r).c_str(), codes(b.c).c_str(), b.num_basic(),
                    b.r.size());
        load_basis(c, b, "E7");
        auto after = get_basis(c);
        print_basis(c, "E7 getbasis after load", after);
        print_msgs("E7 load");
        solve(c, "E7");
        print_basis(c, "E7 final", get_basis(c));
    }
}

// ---------------------------------------------------------------- section F
void section_F() {
    const auto d = small_lp();
    for(int lim : {0, -1}) {
        std::printf(
            "\n=== F1: small, add col+row after get, load extended (lim %d)\n",
            lim);
        xpress_lp m(*API);
        ctx c(m);
        auto bt = build(m, d);
        c.setc(PRESOLVE, 0);
        solve(c, "F1 first");
        auto b = get_basis(c);
        using namespace operators;
        // a new column with positive cost (stays at lower 0) in r5, and a
        // new row that is slack at the optimum
        std::vector<std::pair<con_t, double>> entries{{bt.c[4], 1.0}};
        auto y = m.add_column(entries,
                              vparams_t{.obj_coef = 2., .lower_bound = 0.,
                                        .upper_bound = 10.});
        m.add_constraint(bt.x[0] + bt.x[1] + y <= 50.);
        print_basis(c, "F1 getbasis after add", get_basis(c));
        b.c.push_back(0);
        b.r.push_back(1);
        if(lim >= 0) c.setc(LPITERLIMIT, lim);
        load_basis(c, b, "F1");
        solve(c, "F1");
        print_basis(c, "F1 final", get_basis(c));
    }
    {
        std::printf("\n=== F2: big model, add a binding row after solve; "
                    "load extended with the new row basic\n");
        const auto big = big_lp();
        xpress_lp m(*API);
        ctx c(m);
        auto bt = build(m, big);
        solve(c, "F2 first");
        auto b = get_basis(c);
        using namespace operators;
        m.add_constraint(bt.x[0] + bt.x[1] <= 1e6);  // slack
        b.r.push_back(1);
        load_basis(c, b, "F2");
        solve(c, "F2 (slack new row)");
    }
    {
        std::printf("\n=== F3: big model, fresh model with one extra column "
                    "and row, load A's basis extended\n");
        const auto big = big_lp();
        basis bA;
        {
            xpress_lp m(*API);
            ctx c(m);
            build(m, big);
            solve(c, "F3 A");
            bA = get_basis(c);
        }
        xpress_lp m(*API);
        ctx c(m);
        auto bt = build(m, big);
        using namespace operators;
        auto y = m.add_variable({.obj_coef = -1., .lower_bound = 0.});
        m.add_constraint(bt.x[0] + y <= 1e6);
        bA.c.push_back(0);
        bA.r.push_back(1);
        load_basis(c, bA, "F3");
        solve(c, "F3");
    }
}

// ---------------------------------------------------------------- section G
void section_G() {
    const auto d = small_lp();
    {
        std::printf("\n=== G1: load basis, then add a row, then solve "
                    "(LPITERLIMIT=0)\n");
        basis bS;
        {
            xpress_lp m(*API);
            ctx c(m);
            build(m, d);
            c.setc(PRESOLVE, 0);
            solve(c, "G1 A");
            bS = get_basis(c);
        }
        xpress_lp m(*API);
        ctx c(m);
        auto bt = build(m, d);
        load_basis(c, bS, "G1");
        using namespace operators;
        m.add_constraint(bt.x[0] + bt.x[2] <= 30.);
        print_basis(c, "G1 getbasis after add row", get_basis(c));
        c.setc(LPITERLIMIT, 0);
        solve(c, "G1");
    }
    {
        std::printf("\n=== G2: KEEPBASIS after optimal solve; second solve "
                    "iterations\n");
        const auto big = big_lp();
        xpress_lp m(*API);
        ctx c(m);
        build(m, big);
        std::printf("  KEEPBASIS initial=%d\n", c.getc(KEEPBASIS));
        solve(c, "G2 first");
        solve(c, "G2 second");
    }
    {
        std::printf("\n=== G3: set objective sense change after solve then "
                    "getbasis\n");
        xpress_lp m(*API);
        ctx c(m);
        build(m, d);
        c.setc(PRESOLVE, 0);
        solve(c, "G3");
        m.set_maximization();
        print_basis(c, "G3 after set_maximization", get_basis(c));
        m.set_minimization();
        print_basis(c, "G3 after sense back", get_basis(c));
    }
}


// ---------------------------------------------------------------- section H
// extra: stale basis after a non-basis solve, flipped valid sides, NULL
// arrays, values of superbasic columns after a stop
void section_H() {
    const auto d = small_lp();
    const auto big = big_lp();
    {
        std::printf("\n=== H1: simplex optimal, then barrier CROSSOVER=0 re-solve\n");
        xpress_lp m(*API);
        ctx c(m);
        build(m, big);
        solve(c, "H1 simplex");
        auto b0 = get_basis(c);
        c.setc(LPFLAGS, 4);
        c.setc(CROSSOVER, 0);
        c.setc(KEEPBASIS, 0);  // else the next solve warm starts simplex
        solve(c, "H1 barrier");
        auto b1 = get_basis(c);
        print_basis(c, "H1 after barrier", b1);
        std::printf("  same as simplex basis: %s\n",
                    (b1.rc == 0 && b1.r == b0.r && b1.c == b0.c) ? "yes" : "no");
    }
    {
        std::printf("\n=== H1b: simplex optimal, then barrier CROSSOVER=0 re-solve, KEEPBASIS left 1\n");
        xpress_lp m(*API);
        ctx c(m);
        build(m, big);
        solve(c, "H1b simplex");
        c.setc(LPFLAGS, 4);
        c.setc(CROSSOVER, 0);
        solve(c, "H1b barrier");
        print_basis(c, "H1b after barrier", get_basis(c));
    }
    {
        std::printf("\n=== H2: optimal small, then presolve-detected infeasible re-solve\n");
        xpress_lp m(*API);
        ctx c(m);
        auto bt = build(m, d);
        solve(c, "H2 first");
        auto b0 = get_basis(c);
        // x1 <= 1 contradicts r2: x1 - x2 >= 2 with x2 >= 0 ... and x1+x2<=8
        m.set_variable_upper_bound(bt.x[0], 1.);
        m.set_variable_upper_bound(bt.x[1], 0.);
        solve(c, "H2 infeasible");
        auto b1 = get_basis(c);
        print_basis(c, "H2 after", b1);
        std::printf("  same as optimal basis: %s\n",
                    (b1.rc == 0 && b1.r == b0.r && b1.c == b0.c) ? "yes" : "no");
    }
    {
        std::printf("\n=== H3: values of code-3 columns after LPITERLIMIT stops\n");
        for(int lim : {0, 5}) {
            xpress_lp m(*API);
            ctx c(m);
            build(m, big);
            c.setc(LPITERLIMIT, lim);
            solve(c, "H3");
            auto b = get_basis(c);
            std::vector<double> x(b.c.size(), NAN);
            int st = -1;
            int rc = API->getsolution(c.p, &st, x.data(), 0, int(x.size()) - 1);
            std::printf("  lim %d getsolution rc %d avail %d\n", lim, rc, st);
            int shown = 0;
            for(std::size_t j = 0; j < b.c.size() && shown < 6; ++j)
                if(b.c[j] == 3) {
                    std::printf("   col %zu code 3 x=%g (lb 0, ub inf)\n", j, x[j]);
                    ++shown;
                }
            for(std::size_t j = 0; j < b.c.size() && shown < 9; ++j)
                if(b.c[j] == 0) {
                    std::printf("   col %zu code 0 x=%g\n", j, x[j]);
                    ++shown;
                }
        }
    }
    basis bS;
    {
        xpress_lp m(*API);
        ctx c(m);
        build(m, d);
        c.setc(PRESOLVE, 0);
        solve(c, "H basis");
        bS = get_basis(c);
    }
    struct flip {
        const char * name;
        bool row;
        int index, code;
    };
    std::vector<flip> fl = {
        {"G row r2 -> 0 (slack lb -inf)", true, 1, 0},
        {"E row r3 (code 2) -> 0", true, 2, 0},
        {"E row r8 (code 0) -> 2", true, 7, 2},
        {"R row r4 (code 0) -> 2", true, 3, 2},
        {"R row r7 (code 2) -> 0", true, 6, 0},
        {"N row r9 (basic) -> 0, and r5 basic->... no: count off", true, 8, 0},
        {"fixed x4 (code 0) -> 2", false, 3, 2},
        {"fixed x12 (code 2) -> 0", false, 10, 0},
        {"fixed x4 (code 0) -> 3", false, 3, 3},
        {"free x6 (code 3) -> 3 kept", false, 5, 3},
    };
    for(auto & f : fl) {
        std::printf("\n=== H4: flip %s\n", f.name);
        xpress_lp m(*API);
        ctx c(m);
        build(m, d);
        c.setc(PRESOLVE, 0);
        basis b = bS;
        (f.row ? b.r : b.c)[std::size_t(f.index)] = f.code;
        std::printf("  loading row %s col %s (#basic %d)\n", codes(b.r).c_str(),
                    codes(b.c).c_str(), b.num_basic());
        load_basis(c, b, "H4");
        print_basis(c, "H4 getbasis after load", get_basis(c));
        solve(c, "H4");
    }
    {
        std::printf("\n=== H5: N row r9 basic->0 with x6 3->1? (count kept)\n");
        xpress_lp m(*API);
        ctx c(m);
        build(m, d);
        c.setc(PRESOLVE, 0);
        basis b = bS;
        b.r[8] = 2;
        b.c[3] = 1;  // fixed x4 basic to keep the count
        load_basis(c, b, "H5");
        print_basis(c, "H5 getbasis after load", get_basis(c));
        solve(c, "H5");
    }
}

void section_N(int which) {
    // NULL arrays to loadbasis (separate process: may crash)
    const auto d = small_lp();
    basis bS;
    {
        xpress_lp m(*API);
        ctx c(m);
        build(m, d);
        c.setc(PRESOLVE, 0);
        solve(c, "N basis");
        bS = get_basis(c);
    }
    xpress_lp m(*API);
    ctx c(m);
    build(m, d);
    c.setc(PRESOLVE, 0);
    int rc = which == 0 ? nat::loadbasis(c.p, nullptr, bS.c.data())
                        : nat::loadbasis(c.p, bS.r.data(), nullptr);
    std::printf("  loadbasis(%s NULL) rc=%d %s KEEPBASIS=%d\n",
                which == 0 ? "rowstat" : "colstat", rc,
                rc ? c.err().c_str() : "", c.getc(KEEPBASIS));
    print_basis(c, "N after", get_basis(c));
    print_msgs("N");
    solve(c, "N");
}

void section_L() {
    const auto big = big_lp();
    for(int lim : {0, 5, 20}) {
        std::printf("\n=== L: getlpsol values after LPITERLIMIT=%d\n", lim);
        xpress_lp m(*API);
        ctx c(m);
        build(m, big);
        c.setc(LPITERLIMIT, lim);
        solve(c, "L");
        auto b = get_basis(c);
        std::vector<double> x(b.c.size(), NAN), sl(b.r.size(), NAN);
        int rc = nat::getlpsol(c.p, x.data(), sl.data(), nullptr, nullptr);
        std::printf("  getlpsol rc %d %s\n", rc, rc ? c.err().c_str() : "");
        int k3 = 0, k3atlb = 0, k0 = 0, k0atlb = 0;
        for(std::size_t j = 0; j < b.c.size(); ++j) {
            if(b.c[j] == 3) { ++k3; k3atlb += x[j] == 0.0; }
            if(b.c[j] == 0) { ++k0; k0atlb += x[j] == 0.0; }
        }
        std::printf("  code3 cols %d (x==0: %d), code0 cols %d (x==0: %d)\n",
                    k3, k3atlb, k0, k0atlb);
        int shown = 0;
        for(std::size_t j = 0; j < b.c.size() && shown < 4; ++j)
            if(b.c[j] == 3) { std::printf("   col %zu code3 x=%g\n", j, x[j]); ++shown; }
        print_msgs("L");
    }
}

// wrong-side writes under primal and dual simplex: is a side error visible?
void section_P() {
    const auto d = small_lp();
    basis bS;
    {
        xpress_lp m(*API);
        ctx c(m);
        build(m, d);
        c.setc(PRESOLVE, 0);
        solve(c, "P basis");
        bS = get_basis(c);
    }
    struct flip {
        const char * name;
        bool row;
        int index, code;
    };
    std::vector<flip> fl = {
        {"none", true, 0, 0},
        {"R row r4 (code 0) -> 2", true, 3, 2},
        {"R row r7 (code 2) -> 0", true, 6, 0},
        {"x3 [0,4] at upper (2) -> 0", false, 2, 0},
        {"x7 [0,10] at lower (0) -> 2", false, 6, 2},
        {"fixed x4 (0) -> 2", false, 3, 2},
        {"E row r3 (2) -> 0", true, 2, 0},
    };
    for(int flags : {1, 2}) {
        for(int presolve : {0, 1}) {
            for(auto & f : fl) {
                std::printf("\n=== P: LPFLAGS=%d presolve=%d flip %s\n", flags,
                            presolve, f.name);
                xpress_lp m(*API);
                ctx c(m);
                build(m, d);
                c.setc(PRESOLVE, presolve);
                c.setc(LPFLAGS, flags);
                basis b = bS;
                if(std::strcmp(f.name, "none"))
                    (f.row ? b.r : b.c)[std::size_t(f.index)] = f.code;
                load_basis(c, b, "P");
                solve(c, "P");
            }
        }
    }
}

// zero-dimension models: empty arrays are NULL pointers (std::vector<int>().data())
void section_Z(int mode) {
    xpress_lp m(*API);
    ctx c(m);
    using namespace operators;
    if(mode <= 2) {
        auto x = m.add_variable({.obj_coef = 0., .lower_bound = 0., .upper_bound = 3.});
        auto y = m.add_variable({.obj_coef = 0., .lower_bound = 0., .upper_bound = 3.});
        m.set_objective(x - y);
        m.set_minimization();
        solve(c, "Z");
    }
    std::vector<int> empty;
    std::printf("  empty.data() = %p\n", static_cast<void *>(empty.data()));
    int cols[2] = {-9, -9}, dummy[1] = {-9};
    if(mode == 0) {
        int rc = nat::getbasis(c.p, nullptr, cols);
        std::printf("  getbasis(NULL rows) rc=%d cols %d %d\n", rc, cols[0], cols[1]);
        rc = nat::getbasis(c.p, dummy, cols);
        std::printf("  getbasis(dummy rows) rc=%d dummy %d cols %d %d\n", rc, dummy[0], cols[0], cols[1]);
    }
    if(mode == 1 || mode == 2) {
        nat::getbasis(c.p, dummy, cols);
        int rc = nat::loadbasis(c.p, mode == 1 ? nullptr : dummy, cols);
        std::printf("  loadbasis(%s rows) rc=%d %s\n", mode == 1 ? "NULL" : "dummy", rc,
                    rc ? c.err().c_str() : "");
        solve(c, "Z after load");
    }
    if(mode == 3 || mode == 4) {
        int rc = nat::getbasis(c.p, nullptr, nullptr);
        std::printf("  empty model getbasis(NULL, NULL) rc=%d\n", rc);
        rc = mode == 3 ? nat::loadbasis(c.p, nullptr, nullptr)
                       : nat::loadbasis(c.p, dummy, dummy);
        std::printf("  empty model loadbasis(%s) rc=%d %s\n", mode == 3 ? "NULL,NULL" : "dummy,dummy",
                    rc, rc ? c.err().c_str() : "");
        solve(c, "Z empty");
    }
    print_msgs("Z");
}

int main(int argc, char ** argv) {
    const xpress_api & api = xpress_api::load();
    bind(api);
    std::string which = argc > 1 ? argv[1] : "CDEFG";
    for(char s : which) {
        try {
            switch(s) {
                case 'C': section_C(); break;
                case 'D': section_D(); break;
                case 'E': section_E(); break;
                case 'F': section_F(); break;
                case 'G': section_G(); break;
                case 'H': section_H(); break;
                case 'L': section_L(); break;
                case 'P': section_P(); break;
                case '0': section_Z(0); break;
                case '1': section_Z(1); break;
                case '2': section_Z(2); break;
                case '3': section_Z(3); break;
                case '4': section_Z(4); break;
                case 'n': section_N(0); break;
                case 'm': section_N(1); break;
            }
        } catch(const std::exception & e) {
            std::printf("SECTION %c threw: %s\n", s, e.what());
        }
        std::fflush(stdout);
    }
}
