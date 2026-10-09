// Clp basis survey probe: one program, forked children for crash-prone calls.
#include <dlfcn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

#include "mippp/solvers/clp/all.hpp"

using namespace mippp;
using namespace mippp::operators;

using CS = void;
struct native {
    int (*statusExists)(CS *);
    unsigned char * (*statusArray)(CS *);
    void (*copyinStatus)(CS *, const unsigned char *);
    const double * (*getRowActivity)(CS *);
    void (*setMaximumIterations)(CS *, int);
    int (*numberIterations)(CS *);
    double (*objectiveValue)(CS *);
};
static native N;

static const char * code_name(int c) {
    switch(c) {
        case 0: return "isFree";
        case 1: return "basic";
        case 2: return "atUpper";
        case 3: return "atLower";
        case 4: return "superBasic";
        case 5: return "isFixed";
        default: return "??";
    }
}

static double fin(double v) {
    return v >= 1e30 ? INFINITY : (v <= -1e30 ? -INFINITY : v);
}

struct lp {
    clp_lp m;
    std::vector<model_variable_t<clp_lp>> v;   // x1..x10, y
    std::vector<model_constraint_t<clp_lp>> c; // le ge eq eq2 rngU rngL slack
    std::vector<std::string> vn, cn;
};

// min -3x1 +3x2 -x3 +x4 +2x5 -x6 -3x7 -2x8 +x9 +x10 (+0 y)
// optimum: x1=2(ub) x2=1(fixed,d=+2) x3=2 x4=2 x5=3(free) x6=4 x7=2(fixed,
// d=-3) x8=5 x9=2 x10=0(lb,d=+1) y=0 (free, empty, d=0); obj -15
static void build(lp & P, bool maximize = false) {
    auto & m = P.m;
    m.set_time_limit(std::chrono::duration<double>(10));
    auto x1 = m.add_variable({.lower_bound = 0.0, .upper_bound = 2.0});
    auto x2 = m.add_variable({.lower_bound = 1.0, .upper_bound = 1.0});
    auto x3 = m.add_variable({.lower_bound = 0.0});
    auto x4 = m.add_variable({.lower_bound = 0.0});
    auto x5 = m.add_variable({.lower_bound = std::nullopt});  // free
    auto x6 = m.add_variable({.lower_bound = 0.0});
    auto x7 = m.add_variable({.lower_bound = 2.0, .upper_bound = 2.0});
    auto x8 = m.add_variable({.lower_bound = 0.0});
    auto x9 = m.add_variable({.lower_bound = 0.0});
    auto x10 = m.add_variable({.lower_bound = 0.0});
    auto y = m.add_variable({.lower_bound = std::nullopt});  // free, empty
    P.v = {x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, y};
    P.vn = {"x1[0,2]",  "x2[1,1]", "x3[0,inf]", "x4[0,inf]",
            "x5 free",  "x6[0,inf]", "x7[2,2]",  "x8[0,inf]",
            "x9[0,inf]", "x10[0,inf]", "y free empty"};
    auto obj = -3 * x1 + 3 * x2 - x3 + x4 + 2 * x5 - x6 - 3 * x7 - 2 * x8 +
               x9 + x10;
    if(maximize) {
        m.set_maximization();
        m.set_objective(-1 * obj);
    } else {
        m.set_objective(obj);
    }
    auto le = m.add_constraint(x3 + x1 <= 4);
    auto ge = m.add_constraint(x4 + x2 >= 3);
    auto eq = m.add_constraint(x5 + x1 == 5);
    auto eq2 = m.add_constraint(x8 + x1 == 7);
    auto rngU = m.add_ranged_constraint(1 * x6, 1.0, 4.0);
    auto rngL = m.add_ranged_constraint(1 * x9, 2.0, 6.0);
    auto sl = m.add_constraint(x3 + x4 + x5 + x6 + x7 + x10 <= 100);
    P.c = {le, ge, eq, eq2, rngU, rngL, sl};
    P.cn = {"le  x3+x1<=4 (dual -1)", "ge  x4+x2>=3 (dual +1)",
            "eq  x5+x1==5 (dual +2)", "eq2 x8+x1==7 (dual -2)",
            "rngU 1<=x6<=4 (upper)",  "rngL 2<=x9<=6 (lower)",
            "slack ...<=100"};
}

static int count_basic(clp_lp & m) {
    auto & A = m.native_api();
    auto * M = m.native_model();
    int nb = 0;
    for(int j = 0; j < A.getNumCols(M); ++j) nb += A.getColumnStatus(M, j) == 1;
    for(int i = 0; i < A.getNumRows(M); ++i) nb += A.getRowStatus(M, i) == 1;
    return nb;
}

static void dump(const char * label, lp & P, bool values = true) {
    auto & m = P.m;
    auto & A = m.native_api();
    auto * M = m.native_model();
    int nc = A.getNumCols(M), nr = A.getNumRows(M);
    std::printf("---- %s\n", label);
    std::printf("  Clp_status=%d secondary=%d iterations=%d obj=%g "
                "statusExists=%d cols=%d rows=%d\n",
                A.status(M), A.secondaryStatus(M), N.numberIterations(M),
                A.getObjValue(M), N.statusExists(M), nc, nr);
    if(!N.statusExists(M)) return;
    const double * x = A.primalColumnSolution(M);
    const double * act = N.getRowActivity(M);
    const double * cl = A.columnLower(M);
    const double * cu = A.columnUpper(M);
    const double * rl = A.rowLower(M);
    const double * ru = A.rowUpper(M);
    const double * dj = A.dualColumnSolution(M);
    const double * pi = A.dualRowSolution(M);
    for(int j = 0; j < nc; ++j) {
        int s = A.getColumnStatus(M, j);
        std::printf("  col %2d %-14s code %d %-10s", j,
                    j < (int)P.vn.size() ? P.vn[j].c_str() : "(new)", s,
                    code_name(s));
        if(values)
            std::printf(" x=%-10g lb=%-6g ub=%-6g dj=%g", x[j], fin(cl[j]),
                        fin(cu[j]), dj[j]);
        std::printf("\n");
    }
    for(int i = 0; i < nr; ++i) {
        int s = A.getRowStatus(M, i);
        std::printf("  row %2d %-24s code %d %-10s", i,
                    i < (int)P.cn.size() ? P.cn[i].c_str() : "(new)", s,
                    code_name(s));
        if(values)
            std::printf(" act=%-8g lo=%-6g up=%-6g pi=%g", act[i], fin(rl[i]),
                        fin(ru[i]), pi[i]);
        std::printf("\n");
    }
    const unsigned char * raw = N.statusArray(M);
    std::printf("  raw bytes (cols then rows):");
    for(int k = 0; k < nc + nr; ++k) std::printf(" %02x", raw[k]);
    std::printf("\n  #basic=%d #rows=%d\n", count_basic(m), nr);
}

static std::vector<unsigned char> snapshot(clp_lp & m, bool mask = true) {
    auto & A = m.native_api();
    auto * M = m.native_model();
    int n = A.getNumCols(M) + A.getNumRows(M);
    std::vector<unsigned char> s(N.statusArray(M), N.statusArray(M) + n);
    if(mask)
        for(auto & b : s) b &= 7;
    return s;
}

static void in_child(const char * label, std::function<void()> f) {
    std::fflush(stdout);
    pid_t pid = fork();
    if(pid == 0) {
        alarm(20);
        f();
        std::printf("  [%s] child returned normally\n", label);
        std::fflush(stdout);
        _exit(0);
    }
    int st = 0;
    waitpid(pid, &st, 0);
    if(WIFSIGNALED(st))
        std::printf("  [%s] child killed by signal %d (%s)\n", label,
                    WTERMSIG(st), strsignal(WTERMSIG(st)));
    else
        std::printf("  [%s] child exit code %d\n", label, WEXITSTATUS(st));
}

static const char * wstatus(clp_lp & m) {
    return std::visit(
        [](auto s) -> const char * {
            using S = decltype(s);
            if constexpr(std::is_same_v<S, status::optimal>) return "optimal";
            else if constexpr(std::is_same_v<S, status::infeasible>)
                return "infeasible";
            else if constexpr(std::is_same_v<S, status::unbounded>)
                return "unbounded";
            else if constexpr(std::is_same_v<S, status::limit_reached>)
                return "limit_reached";
            else if constexpr(std::is_same_v<S, status::time_limit>)
                return "time_limit";
            else if constexpr(std::is_same_v<S, status::unknown>)
                return "unknown";
            else
                return "other";
        },
        m.get_status());
}

// B: fresh identical model, basis written in bulk, iteration limit `iters`
static void warm(const char * label, const std::vector<unsigned char> & basis,
                 int iters, bool show = false) {
    lp B;
    build(B);
    auto * M = B.m.native_model();
    N.copyinStatus(M, basis.data());
    N.setMaximumIterations(M, iters);
    B.m.solve();
    std::printf("== %s: wrapper=%s Clp_status=%d secondary=%d iterations=%d "
                "obj=%g #basic=%d\n",
                label, wstatus(B.m), B.m.native_api().status(M),
                B.m.native_api().secondaryStatus(M), N.numberIterations(M),
                B.m.native_api().getObjValue(M), count_basic(B.m));
    if(show) dump(label, B);
}

int main() {
    lp A;
    build(A);
    auto & api = A.m.native_api();
    std::printf("library %s version %s\n", api.library_path().c_str(),
                api.library_version() ? to_string(*api.library_version()).c_str()
                                      : "(none)");
    void * h = dlopen(api.library_path().c_str(), RTLD_NOW | RTLD_NOLOAD);
    if(!h) {
        std::printf("dlopen failed %s\n", dlerror());
        return 1;
    }
    N.statusExists = (int (*)(CS *))dlsym(h, "Clp_statusExists");
    N.statusArray = (unsigned char * (*)(CS *))dlsym(h, "Clp_statusArray");
    N.copyinStatus =
        (void (*)(CS *, const unsigned char *))dlsym(h, "Clp_copyinStatus");
    N.getRowActivity = (const double * (*)(CS *))dlsym(h, "Clp_getRowActivity");
    N.setMaximumIterations =
        (void (*)(CS *, int))dlsym(h, "Clp_setMaximumIterations");
    N.numberIterations = (int (*)(CS *))dlsym(h, "Clp_numberIterations");
    N.objectiveValue = (double (*)(CS *))dlsym(h, "Clp_objectiveValue");
    std::printf("symbols: statusExists=%p statusArray=%p copyinStatus=%p "
                "getRowActivity=%p setMaximumIterations=%p numberIterations=%p "
                "Clp_maximumIterations=%p\n",
                (void *)N.statusExists, (void *)N.statusArray,
                (void *)N.copyinStatus, (void *)N.getRowActivity,
                (void *)N.setMaximumIterations, (void *)N.numberIterations,
                dlsym(h, "Clp_maximumIterations"));
    auto * MA = A.m.native_model();

    std::printf("\n######## D0: before any solve\n");
    std::printf("  statusExists=%d statusArray=%p Clp_status=%d\n",
                N.statusExists(MA), (void *)N.statusArray(MA), api.status(MA));
    in_child("getColumnStatus(0) before solve", [&] {
        std::printf("  getColumnStatus(0)=%d\n", api.getColumnStatus(MA, 0));
    });
    in_child("getRowStatus(0) before solve", [&] {
        std::printf("  getRowStatus(0)=%d\n", api.getRowStatus(MA, 0));
    });
    in_child("setColumnStatus(0,3) before solve",
             [&] { api.setColumnStatus(MA, 0, 3); });
    in_child("setRowStatus(0,1) before solve",
             [&] { api.setRowStatus(MA, 0, 1); });

    std::printf("\n######## C: optimal solve (min), cold start\n");
    A.m.solve();
    std::printf("wrapper status %s\n", wstatus(A.m));
    dump("A optimal (min)", A);
    const int cold_iterations = N.numberIterations(MA);
    auto SA_raw = snapshot(A.m, false);
    auto SA = snapshot(A.m);

    {
        lp Amax;
        build(Amax, true);
        Amax.m.solve();
        std::printf("wrapper status %s\n", wstatus(Amax.m));
        dump("A optimal (max of -obj)", Amax, false);
    }

    std::printf("\n######## E: warm start\n");
    std::printf("cold iterations of A = %d\n", cold_iterations);
    warm("E1 B <- basis(A) masked, maxIter 0", SA, 0, true);
    warm("E1b B <- basis(A) raw bytes, maxIter 0", SA_raw, 0);
    warm("E1c B <- basis(A) masked, maxIter 1000", SA, 1000);
    {
        // cold B with maxIter 0
        lp B;
        build(B);
        N.setMaximumIterations(B.m.native_model(), 0);
        B.m.solve();
        std::printf("== E0 B cold, maxIter 0: wrapper=%s Clp_status=%d "
                    "iterations=%d obj=%g\n",
                    wstatus(B.m), api.status(B.m.native_model()),
                    N.numberIterations(B.m.native_model()),
                    api.getObjValue(B.m.native_model()));
    }
    {
        // per-entity write after creating the array with copyin(all-slack)
        lp B;
        build(B);
        auto * M = B.m.native_model();
        int nc = api.getNumCols(M), nr = api.getNumRows(M);
        std::vector<unsigned char> slack(nc + nr, 3);
        for(int i = 0; i < nr; ++i) slack[nc + i] = 1;
        N.copyinStatus(M, slack.data());
        for(int j = 0; j < nc; ++j) api.setColumnStatus(M, j, SA[j]);
        for(int i = 0; i < nr; ++i) api.setRowStatus(M, i, SA[nc + i]);
        std::printf("== E2 per-entity setters after copyin(all-slack): "
                    "primalColumnSolution now:");
        for(int j = 0; j < nc; ++j)
            std::printf(" %g", api.primalColumnSolution(M)[j]);
        std::printf("\n");
        N.setMaximumIterations(M, 0);
        B.m.solve();
        std::printf("   after solve maxIter 0: wrapper=%s Clp_status=%d "
                    "iterations=%d obj=%g\n",
                    wstatus(B.m), api.status(M), N.numberIterations(M),
                    api.getObjValue(M));
    }
    {
        // gotcha: per-entity setter overwrites primal values
        lp B;
        build(B);
        B.m.solve();
        auto * M = B.m.native_model();
        auto before = B.m.get_solution()[B.v[9]];
        api.setColumnStatus(M, 9, 2);  // x10 atUpper, ub = inf
        api.setColumnStatus(M, 0, 3);  // x1 atLower
        auto sol = B.m.get_solution();
        std::printf("== G1 setColumnStatus overwrites values: x10 %g -> %g, "
                    "x1 2 -> %g; wrapper status still %s\n",
                    before, sol[B.v[9]], sol[B.v[0]], wstatus(B.m));
    }
    {
        // write basis into A after changing a bound (x1 ub 2 -> 1.5)
        lp B;
        build(B);
        B.m.solve();
        auto * M = B.m.native_model();
        B.m.set_variable_upper_bound(B.v[0], 1.5);
        auto kept = snapshot(B.m);
        std::printf("== E3 after bound change, status array unchanged: %s\n",
                    kept == SA ? "yes" : "no");
        N.setMaximumIterations(M, 0);
        B.m.solve();
        std::printf("   resolve maxIter 0 (implicit warm start): wrapper=%s "
                    "Clp_status=%d iterations=%d obj=%g x1=%g\n",
                    wstatus(B.m), api.status(M), N.numberIterations(M),
                    api.getObjValue(M), B.m.get_solution()[B.v[0]]);
        // now write A's basis explicitly, change x3's lb so x3 must leave
        N.copyinStatus(M, SA.data());
        B.m.set_variable_lower_bound(B.v[2], 3.0);  // x3 = 4 - 1.5 = 2.5 < 3
        N.setMaximumIterations(M, 100);
        B.m.solve();
        std::printf("   copyin(A) + x3.lb=3 then solve: wrapper=%s "
                    "Clp_status=%d iterations=%d obj=%g\n",
                    wstatus(B.m), api.status(M), N.numberIterations(M),
                    api.getObjValue(M));
    }
    {
        // does solve() keep a basis written into a SOLVED model?
        lp B;
        build(B);
        B.m.solve();
        auto * M = B.m.native_model();
        int nc = api.getNumCols(M), nr = api.getNumRows(M);
        std::vector<unsigned char> slack(nc + nr, 3);
        for(int i = 0; i < nr; ++i) slack[nc + i] = 1;
        slack[1] = 5;
        slack[6] = 5;
        slack[4] = 0;
        slack[10] = 0;
        N.copyinStatus(M, slack.data());
        N.setMaximumIterations(M, 1000);
        B.m.solve();
        std::printf("== E4 solved model, copyin(all-slack) then solve: "
                    "iterations=%d (cold was %d) obj=%g\n",
                    N.numberIterations(M), cold_iterations,
                    api.getObjValue(M));
    }

    std::printf("\n######## E5: invalid bases (bulk copyin, maxIter 1000)\n");
    const int nc = api.getNumCols(MA), nr = api.getNumRows(MA);
    {
        std::vector<unsigned char> b(nc + nr, 1);
        warm("E5a all basic (too many)", b, 1000, true);
    }
    {
        std::vector<unsigned char> b(nc + nr, 3);
        warm("E5b all atLower, no basic (too few)", b, 1000, true);
    }
    {
        auto b = SA;
        b[9] = 2;  // x10 atUpper, ub inf
        warm("E5c x10 atUpper on infinite ub", b, 1000, true);
        warm("E5c' same, maxIter 0", b, 0);
    }
    {
        auto b = SA;
        b[10] = 3;  // y free atLower
        warm("E5d y(free) atLower, maxIter 0", b, 0, true);
    }
    {
        auto b = SA;
        b[4] = 3;  // x5 free, basic -> atLower: one basic short
        warm("E5e basic free x5 -> atLower (one basic short)", b, 1000, true);
    }
    {
        auto b = SA;
        b[9] = 4;  // x10 superbasic
        warm("E5f x10 superBasic, maxIter 0", b, 0, true);
    }
    {
        auto b = SA;
        b[9] = 5;  // x10 isFixed although [0,inf)
        warm("E5g x10 isFixed although not fixed, maxIter 0", b, 0, true);
    }
    {
        auto b = SA;
        b[nc + 0] = 3;  // le row: atLower although tight at upper (-inf lo)
        warm("E5h le row atLower (lo=-inf), maxIter 0", b, 0, true);
    }
    {
        auto b = SA;
        b[nc + 1] = 2;  // ge row: atUpper (up = +inf)
        warm("E5i ge row atUpper (up=inf), maxIter 0", b, 0, true);
    }
    {
        auto b = SA;
        b[nc + 4] = 3;  // rngU row: wrong side (atLower, finite)
        warm("E5j rngU row on wrong finite side, maxIter 0", b, 0, true);
        warm("E5j' same, maxIter 1000", b, 1000);
    }
    {
        // singular: x6 and row rngU both basic, x9 nonbasic -> rngL uncovered
        auto b = SA;
        b[nc + 4] = 1;  // rngU basic
        b[8] = 3;       // x9 atLower
        warm("E5k singular basis (x6 + rngU logical), maxIter 1000", b, 1000,
             true);
    }
    {
        auto b = SA;
        b[9] = 6;
        in_child("E5l code 6 via copyin", [&] {
            warm("E5l x10 code 6 via copyin, maxIter 1000", b, 1000, true);
        });
        b[9] = 7;
        in_child("E5m code 7 via copyin", [&] {
            warm("E5m x10 code 7 via copyin, maxIter 1000", b, 1000, true);
        });
    }
    {
        lp B;
        build(B);
        B.m.solve();
        auto * M = B.m.native_model();
        api.setColumnStatus(M, 9, 7);
        api.setColumnStatus(M, 0, -1);
        std::printf("== E5n setColumnStatus(x10, 7) / (x1, -1): codes now "
                    "%d / %d (were 3 / 2)\n",
                    api.getColumnStatus(M, 9), api.getColumnStatus(M, 0));
    }

    std::printf("\n######## D: availability\n");
    {
        lp B;  // infeasible
        auto & m = B.m;
        m.set_time_limit(std::chrono::duration<double>(10));
        auto x = m.add_variable();
        auto yv = m.add_variable();
        B.v = {x, yv};
        B.vn = {"x>=0", "y>=0"};
        m.set_objective(x + yv);
        B.c = {m.add_constraint(x + yv <= -1), m.add_constraint(x - yv >= 2)};
        B.cn = {"x+y<=-1", "x-y>=2"};
        m.solve();
        std::printf("wrapper status %s\n", wstatus(m));
        dump("D2 infeasible LP", B);
    }
    {
        lp B;  // unbounded
        auto & m = B.m;
        m.set_time_limit(std::chrono::duration<double>(10));
        auto x = m.add_variable();
        auto yv = m.add_variable();
        B.v = {x, yv};
        B.vn = {"x>=0", "y>=0"};
        m.set_maximization();
        m.set_objective(x + yv);
        B.c = {m.add_constraint(x - yv <= 1), m.add_constraint(x + 2 * yv >= 2)};
        B.cn = {"x-y<=1", "x+2y>=2"};
        m.solve();
        std::printf("wrapper status %s\n", wstatus(m));
        dump("D3 unbounded LP", B);
    }
    {
        lp B;
        build(B);
        N.setMaximumIterations(B.m.native_model(), 1);
        B.m.solve();
        std::printf("wrapper status %s\n", wstatus(B.m));
        dump("D4 iteration limit 1 (cold)", B);
    }
    {
        lp B;
        build(B);
        N.setMaximumIterations(B.m.native_model(), 3);
        B.m.solve();
        std::printf("wrapper status %s\n", wstatus(B.m));
        dump("D4b iteration limit 3 (cold)", B);
    }
    {
        lp B;
        build(B);
        B.m.set_time_limit(std::chrono::duration<double>(0));
        B.m.solve();
        std::printf("wrapper status %s\n", wstatus(B.m));
        dump("D5 time limit 0", B);
    }
    {
        lp B;
        build(B);
        B.m.solve();
        auto * M = B.m.native_model();
        B.m.set_variable_upper_bound(B.v[0], 1.0);
        std::printf("== D6a bound change: statusExists=%d codes kept=%s "
                    "Clp_status=%d wrapper=%s\n",
                    N.statusExists(M), snapshot(B.m) == SA ? "yes" : "no",
                    api.status(M), wstatus(B.m));
        auto r = B.m.add_constraint(B.v[2] + B.v[3] <= 50);
        B.cn.push_back("new row x3+x4<=50");
        (void)r;
        auto xf = B.m.add_variable({.lower_bound = std::nullopt});
        B.vn.push_back("new free col");
        auto xn = B.m.add_variable({.lower_bound = std::nullopt,
                                    .upper_bound = 5.0});
        B.vn.push_back("new col (-inf,5]");
        auto xc = B.m.add_column({{B.c[6], 1.0}},
                                 {.obj_coef = 5.0, .lower_bound = 0.0});
        B.vn.push_back("new add_column [0,inf)");
        (void)xf;
        (void)xn;
        (void)xc;
        std::printf("   after add row + 3 cols: statusExists=%d Clp_status=%d "
                    "wrapper=%s\n",
                    N.statusExists(M), api.status(M), wstatus(B.m));
        dump("D6b after modifications, no re-solve", B, false);
        auto ext = snapshot(B.m);
        std::printf("   old entries preserved: %s\n",
                    std::equal(SA.begin(), SA.begin() + 11, ext.begin()) &&
                            std::equal(SA.begin() + 11, SA.end(),
                                       ext.begin() + 14)
                        ? "yes"
                        : "no");
        N.setMaximumIterations(M, 1000);
        B.m.solve();
        std::printf("   re-solve (implicit warm start): wrapper=%s "
                    "Clp_status=%d iterations=%d obj=%g\n",
                    wstatus(B.m), api.status(M), N.numberIterations(M),
                    api.getObjValue(M));
        dump("D6c after re-solve", B, true);
    }
    {
        lp B;
        auto & m = B.m;
        m.set_time_limit(std::chrono::duration<double>(10));
        auto x = m.add_variable({.upper_bound = 3.0});
        auto yv = m.add_variable({.lower_bound = std::nullopt});
        B.v = {x, yv};
        B.vn = {"x[0,3]", "y free"};
        m.set_objective(-1 * x);
        m.solve();
        std::printf("wrapper status %s\n", wstatus(m));
        dump("D7a columns only, no rows", B);
        m.add_constraint(0 * x <= 1);
        B.cn = {"0x<=1 (stored empty)"};
        m.solve();
        std::printf("wrapper status %s\n", wstatus(m));
        dump("D7b + an all-zero row", B);
    }
    {
        lp B;
        auto & m = B.m;
        m.set_time_limit(std::chrono::duration<double>(10));
        auto x = m.add_variable({.upper_bound = 3.0});
        auto yv = m.add_variable({.upper_bound = 3.0});
        B.v = {x, yv};
        B.vn = {"x[0,3]", "y[0,3]"};
        m.set_objective(-1 * x - yv);
        B.c = {m.add_constraint(x + 0 * yv <= 1)};
        B.cn = {"x<=1"};
        m.solve();
        dump("D7c y has no element", B);
    }
    {
        lp B;
        build(B);
        B.m.solve();
        B.m.remove_variable(B.v[0]);  // x1 at upper
        B.m.solve();
        std::printf("wrapper status %s\n", wstatus(B.m));
        dump("D8 after remove_variable(x1) and re-solve", B);
    }
    {
        lp B;  // empty model
        auto * M = B.m.native_model();
        B.m.solve();
        std::printf("== D9 empty model solve: statusExists=%d wrapper=%s\n",
                    N.statusExists(M), wstatus(B.m));
    }

    std::printf("\n######## F: extended bases\n");
    {
        lp B;
        build(B);
        B.m.solve();
        auto * M = B.m.native_model();
        auto S = snapshot(B.m);
        B.m.add_constraint(B.v[2] + B.v[3] <= 50);  // slack at x
        B.m.add_column({{B.c[6], 1.0}}, {.obj_coef = 5.0, .lower_bound = 0.0});
        B.vn.push_back("new col");
        B.cn.push_back("new row");
        // extended array: S cols + new col atLower, S rows + new row basic
        std::vector<unsigned char> ext(S.begin(), S.begin() + 11);
        ext.push_back(3);
        ext.insert(ext.end(), S.begin() + 11, S.end());
        ext.push_back(1);
        N.copyinStatus(M, ext.data());
        N.setMaximumIterations(M, 0);
        B.m.solve();
        std::printf("== F1 solved A + new col(atLower) + new row(basic), "
                    "copyin, maxIter 0: wrapper=%s Clp_status=%d iterations=%d "
                    "obj=%g\n",
                    wstatus(B.m), api.status(M), N.numberIterations(M),
                    api.getObjValue(M));
        // fresh extended model C gets the extended array
        lp C;
        build(C);
        C.m.add_constraint(C.v[2] + C.v[3] <= 50);
        C.m.add_column({{C.c[6], 1.0}}, {.obj_coef = 5.0, .lower_bound = 0.0});
        auto * MC = C.m.native_model();
        N.copyinStatus(MC, ext.data());
        N.setMaximumIterations(MC, 0);
        C.m.solve();
        std::printf("== F2 fresh extended model, copyin(ext), maxIter 0: "
                    "wrapper=%s Clp_status=%d iterations=%d obj=%g\n",
                    wstatus(C.m), api.status(MC), N.numberIterations(MC),
                    api.getObjValue(MC));
        // free new column given atLower
        lp D;
        build(D);
        D.m.add_constraint(D.v[2] + D.v[3] <= 50);
        D.m.add_column({{D.c[6], 1.0}}, {.obj_coef = 0.0});  // free, 0 cost
        D.vn.push_back("new free col");
        D.cn.push_back("new row");
        auto * MD = D.m.native_model();
        N.copyinStatus(MD, ext.data());
        N.setMaximumIterations(MD, 0);
        D.m.solve();
        std::printf("== F3 new FREE col given atLower, maxIter 0: wrapper=%s "
                    "Clp_status=%d iterations=%d\n",
                    wstatus(D.m), api.status(MD), N.numberIterations(MD));
        dump("F3", D, true);
        // copyin with a SHORT array (old size) into extended fresh model
        lp E;
        build(E);
        E.m.add_constraint(E.v[2] + E.v[3] <= 50);
        E.m.add_column({{E.c[6], 1.0}}, {.obj_coef = 5.0, .lower_bound = 0.0});
        auto * ME = E.m.native_model();
        std::vector<unsigned char> shortb = S;  // 18 bytes for 20 entries
        shortb.reserve(shortb.size());
        std::printf("   (F4 short array copyin is an out-of-bounds read: "
                    "not run)\n");
        (void)ME;
        // subset: per-entity setter changes one entry only
        lp G;
        build(G);
        G.m.solve();
        auto * MG = G.m.native_model();
        auto before = snapshot(G.m);
        api.setColumnStatus(MG, 9, 2);
        auto after = snapshot(G.m);
        int diff = 0;
        for(std::size_t k = 0; k < before.size(); ++k)
            diff += before[k] != after[k];
        std::printf("== F5 per-entity setter touches %d entry\n", diff);
    }
    std::printf("\n######## W: write-side codes that need translation\n");
    {
        auto one = [&](const char * label, int idx, unsigned char code) {
            auto b = SA;
            b[idx] = code;
            lp B;
            build(B);
            auto * M = B.m.native_model();
            N.copyinStatus(M, b.data());
            N.setMaximumIterations(M, 0);
            B.m.solve();
            int after = idx < 11 ? api.getColumnStatus(M, idx)
                                 : api.getRowStatus(M, idx - 11);
            double val = idx < 11 ? api.primalColumnSolution(M)[idx]
                                  : N.getRowActivity(M)[idx - 11];
            std::printf("== %s: wrote %d -> code after solve %d (%s) value %g;"
                        " wrapper=%s iterations=%d obj=%g\n",
                        label, code, after, code_name(after), val,
                        wstatus(B.m), N.numberIterations(M),
                        api.getObjValue(M));
        };
        one("W1 x1[0,2] (opt atUpper) written isFree", 0, 0);
        one("W2 x10[0,inf) (opt atLower) written isFree", 9, 0);
        one("W3 x1[0,2] written superBasic", 0, 4);
        one("W4 x10[0,inf) written superBasic", 9, 4);
        one("W5 x2 fixed written atLower", 1, 3);
        one("W6 x2 fixed written atUpper", 1, 2);
        one("W7 x7 fixed written atLower", 6, 3);
        one("W8 eq row written atLower", 11 + 2, 3);
        one("W9 eq row written atUpper", 11 + 2, 2);
        one("W10 le row (opt atUpper) written isFree", 11 + 0, 0);
        one("W11 rngU row (opt atUpper) written isFree", 11 + 4, 0);
        one("W12 rngU row written superBasic", 11 + 4, 4);
        one("W13 x1[0,2] written atLower (valid, nonoptimal)", 0, 3);
    }
    {
        // bigger random LP: flag bits, #basic, codes 0/4 at limits
        auto run = [&](int iters) {
            lp B;
            auto & m = B.m;
            m.set_time_limit(std::chrono::duration<double>(10));
            unsigned s = 12345;
            auto rnd = [&] {
                s = s * 1103515245u + 12345u;
                return (s >> 16) % 1000 / 1000.0;
            };
            const int n = 60, r = 40;
            std::vector<model_variable_t<clp_lp>> xs;
            for(int j = 0; j < n; ++j) {
                if(j % 7 == 0)
                    xs.push_back(m.add_variable({.lower_bound = std::nullopt}));
                else if(j % 5 == 0)
                    xs.push_back(m.add_variable(
                        {.lower_bound = 1.0, .upper_bound = 1.0}));
                else
                    xs.push_back(m.add_variable(
                        {.lower_bound = 0.0, .upper_bound = 1 + 9 * rnd()}));
            }
            std::vector<double> oc(n);
            for(int j = 0; j < n; ++j) oc[j] = rnd() - 0.3;
            m.set_maximization();
            m.set_objective(xsum(std::views::iota(0, n),
                                 [&](int j) { return oc[j] * xs[j]; }));
            for(int i = 0; i < r; ++i) {
                std::vector<double> a(n, 0.0);
                for(int j = 0; j < n; ++j)
                    if(rnd() < 0.3) a[j] = rnd() + 0.1;
                for(int j = 0; j < n; j += 7) a[j] += 0.01;
                auto e = xsum(std::views::iota(0, n),
                              [&](int j) { return a[j] * xs[j]; });
                if(i % 4 == 0) m.add_constraint(e <= 10 + 10 * rnd());
                else if(i % 4 == 1) m.add_constraint(e >= -5 * rnd());
                else if(i % 4 == 2)
                    m.add_ranged_constraint(e, 1.0, 8 + 5 * rnd());
                else m.add_constraint(e == 4 + rnd());
            }
            auto * M = m.native_model();
            if(iters >= 0) N.setMaximumIterations(M, iters);
            m.solve();
            int nc = api.getNumCols(M), nr = api.getNumRows(M);
            const unsigned char * raw = N.statusArray(M);
            int hist[8] = {}, flags = 0, nb = 0;
            for(int k = 0; k < nc + nr; ++k) {
                hist[raw[k] & 7]++;
                flags |= raw[k] & ~7;
                nb += (raw[k] & 7) == 1;
            }
            std::printf("== R maxIter %d: wrapper=%s iterations=%d obj=%g "
                        "#basic=%d rows=%d codes0..5: %d %d %d %d %d %d "
                        "flag bits seen: 0x%02x\n",
                        iters, wstatus(m), N.numberIterations(M),
                        api.getObjValue(M), nb, nr, hist[0], hist[1], hist[2],
                        hist[3], hist[4], hist[5], flags);
        };
        run(-1);
        run(1);
        run(5);
        run(15);
    }
    {
        lp B;
        build(B);
        B.m.solve();
        auto * M = B.m.native_model();
        auto before = B.m.get_solution();
        auto b = SA;
        b[0] = 3;  // x1 atLower
        b[9] = 2;  // x10 atUpper on inf
        N.copyinStatus(M, b.data());
        auto after = B.m.get_solution();
        bool same = true;
        for(auto v : B.v) same = same && before[v] == after[v];
        std::printf("== X1 copyinStatus on a solved model leaves get_solution "
                    "unchanged: %s; wrapper=%s; Clp_status=%d\n",
                    same ? "yes" : "no", wstatus(B.m), api.status(M));
        N.copyinStatus(M, nullptr);
        std::printf("== X2 copyinStatus(nullptr): statusExists=%d\n",
                    N.statusExists(M));
        B.m.solve();
        std::printf("   then solve: wrapper=%s iterations=%d (cold %d)\n",
                    wstatus(B.m), N.numberIterations(M), cold_iterations);
    }
    std::printf("\nDONE\n");
    return 0;
}
