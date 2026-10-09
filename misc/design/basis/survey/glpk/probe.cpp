// GLPK basis survey probe: builds models with MIP++ (glpk_lp) and reaches the
// native basis routines through native_api()/native_model(); routines the api
// struct lacks are dlsym'ed from api.library_path().
#include <dlfcn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <random>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/quadratic_expression.hpp"
#include "mippp/solvers/glpk/all.hpp"

using namespace mippp;
namespace g = mippp::glpk::impl::v1;
using P = g::glp_prob;

struct extra_api {
    double (*get_row_prim)(P *, int);
    int (*get_it_cnt)(P *);
    int (*warm_up)(P *);
    int (*bf_exists)(P *);
    int (*factorize)(P *);
    void (*std_basis)(P *);
    void (*adv_basis)(P *, int);
    void (*cpx_basis)(P *);
    int (*interior)(P *, const void *);
    void (*init_iptcp)(void *);
    int (*ipt_status)(P *);
};
static extra_api X;

template <typename F>
static F sym(void * h, const char * name) {
    void * s = dlsym(h, name);
    if(!s) {
        std::printf("MISSING SYMBOL %s\n", name);
        std::exit(2);
    }
    return reinterpret_cast<F>(s);
}

static void load_extra(const glpk_api & api) {
    void * h = dlopen(api.library_path().c_str(), RTLD_NOW | RTLD_NOLOAD);
    if(!h) h = dlopen(api.library_path().c_str(), RTLD_NOW);
    X.get_row_prim = sym<decltype(X.get_row_prim)>(h, "glp_get_row_prim");
    X.get_it_cnt = sym<decltype(X.get_it_cnt)>(h, "glp_get_it_cnt");
    X.warm_up = sym<decltype(X.warm_up)>(h, "glp_warm_up");
    X.bf_exists = sym<decltype(X.bf_exists)>(h, "glp_bf_exists");
    X.factorize = sym<decltype(X.factorize)>(h, "glp_factorize");
    X.std_basis = sym<decltype(X.std_basis)>(h, "glp_std_basis");
    X.adv_basis = sym<decltype(X.adv_basis)>(h, "glp_adv_basis");
    X.cpx_basis = sym<decltype(X.cpx_basis)>(h, "glp_cpx_basis");
    X.interior = sym<decltype(X.interior)>(h, "glp_interior");
    X.init_iptcp = sym<decltype(X.init_iptcp)>(h, "glp_init_iptcp");
    X.ipt_status = sym<decltype(X.ipt_status)>(h, "glp_ipt_status");
}

static const char * stat_name(int s) {
    switch(s) {
        case g::GLP_BS: return "BS";
        case g::GLP_NL: return "NL";
        case g::GLP_NU: return "NU";
        case g::GLP_NF: return "NF";
        case g::GLP_NS: return "NS";
        default: return "??";
    }
}
static const char * type_name(int t) {
    switch(t) {
        case g::GLP_FR: return "FR";
        case g::GLP_LO: return "LO";
        case g::GLP_UP: return "UP";
        case g::GLP_DB: return "DB";
        case g::GLP_FX: return "FX";
        default: return "??";
    }
}
static const char * gstatus_name(int s) {
    switch(s) {
        case g::GLP_UNDEF: return "UNDEF";
        case g::GLP_FEAS: return "FEAS";
        case g::GLP_INFEAS: return "INFEAS";
        case g::GLP_NOFEAS: return "NOFEAS";
        case g::GLP_OPT: return "OPT";
        case g::GLP_UNBND: return "UNBND";
        default: return "??";
    }
}
static const char * ret_name(int r) {
    switch(r) {
        case 0: return "0";
        case g::GLP_EBADB: return "GLP_EBADB";
        case g::GLP_ESING: return "GLP_ESING";
        case g::GLP_ECOND: return "GLP_ECOND";
        case g::GLP_EBOUND: return "GLP_EBOUND";
        case g::GLP_EFAIL: return "GLP_EFAIL";
        case g::GLP_EITLIM: return "GLP_EITLIM";
        case g::GLP_ETMLIM: return "GLP_ETMLIM";
        case g::GLP_ENOPFS: return "GLP_ENOPFS";
        case g::GLP_ENODFS: return "GLP_ENODFS";
        default: return "other";
    }
}
static const char * wstatus(const auto & st) {
    if(is<status::optimal>(st)) return "optimal";
    if(is<status::infeasible>(st)) return "infeasible";
    if(is<status::unbounded>(st)) return "unbounded";
    if(is<status::limit_reached>(st)) return "limit_reached";
    if(is<status::time_limit>(st)) return "time_limit";
    if(is<status::failed>(st)) return "failed";
    if(is<status::numerical_failure>(st)) return "numerical_failure";
    if(is<status::unknown>(st)) return "unknown";
    return "?";
}

struct basis_t {
    std::vector<int> cols, rows;
};
static basis_t read_basis(const glpk_lp & m) {
    const auto & a = m.native_api();
    P * p = m.native_model();
    basis_t b;
    for(int j = 1; j <= a.get_num_cols(p); ++j)
        b.cols.push_back(a.get_col_stat(p, j));
    for(int i = 1; i <= a.get_num_rows(p); ++i)
        b.rows.push_back(a.get_row_stat(p, i));
    return b;
}
static void write_basis(const glpk_lp & m, const basis_t & b) {
    const auto & a = m.native_api();
    P * p = m.native_model();
    for(std::size_t j = 0; j < b.cols.size(); ++j)
        a.set_col_stat(p, static_cast<int>(j) + 1, b.cols[j]);
    for(std::size_t i = 0; i < b.rows.size(); ++i)
        a.set_row_stat(p, static_cast<int>(i) + 1, b.rows[i]);
}
static int count_basic(const glpk_lp & m) {
    auto b = read_basis(m);
    int n = 0;
    for(int s : b.cols) n += (s == g::GLP_BS);
    for(int s : b.rows) n += (s == g::GLP_BS);
    return n;
}
static void summary(const glpk_lp & m, const char * label) {
    const auto & a = m.native_api();
    P * p = m.native_model();
    std::printf(
        "  [%s] rows=%d cols=%d #basic=%d bf_exists=%d glp_get_status=%s "
        "prim=%s dual=%s it_cnt=%d obj=%.6g\n",
        label, a.get_num_rows(p), a.get_num_cols(p), count_basic(m),
        X.bf_exists(p), gstatus_name(a.get_status(p)),
        gstatus_name(a.get_prim_stat(p)), gstatus_name(a.get_dual_stat(p)),
        X.get_it_cnt(p), a.get_obj_val(p));
}
static void dump(const glpk_lp & m, const char * label,
                 const std::vector<const char *> & cnames = {},
                 const std::vector<const char *> & rnames = {}) {
    const auto & a = m.native_api();
    P * p = m.native_model();
    summary(m, label);
    for(int j = 1; j <= a.get_num_cols(p); ++j) {
        const double lb = a.get_col_lb(p, j), ub = a.get_col_ub(p, j);
        std::printf(
            "    col %-2d %-4s type=%s lb=%-8.3g ub=%-8.3g stat=%s(%d) "
            "value=%.6g redcost=%.6g\n",
            j, j - 1 < (int)cnames.size() ? cnames[j - 1] : "",
            type_name(a.get_col_type(p, j)), lb, ub,
            stat_name(a.get_col_stat(p, j)), a.get_col_stat(p, j),
            a.get_col_prim(p, j), a.get_col_dual(p, j));
    }
    for(int i = 1; i <= a.get_num_rows(p); ++i) {
        const double lb = a.get_row_lb(p, i), ub = a.get_row_ub(p, i);
        std::printf(
            "    row %-2d %-4s type=%s lb=%-8.3g ub=%-8.3g stat=%s(%d) "
            "activity=%.6g dual=%.6g\n",
            i, i - 1 < (int)rnames.size() ? rnames[i - 1] : "",
            type_name(a.get_row_type(p, i)), lb, ub,
            stat_name(a.get_row_stat(p, i)), a.get_row_stat(p, i),
            X.get_row_prim(p, i), a.get_row_dual(p, i));
    }
}

// glpk_lp's private model_params, mirrored from glpk_lp.hpp:31-46
static g::glp_smcp wrapper_params(const glpk_api & api) {
    g::glp_smcp prm;
    api.init_smcp(&prm);
    prm.msg_lev = g::GLP_MSG_OFF;
    prm.meth = g::GLP_PRIMAL;
    prm.pricing = g::GLP_PT_STD;
    prm.r_test = g::GLP_RT_STD;
    prm.tol_bnd = 1e-7;
    prm.tol_dj = 1e-7;
    prm.tol_piv = 1e-10;
    prm.obj_ll = std::numeric_limits<double>::lowest();
    prm.obj_ul = std::numeric_limits<double>::max();
    prm.it_lim = std::numeric_limits<int>::max();
    prm.tm_lim = 10000;  // finite guard for the probe
    prm.presolve = 0;
    prm.excl = 0;
    prm.shift = 0;
    prm.aorn = g::GLP_USE_AT;
    return prm;
}

static const std::chrono::duration<double> ten_s{10.0};

// ---------------------------------------------------------------- model C
struct C_handles {
    model_variable_t<glpk_lp> a, b, c, d, e, f, z, gg, h, k, w;
    model_constraint_t<glpk_lp> R1, R2, R3, R4, R5, R6;
};
static const std::vector<const char *> C_cols = {
    "a", "b", "c", "d", "e", "f", "z", "g", "h", "k", "w"};
static const std::vector<const char *> C_rows = {
    "R1<=", "R2>=", "R3==", "R4rg", "R5sl", "R6rg"};

static C_handles build_C(glpk_lp & m) {
    using namespace operators;
    constexpr auto none = std::nullopt;
    C_handles H;
    m.set_maximization();
    H.a = m.add_variable({.obj_coef = 5, .lower_bound = 0, .upper_bound = 4});
    H.b = m.add_variable({.obj_coef = 1, .lower_bound = 0});
    H.c = m.add_variable({.obj_coef = -1, .lower_bound = none});
    H.d = m.add_variable({.obj_coef = -2, .lower_bound = 0});
    H.e = m.add_variable({.obj_coef = 2, .lower_bound = 0});
    H.f = m.add_variable({.obj_coef = 0, .lower_bound = 2, .upper_bound = 2});
    H.z = m.add_variable({.obj_coef = 0, .lower_bound = none});
    H.gg = m.add_variable({.obj_coef = -3, .lower_bound = 0});
    H.h = m.add_variable({.obj_coef = -1, .lower_bound = 1, .upper_bound = 7});
    H.k = m.add_variable(
        {.obj_coef = 1, .lower_bound = none, .upper_bound = 3});
    H.w = m.add_variable({.obj_coef = -1, .lower_bound = 0});
    H.R1 = m.add_constraint(H.b + H.a <= 10);
    H.R2 = m.add_constraint(H.c + H.d >= 3);
    H.R3 = m.add_constraint(H.d - H.e == 1);
    H.R4 = m.add_constraint(H.e + H.f <= 6);
    m.set_constraint_lower_bound(H.R4, 2);  // ranged [2, 6]
    H.R5 = m.add_constraint(H.b + H.z + H.gg <= 100);
    H.R6 = m.add_constraint(H.w - H.h >= 2);
    m.set_constraint_upper_bound(H.R6, 9);  // ranged [2, 9]
    m.set_time_limit(ten_s);
    return H;
}

// ---------------------------------------------------------- random LP
static void build_random(glpk_lp & m, int rows, int cols, unsigned seed) {
    using namespace operators;
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> U(0.0, 1.0);
    m.set_maximization();
    std::vector<model_variable_t<glpk_lp>> x;
    for(int j = 0; j < cols; ++j)
        x.push_back(m.add_variable({.obj_coef = 1.0 + 9.0 * U(rng),
                                    .lower_bound = 0,
                                    .upper_bound = 10}));
    for(int i = 0; i < rows; ++i) {
        // half-dense nonnegative row
        std::vector<double> coefs(cols);
        for(int j = 0; j < cols; ++j) coefs[j] = (U(rng) < 0.5) ? U(rng) : 0.0;
        auto c = m.add_constraint(
            xsum(std::views::iota(0, cols),
                 [&](int j) { return coefs[j] * x[j]; }) <=
            20.0 + 30.0 * U(rng));
        (void)c;
    }
    m.set_time_limit(ten_s);
}

// fork a child that runs f; report how it ended
template <typename F>
static void in_child(const char * label, F f) {
    std::fflush(stdout);
    pid_t pid = fork();
    if(pid == 0) {
        f();
        std::fflush(stdout);
        _exit(0);
    }
    int ws = 0;
    waitpid(pid, &ws, 0);
    if(WIFSIGNALED(ws))
        std::printf("  [%s] child killed by signal %d\n", label, WTERMSIG(ws));
    else
        std::printf("  [%s] child exited with code %d\n", label,
                    WEXITSTATUS(ws));
}

int main() {
    const glpk_api & api = glpk_api::load();
    load_extra(api);
    std::printf("GLPK version %s, library %s\n", api.version(),
                api.library_path().c_str());

    // ============================================================ C + D1/D2
    std::printf("\n=== C/D1: model C before any solve ===\n");
    {
        glpk_lp m(api);
        build_C(m);
        dump(m, "fresh, never solved", C_cols, C_rows);
        std::printf("  wrapper status: %s\n", wstatus(m.get_status()));
    }
    std::printf("\n=== C/D2: model C after wrapper solve() ===\n");
    {
        glpk_lp m(api);
        auto H = build_C(m);
        m.solve();
        std::printf("  wrapper status: %s, obj %.6g\n", wstatus(m.get_status()),
                    m.get_solution_value());
        dump(m, "after optimal solve", C_cols, C_rows);
        (void)H;
    }
    std::printf("\n=== C': model C minimized variant (objective negated) ===\n");
    {
        glpk_lp m(api);
        build_C(m);
        // same optimum as maximizing: negate every coefficient and minimize
        const auto & a = m.native_api();
        P * p = m.native_model();
        for(int j = 1; j <= a.get_num_cols(p); ++j)
            a.set_obj_coef(p, j, -a.get_obj_coef(p, j));
        m.set_minimization();
        m.solve();
        std::printf("  wrapper status: %s\n", wstatus(m.get_status()));
        dump(m, "minimize -obj", C_cols, C_rows);
    }

    // ============================================================ D3/D4
    std::printf("\n=== D3: infeasible LP ===\n");
    {
        using namespace operators;
        glpk_lp m(api);
        auto x = m.add_variable();
        auto y = m.add_variable();
        m.set_maximization();
        m.set_objective(x + 2 * y);
        m.add_constraint(x + y <= 4);
        m.add_constraint(x + y >= 6);
        m.add_constraint(x - y <= 1);
        m.set_time_limit(ten_s);
        dump(m, "before solve");
        m.solve();
        std::printf("  wrapper status: %s\n", wstatus(m.get_status()));
        dump(m, "after infeasible solve");
    }
    std::printf("\n=== D4: unbounded LP ===\n");
    {
        using namespace operators;
        glpk_lp m(api);
        auto x = m.add_variable();
        auto y = m.add_variable();
        m.set_maximization();
        m.set_objective(x + y);
        m.add_constraint(x - y <= 1);
        m.add_constraint(x + 2 * y >= 1);
        m.set_time_limit(ten_s);
        m.solve();
        std::printf("  wrapper status: %s\n", wstatus(m.get_status()));
        dump(m, "after unbounded solve");
    }

    // ============================================================ D5
    std::printf("\n=== D5: limits ===\n");
    {
        glpk_lp m(api);
        build_random(m, 40, 60, 1);
        m.solve();
        std::printf("  reference full solve: %s\n", wstatus(m.get_status()));
        summary(m, "random 40x60 full");
    }
    {
        glpk_lp m(api);
        build_random(m, 40, 60, 1);
        m.set_time_limit(std::chrono::duration<double>(0.0));
        m.solve();
        std::printf("  wrapper set_time_limit(0): %s\n",
                    wstatus(m.get_status()));
        summary(m, "after tm_lim=0 stop");
    }
    {
        glpk_lp m(api);
        build_random(m, 40, 60, 1);
        auto prm = wrapper_params(api);
        prm.it_lim = 5;
        int ret = api.simplex(m.native_model(), &prm);
        std::printf("  direct glp_simplex it_lim=5 -> %s\n", ret_name(ret));
        summary(m, "after it_lim=5 stop");
        auto B5 = read_basis(m);
        // resume from the stopped basis
        auto prm2 = wrapper_params(api);
        int it0 = X.get_it_cnt(m.native_model());
        ret = api.simplex(m.native_model(), &prm2);
        std::printf("  resume without limit -> %s, extra iterations %d\n",
                    ret_name(ret), X.get_it_cnt(m.native_model()) - it0);
        (void)B5;
    }

    // ============================================================ D6
    std::printf("\n=== D6: modifications after an optimal solve ===\n");
    {
        using namespace operators;
        glpk_lp m(api);
        auto H = build_C(m);
        m.solve();
        summary(m, "optimal");
        const auto & a = m.native_api();
        P * p = m.native_model();
        auto nr = m.add_constraint(H.b + H.c <= 50);
        std::printf("  add row: new row stat=%s\n",
                    stat_name(a.get_row_stat(p, m.native_id(nr))));
        summary(m, "after add row");
        m.set_variable_upper_bound(H.a, 3);
        std::printf("  a ub 4->3 (type DB kept): a stat=%s\n",
                    stat_name(a.get_col_stat(p, m.native_id(H.a))));
        summary(m, "after bound change (type kept)");
        m.set_variable_upper_bound(H.a, m.infinity());
        std::printf("  a ub 3->inf (type DB->LO): a stat=%s\n",
                    stat_name(a.get_col_stat(p, m.native_id(H.a))));
        m.set_variable_upper_bound(H.a, 4);
        std::printf("  a ub inf->4 (type LO->DB): a stat=%s\n",
                    stat_name(a.get_col_stat(p, m.native_id(H.a))));
        m.set_variable_lower_bound(H.k, -1);
        std::printf("  k (UP, NU) lb -inf->-1 (type UP->DB): k stat=%s\n",
                    stat_name(a.get_col_stat(p, m.native_id(H.k))));
        m.set_variable_lower_bound(H.b, 6);
        m.set_variable_upper_bound(H.b, 6);
        std::printf("  b (BS) fixed at 6: b stat=%s\n",
                    stat_name(a.get_col_stat(p, m.native_id(H.b))));
        m.set_constraint_lower_bound(H.R1, 10);
        std::printf("  R1 (UP,NU) lb := 10 (type UP->FX): R1 stat=%s\n",
                    stat_name(a.get_row_stat(p, m.native_id(H.R1))));
        m.set_constraint_lower_bound(H.R1, m.infinity() * -1);
        std::printf("  R1 lb := -inf (type FX->UP): R1 stat=%s\n",
                    stat_name(a.get_row_stat(p, m.native_id(H.R1))));
        auto v1 = m.add_variable({.lower_bound = -2, .upper_bound = 5});
        auto v2 = m.add_variable({.lower_bound = -5, .upper_bound = 2});
        auto v3 = m.add_variable({.lower_bound = std::nullopt});
        auto v4 = m.add_variable({.lower_bound = 3, .upper_bound = 3});
        auto v5 = m.add_variable();
        auto v6 = m.add_variable(
            {.lower_bound = std::nullopt, .upper_bound = 1});
        auto v7 = m.add_column({{H.R1, 1.0}, {H.R2, 1.0}},
                               {.lower_bound = 0, .upper_bound = 1});
        std::printf(
            "  add cols: DB[-2,5]=%s DB[-5,2]=%s FR=%s FX=%s LO=%s UP=%s "
            "add_column DB[0,1]=%s\n",
            stat_name(a.get_col_stat(p, m.native_id(v1))),
            stat_name(a.get_col_stat(p, m.native_id(v2))),
            stat_name(a.get_col_stat(p, m.native_id(v3))),
            stat_name(a.get_col_stat(p, m.native_id(v4))),
            stat_name(a.get_col_stat(p, m.native_id(v5))),
            stat_name(a.get_col_stat(p, m.native_id(v6))),
            stat_name(a.get_col_stat(p, m.native_id(v7))));
        summary(m, "after all modifications, no re-solve");
        std::printf("  wrapper status still: %s\n", wstatus(m.get_status()));
    }

    // ============================================================ D7
    std::printf("\n=== D7: other algorithms / presolve ===\n");
    {
        glpk_lp m(api);
        build_C(m);
        m.solve();
        auto before = read_basis(m);
        alignas(16) unsigned char iptcp[2048] = {};
        X.init_iptcp(iptcp);
        *reinterpret_cast<int *>(iptcp) = 0;  // msg_lev = GLP_MSG_OFF
        int ret = X.interior(m.native_model(), iptcp);
        auto after = read_basis(m);
        std::printf(
            "  glp_interior after simplex -> ret %s ipt_status=%s; basis "
            "unchanged: %d\n",
            ret_name(ret), gstatus_name(X.ipt_status(m.native_model())),
            int(before.cols == after.cols && before.rows == after.rows));
        summary(m, "after interior");
    }
    {
        glpk_lp m(api);
        build_C(m);
        auto prm = wrapper_params(api);
        prm.presolve = 1;
        int ret = api.simplex(m.native_model(), &prm);
        std::printf("  presolve=1 fresh model C -> %s\n", ret_name(ret));
        dump(m, "after presolved optimal solve", C_cols, C_rows);
    }
    {
        using namespace operators;
        glpk_lp m(api);
        auto x = m.add_variable();
        auto y = m.add_variable();
        m.set_maximization();
        m.set_objective(x + 2 * y);
        m.add_constraint(x + y <= 4);
        m.add_constraint(x + y >= 6);
        m.add_constraint(x - y <= 1);
        // mark a distinctive (valid) basis first
        const auto & a = m.native_api();
        P * p = m.native_model();
        a.set_col_stat(p, 1, g::GLP_BS);
        a.set_row_stat(p, 1, g::GLP_NU);
        auto prm = wrapper_params(api);
        prm.presolve = 1;
        int ret = api.simplex(p, &prm);
        std::printf("  presolve=1 infeasible -> %s\n", ret_name(ret));
        dump(m, "after presolved infeasible solve (input basis x=BS,r1=NU)");
    }

    // ============================================================ E1
    std::printf("\n=== E1: warm start ===\n");
    for(int which = 0; which < 2; ++which) {
        const char * name = which == 0 ? "model C" : "random 40x60";
        auto build = [&](glpk_lp & m) {
            if(which == 0)
                build_C(m);
            else
                build_random(m, 40, 60, 1);
        };
        glpk_lp A(api);
        build(A);
        A.solve();
        const double objA = A.get_solution_value();
        std::printf("  %s: A from scratch: %s obj=%.9g iterations=%d\n", name,
                    wstatus(A.get_status()), objA,
                    X.get_it_cnt(A.native_model()));
        auto basis = read_basis(A);
        {
            glpk_lp B(api);
            build(B);
            write_basis(B, basis);
            std::printf("    B after write: bf_exists=%d #basic=%d\n",
                        X.bf_exists(B.native_model()), count_basic(B));
            B.solve();
            std::printf(
                "    B wrapper solve() with A's basis: %s obj=%.9g "
                "iterations=%d same basis=%d\n",
                wstatus(B.get_status()), B.get_solution_value(),
                X.get_it_cnt(B.native_model()),
                int(read_basis(B).cols == basis.cols &&
                    read_basis(B).rows == basis.rows));
        }
        {
            glpk_lp B(api);
            build(B);
            write_basis(B, basis);
            auto prm = wrapper_params(api);
            prm.it_lim = 0;
            int ret = api.simplex(B.native_model(), &prm);
            std::printf(
                "    B direct glp_simplex it_lim=0 with A's basis -> %s, "
                "glp_get_status=%s obj=%.9g iterations=%d\n",
                ret_name(ret), gstatus_name(api.get_status(B.native_model())),
                api.get_obj_val(B.native_model()),
                X.get_it_cnt(B.native_model()));
        }
        {
            glpk_lp B(api);
            build(B);
            auto prm = wrapper_params(api);
            prm.it_lim = 0;
            int ret = api.simplex(B.native_model(), &prm);
            std::printf(
                "    B direct glp_simplex it_lim=0 WITHOUT basis -> %s, "
                "glp_get_status=%s obj=%.9g\n",
                ret_name(ret), gstatus_name(api.get_status(B.native_model())),
                api.get_obj_val(B.native_model()));
        }
        {
            glpk_lp B(api);
            build(B);
            write_basis(B, basis);
            auto prm = wrapper_params(api);
            prm.meth = g::GLP_DUAL;
            prm.it_lim = 0;
            int ret = api.simplex(B.native_model(), &prm);
            std::printf(
                "    B GLP_DUAL it_lim=0 with A's basis -> %s, "
                "glp_get_status=%s\n",
                ret_name(ret), gstatus_name(api.get_status(B.native_model())));
        }
        {
            glpk_lp B(api);
            build(B);
            write_basis(B, basis);
            int ret = X.warm_up(B.native_model());
            std::printf(
                "    B glp_warm_up with A's basis -> %s, glp_get_status=%s "
                "obj=%.9g iterations=%d\n",
                ret_name(ret), gstatus_name(api.get_status(B.native_model())),
                api.get_obj_val(B.native_model()),
                X.get_it_cnt(B.native_model()));
        }
        {
            glpk_lp B(api);
            build(B);
            write_basis(B, basis);
            auto prm = wrapper_params(api);
            prm.presolve = 1;
            int ret = api.simplex(B.native_model(), &prm);
            std::printf(
                "    B presolve=1 with A's basis -> %s, glp_get_status=%s "
                "iterations=%d (presolve ignores the basis)\n",
                ret_name(ret), gstatus_name(api.get_status(B.native_model())),
                X.get_it_cnt(B.native_model()));
        }
        {
            glpk_lp B(api);
            build(B);
            auto prm = wrapper_params(api);
            prm.presolve = 1;
            int ret = api.simplex(B.native_model(), &prm);
            std::printf(
                "    B presolve=1 without basis -> %s, iterations=%d\n",
                ret_name(ret), X.get_it_cnt(B.native_model()));
        }
        {
            // re-solve A itself: iteration counter is cumulative
            int it0 = X.get_it_cnt(A.native_model());
            A.solve();
            std::printf(
                "    A re-solved in place: %s, it_cnt %d -> %d (cumulative "
                "counter)\n",
                wstatus(A.get_status()), it0, X.get_it_cnt(A.native_model()));
        }
    }

    // ============================================================ E2
    std::printf("\n=== E2: write a basis into A after a bound change ===\n");
    {
        glpk_lp A(api);
        auto H = build_C(A);
        A.solve();
        auto basis = read_basis(A);
        const auto & a = A.native_api();
        P * p = A.native_model();
        // type-preserving change: a ub 4 -> 3 (a is NU)
        A.set_variable_upper_bound(H.a, 3);
        write_basis(A, basis);
        int it0 = X.get_it_cnt(p);
        A.solve();
        std::printf(
            "  a ub 4->3, write basis, solve: %s obj=%.6g extra "
            "iterations=%d a=%s\n",
            wstatus(A.get_status()), A.get_solution_value(),
            X.get_it_cnt(p) - it0, stat_name(a.get_col_stat(p, 1)));
        // type change: a ub -> inf. a was NU, becomes NL; writing NU back
        A.set_variable_upper_bound(H.a, A.infinity());
        std::printf("  a ub ->inf: a stat after bound change=%s\n",
                    stat_name(a.get_col_stat(p, 1)));
        a.set_col_stat(p, 1, g::GLP_NU);
        std::printf("  wrote NU on a (now LO): reads back %s\n",
                    stat_name(a.get_col_stat(p, 1)));
        it0 = X.get_it_cnt(p);
        A.solve();
        std::printf("  solve: %s extra iterations=%d\n",
                    wstatus(A.get_status()), X.get_it_cnt(p) - it0);
    }
    {
        // tighten a basic variable so that the old basis is primal infeasible
        glpk_lp A(api);
        auto H = build_C(A);
        A.solve();
        auto basis = read_basis(A);
        P * p = A.native_model();
        A.set_variable_upper_bound(H.b, 5);  // b was 6, basic
        write_basis(A, basis);
        int it0 = X.get_it_cnt(p);
        A.solve();
        std::printf(
            "  b ub ->5 (basic b=6 now infeasible), write basis, solve: %s "
            "obj=%.6g extra iterations=%d\n",
            wstatus(A.get_status()), A.get_solution_value(),
            X.get_it_cnt(p) - it0);
    }

    // ============================================================ E3
    std::printf("\n=== E3: invalid bases ===\n");
    auto optimal_C_basis = [&] {
        glpk_lp A(api);
        build_C(A);
        A.solve();
        return read_basis(A);
    }();
    {
        glpk_lp B(api);
        auto H = build_C(B);
        write_basis(B, optimal_C_basis);
        const auto & a = B.native_api();
        P * p = B.native_model();
        a.set_col_stat(p, B.native_id(H.a), g::GLP_BS);  // one basic too many
        std::printf("  one basic too many: #basic=%d rows=%d\n",
                    count_basic(B), a.get_num_rows(p));
        int wr = X.warm_up(p);
        std::printf("  glp_warm_up -> %s\n", ret_name(wr));
        B.solve();
        std::printf("  wrapper solve -> %s, #basic after=%d\n",
                    wstatus(B.get_status()), count_basic(B));
        auto prm = wrapper_params(api);
        int ret = api.simplex(p, &prm);
        std::printf("  direct glp_simplex -> %s (again)\n", ret_name(ret));
        B.solve();
        std::printf("  wrapper solve a second time -> %s\n",
                    wstatus(B.get_status()));
        X.adv_basis(p, 0);
        B.solve();
        std::printf("  after glp_adv_basis: wrapper solve -> %s obj=%.6g\n",
                    wstatus(B.get_status()), B.get_solution_value());
    }
    {
        glpk_lp B(api);
        auto H = build_C(B);
        write_basis(B, optimal_C_basis);
        const auto & a = B.native_api();
        P * p = B.native_model();
        a.set_col_stat(p, B.native_id(H.b), g::GLP_NL);  // one basic missing
        std::printf("  one basic missing: #basic=%d\n", count_basic(B));
        auto prm = wrapper_params(api);
        int ret = api.simplex(p, &prm);
        std::printf("  direct glp_simplex -> %s\n", ret_name(ret));
    }
    {
        glpk_lp B(api);
        auto H = build_C(B);
        write_basis(B, optimal_C_basis);
        const auto & a = B.native_api();
        P * p = B.native_model();
        // k appears in no row: making it basic in place of R5 is singular
        a.set_col_stat(p, B.native_id(H.k), g::GLP_BS);
        a.set_row_stat(p, B.native_id(H.R5), g::GLP_NU);
        std::printf("  singular (right count): #basic=%d\n", count_basic(B));
        int wr = X.warm_up(p);
        std::printf("  glp_warm_up -> %s\n", ret_name(wr));
        B.solve();
        std::printf("  wrapper solve -> %s\n", wstatus(B.get_status()));
        auto prm = wrapper_params(api);
        int ret = api.simplex(p, &prm);
        std::printf("  direct glp_simplex -> %s (again)\n", ret_name(ret));
    }
    {
        glpk_lp B(api);
        auto H = build_C(B);
        const auto & a = B.native_api();
        P * p = B.native_model();
        struct {
            const char * what;
            bool col;
            int id;
            int code;
        } cases[] = {
            {"NU on LO column g", true, B.native_id(H.gg), g::GLP_NU},
            {"NL on FR column z", true, B.native_id(H.z), g::GLP_NL},
            {"NU on FR column z", true, B.native_id(H.z), g::GLP_NU},
            {"NS on FR column z", true, B.native_id(H.z), g::GLP_NS},
            {"NF on DB column a", true, B.native_id(H.a), g::GLP_NF},
            {"NS on DB column a", true, B.native_id(H.a), g::GLP_NS},
            {"NU on DB column a", true, B.native_id(H.a), g::GLP_NU},
            {"NL on FX column f", true, B.native_id(H.f), g::GLP_NL},
            {"NU on FX column f", true, B.native_id(H.f), g::GLP_NU},
            {"NL on UP column k", true, B.native_id(H.k), g::GLP_NL},
            {"NF on UP row R1", false, B.native_id(H.R1), g::GLP_NF},
            {"NL on UP row R1", false, B.native_id(H.R1), g::GLP_NL},
            {"NU on LO row R2", false, B.native_id(H.R2), g::GLP_NU},
            {"NL on FX row R3", false, B.native_id(H.R3), g::GLP_NL},
            {"NU on DB row R4", false, B.native_id(H.R4), g::GLP_NU},
            {"NF on DB row R4", false, B.native_id(H.R4), g::GLP_NF},
        };
        for(auto & cs : cases) {
            if(cs.col) {
                a.set_col_stat(p, cs.id, cs.code);
                std::printf("  %-20s wrote %s reads back %s\n", cs.what,
                            stat_name(cs.code),
                            stat_name(a.get_col_stat(p, cs.id)));
            } else {
                a.set_row_stat(p, cs.id, cs.code);
                std::printf("  %-20s wrote %s reads back %s\n", cs.what,
                            stat_name(cs.code),
                            stat_name(a.get_row_stat(p, cs.id)));
            }
        }
    }
    in_child("invalid stat code 6 on a column", [&] {
        glpk_lp B(api);
        build_C(B);
        B.native_api().set_col_stat(B.native_model(), 1, 6);
        std::printf("  (child) returned from set_col_stat(...,6)\n");
    });
    in_child("invalid stat code 0 on a row", [&] {
        glpk_lp B(api);
        build_C(B);
        B.native_api().set_row_stat(B.native_model(), 1, 0);
        std::printf("  (child) returned from set_row_stat(...,0)\n");
    });
    in_child("column index out of range", [&] {
        glpk_lp B(api);
        build_C(B);
        B.native_api().set_col_stat(B.native_model(), 99, g::GLP_BS);
        std::printf("  (child) returned\n");
    });
    in_child("get_col_stat index out of range", [&] {
        glpk_lp B(api);
        build_C(B);
        std::printf("  (child) got %d\n",
                    B.native_api().get_col_stat(B.native_model(), 99));
    });

    // ============================================================ F
    std::printf("\n=== F: extended bases ===\n");
    {
        using namespace operators;
        glpk_lp A(api);
        auto H = build_C(A);
        A.solve();
        auto basis = read_basis(A);
        P * p = A.native_model();
        // a new profitable column and a new binding row
        auto nv = A.add_variable(
            {.obj_coef = 3, .lower_bound = 0, .upper_bound = 2});
        auto nr = A.add_constraint(H.e + nv <= 5);
        const auto & a = A.native_api();
        std::printf("  after add: new col stat=%s new row stat=%s #basic=%d "
                    "rows=%d\n",
                    stat_name(a.get_col_stat(p, A.native_id(nv))),
                    stat_name(a.get_row_stat(p, A.native_id(nr))),
                    count_basic(A), a.get_num_rows(p));
        write_basis(A, basis);  // the old entries only
        int it0 = X.get_it_cnt(p);
        A.solve();
        std::printf("  re-solve in place with the old basis: %s obj=%.6g "
                    "extra iterations=%d\n",
                    wstatus(A.get_status()), A.get_solution_value(),
                    X.get_it_cnt(p) - it0);
        auto extended = read_basis(A);

        // fresh B = C + same additions, only the old entries written
        glpk_lp B(api);
        auto HB = build_C(B);
        auto nvB = B.add_variable(
            {.obj_coef = 3, .lower_bound = 0, .upper_bound = 2});
        B.add_constraint(HB.e + nvB <= 5);
        write_basis(B, basis);
        std::printf("  fresh B, old basis on old entries: #basic=%d\n",
                    count_basic(B));
        B.solve();
        std::printf("  B solve: %s obj=%.6g iterations=%d\n",
                    wstatus(B.get_status()), B.get_solution_value(),
                    X.get_it_cnt(B.native_model()));
        glpk_lp B2(api);
        auto HB2 = build_C(B2);
        auto nvB2 = B2.add_variable(
            {.obj_coef = 3, .lower_bound = 0, .upper_bound = 2});
        B2.add_constraint(HB2.e + nvB2 <= 5);
        B2.solve();
        std::printf("  B2 from scratch: %s iterations=%d\n",
                    wstatus(B2.get_status()), X.get_it_cnt(B2.native_model()));
        (void)extended;
    }
    {
        // subset write: columns only, rows left at their default (BS)
        glpk_lp B(api);
        build_C(B);
        const auto & a = B.native_api();
        P * p = B.native_model();
        for(std::size_t j = 0; j < optimal_C_basis.cols.size(); ++j)
            a.set_col_stat(p, static_cast<int>(j) + 1,
                           optimal_C_basis.cols[j]);
        std::printf("  columns-only write: #basic=%d rows=%d\n",
                    count_basic(B), a.get_num_rows(p));
        auto prm = wrapper_params(api);
        int ret = api.simplex(p, &prm);
        std::printf("  direct glp_simplex -> %s\n", ret_name(ret));
    }

    // ============================================================ G
    std::printf("\n=== G: trivial LP (no nonzeros) ignores the basis ===\n");
    {
        glpk_lp m(api);
        m.set_maximization();
        auto x = m.add_variable({.obj_coef = 1, .lower_bound = 0,
                                 .upper_bound = 5});
        auto y = m.add_variable({.obj_coef = -1, .lower_bound = 0,
                                 .upper_bound = 5});
        m.set_time_limit(ten_s);
        const auto & a = m.native_api();
        P * p = m.native_model();
        a.set_col_stat(p, m.native_id(x), g::GLP_BS);  // invalid: m == 0
        a.set_col_stat(p, m.native_id(y), g::GLP_NU);
        std::printf("  before: x=%s y=%s\n",
                    stat_name(a.get_col_stat(p, 1)),
                    stat_name(a.get_col_stat(p, 2)));
        m.solve();
        std::printf("  solve: %s x=%s y=%s\n", wstatus(m.get_status()),
                    stat_name(a.get_col_stat(p, 1)),
                    stat_name(a.get_col_stat(p, 2)));
    }
    {
        // row without terms: nnz == 0 path too, rows forced basic
        using namespace operators;
        glpk_lp m(api);
        m.set_maximization();
        auto x = m.add_variable({.obj_coef = 1, .lower_bound = 0,
                                 .upper_bound = 5});
        auto c = m.add_constraint(0 * x <= 3);
        m.set_time_limit(ten_s);
        const auto & a = m.native_api();
        P * p = m.native_model();
        std::printf("  nnz=%d\n", a.get_num_nz(p));
        a.set_col_stat(p, 1, g::GLP_BS);
        a.set_row_stat(p, 1, g::GLP_NU);
        m.solve();
        std::printf("  solve: %s x=%s row=%s\n", wstatus(m.get_status()),
                    stat_name(a.get_col_stat(p, 1)),
                    stat_name(a.get_row_stat(p, 1)));
        (void)c;
    }
    std::printf("\n=== G: moved-from / copy semantics: none (no copy) ===\n");
    std::printf("\ndone\n");
    return 0;
}
