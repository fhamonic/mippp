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
    using namespace operators;
    const glpk_api & api = glpk_api::load();
    load_extra(api);
    std::printf("GLPK version %s\n", api.version());
    std::printf("\n=== I1: glp_factorize on m == 0 ===\n");
    in_child("m==0, no columns basic", [&] {
        glpk_lp m(api);
        m.add_variable({.obj_coef = 1, .lower_bound = 0, .upper_bound = 5});
        P * p = m.native_model();
        std::printf("  (child) bf_exists=%d factorize -> %s bf_exists=%d\n",
                    X.bf_exists(p), ret_name(X.factorize(p)), X.bf_exists(p));
    });
    in_child("m==0, a column basic", [&] {
        glpk_lp m(api);
        m.add_variable({.obj_coef = 1, .lower_bound = 0, .upper_bound = 5});
        P * p = m.native_model();
        api.set_col_stat(p, 1, g::GLP_BS);
        std::printf("  (child) factorize -> %s\n", ret_name(X.factorize(p)));
        m.set_time_limit(ten_s);
        m.solve();
        std::printf("  (child) solve -> %s col stat=%s\n",
                    wstatus(m.get_status()), stat_name(api.get_col_stat(p, 1)));
    });
    in_child("m==0, no variables at all", [&] {
        glpk_lp m(api);
        P * p = m.native_model();
        std::printf("  (child) factorize -> %s\n", ret_name(X.factorize(p)));
    });
    std::printf("\n=== I2: glp_factorize on rows with nnz == 0 ===\n");
    in_child("nnz==0, column basic, row nonbasic", [&] {
        glpk_lp m(api);
        m.set_maximization();
        auto x = m.add_variable({.obj_coef = 1, .lower_bound = 0, .upper_bound = 5});
        m.add_constraint(0 * x <= 3);
        P * p = m.native_model();
        std::printf("  (child) nnz=%d\n", api.get_num_nz(p));
        api.set_col_stat(p, 1, g::GLP_BS);
        api.set_row_stat(p, 1, g::GLP_NU);
        std::printf("  (child) factorize -> %s\n", ret_name(X.factorize(p)));
        m.set_time_limit(ten_s);
        m.solve();
        std::printf("  (child) solve -> %s x=%s row=%s\n", wstatus(m.get_status()),
                    stat_name(api.get_col_stat(p, 1)), stat_name(api.get_row_stat(p, 1)));
    });
    in_child("nnz==0, valid (row basic)", [&] {
        glpk_lp m(api);
        auto x = m.add_variable({.obj_coef = 1, .lower_bound = 0, .upper_bound = 5});
        m.add_constraint(0 * x <= 3);
        P * p = m.native_model();
        std::printf("  (child) factorize -> %s\n", ret_name(X.factorize(p)));
    });
    in_child("nnz==0, count wrong (col and row basic)", [&] {
        glpk_lp m(api);
        auto x = m.add_variable({.obj_coef = 1, .lower_bound = 0, .upper_bound = 5});
        m.add_constraint(0 * x <= 3);
        P * p = m.native_model();
        api.set_col_stat(p, 1, g::GLP_BS);
        std::printf("  (child) factorize -> %s\n", ret_name(X.factorize(p)));
        m.set_time_limit(ten_s);
        m.solve();
        std::printf("  (child) solve -> %s\n", wstatus(m.get_status()));
    });
    std::printf("\ndone\n");
    return 0;
}
