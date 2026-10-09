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
    std::printf("GLPK version %s\n", api.version());
    auto optimal_C_basis = [&] {
        glpk_lp A(api);
        build_C(A);
        A.solve();
        return read_basis(A);
    }();

    std::printf("\n=== H1: glp_factorize as a set_basis validator ===\n");
    {
        glpk_lp B(api);
        build_C(B);
        P * p = B.native_model();
        write_basis(B, optimal_C_basis);
        int r = X.factorize(p);
        std::printf("  fresh B + optimal basis: glp_factorize -> %s, "
                    "bf_exists=%d glp_get_status=%s col_prim(b)=%g\n",
                    ret_name(r), X.bf_exists(p),
                    gstatus_name(api.get_status(p)), api.get_col_prim(p, 2));
        B.solve();
        std::printf("  then wrapper solve: %s iterations=%d\n",
                    wstatus(B.get_status()), X.get_it_cnt(p));
    }
    {
        glpk_lp B(api);
        auto H = build_C(B);
        B.solve();
        P * p = B.native_model();
        const auto & a = api;
        auto saved = read_basis(B);
        // singular: k (in no row) basic instead of R5
        a.set_col_stat(p, B.native_id(H.k), g::GLP_BS);
        a.set_row_stat(p, B.native_id(H.R5), g::GLP_NU);
        int r = X.factorize(p);
        std::printf("  solved B + singular basis: glp_factorize -> %s "
                    "bf_exists=%d glp_get_status=%s col_prim(b)=%g wrapper "
                    "status=%s\n",
                    ret_name(r), X.bf_exists(p),
                    gstatus_name(a.get_status(p)), a.get_col_prim(p, 2),
                    wstatus(B.get_status()));
        write_basis(B, saved);
        r = X.factorize(p);
        int it0 = X.get_it_cnt(p);
        B.solve();
        std::printf("  restored + glp_factorize -> %s; solve: %s extra "
                    "iterations=%d\n",
                    ret_name(r), wstatus(B.get_status()),
                    X.get_it_cnt(p) - it0);
        a.set_col_stat(p, B.native_id(H.a), g::GLP_BS);  // count + 1
        r = X.factorize(p);
        std::printf("  count+1: glp_factorize -> %s\n", ret_name(r));
    }
    std::printf("\n=== H2: glp_warm_up side effects on a solved model ===\n");
    {
        glpk_lp B(api);
        auto H = build_C(B);
        B.solve();
        P * p = B.native_model();
        // swap to another valid, non-optimal basis: std basis
        X.std_basis(p);
        int r = X.warm_up(p);
        std::printf("  std basis + glp_warm_up -> %s; glp_get_status=%s "
                    "obj=%g col_prim(b)=%g; wrapper status=%s "
                    "wrapper solution value=%g solution[b]=%g\n",
                    ret_name(r), gstatus_name(api.get_status(p)),
                    api.get_obj_val(p), api.get_col_prim(p, 2),
                    wstatus(B.get_status()), B.get_solution_value(),
                    B.get_solution()[H.b]);
    }
    {
        glpk_lp B(api);
        auto H = build_C(B);
        B.solve();
        P * p = B.native_model();
        X.std_basis(p);
        int r = X.factorize(p);
        std::printf("  std basis + glp_factorize -> %s; glp_get_status=%s "
                    "obj=%g col_prim(b)=%g (solution kept)\n",
                    ret_name(r), gstatus_name(api.get_status(p)),
                    api.get_obj_val(p), api.get_col_prim(p, 2));
        (void)H;
    }
    std::printf("\n=== H3: term_out silences glp_adv_basis ===\n");
    {
        glpk_lp B(api);
        build_C(B);
        std::printf("  -- adv_basis with terminal output on:\n");
        std::fflush(stdout);
        X.adv_basis(B.native_model(), 0);
        int old = api.term_out(0);
        std::printf("  -- adv_basis with glp_term_out(GLP_OFF) (old=%d):\n",
                    old);
        std::fflush(stdout);
        X.adv_basis(B.native_model(), 0);
        X.cpx_basis(B.native_model());
        api.term_out(old);
        std::printf("  -- end\n");
    }
    std::printf("\n=== H4: wrapper solve after crossed bounds keeps basis ===\n");
    {
        glpk_lp B(api);
        auto H = build_C(B);
        B.solve();
        auto before = read_basis(B);
        P * p = B.native_model();
        api.set_col_bnds(p, B.native_id(H.h), g::GLP_DB, 5, 3);
        B.solve();
        auto after = read_basis(B);
        std::printf("  crossed h bounds: wrapper %s; basis unchanged=%d\n",
                    wstatus(B.get_status()),
                    int(before.cols == after.cols && before.rows == after.rows));
    }
    std::printf("\n=== H5: time limit stop mid-search then read ===\n");
    {
        glpk_lp m(api);
        build_random(m, 300, 400, 7);
        m.set_time_limit(std::chrono::duration<double>(0.001));
        m.solve();
        std::printf("  300x400 tm 1ms: %s\n", wstatus(m.get_status()));
        summary(m, "after time limit");
    }
    std::printf("\ndone\n");
    return 0;
}
