#ifndef MIPPP_GLPK_v5_API_HPP
#define MIPPP_GLPK_v5_API_HPP

#if INCLUDE_GLPK_HEADER
#include "glpk.h"
#else
namespace fhamonic::mippp {
namespace glpk::v5 {

using glp_prob = struct glp_prob;

glp_prob * glp_create_prob(void);
void glp_delete_prob(glp_prob * P);
constexpr int GLP_EBADB = 0x01;    // invalid basis
constexpr int GLP_ESING = 0x02;    // singular matrix
constexpr int GLP_ECOND = 0x03;    // ill-conditioned matrix
constexpr int GLP_EBOUND = 0x04;   // invalid bounds
constexpr int GLP_EFAIL = 0x05;    // solver failed
constexpr int GLP_EOBJLL = 0x06;   // objective lower limit reached
constexpr int GLP_EOBJUL = 0x07;   // objective upper limit reached
constexpr int GLP_EITLIM = 0x08;   // iteration limit exceeded
constexpr int GLP_ETMLIM = 0x09;   // time limit exceeded
constexpr int GLP_ENOPFS = 0x0A;   // no primal feasible solution
constexpr int GLP_ENODFS = 0x0B;   // no dual feasible solution
constexpr int GLP_EROOT = 0x0C;    // root LP optimum not provided
constexpr int GLP_ESTOP = 0x0D;    // search terminated by application
constexpr int GLP_EMIPGAP = 0x0E;  // relative mip gap tolerance reached
constexpr int GLP_ENOFEAS = 0x0F;  // no primal/dual feasible solution
constexpr int GLP_ENOCVG = 0x10;   // no convergence
constexpr int GLP_EINSTAB = 0x11;  // numerical instability
constexpr int GLP_EDATA = 0x12;    // invalid data
constexpr int GLP_ERANGE = 0x13;   // result out of range

constexpr int GLP_MIN = 1;
constexpr int GLP_MAX = 2;
void glp_set_obj_dir(glp_prob * P, int dir);
int glp_get_obj_dir(glp_prob * P);

int glp_add_cols(glp_prob * P, int ncs);
void glp_set_mat_col(glp_prob * P, int j, int len, const int ind[],
                     const double val[]);
int glp_add_rows(glp_prob * P, int nrs);
void glp_set_mat_row(glp_prob * P, int i, int len, const int ind[],
                     const double val[]);

void glp_set_obj_coef(glp_prob * P, int j, double coef);
double glp_get_obj_coef(glp_prob * P, int j);
constexpr int GLP_FR = 1;  // free (unbounded) variable
constexpr int GLP_LO = 2;  // variable with lower bound
constexpr int GLP_UP = 3;  // variable with upper bound
constexpr int GLP_DB = 4;  // double-bounded variable
constexpr int GLP_FX = 5;  // fixed variable
void glp_set_col_bnds(glp_prob * P, int j, int type, double lb, double ub);
double glp_get_col_lb(glp_prob * P, int j);
double glp_get_col_ub(glp_prob * P, int j);
void glp_set_col_name(glp_prob * P, int j, const char * name);
const char * glp_get_col_name(glp_prob * P, int j);
constexpr int GLP_CV = 1;  // continuous variable
constexpr int GLP_IV = 2;  // integer variable
constexpr int GLP_BV = 3;  // binary variable
void glp_set_col_kind(glp_prob * P, int j, int kind);
int glp_get_col_kind(glp_prob * P, int j);

int glp_get_mat_row(glp_prob * P, int i, int ind[], double val[]);
int glp_get_row_type(glp_prob * P, int i);
void glp_set_row_bnds(glp_prob * P, int i, int type, double lb, double ub);
double glp_get_row_lb(glp_prob * P, int i);
double glp_get_row_ub(glp_prob * P, int i);
void glp_set_row_name(glp_prob * P, int i, const char * name);
const char * glp_get_row_name(glp_prob * P, int i);

int glp_get_num_cols(glp_prob * P);
int glp_get_num_rows(glp_prob * P);
int glp_get_num_nz(glp_prob * P);

constexpr int GLP_BS = 1;  // basic variable
constexpr int GLP_NL = 2;  // non-basic variable on lower bound
constexpr int GLP_NU = 3;  // non-basic variable on upper bound
constexpr int GLP_NF = 4;  // non-basic free (unbounded) variable
constexpr int GLP_NS = 5;  // non-basic fixed variable
void glp_set_row_stat(glp_prob * P, int i, int stat);
int glp_get_row_stat(glp_prob * P, int i);
void glp_set_col_stat(glp_prob * P, int j, int stat);
int glp_get_col_stat(glp_prob * P, int j);

constexpr int GLP_MSG_OFF = 0;     // msg_lev: no output
constexpr int GLP_MSG_ERR = 1;     // msg_lev: warning and error messages only
constexpr int GLP_MSG_ON = 2;      // msg_lev: normal output
constexpr int GLP_MSG_ALL = 3;     // msg_lev: full output
constexpr int GLP_MSG_DBG = 4;     // msg_lev: debug output
constexpr int GLP_PRIMAL = 1;      // meth: use primal simplex
constexpr int GLP_DUALP = 2;       // meth: use dual; if it fails, use primal
constexpr int GLP_DUAL = 3;        // meth: use dual simplex
constexpr int GLP_PT_STD = 0x11;   // pricing: standard (Dantzig's rule)
constexpr int GLP_PT_PSE = 0x22;   // pricing: projected steepest edge
constexpr int GLP_RT_STD = 0x11;   // r_test: standard (textbook)
constexpr int GLP_RT_HAR = 0x22;   // r_test: Harris' two-pass ratio test
constexpr int GLP_RT_FLIP = 0x33;  // r_test: long-step (flip-flop) ratio test
constexpr int GLP_USE_AT = 1;      // aorn
constexpr int GLP_USE_NT = 2;      // aorn
struct glp_smcp {
    int msg_lev;         // message level
    int meth;            // simplex method option
    int pricing;         // pricing technique
    int r_test;          // ratio test technique
    double tol_bnd;      // primal feasibility tolerance
    double tol_dj;       // dual feasibility tolerance
    double tol_piv;      // pivot tolerance
    double obj_ll;       // lower objective limit
    double obj_ul;       // upper objective limit
    int it_lim;          // simplex iteration limit
    int tm_lim;          // time limit, ms
    int out_frq;         // display output frequency, ms
    int out_dly;         // display output delay, ms
    int presolve;        // enable/disable using LP presolver
    int excl;            // exclude fixed non-basic variables
    int shift;           // shift bounds of variables to zero
    int aorn;            // option to use A or N
    double foo_bar[33];  // (reserved)
};
int glp_simplex(glp_prob * P, const glp_smcp * parm);

constexpr int GLP_UNDEF = 1;   // solution is undefined
constexpr int GLP_FEAS = 2;    // solution is feasible
constexpr int GLP_INFEAS = 3;  // solution is infeasible
constexpr int GLP_NOFEAS = 4;  // no feasible solution exists
constexpr int GLP_OPT = 5;     // solution is optimal
constexpr int GLP_UNBND = 6;   // solution is unbounded
int glp_get_status(glp_prob * P);
int glp_get_prim_stat(glp_prob * P);
int glp_get_dual_stat(glp_prob * P);
double glp_get_obj_val(glp_prob * P);
double glp_get_col_prim(glp_prob * P, int j);
double glp_get_row_dual(glp_prob * P, int i);

constexpr int GLP_BR_FFV = 1;  // br_tech: first fractional variable
constexpr int GLP_BR_LFV = 2;  // br_tech: last fractional variable
constexpr int GLP_BR_MFV = 3;  // br_tech: most fractional variable
constexpr int GLP_BR_DTH = 4;  // br_tech: heuristic by Driebeck and Tomlin
constexpr int GLP_BR_PCH = 5;  // br_tech: hybrid pseudocost heuristic
constexpr int GLP_BT_DFS = 1;  // bt_tech: depth first search
constexpr int GLP_BT_BFS = 2;  // bt_tech: breadth first search
constexpr int GLP_BT_BLB = 3;  // bt_tech: best local bound
constexpr int GLP_BT_BPH = 4;  // bt_tech: best projection heuristic
using glp_tree = struct glp_tree;
constexpr int GLP_PP_NONE = 0;  // disable preprocessing
constexpr int GLP_PP_ROOT = 1;  // preprocessing only on root level
constexpr int GLP_PP_ALL = 2;   // preprocessing on all levels
constexpr int GLP_ON = 1;       // enable something
constexpr int GLP_OFF = 0;      // disable something
struct glp_iocp {               // integer optimizer control parameters
    int msg_lev;                // message level (see glp_smcp)
    int br_tech;                // branching technique:
    int bt_tech;                // backtracking technique:
    double tol_int;             // mip.tol_int
    double tol_obj;             // mip.tol_obj
    int tm_lim;                 // mip.tm_lim (milliseconds)
    int out_frq;                // mip.out_frq (milliseconds)
    int out_dly;                // mip.out_dly (milliseconds)
    void (*cb_func)(glp_tree * T, void * info);  // mip.cb_func
    void * cb_info;                              // mip.cb_info
    int cb_size;                                 // mip.cb_size
    int pp_tech;                                 // preprocessing technique:
    double mip_gap;                              // relative MIP gap tolerance
    int mir_cuts;   // MIR cuts       (GLP_ON/GLP_OFF)
    int gmi_cuts;   // Gomory's cuts  (GLP_ON/GLP_OFF)
    int cov_cuts;   // cover cuts     (GLP_ON/GLP_OFF)
    int clq_cuts;   // clique cuts    (GLP_ON/GLP_OFF)
    int presolve;   // enable/disable using MIP presolver
    int binarize;   // try to binarize integer variables
    int fp_heur;    // feasibility pump heuristic
    int ps_heur;    // proximity search heuristic
    int ps_tm_lim;  // proxy time limit, milliseconds
    int sr_heur;    // simple rounding heuristic
#if 1 /* 24/X-2015; not documented--should not be used */
    int use_sol;           /* use existing solution */
    const char * save_sol; /* filename to save every new solution */
    int alien;             /* use alien solver */
#endif
#if 1 /* 16/III-2016; not documented--should not be used */
    int flip; /* use long-step dual simplex */
#endif
    double foo_bar[23];  // (reserved)
};
int glp_intopt(glp_prob * P, const glp_iocp * parm);
int glp_intfeas1(glp_prob * P, int use_bound, int obj_bound);
double glp_mip_obj_val(glp_prob * P);
double glp_mip_col_val(glp_prob * P, int j);

}  // namespace glpk::v5
}  // namespace fhamonic::mippp
#endif

#include "dylib.hpp"

namespace fhamonic::mippp {
namespace glpk::v5 {

#define GLPK_FUNCTIONS(F)               \
    F(glp_create_prob, create_prob)     \
    F(glp_delete_prob, delete_prob)     \
    F(glp_set_obj_dir, set_obj_dir)     \
    F(glp_get_obj_dir, get_obj_dir)     \
    F(glp_add_cols, add_cols)           \
    F(glp_set_obj_coef, set_obj_coef)   \
    F(glp_get_obj_coef, get_obj_coef)   \
    F(glp_set_col_bnds, set_col_bnds)   \
    F(glp_get_col_lb, get_col_lb)       \
    F(glp_get_col_ub, get_col_ub)       \
    F(glp_set_col_kind, set_col_kind)   \
    F(glp_get_col_kind, get_col_kind)   \
    F(glp_set_col_name, set_col_name)   \
    F(glp_get_col_name, get_col_name)   \
    F(glp_add_rows, add_rows)           \
    F(glp_set_mat_row, set_mat_row)     \
    F(glp_set_mat_col, set_mat_col)     \
    F(glp_set_row_bnds, set_row_bnds)   \
    F(glp_get_row_lb, get_row_lb)       \
    F(glp_get_row_ub, get_row_ub)       \
    F(glp_set_row_name, set_row_name)   \
    F(glp_get_row_name, get_row_name)   \
    F(glp_get_row_type, get_row_type)   \
    F(glp_get_num_cols, get_num_cols)   \
    F(glp_get_num_rows, get_num_rows)   \
    F(glp_get_num_nz, get_num_nz)       \
    F(glp_set_row_stat, set_row_stat)   \
    F(glp_get_row_stat, get_row_stat)   \
    F(glp_set_col_stat, set_col_stat)   \
    F(glp_get_col_stat, get_col_stat)   \
    F(glp_simplex, simplex)             \
    F(glp_get_status, get_status)       \
    F(glp_get_prim_stat, get_prim_stat) \
    F(glp_get_dual_stat, get_dual_stat) \
    F(glp_get_obj_val, get_obj_val)     \
    F(glp_get_col_prim, get_col_prim)   \
    F(glp_get_row_dual, get_row_dual)   \
    F(glp_intopt, intopt)               \
    F(glp_intfeas1, intfeas1)           \
    F(glp_mip_obj_val, mip_obj_val)     \
    F(glp_mip_col_val, mip_col_val)

#define DECLARE_GLPK_FUN(FULL, SHORT)     \
    using SHORT##_fun_t = decltype(FULL); \
    SHORT##_fun_t * SHORT;
#define CONSTRUCT_GLPK_FUN(FULL, SHORT) \
    , SHORT(lib.get_function<SHORT##_fun_t>(#FULL))

class glpk_api {
private:
    dylib lib;

public:
    GLPK_FUNCTIONS(DECLARE_GLPK_FUN)

public:
    inline glpk_api(const char * lib_path = "", const char * lib_name = "glpk")
        : lib(lib_path, lib_name) GLPK_FUNCTIONS(CONSTRUCT_GLPK_FUN) {}
};

}  // namespace glpk::v5
}  // namespace fhamonic::mippp

#endif  // MIPPP_GLPK_v5_API_HPP