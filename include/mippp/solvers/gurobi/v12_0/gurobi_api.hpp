#ifndef MIPPP_GUROBI_v12_0_API_HPP
#define MIPPP_GUROBI_v12_0_API_HPP

#if INCLUDE_GUROBI_HEADER
#include "gurobi_c.h"
#else
namespace fhamonic::mippp {
namespace gurobi::v12_0 {

using GRBenv = struct _GRBenv;
using GRBmodel = struct _GRBmodel;

constexpr double GRB_INFINITY = 1e100;

constexpr int GRB_VERSION_MAJOR = 12;
constexpr int GRB_VERSION_MINOR = 0;
constexpr int GRB_VERSION_TECHNICAL = 1;
void GRBversion(int * majorP, int * minorP, int * technicalP);
int GRBemptyenvinternal(GRBenv ** envP, int major, int minor, int tech);
int GRBstartenv(GRBenv * env);
GRBenv * GRBgetenv(GRBmodel * model);
void GRBfreeenv(GRBenv * env);
int GRBnewmodel(GRBenv * env, GRBmodel ** modelP, const char * Pname,
                int numvars, double * obj, double * lb, double * ub,
                char * vtype, char ** varnames);
int GRBfreemodel(GRBmodel * model);
const char * GRBgeterrormsg(GRBenv * env);
int GRBupdatemodel(GRBmodel * model);

int GRBaddvar(GRBmodel * model, int numnz, int * vind, double * vval,
              double obj, double lb, double ub, char vtype,
              const char * varname);
int GRBaddvars(GRBmodel * model, int numvars, int numnz, int * vbeg, int * vind,
               double * vval, double * obj, double * lb, double * ub,
               char * vtype, char ** varnames);
int GRBaddconstr(GRBmodel * model, int numnz, int * cind, double * cval,
                 char sense, double rhs, const char * constrname);
int GRBaddconstrs(GRBmodel * model, int numconstrs, int numnz, int * cbeg,
                  int * cind, double * cval, char * sense, double * rhs,
                  char ** constrnames);
int GRBaddrangeconstr(GRBmodel * model, int numnz, int * cind, double * cval,
                      double lower, double upper, const char * constrname);
// SOS types
constexpr int GRB_SOS_TYPE1 = 1;
constexpr int GRB_SOS_TYPE2 = 2;
int GRBaddsos(GRBmodel * model, int numsos, int nummembers, int * types,
              int * beg, int * ind, double * weight);
int GRBaddgenconstrIndicator(GRBmodel * model, const char * name, int binvar,
                             int binval, int nvars, const int * vars,
                             const double * vals, char sense, double rhs);

int GRBgetcoeff(GRBmodel * model, int constr, int var, double * valP);
int GRBgetconstrs(GRBmodel * model, int * numnzP, int * cbeg, int * cind,
                  double * cval, int start, int len);

int GRBaddqpterms(GRBmodel * model, int numqnz, int * qrow, int * qcol,
                  double * qval);
int GRBdelq(GRBmodel * model);
int GRBaddqconstr(GRBmodel * model, int numlnz, int * lind, double * lval,
                  int numqnz, int * qrow, int * qcol, double * qval, char sense,
                  double rhs, const char * QCname);

int GRBoptimize(GRBmodel * model);

constexpr const char * GRB_INT_PAR_DUALREDUCTIONS = "DualReductions";
constexpr const char * GRB_INT_PAR_LAZYCONSTRAINTS = "LazyConstraints";
int GRBsetintparam(GRBenv * env, const char * paramname, int value);
int GRBgetintparam(GRBenv * env, const char * paramname, int * valueP);
constexpr const char * GRB_DBL_PAR_NODELIMIT = "NodeLimit";
constexpr const char * GRB_DBL_PAR_TIMELIMIT = "TimeLimit";
constexpr const char * GRB_DBL_PAR_FEASIBILITYTOL = "FeasibilityTol";
constexpr const char * GRB_DBL_PAR_OPTIMALITYTOL = "OptimalityTol";
constexpr const char * GRB_DBL_PAR_MIPGAP = "MIPGap";
int GRBsetdblparam(GRBenv * env, const char * paramname, double value);
int GRBgetdblparam(GRBenv * env, const char * paramname, double * valueP);
constexpr const char * GRB_INT_ATTR_MODELSENSE = "ModelSense";
enum ModelSense : int { GRB_MINIMIZE = 1, GRB_MAXIMIZE = -1 };
constexpr const char * GRB_INT_ATTR_NUMVARS = "NumVars";
constexpr const char * GRB_INT_ATTR_NUMCONSTRS = "NumConstrs";
constexpr const char * GRB_INT_ATTR_NUMNZS = "NumNZs";
// Model statuses
constexpr const char * GRB_INT_ATTR_STATUS = "Status";
enum ModelStatus : int {
    GRB_LOADED = 1,
    GRB_OPTIMAL = 2,
    GRB_INFEASIBLE = 3,
    GRB_INF_OR_UNBD = 4,
    GRB_UNBOUNDED = 5,
    GRB_CUTOFF = 6,
    GRB_ITERATION_LIMIT = 7,
    GRB_NODE_LIMIT = 8,
    GRB_TIME_LIMIT = 9,
    GRB_SOLUTION_LIMIT = 10,
    GRB_INTERRUPTED = 11,
    GRB_NUMERIC = 12,
    GRB_SUBOPTIMAL = 13,
    GRB_INPROGRESS = 14,
    GRB_USER_OBJ_LIMIT = 15,
    GRB_WORK_LIMIT = 16,
    GRB_MEM_LIMIT = 17
};
int GRBsetintattr(GRBmodel * model, const char * attrname, int newvalue);
int GRBgetintattr(GRBmodel * model, const char * attrname, int * valueP);
int GRBsetintattrelement(GRBmodel * model, const char * attrname, int element,
                         int newvalue);
int GRBgetintattrelement(GRBmodel * model, const char * attrname, int element,
                         int * valueP);
int GRBsetintattrarray(GRBmodel * model, const char * attrname, int first,
                       int len, int * newvalues);
int GRBgetintattrarray(GRBmodel * model, const char * attrname, int first,
                       int len, int * values);
constexpr const char * GRB_DBL_ATTR_OBJCON = "ObjCon";  // objective offset
constexpr const char * GRB_DBL_ATTR_OBJ = "Obj";        // variable obj coef
constexpr const char * GRB_DBL_ATTR_LB = "LB";          // variable lower bound
constexpr const char * GRB_DBL_ATTR_UB = "UB";          // variable upper bound
constexpr const char * GRB_DBL_ATTR_RHS = "RHS";        // constraint rhs
constexpr const char * GRB_DBL_ATTR_OBJVAL = "ObjVal";  // solution value
constexpr const char * GRB_DBL_ATTR_X = "X";            // solution
constexpr const char * GRB_DBL_ATTR_PI = "Pi";          // dual solution
constexpr const char * GRB_DBL_ATTR_START = "Start";    // MIP start
int GRBsetdblattr(GRBmodel * model, const char * attrname, double newvalue);
int GRBgetdblattr(GRBmodel * model, const char * attrname, double * valueP);
int GRBsetdblattrelement(GRBmodel * model, const char * attrname, int element,
                         double newvalue);
int GRBgetdblattrelement(GRBmodel * model, const char * attrname, int element,
                         double * valueP);
int GRBsetdblattrarray(GRBmodel * model, const char * attrname, int first,
                       int len, double * newvalues);

int GRBgetdblattrarray(GRBmodel * model, const char * attrname, int first,
                       int len, double * values);
int GRBsetdblattrlist(GRBmodel * model, const char * attrname, int len,
                      int * ind, double * newvalues);
constexpr const char * GRB_CHAR_ATTR_SENSE = "Sense";
enum ConstraintSense : char {
    GRB_LESS_EQUAL = '<',
    GRB_GREATER_EQUAL = '>',
    GRB_EQUAL = '='
};
constexpr const char * GRB_CHAR_ATTR_VTYPE = "VType";
enum VariableType : char {
    GRB_CONTINUOUS = 'C',
    GRB_BINARY = 'B',
    GRB_INTEGER = 'I'
};
int GRBsetcharattrelement(GRBmodel * model, const char * attrname, int element,
                          char newvalue);
int GRBgetcharattrelement(GRBmodel * model, const char * attrname, int element,
                          char * valueP);
int GRBsetcharattrarray(GRBmodel * model, const char * attrname, int first,
                        int len, char * newvalues);
int GRBgetcharattrarray(GRBmodel * model, const char * attrname, int first,
                        int len, char * values);
constexpr const char * GRB_STR_ATTR_VARNAME = "VarName";
constexpr const char * GRB_STR_ATTR_CONSTRNAME = "ConstrName";
int GRBsetstrattrelement(GRBmodel * model, const char * attrname, int element,
                         const char * newvalue);
int GRBgetstrattrelement(GRBmodel * model, const char * attrname, int element,
                         char ** valueP);

enum CallbackWhere : int {
    GRB_CB_POLLING = 0,
    GRB_CB_PRESOLVE = 1,
    GRB_CB_SIMPLEX = 2,
    GRB_CB_MIP = 3,
    GRB_CB_MIPSOL = 4,
    GRB_CB_MIPNODE = 5,
    GRB_CB_MESSAGE = 6,
    GRB_CB_BARRIER = 7,
    GRB_CB_MULTIOBJ = 8,
    GRB_CB_IIS = 9
};
using callback_func_t = int(GRBmodel *, void *, int, void *);
int GRBsetcallbackfunc(GRBmodel * model, callback_func_t * cb, void * usrdata);
int GRBcbproceed(void * cbdata);
constexpr int GRB_CB_MIPSOL_SOL = 4001;
int GRBcbget(void * cbdata, int where, int what, void * resultP);
int GRBcbsetintparam(void * cbdata, const char * paramname, int newvalue);
int GRBcbsetdblparam(void * cbdata, const char * paramname, double newvalue);
int GRBcbsetstrparam(void * cbdata, const char * paramname,
                     const char * newvalue);
int GRBcbsetparam(void * cbdata, const char * paramname, const char * newvalue);
int GRBcbsolution(void * cbdata, const double * solution, double * objvalP);
int GRBcbcut(void * cbdata, int cutlen, const int * cutind,
             const double * cutval, char cutsense, double cutrhs);
int GRBcblazy(void * cbdata, int lazylen, const int * lazyind,
              const double * lazyval, char lazysense, double lazyrhs);
}  // namespace gurobi::v12_0
}  // namespace fhamonic::mippp
#endif

#include "dylib.hpp"

namespace fhamonic::mippp {
namespace gurobi::v12_0 {

#define GRB_FUNCTIONS(F)                               \
    F(GRBversion, version)                             \
    F(GRBemptyenvinternal, emptyenvinternal)           \
    F(GRBstartenv, startenv)                           \
    F(GRBgetenv, getenv)                               \
    F(GRBfreeenv, freeenv)                             \
    F(GRBnewmodel, newmodel)                           \
    F(GRBfreemodel, freemodel)                         \
    F(GRBgeterrormsg, geterrormsg)                     \
    F(GRBupdatemodel, updatemodel)                     \
    F(GRBaddvar, addvar)                               \
    F(GRBaddvars, addvars)                             \
    F(GRBaddqpterms, addqpterms)                       \
    F(GRBdelq, delq)                                   \
    F(GRBaddconstr, addconstr)                         \
    F(GRBaddconstrs, addconstrs)                       \
    F(GRBaddrangeconstr, addrangeconstr)               \
    F(GRBaddqconstr, addqconstr)                       \
    F(GRBaddsos, addsos)                               \
    F(GRBaddgenconstrIndicator, addgenconstrIndicator) \
    F(GRBgetcoeff, getcoeff)                           \
    F(GRBgetconstrs, getconstrs)                       \
    F(GRBoptimize, optimize)                           \
    F(GRBsetintparam, setintparam)                     \
    F(GRBgetintparam, getintparam)                     \
    F(GRBsetdblparam, setdblparam)                     \
    F(GRBgetdblparam, getdblparam)                     \
    F(GRBsetintattr, setintattr)                       \
    F(GRBgetintattr, getintattr)                       \
    F(GRBsetintattrelement, setintattrelement)         \
    F(GRBgetintattrelement, getintattrelement)         \
    F(GRBsetintattrarray, setintattrarray)             \
    F(GRBgetintattrarray, getintattrarray)             \
    F(GRBsetdblattr, setdblattr)                       \
    F(GRBgetdblattr, getdblattr)                       \
    F(GRBsetdblattrelement, setdblattrelement)         \
    F(GRBgetdblattrelement, getdblattrelement)         \
    F(GRBsetdblattrarray, setdblattrarray)             \
    F(GRBgetdblattrarray, getdblattrarray)             \
    F(GRBsetdblattrlist, setdblattrlist)               \
    F(GRBsetcharattrelement, setcharattrelement)       \
    F(GRBgetcharattrelement, getcharattrelement)       \
    F(GRBsetcharattrarray, setcharattrarray)           \
    F(GRBgetcharattrarray, getcharattrarray)           \
    F(GRBsetstrattrelement, setstrattrelement)         \
    F(GRBgetstrattrelement, getstrattrelement)         \
    F(GRBsetcallbackfunc, setcallbackfunc)             \
    F(GRBcbproceed, cbproceed)                         \
    F(GRBcbget, cbget)                                 \
    F(GRBcbsetintparam, cbsetintparam)                 \
    F(GRBcbsetdblparam, cbsetdblparam)                 \
    F(GRBcbsetstrparam, cbsetstrparam)                 \
    F(GRBcbsetparam, cbsetparam)                       \
    F(GRBcbsolution, cbsolution)                       \
    F(GRBcbcut, cbcut)                                 \
    F(GRBcblazy, cblazy)

#define DECLARE_GUROBI_FUN(FULL, SHORT)   \
    using SHORT##_fun_t = decltype(FULL); \
    SHORT##_fun_t * SHORT;
#define CONSTRUCT_GUROBI_FUN(FULL, SHORT) \
    , SHORT(lib.get_function<SHORT##_fun_t>(#FULL))

class gurobi_api {
private:
    dylib lib;

public:
    GRB_FUNCTIONS(DECLARE_GUROBI_FUN)

public:
    gurobi_api(const char * lib_path = "", const char * lib_name = "gurobi120")
        : lib(lib_path, lib_name) GRB_FUNCTIONS(CONSTRUCT_GUROBI_FUN) {}
};

}  // namespace gurobi::v12_0
}  // namespace fhamonic::mippp

#endif  // MIPPP_GUROBI_v12_0_API_HPP