#pragma once

#if INCLUDE_CPLEX_HEADER
#include "ilcplex/cplex.h"
#else
namespace mippp {
namespace cplex::v22_1_2 {

struct cpxenv;
using CPXENVptr = struct cpxenv *;
using CPXCENVptr = const struct cpxenv *;
struct cpxlp;
using CPXLPptr = struct cpxlp *;
using CPXCLPptr = const struct cpxlp *;
using CPXCCHARptr = const char *;

constexpr double CPX_INFBOUND = 1e20;

CPXENVptr CPXopenCPLEX(int * status_p);
int CPXcloseCPLEX(CPXENVptr * env_p);
CPXCCHARptr CPXversion(CPXCENVptr env);
int CPXversionnumber(CPXCENVptr env, int * version_p);
CPXLPptr CPXcreateprob(CPXCENVptr env, int * status_p,
                       char const * probname_str);
constexpr int CPXPROB_LP = 0;
constexpr int CPXPROB_MILP = 1;
int CPXgetprobtype(CPXCENVptr env, CPXCLPptr lp);
int CPXchgprobtype(CPXCENVptr env, CPXLPptr lp, int type);
int CPXfreeprob(CPXCENVptr env, CPXLPptr * lp_p);
constexpr int CPXMESSAGEBUFSIZE = 1024;
const char * CPXgeterrorstring(CPXCENVptr env, int errcode, char * buffer_str);

constexpr int CPX_MAX = -1;
constexpr int CPX_MIN = 1;
int CPXchgobjsen(CPXCENVptr env, CPXLPptr lp, int maxormin);
int CPXgetobjsen(CPXCENVptr env, CPXCLPptr lp);
int CPXchgobjoffset(CPXCENVptr env, CPXLPptr lp, double offset);
int CPXgetobjoffset(CPXCENVptr env, CPXCLPptr lp, double * objoffset_p);

int CPXnewcols(CPXCENVptr env, CPXLPptr lp, int ccnt, double const * obj,
               double const * lb, double const * ub, char const * xctype,
               char ** colname);
int CPXaddcols(CPXCENVptr env, CPXLPptr lp, int ccnt, int nzcnt,
               double const * obj, int const * cmatbeg, int const * cmatind,
               double const * cmatval, double const * lb, double const * ub,
               char ** colname);
int CPXdelsetcols(CPXCENVptr env, CPXLPptr lp, int * delstat);
int CPXaddrows(CPXCENVptr env, CPXLPptr lp, int ccnt, int rcnt, int nzcnt,
               double const * rhs, char const * sense, int const * rmatbeg,
               int const * rmatind, double const * rmatval, char ** colname,
               char ** rowname);
int CPXaddindconstr(CPXCENVptr env, CPXLPptr lp, int indvar, int complemented,
                    int nzcnt, double rhs, int sense, int const * linind,
                    double const * linval, char const * indname_str);
int CPXaddsos(CPXCENVptr env, CPXLPptr lp, int numsos, int numsosnz,
              char const * sostype, int const * sosbeg, int const * sosind,
              double const * soswt, char ** sosname);

int CPXchgobj(CPXCENVptr env, CPXLPptr lp, int cnt, int const * indices,
              double const * values);
int CPXgetobj(CPXCENVptr env, CPXCLPptr lp, double * obj, int begin, int end);
int CPXchgbds(CPXCENVptr env, CPXLPptr lp, int cnt, int const * indices,
              char const * lu, double const * bd);
int CPXgetlb(CPXCENVptr env, CPXCLPptr lp, double * lb, int begin, int end);
int CPXgetub(CPXCENVptr env, CPXCLPptr lp, double * ub, int begin, int end);
int CPXchgcolname(CPXCENVptr env, CPXLPptr lp, int cnt, int const * indices,
                  char ** newname);
int CPXgetcolname(CPXCENVptr env, CPXCLPptr lp, char ** name, char * namestore,
                  int storespace, int * surplus_p, int begin, int end);
constexpr char CPX_CONTINUOUS = 'C';
constexpr char CPX_INTEGER = 'I';
constexpr char CPX_BINARY = 'B';
int CPXchgctype(CPXCENVptr env, CPXLPptr lp, int cnt, int const * indices,
                char const * xctype);
int CPXgetctype(CPXCENVptr env, CPXCLPptr lp, char * xctype, int begin,
                int end);

int CPXgetrows(CPXCENVptr env, CPXCLPptr lp, int * nzcnt_p, int * rmatbeg,
               int * rmatind, double * rmatval, int rmatspace, int * surplus_p,
               int begin, int end);
int CPXchgsense(CPXCENVptr env, CPXLPptr lp, int cnt, int const * indices,
                char const * sense);
int CPXgetsense(CPXCENVptr env, CPXCLPptr lp, char * sense, int begin, int end);
int CPXchgrhs(CPXCENVptr env, CPXLPptr lp, int cnt, int const * indices,
              double const * values);
int CPXgetrhs(CPXCENVptr env, CPXCLPptr lp, double * rhs, int begin, int end);
int CPXchgrowname(CPXCENVptr env, CPXLPptr lp, int cnt, int const * indices,
                  char ** newname);
int CPXgetrowname(CPXCENVptr env, CPXCLPptr lp, char ** name, char * namestore,
                  int storespace, int * surplus_p, int begin, int end);

int CPXgetnumcols(CPXCENVptr env, CPXCLPptr lp);
int CPXgetnumrows(CPXCENVptr env, CPXCLPptr lp);
int CPXgetnumnz(CPXCENVptr env, CPXCLPptr lp);

enum MipStartEffort : int {
    CPX_MIPSTART_AUTO = 0,
    CPX_MIPSTART_CHECKFEAS = 1,
    CPX_MIPSTART_SOLVEFIXED = 2,
    CPX_MIPSTART_SOLVEMIP = 3,
    CPX_MIPSTART_REPAIR = 4,
    CPX_MIPSTART_NOCHECK = 5
};

int CPXaddmipstarts(CPXCENVptr env, CPXLPptr lp, int mcnt, int nzcnt,
                    int const * beg, int const * varindices,
                    double const * values, int const * effortlevel,
                    char ** mipstartname);

constexpr int CPXPARAM_Simplex_Tolerances_Feasibility = 1016;
// constexpr int CPXPARAM_Simplex_Tolerances_Markowitz = 1013;
constexpr int CPXPARAM_Simplex_Tolerances_Optimality = 1014;
constexpr int CPXPARAM_TimeLimit = 1039;
constexpr int CPXPARAM_Simplex_Limits_Iterations = 1020;
constexpr int CPXPARAM_MIP_Limits_Nodes = 2017;
constexpr int CPXPARAM_MIP_Limits_Solutions = 2015;
constexpr int CPXPARAM_MIP_Limits_TreeMemory = 2027;
constexpr int CPXPARAM_MIP_Tolerances_MIPGap = 2009;
constexpr int CPXPARAM_MIP_Tolerances_Linearization = 2068;
// constexpr int CPXPARAM_MIP_Tolerances_Integrality = 2010;
int CPXgetdblparam(CPXCENVptr env, int whichparam, double * value_p);
int CPXsetdblparam(CPXENVptr env, int whichparam, double newvalue);

constexpr int CPXPARAM_Advance = 1001;
constexpr int CPXPARAM_Preprocessing_Reduce = 1057;
enum PrereduceType : int {
    CPX_PREREDUCE_NOPRIMALORDUAL = 0,
    CPX_PREREDUCE_PRIMALONLY = 1,
    CPX_PREREDUCE_DUALONLY = 2,
    CPX_PREREDUCE_PRIMALANDDUAL = 3
};
int CPXgetintparam(CPXCENVptr env, int whichparam, int * value_p);
int CPXsetintparam(CPXENVptr env, int whichparam, int newvalue);

int CPXprimopt(CPXCENVptr env, CPXLPptr lp);
int CPXdualopt(CPXCENVptr env, CPXLPptr lp);
int CPXlpopt(CPXCENVptr env, CPXLPptr lp);
int CPXfeasopt(CPXCENVptr env, CPXLPptr lp, double const * rhs,
               double const * rng, double const * lb, double const * ub);
int CPXmipopt(CPXCENVptr env, CPXLPptr lp);
int CPXbendersopt(CPXCENVptr env, CPXLPptr lp);

constexpr int CPX_STAT_UNKNOWN = 0;
constexpr int CPX_STAT_OPTIMAL = 1;
constexpr int CPX_STAT_UNBOUNDED = 2;
constexpr int CPX_STAT_INFEASIBLE = 3;
constexpr int CPX_STAT_INForUNBD = 4;
constexpr int CPX_STAT_OPTIMAL_INFEAS = 5;
constexpr int CPX_STAT_NUM_BEST = 6;
constexpr int CPX_STAT_ABORT_IT_LIM = 10;
constexpr int CPX_STAT_ABORT_TIME_LIM = 11;
constexpr int CPX_STAT_ABORT_OBJ_LIM = 12;
constexpr int CPX_STAT_ABORT_USER = 13;
constexpr int CPX_STAT_FEASIBLE_RELAXED_SUM = 14;   // after CPXfeasopt
constexpr int CPX_STAT_OPTIMAL_RELAXED_SUM = 15;    // after CPXfeasopt
constexpr int CPX_STAT_FEASIBLE_RELAXED_INF = 16;   // after CPXfeasopt
constexpr int CPX_STAT_OPTIMAL_RELAXED_INF = 17;    // after CPXfeasopt
constexpr int CPX_STAT_FEASIBLE_RELAXED_QUAD = 18;  // after CPXfeasopt
constexpr int CPX_STAT_OPTIMAL_RELAXED_QUAD = 19;   // after CPXfeasopt
constexpr int CPX_STAT_OPTIMAL_FACE_UNBOUNDED = 20;
constexpr int CPX_STAT_ABORT_PRIM_OBJ_LIM = 21;
constexpr int CPX_STAT_ABORT_DUAL_OBJ_LIM = 22;
constexpr int CPX_STAT_FEASIBLE = 23;  // after CPXfeasopt
constexpr int CPX_STAT_ABORT_DETTIME_LIM = 25;

// constexpr int CPX_STAT_CONFLICT_FEASIBLE = 30;
// constexpr int CPX_STAT_CONFLICT_MINIMAL = 31;
// constexpr int CPX_STAT_CONFLICT_ABORT_CONTRADICTION = 32;
// constexpr int CPX_STAT_CONFLICT_ABORT_TIME_LIM = 33;
// constexpr int CPX_STAT_CONFLICT_ABORT_IT_LIM = 34;
// constexpr int CPX_STAT_CONFLICT_ABORT_NODE_LIM = 35;
// constexpr int CPX_STAT_CONFLICT_ABORT_OBJ_LIM = 36;
// constexpr int CPX_STAT_CONFLICT_ABORT_MEM_LIM = 37;
// constexpr int CPX_STAT_CONFLICT_ABORT_USER = 38;
// constexpr int CPX_STAT_CONFLICT_ABORT_DETTIME_LIM = 39;

constexpr int CPXMIP_OPTIMAL = 101;
constexpr int CPXMIP_OPTIMAL_TOL = 102;
constexpr int CPXMIP_OPTIMAL_INFEAS = 115;
constexpr int CPXMIP_OPTIMAL_POPULATED = 129;      // after CPXpopulate
constexpr int CPXMIP_OPTIMAL_POPULATED_TOL = 130;  // after CPXpopulate

constexpr int CPXMIP_INForUNBD = 119;
constexpr int CPXMIP_UNBOUNDED = 118;
constexpr int CPXMIP_INFEASIBLE = 103;
// for the statuses below: *_FEAS/*_INFEAS = a solution is/isn't available
constexpr int CPXMIP_ABORT_FEAS = 113;
constexpr int CPXMIP_ABORT_INFEAS = 114;
constexpr int CPXMIP_ABORT_RELAXATION_UNBOUNDED = 133;  // only (MI)QP
constexpr int CPXMIP_ABORT_RELAXED = 126;               // after CPXfeasopt
// Errors
constexpr int CPXMIP_FAIL_FEAS = 109;
constexpr int CPXMIP_FAIL_FEAS_NO_TREE = 116;  // out of memory
constexpr int CPXMIP_FAIL_INFEAS = 110;
constexpr int CPXMIP_FAIL_INFEAS_NO_TREE = 117;  // out of memory
// Feasibility
constexpr int CPXMIP_FEASIBLE = 127;               // after CPXfeasopt
constexpr int CPXMIP_FEASIBLE_RELAXED_INF = 122;   // after CPXfeasopt
constexpr int CPXMIP_FEASIBLE_RELAXED_QUAD = 124;  // after CPXfeasopt
constexpr int CPXMIP_FEASIBLE_RELAXED_SUM = 120;   // after CPXfeasopt
constexpr int CPXMIP_OPTIMAL_RELAXED_INF = 123;    // after CPXfeasopt
constexpr int CPXMIP_OPTIMAL_RELAXED_QUAD = 125;   // after CPXfeasopt
constexpr int CPXMIP_OPTIMAL_RELAXED_SUM = 121;    // after CPXfeasopt
// Limits
constexpr int CPXMIP_MEM_LIM_FEAS = 111;
constexpr int CPXMIP_MEM_LIM_INFEAS = 112;
constexpr int CPXMIP_NODE_LIM_FEAS = 105;
constexpr int CPXMIP_NODE_LIM_INFEAS = 106;
constexpr int CPXMIP_POPULATESOL_LIM = 128;  // after CPXpopulate
constexpr int CPXMIP_SOL_LIM = 104;
constexpr int CPXMIP_TIME_LIM_FEAS = 107;
constexpr int CPXMIP_TIME_LIM_INFEAS = 108;
constexpr int CPXMIP_DETTIME_LIM_FEAS = 131;
constexpr int CPXMIP_DETTIME_LIM_INFEAS = 132;

int CPXgetstat(CPXCENVptr env, CPXLPptr lp);
int CPXsolninfo(CPXCENVptr env, CPXCLPptr lp, int * solnmethod_p,
                int * solntype_p, int * pfeasind_p, int * dfeasind_p);

int CPXgetobjval(CPXCENVptr env, CPXCLPptr lp, double * objval_p);
int CPXgetbestobjval(CPXCENVptr env, CPXCLPptr lp, double * objval_p);
int CPXgetx(CPXCENVptr env, CPXCLPptr lp, double * x, int begin, int end);
int CPXgetpi(CPXCENVptr env, CPXCLPptr lp, double * pi, int begin, int end);
int CPXsolution(CPXCENVptr env, CPXCLPptr lp, int * lpstat_p, double * objval_p,
                double * x, double * pi, double * slack, double * dj);

using CPXLONG = long long;
constexpr CPXLONG CPX_CALLBACKCONTEXT_BRANCHING = 0x0080;
constexpr CPXLONG CPX_CALLBACKCONTEXT_CANDIDATE = 0x0020;
constexpr CPXLONG CPX_CALLBACKCONTEXT_GLOBAL_PROGRESS = 0x0010;
constexpr CPXLONG CPX_CALLBACKCONTEXT_LOCAL_PROGRESS = 0x0008;
constexpr CPXLONG CPX_CALLBACKCONTEXT_RELAXATION = 0x0040;
constexpr CPXLONG CPX_CALLBACKCONTEXT_THREAD_DOWN = 0x0004;
constexpr CPXLONG CPX_CALLBACKCONTEXT_THREAD_UP = 0x0002;
using CPXCALLBACKCONTEXTptr = struct cpxcallbackcontext *;
using CPXCALLBACKFUNC = int(CPXCALLBACKCONTEXTptr context, CPXLONG contextid,
                            void * userhandle);
int CPXcallbacksetfunc(CPXENVptr env, CPXLPptr lp, CPXLONG contextmask,
                       CPXCALLBACKFUNC callback, void * userhandle);
int CPXcallbackgetcandidatepoint(CPXCALLBACKCONTEXTptr context, double * x,
                                 int begin, int end, double * obj_p);
int CPXcallbackrejectcandidate(CPXCALLBACKCONTEXTptr context, int rcnt,
                               int nzcnt, double const * rhs,
                               char const * sense, int const * rmatbeg,
                               int const * rmatind, double const * rmatval);
int CPXcallbackrejectcandidatelocal(CPXCALLBACKCONTEXTptr context, int rcnt,
                                    int nzcnt, double const * rhs,
                                    char const * sense, int const * rmatbeg,
                                    int const * rmatind,
                                    double const * rmatval);
int CPXcallbackaddusercuts(CPXCALLBACKCONTEXTptr context, int rcnt, int nzcnt,
                           double const * rhs, char const * sense,
                           int const * rmatbeg, int const * rmatind,
                           double const * rmatval, int const * purgeable,
                           int const * local);

void CPXcallbackabort(CPXCALLBACKCONTEXTptr context);

}  // namespace cplex::v22_1_2
}  // namespace mippp
#endif

#include "dylib.hpp"

#include "mippp/utility/solver_exceptions.hpp"
#include "mippp/detail/solver_library.hpp"

namespace mippp {
namespace cplex::v22_1_2 {

#define CPLEX_FUNCTIONS(F)                                           \
    F(CPXopenCPLEX, openCPLEX)                                       \
    F(CPXcloseCPLEX, closeCPLEX)                                     \
    F(CPXversion, version)                                           \
    F(CPXversionnumber, versionnumber)                               \
    F(CPXcreateprob, createprob)                                     \
    F(CPXgetprobtype, getprobtype)                                   \
    F(CPXchgprobtype, chgprobtype)                                   \
    F(CPXfreeprob, freeprob)                                         \
    F(CPXgeterrorstring, geterrorstring)                             \
    F(CPXchgobjsen, chgobjsen)                                       \
    F(CPXgetobjsen, getobjsen)                                       \
    F(CPXchgobjoffset, chgobjoffset)                                 \
    F(CPXgetobjoffset, getobjoffset)                                 \
    F(CPXnewcols, newcols)                                           \
    F(CPXaddcols, addcols)                                           \
    F(CPXdelsetcols, delsetcols)                                     \
    F(CPXaddrows, addrows)                                           \
    F(CPXaddindconstr, addindconstr)                                 \
    F(CPXaddsos, addsos)                                             \
    F(CPXchgobj, chgobj)                                             \
    F(CPXgetobj, getobj)                                             \
    F(CPXchgbds, chgbds)                                             \
    F(CPXgetlb, getlb)                                               \
    F(CPXgetub, getub)                                               \
    F(CPXchgcolname, chgcolname)                                     \
    F(CPXgetcolname, getcolname)                                     \
    F(CPXchgctype, chgctype)                                         \
    F(CPXgetctype, getctype)                                         \
    F(CPXgetrows, getrows)                                           \
    F(CPXchgsense, chgsense)                                         \
    F(CPXgetsense, getsense)                                         \
    F(CPXchgrhs, chgrhs)                                             \
    F(CPXgetrhs, getrhs)                                             \
    F(CPXchgrowname, chgrowname)                                     \
    F(CPXgetrowname, getrowname)                                     \
    F(CPXgetnumcols, getnumcols)                                     \
    F(CPXgetnumrows, getnumrows)                                     \
    F(CPXgetnumnz, getnumnz)                                         \
    F(CPXaddmipstarts, addmipstarts)                                 \
    F(CPXgetdblparam, getdblparam)                                   \
    F(CPXsetdblparam, setdblparam)                                   \
    F(CPXgetintparam, getintparam)                                   \
    F(CPXsetintparam, setintparam)                                   \
    F(CPXprimopt, primopt)                                           \
    F(CPXdualopt, dualopt)                                           \
    F(CPXlpopt, lpopt)                                               \
    F(CPXfeasopt, feasopt)                                           \
    F(CPXmipopt, mipopt)                                             \
    F(CPXbendersopt, bendersopt)                                     \
    F(CPXgetstat, getstat)                                           \
    F(CPXsolninfo, solninfo)                                         \
    F(CPXgetobjval, getobjval)                                       \
    F(CPXgetbestobjval, getbestobjval)                               \
    F(CPXgetx, getx)                                                 \
    F(CPXgetpi, getpi)                                               \
    F(CPXsolution, solution)                                         \
    F(CPXcallbacksetfunc, callbacksetfunc)                           \
    F(CPXcallbackgetcandidatepoint, callbackgetcandidatepoint)       \
    F(CPXcallbackrejectcandidate, callbackrejectcandidate)           \
    F(CPXcallbackrejectcandidatelocal, callbackrejectcandidatelocal) \
    F(CPXcallbackaddusercuts, callbackaddusercuts)                   \
    F(CPXcallbackabort, callbackabort)

#define DECLARE_CPLEX_FUN(FULL, SHORT)    \
    using SHORT##_fun_t = decltype(FULL); \
    SHORT##_fun_t * SHORT;
#define CONSTRUCT_CPLEX_FUN(FULL, SHORT) \
    , SHORT(lib.get_function<SHORT##_fun_t>(#FULL))

class cplex_api {
private:
    dylib::library lib;

public:
    CPLEX_FUNCTIONS(DECLARE_CPLEX_FUN)

public:
    inline cplex_api(const char * lib_path = nullptr)
        : lib(detail::load_solver_library(lib_path, "CPLEX", {"cplex2212"}))
              CPLEX_FUNCTIONS(CONSTRUCT_CPLEX_FUN) {
        CPXENVptr env = _create_env();
        int version_;
        _check(env, versionnumber(env, &version_));
        detail::warn_on_version_mismatch("CPLEX", 2212, version_);
        _close_env(env);
    }

    CPXENVptr _create_env() const {
        int status;
        CPXENVptr env = openCPLEX(&status);
        if(status != 0) throw solver_error("CPXopenCPLEX");
        return env;
    }
    CPXLPptr _create_prob(CPXENVptr env) const {
        int status;
        CPXLPptr prob = createprob(env, &status, "cplex_base");
        if(status != 0) throw solver_error("CPXcreateprob");
        return prob;
    }
    void _close_env(CPXENVptr & env) const {
        if(closeCPLEX(&env) != 0) throw solver_error("CPXcloseCPLEX");
    }

    void _check(CPXENVptr env, const int error) const {
        if(error == 0) return;
        char errmsg[CPXMESSAGEBUFSIZE];
        geterrorstring(env, error, errmsg);
        if(error == 1016) throw license_error(errmsg);
        throw solver_error(errmsg);
    }
};

}  // namespace cplex::v22_1_2
}  // namespace mippp
