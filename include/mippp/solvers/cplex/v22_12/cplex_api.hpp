#ifndef MIPPP_CPLEX_v22_12_API_HPP
#define MIPPP_CPLEX_v22_12_API_HPP

#if INCLUDE_CPLEX_HEADER
#include "ilcplex/cplex.h"
#else
namespace fhamonic::mippp {
namespace cplex::v22_12 {

struct cpxenv;
using CPXENVptr = struct cpxenv *;
using CPXCENVptr = const struct cpxenv *;
struct cpxlp;
using CPXLPptr = struct cpxlp *;
using CPXCLPptr = const struct cpxlp *;

constexpr double CPX_INFBOUND = 1e20;

CPXENVptr CPXopenCPLEX(int * status_p);
CPXLPptr CPXcreateprob(CPXCENVptr env, int * status_p,
                       char const * probname_str);
constexpr int CPXPROB_LP = 0;
constexpr int CPXPROB_MILP = 1;
int CPXgetprobtype(CPXCENVptr env, CPXCLPptr lp);
int CPXchgprobtype(CPXCENVptr env, CPXLPptr lp, int type);
int CPXfreeprob(CPXCENVptr env, CPXLPptr * lp_p);
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

int CPXaddmipstarts(CPXCENVptr env, CPXLPptr lp, int mcnt, int nzcnt,
                    int const * beg, int const * varindices,
                    double const * values, int const * effortlevel,
                    char ** mipstartname);

constexpr int CPXPARAM_Simplex_Tolerances_Feasibility = 1016;
// constexpr int CPXPARAM_Simplex_Tolerances_Markowitz = 1013;
constexpr int CPXPARAM_Simplex_Tolerances_Optimality = 1014;
constexpr int CPXPARAM_MIP_Tolerances_MIPGap = 2009;
constexpr int CPXPARAM_MIP_Tolerances_Linearization = 2068;
// constexpr int CPXPARAM_MIP_Tolerances_Integrality = 2010;
int CPXgetdblparam(CPXCENVptr env, int whichparam, double * value_p);
int CPXsetdblparam(CPXENVptr env, int whichparam, double newvalue);

int CPXprimopt(CPXCENVptr env, CPXLPptr lp);
int CPXdualopt(CPXCENVptr env, CPXLPptr lp);
int CPXlpopt(CPXCENVptr env, CPXLPptr lp);
int CPXfeasopt(CPXCENVptr env, CPXLPptr lp, double const * rhs,
               double const * rng, double const * lb, double const * ub);
int CPXmipopt(CPXCENVptr env, CPXLPptr lp);
int CPXbendersopt(CPXCENVptr env, CPXLPptr lp);

int CPXcheckpfeas(CPXCENVptr env, CPXLPptr lp, int * infeas_p);
int CPXcheckdfeas(CPXCENVptr env, CPXLPptr lp, int * infeas_p);

int CPXgetobjval(CPXCENVptr env, CPXCLPptr lp, double * objval_p);
int CPXgetbestobjval(CPXCENVptr env, CPXCLPptr lp, double * objval_p);
int CPXgetx(CPXCENVptr env, CPXCLPptr lp, double * x, int begin, int end);
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

}  // namespace cplex::v22_12
}  // namespace fhamonic::mippp
#endif

#include "dylib.hpp"

namespace fhamonic::mippp {
namespace cplex::v22_12 {

#define CPLEX_FUNCTIONS(F)                                           \
    F(CPXopenCPLEX, openCPLEX)                                       \
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
    F(CPXprimopt, primopt)                                           \
    F(CPXdualopt, dualopt)                                           \
    F(CPXlpopt, lpopt)                                               \
    F(CPXfeasopt, feasopt)                                           \
    F(CPXmipopt, mipopt)                                             \
    F(CPXbendersopt, bendersopt)                                     \
    F(CPXcheckpfeas, checkpfeas)                                     \
    F(CPXcheckdfeas, checkdfeas)                                     \
    F(CPXgetobjval, getobjval)                                       \
    F(CPXgetbestobjval, getbestobjval)                               \
    F(CPXgetx, getx)                                                 \
    F(CPXsolution, solution)                                         \
    F(CPXcallbacksetfunc, callbacksetfunc)                           \
    F(CPXcallbackgetcandidatepoint, callbackgetcandidatepoint)       \
    F(CPXcallbackrejectcandidate, callbackrejectcandidate)           \
    F(CPXcallbackrejectcandidatelocal, callbackrejectcandidatelocal) \
    F(CPXcallbackaddusercuts, callbackaddusercuts)

#define DECLARE_CPLEX_FUN(FULL, SHORT)    \
    using SHORT##_fun_t = decltype(FULL); \
    SHORT##_fun_t * SHORT;
#define CONSTRUCT_CPLEX_FUN(FULL, SHORT) \
    , SHORT(lib.get_function<SHORT##_fun_t>(#FULL))

class cplex_api {
private:
    dylib lib;

public:
    CPLEX_FUNCTIONS(DECLARE_CPLEX_FUN)

public:
    inline cplex_api(const char * lib_path = "",
                     const char * lib_name = "cplex2212")
        : lib(lib_path, lib_name) CPLEX_FUNCTIONS(CONSTRUCT_CPLEX_FUN) {}
};

}  // namespace cplex::v22_12
}  // namespace fhamonic::mippp

#endif  // MIPPP_CPLEX_v22_12_API_HPP