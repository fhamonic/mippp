#ifndef MIPPP_XPRESS_v45_1_API_HPP
#define MIPPP_XPRESS_v45_1_API_HPP

#if INCLUDE_XPRESS_HEADER
#include "xprs.h"
#else
namespace fhamonic::mippp {
namespace xpress::v45_1 {

constexpr double XPRS_PLUSINFINITY = 1.0e+20;
constexpr double XPRS_MINUSINFINITY = (-1.0e+20);

using XPRSprob = struct xo_prob_struct *;

int XPRSinit(const char * path);
int XPRSfree(void);
int XPRSgetlicerrmsg(char * buffer, int maxbytes);

int XPRScreateprob(XPRSprob * p_prob);
int XPRSdestroyprob(XPRSprob prob);
int XPRSgetlasterror(XPRSprob prob, char * errmsg);

enum ObjSense : int { XPRS_OBJ_MINIMIZE = 1, XPRS_OBJ_MAXIMIZE = -1 };
int XPRSchgobjsense(XPRSprob prob, int objsense);
int XPRSchgobj(XPRSprob prob, int ncols, const int colind[],
               const double objcoef[]);
int XPRSgetobj(XPRSprob prob, double objcoef[], int first, int last);

int XPRSchgmqobj(XPRSprob prob, int ncoefs, const int objqcol1[],
                 const int objqcol2[], const double objqcoef[]);

int XPRSaddcols(XPRSprob prob, int ncols, int ncoefs, const double objcoef[],
                const int start[], const int rowind[], const double rowcoef[],
                const double lb[], const double ub[]);
int XPRSchgbounds(XPRSprob prob, int nbounds, const int colind[],
                  const char bndtype[], const double bndval[]);

int XPRSgetlb(XPRSprob prob, double lb[], int first, int last);
int XPRSgetub(XPRSprob prob, double ub[], int first, int last);
int XPRSchgcoltype(XPRSprob prob, int ncols, const int colind[],
                   const char coltype[]);

int XPRSaddrows(XPRSprob prob, int nrows, int ncoefs, const char rowtype[],
                const double rhs[], const double rng[], const int start[],
                const int colind[], const double rowcoef[]);
int XPRSchgrowtype(XPRSprob prob, int nrows, const int rowind[],
                   const char rowtype[]);
int XPRSchgrhs(XPRSprob prob, int nrows, const int rowind[],
               const double rhs[]);

enum IntergerAttribute : int {
    XPRS_COLS = 1018,
    XPRS_ROWS = 1001,
    XPRS_ELEMS = 1006,
    XPRS_LPSTATUS = 1010
};
int XPRSgetintattrib(XPRSprob prob, int attrib, int * p_value);
int XPRSgetstrattrib(XPRSprob prob, int attrib, char * value);
enum DoubleAttribute : int { XPRS_LPOBJVAL = 2001, XPRS_MIPOBJVAL = 2003 };
int XPRSgetdblattrib(XPRSprob prob, int attrib, double * p_value);

enum NameType : int { XPRS_NAMES_ROW = 1, XPRS_NAMES_COLUMN = 2 };
int XPRSaddnames(XPRSprob prob, int type, const char names[], int first,
                 int last);
int XPRSgetnamelist(XPRSprob prob, int type, char names[], int maxbytes,
                    int * p_nbytes, int first, int last);

int XPRSlpoptimize(XPRSprob prob, const char * flags);
int XPRSmipoptimize(XPRSprob prob, const char * flags);

int XPRSgetsolution(XPRSprob prob, int * status, double x[], int first,
                    int last);
int XPRSgetduals(XPRSprob prob, int * status, double duals[], int first,
                 int last);

enum LPStatus : int {
    XPRS_LP_UNSTARTED = 0,
    XPRS_LP_OPTIMAL = 1,
    XPRS_LP_INFEAS = 2,
    XPRS_LP_CUTOFF = 3,
    XPRS_LP_UNFINISHED = 4,
    XPRS_LP_UNBOUNDED = 5,
    XPRS_LP_CUTOFF_IN_DUAL = 6,
    XPRS_LP_UNSOLVED = 7,
    XPRS_LP_NONCONVEX = 8
};

int XPRSaddcbintsol(XPRSprob prob,
                    void (*intsol)(XPRSprob cbprob, void * cbdata), void * data,
                    int priority);
int XPRSremovecbintsol(XPRSprob prob,
                       void (*intsol)(XPRSprob cbprob, void * cbdata),
                       void * data);
int XPRSaddcboptnode(XPRSprob prob,
                     void (*optnode)(XPRSprob cbprob, void * cbdata,
                                     int * p_infeasible),
                     void * data, int priority);
int XPRSremovecboptnode(XPRSprob prob,
                        void (*optnode)(XPRSprob cbprob, void * cbdata,
                                        int * p_infeasible),
                        void * data);

int XPRSaddcuts(XPRSprob prob, int ncuts, const int cuttype[],
                const char rowtype[], const double rhs[], const int start[],
                const int colind[], const double cutcoef[]);
int XPRSloaddelayedrows(XPRSprob prob, int nrows, const int rowind[]);

}  // namespace xpress::v45_1
}  // namespace fhamonic::mippp
#endif

#include "dylib.hpp"

namespace fhamonic::mippp {
namespace xpress::v45_1 {

#define XPRESS_FUNCTIONS(F)                 \
    F(XPRSinit, init)                       \
    F(XPRSfree, free)                       \
    F(XPRSgetlicerrmsg, getlicerrmsg)       \
    F(XPRScreateprob, createprob)           \
    F(XPRSdestroyprob, destroyprob)         \
    F(XPRSgetlasterror, getlasterror)       \
    F(XPRSchgobjsense, chgobjsense)         \
    F(XPRSchgobj, chgobj)                   \
    F(XPRSgetobj, getobj)                   \
    F(XPRSchgmqobj, chgmqobj)               \
    F(XPRSaddcols, addcols)                 \
    F(XPRSchgbounds, chgbounds)             \
    F(XPRSgetlb, getlb)                     \
    F(XPRSgetub, getub)                     \
    F(XPRSchgcoltype, chgcoltype)           \
    F(XPRSaddrows, addrows)                 \
    F(XPRSchgrowtype, chgrowtype)           \
    F(XPRSchgrhs, chgrhs)                   \
    F(XPRSgetintattrib, getintattrib)       \
    F(XPRSgetstrattrib, getstrattrib)       \
    F(XPRSgetdblattrib, getdblattrib)       \
    F(XPRSaddnames, addnames)               \
    F(XPRSgetnamelist, getnamelist)         \
    F(XPRSlpoptimize, lpoptimize)           \
    F(XPRSmipoptimize, mipoptimize)         \
    F(XPRSgetsolution, getsolution)         \
    F(XPRSgetduals, getduals)               \
    F(XPRSaddcbintsol, addcbintsol)         \
    F(XPRSremovecbintsol, removecbintsol)   \
    F(XPRSaddcboptnode, addcboptnode)       \
    F(XPRSremovecboptnode, removecboptnode) \
    F(XPRSaddcuts, addcuts)                 \
    F(XPRSloaddelayedrows, loaddelayedrows)

#define DECLARE_XPRESS_FUN(FULL, SHORT)   \
    using SHORT##_fun_t = decltype(FULL); \
    SHORT##_fun_t * SHORT;
#define CONSTRUCT_XPRESS_FUN(FULL, SHORT) \
    , SHORT(lib.get_function<SHORT##_fun_t>(#FULL))

class xpress_api {
private:
    dylib lib;

public:
    XPRESS_FUNCTIONS(DECLARE_XPRESS_FUN)

public:
    inline xpress_api(const char * lib_name = "xprs",
                      const char * lib_path = "")
        : lib(lib_path, lib_name) XPRESS_FUNCTIONS(CONSTRUCT_XPRESS_FUN) {}
};

}  // namespace xpress::v45_1
}  // namespace fhamonic::mippp

#endif  // MIPPP_XPRESS_v45_1_API_HPP