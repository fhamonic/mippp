#ifndef MIPPP_MOSEK_v11_API_HPP
#define MIPPP_MOSEK_v11_API_HPP

#if INCLUDE_MOSEK_HEADER
#include "mosek.h"
#else
namespace fhamonic::mippp {
namespace mosek::v11 {

using MSKenv = struct mskenvt;
using MSKenv_t = MSKenv *;
using MSKtask = struct msktaskt;
using MSKtask_t = MSKtask *;

using MSKint32t = int32_t;
using MSKrealt = double;
using MSKstring_t = char *;

using MSKrescodee = int;

constexpr double MSK_INFINITY = 1.0e30;
constexpr int MSK_MAX_STR_LEN = 1024;

MSKrescodee MSK_makeenv(MSKenv_t * env, const char * dbgfile);
MSKrescodee MSK_makeemptytask(MSKenv_t env, MSKtask_t * task);
enum MSKproblemtypee : int {
    MSK_PROBTYPE_LO = 0,
    MSK_PROBTYPE_QO = 1,
    MSK_PROBTYPE_QCQO = 2,
    MSK_PROBTYPE_CONIC = 3,
    MSK_PROBTYPE_MIXED = 4
};
MSKrescodee MSK_getprobtype(MSKtask_t task, MSKproblemtypee * probtype);
MSKrescodee MSK_deletetask(MSKtask_t * task);
MSKrescodee MSK_deleteenv(MSKenv_t * env);
MSKrescodee MSK_getcodedesc(MSKrescodee code, char * symname, char * str);
enum MSKstreamtypee : int {
    MSK_STREAM_LOG = 0,
    MSK_STREAM_MSG = 1,
    MSK_STREAM_ERR = 2,
    MSK_STREAM_WRN = 3
};
using MSKuserhandle_t = void *;
using MSKstreamfunc = void(MSKuserhandle_t handle, const char * str);
MSKrescodee MSK_linkfunctotaskstream(MSKtask_t task, MSKstreamtypee whichstream,
                                     MSKuserhandle_t handle,
                                     MSKstreamfunc func);

enum MSKobjsensee : int {
    MSK_OBJECTIVE_SENSE_MINIMIZE = 0,
    MSK_OBJECTIVE_SENSE_MAXIMIZE = 1
};
MSKrescodee MSK_putobjsense(MSKtask_t task, MSKobjsensee sense);
MSKrescodee MSK_getobjsense(MSKtask_t task, MSKobjsensee * sense);
MSKrescodee MSK_putcslice(MSKtask_t task, MSKint32t first, MSKint32t last,
                          const MSKrealt * slice);
MSKrescodee MSK_putclist(MSKtask_t task, MSKint32t num, const MSKint32t * subj,
                         const MSKrealt * val);
MSKrescodee MSK_putcfix(MSKtask_t task, MSKrealt cfix);
MSKrescodee MSK_getcfix(MSKtask_t task, MSKrealt * cfix);

MSKrescodee MSK_appendvars(MSKtask_t task, MSKint32t num);
MSKrescodee MSK_appendcons(MSKtask_t task, MSKint32t num);

MSKrescodee MSK_putcj(MSKtask_t task, MSKint32t j, MSKrealt cj);
MSKrescodee MSK_getcj(MSKtask_t task, MSKint32t j, MSKrealt * cj);
enum MSKboundkeye : int {
    MSK_BK_LO = 0,  // lower bound
    MSK_BK_UP = 1,  // upper bound
    MSK_BK_FX = 2,  // fixed
    MSK_BK_FR = 3,  // free
    MSK_BK_RA = 4   // ranged
};
MSKrescodee MSK_putvarbound(MSKtask_t task, MSKint32t j, MSKboundkeye bkx,
                            MSKrealt blx, MSKrealt bux);
MSKrescodee MSK_putvarboundsliceconst(MSKtask_t task, MSKint32t first,
                                      MSKint32t last, MSKboundkeye bkx,
                                      MSKrealt blx, MSKrealt bux);
MSKrescodee MSK_chgvarbound(MSKtask_t task, MSKint32t j, MSKint32t lower,
                            MSKint32t finite, MSKrealt value);
MSKrescodee MSK_getvarbound(MSKtask_t task, MSKint32t i, MSKboundkeye * bk,
                            MSKrealt * bl, MSKrealt * bu);
MSKrescodee MSK_putvarname(MSKtask_t task, MSKint32t j, const char * name);
MSKrescodee MSK_getvarnamelen(MSKtask_t task, MSKint32t i, MSKint32t * len);
MSKrescodee MSK_getvarname(MSKtask_t task, MSKint32t j, MSKint32t sizename,
                           char * name);
enum MSKvariabletypee : int { MSK_VAR_TYPE_CONT = 0, MSK_VAR_TYPE_INT = 1 };
MSKrescodee MSK_putvartype(MSKtask_t task, MSKint32t j,
                           MSKvariabletypee vartype);
MSKrescodee MSK_putvartypelist(MSKtask_t task, MSKint32t num,
                               const MSKint32t * subj,
                               const MSKvariabletypee * vartype);

MSKrescodee MSK_putarow(MSKtask_t task, MSKint32t i, MSKint32t nzi,
                        const MSKint32t * subi, const MSKrealt * vali);
MSKrescodee MSK_getarow(MSKtask_t task, MSKint32t i, MSKint32t * nzi,
                        MSKint32t * subi, MSKrealt * vali);
MSKrescodee MSK_putarowslice(MSKtask_t task, MSKint32t first, MSKint32t last,
                             const MSKint32t * ptrb, const MSKint32t * ptre,
                             const MSKint32t * asub, const MSKrealt * aval);
MSKrescodee MSK_putconbound(MSKtask_t task, MSKint32t i, MSKboundkeye bkc,
                            MSKrealt blc, MSKrealt buc);
MSKrescodee MSK_chgconbound(MSKtask_t task, MSKint32t i, MSKint32t lower,
                            MSKint32t finite, MSKrealt value);
MSKrescodee MSK_getconbound(MSKtask_t task, MSKint32t i, MSKboundkeye * bk,
                            MSKrealt * bl, MSKrealt * bu);
MSKrescodee MSK_putconboundslice(MSKtask_t task, MSKint32t first,
                                 MSKint32t last, const MSKboundkeye * bkc,
                                 const MSKrealt * blc, const MSKrealt * buc);
MSKrescodee MSK_putconname(MSKtask_t task, MSKint32t i, const char * name);
MSKrescodee MSK_getconnamelen(MSKtask_t task, MSKint32t i, MSKint32t * len);
MSKrescodee MSK_getconname(MSKtask_t task, MSKint32t i, MSKint32t sizename,
                           char * name);

MSKrescodee MSK_getacol(MSKtask_t task, MSKint32t j, MSKint32t * nzj,
                        MSKint32t * subj, MSKrealt * valj);
MSKrescodee MSK_getaij(MSKtask_t task, MSKint32t i, MSKint32t j,
                       MSKrealt * aij);

MSKrescodee MSK_getnumvar(MSKtask_t task, MSKint32t * numvar);
MSKrescodee MSK_getnumcon(MSKtask_t task, MSKint32t * numcon);
MSKrescodee MSK_getnumanz(MSKtask_t task, MSKint32t * numanz);

enum MSKiparame : int { MSK_IPAR_OPTIMIZER = 110 };
enum MSKoptimizertypee : int {
    MSK_OPTIMIZER_CONIC = 0,
    MSK_OPTIMIZER_DUAL_SIMPLEX = 1,
    MSK_OPTIMIZER_FREE = 2,
    MSK_OPTIMIZER_FREE_SIMPLEX = 3,
    MSK_OPTIMIZER_INTPNT = 4,
    MSK_OPTIMIZER_MIXED_INT = 5,
    MSK_OPTIMIZER_NEW_DUAL_SIMPLEX = 6,
    MSK_OPTIMIZER_NEW_PRIMAL_SIMPLEX = 7,
    MSK_OPTIMIZER_PRIMAL_SIMPLEX = 8
};
MSKrescodee MSK_putintparam(MSKtask_t task, MSKiparame param,
                            MSKint32t parvalue);
MSKrescodee MSK_getintparam(MSKtask_t task, MSKiparame param,
                            MSKint32t * parvalue);
using MSKdparame = int;
MSKrescodee MSK_putdouparam(MSKtask_t task, MSKdparame param,
                            MSKrealt parvalue);
MSKrescodee MSK_getdouparam(MSKtask_t task, MSKdparame param,
                            MSKrealt * parvalue);

enum MSKsoltypee : int { MSK_SOL_ITR = 0, MSK_SOL_BAS = 1, MSK_SOL_ITG = 2 };
MSKrescodee MSK_optimize(MSKtask_t task);
enum MSKsolstae : int {
    MSK_SOL_STA_UNKNOWN = 0,
    MSK_SOL_STA_OPTIMAL = 1,
    MSK_SOL_STA_PRIM_FEAS = 2,
    MSK_SOL_STA_DUAL_FEAS = 3,
    MSK_SOL_STA_PRIM_AND_DUAL_FEAS = 4,
    MSK_SOL_STA_PRIM_INFEAS_CER = 5,
    MSK_SOL_STA_DUAL_INFEAS_CER = 6,
    MSK_SOL_STA_PRIM_ILLPOSED_CER = 7,
    MSK_SOL_STA_DUAL_ILLPOSED_CER = 8,
    MSK_SOL_STA_INTEGER_OPTIMAL = 9
};
MSKrescodee MSK_getsolsta(MSKtask_t task, MSKsoltypee whichsol,
                          MSKsolstae * solutionsta);
MSKrescodee MSK_getprimalobj(MSKtask_t task, MSKsoltypee whichsol,
                             MSKrealt * primalobj);
MSKrescodee MSK_getxx(MSKtask_t task, MSKsoltypee whichsol, MSKrealt * xx);
enum MSKprostae : int {
    MSK_PRO_STA_UNKNOWN = 0,
    MSK_PRO_STA_PRIM_AND_DUAL_FEAS = 1,
    MSK_PRO_STA_PRIM_FEAS = 2,
    MSK_PRO_STA_DUAL_FEAS = 3,
    MSK_PRO_STA_PRIM_INFEAS = 4,
    MSK_PRO_STA_DUAL_INFEAS = 5,
    MSK_PRO_STA_PRIM_AND_DUAL_INFEAS = 6,
    MSK_PRO_STA_ILL_POSED = 7,
    MSK_PRO_STA_PRIM_INFEAS_OR_UNBOUNDED = 8
};
enum MSKstakeye : int {
    MSK_SK_UNK = 0,
    MSK_SK_BAS = 1,
    MSK_SK_SUPBAS = 2,
    MSK_SK_LOW = 3,
    MSK_SK_UPR = 4,
    MSK_SK_FIX = 5,
    MSK_SK_INF = 6
};
MSKrescodee MSK_getsolution(MSKtask_t task, MSKsoltypee whichsol,
                            MSKprostae * problemsta, MSKsolstae * solutionsta,
                            MSKstakeye * skc, MSKstakeye * skx,
                            MSKstakeye * skn, MSKrealt * xc, MSKrealt * xx,
                            MSKrealt * y, MSKrealt * slc, MSKrealt * suc,
                            MSKrealt * slx, MSKrealt * sux, MSKrealt * snx);
MSKrescodee MSK_deletesolution(MSKtask_t task, MSKsoltypee whichsol);

}  // namespace mosek::v11
}  // namespace fhamonic::mippp
#endif

#include "dylib.hpp"

namespace fhamonic::mippp {
namespace mosek::v11 {

#define MOSEK_FUNCTIONS(F)                              \
    F(MSK_makeenv, makeenv)                             \
    F(MSK_makeemptytask, makeemptytask)                 \
    F(MSK_getprobtype, getprobtype)                     \
    F(MSK_deletetask, deletetask)                       \
    F(MSK_deleteenv, deleteenv)                         \
    F(MSK_getcodedesc, getcodedesc)                     \
    F(MSK_linkfunctotaskstream, linkfunctotaskstream)   \
    F(MSK_putobjsense, putobjsense)                     \
    F(MSK_getobjsense, getobjsense)                     \
    F(MSK_putcslice, putcslice)                         \
    F(MSK_putclist, putclist)                           \
    F(MSK_putcfix, putcfix)                             \
    F(MSK_getcfix, getcfix)                             \
    F(MSK_appendvars, appendvars)                       \
    F(MSK_appendcons, appendcons)                       \
    F(MSK_putcj, putcj)                                 \
    F(MSK_getcj, getcj)                                 \
    F(MSK_putvarbound, putvarbound)                     \
    F(MSK_putvarboundsliceconst, putvarboundsliceconst) \
    F(MSK_chgvarbound, chgvarbound)                     \
    F(MSK_getvarbound, getvarbound)                     \
    F(MSK_putvarname, putvarname)                       \
    F(MSK_getvarnamelen, getvarnamelen)                 \
    F(MSK_getvarname, getvarname)                       \
    F(MSK_putvartype, putvartype)                       \
    F(MSK_putvartypelist, putvartypelist)               \
    F(MSK_putarow, putarow)                             \
    F(MSK_getarow, getarow)                             \
    F(MSK_putarowslice, putarowslice)                   \
    F(MSK_putconbound, putconbound)                     \
    F(MSK_chgconbound, chgconbound)                     \
    F(MSK_getconbound, getconbound)                     \
    F(MSK_putconboundslice, putconboundslice)           \
    F(MSK_putconname, putconname)                       \
    F(MSK_getconnamelen, getconnamelen)                 \
    F(MSK_getconname, getconname)                       \
    F(MSK_getacol, getacol)                             \
    F(MSK_getaij, getaij)                               \
    F(MSK_getnumvar, getnumvar)                         \
    F(MSK_getnumcon, getnumcon)                         \
    F(MSK_getnumanz, getnumanz)                         \
    F(MSK_putintparam, putintparam)                     \
    F(MSK_getintparam, getintparam)                     \
    F(MSK_putdouparam, putdouparam)                     \
    F(MSK_getdouparam, getdouparam)                     \
    F(MSK_optimize, optimize)                           \
    F(MSK_getsolsta, getsolsta)                         \
    F(MSK_getprimalobj, getprimalobj)                   \
    F(MSK_getxx, getxx)                                 \
    F(MSK_getsolution, getsolution)                     \
    F(MSK_deletesolution, deletesolution)

#define DECLARE_MOSEK_FUN(FULL, SHORT)    \
    using SHORT##_fun_t = decltype(FULL); \
    SHORT##_fun_t * SHORT;
#define CONSTRUCT_MOSEK_FUN(FULL, SHORT) \
    , SHORT(lib.get_function<SHORT##_fun_t>(#FULL))

class mosek_api {
private:
    dylib lib;

public:
    MOSEK_FUNCTIONS(DECLARE_MOSEK_FUN)

public:
    inline mosek_api(const char * lib_path = "",
                     const char * lib_name = "mosek64")
        : lib(lib_path, lib_name) MOSEK_FUNCTIONS(CONSTRUCT_MOSEK_FUN) {}
};

}  // namespace mosek::v11
}  // namespace fhamonic::mippp

#endif  // MIPPP_MOSEK_v11_API_HPP