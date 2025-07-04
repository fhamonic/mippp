#ifndef MIPPP_SCIP_v8_API_HPP
#define MIPPP_SCIP_v8_API_HPP

#include <cstddef>
#include <limits>

#if INCLUDE_SCIP_HEADER
#include <scip/scipdefplugins.h>
#include "scip/cons_linear.h"
#include "scip/retcode.h"
#include "scip/scip.h"
#else
namespace fhamonic::mippp {
namespace scip::v8 {

using SCIP_Real = double;
constexpr SCIP_Real SCIP_REAL_MAX = std::numeric_limits<SCIP_Real>::max();
constexpr SCIP_Real SCIP_REAL_MIN = -std::numeric_limits<SCIP_Real>::lowest();
using SCIP_Longint = long long;
using SCIP_Bool = unsigned int;

using SCIP = struct Scip;
using SCIP_VAR = struct SCIP_Var;
using SCIP_CONS = struct SCIP_Cons;
using SCIP_SOL = struct SCIP_Sol;

enum SCIP_RETCODE {
    SCIP_OKAY = +1,             // normal termination
    SCIP_ERROR = 0,             // unspecified error
    SCIP_NOMEMORY = -1,         // insufficient memory error
    SCIP_READERROR = -2,        // read error
    SCIP_WRITEERROR = -3,       // write error
    SCIP_NOFILE = -4,           // file not found error
    SCIP_FILECREATEERROR = -5,  // cannot create file
    SCIP_LPERROR = -6,          // error in LP solver
    SCIP_NOPROBLEM = -7,        // no problem exists
    SCIP_INVALIDCALL =
        -8,  // method cannot be called at this time in solution process
    SCIP_INVALIDDATA = -9,      // error in input data
    SCIP_INVALIDRESULT = -10,   // method returned an invalid result code
    SCIP_PLUGINNOTFOUND = -11,  // a required plugin was not found
    SCIP_PARAMETERUNKNOWN =
        -12,  // the parameter with the given name was not found
    SCIP_PARAMETERWRONGTYPE = -13,  // the parameter is not of the expected type
    SCIP_PARAMETERWRONGVAL =
        -14,  // the value is invalid for the given parameter
    SCIP_KEYALREADYEXISTING =
        -15,                   // the given key is already existing in table
    SCIP_MAXDEPTHLEVEL = -16,  // maximal branching depth level exceeded
    SCIP_BRANCHERROR = -17,    // no branching could be created
    SCIP_NOTIMPLEMENTED = -18  // function not implemented
};

SCIP_Real SCIPinfinity(SCIP * scip);
SCIP_RETCODE SCIPcreate(SCIP ** scip);
SCIP_RETCODE SCIPincludeDefaultPlugins(SCIP * scip);
SCIP_RETCODE SCIPcreateProbBasic(SCIP * scip, const char * name);
SCIP_RETCODE SCIPreleaseVar(SCIP * scip, SCIP_VAR ** var);
SCIP_RETCODE SCIPreleaseCons(SCIP * scip, SCIP_CONS ** cons);
SCIP_RETCODE SCIPfree(SCIP ** scip);

enum SCIP_OBJSENSE : int {
    SCIP_OBJSENSE_MAXIMIZE = -1,
    SCIP_OBJSENSE_MINIMIZE = +1
};
SCIP_RETCODE SCIPsetObjsense(SCIP * scip, SCIP_OBJSENSE objsense);
SCIP_OBJSENSE SCIPgetObjsense(SCIP * scip);
SCIP_RETCODE SCIPaddOrigObjoffset(SCIP * scip, SCIP_Real addval);
SCIP_Real SCIPgetOrigObjoffset(SCIP * scip);

enum SCIP_VARTYPE : int {
    SCIP_VARTYPE_BINARY = 0,
    SCIP_VARTYPE_INTEGER = 1,
    SCIP_VARTYPE_IMPLINT =
        2,  // implicit integer variable: Integrality of this variable is
            // implied for every optimal solution of the remaining problem after
            // any fixing all integer and binary variables, without the explicit
            // need to enforce integrality further
    SCIP_VARTYPE_CONTINUOUS = 3
};
SCIP_RETCODE SCIPcreateVarBasic(SCIP * scip, SCIP_VAR ** var, const char * name,
                                SCIP_Real lb, SCIP_Real ub, SCIP_Real obj,
                                SCIP_VARTYPE vartype);
SCIP_RETCODE SCIPaddVar(SCIP * scip, SCIP_VAR * var);
SCIP_RETCODE SCIPcreateConsBasicLinear(SCIP * scip, SCIP_CONS ** cons,
                                       const char * name, int nnzs,
                                       SCIP_VAR ** vars, SCIP_Real * vals,
                                       SCIP_Real lhs, SCIP_Real rhs);
SCIP_RETCODE SCIPaddCons(SCIP * scip, SCIP_CONS * cons);

SCIP_RETCODE SCIPchgVarObj(SCIP * scip, SCIP_VAR * var, SCIP_Real newobj);
SCIP_Real SCIPvarGetObj(SCIP_VAR * var);
SCIP_RETCODE SCIPchgVarLb(SCIP * scip, SCIP_VAR * var, SCIP_Real newbound);
SCIP_RETCODE SCIPchgVarUb(SCIP * scip, SCIP_VAR * var, SCIP_Real newbound);
SCIP_Real SCIPvarGetLbGlobal(SCIP_VAR * var);
SCIP_Real SCIPvarGetUbGlobal(SCIP_VAR * var);
SCIP_RETCODE SCIPchgVarType(SCIP * scip, SCIP_VAR * var, SCIP_VARTYPE vartype,
                            SCIP_Bool * become_infeasible);
const char * SCIPvarGetName(SCIP_VAR * var);
SCIP_RETCODE SCIPchgVarName(SCIP * scip, SCIP_VAR * var, const char * name);

SCIP_RETCODE SCIPaddCoefLinear(SCIP * scip, SCIP_CONS * cons, SCIP_VAR * var,
                               SCIP_Real val);

int SCIPgetNVars(SCIP * scip);
int SCIPgetNContVars(SCIP * scip);
int SCIPgetNIntVars(SCIP * scip);
int SCIPgetNBinVars(SCIP * scip);
int SCIPgetNConss(SCIP * scip);
SCIP_Longint SCIPgetNNZs(SCIP * scip);

SCIP_RETCODE SCIPsetRealParam(SCIP * scip, const char * name, SCIP_Real value);
SCIP_RETCODE SCIPgetRealParam(SCIP * scip, const char * name,
                              SCIP_Real * value);

SCIP_RETCODE SCIPsolve(SCIP * scip);
SCIP_Real SCIPgetPrimalbound(SCIP * scip);
SCIP_SOL * SCIPgetBestSol(SCIP * scip);
SCIP_Real SCIPgetSolVal(SCIP * scip, SCIP_SOL * sol, SCIP_VAR * var);

// SCIPfeastol

// SCIPcreateEmptyRowConshdlr
// SCIPcacheRowExtensions
// SCIPaddVarToRow
// SCIPflushRowExtensions
// SCIPaddRow

using SCIP_CONSHDLR = struct SCIP_Conshdlr;

enum SCIP_RESULT {
    SCIP_DIDNOTRUN = 1, /**< the method was not executed */
    SCIP_DELAYED =
        2, /**< the method was not executed, but should be called again later */
    SCIP_DIDNOTFIND =
        3, /**< the method was executed, but failed finding anything */
    SCIP_FEASIBLE = 4,   /**< no infeasibility could be found */
    SCIP_INFEASIBLE = 5, /**< an infeasibility was detected */
    SCIP_UNBOUNDED = 6,  /**< an unboundedness was detected */
    SCIP_CUTOFF = 7, /**< the current node is infeasible and can be cut off */
    SCIP_SEPARATED = 8,    /**< the method added a cutting plane */
    SCIP_NEWROUND = 9,     /**< the method added a cutting plane and a new
                              separation round should immediately start */
    SCIP_REDUCEDDOM = 10,  /**< the method reduced the domain of a variable */
    SCIP_CONSADDED = 11,   /**< the method added a constraint */
    SCIP_CONSCHANGED = 12, /**< the method changed a constraint */
    SCIP_BRANCHED = 13,    /**< the method created a branching */
    SCIP_SOLVELP = 14,     /**< the current node's LP must be solved */
    SCIP_FOUNDSOL = 15,    /**< the method found a feasible primal solution */
    SCIP_SUSPENDED = 16,   /**< the method interrupted its execution, but can
                              continue if needed */
    SCIP_SUCCESS = 17,     /**< the method was successfully executed */
    SCIP_DELAYNODE = 18 /**< the processing of the branch-and-bound node should
                           stopped and continued later */
};

#define SCIP_DECL_CONSENFOLP(x)                                               \
    SCIP_RETCODE x(SCIP * scip, SCIP_CONSHDLR * conshdlr, SCIP_CONS ** conss, \
                   int nconss, int nusefulconss, SCIP_Bool solinfeasible,     \
                   SCIP_RESULT * result)

#define SCIP_DECL_CONSENFOPS(x)                                               \
    SCIP_RETCODE x(SCIP * scip, SCIP_CONSHDLR * conshdlr, SCIP_CONS ** conss, \
                   int nconss, int nusefulconss, SCIP_Bool solinfeasible,     \
                   SCIP_Bool objinfeasible, SCIP_RESULT * result)

#define SCIP_DECL_CONSCHECK(x)                                                \
    SCIP_RETCODE x(SCIP * scip, SCIP_CONSHDLR * conshdlr, SCIP_CONS ** conss, \
                   int nconss, SCIP_SOL * sol, SCIP_Bool checkintegrality,    \
                   SCIP_Bool checklprows, SCIP_Bool printreason,              \
                   SCIP_Bool completely, SCIP_RESULT * result)

enum SCIP_LOCKTYPE {
    SCIP_LOCKTYPE_MODEL =
        0, /**< variable locks for model and check constraints */
    SCIP_LOCKTYPE_CONFLICT = 1 /**< variable locks for conflict constraints */
};
#define SCIP_DECL_CONSLOCK(x)                                               \
    SCIP_RETCODE x(SCIP * scip, SCIP_CONSHDLR * conshdlr, SCIP_CONS * cons, \
                   SCIP_LOCKTYPE locktype, int nlockspos, int nlocksneg)

using SCIP_CONSHDLRDATA = struct SCIP_ConshdlrData;

SCIP_RETCODE SCIPincludeConshdlrBasic(
    SCIP * scip, /**< SCIP data structure */
    SCIP_CONSHDLR **
        conshdlrptr, /**< reference to a constraint handler pointer, or NULL */
    const char * name,   /**< name of constraint handler */
    const char * desc,   /**< description of constraint handler */
    int enfopriority,    /**< priority of the constraint handler for constraint
                            enforcing */
    int chckpriority,    /**< priority of the constraint handler for checking
                            feasibility (and propagation) */
    int eagerfreq,       /**< frequency for using all instead of only the useful
                          * constraints in separation,       propagation and
                          * enforcement, -1       for no eager evaluations, 0 for
                          * first only
                          */
    SCIP_Bool needscons, /**< should the constraint handler be skipped, if no
                            constraints are available? */
    SCIP_DECL_CONSENFOLP(
        (*consenfolp)), /**< enforcing constraints for LP solutions */
    SCIP_DECL_CONSENFOPS(
        (*consenfops)), /**< enforcing constraints for pseudo solutions */
    SCIP_DECL_CONSCHECK(
        (*conscheck)), /**< check feasibility of primal solution */
    SCIP_DECL_CONSLOCK((*conslock)), /**< variable rounding lock method */
    SCIP_CONSHDLRDATA * conshdlrdata /**< constraint handler data */
);

}  // namespace scip::v8
}  // namespace fhamonic::mippp
#endif

#include "dylib.hpp"

namespace fhamonic::mippp {
namespace scip::v8 {

#define SCIP_FUNCTIONS(F)                               \
    F(SCIPcreate, create)                               \
    F(SCIPincludeDefaultPlugins, includeDefaultPlugins) \
    F(SCIPcreateProbBasic, createProbBasic)             \
    F(SCIPreleaseVar, releaseVar)                       \
    F(SCIPreleaseCons, releaseCons)                     \
    F(SCIPfree, free)                                   \
    F(SCIPinfinity, infinity)                           \
    F(SCIPsetRealParam, setRealParam)                   \
    F(SCIPgetRealParam, getRealParam)                   \
    F(SCIPsetObjsense, setObjsense)                     \
    F(SCIPaddOrigObjoffset, addOrigObjoffset)           \
    F(SCIPgetOrigObjoffset, getOrigObjoffset)           \
    F(SCIPcreateVarBasic, createVarBasic)               \
    F(SCIPaddVar, addVar)                               \
    F(SCIPchgVarObj, chgVarObj)                         \
    F(SCIPvarGetObj, varGetObj)                         \
    F(SCIPchgVarLb, chgVarLb)                           \
    F(SCIPchgVarUb, chgVarUb)                           \
    F(SCIPvarGetLbGlobal, varGetLbGlobal)               \
    F(SCIPvarGetUbGlobal, varGetUbGlobal)               \
    F(SCIPchgVarType, chgVarType)                       \
    F(SCIPchgVarName, chgVarName)                       \
    F(SCIPvarGetName, varGetName)                       \
    F(SCIPcreateConsBasicLinear, createConsBasicLinear) \
    F(SCIPaddCons, addCons)                             \
    F(SCIPaddCoefLinear, addCoefLinear)                 \
    F(SCIPgetNVars, getNVars)                           \
    F(SCIPgetNContVars, getNContVars)                   \
    F(SCIPgetNIntVars, getNIntVars)                     \
    F(SCIPgetNBinVars, getNBinVars)                     \
    F(SCIPgetNConss, getNConss)                         \
    F(SCIPgetNNZs, getNNZs)                             \
    F(SCIPsolve, solve)                                 \
    F(SCIPgetPrimalbound, getPrimalbound)               \
    F(SCIPgetBestSol, getBestSol)                       \
    F(SCIPgetSolVal, getSolVal)

#define DECLARE_SCIP_FUN(FULL, SHORT)     \
    using SHORT##_fun_t = decltype(FULL); \
    SHORT##_fun_t * SHORT;
#define CONSTRUCT_SCIP_FUN(FULL, SHORT) \
    , SHORT(lib.get_function<SHORT##_fun_t>(#FULL))

class scip_api {
private:
    dylib lib;

public:
    SCIP_FUNCTIONS(DECLARE_SCIP_FUN)

public:
    inline scip_api(const char * lib_path = "", const char * lib_name = "scip")
        : lib(lib_path, lib_name) SCIP_FUNCTIONS(CONSTRUCT_SCIP_FUN) {}
};

}  // namespace scip::v8
}  // namespace fhamonic::mippp

#endif  // MIPPP_SCIP_v8_API_HPP