#ifndef MIPPP_SCIP_API_HPP
#define MIPPP_SCIP_API_HPP

#include "dylib.hpp"

#include <scip/scipdefplugins.h>
#include "scip/cons_linear.h"
#include "scip/retcode.h"
#include "scip/scip.h"

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

// SCIP_OBJSENSE_MINIMIZE
// SCIP_OBJSENSE_MAXIMIZE

// SCIP_VARTYPE_CONTINUOUS
// SCIP_VARTYPE_INTEGER
// SCIP_VARTYPE_BINARY

#define DECLARE_SCIP_FUN(FULL, SHORT)     \
    using SHORT##_fun_t = decltype(FULL); \
    SHORT##_fun_t * SHORT;
#define CONSTRUCT_SCIP_FUN(FULL, SHORT) \
    , SHORT(lib.get_function<SHORT##_fun_t>(#FULL))

namespace fhamonic {
namespace mippp {

class scip_api {
private:
    dylib lib;

public:
    SCIP_FUNCTIONS(DECLARE_SCIP_FUN)

public:
    inline scip_api(const char * lib_name = "scip", const char * lib_path = "")
        : lib(lib_path, lib_name) SCIP_FUNCTIONS(CONSTRUCT_SCIP_FUN) {}
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_SCIP_API_HPP