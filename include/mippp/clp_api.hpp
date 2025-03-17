#ifndef MIPPP_CLP_API_HPP
#define MIPPP_CLP_API_HPP

#include "dylib.hpp"

#include "coin/Clp_C_Interface.h"

#define DECLARE_CLP_FUN(FN)                    \
    using CLP##FN##_fun_t = decltype(CLP##FN); \
    const CLP##FN##_fun_t * FN;

#define CONSTRUCT_CLP_FUN(FN) FN(lib.get_function<CLP##FN##_fun_t>("CLP" #FN))

namespace fhamonic {
namespace mippp {
#define DECLARE_CLP_FUN(FN)                      \
    using CLP_##FN##_fun_t = decltype(Clp_##FN); \
    CLP_##FN##_fun_t * FN;

#define CONSTRUCT_CLP_FUN(FN) FN(lib.get_function<CLP_##FN##_fun_t>("Clp_" #FN))

class clp_api {
private:
    dylib lib;

public:
    DECLARE_CLP_FUN(newModel)
    DECLARE_CLP_FUN(deleteModel)
    DECLARE_CLP_FUN(lengthNames)
    DECLARE_CLP_FUN(setObjSense)
    DECLARE_CLP_FUN(addColumns)
    DECLARE_CLP_FUN(columnLower)
    DECLARE_CLP_FUN(columnUpper)
    DECLARE_CLP_FUN(columnName)
    DECLARE_CLP_FUN(setColumnName)
    DECLARE_CLP_FUN(addRows)
    DECLARE_CLP_FUN(rowLower)
    DECLARE_CLP_FUN(rowUpper)
    DECLARE_CLP_FUN(rowName)
    DECLARE_CLP_FUN(setRowName)
    DECLARE_CLP_FUN(objective)
    DECLARE_CLP_FUN(objectiveOffset)
    DECLARE_CLP_FUN(setObjectiveOffset)
    DECLARE_CLP_FUN(getNumRows)
    DECLARE_CLP_FUN(getNumCols)
    DECLARE_CLP_FUN(getNumElements)
    DECLARE_CLP_FUN(primalTolerance)
    DECLARE_CLP_FUN(setPrimalTolerance)
    DECLARE_CLP_FUN(primal)
    DECLARE_CLP_FUN(getObjValue)
    DECLARE_CLP_FUN(primalRowSolution)
    DECLARE_CLP_FUN(primalColumnSolution)
    DECLARE_CLP_FUN(writeMps)

public:
    inline clp_api(const char * lib_name = "Clp", const char * lib_path = "")
        : lib(lib_path, lib_name)
        , CONSTRUCT_CLP_FUN(newModel)
        , CONSTRUCT_CLP_FUN(deleteModel)
        , CONSTRUCT_CLP_FUN(lengthNames)
        , CONSTRUCT_CLP_FUN(setObjSense)
        , CONSTRUCT_CLP_FUN(addColumns)
        , CONSTRUCT_CLP_FUN(columnLower)
        , CONSTRUCT_CLP_FUN(columnUpper)
        , CONSTRUCT_CLP_FUN(columnName)
        , CONSTRUCT_CLP_FUN(setColumnName)
        , CONSTRUCT_CLP_FUN(addRows)
        , CONSTRUCT_CLP_FUN(rowLower)
        , CONSTRUCT_CLP_FUN(rowUpper)
        , CONSTRUCT_CLP_FUN(rowName)
        , CONSTRUCT_CLP_FUN(setRowName)
        , CONSTRUCT_CLP_FUN(objective)
        , CONSTRUCT_CLP_FUN(objectiveOffset)
        , CONSTRUCT_CLP_FUN(setObjectiveOffset)
        , CONSTRUCT_CLP_FUN(getNumRows)
        , CONSTRUCT_CLP_FUN(getNumCols)
        , CONSTRUCT_CLP_FUN(getNumElements)
        , CONSTRUCT_CLP_FUN(primalTolerance)
        , CONSTRUCT_CLP_FUN(setPrimalTolerance)
        , CONSTRUCT_CLP_FUN(primal)
        , CONSTRUCT_CLP_FUN(getObjValue)
        , CONSTRUCT_CLP_FUN(primalRowSolution)
        , CONSTRUCT_CLP_FUN(primalColumnSolution)
        , CONSTRUCT_CLP_FUN(writeMps) {}
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_CLP_API_HPP