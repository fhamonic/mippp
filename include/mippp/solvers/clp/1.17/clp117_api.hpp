#ifndef MIPPP_CLP_117_API_HPP
#define MIPPP_CLP_117_API_HPP

#include "dylib.hpp"

#include "coin/Clp_C_Interface.h"
#include "coin/CoinFinite.hpp"

#define CLP_FUNCTIONS(F)                              \
    F(Clp_newModel, newModel)                         \
    F(Clp_deleteModel, deleteModel)                   \
    F(Clp_lengthNames, lengthNames)                   \
    F(Clp_setObjSense, setObjSense)                   \
    F(Clp_addColumns, addColumns)                     \
    F(Clp_columnLower, columnLower)                   \
    F(Clp_columnUpper, columnUpper)                   \
    F(Clp_columnName, columnName)                     \
    F(Clp_setColumnName, setColumnName)               \
    F(Clp_addRows, addRows)                           \
    F(Clp_rowLower, rowLower)                         \
    F(Clp_rowUpper, rowUpper)                         \
    F(Clp_rowName, rowName)                           \
    F(Clp_setRowName, setRowName)                     \
    F(Clp_objective, objective)                       \
    F(Clp_objectiveOffset, objectiveOffset)           \
    F(Clp_setObjectiveOffset, setObjectiveOffset)     \
    F(Clp_getNumRows, getNumRows)                     \
    F(Clp_getNumCols, getNumCols)                     \
    F(Clp_getNumElements, getNumElements)             \
    F(Clp_primalTolerance, primalTolerance)           \
    F(Clp_setPrimalTolerance, setPrimalTolerance)     \
    F(Clp_getColumnStatus, getColumnStatus)           \
    F(Clp_getRowStatus, getRowStatus)                 \
    F(Clp_setColumnStatus, setColumnStatus)           \
    F(Clp_setRowStatus, setRowStatus)                 \
    F(Clp_initialSolve, initialSolve)                 \
    F(Clp_primal, primal)                             \
    F(Clp_status, status)                             \
    F(Clp_getObjValue, getObjValue)                   \
    F(Clp_dualRowSolution, dualRowSolution)           \
    F(Clp_primalColumnSolution, primalColumnSolution) \
    F(Clp_writeMps, writeMps)

#define DECLARE_CLP_FUN(FULL, SHORT)      \
    using SHORT##_fun_t = decltype(FULL); \
    SHORT##_fun_t * SHORT;
#define CONSTRUCT_CLP_FUN(FULL, SHORT) \
    , SHORT(lib.get_function<SHORT##_fun_t>(#FULL))

namespace fhamonic {
namespace mippp {

class clp117_api {
private:
    dylib lib;

public:
    CLP_FUNCTIONS(DECLARE_CLP_FUN)

public:
    inline clp117_api(const char * lib_path = "", const char * lib_name = "Clp")
        : lib(lib_path, lib_name) CLP_FUNCTIONS(CONSTRUCT_CLP_FUN) {}
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_CLP_117_API_HPP