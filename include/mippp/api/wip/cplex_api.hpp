#ifndef MIPPP_SOPLEX_API_HPP
#define MIPPP_SOPLEX_API_HPP

#include "dylib.hpp"

#include "cplex.h"

#define SOPLEX_FUNCTIONS(F)                              \
    F(SOPLEX_newModel, newModel)                         \
    F(SOPLEX_deleteModel, deleteModel)                   \
    F(SOPLEX_lengthNames, lengthNames)                   \
    F(SOPLEX_setObjSense, setObjSense)                   \
    F(SOPLEX_addColumns, addColumns)                     \
    F(SOPLEX_columnLower, columnLower)                   \
    F(SOPLEX_columnUpper, columnUpper)                   \
    F(SOPLEX_columnName, columnName)                     \
    F(SOPLEX_setColumnName, setColumnName)               \
    F(SOPLEX_addRows, addRows)                           \
    F(SOPLEX_rowLower, rowLower)                         \
    F(SOPLEX_rowUpper, rowUpper)                         \
    F(SOPLEX_rowName, rowName)                           \
    F(SOPLEX_setRowName, setRowName)                     \
    F(SOPLEX_objective, objective)                       \
    F(SOPLEX_objectiveOffset, objectiveOffset)           \
    F(SOPLEX_setObjectiveOffset, setObjectiveOffset)     \
    F(SOPLEX_getNumRows, getNumRows)                     \
    F(SOPLEX_getNumCols, getNumCols)                     \
    F(SOPLEX_getNumElements, getNumElements)             \
    F(SOPLEX_primalTolerance, primalTolerance)           \
    F(SOPLEX_setPrimalTolerance, setPrimalTolerance)     \
    F(SOPLEX_primal, primal)                             \
    F(SOPLEX_getObjValue, getObjValue)                   \
    F(SOPLEX_primalRowSolution, primalRowSolution)       \
    F(SOPLEX_primalColumnSolution, primalColumnSolution) \
    F(SOPLEX_writeMps, writeMps)

#define DECLARE_SOPLEX_FUN(FULL, SHORT)      \
    using SHORT##_fun_t = decltype(FULL); \
    SHORT##_fun_t * SHORT;
#define CONSTRUCT_SOPLEX_FUN(FULL, SHORT) \
    , SHORT(lib.get_function<SHORT##_fun_t>(#FULL))

namespace fhamonic {
namespace mippp {

class soplex_api {
private:
    dylib lib;

public:
    SOPLEX_FUNCTIONS(DECLARE_SOPLEX_FUN)

public:
    inline soplex_api(const char * lib_name = "soplexshared", const char * lib_path = "")
        : lib(lib_path, lib_name) SOPLEX_FUNCTIONS(CONSTRUCT_SOPLEX_FUN) {}
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_SOPLEX_API_HPP