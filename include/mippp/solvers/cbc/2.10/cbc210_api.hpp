#ifndef MIPPP_CBC_210_API_HPP
#define MIPPP_CBC_210_API_HPP

#include "dylib.hpp"

#include "coin/Cbc_C_Interface.h"
#include "coin/CoinFinite.hpp"

#define CBC_FUNCTIONS(F)                                    \
    F(Cbc_newModel, newModel)                               \
    F(Cbc_deleteModel, deleteModel)                         \
    F(Cbc_setObjSense, setObjSense)                         \
    F(Cbc_addCol, addCol)                                   \
    F(Cbc_setObjCoeff, setObjCoeff)                         \
    F(Cbc_setColLower, setColLower)                         \
    F(Cbc_setColUpper, setColUpper)                         \
    F(Cbc_setColName, setColName)                           \
    F(Cbc_setContinuous, setContinuous)                     \
    F(Cbc_setInteger, setInteger)                           \
    F(Cbc_getObjCoefficients, getObjCoefficients)           \
    F(Cbc_getColLower, getColLower)                         \
    F(Cbc_getColUpper, getColUpper)                         \
    F(Cbc_getColName, getColName)                           \
    F(Cbc_isInteger, isInteger)                             \
    F(Cbc_addRow, addRow)                                   \
    F(Cbc_setRowName, setRowName)                           \
    F(Cbc_setRowLower, setRowLower)                         \
    F(Cbc_setRowUpper, setRowUpper)                         \
    F(Cbc_getRowSense, getRowSense)                         \
    F(Cbc_getRowRHS, getRowRHS)                             \
    F(Cbc_getRowLower, getRowLower)                         \
    F(Cbc_getRowUpper, getRowUpper)                         \
    F(Cbc_getRowName, getRowName)                           \
    F(Cbc_getNumElements, getNumElements)                   \
    F(Cbc_getNumCols, getNumCols)                           \
    F(Cbc_getNumIntegers, getNumIntegers)                   \
    F(Cbc_getNumRows, getNumRows)                           \
    F(Cbc_maxNameLength, maxNameLength)                     \
    F(Cbc_setMIPStartI, setMIPStartI)                       \
    F(Cbc_solve, solve)                                     \
    F(Cbc_getColSolution, getColSolution)                   \
    F(Cbc_getObjValue, getObjValue)                         \
    F(Cbc_getBestPossibleObjValue, getBestPossibleObjValue) \
    F(Cbc_bestSolution, bestSolution)                       \
    F(Cbc_setParameter, setParameter)                       \
    F(Cbc_setAllowableGap, setAllowableGap)                 \
    F(Cbc_getAllowableGap, getAllowableGap)

#define DECLARE_CBC_FUN(FULL, SHORT)      \
    using SHORT##_fun_t = decltype(FULL); \
    SHORT##_fun_t * SHORT;
#define CONSTRUCT_CBC_FUN(FULL, SHORT) \
    , SHORT(lib.get_function<SHORT##_fun_t>(#FULL))

namespace fhamonic {
namespace mippp {

class cbc210_api {
private:
    dylib lib;

public:
    CBC_FUNCTIONS(DECLARE_CBC_FUN)

public:
    inline cbc210_api(const char * lib_name = "CbcSolver", const char * lib_path = "")
        : lib(lib_path, lib_name) CBC_FUNCTIONS(CONSTRUCT_CBC_FUN) {}
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_CBC_210_API_HPP
