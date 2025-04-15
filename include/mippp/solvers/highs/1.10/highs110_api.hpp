#ifndef MIPPP_HIGHS_110_API_HPP
#define MIPPP_HIGHS_110_API_HPP

#include "dylib.hpp"

#include "interfaces/highs_c_api.h"

#define HIGHS_FUNCTIONS(F)                                              \
    F(Highs_create, create)                                             \
    F(Highs_destroy, destroy)                                           \
    F(Highs_changeObjectiveSense, changeObjectiveSense)                 \
    F(Highs_getObjectiveSense, getObjectiveSense)                       \
    F(Highs_changeObjectiveOffset, changeObjectiveOffset)               \
    F(Highs_getObjectiveOffset, getObjectiveOffset)                     \
    F(Highs_addVar, addVar)                                             \
    F(Highs_addVars, addVars)                                           \
    F(Highs_changeColCost, changeColCost)                               \
    F(Highs_changeColsCostByRange, changeColsCostByRange)               \
    F(Highs_changeColsCostBySet, changeColsCostBySet)                   \
    F(Highs_changeColBounds, changeColBounds)                           \
    F(Highs_changeColsBoundsByRange, changeColsBoundsByRange)           \
    F(Highs_getColsByRange, getColsByRange)                             \
    F(Highs_changeColIntegrality, changeColIntegrality)                 \
    F(Highs_changeColsIntegralityByRange, changeColsIntegralityByRange) \
    F(Highs_passColName, passColName)                                   \
    F(Highs_getColName, getColName)                                     \
    F(Highs_addCol, addCol)                                             \
    F(Highs_addCols, addCols)                                           \
    F(Highs_addRow, addRow)                                             \
    F(Highs_addRows, addRows)                                           \
    F(Highs_changeRowBounds, changeRowBounds)                           \
    F(Highs_passRowName, passRowName)                                   \
    F(Highs_getRowName, getRowName)                                     \
    F(Highs_getNumCol, getNumCol)                                       \
    F(Highs_getNumRow, getNumRow)                                       \
    F(Highs_getNumNz, getNumNz)                                         \
    F(Highs_getInfinity, getInfinity)                                   \
    F(Highs_getModelStatus, getModelStatus)                             \
    F(Highs_run, run)                                                   \
    F(Highs_getObjectiveValue, getObjectiveValue)                       \
    F(Highs_getSolution, getSolution)                                   \
    F(Highs_getBasis, getBasis)                                         \
    F(Highs_setBasis, setBasis)

#define DECLARE_HIGHS_FUN(FULL, SHORT)    \
    using SHORT##_fun_t = decltype(FULL); \
    SHORT##_fun_t * SHORT;
#define CONSTRUCT_HIGHS_FUN(FULL, SHORT) \
    , SHORT(lib.get_function<SHORT##_fun_t>(#FULL))

namespace fhamonic {
namespace mippp {

class highs110_api {
private:
    dylib lib;

public:
    HIGHS_FUNCTIONS(DECLARE_HIGHS_FUN)

public:
    inline highs110_api(const char * lib_name = "highs",
                     const char * lib_path = "")
        : lib(lib_path, lib_name) HIGHS_FUNCTIONS(CONSTRUCT_HIGHS_FUN) {}
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_HIGHS_110_API_HPP