#ifndef MIPPP_CLP_v1_17_API_HPP
#define MIPPP_CLP_v1_17_API_HPP

#if INCLUDE_CLP_HEADER
#include "coin/Clp_C_Interface.h"
#include "coin/CoinFinite.hpp"
#else
#include <limits>
namespace fhamonic::mippp {
namespace clp::v1_17 {

constexpr double COIN_DBL_MAX = std::numeric_limits<double>::max();
using Clp_Simplex = void;
using CoinBigIndex = int;

Clp_Simplex * Clp_newModel(void);
void Clp_deleteModel(Clp_Simplex * model);

void Clp_setObjSense(Clp_Simplex * model, double objsen);
void Clp_setObjectiveOffset(Clp_Simplex * model, double value);
double Clp_objectiveOffset(Clp_Simplex * model);

void Clp_addColumns(Clp_Simplex * model, int number, const double * columnLower,
                    const double * columnUpper, const double * objective,
                    const CoinBigIndex * columnStarts, const int * rows,
                    const double * elements);
void Clp_addRows(Clp_Simplex * model, int number, const double * rowLower,
                 const double * rowUpper, const CoinBigIndex * rowStarts,
                 const int * columns, const double * elements);

double * Clp_objective(Clp_Simplex * model);
double * Clp_columnLower(Clp_Simplex * model);
double * Clp_columnUpper(Clp_Simplex * model);
void Clp_setColumnName(Clp_Simplex * model, int iColumn, char * name);
void Clp_columnName(Clp_Simplex * model, int iColumn, char * name);

double * Clp_rowLower(Clp_Simplex * model);
double * Clp_rowUpper(Clp_Simplex * model);
void Clp_setRowName(Clp_Simplex * model, int iRow, char * name);
void Clp_rowName(Clp_Simplex * model, int iRow, char * name);

int Clp_getNumCols(Clp_Simplex * model);
int Clp_getNumRows(Clp_Simplex * model);
int Clp_getNumElements(Clp_Simplex * model);
int Clp_lengthNames(Clp_Simplex * model);

double Clp_primalTolerance(Clp_Simplex * model);
void Clp_setPrimalTolerance(Clp_Simplex * model, double value);

void Clp_setColumnStatus(Clp_Simplex * model, int sequence, int value);
int Clp_getColumnStatus(Clp_Simplex * model, int sequence);
void Clp_setRowStatus(Clp_Simplex * model, int sequence, int value);
int Clp_getRowStatus(Clp_Simplex * model, int sequence);

int Clp_initialSolve(Clp_Simplex * model);
int Clp_primal(Clp_Simplex * model, int ifValuesPass);
int Clp_status(Clp_Simplex * model);
double Clp_getObjValue(Clp_Simplex * model);
double * Clp_primalColumnSolution(Clp_Simplex * model);
double * Clp_dualRowSolution(Clp_Simplex * model);

}  // namespace clp::v1_17
}  // namespace fhamonic::mippp
#endif

#include "dylib.hpp"

namespace fhamonic::mippp {
namespace clp::v1_17 {

#define CLP_FUNCTIONS(F)                              \
    F(Clp_newModel, newModel)                         \
    F(Clp_deleteModel, deleteModel)                   \
    F(Clp_setObjSense, setObjSense)                   \
    F(Clp_setObjectiveOffset, setObjectiveOffset)     \
    F(Clp_objectiveOffset, objectiveOffset)           \
    F(Clp_addColumns, addColumns)                     \
    F(Clp_addRows, addRows)                           \
    F(Clp_objective, objective)                       \
    F(Clp_columnLower, columnLower)                   \
    F(Clp_columnUpper, columnUpper)                   \
    F(Clp_setColumnName, setColumnName)               \
    F(Clp_columnName, columnName)                     \
    F(Clp_rowLower, rowLower)                         \
    F(Clp_rowUpper, rowUpper)                         \
    F(Clp_setRowName, setRowName)                     \
    F(Clp_rowName, rowName)                           \
    F(Clp_getNumCols, getNumCols)                     \
    F(Clp_getNumRows, getNumRows)                     \
    F(Clp_getNumElements, getNumElements)             \
    F(Clp_lengthNames, lengthNames)                   \
    F(Clp_primalTolerance, primalTolerance)           \
    F(Clp_setPrimalTolerance, setPrimalTolerance)     \
    F(Clp_setColumnStatus, setColumnStatus)           \
    F(Clp_getColumnStatus, getColumnStatus)           \
    F(Clp_setRowStatus, setRowStatus)                 \
    F(Clp_getRowStatus, getRowStatus)                 \
    F(Clp_initialSolve, initialSolve)                 \
    F(Clp_primal, primal)                             \
    F(Clp_status, status)                             \
    F(Clp_getObjValue, getObjValue)                   \
    F(Clp_primalColumnSolution, primalColumnSolution) \
    F(Clp_dualRowSolution, dualRowSolution)

#define DECLARE_CLP_FUN(FULL, SHORT)      \
    using SHORT##_fun_t = decltype(FULL); \
    SHORT##_fun_t * SHORT;
#define CONSTRUCT_CLP_FUN(FULL, SHORT) \
    , SHORT(lib.get_function<SHORT##_fun_t>(#FULL))

class clp_api {
private:
    dylib lib;

public:
    CLP_FUNCTIONS(DECLARE_CLP_FUN)

public:
    inline clp_api(const char * lib_path = "", const char * lib_name = "Clp")
        : lib(lib_path, lib_name) CLP_FUNCTIONS(CONSTRUCT_CLP_FUN) {}
};

}  // namespace clp::v1_17
}  // namespace fhamonic::mippp

#endif  // MIPPP_CLP_v1_17_API_HPP