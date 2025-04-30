#ifndef MIPPP_CBC_v2_10_12_API_HPP
#define MIPPP_CBC_v2_10_12_API_HPP

#if INCLUDE_CBC_HEADER
#include "coin/Cbc_C_Interface.h"
#include "coin/CoinFinite.hpp"
#else
#include <limits>
namespace fhamonic::mippp {
namespace cbc::v2_10_12 {

constexpr double COIN_DBL_MAX = std::numeric_limits<double>::max();
using Cbc_Model = void;

const char * Cbc_getVersion(void);
Cbc_Model * Cbc_newModel(void);
void Cbc_deleteModel(Cbc_Model * model);

void Cbc_setObjSense(Cbc_Model * model, double sense);

void Cbc_addCol(Cbc_Model * model, const char * name, double lb, double ub,
                double obj, char isInteger, int nz, int * rows, double * coefs);
void Cbc_addRow(Cbc_Model * model, const char * name, int nz, const int * cols,
                const double * coefs, char sense, double rhs);
void Cbc_addSOS(Cbc_Model * model, int numRows, const int * rowStarts,
                const int * colIndices, const double * weights, const int type);

void Cbc_setObjCoeff(Cbc_Model * model, int index, double value);
const double * Cbc_getObjCoefficients(Cbc_Model * model);
void Cbc_setColLower(Cbc_Model * model, int index, double value);
const double * Cbc_getColLower(Cbc_Model * model);
void Cbc_setColUpper(Cbc_Model * model, int index, double value);
const double * Cbc_getColUpper(Cbc_Model * model);
void Cbc_setColName(Cbc_Model * model, int iColumn, const char * name);
void Cbc_getColName(Cbc_Model * model, int iColumn, char * name,
                    size_t maxLength);

int Cbc_getRowNz(Cbc_Model * model, int row);
const int * Cbc_getRowIndices(Cbc_Model * model, int row);
const double * Cbc_getRowCoeffs(Cbc_Model * model, int row);
char Cbc_getRowSense(Cbc_Model * model, int row);
double Cbc_getRowRHS(Cbc_Model * model, int row);
void Cbc_setRowLower(Cbc_Model * model, int index, double value);
void Cbc_setRowUpper(Cbc_Model * model, int index, double value);
const double * Cbc_getRowLower(Cbc_Model * model);
const double * Cbc_getRowUpper(Cbc_Model * model);
void Cbc_getRowName(Cbc_Model * model, int iRow, char * name, size_t maxLength);

void Cbc_setContinuous(Cbc_Model * model, int iColumn);
void Cbc_setInteger(Cbc_Model * model, int iColumn);
int Cbc_isInteger(Cbc_Model * model, int i);

int Cbc_getNumCols(Cbc_Model * model);
int Cbc_getNumIntegers(Cbc_Model * model);
int Cbc_getNumRows(Cbc_Model * model);
int Cbc_getNumElements(Cbc_Model * model);
size_t Cbc_maxNameLength(Cbc_Model * model);

void Cbc_setMIPStartI(Cbc_Model * model, int count, const int colIdxs[],
                      const double colValues[]);

void Cbc_setParameter(Cbc_Model * model, const char * name, const char * value);
double Cbc_getAllowableFractionGap(Cbc_Model * model);
void Cbc_setAllowableFractionGap(Cbc_Model * model, double allowedFracionGap);

int Cbc_solve(Cbc_Model * model);
int Cbc_status(Cbc_Model * model);
int Cbc_isProvenOptimal(Cbc_Model * model);
int Cbc_isProvenInfeasible(Cbc_Model * model);
int Cbc_isContinuousUnbounded(Cbc_Model * model);
double Cbc_getObjValue(Cbc_Model * model);
const double * Cbc_getColSolution(Cbc_Model * model);

}  // namespace cbc::v2_10_12
}  // namespace fhamonic::mippp
#endif

#include "dylib.hpp"

namespace fhamonic::mippp {
namespace cbc::v2_10_12 {

#define CBC_FUNCTIONS(F)                                    \
    F(Cbc_getVersion, getVersion)                           \
    F(Cbc_newModel, newModel)                               \
    F(Cbc_deleteModel, deleteModel)                         \
    F(Cbc_setObjSense, setObjSense)                         \
    F(Cbc_addCol, addCol)                                   \
    F(Cbc_addRow, addRow)                                   \
    F(Cbc_addSOS, addSOS)                                   \
    F(Cbc_setObjCoeff, setObjCoeff)                         \
    F(Cbc_getObjCoefficients, getObjCoefficients)           \
    F(Cbc_setColLower, setColLower)                         \
    F(Cbc_getColLower, getColLower)                         \
    F(Cbc_setColUpper, setColUpper)                         \
    F(Cbc_getColUpper, getColUpper)                         \
    F(Cbc_setColName, setColName)                           \
    F(Cbc_getColName, getColName)                           \
    F(Cbc_getRowNz, getRowNz)                               \
    F(Cbc_getRowIndices, getRowIndices)                     \
    F(Cbc_getRowCoeffs, getRowCoeffs)                       \
    F(Cbc_getRowSense, getRowSense)                         \
    F(Cbc_getRowRHS, getRowRHS)                             \
    F(Cbc_setRowLower, setRowLower)                         \
    F(Cbc_setRowUpper, setRowUpper)                         \
    F(Cbc_getRowLower, getRowLower)                         \
    F(Cbc_getRowUpper, getRowUpper)                         \
    F(Cbc_getRowName, getRowName)                           \
    F(Cbc_setContinuous, setContinuous)                     \
    F(Cbc_setInteger, setInteger)                           \
    F(Cbc_isInteger, isInteger)                             \
    F(Cbc_getNumCols, getNumCols)                           \
    F(Cbc_getNumIntegers, getNumIntegers)                   \
    F(Cbc_getNumRows, getNumRows)                           \
    F(Cbc_getNumElements, getNumElements)                   \
    F(Cbc_maxNameLength, maxNameLength)                     \
    F(Cbc_setMIPStartI, setMIPStartI)                       \
    F(Cbc_setParameter, setParameter)                       \
    F(Cbc_getAllowableFractionGap, getAllowableFractionGap) \
    F(Cbc_setAllowableFractionGap, setAllowableFractionGap) \
    F(Cbc_solve, solve)                                     \
    F(Cbc_status, status)                                   \
    F(Cbc_isProvenOptimal, isProvenOptimal)                 \
    F(Cbc_isProvenInfeasible, isProvenInfeasible)           \
    F(Cbc_isContinuousUnbounded, isContinuousUnbounded)     \
    F(Cbc_getObjValue, getObjValue)                         \
    F(Cbc_getColSolution, getColSolution)

#define DECLARE_CBC_FUN(FULL, SHORT)      \
    using SHORT##_fun_t = decltype(FULL); \
    decltype(FULL) * SHORT;
#define CONSTRUCT_CBC_FUN(FULL, SHORT) \
    , SHORT(lib.get_function<decltype(FULL)>(#FULL))

class cbc_api {
private:
    dylib lib;

public:
    CBC_FUNCTIONS(DECLARE_CBC_FUN)

public:
    inline cbc_api(const char * lib_path = "", const char * lib_name = "Cbc")
        : lib(lib_path, lib_name) CBC_FUNCTIONS(CONSTRUCT_CBC_FUN) {}
};

}  // namespace cbc::v2_10_12
}  // namespace fhamonic::mippp

#endif  // MIPPP_CBC_v2_10_12_API_HPP
