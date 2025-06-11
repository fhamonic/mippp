#ifndef MIPPP_HIGHS_v1_10_API_HPP
#define MIPPP_HIGHS_v1_10_API_HPP

#include <cstdint>

#if INCLUDE_HIGHS_HEADER
#include "interfaces/highs_c_api.h"
#else
namespace fhamonic::mippp {
namespace highs::v1_10 {

using HighsInt = int;
double Highs_getInfinity(const void * highs);

void * Highs_create(void);
void Highs_destroy(void * highs);

HighsInt Highs_setHighsDoubleOptionValue(void * highs, const char * option,
                                         const double value);
HighsInt Highs_getHighsDoubleOptionValue(const void * highs,
                                         const char * option, double * value);
HighsInt Highs_setHighsStringOptionValue(void * highs, const char * option,
                                         const char * value);
HighsInt Highs_getHighsStringOptionValue(const void * highs,
                                         const char * option, char * value);

// return status
constexpr HighsInt kHighsStatusError = -1;
constexpr HighsInt kHighsStatusOk = 0;
constexpr HighsInt kHighsStatusWarning = 1;
// objective sense
enum ObjSense : HighsInt {
    kHighsObjSenseMinimize = 1,
    kHighsObjSenseMaximize = -1
};
HighsInt Highs_changeObjectiveSense(void * highs, const HighsInt sense);
HighsInt Highs_getObjectiveSense(const void * highs, HighsInt * sense);
HighsInt Highs_changeObjectiveOffset(void * highs, const double offset);
HighsInt Highs_getObjectiveOffset(const void * highs, double * offset);

HighsInt Highs_addVar(void * highs, const double lower, const double upper);
HighsInt Highs_addVars(void * highs, const HighsInt num_new_var,
                       const double * lower, const double * upper);
HighsInt Highs_addCol(void * highs, const double cost, const double lower,
                      const double upper, const HighsInt num_new_nz,
                      const HighsInt * index, const double * value);
HighsInt Highs_addCols(void * highs, const HighsInt num_new_col,
                       const double * costs, const double * lower,
                       const double * upper, const HighsInt num_new_nz,
                       const HighsInt * starts, const HighsInt * index,
                       const double * value);
HighsInt Highs_addRow(void * highs, const double lower, const double upper,
                      const HighsInt num_new_nz, const HighsInt * index,
                      const double * value);
HighsInt Highs_addRows(void * highs, const HighsInt num_new_row,
                       const double * lower, const double * upper,
                       const HighsInt num_new_nz, const HighsInt * starts,
                       const HighsInt * index, const double * value);

HighsInt Highs_changeColCost(void * highs, const HighsInt col,
                             const double cost);
HighsInt Highs_changeColsCostByRange(void * highs, const HighsInt from_col,
                                     const HighsInt to_col,
                                     const double * cost);
HighsInt Highs_changeColsCostBySet(void * highs, const HighsInt num_set_entries,
                                   const HighsInt * set, const double * cost);
HighsInt Highs_changeColBounds(void * highs, const HighsInt col,
                               const double lower, const double upper);
HighsInt Highs_getColsByRange(const void * highs, const HighsInt from_col,
                              const HighsInt to_col, HighsInt * num_col,
                              double * costs, double * lower, double * upper,
                              HighsInt * num_nz, HighsInt * matrix_start,
                              HighsInt * matrix_index, double * matrix_value);
enum VarType : HighsInt {
    kHighsVarTypeContinuous = 0,
    kHighsVarTypeInteger = 1
};
HighsInt Highs_changeColIntegrality(void * highs, const HighsInt col,
                                    const HighsInt integrality);
HighsInt Highs_changeColsIntegralityByRange(void * highs,
                                            const HighsInt from_col,
                                            const HighsInt to_col,
                                            const HighsInt * integrality);
HighsInt Highs_getColIntegrality(const void * highs, const HighsInt col,
                                 HighsInt * integrality);
HighsInt Highs_passColName(const void * highs, const HighsInt col,
                           const char * name);
constexpr HighsInt kHighsMaximumStringLength = 512;
HighsInt Highs_getColName(const void * highs, const HighsInt col, char * name);

HighsInt Highs_changeRowBounds(void * highs, const HighsInt row,
                               const double lower, const double upper);
HighsInt Highs_getRowsByRange(const void * highs, const HighsInt from_row,
                              const HighsInt to_row, HighsInt * num_row,
                              double * lower, double * upper, HighsInt * num_nz,
                              HighsInt * matrix_start, HighsInt * matrix_index,
                              double * matrix_value);
HighsInt Highs_passRowName(const void * highs, const HighsInt row,
                           const char * name);
HighsInt Highs_getRowName(const void * highs, const HighsInt row, char * name);

HighsInt Highs_getNumCol(const void * highs);
HighsInt Highs_getNumRow(const void * highs);
HighsInt Highs_getNumNz(const void * highs);

HighsInt Highs_run(void * highs);
enum ModelStatus : HighsInt {
    kHighsModelStatusNotset = 0,
    kHighsModelStatusLoadError = 1,
    kHighsModelStatusModelError = 2,
    kHighsModelStatusPresolveError = 3,
    kHighsModelStatusSolveError = 4,
    kHighsModelStatusPostsolveError = 5,
    kHighsModelStatusModelEmpty = 6,
    kHighsModelStatusOptimal = 7,
    kHighsModelStatusInfeasible = 8,
    kHighsModelStatusUnboundedOrInfeasible = 9,
    kHighsModelStatusUnbounded = 10,
    kHighsModelStatusObjectiveBound = 11,
    kHighsModelStatusObjectiveTarget = 12,
    kHighsModelStatusTimeLimit = 13,
    kHighsModelStatusIterationLimit = 14,
    kHighsModelStatusUnknown = 15,
    kHighsModelStatusSolutionLimit = 16,
    kHighsModelStatusInterrupt = 17
};
HighsInt Highs_getModelStatus(const void * highs);
double Highs_getObjectiveValue(const void * highs);
HighsInt Highs_getSolution(const void * highs, double * col_value,
                           double * col_dual, double * row_value,
                           double * row_dual);
enum BasisStatus : HighsInt {
    kHighsBasisStatusLower = 0,  // non-basic at lower bound
    kHighsBasisStatusBasic = 1,  // basic
    kHighsBasisStatusUpper = 2,  // non-basic at upper bound
    kHighsBasisStatusZero = 3,
    kHighsBasisStatusNonbasic = 4  // non-basic
};
HighsInt Highs_setBasis(void * highs, const HighsInt * col_status,
                        const HighsInt * row_status);
HighsInt Highs_getBasis(const void * highs, HighsInt * col_status,
                        HighsInt * row_status);

struct HighsCallbackDataOut {
    int log_type;  // cast of HighsLogType
    double running_time;
    HighsInt simplex_iteration_count;
    HighsInt ipm_iteration_count;
    HighsInt pdlp_iteration_count;
    double objective_function_value;
    int64_t mip_node_count;
    int64_t mip_total_lp_iterations;
    double mip_primal_bound;
    double mip_dual_bound;
    double mip_gap;
    double * mip_solution;
    HighsInt cutpool_num_col;
    HighsInt cutpool_num_cut;
    HighsInt cutpool_num_nz;
    HighsInt * cutpool_start;
    HighsInt * cutpool_index;
    double * cutpool_value;
    double * cutpool_lower;
    double * cutpool_upper;
    HighsInt user_solution_callback_origin;
};
struct HighsCallbackDataIn {
    int user_interrupt;
    double * user_solution;
};
using HighsCCallbackType = void(int, const char *, const HighsCallbackDataOut *,
                                HighsCallbackDataIn *, void *);

HighsInt Highs_setCallback(void * highs, HighsCCallbackType user_callback,
                           void * user_callback_data);
HighsInt Highs_startCallback(void * highs, const int callback_type);

}  // namespace highs::v1_10
}  // namespace fhamonic::mippp
#endif

#include "dylib.hpp"

namespace fhamonic::mippp {
namespace highs::v1_10 {

#define HIGHS_FUNCTIONS(F)                                              \
    F(Highs_getInfinity, getInfinity)                                   \
    F(Highs_create, create)                                             \
    F(Highs_destroy, destroy)                                           \
    F(Highs_setHighsDoubleOptionValue, setHighsDoubleOptionValue)       \
    F(Highs_getHighsDoubleOptionValue, getHighsDoubleOptionValue)       \
    F(Highs_setHighsStringOptionValue, setHighsStringOptionValue)       \
    F(Highs_getHighsStringOptionValue, getHighsStringOptionValue)       \
    F(Highs_changeObjectiveSense, changeObjectiveSense)                 \
    F(Highs_getObjectiveSense, getObjectiveSense)                       \
    F(Highs_changeObjectiveOffset, changeObjectiveOffset)               \
    F(Highs_getObjectiveOffset, getObjectiveOffset)                     \
    F(Highs_addVar, addVar)                                             \
    F(Highs_addVars, addVars)                                           \
    F(Highs_addCol, addCol)                                             \
    F(Highs_addCols, addCols)                                           \
    F(Highs_addRow, addRow)                                             \
    F(Highs_addRows, addRows)                                           \
    F(Highs_changeColCost, changeColCost)                               \
    F(Highs_changeColsCostByRange, changeColsCostByRange)               \
    F(Highs_changeColsCostBySet, changeColsCostBySet)                   \
    F(Highs_changeColBounds, changeColBounds)                           \
    F(Highs_getColsByRange, getColsByRange)                             \
    F(Highs_changeColIntegrality, changeColIntegrality)                 \
    F(Highs_changeColsIntegralityByRange, changeColsIntegralityByRange) \
    F(Highs_getColIntegrality, getColIntegrality)                       \
    F(Highs_passColName, passColName)                                   \
    F(Highs_getColName, getColName)                                     \
    F(Highs_changeRowBounds, changeRowBounds)                           \
    F(Highs_getRowsByRange, getRowsByRange)                             \
    F(Highs_passRowName, passRowName)                                   \
    F(Highs_getRowName, getRowName)                                     \
    F(Highs_getNumCol, getNumCol)                                       \
    F(Highs_getNumRow, getNumRow)                                       \
    F(Highs_getNumNz, getNumNz)                                         \
    F(Highs_run, run)                                                   \
    F(Highs_getModelStatus, getModelStatus)                             \
    F(Highs_getObjectiveValue, getObjectiveValue)                       \
    F(Highs_getSolution, getSolution)                                   \
    F(Highs_setBasis, setBasis)                                         \
    F(Highs_getBasis, getBasis)

#define DECLARE_HIGHS_FUN(FULL, SHORT)    \
    using SHORT##_fun_t = decltype(FULL); \
    SHORT##_fun_t * SHORT;
#define CONSTRUCT_HIGHS_FUN(FULL, SHORT) \
    , SHORT(lib.get_function<SHORT##_fun_t>(#FULL))

class highs_api {
private:
    dylib lib;

public:
    HIGHS_FUNCTIONS(DECLARE_HIGHS_FUN)

public:
    inline highs_api(const char * lib_path = "",
                     const char * lib_name = "highs")
        : lib(lib_path, lib_name) HIGHS_FUNCTIONS(CONSTRUCT_HIGHS_FUN) {}
};

}  // namespace highs::v1_10
}  // namespace fhamonic::mippp

#endif  // MIPPP_HIGHS_v1_10_API_HPP