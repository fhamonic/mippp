#pragma once

#include <cstdint>

#if INCLUDE_HIGHS_HEADER
#include "interfaces/highs_c_api.h"
#else
namespace mippp {
namespace highs::v1_10 {

using HighsInt = int;
double Highs_getInfinity(const void * highs);

const char * Highs_version(void);
HighsInt Highs_versionMajor(void);
HighsInt Highs_versionMinor(void);
HighsInt Highs_versionPatch(void);

void * Highs_create(void);
void Highs_destroy(void * highs);

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
HighsInt Highs_deleteColsBySet(void * highs, const HighsInt num_set_entries,
                               const HighsInt * set);
HighsInt Highs_deleteColsByMask(void * highs, HighsInt * mask);

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
HighsInt Highs_getColsBySet(const void * highs, const HighsInt num_set_entries,
                            const HighsInt * set, HighsInt * num_col,
                            double * costs, double * lower, double * upper,
                            HighsInt * num_nz, HighsInt * matrix_start,
                            HighsInt * matrix_index, double * matrix_value);

HighsInt Highs_addRow(void * highs, const double lower, const double upper,
                      const HighsInt num_new_nz, const HighsInt * index,
                      const double * value);
HighsInt Highs_addRows(void * highs, const HighsInt num_new_row,
                       const double * lower, const double * upper,
                       const HighsInt num_new_nz, const HighsInt * starts,
                       const HighsInt * index, const double * value);

enum HessianFromat : HighsInt {
    kHighsHessianFormatTriangular = 1,
    kHighsHessianFormatSquare = 2
};
HighsInt Highs_passHessian(void * highs, const HighsInt dim,
                           const HighsInt num_nz, const HighsInt format,
                           const HighsInt * start, const HighsInt * index,
                           const double * value);

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

HighsInt Highs_getNumOptions(const void * highs);
HighsInt Highs_getOptionName(const void * highs, const HighsInt index,
                             char ** name);
HighsInt Highs_setBoolOptionValue(void * highs, const char * option,
                                  const HighsInt value);
HighsInt Highs_setIntOptionValue(void * highs, const char * option,
                                 const HighsInt value);
HighsInt Highs_setDoubleOptionValue(void * highs, const char * option,
                                    const double value);
HighsInt Highs_setStringOptionValue(void * highs, const char * option,
                                    const char * value);
HighsInt Highs_getBoolOptionValue(const void * highs, const char * option,
                                  HighsInt * value);
HighsInt Highs_getIntOptionValue(const void * highs, const char * option,
                                 HighsInt * value);
HighsInt Highs_getDoubleOptionValue(const void * highs, const char * option,
                                    double * value);
HighsInt Highs_getStringOptionValue(const void * highs, const char * option,
                                    char * value);

enum SolutionStatus : HighsInt {
    kHighsSolutionStatusNone = 0,
    kHighsSolutionStatusInfeasible = 1,
    kHighsSolutionStatusFeasible = 2
};

HighsInt Highs_getIntInfoValue(const void * highs, const char * info,
                               HighsInt * value);
HighsInt Highs_getInt64InfoValue(const void * highs, const char * info,
                                 int64_t * value);
HighsInt Highs_getDoubleInfoValue(const void * highs, const char * info,
                                  double * value);

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

HighsInt Highs_setSolution(void * highs, const double * col_value,
                           const double * row_value, const double * col_dual,
                           const double * row_dual);
HighsInt Highs_setSparseSolution(void * highs, const HighsInt num_entries,
                                 const HighsInt * index, const double * value);

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
}  // namespace mippp
#endif

#include "dylib.hpp"

#include "mippp/detail/solver_library.hpp"
#include "mippp/utility/solver_exceptions.hpp"

namespace mippp {
namespace highs::v1_10 {

#define HIGHS_FUNCTIONS(F)                                              \
    F(Highs_getInfinity, getInfinity)                                   \
    F(Highs_version, version)                                           \
    F(Highs_versionMajor, versionMajor)                                 \
    F(Highs_versionMinor, versionMinor)                                 \
    F(Highs_versionPatch, versionPatch)                                 \
    F(Highs_create, create)                                             \
    F(Highs_destroy, destroy)                                           \
    F(Highs_changeObjectiveSense, changeObjectiveSense)                 \
    F(Highs_getObjectiveSense, getObjectiveSense)                       \
    F(Highs_changeObjectiveOffset, changeObjectiveOffset)               \
    F(Highs_getObjectiveOffset, getObjectiveOffset)                     \
    F(Highs_addVar, addVar)                                             \
    F(Highs_addVars, addVars)                                           \
    F(Highs_addCol, addCol)                                             \
    F(Highs_addCols, addCols)                                           \
    F(Highs_deleteColsBySet, deleteColsBySet)                           \
    F(Highs_deleteColsByMask, deleteColsByMask)                         \
    F(Highs_changeColCost, changeColCost)                               \
    F(Highs_changeColsCostByRange, changeColsCostByRange)               \
    F(Highs_changeColsCostBySet, changeColsCostBySet)                   \
    F(Highs_changeColBounds, changeColBounds)                           \
    F(Highs_getColsByRange, getColsByRange)                             \
    F(Highs_getColsBySet, getColsBySet)                                 \
    F(Highs_addRow, addRow)                                             \
    F(Highs_addRows, addRows)                                           \
    F(Highs_passHessian, passHessian)                                   \
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
    F(Highs_getNumOptions, getNumOptions)                               \
    F(Highs_getOptionName, getOptionName)                               \
    F(Highs_setBoolOptionValue, setBoolOptionValue)                     \
    F(Highs_setIntOptionValue, setIntOptionValue)                       \
    F(Highs_setDoubleOptionValue, setDoubleOptionValue)                 \
    F(Highs_setStringOptionValue, setStringOptionValue)                 \
    F(Highs_getBoolOptionValue, getBoolOptionValue)                     \
    F(Highs_getIntOptionValue, getIntOptionValue)                       \
    F(Highs_getDoubleOptionValue, getDoubleOptionValue)                 \
    F(Highs_getStringOptionValue, getStringOptionValue)                 \
    F(Highs_getIntInfoValue, getIntInfoValue)                           \
    F(Highs_getInt64InfoValue, getInt64InfoValue)                       \
    F(Highs_getDoubleInfoValue, getDoubleInfoValue)                     \
    F(Highs_run, run)                                                   \
    F(Highs_getModelStatus, getModelStatus)                             \
    F(Highs_getObjectiveValue, getObjectiveValue)                       \
    F(Highs_getSolution, getSolution)                                   \
    F(Highs_setBasis, setBasis)                                         \
    F(Highs_getBasis, getBasis)                                         \
    F(Highs_setSolution, setSolution)                                   \
    F(Highs_setSparseSolution, setSparseSolution)

#define DECLARE_HIGHS_FUNCTIONS(FULL, SHORT) \
    using SHORT##_fun_t = decltype(FULL);    \
    SHORT##_fun_t const * SHORT;
#define CONSTRUCT_HIGHS_FUNCTIONS(FULL, SHORT) \
    , SHORT(lib.get_function<SHORT##_fun_t>(#FULL))

class highs_api {
private:
    dylib::library lib;

public:
    HIGHS_FUNCTIONS(DECLARE_HIGHS_FUNCTIONS)

public:
    inline highs_api(const char * lib_path = nullptr)
        : lib(detail::load_solver_library(lib_path, "HIGHS", {"highs"}))
              HIGHS_FUNCTIONS(CONSTRUCT_HIGHS_FUNCTIONS) {}

    void _check(const int status) const {
        if(status == kHighsStatusOk) return;
        if(status == kHighsStatusError)
            throw solver_error("HiGHS kHighsStatusError");
        if(status == kHighsStatusWarning)
            throw solver_error("HiGHS kHighsStatusWarning");
    }
};

#undef CONSTRUCT_HIGHS_FUNCTIONS
#undef DECLARE_HIGHS_FUNCTIONS
#undef HIGHS_FUNCTIONS

}  // namespace highs::v1_10
}  // namespace mippp
