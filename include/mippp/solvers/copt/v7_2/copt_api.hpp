#ifndef MIPPP_COPT_API_HPP
#define MIPPP_COPT_API_HPP

// #define INCLUDE_COPT_HEADER 1

#if INCLUDE_COPT_HEADER
#include "copt.h"
#else
namespace fhamonic::mippp {
namespace copt::v7_2 {

constexpr double COPT_INFINITY = 1e30;

using copt_env_config = struct copt_env_config_s;
using copt_env = struct copt_env_s;
using copt_prob = struct copt_prob_s;

enum ret_code : int {
    COPT_RETCODE_OK = 0,
    COPT_RETCODE_MEMORY = 1,
    COPT_RETCODE_FILE = 2,
    COPT_RETCODE_INVALID = 3,
    COPT_RETCODE_LICENSE = 4,
    COPT_RETCODE_INTERNAL = 5,
    COPT_RETCODE_THREAD = 6,
    COPT_RETCODE_SERVER = 7,
    COPT_RETCODE_NONCONVEX = 8
};
ret_code COPT_CreateEnv(copt_env ** p_env);
ret_code COPT_DeleteEnv(copt_env ** p_env);
ret_code COPT_CreateProb(copt_env * env, copt_prob ** p_prob);
ret_code COPT_DeleteProb(copt_prob ** p_prob);

ret_code COPT_GetIntAttr(copt_prob * prob, const char * attrName,
                         int * p_intAttr);
ret_code COPT_GetDblAttr(copt_prob * prob, const char * attrName,
                         double * p_dblAttr);

constexpr int COPT_MINIMIZE = 1;
constexpr int COPT_MAXIMIZE = -1;
ret_code COPT_SetObjSense(copt_prob * prob, int iObjSense);
constexpr const char * COPT_INTATTR_OBJSENSE = "ObjSense";
ret_code COPT_SetObjConst(copt_prob * prob, double dObjConst);
constexpr const char * COPT_DBLATTR_OBJCONST = "ObjConst";

ret_code COPT_AddCol(copt_prob * prob, double dColObj, int nColMatCnt,
                     const int * colMatIdx, const double * colMatElem,
                     char cColType, double dColLower, double dColUpper,
                     const char * colName);
ret_code COPT_AddCols(copt_prob * prob, int nAddCol, const double * colObj,
                      const int * colMatBeg, const int * colMatCnt,
                      const int * colMatIdx, const double * colMatElem,
                      const char * colType, const double * colLower,
                      const double * colUpper, char const * const * colNames);
ret_code COPT_AddRow(copt_prob * prob, int nRowMatCnt, const int * rowMatIdx,
                     const double * rowMatElem, char cRowSense,
                     double dRowBound, double dRowUpper, const char * rowName);
ret_code COPT_AddRows(copt_prob * prob, int nAddRow, const int * rowMatBeg,
                      const int * rowMatCnt, const int * rowMatIdx,
                      const double * rowMatElem, const char * rowSense,
                      const double * rowBound, const double * rowUpper,
                      char const * const * rowNames);
ret_code COPT_AddSOSs(copt_prob * prob, int nAddSOS, const int * sosType,
                      const int * sosMatBeg, const int * sosMatCnt,
                      const int * sosMatIdx, const double * sosMatWt);
ret_code COPT_AddIndicator(copt_prob * prob, int binColIdx, int binColVal,
                           int nRowMatCnt, const int * rowMatIdx,
                           const double * rowMatElem, char cRowSense,
                           double dRowBound);

//////////////////////////////////// Cols /////////////////////////////////////
ret_code COPT_ReplaceColObj(copt_prob * prob, int num, const int * list,
                            const double * obj);  // replaces all objective
ret_code COPT_SetColObj(copt_prob * prob, int num, const int * list,
                        const double * obj);
constexpr const char * COPT_DBLINFO_OBJ = "Obj";
ret_code COPT_SetColLower(copt_prob * prob, int num, const int * list,
                          const double * lower);
constexpr const char * COPT_DBLINFO_LB = "LB";
ret_code COPT_SetColUpper(copt_prob * prob, int num, const int * list,
                          const double * upper);
constexpr const char * COPT_DBLINFO_UB = "UB";
ret_code COPT_GetColInfo(copt_prob * prob, const char * infoName, int num,
                         const int * list, double * info);
ret_code COPT_SetColNames(copt_prob * prob, int num, const int * list,
                          char const * const * names);
ret_code COPT_GetColName(copt_prob * prob, int iCol, char * buff, int buffSize,
                         int * pReqSize);
constexpr char COPT_CONTINUOUS = 'C';
constexpr char COPT_BINARY = 'B';
constexpr char COPT_INTEGER = 'I';
ret_code COPT_SetColType(copt_prob * prob, int num, const int * list,
                         const char * type);
ret_code COPT_GetColType(copt_prob * prob, int num, const int * list,
                         char * type);

//////////////////////////////////// Rows /////////////////////////////////////
constexpr char COPT_LESS_EQUAL = 'L';
constexpr char COPT_GREATER_EQUAL = 'G';
constexpr char COPT_EQUAL = 'E';
constexpr char COPT_FREE = 'N';
constexpr char COPT_RANGE = 'R';
ret_code COPT_SetRowLower(copt_prob * prob, int num, const int * list,
                          const double * lower);
ret_code COPT_SetRowUpper(copt_prob * prob, int num, const int * list,
                          const double * upper);
ret_code COPT_SetRowNames(copt_prob * prob, int num, const int * list,
                          char const * const * names);
ret_code COPT_GetRowName(copt_prob * prob, int iRow, char * buff, int buffSize,
                         int * pReqSize);
constexpr const char * COPT_INTATTR_COLS = "Cols";
constexpr const char * COPT_INTATTR_ROWS = "Rows";
constexpr const char * COPT_INTATTR_ELEMS = "Elems";
constexpr const char * COPT_INTATTR_BINS = "Bins";
constexpr const char * COPT_INTATTR_INTS = "Ints";
ret_code COPT_AddMipStart(copt_prob * prob, int num, const int * list,
                          const double * colVal);

//////////////////////////////////// Solve ////////////////////////////////////
constexpr const char * COPT_INTATTR_ISMIP = "IsMIP";
ret_code COPT_SolveLp(copt_prob * prob);
ret_code COPT_Solve(copt_prob * prob);
constexpr const char * COPT_INTATTR_LPSTATUS = "LpStatus";
constexpr const char * COPT_DBLATTR_LPOBJVAL = "LpObjval";
constexpr const char * COPT_DBLATTR_BESTOBJ = "BestObj";
ret_code COPT_GetSolution(copt_prob * prob, double * colVal);
ret_code COPT_GetLpSolution(copt_prob * prob, double * value, double * slack,
                            double * rowDual, double * redCost);
ret_code COPT_SetLpSolution(copt_prob * prob, const double * value,
                            const double * slack, const double * rowDual,
                            const double * redCost);
ret_code COPT_GetBasis(copt_prob * prob, int * colBasis, int * rowBasis);
ret_code COPT_SetBasis(copt_prob * prob, const int * colBasis,
                       const int * rowBasis);

}  // namespace copt::v7_2
}  // namespace fhamonic::mippp
#endif

#include "dylib.hpp"

namespace fhamonic::mippp {
namespace copt::v7_2 {

#define COPT_FUNCTIONS(F)                \
    F(COPT_CreateEnv, CreateEnv)         \
    F(COPT_DeleteEnv, DeleteEnv)         \
    F(COPT_CreateProb, CreateProb)       \
    F(COPT_DeleteProb, DeleteProb)       \
    F(COPT_GetIntAttr, GetIntAttr)       \
    F(COPT_GetDblAttr, GetDblAttr)       \
    F(COPT_SetObjSense, SetObjSense)     \
    F(COPT_SetObjConst, SetObjConst)     \
    F(COPT_AddCol, AddCol)               \
    F(COPT_AddCols, AddCols)             \
    F(COPT_AddRow, AddRow)               \
    F(COPT_AddRows, AddRows)             \
    F(COPT_AddSOSs, AddSOSs)             \
    F(COPT_AddIndicator, AddIndicator)   \
    F(COPT_ReplaceColObj, ReplaceColObj) \
    F(COPT_SetColObj, SetColObj)         \
    F(COPT_SetColLower, SetColLower)     \
    F(COPT_SetColUpper, SetColUpper)     \
    F(COPT_GetColInfo, GetColInfo)       \
    F(COPT_SetColNames, SetColNames)     \
    F(COPT_GetColName, GetColName)       \
    F(COPT_SetColType, SetColType)       \
    F(COPT_GetColType, GetColType)       \
    F(COPT_SetRowLower, SetRowLower)     \
    F(COPT_SetRowUpper, SetRowUpper)     \
    F(COPT_SetRowNames, SetRowNames)     \
    F(COPT_GetRowName, GetRowName)       \
    F(COPT_AddMipStart, AddMipStart)     \
    F(COPT_SolveLp, SolveLp)             \
    F(COPT_Solve, Solve)                 \
    F(COPT_GetSolution, GetSolution)     \
    F(COPT_GetLpSolution, GetLpSolution) \
    F(COPT_SetLpSolution, SetLpSolution) \
    F(COPT_GetBasis, GetBasis)           \
    F(COPT_SetBasis, SetBasis)

#define DECLARE_COPT_FUN(FULL, SHORT)     \
    using SHORT##_fun_t = decltype(FULL); \
    SHORT##_fun_t * SHORT;
#define CONSTRUCT_COPT_FUN(FULL, SHORT) \
    , SHORT(lib.get_function<SHORT##_fun_t>(#FULL))

class copt_api {
private:
    dylib lib;

public:
    COPT_FUNCTIONS(DECLARE_COPT_FUN)

public:
    inline copt_api(const char * lib_path = "", const char * lib_name = "copt")
        : lib(lib_path, lib_name) COPT_FUNCTIONS(CONSTRUCT_COPT_FUN) {}
};

}  // namespace copt::v7_2
}  // namespace fhamonic::mippp

#endif  // MIPPP_COPT_API_HPP