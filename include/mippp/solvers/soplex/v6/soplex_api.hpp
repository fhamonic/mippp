#ifndef MIPPP_SOPLEX_v6_API_HPP
#define MIPPP_SOPLEX_v6_API_HPP

#if INCLUDE_SOPLEX_HEADER
#include "soplex_interface.h"
#else
namespace fhamonic::mippp {
namespace soplex::v6 {

void * SoPlex_create();
void SoPlex_free(void * soplex);

void SoPlex_addColReal(void * soplex, double * colentries, int colsize,
                       int nnonzeros, double objval, double lb, double ub);
void SoPlex_addRowReal(void * soplex, double * rowentries, int rowsize,
                       int nnonzeros, double lb, double ub);

void SoPlex_changeObjReal(void * soplex, double * obj, int dim);

int SoPlex_numCols(void * soplex);
int SoPlex_numRows(void * soplex);

void SoPlex_setIntParam(void * soplex, int paramcode, int paramvalue);
int SoPlex_getIntParam(void * soplex, int paramcode);

int SoPlex_optimize(void * soplex);
double SoPlex_objValueReal(void * soplex);
void SoPlex_getPrimalReal(void * soplex, double * primal, int dim);
void SoPlex_getDualReal(void * soplex, double * dual, int dim);

}  // namespace soplex::v6
}  // namespace fhamonic::mippp
#endif

#include "dylib.hpp"

namespace fhamonic::mippp {
namespace soplex::v6 {

#define SOPLEX_FUNCTIONS(F)                \
    F(SoPlex_create, create)               \
    F(SoPlex_free, free)                   \
    F(SoPlex_changeObjReal, changeObjReal) \
    F(SoPlex_addColReal, addColReal)       \
    F(SoPlex_addRowReal, addRowReal)       \
    F(SoPlex_numRows, numRows)             \
    F(SoPlex_numCols, numCols)             \
    F(SoPlex_setIntParam, setIntParam)     \
    F(SoPlex_getIntParam, getIntParam)     \
    F(SoPlex_optimize, optimize)           \
    F(SoPlex_objValueReal, objValueReal)   \
    F(SoPlex_getPrimalReal, getPrimalReal) \
    F(SoPlex_getDualReal, getDualReal)

#define DECLARE_SOPLEX_FUN(FULL, SHORT)   \
    using SHORT##_fun_t = decltype(FULL); \
    SHORT##_fun_t * SHORT;
#define CONSTRUCT_SOPLEX_FUN(FULL, SHORT) \
    , SHORT(lib.get_function<SHORT##_fun_t>(#FULL))

class soplex_api {
private:
    dylib lib;

public:
    SOPLEX_FUNCTIONS(DECLARE_SOPLEX_FUN)

public:
    inline soplex_api(const char * lib_path = "",
                      const char * lib_name = "soplexshared")
        : lib(lib_path, lib_name) SOPLEX_FUNCTIONS(CONSTRUCT_SOPLEX_FUN) {}
};

}  // namespace soplex::v6
}  // namespace fhamonic::mippp

#endif  // MIPPP_SOPLEX_v6_API_HPP