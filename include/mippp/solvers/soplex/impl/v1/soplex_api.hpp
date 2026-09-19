#pragma once

#include <array>

#include <filesystem>
#include <utility>

#if defined(MIPPP_INCLUDE_SOPLEX_HEADER) && MIPPP_INCLUDE_SOPLEX_HEADER
#include "soplex_interface.h"
#else

namespace mippp::soplex::impl::v1 {

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

}  // namespace mippp::soplex::impl::v1
#endif

namespace mippp::soplex::impl::v1 {
enum Status {
    ERROR_ = -15,  // ERROR clashes with a macro imported by <windows.h>...
    NO_RATIOTESTER = -14,
    NO_PRICER = -13,
    NO_SOLVER = -12,
    NOT_INIT = -11,
    ABORT_CYCLING = -8,
    ABORT_TIME = -7,
    ABORT_ITER = -6,
    ABORT_VALUE = -5,
    SINGULAR = -4,
    NO_PROBLEM = -3,
    REGULAR = -2,
    RUNNING = -1,
    UNKNOWN = 0,
    OPTIMAL = 1,
    UNBOUNDED = 2,
    INFEASIBLE = 3,
    INForUNBD = 4,
    OPTIMAL_UNSCALED_VIOLATIONS = 5
};
}  // namespace mippp::soplex::impl::v1

#include "mippp/detail/dynamic_library.hpp"

#include "mippp/detail/solver_library.hpp"

namespace mippp {
namespace soplex::impl::v1 {

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

#define DECLARE_SOPLEX_FUNCTIONS(FULL, SHORT) \
    using SHORT##_fun_t = decltype(FULL);     \
    SHORT##_fun_t * const SHORT;
#define CONSTRUCT_SOPLEX_FUNCTIONS(FULL, SHORT) \
    , SHORT(lib.get_function<SHORT##_fun_t>(#FULL))

class soplex_api : public detail::solver_api<soplex_api> {
    friend detail::solver_api<soplex_api>;

public:
    SOPLEX_FUNCTIONS(DECLARE_SOPLEX_FUNCTIONS)

    static constexpr const char * key = "SOPLEX";
    static constexpr std::array library_names = {"soplexshared"};
    // the releases driven through the full suite, see solver_version_range;
    // not checked at runtime, the C API reports no version
    static constexpr std::array validated_versions = {
        solver_version_range{{6, 0, 3}, {8, 0, 4}}};

private:
    explicit soplex_api(detail::dynamic_library && library)
        : solver_api(std::move(library))
              SOPLEX_FUNCTIONS(CONSTRUCT_SOPLEX_FUNCTIONS) {}
};

#undef CONSTRUCT_SOPLEX_FUNCTIONS
#undef DECLARE_SOPLEX_FUNCTIONS
#undef SOPLEX_FUNCTIONS

}  // namespace soplex::impl::v1
}  // namespace mippp
