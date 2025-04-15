#ifndef MIPPP_SOPLEX_6_API_HPP
#define MIPPP_SOPLEX_6_API_HPP

#include "dylib.hpp"

#include "soplex_interface.h"

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

namespace fhamonic {
namespace mippp {

class soplex6_api {
private:
    dylib lib;

public:
    SOPLEX_FUNCTIONS(DECLARE_SOPLEX_FUN)

public:
    inline soplex6_api(const char * lib_name = "soplexshared",
                      const char * lib_path = "")
        : lib(lib_path, lib_name) SOPLEX_FUNCTIONS(CONSTRUCT_SOPLEX_FUN) {}
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_SOPLEX_6_API_HPP