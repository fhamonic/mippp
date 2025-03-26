#ifndef MIPPP_COPT_API_HPP
#define MIPPP_COPT_API_HPP

#include "dylib.hpp"

// #include "copt.h"

#define COPT_FUNCTIONS(F) F(COPT_newModel, newModel)

#define DECLARE_COPT_FUN(FULL, SHORT)   \
    using SHORT##_fun_t = decltype(FULL); \
    SHORT##_fun_t * SHORT;
#define CONSTRUCT_COPT_FUN(FULL, SHORT) \
    , SHORT(lib.get_function<SHORT##_fun_t>(#FULL))

namespace fhamonic {
namespace mippp {

class copt_api {
private:
    dylib lib;

public:
    COPT_FUNCTIONS(DECLARE_COPT_FUN)

public:
    inline copt_api(const char * lib_name = "copt", const char * lib_path = "")
        : lib(lib_path, lib_name) COPT_FUNCTIONS(CONSTRUCT_COPT_FUN) {}
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_COPT_API_HPP