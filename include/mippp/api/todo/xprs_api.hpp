#ifndef MIPPP_XPRESS_API_HPP
#define MIPPP_XPRESS_API_HPP

#include "dylib.hpp"

#include "xprs.h"

#define XPRESS_FUNCTIONS(F) F(XPRESS_newModel, newModel)

#define DECLARE_XPRESS_FUN(FULL, SHORT)   \
    using SHORT##_fun_t = decltype(FULL); \
    SHORT##_fun_t * SHORT;
#define CONSTRUCT_XPRESS_FUN(FULL, SHORT) \
    , SHORT(lib.get_function<SHORT##_fun_t>(#FULL))

namespace fhamonic {
namespace mippp {

class xprs_api {
private:
    dylib lib;

public:
    XPRESS_FUNCTIONS(DECLARE_XPRESS_FUN)

public:
    inline xprs_api(const char * lib_name = "xprs", const char * lib_path = "")
        : lib(lib_path, lib_name) XPRESS_FUNCTIONS(CONSTRUCT_XPRESS_FUN) {}
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_XPRESS_API_HPP