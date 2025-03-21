#ifndef MIPPP_SCIP_API_HPP
#define MIPPP_SCIP_API_HPP

#include "dylib.hpp"

// #include "coin/SCIP_C_Interface.h"
// #include "coin/CoinFinite.hpp"

#define SCIP_FUNCTIONS(F)                              \
    F(SCIP_newModel, newModel)

#define DECLARE_SCIP_FUN(FULL, SHORT)      \
    using SHORT##_fun_t = decltype(FULL); \
    SHORT##_fun_t * SHORT;
#define CONSTRUCT_SCIP_FUN(FULL, SHORT) \
    , SHORT(lib.get_function<SHORT##_fun_t>(#FULL))

namespace fhamonic {
namespace mippp {

class scip_api {
private:
    dylib lib;

public:
    SCIP_FUNCTIONS(DECLARE_SCIP_FUN)

public:
    inline scip_api(const char * lib_name = "scip", const char * lib_path = "")
        : lib(lib_path, lib_name) SCIP_FUNCTIONS(CONSTRUCT_SCIP_FUN) {}
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_SCIP_API_HPP