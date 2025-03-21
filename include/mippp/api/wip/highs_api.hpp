#ifndef MIPPP_HIGHS_API_HPP
#define MIPPP_HIGHS_API_HPP

#include "dylib.hpp"

// #include "coin/HIGHS_C_Interface.h"
// #include "coin/CoinFinite.hpp"

#define HIGHS_FUNCTIONS(F)                              \
    F(HIGHS_newModel, newModel)

#define DECLARE_HIGHS_FUN(FULL, SHORT)      \
    using SHORT##_fun_t = decltype(FULL); \
    SHORT##_fun_t * SHORT;
#define CONSTRUCT_HIGHS_FUN(FULL, SHORT) \
    , SHORT(lib.get_function<SHORT##_fun_t>(#FULL))

namespace fhamonic {
namespace mippp {

class highs_api {
private:
    dylib lib;

public:
    HIGHS_FUNCTIONS(DECLARE_HIGHS_FUN)

public:
    inline highs_api(const char * lib_name = "HIGHS", const char * lib_path = "")
        : lib(lib_path, lib_name) HIGHS_FUNCTIONS(CONSTRUCT_HIGHS_FUN) {}
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_HIGHS_API_HPP