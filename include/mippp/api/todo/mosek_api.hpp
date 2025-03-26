#ifndef MIPPP_MOSEK_API_HPP
#define MIPPP_MOSEK_API_HPP

#include "dylib.hpp"

#include "mosek.h"

#define MOSEK_FUNCTIONS(F)              \
    F(MSK_makeenv, makeenv)             \
    F(MSK_makeemptytask, makeemptytask) \
    F(MSK_deletetask, deletetask)       \
    F(MSK_deleteenv, deleteenv)         \
    F(MSK_putobjsense, putobjsense)     \
    F(MSK_getobjsense, getobjsense)     \
    F(MSK_putclist, putclist)           \
    F(MSK_getc, _getc)                   \
    F(MSK_getclist, getclist)           \
    F(MSK_putcfix, putcfix)             \
    F(MSK_getcfix, getcfix)             \
    F(MSK_appendvars, appendvars)       \
    F(MSK_putcj, putcj)                 \
    F(MSK_chgvarbound, chgvarbound)     \
    F(MSK_putvarname, putvarname)       \
    F(MSK_getcj, getcj)                 \
    F(MSK_getvarbound, getvarbound)     \
    F(MSK_getvarnamelen, getvarnamelen) \
    F(MSK_getvarname, getvarname)       \
    F(MSK_appendcons, appendcons)       \
    F(MSK_putconbound, putconbound)     \
    F(MSK_chgconbound, chgconbound)     \
    F(MSK_putconname, putconname)       \
    F(MSK_getarow, getarow)             \
    F(MSK_getconbound, getconbound)     \
    F(MSK_getconnamelen, getconnamelen) \
    F(MSK_getconname, getconname)       \
    F(MSK_getacol, getacol)             \
    F(MSK_getaij, getaij)               \
    F(MSK_putdouparam, putdouparam)     \
    F(MSK_getdouparam, getdouparam)     \
    F(MSK_putintparam, putintparam)     \
    F(MSK_getintparam, getintparam)     \
    F(MSK_putlintparam, putlintparam)   \
    F(MSK_getlintparam, getlintparam)   \
    F(MSK_getnumvar, getnumvar)         \
    F(MSK_getnumcon, getnumcon)         \
    F(MSK_getnumanz, getnumanz)         \
    F(MSK_optimize, optimize)           \
    F(MSK_getprimalobj, getprimalobj)   \
    F(MSK_getsolution, getsolution)     \
    F(MSK_deletesolution, deletesolution)

#define DECLARE_MOSEK_FUN(FULL, SHORT)    \
    using SHORT##_fun_t = decltype(FULL); \
    SHORT##_fun_t * SHORT;
#define CONSTRUCT_MOSEK_FUN(FULL, SHORT) \
    , SHORT(lib.get_function<SHORT##_fun_t>(#FULL))

namespace fhamonic {
namespace mippp {

class mosek_api {
private:
    dylib lib;

public:
    MOSEK_FUNCTIONS(DECLARE_MOSEK_FUN)

public:
    inline mosek_api(const char * lib_name = "mosek64",
                     const char * lib_path = "")
        : lib(lib_path, lib_name) MOSEK_FUNCTIONS(CONSTRUCT_MOSEK_FUN) {}
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_MOSEK_API_HPP