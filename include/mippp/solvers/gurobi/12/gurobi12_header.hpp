#ifndef MIPPP_GUROBI_12_API_HPP
#define MIPPP_GUROBI_12_API_HPP

#include "dylib.hpp"

#include "gurobi_c.h"

#define GRB_FUNCTIONS(F)                               \
    F(GRBgeterrormsg, geterrormsg)                     \
    F(GRBemptyenvinternal, emptyenvinternal)           \
    F(GRBstartenv, startenv)                           \
    F(GRBgetenv, getenv)                               \
    F(GRBfreeenv, freeenv)                             \
    F(GRBnewmodel, newmodel)                           \
    F(GRBfreemodel, freemodel)                         \
    F(GRBupdatemodel, updatemodel)                     \
    F(GRBaddvar, addvar)                               \
    F(GRBaddvars, addvars)                             \
    F(GRBaddconstr, addconstr)                         \
    F(GRBaddconstrs, addconstrs)                       \
    F(GRBaddrangeconstr, addrangeconstr)               \
    F(GRBaddsos, addsos)                               \
    F(GRBaddgenconstrIndicator, addgenconstrIndicator) \
    F(GRBgetconstrs, getconstrs)                       \
    F(GRBoptimize, optimize)                           \
    F(GRBsetintparam, setintparam)                     \
    F(GRBgetintparam, getintparam)                     \
    F(GRBsetdblparam, setdblparam)                     \
    F(GRBgetdblparam, getdblparam)                     \
    F(GRBsetstrparam, setstrparam)                     \
    F(GRBgetstrparam, getstrparam)                     \
    F(GRBsetintattr, setintattr)                       \
    F(GRBgetintattr, getintattr)                       \
    F(GRBsetintattrelement, setintattrelement)         \
    F(GRBgetintattrelement, getintattrelement)         \
    F(GRBsetintattrarray, setintattrarray)             \
    F(GRBgetintattrarray, getintattrarray)             \
    F(GRBsetdblattr, setdblattr)                       \
    F(GRBgetdblattr, getdblattr)                       \
    F(GRBsetdblattrelement, setdblattrelement)         \
    F(GRBgetdblattrelement, getdblattrelement)         \
    F(GRBsetdblattrarray, setdblattrarray)             \
    F(GRBgetdblattrarray, getdblattrarray)             \
    F(GRBsetdblattrlist, setdblattrlist)               \
    F(GRBsetcharattrelement, setcharattrelement)       \
    F(GRBgetcharattrelement, getcharattrelement)       \
    F(GRBsetcharattrarray, setcharattrarray)           \
    F(GRBgetcharattrarray, getcharattrarray)           \
    F(GRBsetstrattr, setstrattr)                       \
    F(GRBgetstrattr, getstrattr)                       \
    F(GRBsetstrattrelement, setstrattrelement)         \
    F(GRBgetstrattrelement, getstrattrelement)         \
    F(GRBsetstrattrarray, setstrattrarray)             \
    F(GRBgetstrattrarray, getstrattrarray)

#define DECLARE_GUROBI_FUN(FULL, SHORT)   \
    using SHORT##_fun_t = decltype(FULL); \
    SHORT##_fun_t * SHORT;
#define CONSTRUCT_GUROBI_FUN(FULL, SHORT) \
    , SHORT(lib.get_function<SHORT##_fun_t>(#FULL))

namespace fhamonic {
namespace mippp {

class gurobi12_api {
private:
    dylib lib;

public:
    GRB_FUNCTIONS(DECLARE_GUROBI_FUN)

public:
    gurobi12_api(const char * lib_path = "",
                 const char * lib_name = "gurobi120")
        : lib(lib_path, lib_name) GRB_FUNCTIONS(CONSTRUCT_GUROBI_FUN) {}
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_GUROBI_12_API_HPP