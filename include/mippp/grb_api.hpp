#ifndef MIPPP_grb_api_HPP
#define MIPPP_grb_api_HPP

#include "dylib.hpp"

#include "gurobi_c.h"

#define DECLARE_GRB_FUN(FN)                    \
    using GRB##FN##_fun_t = decltype(GRB##FN); \
    const GRB##FN##_fun_t * FN;

#define CONSTRUCT_GRB_FUN(FN) FN(lib.get_function<GRB##FN##_fun_t>("GRB" #FN))

namespace fhamonic {
namespace mippp {

class grb_api {
private:
    dylib lib;

public:
    DECLARE_GRB_FUN(geterrormsg)
    DECLARE_GRB_FUN(emptyenvinternal)
    DECLARE_GRB_FUN(startenv)
    DECLARE_GRB_FUN(freeenv)
    DECLARE_GRB_FUN(newmodel)
    DECLARE_GRB_FUN(freemodel)
    DECLARE_GRB_FUN(updatemodel)
    DECLARE_GRB_FUN(addvar)
    DECLARE_GRB_FUN(addvars)
    DECLARE_GRB_FUN(addconstr)
    DECLARE_GRB_FUN(addrangeconstr)
    DECLARE_GRB_FUN(getconstrs)
    DECLARE_GRB_FUN(optimize)
    DECLARE_GRB_FUN(setintparam)
    DECLARE_GRB_FUN(getintparam)
    DECLARE_GRB_FUN(setdblparam)
    DECLARE_GRB_FUN(getdblparam)
    DECLARE_GRB_FUN(setstrparam)
    DECLARE_GRB_FUN(getstrparam)
    DECLARE_GRB_FUN(setintattr)
    DECLARE_GRB_FUN(getintattr)
    DECLARE_GRB_FUN(setintattrelement)
    DECLARE_GRB_FUN(getintattrelement)
    DECLARE_GRB_FUN(setintattrarray)
    DECLARE_GRB_FUN(getintattrarray)
    DECLARE_GRB_FUN(setintattrlist)
    DECLARE_GRB_FUN(getintattrlist)
    DECLARE_GRB_FUN(setdblattr)
    DECLARE_GRB_FUN(getdblattr)
    DECLARE_GRB_FUN(setdblattrelement)
    DECLARE_GRB_FUN(getdblattrelement)
    DECLARE_GRB_FUN(setdblattrarray)
    DECLARE_GRB_FUN(getdblattrarray)
    DECLARE_GRB_FUN(setdblattrlist)
    DECLARE_GRB_FUN(getdblattrlist)
    DECLARE_GRB_FUN(setcharattrelement)
    DECLARE_GRB_FUN(getcharattrelement)
    DECLARE_GRB_FUN(setcharattrarray)
    DECLARE_GRB_FUN(getcharattrarray)
    DECLARE_GRB_FUN(setcharattrlist)
    DECLARE_GRB_FUN(getcharattrlist)
    DECLARE_GRB_FUN(setstrattr)
    DECLARE_GRB_FUN(getstrattr)
    DECLARE_GRB_FUN(setstrattrelement)
    DECLARE_GRB_FUN(getstrattrelement)
    DECLARE_GRB_FUN(setstrattrarray)
    DECLARE_GRB_FUN(getstrattrarray)
    DECLARE_GRB_FUN(setstrattrlist)
    DECLARE_GRB_FUN(getstrattrlist)

public:
    grb_api(const char * lib_name = "gurobi120", const char * lib_path = "")
        : lib(lib_path, lib_name)
        , CONSTRUCT_GRB_FUN(geterrormsg)
        , CONSTRUCT_GRB_FUN(emptyenvinternal)
        , CONSTRUCT_GRB_FUN(startenv)
        , CONSTRUCT_GRB_FUN(freeenv)
        , CONSTRUCT_GRB_FUN(newmodel)
        , CONSTRUCT_GRB_FUN(freemodel)
        , CONSTRUCT_GRB_FUN(updatemodel)
        , CONSTRUCT_GRB_FUN(addvar)
        , CONSTRUCT_GRB_FUN(addvars)
        , CONSTRUCT_GRB_FUN(addconstr)
        , CONSTRUCT_GRB_FUN(addrangeconstr)
        , CONSTRUCT_GRB_FUN(getconstrs)
        , CONSTRUCT_GRB_FUN(optimize)
        , CONSTRUCT_GRB_FUN(setintparam)
        , CONSTRUCT_GRB_FUN(getintparam)
        , CONSTRUCT_GRB_FUN(setdblparam)
        , CONSTRUCT_GRB_FUN(getdblparam)
        , CONSTRUCT_GRB_FUN(setstrparam)
        , CONSTRUCT_GRB_FUN(getstrparam)
        , CONSTRUCT_GRB_FUN(setintattr)
        , CONSTRUCT_GRB_FUN(getintattr)
        , CONSTRUCT_GRB_FUN(setintattrelement)
        , CONSTRUCT_GRB_FUN(getintattrelement)
        , CONSTRUCT_GRB_FUN(setintattrarray)
        , CONSTRUCT_GRB_FUN(getintattrarray)
        , CONSTRUCT_GRB_FUN(setintattrlist)
        , CONSTRUCT_GRB_FUN(getintattrlist)
        , CONSTRUCT_GRB_FUN(setdblattr)
        , CONSTRUCT_GRB_FUN(getdblattr)
        , CONSTRUCT_GRB_FUN(setdblattrelement)
        , CONSTRUCT_GRB_FUN(getdblattrelement)
        , CONSTRUCT_GRB_FUN(setdblattrarray)
        , CONSTRUCT_GRB_FUN(getdblattrarray)
        , CONSTRUCT_GRB_FUN(setdblattrlist)
        , CONSTRUCT_GRB_FUN(getdblattrlist)
        , CONSTRUCT_GRB_FUN(setcharattrelement)
        , CONSTRUCT_GRB_FUN(getcharattrelement)
        , CONSTRUCT_GRB_FUN(setcharattrarray)
        , CONSTRUCT_GRB_FUN(getcharattrarray)
        , CONSTRUCT_GRB_FUN(setcharattrlist)
        , CONSTRUCT_GRB_FUN(getcharattrlist)
        , CONSTRUCT_GRB_FUN(setstrattr)
        , CONSTRUCT_GRB_FUN(getstrattr)
        , CONSTRUCT_GRB_FUN(setstrattrelement)
        , CONSTRUCT_GRB_FUN(getstrattrelement)
        , CONSTRUCT_GRB_FUN(setstrattrarray)
        , CONSTRUCT_GRB_FUN(getstrattrarray)
        , CONSTRUCT_GRB_FUN(setstrattrlist)
        , CONSTRUCT_GRB_FUN(getstrattrlist) {}
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_grb_api_HPP