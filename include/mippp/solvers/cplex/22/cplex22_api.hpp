#ifndef MIPPP_CPLEX_22_API_HPP
#define MIPPP_CPLEX_22_API_HPP

#include "dylib.hpp"

#include "ilcplex/cplex.h"

#define CPLEX_FUNCTIONS(F)                     \
    F(CPXopenCPLEX, openCPLEX)                 \
    F(CPXcreateprob, createprob)               \
    F(CPXgetprobtype, getprobtype)             \
    F(CPXchgprobtype, chgprobtype)             \
    F(CPXfreeprob, freeprob)                   \
    F(CPXgeterrorstring, geterrorstring)       \
    F(CPXchgobjsen, chgobjsen)                 \
    F(CPXgetobjsen, getobjsen)                 \
    F(CPXchgobjoffset, chgobjoffset)           \
    F(CPXgetobjoffset, getobjoffset)           \
    F(CPXnewcols, newcols)                     \
    F(CPXaddcols, addcols)                     \
    F(CPXchgobj, chgobj)                       \
    F(CPXchgbds, chgbds)                       \
    F(CPXchgctype, chgctype)                   \
    F(CPXchgcolname, chgcolname)               \
    F(CPXgetobj, getobj)                       \
    F(CPXgetlb, getlb)                         \
    F(CPXgetub, getub)                         \
    F(CPXgetcolname, getcolname)               \
    F(CPXaddrows, addrows)                     \
    F(CPXchgsense, chgsense)                   \
    F(CPXchgrhs, chgrhs)                       \
    F(CPXchgrowname, chgrowname)               \
    F(CPXgetrows, getrows)                     \
    F(CPXgetsense, getsense)                   \
    F(CPXgetrhs, getrhs)                       \
    F(CPXgetrowname, getrowname)               \
    F(CPXchgcoef, chgcoef)                     \
    F(CPXchgcoeflist, chgcoeflist)             \
    F(CPXgetcoef, getcoef)                     \
    F(CPXdelcols, delcols)                     \
    F(CPXdelrows, delrows)                     \
    F(CPXgetnumcols, getnumcols)               \
    F(CPXgetnumrows, getnumrows)               \
    F(CPXgetnumnz, getnumnz)                   \
    F(CPXsetdblparam, setdblparam)             \
    F(CPXgetdblparam, getdblparam)             \
    F(CPXsetintparam, setintparam)             \
    F(CPXgetintparam, getintparam)             \
    F(CPXsetlongparam, setlongparam)           \
    F(CPXgetlongparam, getlongparam)           \
    F(CPXaddindconstraints, addindconstraints) \
    F(CPXaddmipstarts, addmipstarts)           \
    F(CPXaddsos, addsos)                       \
    F(CPXprimopt, primopt)                     \
    F(CPXdualopt, dualopt)                     \
    F(CPXlpopt, lpopt)                         \
    F(CPXfeasopt, feasopt)                     \
    F(CPXmipopt, mipopt)                     \
    F(CPXbendersopt, bendersopt)               \
    F(CPXcheckpfeas, checkpfeas)               \
    F(CPXcheckdfeas, checkdfeas)               \
    F(CPXgetobjval, getobjval)                 \
    F(CPXgetbestobjval, getbestobjval)         \
    F(CPXgetx, getx)                           \
    F(CPXsolution, solution)

#define DECLARE_CPLEX_FUN(FULL, SHORT)    \
    using SHORT##_fun_t = decltype(FULL); \
    SHORT##_fun_t * SHORT;
#define CONSTRUCT_CPLEX_FUN(FULL, SHORT) \
    , SHORT(lib.get_function<SHORT##_fun_t>(#FULL))

namespace fhamonic {
namespace mippp {

class cplex22_api {
private:
    dylib lib;

public:
    CPLEX_FUNCTIONS(DECLARE_CPLEX_FUN)

public:
    inline cplex22_api(const char * lib_path = "",
                       const char * lib_name = "cplex2212")
        : lib(lib_path, lib_name) CPLEX_FUNCTIONS(CONSTRUCT_CPLEX_FUN) {}
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_CPLEX_22_API_HPP