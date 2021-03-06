#ifndef MIPPP_GUROBI_TRAITS_HPP
#define MIPPP_GUROBI_TRAITS_HPP

#include "scip/scip.h"

namespace fhamonic {
namespace mippp {

struct ScipTraits {
    enum OptSense : int {
        MINIMIZE = SCIP_OBJSENSE_MINIMIZE,
        MAXIMIZE = SCIP_OBJSENSE_MAXIMIZE
    };
    enum ColType : char {
        CONTINUOUS = SCIP_VARTYPE_CONTINUOUS,
        INTEGER = SCIP_VARTYPE_INTEGER,
        BINARY = SCIP_VARTYPE_BINARY
    };
    using ModelType = void;

    static void build(OptSense opt_sense, int nb_vars, double * obj,
                      double * col_lb, double * col_ub, ColType * vtype,
                      int nb_rows, int nb_elems, int * row_begins,
                      int * indices, double * coefs, double * row_lb,
                      double * row_ub) {
        return;
    }
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_GUROBI_TRAITS_HPP