#ifndef MIPPP_GUROBI_TRAITS_HPP
#define MIPPP_GUROBI_TRAITS_HPP

#include "scip/scip.h"

namespace fhamonic {
namespace mippp {

struct ScipTraits {
    enum opt_sense : int {
        min = SCIP_OBJSENSE_min,
        max = SCIP_OBJSENSE_max
    };
    enum var_category : char {
        continuous = SCIP_VARTYPE_continuous,
        integer = SCIP_VARTYPE_integer,
        binary = SCIP_VARTYPE_binary
    };
    using model_wrapper = void;

    static void build(opt_sense opt_sense, int nb_vars, double * obj,
                      double * col_lb, double * col_ub, var_category * vtype,
                      int nb_rows, int nb_elems, int * row_begins,
                      int * indices, double * coefs, double * row_lb,
                      double * row_ub) {
        return;
    }
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_GUROBI_TRAITS_HPP