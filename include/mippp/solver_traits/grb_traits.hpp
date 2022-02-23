#ifndef MIPPP_GUROBI_TRAITS_HPP
#define MIPPP_GUROBI_TRAITS_HPP

#include <gurobi_c.h>

namespace fhamonic {
namespace mippp {

struct GRBModelWrap {
    GRBenv * env;
    GRBmodel * model;
    GRBModelWrap() : env(nullptr), model(nullptr) {}
    GRBModelWrap(GRBenv * e, GRBmodel * m) : env(e), model(m) {}
    ~GRBModelWrap() {
        if(model != nullptr) GRBfreemodel(model);
        if(env != nullptr) GRBfreeenv(env);
    }
};

struct GrbTraits {
    enum OptSense : int { MINIMIZE = GRB_MINIMIZE, MAXIMIZE = GRB_MAXIMIZE };
    enum ColType : char {
        CONTINUOUS = GRB_CONTINUOUS,
        INTEGER = GRB_INTEGER,
        BINARY = GRB_BINARY
    };
    using ModelType = GRBModelWrap;

    static GRBModelWrap build(OptSense opt_sense, int nb_vars, double * obj,
                              double * col_lb, double * col_ub, ColType * vtype,
                              int nb_rows, int nb_elems, int * row_begins,
                              int * indices, double * coefs, double * row_lb,
                              double * row_ub) {
        GRBModelWrap grb;
        GRBemptyenv(&grb.env);
        GRBstartenv(grb.env);

        GRBsetintparam(grb.env, GRB_INT_PAR_LOGTOCONSOLE, 0);

        GRBnewmodel(grb.env, &grb.model, "PL", nb_vars, obj, col_lb, col_ub,
                    reinterpret_cast<char *>(vtype), NULL);
        GRBaddrangeconstrs(grb.model, nb_rows, nb_elems, row_begins, indices,
                           coefs, row_lb, row_ub, NULL);
        GRBsetintattr(grb.model, GRB_INT_ATTR_MODELSENSE, opt_sense);

        return grb;
    }
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_GUROBI_TRAITS_HPP