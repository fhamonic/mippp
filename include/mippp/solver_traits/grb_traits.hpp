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

    int optimize() noexcept { return GRBoptimize(model); }

    std::vector<double> get_solution() noexcept {
        int nb_vars;
        GRBgetintattr(model, GRB_INT_ATTR_NUMVARS, &nb_vars);
        std::vector<double> solution(static_cast<std::size_t>(nb_vars));
        GRBgetdblattrarray(model, GRB_DBL_ATTR_X, 0, nb_vars, solution.data());
        return solution;
    }
};

struct grb_traits {
    enum opt_sense : int { min = GRB_min, max = GRB_max };
    enum var_category : char {
        continuous = GRB_continuous,
        integer = GRB_integer,
        binary = GRB_binary
    };
    using model_wrapper = GRBModelWrap;

    static GRBModelWrap build(opt_sense opt_sense, int nb_vars, double * obj,
                              double * col_lb, double * col_ub, var_category * vtype,
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