#ifndef MIPPP_GUROBI_TRAITS_HPP
#define MIPPP_GUROBI_TRAITS_HPP

#include <gurobi_c.h>

namespace fhamonic {
namespace mippp {

struct grb_solver_wrapper {
    GRBenv * env;
    GRBmodel * model;

    [[nodiscard]] grb_solver_wrapper() : env(nullptr), model(nullptr) {}
    [[nodiscard]] grb_solver_wrapper(GRBenv * e, GRBmodel * m)
        : env(e), model(m) {}
    ~grb_solver_wrapper() {
        if(model != nullptr) GRBfreemodel(model);
        if(env != nullptr) GRBfreeenv(env);
    }

    void set_loglevel(int loglevel) noexcept {
        GRBsetintparam(env, GRB_INT_PAR_LOGTOCONSOLE, loglevel);
    }
    void set_timeout(int timeout_s) noexcept {
        GRBsetdblparam(env, GRB_DBL_PAR_TIMELIMIT, timeout_s);
    }

    // GRBsetdblparam(env, GRB_DBL_PAR_MIPGAP, 1e-8);
    // GRBsetintparam(env, GRB_INT_PAR_THREADS, 8);

    int optimize() noexcept { return GRBoptimize(model); }

    [[nodiscard]] std::vector<double> get_solution() const noexcept {
        int nb_vars;
        GRBgetintattr(const_cast<GRBmodel *>(model), GRB_INT_ATTR_NUMVARS,
                      &nb_vars);
        std::vector<double> solution(static_cast<std::size_t>(nb_vars));
        GRBgetdblattrarray(const_cast<GRBmodel *>(model), GRB_DBL_ATTR_X, 0,
                           nb_vars, solution.data());
        return solution;
    }
};

struct grb_traits {
    enum opt_sense : int { min = GRB_MIN, max = GRB_MAX };
    enum var_category : char {
        continuous = GRB_CONTINUOUS,
        integer = GRB_INTEGER,
        binary = GRB_BINARY
    };
    using model_wrapper = grb_solver_wrapper;

    [[nodiscard]] static grb_solver_wrapper build(
        opt_sense opt_sense, int nb_vars, double * obj, double * col_lb,
        double * col_ub, var_category * vtype, int nb_rows, int nb_elems,
        int * row_begins, int * indices, double * coefs, double * row_lb,
        double * row_ub) {
        grb_solver_wrapper grb;
        GRBemptyenv(&grb.env);
        GRBstartenv(grb.env);
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