#ifndef MIPPP_SHARED_GUROBI_TRAITS_HPP
#define MIPPP_SHARED_GUROBI_TRAITS_HPP

#include <gurobi_c.h>

namespace fhamonic {
namespace mippp {

struct shared_grb_solver_wrapper {
    GRBenv * env;
    GRBmodel * model;

    [[nodiscard]] shared_grb_solver_wrapper() : env(nullptr), model(nullptr) {}
    [[nodiscard]] shared_grb_solver_wrapper(GRBenv * e, GRBmodel * m)
        : env(e), model(m) {}
    ~shared_grb_solver_wrapper() {
        if(model != nullptr) GRBfreemodel(model);
        if(env != nullptr) GRBfreeenv(env);
    }

    void set_loglevel(int loglevel) noexcept {
        GRBsetintparam(env, GRB_INT_PAR_LOGTOCONSOLE, loglevel);
    }
    void set_timeout(int timeout_s) noexcept {
        GRBsetdblparam(env, GRB_DBL_PAR_TIMELIMIT, timeout_s);
    }
    void set_mip_gap(double precision) noexcept {
        GRBsetdblparam(env, GRB_DBL_PAR_MIPGAP, precision);
    }

    // GRBsetdblparam(env, GRB_DBL_PAR_MIPGAP, 1e-8);
    // GRBsetintparam(env, GRB_INT_PAR_THREADS, 8);

    int optimize() noexcept { return GRBoptimize(model); }

    [[nodiscard]] std::vector<double> get_solution() const noexcept {
        int nb_vars;
        GRBgetintattr(const_cast<GRBmodel *>(model), GRB_INT_ATTR_NUMVARS,
                      &nb_vars);

        std::cout << nb_vars << std::endl;

        std::vector<double> solution(static_cast<std::size_t>(nb_vars));
        GRBgetdblattrarray(const_cast<GRBmodel *>(model), GRB_DBL_ATTR_X, 0,
                           nb_vars, solution.data());
        return solution;
    }
    [[nodiscard]] double get_objective_value() const noexcept {
        double obj;
        GRBgetdblattr(const_cast<GRBmodel *>(model), GRB_DBL_ATTR_OBJVAL, &obj);
        return obj;
    }
};

struct shared_grb_traits {
    enum opt_sense : int { min = GRB_MINIMIZE, max = GRB_MAXIMIZE };
    enum var_category : char {
        continuous = GRB_CONTINUOUS,
        integer = GRB_INTEGER,
        binary = GRB_BINARY
    };
    enum ret_code : int {
        success = GRB_BASIC,
        infeasible = GRB_INFEASIBLE,
        timeout = GRB_TIME_LIMIT
    };
    using solver_wrapper = shared_grb_solver_wrapper;

    [[nodiscard]] static shared_grb_solver_wrapper build(const auto & model) {
        opt_sense sense = model.optimization_sense();
        int nb_vars = static_cast<int>(model.nb_variables());
        double * obj = const_cast<double *>(model.column_coefs());
        double * col_lb = const_cast<double *>(model.column_lower_bounds());
        double * col_ub = const_cast<double *>(model.column_upper_bounds());
        var_category * vtype = const_cast<var_category *>(model.column_types());
        int nb_rows = static_cast<int>(model.nb_constraints());
        int nb_elems = static_cast<int>(model.nb_entries());
        int * row_begins = const_cast<int *>(model.row_begins());
        int * indices = const_cast<int *>(model.var_entries());
        double * coefs = const_cast<double *>(model.coef_entries());
        double * row_lb = const_cast<double *>(model.row_lower_bounds());
        double * row_ub = const_cast<double *>(model.row_upper_bounds());

        shared_grb_solver_wrapper grb;

        int error;

        error = GRBemptyenv(&grb.env);
        if(error)
            std::cerr << "GRBemptyenv" << error << ": "
                      << GRBgeterrormsg(grb.env) << std::endl;

        error = GRBstartenv(grb.env);
        if(error)
            std::cerr << "GRBstartenv" << error << ": "
                      << GRBgeterrormsg(grb.env) << std::endl;

        error =
            GRBnewmodel(grb.env, &grb.model, "PL", nb_vars, obj, col_lb, col_ub,
                        reinterpret_cast<char *>(vtype), NULL);
        if(error)
            std::cerr << "GRBnewmodel" << error << ": "
                      << GRBgeterrormsg(grb.env) << std::endl;

        error = GRBaddrangeconstrs(grb.model, nb_rows, nb_elems, row_begins,
                                   indices, coefs, row_lb, row_ub, NULL);
        if(error)
            std::cerr << "GRBaddrangeconstrs" << error << ": "
                      << GRBgeterrormsg(grb.env) << std::endl;

        // for(int row = 0; row < nb_rows; ++row) {
        //     int offset = row_begins[row];
        //     int numnz =
        //         ((row + 1 < nb_rows) ? row_begins[row + 1] : nb_elems) -
        //         offset;
        //     error = GRBaddrangeconstr(grb.model, numnz, indices + offset,
        //                               coefs + offset, col_lb[row],
        //                               col_ub[row], NULL);
        //     if(error) {
        //         std::cerr << row << " : ";
        //         std::cerr << "GRBaddrangeconstr:" << error << " "
        //                   << GRBgeterrormsg(grb.env) << std::endl;
        //     }
        // }

        //
        // int _nb_vars;
        // GRBgetintattr(grb.model, GRB_INT_ATTR_NUMVARS, &_nb_vars);
        // int _nb_rows;
        // GRBgetintattr(grb.model, GRB_INT_ATTR_NUMCONSTRS, &_nb_rows);
        // std::cout << "given : vars: " << nb_vars << " rows: " << nb_rows
        //           << std::endl;
        // std::cout << "actual : vars: " << _nb_vars << " rows: " << _nb_rows
        //           << std::endl;
        //

        error = GRBsetintattr(grb.model, GRB_INT_ATTR_MODELSENSE, sense);
        if(error) std::cerr << GRBgeterrormsg(grb.env) << std::endl;
        return grb;
    }
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_SHARED_GUROBI_TRAITS_HPP