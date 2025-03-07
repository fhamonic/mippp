#ifndef MIPPP_LINKED_GUROBI_TRAITS_HPP
#define MIPPP_LINKED_GUROBI_TRAITS_HPP

#include <limits>

#include <gurobi_c.h>

#include "mippp/detail/infinity_helper.hpp"
#include "mippp/solvers/abstract_solver_wrapper.hpp"

namespace fhamonic {
namespace mippp {

struct linked_grb_model_traits {
    using variable_id_t = int;
    using constraint_id_t = std::size_t;
    using scalar_t = double;

    enum opt_sense : int { min = GRB_MINIMIZE, max = GRB_MAXIMIZE };
    enum var_category : char {
        continuous = GRB_CONTINUOUS,
        integer = GRB_INTEGER,
        binary = GRB_BINARY
    };

    static constexpr scalar_t minus_infinity =
        detail::minus_infinity_or_lowest<scalar_t>();
    static constexpr scalar_t infinity = detail::infinity_or_max<scalar_t>();
};

struct linked_grb_solver : public abstract_solver_wrapper {
    enum ret_code : int {
        success = GRB_BASIC,
        infeasible = GRB_INFEASIBLE,
        timeout = GRB_TIME_LIMIT
    };

    GRBenv * env;
    GRBmodel * model;

    [[nodiscard]] linked_grb_solver(const auto & m_model)
        : env(nullptr), model(nullptr) {
        using traits = typename std::decay_t<decltype(m_model)>::solver_traits;
        typename traits::opt_sense sense = m_model.optimization_sense();
        int num_vars = static_cast<int>(m_model.num_variables());
        double * obj = const_cast<double *>(m_model.column_coefs());
        double * col_lb = const_cast<double *>(m_model.column_lower_bounds());
        double * col_ub = const_cast<double *>(m_model.column_upper_bounds());
        typename traits::var_category * vtype =
            const_cast<typename traits::var_category *>(m_model.column_types());
        int num_rows = static_cast<int>(m_model.num_constraints());
        int num_elems = static_cast<int>(m_model.num_entries());
        int * row_begins = const_cast<int *>(m_model.row_begins());
        int * indices = const_cast<int *>(m_model.var_entries());
        double * coefs = const_cast<double *>(m_model.coef_entries());
        double * row_lb = const_cast<double *>(m_model.row_lower_bounds());
        double * row_ub = const_cast<double *>(m_model.row_upper_bounds());

        int error;

        error = GRBemptyenv(&this->env);
        if(error)
            std::cerr << "GRBemptyenv" << error << ": "
                      << GRBgeterrormsg(this->env) << std::endl;
        error = GRBstartenv(this->env);
        if(error)
            std::cerr << "GRBstartenv" << error << ": "
                      << GRBgeterrormsg(this->env) << std::endl;
        error =
            GRBnewmodel(this->env, &this->model, "PL", num_vars, obj, col_lb,
                        col_ub, reinterpret_cast<char *>(vtype), NULL);
        if(error)
            std::cerr << "GRBnewmodel" << error << ": "
                      << GRBgeterrormsg(this->env) << std::endl;
        error = GRBaddrangeconstrs(this->model, num_rows, num_elems, row_begins,
                                   indices, coefs, row_lb, row_ub, NULL);
        if(error)
            std::cerr << "GRBaddrangeconstrs" << error << ": "
                      << GRBgeterrormsg(this->env) << std::endl;

        // for(int row = 0; row < num_rows; ++row) {
        //     int offset = row_begins[row];
        //     int numnz =
        //         ((row + 1 < num_rows) ? row_begins[row + 1] : num_elems) -
        //         offset;
        //     error = GRBaddrangeconstr(this->model, numnz, indices +
        //     offset,
        //                               coefs + offset, col_lb[row],
        //                               col_ub[row], NULL);
        //     if(error) {
        //         std::cerr << row << " : ";
        //         std::cerr << "GRBaddrangeconstr:" << error << " "
        //                   << GRBgeterrormsg(this->env) << std::endl;
        //     }
        // }

        //
        // int _num_vars;
        // GRBgetintattr(this->model, GRB_INT_ATTR_NUMVARS, &_num_vars);
        // int _num_rows;
        // GRBgetintattr(this->model, GRB_INT_ATTR_NUMCONSTRS, &_num_rows);
        // std::cout << "given : vars: " << num_vars << " rows: " << num_rows
        //           << std::endl;
        // std::cout << "actual : vars: " << _num_vars << " rows: " <<
        // _num_rows
        //           << std::endl;
        //

        error = GRBsetintattr(this->model, GRB_INT_ATTR_MODELSENSE, sense);
        if(error) std::cerr << GRBgeterrormsg(this->env) << std::endl;
    }
    [[nodiscard]] linked_grb_solver(GRBenv * e, GRBmodel * m)
        : env(e), model(m) {}
    ~linked_grb_solver() {
        if(model != nullptr) GRBfreemodel(model);
        if(env != nullptr) GRBfreeenv(env);
    }

    void set_loglevel(int loglevel) noexcept {
        GRBsetintparam(env, GRB_INT_PAR_LOGTOCONSOLE, loglevel);
    }
    void set_timeout(int timeout_s) noexcept {
        GRBsetdblparam(env, GRB_DBL_PAR_TIMELIMIT, timeout_s);
    }
    void set_mip_optimality_gap(double precision) noexcept {
        GRBsetdblparam(env, GRB_DBL_PAR_MIPGAP, precision);
    }

    // GRBsetintparam(env, GRB_INT_PAR_THREADS, 8);

    int optimize() noexcept { return GRBoptimize(model); }

    [[nodiscard]] std::vector<double> get_solution() const noexcept {
        int num_vars;
        GRBgetintattr(const_cast<GRBmodel *>(model), GRB_INT_ATTR_NUMVARS,
                      &num_vars);

        std::cout << num_vars << std::endl;

        std::vector<double> solution(static_cast<std::size_t>(num_vars));
        GRBgetdblattrarray(const_cast<GRBmodel *>(model), GRB_DBL_ATTR_X, 0,
                           num_vars, solution.data());
        return solution;
    }
    [[nodiscard]] double get_objective_value() const noexcept {
        double obj;
        GRBgetdblattr(const_cast<GRBmodel *>(model), GRB_DBL_ATTR_OBJVAL, &obj);
        return obj;
    }
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_LINKED_GUROBI_TRAITS_HPP