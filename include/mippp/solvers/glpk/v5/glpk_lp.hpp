#pragma once

#include <limits>
#include <optional>

#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/glpk/v5/glpk_base.hpp"

namespace mippp {
namespace glpk::v5 {

class glpk_lp : public glpk_base {
private:
    glp_smcp model_params;

public:
    [[nodiscard]] explicit glpk_lp(const glpk_api & api)
        : glpk_base(api), model_params() {
        model_params.msg_lev = GLP_MSG_ALL;
        model_params.meth = GLP_PRIMAL;
        model_params.pricing = GLP_PT_STD;
        model_params.r_test = GLP_RT_STD;
        model_params.tol_bnd = 1e-7;
        model_params.tol_dj = 1e-7;
        model_params.tol_piv = 1e-10;
        model_params.obj_ll = std::numeric_limits<double>::lowest();
        model_params.obj_ul = std::numeric_limits<double>::max();
        model_params.it_lim = std::numeric_limits<int>::max();
        model_params.tm_lim = std::numeric_limits<int>::max();
        model_params.presolve = 0;  // PRESOLVE
        model_params.excl = 0;
        model_params.shift = 0;
        model_params.aorn = GLP_USE_AT;
    }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////// Tolerance parameters ///////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void set_feasibility_tolerance(double tol) {
        model_params.tol_bnd = model_params.tol_dj = tol;
        model_params.tol_piv = tol / 100;
    }
    double get_feasibility_tolerance() { return model_params.tol_bnd; }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////// Solve status ///////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    // clang-format off
private:
    using status_variant = std::variant<
            status::unknown, // default value
            status::optimal,
            status::infeasible,
            status::unbounded>;

    status_variant _status;

public:
    const status_variant & solve_status() const { return _status; }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////////// Solve //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void solve() {
        switch(glp.simplex(model, &model_params)) {
            case GLP_ENOPFS:
                _status.emplace<status::unbounded>();
                return;
            case GLP_ENODFS:
                _status.emplace<status::infeasible>();
                return;
        }
        const int primal_status = glp.get_status(model);
        if(primal_status == GLP_UNBND || !std::isfinite(get_solution_value())) {
                _status.emplace<status::unbounded>();
            return;
        }
        if(primal_status == GLP_OPT) {
                _status.emplace<status::optimal>();
            return;
        }
        if(primal_status == GLP_INFEAS || primal_status == GLP_NOFEAS) {
                _status.emplace<status::infeasible>();
            return;
        }
    }
    double get_solution_value() {
        return objective_offset + glp.get_obj_val(model);
    }
    auto get_solution() {
        auto num_vars = num_variables();
        auto solution = std::make_unique_for_overwrite<double[]>(num_vars);
        for(std::size_t var = 0u; var < num_vars; ++var) {
            solution[var] = glp.get_col_prim(model, static_cast<int>(var) + 1);
        }
        return variable_mapping(std::move(solution));
    }
    auto get_dual_solution() {
        auto num_constrs = num_constraints();
        auto solution = std::make_unique_for_overwrite<double[]>(num_constrs);
        for(std::size_t constr = 0u; constr < num_constrs; ++constr) {
            solution[constr] =
                glp.get_row_dual(model, static_cast<int>(constr) + 1);
        }
        return constraint_mapping(std::move(solution));
    }
    auto get_reduced_costs() {
        auto num_vars = num_variables();
        auto reduced_costs = std::make_unique_for_overwrite<double[]>(num_vars);
        for(std::size_t var = 0u; var < num_vars; ++var) {
            reduced_costs[var] =
                glp.get_col_dual(model, static_cast<int>(var) + 1);
        }
        return variable_mapping(std::move(reduced_costs));
    }
};

}  // namespace glpk::v5
}  // namespace mippp
