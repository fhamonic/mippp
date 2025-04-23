#ifndef MIPPP_GLPK_5_LP_HPP
#define MIPPP_GLPK_5_LP_HPP

#include <limits>
#include <optional>

#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/glpk/5/glpk5_base_model.hpp"

namespace fhamonic {
namespace mippp {

class glpk5_lp : public glpk5_base_model {
public:
    using variable_id = int;
    using constraint_id = int;
    using scalar = double;
    using variable = model_variable<variable_id, scalar>;
    using constraint = model_constraint<constraint_id, scalar>;

private:
    glp_smcp model_params;

public:
    [[nodiscard]] explicit glpk5_lp(const glpk5_api & api)
        : glpk5_base_model(api), model_params() {
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

    void set_feasibility_tolerance(double tol) {
        model_params.tol_bnd = model_params.tol_dj = tol;
        model_params.tol_piv = tol / 100;
    }
    double get_feasibility_tolerance() { return model_params.tol_bnd; }

    void solve() {
        switch(glp.simplex(model, &model_params)) {
            case GLP_ENOPFS:
                opt_lp_status.emplace(lp_status::unbounded);
                return;
            case GLP_ENODFS:
                opt_lp_status.emplace(lp_status::infeasible);
                return;
        }
        const int primal_status = glp.get_status(model);
        if(primal_status == GLP_UNBND || !std::isfinite(get_solution_value())) {
            opt_lp_status.emplace(lp_status::unbounded);
            return;
        }
        if(primal_status == GLP_OPT) {
            opt_lp_status.emplace(lp_status::optimal);
            return;
        }
        if(primal_status == GLP_INFEAS || primal_status == GLP_NOFEAS) {
            opt_lp_status.emplace(lp_status::infeasible);
            return;
        }
    }
    std::optional<lp_status> get_lp_status() { return opt_lp_status; }

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
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_GLPK_5_LP_HPP