#ifndef MIPPP_GLPK_LP_MODEL_HPP
#define MIPPP_GLPK_LP_MODEL_HPP

#include <cmath>
#include <limits>
// #include <ranges>
#include <optional>
#include <vector>

#include <range/v3/view/iota.hpp>
#include <range/v3/view/span.hpp>
#include <range/v3/view/zip.hpp>

#include "mippp/detail/function_traits.hpp"
#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_variable.hpp"

#include "mippp/api/glpk_api.hpp"

namespace fhamonic {
namespace mippp {

class glpk_lp_model {
private:
    const glpk_api & glp;
    glp_prob * model;
    glp_smcp params;
    std::optional<lp_status> opt_lp_status;
    double ojective_offset;

    static constexpr int constraint_relation_to_glp_row_type(
        constraint_relation rel) {
        if(rel == constraint_relation::less_equal_zero) return GLP_UP;
        if(rel == constraint_relation::equal_zero) return GLP_LO;
        return GLP_FX;
    }
    static constexpr constraint_relation glp_row_type_to_constraint_relation(
        int type) {
        if(type == GLP_UP) return constraint_relation::less_equal_zero;
        if(type == GLP_FX) return constraint_relation::equal_zero;
        if(type == GLP_LO) return constraint_relation::greater_equal_zero;
        throw std::runtime_error("glpk_lp_model: Cannot convert row type '" +
                                 std::to_string(type) +
                                 "' to constraint_relation.");
    }

    std::vector<int> tmp_variables;
    std::vector<double> tmp_scalars;

public:
    using variable_id = int;
    using scalar = double;
    using variable = model_variable<variable_id, scalar>;
    using constraint = int;

    struct variable_params {
        scalar obj_coef = scalar{0};
        scalar lower_bound = scalar{0};
        scalar upper_bound = std::numeric_limits<scalar>::infinity();
    };

public:
    [[nodiscard]] explicit glpk_lp_model(const glpk_api & api)
        : glp(api), model(glp.create_prob()), params(), ojective_offset(0.0) {
        params.msg_lev = GLP_MSG_ALL;
        params.meth = GLP_PRIMAL;
        params.pricing = GLP_PT_STD;
        params.r_test = GLP_RT_STD;
        params.tol_bnd = 1e-13;
        params.tol_dj = 1e-13;
        params.tol_piv = 1e-13;
        params.obj_ll = std::numeric_limits<double>::lowest();
        params.obj_ul = std::numeric_limits<double>::max();
        params.it_lim = std::numeric_limits<int>::max();
        params.tm_lim = std::numeric_limits<int>::max();
        params.presolve = 0;  // PRESOLVE
        params.excl = 0;
        params.shift = 0;
        params.aorn = GLP_USE_AT;
    }
    ~glpk_lp_model() { glp.delete_prob(model); }

    std::size_t num_variables() {
        return static_cast<std::size_t>(glp.get_num_cols(model));
    }
    std::size_t num_constraints() {
        return static_cast<std::size_t>(glp.get_num_rows(model));
    }
    std::size_t num_entries() {
        return static_cast<std::size_t>(glp.get_num_nz(model));
    }

    void set_maximization() { glp.set_obj_dir(model, GLP_MAX); }
    void set_minimization() { glp.set_obj_dir(model, GLP_MIN); }

    void set_objective_offset(double constant) { ojective_offset = constant; }
    void set_objective(linear_expression auto && le) {
        auto num_vars = static_cast<int>(num_variables());
        for(int var = 0; var < num_vars; ++var) {
            glp.set_obj_coef(model, var + 1, 0.0);
        }
        for(auto && [var, coef] : le.linear_terms()) {
            glp.set_obj_coef(model, var + 1, coef);
        }
        set_objective_offset(le.constant());
    }
    void add_objective(linear_expression auto && le) {
        for(auto && [var, coef] : le.linear_terms()) {
            glp.set_obj_coef(model, var + 1,
                             glp.get_obj_coef(model, var) + coef);
        }
        set_objective_offset(get_objective_offset() + le.constant());
    }
    double get_objective_offset() { return ojective_offset; }

    variable add_variable(
        const variable_params p = {
            .obj_coef = 0,
            .lower_bound = 0,
            .upper_bound = std::numeric_limits<double>::infinity()}) {
        int var_id = static_cast<int>(num_variables());
        glp.add_cols(model, 1);
        glp.set_obj_coef(model, var_id + 1, p.obj_coef);
        glp.set_col_bnds(model, var_id + 1, GLP_DB, p.lower_bound,
                         p.upper_bound);
        return variable(var_id);
    }
    // variables_range add_variables(std::size_t count,
    //                               const variable_params p);

    constraint add_constraint(linear_constraint auto && lc) {
        auto constr_id = num_constraints();
        glp.add_rows(model, 1);
        tmp_variables.resize(1);
        tmp_scalars.resize(1);
        for(auto && [var, coef] : lc.expression().linear_terms()) {
            tmp_variables.emplace_back(var + 1);
            tmp_scalars.emplace_back(coef);
        }
        glp.set_mat_row(model, static_cast<int>(constr_id) + 1,
                        static_cast<int>(tmp_variables.size()) - 1,
                        tmp_variables.data(), tmp_scalars.data());
        const double b = -lc.expression().constant();
        glp.set_row_bnds(model, static_cast<int>(constr_id) + 1,
                         constraint_relation_to_glp_row_type(lc.relation()), b,
                         b);
        return static_cast<int>(constr_id);
    }

    void optimize() {
        switch(glp.simplex(model, &params)) {
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
        return ojective_offset + glp.get_obj_val(model);
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
        return variable_mapping(std::move(solution));
    }
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_GLPK_LP_MODEL_HPP