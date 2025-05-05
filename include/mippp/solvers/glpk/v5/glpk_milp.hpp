#ifndef MIPPP_GLPK_v5_MILP_HPP
#define MIPPP_GLPK_v5_MILP_HPP

#include <cmath>
#include <limits>
#include <optional>

#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/glpk/v5/glpk_base.hpp"

namespace fhamonic::mippp {
namespace glpk::v5 {

class glpk_milp : public glpk_base {
private:
    glp_iocp model_params;

public:
    [[nodiscard]] explicit glpk_milp(const glpk_api & api)
        : glpk_base(api), model_params() {
        model_params.msg_lev = GLP_MSG_ALL;
        model_params.br_tech = GLP_BR_PCH;
        model_params.bt_tech = GLP_BT_BLB;
        model_params.tol_int = 1e-6;
        model_params.tol_obj = 1e-7;
        model_params.tm_lim = std::numeric_limits<int>::max();
        model_params.pp_tech = GLP_PP_ROOT;
        model_params.mip_gap = 1e-4;
        model_params.mir_cuts = GLP_ON;
        model_params.gmi_cuts = GLP_ON;
        model_params.cov_cuts = GLP_ON;
        model_params.clq_cuts = GLP_ON;
        model_params.presolve = GLP_ON;
        model_params.binarize = GLP_OFF;
        model_params.fp_heur = GLP_ON;
        model_params.ps_heur = GLP_OFF;
        model_params.ps_tm_lim = 1000;
        model_params.sr_heur = GLP_ON;
    }

    variable add_integer_variable(
        const variable_params params = default_variable_params) {
        int var_id = static_cast<int>(num_variables());
        _add_variable(var_id, params, GLP_IV);
        return variable(var_id);
    }
    auto add_integer_variables(
        std::size_t count,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params, GLP_IV);
        return _make_variables_range(offset, count);
    }
    template <typename IL>
    auto add_integer_variables(
        std::size_t count, IL && id_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params, GLP_IV);
        return _make_indexed_variables_range(offset, count,
                                             std::forward<IL>(id_lambda));
    }

private:
    inline void _add_binary_variables(const std::size_t & offset,
                                      const std::size_t & count) {
        glp.add_cols(model, static_cast<int>(count));
        for(std::size_t i = offset + 1; i <= offset + count; ++i)
            glp.set_col_kind(model, static_cast<int>(i), GLP_BV);
    }

public:
    variable add_binary_variable() {
        int var_id = static_cast<int>(num_variables());
        _add_variable(var_id,
                      {.obj_coef = 0.0, .lower_bound = 0.0, .upper_bound = 1.0},
                      GLP_BV);
        return variable(var_id);
    }
    auto add_binary_variables(std::size_t count) noexcept {
        const std::size_t offset = num_variables();
        _add_binary_variables(offset, count);
        return _make_variables_range(offset, count);
    }
    template <typename IL>
    auto add_binary_variables(std::size_t count, IL && id_lambda) noexcept {
        const std::size_t offset = num_variables();
        _add_binary_variables(offset, count);
        return _make_indexed_variables_range(offset, count,
                                             std::forward<IL>(id_lambda));
    }

    void set_continuous(variable v) noexcept {
        glp.set_col_kind(model, v.id(), GLP_CV);
    }
    void set_integer(variable v) noexcept {
        glp.set_col_kind(model, v.id(), GLP_IV);
    }
    void set_binary(variable v) noexcept {
        glp.set_col_kind(model, v.id(), GLP_BV);
    }

    // add_sos1_constraint
    // add_sos2_constraint
    // add_indicator_constraint

    void set_feasibility_tolerance(double tol) {
        model_params.tol_int = tol;
        model_params.tol_obj = tol / 10;
    }
    double get_feasibility_tolerance() { return model_params.tol_int; }

    void solve() {
        glp.intopt(model, &model_params);
        // glp.intfeas1(model, 0, 0);
    }

    double get_solution_value() {
        return objective_offset + glp.mip_obj_val(model);
    }
    auto get_solution() {
        auto num_vars = num_variables();
        auto solution = std::make_unique_for_overwrite<double[]>(num_vars);
        for(std::size_t var = 0u; var < num_vars; ++var) {
            solution[var] = glp.mip_col_val(model, static_cast<int>(var) + 1);
        }
        return variable_mapping(std::move(solution));
    }
};

}  // namespace glpk::v5
}  // namespace fhamonic::mippp

#endif  // MIPPP_GLPK_v5_MILP_HPP