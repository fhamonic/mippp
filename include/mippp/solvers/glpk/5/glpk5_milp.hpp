#ifndef MIPPP_GLPK_5_MILP_HPP
#define MIPPP_GLPK_5_MILP_HPP

#include <cmath>
#include <limits>
#include <optional>

#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/glpk/5/glpk5_base_model.hpp"

namespace fhamonic {
namespace mippp {

class glpk5_milp : public glpk5_base_model {
public:
    using variable_id = int;
    using constraint_id = int;
    using scalar = double;
    using variable = model_variable<variable_id, scalar>;
    using constraint = model_constraint<constraint_id, scalar>;

    [[nodiscard]] explicit glpk5_milp(const glpk5_api & api)
        : glpk5_base_model(api) {
        model_params.msg_lev = GLP_MSG_ALL;
        model_params.meth = GLP_PRIMAL;
        model_params.pricing = GLP_PT_STD;
        model_params.r_test = GLP_RT_STD;
        model_params.tol_bnd = 1e-13;
        model_params.tol_dj = 1e-13;
        model_params.tol_piv = 1e-13;
        model_params.obj_ll = std::numeric_limits<double>::lowest();
        model_params.obj_ul = std::numeric_limits<double>::max();
        model_params.it_lim = std::numeric_limits<int>::max();
        model_params.tm_lim = std::numeric_limits<int>::max();
        model_params.presolve = 0;  // PRESOLVE
        model_params.excl = 0;
        model_params.shift = 0;
        model_params.aorn = GLP_USE_AT;
    }

    //     variable add_variable(const variable_params params = {
    //                               .obj_coef = 0,
    //                               .lower_bound = 0,
    //                               .upper_bound = std::nullopt}) {
    //         int var_id = static_cast<int>(num_variables());
    //         glp.add_cols(model, 1);
    //         glp.set_obj_coef(model, var_id + 1, params.obj_coef);
    //         glp.set_col_bnds(
    //             model, var_id + 1, GLP_DB,
    //             params.lower_bound.value_or(-std::numeric_limits<double>::infinity()),
    //             params.upper_bound.value_or(std::numeric_limits<double>::infinity()));
    //         return variable(var_id);
    //     }

    // private:
    //     void _add_variables(std::size_t offset, std::size_t count,
    //                         const variable_params & params) {
    //         glp.add_cols(model, static_cast<int>(count));
    //         if(auto obj = params.obj_coef; obj != 0.0) {
    //             for(std::size_t i = offset + 1; i <= offset + count; ++i)
    //                 glp.set_obj_coef(model, static_cast<int>(i), obj);
    //         }
    //         if(auto lb = params.lower_bound.value_or(
    //                -std::numeric_limits<double>::infinity()),
    //            ub = params.upper_bound.value_or(
    //                std::numeric_limits<double>::infinity());
    //            lb != 0.0 || !std::isinf(ub)) {
    //             for(std::size_t i = offset + 1; i <= offset + count; ++i)
    //                 glp.set_col_bnds(model, static_cast<int>(i), GLP_DB, lb,
    //                 ub);
    //         }
    //     }

    // public:
    //     auto add_variables(std::size_t count,
    //                        variable_params params = {
    //                            .obj_coef = 0,
    //                            .lower_bound = 0,
    //                            .upper_bound = std::nullopt}) noexcept {
    //         const std::size_t offset = num_variables();
    //         _add_variables(offset, count, params);
    //         return make_variables_range(ranges::view::transform(
    //             ranges::view::iota(static_cast<variable_id>(offset),
    //                                static_cast<variable_id>(offset + count)),
    //             [](auto && i) { return variable{i}; }));
    //     }
    //     template <typename IL>
    //     auto add_variables(std::size_t count, IL && id_lambda,
    //                        variable_params params = {
    //                            .obj_coef = 0,
    //                            .lower_bound = 0,
    //                            .upper_bound = std::nullopt}) noexcept {
    //         const std::size_t offset = num_variables();
    //         _add_variables(offset, count, params);
    //         return make_indexed_variables_range(
    //             typename detail::function_traits<IL>::arg_types(),
    //             ranges::view::transform(
    //                 ranges::view::iota(static_cast<variable_id>(offset),
    //                                    static_cast<variable_id>(offset +
    //                                    count)),
    //                 [](auto && i) { return variable{i}; }),
    //             std::forward<IL>(id_lambda));
    //     }

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

#endif  // MIPPP_GLPK_5_MILP_HPP