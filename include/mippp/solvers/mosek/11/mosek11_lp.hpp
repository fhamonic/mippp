#ifndef MIPPP_MOSEK_11_LP_HPP
#define MIPPP_MOSEK_11_LP_HPP

#include <limits>
#include <optional>
#include <vector>

#include <range/v3/view/iota.hpp>
#include <range/v3/view/span.hpp>
#include <range/v3/view/zip.hpp>

#include "mippp/detail/function_traits.hpp"
#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/mosek/11/mosek11_api.hpp"

namespace fhamonic {
namespace mippp {

class mosek11_lp {
private:
    const mosek11_api & MSK;
    MSKenv_t env;
    MSKtask_t task;

    // std::optional<lp_status> opt_lp_status;
    std::vector<int> tmp_variables;
    std::vector<double> tmp_scalars;

    void check(MSKrescodee error) const {
        if(error == 0) return;
        throw std::runtime_error(MSK.error_message.at(error));
    }

    static constexpr MSKboundkeye constraint_relation_to_mosek_sense(
        constraint_relation rel) {
        if(rel == constraint_relation::less_equal_zero) return MSK_BK_UP;
        if(rel == constraint_relation::equal_zero) return MSK_BK_FX;
        return MSK_BK_LO;
    }
    static constexpr constraint_relation mosek_sense_to_constraint_relation(
        MSKboundkeye sense) {
        if(sense == MSK_BK_UP) return constraint_relation::less_equal_zero;
        if(sense == MSK_BK_FX) return constraint_relation::equal_zero;
        return constraint_relation::greater_equal_zero;
    }

public:
    using variable_id = int;
    using constraint_id = int;
    using scalar = double;
    using variable = model_variable<variable_id, scalar>;
    using constraint = model_constraint<constraint_id, scalar>;

    struct variable_params {
        scalar obj_coef = scalar{0};
        std::optional<scalar> lower_bound = 0.0;
        std::optional<scalar> upper_bound = std::nullopt;
    };

public:
    [[nodiscard]] explicit mosek11_lp(const mosek11_api & api)
        : MSK(api), env(NULL), task(NULL) {
        check(MSK.makeenv(&env, NULL));
        check(MSK.makeemptytask(env, &task));
    }
    ~mosek11_lp() {
        check(MSK.deletetask(&task));
        check(MSK.deleteenv(&env));
    }

    std::size_t num_variables() {
        MSKint32t num;
        check(MSK.getnumvar(task, &num));
        return static_cast<std::size_t>(num);
    }
    std::size_t num_constraints() {
        MSKint32t num;
        check(MSK.getnumcon(task, &num));
        return static_cast<std::size_t>(num);
    }
    std::size_t num_entries() {
        MSKint32t num;
        check(MSK.getnumanz(task, &num));
        return static_cast<std::size_t>(num);
    }

    void set_maximization() {
        check(MSK.putobjsense(task, MSK_OBJECTIVE_SENSE_MAXIMIZE));
    }
    void set_minimization() {
        check(MSK.putobjsense(task, MSK_OBJECTIVE_SENSE_MINIMIZE));
    }

    void set_objective_offset(double constant) {
        check(MSK.putcfix(task, constant));
    }

    void set_objective(linear_expression auto && le) {
        auto num_vars = num_variables();
        tmp_scalars.resize(num_vars);
        std::fill(tmp_scalars.begin(), tmp_scalars.end(), 0.0);
        for(auto && [var, coef] : le.linear_terms()) {
            tmp_scalars[var.uid()] += coef;
        }
        check(MSK.putcslice(task, 0, static_cast<int>(num_vars),
                            tmp_scalars.data()));
        set_objective_offset(le.constant());
    }

    double get_objective_offset() {
        double objective_offset;
        check(MSK.getcfix(task, &objective_offset));
        return objective_offset;
    }

    variable add_variable(const variable_params p = {
                              .obj_coef = 0,
                              .lower_bound = 0,
                              .upper_bound = std::nullopt}) {
        int var_id = static_cast<int>(num_variables());
        check(MSK.appendvars(task, 1));
        check(MSK.chgvarbound(task, var_id, 1, p.lower_bound.has_value(),
                              p.lower_bound.value_or(0)));
        check(MSK.chgvarbound(task, var_id, 0, p.upper_bound.has_value(),
                              p.upper_bound.value_or(0)));
        check(MSK.putcj(task, var_id, p.obj_coef));
        return variable(var_id);
    }

private:
    void _add_variables(std::size_t offset, std::size_t count,
                        const variable_params & params) {
        check(MSK.appendvars(task, static_cast<int>(count)));
        if(auto obj = params.obj_coef; obj != 0.0) {
            tmp_scalars.resize(count);
            std::fill(tmp_scalars.begin(), tmp_scalars.end(), obj);
            MSK.putcslice(task, static_cast<variable_id>(offset),
                          static_cast<variable_id>(offset + count),
                          tmp_scalars.data());
        }
        const auto lb = params.lower_bound.value_or(0);
        const auto ub = params.upper_bound.value_or(0);

        MSKboundkeye boundkey = MSK_BK_FR;
        if(params.lower_bound.has_value() && !params.upper_bound.has_value())
            boundkey = MSK_BK_LO;
        if(!params.lower_bound.has_value() && params.upper_bound.has_value())
            boundkey = MSK_BK_UP;
        if(params.lower_bound.has_value() && params.upper_bound.has_value()) {
            boundkey = (lb == ub) ? MSK_BK_FX : MSK_BK_RA;
        }
        MSK.putvarboundsliceconst(task, static_cast<variable_id>(offset),
                                  static_cast<variable_id>(offset + count),
                                  boundkey, lb, ub);
    }

public:
    auto add_variables(std::size_t count,
                       variable_params params = {
                           .obj_coef = 0,
                           .lower_bound = 0,
                           .upper_bound = std::nullopt}) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params);
        return make_variables_range(ranges::view::transform(
            ranges::view::iota(static_cast<variable_id>(offset),
                               static_cast<variable_id>(offset + count)),
            [](auto && i) { return variable{i}; }));
    }
    template <typename IL>
    auto add_variables(std::size_t count, IL && id_lambda,
                       variable_params params = {
                           .obj_coef = 0,
                           .lower_bound = 0,
                           .upper_bound = std::nullopt}) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params);
        return make_indexed_variables_range(
            typename detail::function_traits<IL>::arg_types(),
            ranges::view::transform(
                ranges::view::iota(static_cast<variable_id>(offset),
                                   static_cast<variable_id>(offset + count)),
                [](auto && i) { return variable{i}; }),
            std::forward<IL>(id_lambda));
    }

    constraint add_constraint(linear_constraint auto && lc) {
        auto constr_id = static_cast<int>(num_constraints());
        check(MSK.appendcons(task, 1));
        tmp_variables.resize(0);
        tmp_scalars.resize(0);
        for(auto && [var, coef] : lc.expression().linear_terms()) {
            tmp_variables.emplace_back(var.id());
            tmp_scalars.emplace_back(coef);
        }
        check(MSK.putarow(task, constr_id,
                          static_cast<int>(tmp_variables.size()),
                          tmp_variables.data(), tmp_scalars.data()));
        const double b = -lc.expression().constant();
        check(MSK.putconbound(task, constr_id,
                              constraint_relation_to_mosek_sense(lc.relation()),
                              b, b));
        return constraint(constr_id);
    }

    void optimize() {
        // if(num_variables() == 0u) {
        //     opt_lp_status.emplace(lp_status::optimal);
        //     return;
        // }
        check(MSK.optimize(task));
    }

    double get_solution_value() {
        double val;
        check(MSK.getprimalobj(task, MSK_SOL_BAS, &val));
        return val;
    }
    auto get_solution() {
        auto solution =
            std::make_unique_for_overwrite<double[]>(num_variables());
        check(MSK.getsolution(task, MSK_SOL_BAS, NULL, NULL, NULL, NULL, NULL,
                              NULL, solution.get(), NULL, NULL, NULL, NULL,
                              NULL, NULL));
        return variable_mapping(std::move(solution));
    }
    auto get_dual_solution() {
        auto solution =
            std::make_unique_for_overwrite<double[]>(num_constraints());
        check(MSK.getsolution(task, MSK_SOL_BAS, NULL, NULL, NULL, NULL, NULL,
                              NULL, NULL, solution.get(), NULL, NULL, NULL,
                              NULL, NULL));
        return constraint_mapping(std::move(solution));
    }
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_MOSEK_11_LP_HPP