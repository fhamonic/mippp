#ifndef MIPPP_MOSEK_11_BASE_MODEL_HPP
#define MIPPP_MOSEK_11_BASE_MODEL_HPP

#include <filesystem>
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

class mosek11_base_model {
public:
    using variable_id = int;
    using constraint_id = int;
    using scalar = double;
    using variable = model_variable<variable_id, scalar>;
    using constraint = model_constraint<constraint_id, scalar>;

    struct variable_params {
        scalar obj_coef = scalar{0};
        std::optional<scalar> lower_bound = std::nullopt;
        std::optional<scalar> upper_bound = std::nullopt;
    };

    static constexpr variable_params default_variable_params = {
        .obj_coef = 0, .lower_bound = 0, .upper_bound = std::nullopt};

protected:
    const mosek11_api & MSK;
    MSKenv_t env;
    MSKtask_t task;

    // std::optional<lp_status> opt_lp_status;

    std::vector<std::pair<constraint_id, unsigned int>>
        tmp_constraint_entry_cache;
    std::vector<int> tmp_variables;
    std::vector<double> tmp_scalars;

    void check(MSKrescodee error) const {
        // if(error == 0) return;
        // throw std::runtime_error(MSK.error_message.at(error));

        if(error == 0) return;
        char str[MSK_MAX_STR_LEN];
        MSK.getcodedesc(error, NULL, str);
        throw std::runtime_error(str);
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
    [[nodiscard]] explicit mosek11_base_model(const mosek11_api & api)
        : MSK(api), env(NULL), task(NULL) {
        check(MSK.makeenv(
            &env,
            (std::filesystem::temp_directory_path() / "mosek11_").c_str()));
        check(MSK.makeemptytask(env, &task));
    }
    ~mosek11_base_model() {
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

protected:
    void _add_variable(const int & var_id, const variable_params & params) {
        check(MSK.appendvars(task, 1));
        MSKboundkeye boundkey = MSK_BK_FR;
        double lb = params.lower_bound.value_or(-MSK_INFINITY);
        double ub = params.upper_bound.value_or(+MSK_INFINITY);
        // check(MSK.putvarbound(task, var_id, MSK_BK_RA, lb, ub));
        if(params.lower_bound.has_value() && params.upper_bound.has_value()) {
            boundkey = (lb == ub) ? MSK_BK_FX : MSK_BK_RA;
        } else if(params.lower_bound.has_value()) {
            boundkey = MSK_BK_LO;
        } else if(params.upper_bound.has_value()) {
            boundkey = MSK_BK_UP;
        }
        check(MSK.putvarbound(task, var_id, boundkey, lb, ub));
        check(MSK.putcj(task, var_id, params.obj_coef));
    }
    void _add_variables(std::size_t offset, std::size_t count,
                        const variable_params & params) {
        check(MSK.appendvars(task, static_cast<int>(count)));
        if(auto obj = params.obj_coef; obj != 0.0) {
            tmp_scalars.resize(count);
            std::fill(tmp_scalars.begin(), tmp_scalars.end(), obj);
            check(MSK.putcslice(task, static_cast<variable_id>(offset),
                                static_cast<variable_id>(offset + count),
                                tmp_scalars.data()));
        }
        MSKboundkeye boundkey = MSK_BK_FR;
        double lb = params.lower_bound.value_or(-MSK_INFINITY);
        double ub = params.upper_bound.value_or(+MSK_INFINITY);
        if(params.lower_bound.has_value() && params.upper_bound.has_value()) {
            boundkey = (lb == ub) ? MSK_BK_FX : MSK_BK_RA;
        } else if(params.lower_bound.has_value()) {
            boundkey = MSK_BK_LO;
        } else if(params.upper_bound.has_value()) {
            boundkey = MSK_BK_UP;
        }
        check(MSK.putvarboundsliceconst(
            task, static_cast<variable_id>(offset),
            static_cast<variable_id>(offset + count), boundkey, lb, ub));
    }
    inline auto _make_variables_range(const std::size_t & offset,
                                      const std::size_t & count) {
        return make_variables_range(ranges::view::transform(
            ranges::view::iota(static_cast<variable_id>(offset),
                               static_cast<variable_id>(offset + count)),
            [](auto && i) { return variable{i}; }));
    }
    template <typename IL>
    inline auto _make_indexed_variables_range(const std::size_t & offset,
                                              const std::size_t & count,
                                              IL && id_lambda) {
        return make_indexed_variables_range(
            typename detail::function_traits<IL>::arg_types(),
            ranges::view::transform(
                ranges::view::iota(static_cast<variable_id>(offset),
                                   static_cast<variable_id>(offset + count)),
                [](auto && i) { return variable{i}; }),
            std::forward<IL>(id_lambda));
    }

public:
    variable add_variable(
        const variable_params params = default_variable_params) {
        int var_id = static_cast<int>(num_variables());
        _add_variable(var_id, params);
        return variable(var_id);
    }
    auto add_variables(
        std::size_t count,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params);
        return _make_variables_range(offset, count);
    }
    template <typename IL>
    auto add_variables(
        std::size_t count, IL && id_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params);
        return _make_indexed_variables_range(offset, count,
                                             std::forward<IL>(id_lambda));
    }

    void set_objective_coefficient(variable v, double c) {
        check(MSK.putcj(task, v.id(), c));
    }
    void set_variable_lower_bound(variable v, double lb) {
        check(MSK.chgvarbound(task, v.id(), 1, 1, lb));
    }
    void set_variable_upper_bound(variable v, double ub) {
        check(MSK.chgvarbound(task, v.id(), 0, 1, ub));
    }

    double get_objective_coefficient(variable v) {
        double coef;
        check(MSK.getcj(task, v.id(), &coef));
        return coef;
    }
    double get_variable_lower_bound(variable v) {
        MSKboundkeye boundkey;
        double lb, ub;
        check(MSK.getvarbound(task, v.id(), &boundkey, &lb, &ub));
        return lb;
    }
    double get_variable_upper_bound(variable v) {
        MSKboundkeye boundkey;
        double lb, ub;
        check(MSK.getvarbound(task, v.id(), &boundkey, &lb, &ub));
        return ub;
    }

    constraint add_constraint(linear_constraint auto && lc) {
        auto constr_id = static_cast<int>(num_constraints());
        check(MSK.appendcons(task, 1));
        tmp_constraint_entry_cache.resize(num_variables());
        tmp_variables.resize(0);
        tmp_scalars.resize(0);
        for(auto && [var, coef] : lc.expression().linear_terms()) {
            auto & p = tmp_constraint_entry_cache[var.uid()];
            if(p.first == constr_id + 1) {
                tmp_scalars[p.second] += coef;
                continue;
            }
            p = std::make_pair(constr_id + 1, tmp_variables.size());
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
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_MOSEK_11_BASE_MODEL_HPP