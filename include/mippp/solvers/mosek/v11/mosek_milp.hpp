#pragma once

#include <optional>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/mosek/v11/mosek_base.hpp"

namespace mippp {
namespace mosek::v11 {

class mosek_milp : public mosek_base {
public:
    [[nodiscard]] explicit mosek_milp(const mosek_api & api) : mosek_base(api) {
        check(
            MSK.putintparam(task, MSK_IPAR_OPTIMIZER, MSK_OPTIMIZER_MIXED_INT));
    }

    variable add_integer_variable(
        const variable_params params = default_variable_params) {
        int var_id = static_cast<int>(num_variables());
        _add_variable(var_id, params, MSK_VAR_TYPE_INT);
        return variable(var_id);
    }
    auto add_integer_variables(std::size_t count, const variable_params params =
                                                      default_variable_params) {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params, MSK_VAR_TYPE_INT);
        return _make_variables_view(offset, count);
    }
    template <typename IL>
    auto add_integer_variables(
        std::size_t count, IL && id_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params, MSK_VAR_TYPE_INT);
        return _make_indexed_variables_view(offset, count,
                                            std::forward<IL>(id_lambda));
    }
    variable add_binary_variable() {
        int var_id = static_cast<int>(num_variables());
        _add_variable(var_id,
                      variable_params{.lower_bound = 0, .upper_bound = 1},
                      MSK_VAR_TYPE_INT);
        return variable(var_id);
    }
    auto add_binary_variables(std::size_t count) {
        const std::size_t offset = num_variables();
        _add_variables(offset, count,
                       variable_params{.lower_bound = 0, .upper_bound = 1},
                       MSK_VAR_TYPE_INT);
        return _make_variables_view(offset, count);
    }
    template <typename IL>
    auto add_binary_variables(std::size_t count, IL && id_lambda) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count,
                       variable_params{.lower_bound = 0, .upper_bound = 1},
                       MSK_VAR_TYPE_INT);
        return _make_indexed_variables_view(offset, count,
                                            std::forward<IL>(id_lambda));
    }

    void set_continuous(variable v) noexcept {
        check(MSK.putvartype(task, v.id(), MSK_VAR_TYPE_CONT));
    }
    void set_integer(variable v) noexcept {
        check(MSK.putvartype(task, v.id(), MSK_VAR_TYPE_INT));
    }
    void set_binary(variable v) noexcept {
        set_integer(v);
        set_variable_lower_bound(v, 0);
        set_variable_upper_bound(v, 1);
    }
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// MIP start ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    template <typename ER>
    inline void _add_mip_start(ER && entries) {
        tmp_scalars.resize(num_variables());
        std::fill(tmp_scalars.begin(), tmp_scalars.end(), 0.0);
        for(auto && [var, coef] : entries) {
            tmp_scalars[var.uid()] += coef;
        }
        check(MSK.putxx(task, MSK_SOL_ITG, tmp_scalars.data()));
    }

public:
    template <std::ranges::range ER>
    void add_mip_start(ER && entries) {
        _add_mip_start(entries);
    }
    void add_mip_start(
        std::initializer_list<std::pair<variable, scalar>> entries) {
        _add_mip_start(entries);
    }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////// Solve status ///////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    // clang-format off
private:
    using status_variant = std::variant<
            status::unknown,
            status::optimal,
            status::infeasible_or_unbounded,
            status::infeasible,
            status::primal_and_dual_infeasible,
            status::unbounded,
            status::limit_reached,
            status::time_limit,
            status::iteration_limit,
            status::node_limit, 
            status::solution_limit,
            status::failed,
            status::numerical_failure,
            status::interrupted>;

    status_variant _status;

    MSKsoltypee _pick_sol() {
        MSKbooleant def = 0;
        if (MSK.solutiondef(task, MSK_SOL_ITG, &def) == MSK_RES_OK && def) return MSK_SOL_ITG;
        if (MSK.solutiondef(task, MSK_SOL_BAS, &def) == MSK_RES_OK && def) return MSK_SOL_BAS;
        return MSK_SOL_ITR;
    }

    status_variant _get_status() {
        using namespace status;
        MSKrestrmcode trm;
        check(MSK.optimizetrm(task, &trm));
        switch(trm) {
            case MSK_RES_OK: {
                MSKsoltypee soltype = _pick_sol();
                MSKprostae prosta;
                check(MSK.getprosta(task, soltype, &prosta));
                switch(prosta) {
                    case MSK_PRO_STA_PRIM_AND_DUAL_FEAS: {
                        MSKsolstae solsta;
                        check(MSK.getsolsta(task, soltype, &solsta));
                        switch (solsta) {
                            case MSK_SOL_STA_OPTIMAL:
                            case MSK_SOL_STA_INTEGER_OPTIMAL:  return optimal{};
                            default: 
                                return unknown{};
                        }
                    }
                    case MSK_PRO_STA_PRIM_INFEAS_OR_UNBOUNDED:
                                                  return infeasible_or_unbounded{};
                    case MSK_PRO_STA_PRIM_INFEAS: return infeasible{};
                    case MSK_PRO_STA_PRIM_AND_DUAL_INFEAS:
                                                  return primal_and_dual_infeasible{};
                    case MSK_PRO_STA_DUAL_INFEAS: return unbounded{};
                    case MSK_PRO_STA_PRIM_FEAS:
                    case MSK_PRO_STA_DUAL_FEAS:
                    case MSK_PRO_STA_ILL_POSED:
                    case MSK_PRO_STA_UNKNOWN:
                    default: 
                        return unknown{};
                }
            }
            case MSK_RES_TRM_MAX_TIME:          return time_limit{};
            case MSK_RES_TRM_MAX_ITERATIONS:    return iteration_limit{};
            case MSK_RES_TRM_MIO_NUM_BRANCHES:
            case MSK_RES_TRM_MIO_NUM_RELAXS:    return node_limit{};
            case MSK_RES_TRM_NUM_MAX_NUM_INT_SOLUTIONS:    
                                                return solution_limit{};
            case MSK_RES_TRM_OBJECTIVE_RANGE:   return limit_reached{};
            case MSK_RES_TRM_USER_CALLBACK:     return interrupted{};
            case MSK_RES_TRM_NUMERICAL_PROBLEM:
            case MSK_RES_TRM_MAX_NUM_SETBACKS:
            case MSK_RES_TRM_STALL:             return numerical_failure{};
            case MSK_RES_TRM_LOST_RACE:
            case MSK_RES_TRM_INTERNAL:
            case MSK_RES_TRM_INTERNAL_STOP:     return failed{};
            default: 
                return unknown{};
        }
    }
    // clang-format on
public:
    const status_variant & solve_status() const { return _status; }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////////// Solve //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void solve() {
        check(MSK.optimize(task));
        _status = _get_status();
    }
    double get_solution_value() {
        double val = 0.0;
        if(num_variables() > 0)
            check(MSK.getprimalobj(task, MSK_SOL_ITG, &val));
        return val;
    }
    auto get_solution() {
        const auto num_vars = num_variables();
        auto solution = std::make_unique_for_overwrite<double[]>(num_vars);
        if(num_vars > 0) check(MSK.getxx(task, MSK_SOL_ITG, solution.get()));
        return variable_mapping(std::move(solution));
    }
};

}  // namespace mosek::v11
}  // namespace mippp
