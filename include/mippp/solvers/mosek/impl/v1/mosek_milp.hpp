#pragma once

#include <algorithm>
#include <cstddef>
#include <memory>
#include <optional>
#include <ranges>
#include <utility>
#include <variant>

#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/mosek/impl/v1/mosek_base.hpp"

namespace mippp {
namespace mosek::impl::v1 {

class mosek_milp : public mosek_base {
public:
    [[nodiscard]] mosek_milp() : mosek_milp(mosek_api::load()) {}
    [[nodiscard]] explicit mosek_milp(const mosek_api & api) : mosek_base(api) {
        check(MSK->putintparam(task, MSK_IPAR_OPTIMIZER,
                               MSK_OPTIMIZER_MIXED_INT));
    }

    using model_base<int, double>::add_integer_variable;
    using model_base<int, double>::add_integer_variables;
    using model_base<int, double>::add_binary_variable;
    using model_base<int, double>::add_binary_variables;

    void set_continuous(variable v) {
        check(MSK->putvartype(task, v.id(), MSK_VAR_TYPE_CONT));
    }
    void set_integer(variable v) {
        check(MSK->putvartype(task, v.id(), MSK_VAR_TYPE_INT));
    }
    void set_binary(variable v) {
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
        check(MSK->putxx(task, MSK_SOL_ITG, tmp_scalars.data()));
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

    status_variant _status = status::unknown{};
    std::optional<MSKsoltypee> _solution;

    std::optional<MSKsoltypee> _pick_solution() const {
        if(_slot_state(MSK_SOL_ITG)) return MSK_SOL_ITG;
        return _pick_continuous_solution();
    }
    MSKsoltypee _require_solution() const {
        if(!_solution) throw solver_error("MOSEK holds no solution");
        return *_solution;
    }

    // whether a stopped solve left the solution get_solution() reads
    bool _has_solution() {
        if(!_solution) return false;
        MSKsolstae solsta;
        check(MSK->getsolsta(task, *_solution, &solsta));
        return _has_primal_solution(solsta);
    }

    status_variant _get_status(MSKrescodee trm) {
        using namespace status;
        switch(trm) {
            case MSK_RES_OK: {
                // MOSEK 11's mixed-integer optimizer defines no slot for an
                // empty task, nor for a column-less one with a violated row
                if(!_solution) {
                    if(num_variables() == 0 && num_constraints() == 0)
                        return optimal{};
                    return unknown{};
                }
                const MSKsoltypee soltype = *_solution;
                MSKprostae prosta;
                check(MSK->getprosta(task, soltype, &prosta));
                switch(prosta) {
                    // an integer solution has no dual, so an optimal MIP
                    // reports PRIM_FEAS, not PRIM_AND_DUAL_FEAS
                    case MSK_PRO_STA_PRIM_FEAS:
                    case MSK_PRO_STA_PRIM_AND_DUAL_FEAS: {
                        MSKsolstae solsta;
                        check(MSK->getsolsta(task, soltype, &solsta));
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
                    case MSK_PRO_STA_DUAL_FEAS:
                    case MSK_PRO_STA_ILL_POSED:
                    case MSK_PRO_STA_UNKNOWN:
                    default: 
                        return unknown{};
                }
            }
            case MSK_RES_TRM_MAX_TIME:          return time_limit{_has_solution()};
            case MSK_RES_TRM_MAX_ITERATIONS:    return iteration_limit{_has_solution()};
            case MSK_RES_TRM_MIO_NUM_BRANCHES:
            case MSK_RES_TRM_MIO_NUM_RELAXS:    return node_limit{_has_solution()};
            case MSK_RES_TRM_NUM_MAX_NUM_INT_SOLUTIONS:
                                                return solution_limit{_has_solution()};
            case MSK_RES_TRM_OBJECTIVE_RANGE:   return limit_reached{_has_solution()};
            case MSK_RES_TRM_USER_CALLBACK:     return interrupted{_has_solution()};
            case MSK_RES_TRM_NUMERICAL_PROBLEM:
            case MSK_RES_TRM_MAX_NUM_SETBACKS:
            case MSK_RES_TRM_STALL:             return numerical_failure{_has_solution()};
            case MSK_RES_TRM_LOST_RACE:
            case MSK_RES_TRM_INTERNAL:
            case MSK_RES_TRM_INTERNAL_STOP:     return failed{_has_solution()};
            default: 
                return unknown{};
        }
    }
    // clang-format on
public:
    const status_variant & get_status() const { return _status; }
    void reset_status() noexcept {
        _status = status::unknown{};
        _solution.reset();
    }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////////// Solve //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void solve() {
        _status = status::unknown{};
        _solution.reset();
        // MSK_optimize reports a limit as an error code; MSK_optimizetrm
        // returns it as the termination code, and is the solve itself
        MSKrescodee trm = MSK_RES_OK;
        check(MSK->optimizetrm(task, &trm));
        _solution = _pick_solution();
        _status = _get_status(trm);
    }
    double get_solution_value() {
        double val = 0.0;
        if(num_variables() > 0)
            check(MSK->getprimalobj(task, _require_solution(), &val));
        return val;
    }
    auto get_solution() {
        const auto num_vars = num_variables();
        auto solution = std::make_unique_for_overwrite<double[]>(num_vars);
        if(num_vars > 0)
            check(MSK->getxx(task, _require_solution(), solution.get()));
        return variable_mapping(std::move(solution));
    }
};

}  // namespace mosek::impl::v1
}  // namespace mippp
