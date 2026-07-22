#pragma once

#include <iostream>
#include <optional>

#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/mosek/v11/mosek_base.hpp"

namespace mippp {
namespace mosek::v11 {

class mosek_lp : public mosek_base {
private:
    MSKprostae lp_status;

public:
    [[nodiscard]] explicit mosek_lp(const mosek_api & api) : mosek_base(api) {}

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
            status::failed,
            status::numerical_failure,
            status::interrupted>;

    status_variant _status;

    status_variant _get_status() {
        using namespace status;
        MSKrestrmcode trm;
        check(MSK.optimizetrm(task, &trm));
        switch(trm) {
            case MSK_RES_OK: {
                MSKprostae prosta;
                check(MSK.getprosta(task, MSK_SOL_BAS, &prosta));
                switch(prosta) {
                    case MSK_PRO_STA_PRIM_AND_DUAL_FEAS: {
                        MSKsolstae solsta;
                        check(MSK.getsolsta(task, MSK_SOL_BAS, &solsta));
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
        check(MSK.getprosta(task, MSK_SOL_BAS, &lp_status));
        _status = _get_status();
    }
    double get_solution_value() {
        double val;
        check(MSK.getprimalobj(task, MSK_SOL_BAS, &val));
        return val;
    }
    auto get_solution() {
        auto solution =
            std::make_unique_for_overwrite<double[]>(num_variables());
        check(MSK.getxx(task, MSK_SOL_BAS, solution.get()));
        return variable_mapping(std::move(solution));
    }
    auto get_dual_solution() {
        auto dual_solution =
            std::make_unique_for_overwrite<double[]>(num_constraints());
        check(MSK.getsolution(task, MSK_SOL_BAS, nullptr, nullptr, nullptr,
                              nullptr, nullptr, nullptr, nullptr,
                              dual_solution.get(), nullptr, nullptr, nullptr,
                              nullptr, nullptr));
        return constraint_mapping(std::move(dual_solution));
    }
    auto get_reduced_costs() {
        const auto num_vars = num_variables();
        auto reduced_costs = std::make_unique_for_overwrite<double[]>(num_vars);
        check(MSK.getreducedcosts(task, MSK_SOL_BAS, 0,
                                  static_cast<int>(num_vars),
                                  reduced_costs.get()));
        return variable_mapping(std::move(reduced_costs));
    }
};

}  // namespace mosek::v11
}  // namespace mippp
