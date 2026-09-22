#pragma once

#include <memory>
#include <variant>

#include "mippp/infeasibility_certificate.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/mosek/impl/v1/mosek_base.hpp"

namespace mippp {
namespace mosek::impl::v1 {

class mosek_lp : public mosek_base {
public:
    [[nodiscard]] mosek_lp() : mosek_lp(mosek_api::load()) {}
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

    status_variant _status = status::unknown{};
    std::optional<MSKsoltypee> _solution;

    status_variant _get_status(MSKrescodee trm) {
        using namespace status;
        switch(trm) {
            case MSK_RES_OK: {
                if(!_solution) return unknown{};
                MSKprostae prosta;
                check(MSK->getprosta(task, *_solution, &prosta));
                switch(prosta) {
                    case MSK_PRO_STA_PRIM_AND_DUAL_FEAS: {
                        MSKsolstae solsta;
                        check(MSK->getsolsta(task, *_solution, &solsta));
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
    MSKsoltypee _require_solution() const {
        if(!_solution) throw solver_error("MOSEK has no continuous solution");
        return *_solution;
    }
    // clang-format on
public:
    const status_variant & get_status() const { return _status; }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////////// Solve //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void solve() {
        _status = status::unknown{};
        _solution.reset();
        MSKrescodee trm = MSK_RES_OK;
        // optimizetrm performs optimization; it is NOT a status accessor.
        check(MSK->optimizetrm(task, &trm));
        _solution = _pick_continuous_solution();
        _status = _get_status(trm);
    }
    double get_solution_value() {
        double val;
        check(MSK->getprimalobj(task, _require_solution(), &val));
        return val;
    }
    auto get_solution() {
        auto solution =
            std::make_unique_for_overwrite<double[]>(num_variables());
        check(MSK->getxx(task, _require_solution(), solution.get()));
        return variable_mapping(std::move(solution));
    }
    auto get_dual_solution() {
        auto dual_solution =
            std::make_unique_for_overwrite<double[]>(num_constraints());
        check(MSK->getsolution(task, _require_solution(), nullptr, nullptr,
                               nullptr, nullptr, nullptr, nullptr, nullptr,
                               dual_solution.get(), nullptr, nullptr, nullptr,
                               nullptr, nullptr));
        return constraint_mapping(std::move(dual_solution));
    }
    auto get_reduced_costs() {
        const auto num_vars = num_variables();
        auto reduced_costs = std::make_unique_for_overwrite<double[]>(num_vars);
        check(MSK->getreducedcosts(task, _require_solution(), 0,
                                   static_cast<int>(num_vars),
                                   reduced_costs.get()));
        return variable_mapping(std::move(reduced_costs));
    }

    std::optional<linear_infeasibility_certificate<double>>
    get_infeasibility_certificate() {
        if(!std::holds_alternative<status::infeasible>(_status))
            return std::nullopt;
        // Check certificate status independently for BOTH continuous solution
        // types. Problem infeasibility alone does not guarantee usable duals.
        for(auto type : {MSK_SOL_BAS, MSK_SOL_ITR}) {
            MSKbooleant defined = 0;
            check(MSK->solutiondef(task, type, &defined));
            if(!defined) continue;
            MSKprostae problem;
            MSKsolstae solution;
            check(MSK->getprosta(task, type, &problem));
            check(MSK->getsolsta(task, type, &solution));
            if(problem != MSK_PRO_STA_PRIM_INFEAS ||
               solution != MSK_SOL_STA_PRIM_INFEAS_CER)
                continue;
            linear_infeasibility_certificate<double> certificate;
            certificate.row_lower.resize(num_constraints());
            certificate.row_upper.resize(num_constraints());
            certificate.variable_lower.resize(num_variables());
            certificate.variable_upper.resize(num_variables());
            check(MSK->getsolution(
                task, type, nullptr, nullptr, nullptr, nullptr, nullptr,
                nullptr, nullptr, nullptr, certificate.row_lower.data(),
                certificate.row_upper.data(), certificate.variable_lower.data(),
                certificate.variable_upper.data(), nullptr));
            return certificate;
        }
        return std::nullopt;
    }
};

}  // namespace mosek::impl::v1
}  // namespace mippp
