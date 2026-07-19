#pragma once

#include <optional>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/highs/v1_10/highs_base.hpp"

namespace mippp {
namespace highs::v1_10 {

class highs_lp : public highs_base {
private:
    int lp_status;

public:
    [[nodiscard]] explicit highs_lp(const highs_api & api) : highs_base(api) {}

    void solve() {
        if(num_variables() == 0u) {
            lp_status = kHighsModelStatusModelEmpty;
            return;
        }
        check(Highs.run(model));
        lp_status = Highs.getModelStatus(model);
        check_model_status(lp_status);
    }

    // private:
    //     void _refine_lp_status() {
    //         char tmp_presolve[4];
    //         check(Highs.getHighsStringOptionValue(model, "presolve",
    //         tmp_presolve)); check(Highs.setHighsStringOptionValue(model,
    //         "presolve", "off")); check(Highs.run(model)); lp_status =
    //         Highs.getModelStatus(model);
    //         check(Highs.setHighsStringOptionValue(model, "presolve",
    //         tmp_presolve)); check_model_status(lp_status);
    //     }

    // public:
    bool proven_optimal() {
        return lp_status == kHighsModelStatusModelEmpty ||
               lp_status == kHighsModelStatusOptimal;
    }
    bool proven_infeasible() {
        // if(lp_status == kHighsModelStatusUnboundedOrInfeasible)
        //     _refine_lp_status();
        return lp_status == kHighsModelStatusInfeasible;
    }
    bool proven_unbounded() {
        // if(lp_status == kHighsModelStatusUnboundedOrInfeasible)
        //     _refine_lp_status();
        return lp_status == kHighsModelStatusUnbounded;
    }

    ///////////////////////////////// Limits //////////////////////////////////
    void set_iteration_limit(std::size_t n) {
        check(Highs.setIntOptionValue(model, "simplex_iteration_limit",
                                      static_cast<int>(n)));
    }
    std::size_t get_iteration_limit() {
        int n;
        check(Highs.getIntOptionValue(model, "simplex_iteration_limit", &n));
        return static_cast<std::size_t>(n);
    }

    /////////////////////////// Termination reason ////////////////////////////
    // clang-format off
    using termination_reasons = std::variant<
            reason::optimal,
            reason::infeasible_or_unbounded,
            reason::infeasible,
            reason::unbounded,
            reason::interrupted,
            reason::failed,
            reason::time_limit,
            reason::iteration_limit,
            reason::solution_limit,
            reason::unknown>;

    termination_reasons termination_reason() {
        using namespace reason;
        switch (lp_status) {            
            case kHighsModelStatusOptimal:        return optimal{};
            case kHighsModelStatusUnboundedOrInfeasible: 
                                                  return infeasible_or_unbounded{};
            case kHighsModelStatusInfeasible:     return infeasible{};
            case kHighsModelStatusUnbounded:      return unbounded{};
            case kHighsModelStatusInterrupt:      return interrupted{};
            case kHighsModelStatusLoadError:
            case kHighsModelStatusModelError:
            case kHighsModelStatusPresolveError:
            case kHighsModelStatusSolveError:
            case kHighsModelStatusPostsolveError: return failed{};
            case kHighsModelStatusObjectiveBound:
            case kHighsModelStatusObjectiveTarget:
            case kHighsModelStatusTimeLimit:      return time_limit{};
            case kHighsModelStatusIterationLimit: return iteration_limit{};
            case kHighsModelStatusModelEmpty:
            case kHighsModelStatusNotset:
            case kHighsModelStatusUnknown:        return unknown{};        
            default:
                return unknown{};
        }
    }
    // clang-format on

    //////////////////////////////// Solution /////////////////////////////////
    double get_solution_value() { return Highs.getObjectiveValue(model); }
    auto get_solution() {
        auto num_vars = num_variables();
        auto solution = std::make_unique_for_overwrite<double[]>(num_vars);
        check(Highs.getSolution(model, solution.get(), nullptr, nullptr,
                                nullptr));
        return variable_mapping(
            [this, solution = std::move(solution)](const variable & v) {
                return *(solution.get() + _native_id(v));
            });
    }
    auto get_dual_solution() {
        auto num_constrs = num_constraints();
        auto solution = std::make_unique_for_overwrite<double[]>(num_constrs);
        check(Highs.getSolution(model, nullptr, nullptr, nullptr,
                                solution.get()));
        return constraint_mapping(std::move(solution));
    }
    auto get_reduced_costs() {
        auto num_vars = num_variables();
        auto reduced_costs = std::make_unique_for_overwrite<double[]>(num_vars);
        check(Highs.getSolution(model, nullptr, reduced_costs.get(), nullptr,
                                nullptr));
        return variable_mapping([this, reduced_costs = std::move(
                                           reduced_costs)](const variable & v) {
            return *(reduced_costs.get() + _native_id(v));
        });
    }
};

}  // namespace highs::v1_10
}  // namespace mippp
