#pragma once

#include <cstddef>
#include <memory>
#include <ranges>
#include <utility>
#include <variant>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/highs/impl/v1/highs_base.hpp"

namespace mippp {
namespace highs::impl::v1 {

class highs_milp : public highs_base {
public:
    [[nodiscard]] highs_milp() : highs_milp(highs_api::load()) {}
    [[nodiscard]] explicit highs_milp(const highs_api & api)
        : highs_base(api) {}

    using model_base<int, double>::add_integer_variable;
    using model_base<int, double>::add_integer_variables;
    using model_base<int, double>::add_binary_variable;
    using model_base<int, double>::add_binary_variables;
    void set_continuous(variable v) {
        check(Highs->changeColIntegrality(model, _native_id(v),
                                          kHighsVarTypeContinuous));
    }
    void set_integer(variable v) {
        check(Highs->changeColIntegrality(model, _native_id(v),
                                          kHighsVarTypeInteger));
    }
    void set_binary(variable v) {
        _set_variable_bounds(v, 0.0, 1.0);
        check(Highs->changeColIntegrality(model, _native_id(v),
                                          kHighsVarTypeInteger));
    }
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// MIP start ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    template <typename ER>
    inline void _add_mip_start(ER && entries) {
        _reset_cache();
        _register_variables_entries<true>(entries);
        if(!Highs->setSparseSolution)
            throw solver_error("Highs_setSparseSolution not available.");
        check(Highs->setSparseSolution(model,
                                       static_cast<int>(tmp_indices.size()),
                                       tmp_indices.data(), tmp_scalars.data()));
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
    ////////////////////////// Tolerance parameters ///////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void set_optimality_tolerance(double tol) {
        check(Highs->setDoubleOptionValue(model, "mip_rel_gap", tol));
    }
    double get_optimality_tolerance() {
        double tol;
        check(Highs->getDoubleOptionValue(model, "mip_rel_gap", &tol));
        return tol;
    }
    // HiGHS has no integrality tolerance of its own: this one also bounds the
    // row and bound violations its MIP solver accepts.
    void set_integrality_tolerance(double tol) {
        check(Highs->setDoubleOptionValue(model, "mip_feasibility_tolerance",
                                          tol));
    }
    double get_integrality_tolerance() {
        double tol;
        check(Highs->getDoubleOptionValue(model, "mip_feasibility_tolerance",
                                          &tol));
        return tol;
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
            status::unbounded,
            status::limit_reached,
            status::time_limit,
            status::iteration_limit,
            status::solution_limit,
            status::failed,
            status::interrupted>;

    status_variant _status = status::unknown{};

    status_variant _get_status() {
        using namespace status;
        const int status_ = Highs->getModelStatus(model);
        switch (status_) {         
            case kHighsModelStatusOptimal:        return optimal{};
            case kHighsModelStatusUnboundedOrInfeasible: 
                                                  return infeasible_or_unbounded{};
            case kHighsModelStatusInfeasible:     return infeasible{};
            case kHighsModelStatusUnbounded:      return unbounded{};
        }
        int psolstatus;
        check(Highs->getIntInfoValue(model, "primal_solution_status", &psolstatus));
        const bool has_sol = (psolstatus == kHighsSolutionStatusFeasible);
        switch (status_) { 
            case kHighsModelStatusInterrupt:      return interrupted{has_sol};
            case kHighsModelStatusLoadError:
            case kHighsModelStatusModelError:
            case kHighsModelStatusPresolveError:
            case kHighsModelStatusSolveError:
            case kHighsModelStatusPostsolveError: return failed{has_sol};
            case kHighsModelStatusObjectiveBound:
            case kHighsModelStatusObjectiveTarget: return limit_reached{has_sol};
            case kHighsModelStatusTimeLimit:      return time_limit{has_sol};
            case kHighsModelStatusSolutionLimit:  return solution_limit{has_sol};
            case kHighsModelStatusModelEmpty:
            case kHighsModelStatusNotset:
            case kHighsModelStatusUnknown:        return unknown{has_sol};        
            default:
                return unknown{has_sol};
        }
    }
    // clang-format on
public:
    const status_variant & get_status() const { return _status; }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////////// Solve //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void solve() {
        if(num_variables() == 0u) {
            _status = status::unknown{};
            return;
        }
        check(Highs->run(model));
        _status = _get_status();
    }
    double get_solution_value() { return Highs->getObjectiveValue(model); }
    auto get_solution() {
        auto num_vars = num_variables();
        auto solution = std::make_unique_for_overwrite<double[]>(num_vars);
        check(Highs->getSolution(model, solution.get(), nullptr, nullptr,
                                 nullptr));
        return variable_mapping(
            [this, solution = std::move(solution)](const variable & v) {
                return *(solution.get() + _native_id(v));
            });
    }
};

}  // namespace highs::impl::v1
}  // namespace mippp
