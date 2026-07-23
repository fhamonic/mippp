#pragma once

#include <limits>
#include <optional>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/highs/v1_10/highs_base.hpp"

namespace mippp {
namespace highs::v1_10 {

class highs_milp : public highs_base {
public:
    [[nodiscard]] explicit highs_milp(const highs_api & api)
        : highs_base(api) {}

    variable add_integer_variable(
        const variable_params params = default_variable_params) {
        return _add_variable(params, kHighsVarTypeInteger);
    }
    auto add_integer_variables(
        std::size_t count,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset =
            _add_variables(count, params, kHighsVarTypeInteger);
        return _make_variables_view(offset, count);
    }
    template <typename IL>
    auto add_integer_variables(
        std::size_t count, IL && id_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset =
            _add_variables(count, params, kHighsVarTypeInteger);
        return _make_indexed_variables_view(offset, count,
                                            std::forward<IL>(id_lambda));
    }
    variable add_binary_variable() {
        return add_integer_variable(
            {.obj_coef = 0, .lower_bound = 0.0, .upper_bound = 1.0});
    }
    auto add_binary_variables(std::size_t count) noexcept {
        return add_integer_variables(
            count, {.obj_coef = 0, .lower_bound = 0.0, .upper_bound = 1.0});
    }
    template <typename IL>
    auto add_binary_variables(std::size_t count, IL && id_lambda) noexcept {
        return add_integer_variables(
            count, std::forward<IL>(id_lambda),
            {.obj_coef = 0, .lower_bound = 0.0, .upper_bound = 1.0});
    }
    void set_continuous(variable v) noexcept {
        check(Highs->changeColIntegrality(model, v.id(),
                                          kHighsVarTypeContinuous));
    }
    void set_integer(variable v) noexcept {
        check(Highs->changeColIntegrality(model, v.id(), kHighsVarTypeInteger));
    }
    void set_binary(variable v) noexcept {
        _set_variable_bounds(v, 0.0, 1.0);
        check(Highs->changeColIntegrality(model, v.id(), kHighsVarTypeInteger));
    }
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// MIP start ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    template <typename ER>
    inline void _add_mip_start(ER && entries) {
        _reset_raw_cache();
        _register_raw_entries(entries);
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
            status::time_limit,
            status::iteration_limit,
            status::solution_limit,
            status::failed,
            status::interrupted>;

    status_variant _status;

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
            case kHighsModelStatusObjectiveTarget:
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
    const status_variant & solve_status() const { return _status; }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////////// Solve //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void solve() {
        if(num_variables() == 0u) {
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

}  // namespace highs::v1_10
}  // namespace mippp
