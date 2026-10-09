#pragma once

#include <algorithm>
#include <concepts>
#include <cstddef>
#include <limits>
#include <memory>
#include <utility>
#include <variant>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"
#include "mippp/utility/basis_snapshot.hpp"
#include "mippp/utility/solver_exceptions.hpp"

#include "mippp/solvers/highs/impl/v1/highs_base.hpp"

namespace mippp {
namespace highs::impl::v1 {

class highs_lp : public highs_base {
public:
    [[nodiscard]] highs_lp() : highs_lp(highs_api::load()) {}
    [[nodiscard]] explicit highs_lp(const highs_api & api) : highs_base(api) {}

    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////// Limits //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void set_iteration_limit(std::size_t n) {
        check(
            Highs->setIntOptionValue(model, "simplex_iteration_limit",
                                     static_cast<int>(std::min<std::size_t>(
                                         n, std::numeric_limits<int>::max()))));
    }
    std::size_t get_iteration_limit() {
        int n;
        check(Highs->getIntOptionValue(model, "simplex_iteration_limit", &n));
        return static_cast<std::size_t>(n);
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
        // HiGHS leaves it unwritten whenever its info is invalid, and returns
        // a warning that check() lets through
        int psolstatus = kHighsSolutionStatusNone;
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
            case kHighsModelStatusIterationLimit: return iteration_limit{has_sol};
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
    void reset_status() noexcept { _status = status::unknown{}; }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////////// Solve //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void solve() {
        reset_status();  // a solve that throws reports unknown
        if(num_variables() == 0u) return;
        _run();
        _status = _get_status();
    }
    // The routine re-solves and overwrites the model status: the reported
    // status is reset before the first native call, throw or return.
    auto compute_iis() {
        reset_status();
        return _compute_iis();
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
    auto get_dual_solution() {
        auto num_constrs = num_constraints();
        auto solution = std::make_unique_for_overwrite<double[]>(num_constrs);
        check(Highs->getSolution(model, nullptr, nullptr, nullptr,
                                 solution.get()));
        return constraint_mapping(std::move(solution));
    }
    auto get_reduced_costs() {
        auto num_vars = num_variables();
        auto reduced_costs = std::make_unique_for_overwrite<double[]>(num_vars);
        check(Highs->getSolution(model, nullptr, reduced_costs.get(), nullptr,
                                 nullptr));
        return variable_mapping([this, reduced_costs = std::move(
                                           reduced_costs)](const variable & v) {
            return *(reduced_costs.get() + _native_id(v));
        });
    }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////////// Basis //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    // HiGHS names no fixed entity: a fixed one reads the side of its dual
    using basis_variable_status =
        std::variant<basis_status::nonbasic, basis_status::basic,
                     basis_status::nonbasic_at_lower_bound,
                     basis_status::nonbasic_at_upper_bound,
                     basis_status::nonbasic_free>;
    using basis_constraint_status =
        std::variant<basis_status::basic, basis_status::nonbasic,
                     basis_status::nonbasic_at_lower_bound,
                     basis_status::nonbasic_at_upper_bound,
                     basis_status::nonbasic_free>;
    using basis_type =
        basis_snapshot<variable, constraint, basis_variable_status,
                       basis_constraint_status>;

    template <typename Status>
    static Status _basis_status(HighsInt code) noexcept {
        switch(code) {
            case kHighsBasisStatusBasic:
                return Status(std::in_place_type<basis_status::basic>);
            case kHighsBasisStatusLower:
                return Status(
                    std::in_place_type<basis_status::nonbasic_at_lower_bound>);
            case kHighsBasisStatusUpper:
                return Status(
                    std::in_place_type<basis_status::nonbasic_at_upper_bound>);
            case kHighsBasisStatusZero:
                return Status(std::in_place_type<basis_status::nonbasic_free>);
            default:
                return Status(std::in_place_type<basis_status::nonbasic>);
        }
    }
    // A tag this variant lacks, as nonbasic_fixed or superbasic, arrives as
    // bare nonbasic, whose Nonbasic code lets HiGHS pick the side.
    template <typename Status>
    static HighsInt _basis_code(const Status & status) noexcept {
        return std::visit(
            []<typename Tag>(Tag) -> HighsInt {
                using namespace basis_status;
                if constexpr(std::same_as<Tag, basic>)
                    return kHighsBasisStatusBasic;
                else if constexpr(std::same_as<Tag, nonbasic_at_lower_bound>)
                    return kHighsBasisStatusLower;
                else if constexpr(std::same_as<Tag, nonbasic_at_upper_bound>)
                    return kHighsBasisStatusUpper;
                else if constexpr(std::same_as<Tag, nonbasic_free>)
                    return kHighsBasisStatusZero;
                else {
                    static_assert(std::same_as<Tag, nonbasic>);
                    return kHighsBasisStatusNonbasic;
                }
            },
            status);
    }

public:
    // HiGHS keeps and extends its basis through changes of the model, but
    // reports it valid only until the first one.
    auto get_basis() {
        if(num_variables() == 0u) return basis_type{};
        HighsInt validity = 0;  // left unwritten when HiGHS holds no info
        check(Highs->getIntInfoValue(model, "basis_validity", &validity));
        if(validity != kHighsBasisValidityValid)
            throw solver_error(
                "mippp: get_basis() found no basis, HiGHS holds none from "
                "its last solve");
        const std::size_t num_col = _num_var_native_ids();
        const std::size_t num_row = num_constraints();
        std::vector<HighsInt> col_status(num_col), row_status(num_row);
        check(Highs->getBasis(model, col_status.data(), row_status.data()));
        detail::handle_status_table<basis_variable_status> columns(
            _handle_id_bound(num_col));
        for(std::size_t j = 0; j < num_col; ++j)
            columns.set(_var_handle(static_cast<int>(j)).uid(),
                        _basis_status<basis_variable_status>(col_status[j]));
        detail::handle_status_table<basis_constraint_status> rows(num_row);
        for(std::size_t i = 0; i < num_row; ++i)
            rows.set(i, _basis_status<basis_constraint_status>(row_status[i]));
        return basis_type(std::move(columns), std::move(rows));
    }
    template <detail::lp_basis_over<variable, constraint> B>
    void set_basis(const B & basis) {
        const std::size_t num_col = _num_var_native_ids();
        const std::size_t num_row = num_constraints();
        std::vector<HighsInt> col_status(num_col), row_status(num_row);
        for(std::size_t j = 0; j < num_col; ++j)
            col_status[j] =
                _basis_code(detail::basis_status_as<basis_variable_status>(
                    basis.get_status(_var_handle(static_cast<int>(j)))));
        for(std::size_t i = 0; i < num_row; ++i)
            row_status[i] =
                _basis_code(detail::basis_status_as<basis_constraint_status>(
                    basis.get_status(constraint(static_cast<int>(i)))));
        check(Highs->setBasis(model, col_status.data(), row_status.data()));
    }
};

}  // namespace highs::impl::v1
}  // namespace mippp
