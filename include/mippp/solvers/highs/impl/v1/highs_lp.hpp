#pragma once

#include <algorithm>
#include <concepts>
#include <cstddef>
#include <limits>
#include <memory>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include "mippp/detail/lp_algorithm_codec.hpp"
#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"
#include "mippp/utility/basis_snapshot.hpp"
#include "mippp/utility/solver_exceptions.hpp"
#include "mippp/utility/variant.hpp"

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
    ////////////////////////////// LP algorithm ///////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    // No interior point without crossover: HiGHS then reports a feasible LP
    // as unknown where its dual point fails postsolve, and Highs_run fails
    // on an unbounded one.
    using lp_algorithm_variant =
        std::variant<lp_algorithm::automatic, lp_algorithm::simplex,
                     lp_algorithm::primal_simplex, lp_algorithm::dual_simplex,
                     lp_algorithm::interior_point, lp_algorithm::other>;

protected:
    // views over string literals, so that data() is a C string
    struct lp_algorithm_knobs {
        std::string_view solver;
        HighsInt simplex_strategy;
        std::string_view run_crossover;
        bool operator==(const lp_algorithm_knobs &) const = default;
    };
    // simplex_strategy: 1, the dual simplex, is HiGHS's default, 4 is the
    // primal, and under 0 HiGHS picks the side. run_crossover defaults to on.
    template <typename A>
    static constexpr lp_algorithm_knobs _lp_algorithm_knobs(A) noexcept {
        using namespace lp_algorithm;
        if constexpr(std::same_as<A, automatic>)
            return {"choose", 1, "on"};
        else if constexpr(std::same_as<A, simplex>)
            return {"simplex", 0, "on"};
        else if constexpr(std::same_as<A, primal_simplex>)
            return {"simplex", 4, "on"};
        else if constexpr(std::same_as<A, dual_simplex>)
            return {"simplex", 1, "on"};
        else if constexpr(std::same_as<A, interior_point>)
            return {"ipm", 1, "on"};
        else
            static_assert(std::same_as<A, void>, "a listed tag has no row");
    }

public:
    // The three options at every call: a simplex_strategy left at 4 would
    // make the next automatic run the primal simplex.
    template <detail::settable_lp_algorithm<lp_algorithm_variant> A>
    void set_lp_algorithm(A algorithm) {
        const lp_algorithm_knobs knobs = _lp_algorithm_knobs(algorithm);
        check(
            Highs->setStringOptionValue(model, "solver", knobs.solver.data()));
        check(Highs->setIntOptionValue(model, "simplex_strategy",
                                       knobs.simplex_strategy));
        check(Highs->setStringOptionValue(model, "run_crossover",
                                          knobs.run_crossover.data()));
    }
    lp_algorithm_variant get_lp_algorithm() {
        char solver[kHighsMaximumStringLength];
        char run_crossover[kHighsMaximumStringLength];
        HighsInt simplex_strategy;
        check(Highs->getStringOptionValue(model, "solver", solver));
        check(Highs->getIntOptionValue(model, "simplex_strategy",
                                       &simplex_strategy));
        check(
            Highs->getStringOptionValue(model, "run_crossover", run_crossover));
        return detail::decode_lp_algorithm(
            lp_algorithm_variant{lp_algorithm::other{}},
            lp_algorithm_knobs{solver, simplex_strategy, run_crossover},
            [](auto algorithm) { return _lp_algorithm_knobs(algorithm); });
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
    const status_variant & get_status() const {
        return _results_valid ? _status
                              : detail::unknown_status<status_variant>;
    }
    void reset_status() noexcept {
        _status = status::unknown{};
        _invalidate_results();
    }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////////// Solve //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void solve() {
        reset_status();  // a solve that throws reports unknown
        if(num_variables() == 0u) {
            _mark_results_valid();
            return;
        }
        _run();
        _status = _get_status();
        _mark_results_valid();
    }
    // The routine re-solves and overwrites the model status: the reported
    // status is reset before the first native call, throw or return.
    auto compute_iis() {
        reset_status();
        return _compute_iis();
    }
    double get_solution_value() {
        _require_valid_results();
        return Highs->getObjectiveValue(model);
    }
    auto get_solution() {
        _require_valid_results();
        auto num_vars = num_variables();
        auto solution = std::make_unique_for_overwrite<double[]>(num_vars);
        check(Highs->getSolution(model, solution.get(), nullptr, nullptr,
                                 nullptr));
        return variable_mapping(
            [native_id = _frozen_native_id(),
             solution = std::move(solution)](const variable & v) {
                return *(solution.get() + native_id(v));
            });
    }
    auto get_dual_solution() {
        _require_valid_results();
        auto num_constrs = num_constraints();
        auto solution = std::make_unique_for_overwrite<double[]>(num_constrs);
        check(Highs->getSolution(model, nullptr, nullptr, nullptr,
                                 solution.get()));
        return constraint_mapping(std::move(solution));
    }
    auto get_reduced_costs() {
        _require_valid_results();
        auto num_vars = num_variables();
        auto reduced_costs = std::make_unique_for_overwrite<double[]>(num_vars);
        check(Highs->getSolution(model, nullptr, reduced_costs.get(), nullptr,
                                 nullptr));
        return variable_mapping(
            [native_id = _frozen_native_id(),
             reduced_costs = std::move(reduced_costs)](const variable & v) {
                return *(reduced_costs.get() + native_id(v));
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

    // clang-format off
    template <typename Status>
    static Status _basis_status(HighsInt code) noexcept {
        using namespace basis_status;
        switch(code) {
            case kHighsBasisStatusBasic: return basic{};
            case kHighsBasisStatusLower: return nonbasic_at_lower_bound{};
            case kHighsBasisStatusUpper: return nonbasic_at_upper_bound{};
            case kHighsBasisStatusZero:  return nonbasic_free{};
            default:
                                         return nonbasic{};
        }
    }

protected:
    // A tag this variant lacks, as nonbasic_fixed or superbasic, arrives as
    // bare nonbasic, whose Nonbasic code lets HiGHS pick the side.
    template <typename Status>
    static constexpr HighsInt _basis_code(const Status & status) noexcept {
        using namespace basis_status;
        return std::visit<HighsInt>(detail::overloaded{
            [](basic)                   { return kHighsBasisStatusBasic; },
            [](nonbasic_at_lower_bound) { return kHighsBasisStatusLower; },
            [](nonbasic_at_upper_bound) { return kHighsBasisStatusUpper; },
            [](nonbasic_free)           { return kHighsBasisStatusZero; },
            [](nonbasic)                { return kHighsBasisStatusNonbasic; }},
            status);
    }
    // clang-format on

public:
    // HiGHS keeps and extends its basis through changes of the model, but
    // reports it valid only until the first one.
    auto get_basis() {
        _require_valid_results();
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
        _invalidate_results();
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
