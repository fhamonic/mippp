#pragma once

#include <concepts>
#include <cstddef>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>
#include <variant>
#include <vector>

#include "mippp/detail/restore_guard.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/utility/basis_snapshot.hpp"
#include "mippp/utility/variant.hpp"

#include "mippp/solvers/gurobi/impl/v1/gurobi_base.hpp"

namespace mippp {
namespace gurobi::impl::v1 {

class gurobi_lp : public gurobi_base {
public:
    [[nodiscard]] gurobi_lp() : gurobi_lp(gurobi_api::load()) {}
    [[nodiscard]] explicit gurobi_lp(const gurobi_api & api)
        : gurobi_base(api) {}

    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////// Limits //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void set_iteration_limit(std::size_t n) {
        check(GRB->setdblparam(env, GRB_DBL_PAR_ITERATIONLIMIT,
                               static_cast<double>(n)));
    }
    std::size_t get_iteration_limit() {
        double n;
        check(GRB->getdblparam(env, GRB_DBL_PAR_ITERATIONLIMIT, &n));
        // the default, 1e100, is beyond std::size_t: casting it is undefined
        constexpr auto max = std::numeric_limits<std::size_t>::max();
        if(n >= static_cast<double>(max)) return max;
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
            status::memory_limit,
            status::failed,
            status::numerical_failure,
            status::interrupted>;

    status_variant _status = status::unknown{};

    status_variant _get_status() {
        using namespace status;
        int status_;
        check(GRB->getintattr(model, GRB_INT_ATTR_STATUS, &status_));
        switch(status_) {
            case GRB_OPTIMAL:         return optimal{};
            case GRB_INF_OR_UNBD:     return infeasible_or_unbounded{};
            case GRB_INFEASIBLE:      return infeasible{};
            case GRB_UNBOUNDED:       return unbounded{};
        }
        int sol_count;
        check(GRB->getintattr(model, GRB_INT_ATTR_SOLCOUNT, &sol_count));
        const bool sol_available = sol_count > 0;
        switch(status_) {
            case GRB_TIME_LIMIT:      return time_limit{sol_available};
            case GRB_ITERATION_LIMIT: return iteration_limit{sol_available};
            case GRB_MEM_LIMIT:       return memory_limit{sol_available};
            case GRB_CUTOFF:
            case GRB_USER_OBJ_LIMIT:
            case GRB_WORK_LIMIT:      return limit_reached{sol_available};
            case GRB_SUBOPTIMAL:      return failed{sol_available};
            case GRB_NUMERIC:         return numerical_failure{sol_available};
            case GRB_INTERRUPTED:     return interrupted{sol_available};
            default:
                return unknown{sol_available}; 
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
        check(GRB->optimize(model));
        _status = _get_status();
        _mark_results_valid();
    }
    void refine_lp_status() {
        if(!is<status::infeasible_or_unbounded>(get_status())) return;
        int dual_reductions;
        check(GRB->getintparam(env, GRB_INT_PAR_DUALREDUCTIONS,
                               &dual_reductions));
        check(GRB->setintparam(env, GRB_INT_PAR_DUALREDUCTIONS, 0));
        detail::restore_guard guard([&] {
            check(GRB->setintparam(env, GRB_INT_PAR_DUALREDUCTIONS,
                                   dual_reductions));
        });
        check(GRB->optimize(model));
        _status = _get_status();
        guard.restore();
        if(is<status::infeasible_or_unbounded>(_status))
            throw std::runtime_error("Failed to refine LP status.");
    }
    // The routine solves when it has no status to trust and overwrites the
    // Status attribute either way, a stop leaving a limit code there: the
    // reported status is reset before the first native call, throw or return.
    auto compute_iis() {
        reset_status();
        return _compute_iis();
    }
    double get_solution_value() {
        _require_valid_results();
        double value;
        check(GRB->getdblattr(model, GRB_DBL_ATTR_OBJVAL, &value));
        return value;
    }
    auto get_solution() {
        _require_valid_results();
        auto solution =
            std::make_unique_for_overwrite<double[]>(_num_var_native_ids);
        check(GRB->getdblattrarray(model, GRB_DBL_ATTR_X, 0,
                                   static_cast<int>(_num_var_native_ids),
                                   solution.get()));
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
        check(GRB->getdblattrarray(model, GRB_DBL_ATTR_PI, 0,
                                   static_cast<int>(num_constrs),
                                   solution.get()));
        return constraint_mapping(std::move(solution));
    }
    auto get_reduced_costs() {
        _require_valid_results();
        auto reduced_costs =
            std::make_unique_for_overwrite<double[]>(_num_var_native_ids);
        check(GRB->getdblattrarray(model, GRB_DBL_ATTR_RC, 0,
                                   static_cast<int>(_num_var_native_ids),
                                   reduced_costs.get()));
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
    // Gurobi names no fixed column, a fixed one reads the side it gives, and
    // -3 is its one code for a free column and a superbasic one.
    using basis_variable_status =
        std::variant<basis_status::nonbasic, basis_status::basic,
                     basis_status::nonbasic_at_lower_bound,
                     basis_status::nonbasic_at_upper_bound,
                     basis_status::superbasic>;
    using basis_constraint_status =
        std::variant<basis_status::basic, basis_status::nonbasic,
                     basis_status::nonbasic_at_lower_bound,
                     basis_status::nonbasic_at_upper_bound,
                     basis_status::nonbasic_fixed>;
    using basis_type =
        basis_snapshot<variable, constraint, basis_variable_status,
                       basis_constraint_status>;

    // clang-format off
    static basis_variable_status _basis_variable_status(int code) noexcept {
        using namespace basis_status;
        switch(code) {
            case GRB_BASIC:          return basic{};
            case GRB_NONBASIC_LOWER: return nonbasic_at_lower_bound{};
            case GRB_NONBASIC_UPPER: return nonbasic_at_upper_bound{};
            case GRB_SUPERBASIC:     return superbasic{};
            default:
                                     return nonbasic{};
        }
    }
    // CBasis names no side: -1 holds the slack at zero, so the activity at
    // the right-hand side, which the sense places.
    static basis_constraint_status _basis_constraint_status(
        int code, char sense) noexcept {
        using namespace basis_status;
        if(code == GRB_BASIC)           return basic{};
        if(code == GRB_NONBASIC_LOWER) {
            switch(sense) {
                case GRB_LESS_EQUAL:    return nonbasic_at_upper_bound{};
                case GRB_GREATER_EQUAL: return nonbasic_at_lower_bound{};
                case GRB_EQUAL:         return nonbasic_fixed{};
            }
        }
        return nonbasic{};
    }

protected:
    // Gurobi repairs a side the bounds lack at the next solve.
    template <typename Status>
    static constexpr int _basis_variable_code(const Status & status) noexcept {
        using namespace basis_status;
        return std::visit<int>(detail::overloaded{
            [](basic)                   { return GRB_BASIC; },
            [](nonbasic_at_lower_bound) { return GRB_NONBASIC_LOWER; },
            [](nonbasic_at_upper_bound) { return GRB_NONBASIC_UPPER; },
            [](superbasic)              { return GRB_SUPERBASIC; },
            [](nonbasic)                { return GRB_NONBASIC_LOWER; }},
            status);
    }
    // clang-format on
    // A row's side is not written: Gurobi holds a nonbasic row at its
    // right-hand side.
    static constexpr int _basis_constraint_code(
        const basis_constraint_status & status) noexcept {
        return is<basis_status::basic>(status) ? GRB_BASIC : GRB_NONBASIC_LOWER;
    }

public:
    // Gurobi holds the basis of its last simplex solve until the update that
    // commits a change of the model, which get_basis() runs itself.
    auto get_basis() {
        _require_valid_results();
        if(num_variables() == 0u) return basis_type{};
        _lazily_remove_variables();
        update_gurobi_model();
        const std::size_t num_col = _num_var_native_ids;
        const std::size_t num_row = num_constraints();
        std::vector<int> col_codes(num_col), row_codes(num_row);
        std::vector<char> senses(num_row);
        check(GRB->getintattrarray(model, GRB_INT_ATTR_VBASIS, 0,
                                   static_cast<int>(num_col),
                                   col_codes.data()));
        check(GRB->getintattrarray(model, GRB_INT_ATTR_CBASIS, 0,
                                   static_cast<int>(num_row),
                                   row_codes.data()));
        check(GRB->getcharattrarray(model, GRB_CHAR_ATTR_SENSE, 0,
                                    static_cast<int>(num_row), senses.data()));
        detail::handle_status_table<basis_variable_status> columns(
            _handle_id_bound(num_col));
        for(std::size_t j = 0; j < num_col; ++j)
            columns.set(_var_handle(static_cast<int>(j)).uid(),
                        _basis_variable_status(col_codes[j]));
        detail::handle_status_table<basis_constraint_status> rows(num_row);
        for(std::size_t i = 0; i < num_row; ++i)
            rows.set(i, _basis_constraint_status(row_codes[i], senses[i]));
        return basis_type(std::move(columns), std::move(rows));
    }
    // A basis written over queued entities is lost, and one written over a
    // solved model is ignored, in ways that differ between releases: the
    // update and GRBreset come first. The reset drops the solution, whose
    // getters fail until the next solve, while get_status() still reports
    // the last one.
    template <detail::lp_basis_over<variable, constraint> B>
    void set_basis(const B & basis) {
        _invalidate_results();
        _lazily_remove_variables();
        update_gurobi_model();
        const std::size_t num_col = _num_var_native_ids;
        const std::size_t num_row = num_constraints();
        std::vector<int> col_codes(num_col), row_codes(num_row);
        for(std::size_t j = 0; j < num_col; ++j)
            col_codes[j] = _basis_variable_code(
                detail::basis_status_as<basis_variable_status>(
                    basis.get_status(_var_handle(static_cast<int>(j)))));
        for(std::size_t i = 0; i < num_row; ++i)
            row_codes[i] = _basis_constraint_code(
                detail::basis_status_as<basis_constraint_status>(
                    basis.get_status(constraint(static_cast<int>(i)))));
        check(GRB->reset(model, 0));
        check(GRB->setintattrarray(model, GRB_INT_ATTR_VBASIS, 0,
                                   static_cast<int>(num_col),
                                   col_codes.data()));
        check(GRB->setintattrarray(model, GRB_INT_ATTR_CBASIS, 0,
                                   static_cast<int>(num_row),
                                   row_codes.data()));
    }
};

}  // namespace gurobi::impl::v1
}  // namespace mippp
