#pragma once

#include <algorithm>
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
#include "mippp/model_entities.hpp"
#include "mippp/utility/basis_snapshot.hpp"
#include "mippp/utility/variant.hpp"

#include "mippp/solvers/cplex/impl/v1/cplex_base.hpp"

namespace mippp {
namespace cplex::impl::v1 {

class cplex_lp : public cplex_base {
public:
    [[nodiscard]] cplex_lp() : cplex_lp(cplex_api::load()) {}
    [[nodiscard]] explicit cplex_lp(const cplex_api & api) : cplex_base(api) {}

    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////// Limits //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void set_iteration_limit(std::size_t count) {
        check(CPX->setintparam(env, CPXPARAM_Simplex_Limits_Iterations,
                               static_cast<int>(std::min<std::size_t>(
                                   count, std::numeric_limits<int>::max()))));
    }
    auto get_iteration_limit() {
        int count;
        check(
            CPX->getintparam(env, CPXPARAM_Simplex_Limits_Iterations, &count));
        return static_cast<std::size_t>(count);
    }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////// Tolerance parameters ///////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void set_optimality_tolerance(double tol) {
        check(
            CPX->setdblparam(env, CPXPARAM_Simplex_Tolerances_Optimality, tol));
    }
    double get_optimality_tolerance() {
        double tol;
        check(CPX->getdblparam(env, CPXPARAM_Simplex_Tolerances_Optimality,
                               &tol));
        return tol;
    }
    void set_feasibility_tolerance(double tol) {
        check(CPX->setdblparam(env, CPXPARAM_Simplex_Tolerances_Feasibility,
                               tol));
    }
    double get_feasibility_tolerance() {
        double tol;
        check(CPX->getdblparam(env, CPXPARAM_Simplex_Tolerances_Feasibility,
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
            status::optimal_face_unbounded,
            status::optimal_infeasible_unscaled,
            status::infeasible_or_unbounded,
            status::infeasible,
            status::unbounded,
            status::limit_reached,
            status::time_limit,
            status::iteration_limit,
            status::numerical_failure,
            status::interrupted>;

    status_variant _status = status::unknown{};
    
    status_variant _get_status() {
        using namespace status;
        switch(CPX->getstat(env, lp)) {
            case CPX_STAT_OPTIMAL:        return optimal{};
            case CPX_STAT_OPTIMAL_FACE_UNBOUNDED: return optimal_face_unbounded{};
            case CPX_STAT_OPTIMAL_INFEAS: return optimal_infeasible_unscaled{};
            case CPX_STAT_INForUNBD:      return infeasible_or_unbounded{};
            case CPX_STAT_INFEASIBLE:     return infeasible{};
            case CPX_STAT_UNBOUNDED:      return unbounded{};
            case CPX_STAT_ABORT_TIME_LIM: return time_limit{};
            case CPX_STAT_ABORT_IT_LIM:   return iteration_limit{};
            case CPX_STAT_NUM_BEST:       return numerical_failure{true};
            case CPX_STAT_ABORT_USER:     return interrupted{};
            // no dedicated status type: these collapse into limit_reached
            case CPX_STAT_ABORT_OBJ_LIM:      return limit_reached{true};
            case CPX_STAT_ABORT_PRIM_OBJ_LIM: return limit_reached{};
            case CPX_STAT_ABORT_DUAL_OBJ_LIM: return limit_reached{};
            case CPX_STAT_ABORT_DETTIME_LIM:  return limit_reached{};
            case CPX_STAT_UNKNOWN:
            default:
                return unknown{};
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
        check(CPX->primopt(env, lp));
        _status = _get_status();
    }
    void refine_lp_status() {
        if(!is<status::infeasible_or_unbounded>(_status)) return;
        int advance, reduce;
        check(CPX->getintparam(env, CPXPARAM_Advance, &advance));
        check(CPX->getintparam(env, CPXPARAM_Preprocessing_Reduce, &reduce));
        // armed before the writes: a failed second write needs the first
        // one written back too
        detail::restore_guard guard([&] {
            const int advance_status =
                CPX->setintparam(env, CPXPARAM_Advance, advance);
            const int reduce_status =
                CPX->setintparam(env, CPXPARAM_Preprocessing_Reduce, reduce);
            check(advance_status);
            check(reduce_status);
        });
        check(CPX->setintparam(env, CPXPARAM_Advance, 0));
        check(CPX->setintparam(env, CPXPARAM_Preprocessing_Reduce,
                               CPX_PREREDUCE_NOPRIMALORDUAL));
        check(CPX->primopt(env, lp));
        _status = _get_status();
        guard.restore();
        if(is<status::infeasible_or_unbounded>(_status))
            throw std::runtime_error("Failed to refine LP status.");
    }
    // The refiner replaces CPXgetstat with its own statuses, which do not
    // describe the held solution: the reported status is reset before the
    // native call, throw or return.
    auto compute_iis() {
        reset_status();
        return _compute_iis();
    }
    double get_solution_value() {
        double val;
        check(CPX->solution(env, lp, nullptr, &val, nullptr, nullptr, nullptr,
                            nullptr));
        return val;
    }
    auto get_solution() {
        auto solution =
            std::make_unique_for_overwrite<double[]>(num_variables());
        check(CPX->solution(env, lp, nullptr, nullptr, solution.get(), nullptr,
                            nullptr, nullptr));
        return variable_mapping(
            [this, solution = std::move(solution)](const variable & v) {
                return *(solution.get() + _native_id(v));
            });
    }
    auto get_dual_solution() {
        auto dual_solution =
            std::make_unique_for_overwrite<double[]>(num_constraints());
        check(CPX->solution(env, lp, nullptr, nullptr, nullptr,
                            dual_solution.get(), nullptr, nullptr));
        return constraint_mapping(std::move(dual_solution));
    }
    auto get_reduced_costs() {
        auto reduced_costs =
            std::make_unique_for_overwrite<double[]>(num_variables());
        check(CPX->solution(env, lp, nullptr, nullptr, nullptr, nullptr,
                            nullptr, reduced_costs.get()));
        return variable_mapping([this, reduced_costs = std::move(
                                           reduced_costs)](const variable & v) {
            return *(reduced_costs.get() + _native_id(v));
        });
    }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////////// Basis //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    // CPLEX names no fixed column, a fixed one reads the side of its reduced
    // cost, and one code covers free and superbasic columns
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
    static basis_variable_status _column_basis_status(int code) noexcept {
        using namespace basis_status;
        switch(code) {
            case CPX_BASIC:      return basic{};
            case CPX_AT_LOWER:   return nonbasic_at_lower_bound{};
            case CPX_AT_UPPER:   return nonbasic_at_upper_bound{};
            case CPX_FREE_SUPER: return superbasic{};
            default:
                                 return nonbasic{};
        }
    }
    // A row code names the side of CPLEX's slack s >= 0, a.x + s = rhs on an
    // 'L' row and a.x - s = rhs on a 'G' or 'R' row: at lower, the activity
    // of an 'L' row is at its upper side.
    static basis_constraint_status _row_basis_status(
        int code, char sense) noexcept {
        using namespace basis_status;
        if(code == CPX_BASIC) return basic{};
        if(code == CPX_AT_LOWER) {
            switch(sense) {
                case 'L':     return nonbasic_at_upper_bound{};
                case 'E':     return nonbasic_fixed{};
                case 'G':
                case 'R':     return nonbasic_at_lower_bound{};
            }
        }
        if(code == CPX_AT_UPPER && sense == 'R')
                              return nonbasic_at_upper_bound{};
        return nonbasic{};
    }

protected:
    // CPLEX snaps a side the bounds do not have at the next solve.
    template <typename Status>
    static constexpr int _column_basis_code(const Status & status) noexcept {
        using namespace basis_status;
        return std::visit<int>(detail::overloaded{
            [](basic)                   { return CPX_BASIC; },
            [](nonbasic_at_lower_bound) { return CPX_AT_LOWER; },
            [](nonbasic_at_upper_bound) { return CPX_AT_UPPER; },
            [](superbasic)              { return CPX_FREE_SUPER; },
            [](nonbasic)                { return CPX_AT_LOWER; }},
            status);
    }
    // clang-format on
    // only the slack of an 'R' row has two distinct bounds
    static constexpr int _row_basis_code(const basis_constraint_status & status,
                                         char sense) noexcept {
        if(is<basis_status::basic>(status)) return CPX_BASIC;
        if(is<basis_status::nonbasic_at_upper_bound>(status) && sense == 'R')
            return CPX_AT_UPPER;
        return CPX_AT_LOWER;
    }

private:
    std::vector<char> _row_senses(std::size_t num_row) {
        std::vector<char> senses(num_row);
        if(num_row > 0)
            check(CPX->getsense(env, lp, senses.data(), 0,
                                static_cast<int>(num_row) - 1));
        return senses;
    }

public:
    // CPLEX keeps and adjusts its basis through changes of the model, and
    // reads a written one back verbatim until the next solve.
    auto get_basis() {
        if(num_variables() == 0u) return basis_type{};
        const std::size_t num_col = _num_var_native_ids();
        const std::size_t num_row = num_constraints();
        std::vector<int> col_status(num_col), row_status(num_row);
        check(CPX->getbase(env, lp, col_status.data(), row_status.data()));
        const std::vector<char> senses = _row_senses(num_row);
        detail::handle_status_table<basis_variable_status> columns(
            _handle_id_bound(num_col));
        for(std::size_t j = 0; j < num_col; ++j)
            columns.set(_var_handle(static_cast<int>(j)).uid(),
                        _column_basis_status(col_status[j]));
        detail::handle_status_table<basis_constraint_status> rows(num_row);
        for(std::size_t i = 0; i < num_row; ++i)
            rows.set(i, _row_basis_status(row_status[i], senses[i]));
        return basis_type(std::move(columns), std::move(rows));
    }
    // CPXcopybase refuses a null array, even for an empty dimension.
    template <detail::lp_basis_over<variable, constraint> B>
    void set_basis(const B & basis) {
        const std::size_t num_col = _num_var_native_ids();
        const std::size_t num_row = num_constraints();
        const std::vector<char> senses = _row_senses(num_row);
        std::vector<int> col_status(std::max<std::size_t>(num_col, 1)),
            row_status(std::max<std::size_t>(num_row, 1));
        for(std::size_t j = 0; j < num_col; ++j)
            col_status[j] = _column_basis_code(
                detail::basis_status_as<basis_variable_status>(
                    basis.get_status(_var_handle(static_cast<int>(j)))));
        for(std::size_t i = 0; i < num_row; ++i)
            row_status[i] = _row_basis_code(
                detail::basis_status_as<basis_constraint_status>(
                    basis.get_status(constraint(static_cast<int>(i)))),
                senses[i]);
        check(CPX->copybase(env, lp, col_status.data(), row_status.data()));
    }
};

}  // namespace cplex::impl::v1
}  // namespace mippp
