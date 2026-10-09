#pragma once

#include <cstddef>
#include <memory>
#include <utility>
#include <variant>
#include <vector>

#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"
#include "mippp/utility/basis_snapshot.hpp"
#include "mippp/utility/solver_exceptions.hpp"

#include "mippp/solvers/xpress/impl/v1/xpress_base.hpp"

namespace mippp {
namespace xpress::impl::v1 {

class xpress_lp : public xpress_base {
public:
    [[nodiscard]] xpress_lp() : xpress_lp(xpress_api::load()) {}
    [[nodiscard]] explicit xpress_lp(const xpress_api & api)
        : xpress_base(api) {}

    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////// Solve status ///////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    // clang-format off
private:
    using status_variant = std::variant<
            status::unknown,
            status::optimal,
            status::infeasible,
            status::unbounded,
            status::limit_reached,
            status::time_limit,
            status::iteration_limit,
            status::failed,
            status::numerical_failure,
            status::out_of_memory,
            status::interrupted>;

    status_variant _status = status::unknown{};

    status_variant _get_status() {
        using namespace status;
        int stop_status;
        check(XPRS->getintattrib(prob, XPRS_STOPSTATUS, &stop_status));    
        switch(stop_status) {
            case XPRS_STOP_NONE:
            case XPRS_STOP_SOLVECOMPLETE: {
                int lp_status;
                check(XPRS->getintattrib(prob, XPRS_LPSTATUS, &lp_status));    
                switch(lp_status) {
                    case XPRS_LP_OPTIMAL:    return optimal{};
                    case XPRS_LP_INFEAS:     return infeasible{};
                    case XPRS_LP_UNBOUNDED:  return unbounded{};
                    case XPRS_LP_CUTOFF_IN_DUAL:
                    case XPRS_LP_CUTOFF:     return limit_reached{};
                    case XPRS_LP_UNSOLVED:   return failed{};
                    case XPRS_LP_UNFINISHED: return interrupted{};
                    case XPRS_LP_UNSTARTED:
                    default:
                        return unknown{};
                } 
            }
            case XPRS_STOP_TIMELIMIT:      return time_limit{};
            case XPRS_STOP_ITERLIMIT:      return iteration_limit{};
            case XPRS_STOP_WORKLIMIT:      return limit_reached{};
            case XPRS_STOP_MEMORYERROR:    return out_of_memory{};
            case XPRS_STOP_NUMERICALERROR: return numerical_failure{};
            case XPRS_STOP_GENERICERROR:   return failed{};
            case XPRS_STOP_USER:
            case XPRS_STOP_LICENSELOST:
            case XPRS_STOP_CTRLC:          return interrupted{};
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
        if(_bounds_hold_no_value()) {
            _status = status::infeasible{};
            return;
        }
        check(XPRS->lpoptimize(prob, nullptr));
        _status = _get_status();
        // A stopped solve can leave the problem LP-presolved (45.01 does),
        // where XPRS_ROWS, the row getters and setters describe the presolved
        // rows and nothing can be added. Postsolving an original problem
        // does nothing.
        check(XPRS->postsolve(prob));
    }
    // The routine solves on its own and overwrites the status attributes and
    // the held solution: the reported status is reset before the first native
    // call, throw or return.
    auto compute_iis() {
        reset_status();
        return _compute_iis<detail::iis_sided_status, detail::iis_sided_status>(
            false);
    }
    double get_solution_value() {
        double val;
        check(XPRS->getdblattrib(prob, XPRS_LPOBJVAL, &val));
        return objective_offset + val;
    }
    auto get_solution() {
        const auto num_vars = num_variables();
        auto solution = std::make_unique_for_overwrite<double[]>(num_vars);
        check(XPRS->getsolution(prob, nullptr, solution.get(), 0,
                                static_cast<int>(num_vars) - 1));
        return variable_mapping(std::move(solution));
    }
    auto get_dual_solution() {
        const auto num_constrs = num_constraints();
        auto dual_solution =
            std::make_unique_for_overwrite<double[]>(num_constrs);
        check(XPRS->getduals(prob, nullptr, dual_solution.get(), 0,
                             static_cast<int>(num_constrs) - 1));
        return constraint_mapping(std::move(dual_solution));
    }
    auto get_reduced_costs() {
        const auto num_vars = num_variables();
        auto reduced_costs = std::make_unique_for_overwrite<double[]>(num_vars);
        check(XPRS->getredcosts(prob, nullptr, reduced_costs.get(), 0,
                                static_cast<int>(num_vars) - 1));
        return variable_mapping(std::move(reduced_costs));
    }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////////// Basis //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    // Xpress names no fixed entity, a fixed one reads the side of its dual,
    // and its superbasic code covers free entities too.
    using basis_variable_status =
        std::variant<basis_status::nonbasic, basis_status::basic,
                     basis_status::nonbasic_at_lower_bound,
                     basis_status::nonbasic_at_upper_bound,
                     basis_status::superbasic>;
    using basis_constraint_status =
        std::variant<basis_status::basic, basis_status::nonbasic,
                     basis_status::nonbasic_at_lower_bound,
                     basis_status::nonbasic_at_upper_bound,
                     basis_status::superbasic>;
    using basis_type =
        basis_snapshot<variable, constraint, basis_variable_status,
                       basis_constraint_status>;

    // A row's code names the side of its slack rhs - a.x, the opposite of the
    // side of its activity, on every row type.
    static constexpr int _column_at_lower = XPRS_BASISSTATUS_NONBASIC_LOWER;
    static constexpr int _column_at_upper = XPRS_BASISSTATUS_NONBASIC_UPPER;
    static constexpr int _row_at_lower = XPRS_BASISSTATUS_NONBASIC_UPPER;
    static constexpr int _row_at_upper = XPRS_BASISSTATUS_NONBASIC_LOWER;

    template <typename Status, int AtLower, int AtUpper>
    static Status _basis_status(int code) noexcept {
        switch(code) {
            case XPRS_BASISSTATUS_BASIC:
                return Status(std::in_place_type<basis_status::basic>);
            case AtLower:
                return Status(
                    std::in_place_type<basis_status::nonbasic_at_lower_bound>);
            case AtUpper:
                return Status(
                    std::in_place_type<basis_status::nonbasic_at_upper_bound>);
            case XPRS_BASISSTATUS_SUPERBASIC:
                return Status(std::in_place_type<basis_status::superbasic>);
            default:
                return Status(std::in_place_type<basis_status::nonbasic>);
        }
    }
    // A fixed entity sits at both its sides, and Xpress snaps at the write a
    // side the bounds lack, so the native lower code serves a fixed entity
    // and a bare nonbasic one.
    template <int AtLower, int AtUpper, typename Status>
    static int _basis_code(const Status & status) noexcept {
        return std::visit(
            []<typename Tag>(Tag) -> int {
                using namespace basis_status;
                if constexpr(std::same_as<Tag, basic>)
                    return XPRS_BASISSTATUS_BASIC;
                else if constexpr(std::same_as<Tag, nonbasic_at_lower_bound>)
                    return AtLower;
                else if constexpr(std::same_as<Tag, nonbasic_at_upper_bound>)
                    return AtUpper;
                else if constexpr(std::same_as<Tag, superbasic>)
                    return XPRS_BASISSTATUS_SUPERBASIC;
                else {
                    static_assert(std::same_as<Tag, nonbasic>);
                    return XPRS_BASISSTATUS_NONBASIC_LOWER;
                }
            },
            status);
    }

public:
    // Xpress answers before any solve with the slack basis, and after a
    // re-solve whose presolve proves the LP infeasible or unbounded with the
    // previous solve's basis; the valid-solution bit of PRESOLVESTATE is
    // clear in both, and any change of the model clears it. set_basis()
    // leaves the bit alone: a basis written over a solved model reads back as
    // written, one written before any solve still throws here.
    auto get_basis() {
        const std::size_t num_col = num_variables();
        if(num_col == 0u) return basis_type{};
        int presolve_state = 0;
        check(XPRS->getintattrib(prob, XPRS_PRESOLVESTATE, &presolve_state));
        if((presolve_state & XPRS_PRESOLVESTATE_SOLUTIONVALID) == 0)
            throw solver_error(
                "mippp: get_basis() found no basis, Xpress holds none from "
                "its last solve");
        const std::size_t num_row = num_constraints();
        std::vector<int> col_status(num_col), row_status(num_row);
        check(XPRS->getbasis(prob, row_status.data(), col_status.data()));
        detail::handle_status_table<basis_variable_status> columns(num_col);
        for(std::size_t j = 0; j < num_col; ++j)
            columns.set(j,
                        _basis_status<basis_variable_status, _column_at_lower,
                                      _column_at_upper>(col_status[j]));
        detail::handle_status_table<basis_constraint_status> rows(num_row);
        for(std::size_t i = 0; i < num_row; ++i)
            rows.set(i, _basis_status<basis_constraint_status, _row_at_lower,
                                      _row_at_upper>(row_status[i]));
        return basis_type(std::move(columns), std::move(rows));
    }
    // Xpress reads both arrays whole, through a null pointer too, and refuses
    // a presolved problem, which solve() never leaves.
    template <detail::lp_basis_over<variable, constraint> B>
    void set_basis(const B & basis) {
        const std::size_t num_col = num_variables();
        const std::size_t num_row = num_constraints();
        std::vector<int> col_status(num_col), row_status(num_row);
        for(std::size_t j = 0; j < num_col; ++j)
            col_status[j] = _basis_code<_column_at_lower, _column_at_upper>(
                detail::basis_status_as<basis_variable_status>(
                    basis.get_status(variable(static_cast<int>(j)))));
        for(std::size_t i = 0; i < num_row; ++i)
            row_status[i] = _basis_code<_row_at_lower, _row_at_upper>(
                detail::basis_status_as<basis_constraint_status>(
                    basis.get_status(constraint(static_cast<int>(i)))));
        check(XPRS->loadbasis(prob, row_status.data(), col_status.data()));
    }
};

}  // namespace xpress::impl::v1
}  // namespace mippp
