#pragma once

#include <algorithm>
#include <cstddef>
#include <memory>
#include <utility>
#include <variant>
#include <vector>

#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"
#include "mippp/utility/basis_snapshot.hpp"
#include "mippp/utility/solver_exceptions.hpp"

#include "mippp/solvers/copt/impl/v1/copt_base.hpp"

namespace mippp {
namespace copt::impl::v1 {

class copt_lp : public copt_base {
public:
    [[nodiscard]] copt_lp() : copt_lp(copt_api::load()) {}
    [[nodiscard]] explicit copt_lp(const copt_api & api) : copt_base(api) {}

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
            status::time_limit,
            status::numerical_failure,
            status::interrupted>;

    status_variant _status = status::unknown{};

    status_variant _get_status() {
        using namespace status;
        int status_;
        check(COPT->GetIntAttr(prob, COPT_INTATTR_LPSTATUS, &status_));
        int has_sol;
        check(COPT->GetIntAttr(prob, COPT_INTATTR_HASLPSOL, &has_sol));
        switch(status_) {
            case COPT_LPSTATUS_OPTIMAL: return optimal{};
            case COPT_LPSTATUS_INFEASIBLE: return infeasible{};
            case COPT_LPSTATUS_UNBOUNDED: return unbounded{};
            case COPT_LPSTATUS_TIMEOUT: return time_limit{has_sol != 0};
            case COPT_LPSTATUS_NUMERICAL:
            case COPT_LPSTATUS_IMPRECISE:
            case COPT_LPSTATUS_UNFINISHED: return numerical_failure{has_sol != 0};
            case COPT_LPSTATUS_INTERRUPTED: return interrupted{has_sol != 0};
            case COPT_LPSTATUS_UNSTARTED: return unknown{};
            default:
                return unknown{has_sol != 0};
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
        check(COPT->SolveLp(prob));
        _status = _get_status();
    }
    // A generic code from the routine is confirmed by one LP solve of the
    // wrapper: the reported status is reset before the first native call,
    // throw or return.
    auto compute_iis() {
        reset_status();
        return _compute_iis<detail::iis_sided_status, detail::iis_sided_status>(
            false);
    }
    double get_solution_value() {
        double val;
        check(COPT->GetDblAttr(prob, COPT_DBLATTR_LPOBJVAL, &val));
        return val;
    }
    auto get_solution() {
        auto solution =
            std::make_unique_for_overwrite<double[]>(num_variables());
        check(COPT->GetLpSolution(prob, solution.get(), nullptr, nullptr,
                                  nullptr));
        return variable_mapping(std::move(solution));
    }
    auto get_dual_solution() {
        auto dual_solution =
            std::make_unique_for_overwrite<double[]>(num_constraints());
        check(COPT->GetLpSolution(prob, nullptr, nullptr, dual_solution.get(),
                                  nullptr));
        return constraint_mapping(std::move(dual_solution));
    }
    auto get_reduced_costs() {
        auto reduced_costs =
            std::make_unique_for_overwrite<double[]>(num_variables());
        check(COPT->GetLpSolution(prob, nullptr, nullptr, nullptr,
                                  reduced_costs.get()));
        return variable_mapping(std::move(reduced_costs));
    }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////////// Basis //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    // COPT's superbasic code covers free entities too: no nonbasic_free
    using basis_variable_status =
        std::variant<basis_status::nonbasic, basis_status::basic,
                     basis_status::nonbasic_at_lower_bound,
                     basis_status::nonbasic_at_upper_bound,
                     basis_status::nonbasic_fixed, basis_status::superbasic>;
    using basis_constraint_status =
        std::variant<basis_status::basic, basis_status::nonbasic,
                     basis_status::nonbasic_at_lower_bound,
                     basis_status::nonbasic_at_upper_bound,
                     basis_status::nonbasic_fixed, basis_status::superbasic>;
    using basis_type =
        basis_snapshot<variable, constraint, basis_variable_status,
                       basis_constraint_status>;

    template <typename Status>
    static Status _basis_status(int code) noexcept {
        switch(code) {
            case COPT_BASIS_BASIC:
                return Status(std::in_place_type<basis_status::basic>);
            case COPT_BASIS_LOWER:
                return Status(
                    std::in_place_type<basis_status::nonbasic_at_lower_bound>);
            case COPT_BASIS_UPPER:
                return Status(
                    std::in_place_type<basis_status::nonbasic_at_upper_bound>);
            case COPT_BASIS_FIXED:
                return Status(std::in_place_type<basis_status::nonbasic_fixed>);
            case COPT_BASIS_SUPERBASIC:
                return Status(std::in_place_type<basis_status::superbasic>);
            default:
                return Status(std::in_place_type<basis_status::nonbasic>);
        }
    }
    // COPT snaps a side the bounds do not have at the next solve.
    template <typename Status>
    static int _basis_code(const Status & status) noexcept {
        return std::visit(
            []<typename Tag>(Tag) -> int {
                using namespace basis_status;
                if constexpr(std::same_as<Tag, basic>)
                    return COPT_BASIS_BASIC;
                else if constexpr(std::same_as<Tag, nonbasic_at_lower_bound>)
                    return COPT_BASIS_LOWER;
                else if constexpr(std::same_as<Tag, nonbasic_at_upper_bound>)
                    return COPT_BASIS_UPPER;
                else if constexpr(std::same_as<Tag, nonbasic_fixed>)
                    return COPT_BASIS_FIXED;
                else if constexpr(std::same_as<Tag, superbasic>)
                    return COPT_BASIS_SUPERBASIC;
                else {
                    static_assert(std::same_as<Tag, nonbasic>);
                    return COPT_BASIS_LOWER;
                }
            },
            status);
    }

public:
    // COPT_Reset, which compute_iis() runs, leaves zeros that COPT_GetBasis
    // returns as a basis, every entity at lower and none basic: a count of
    // basic entries other than the rows is no basis.
    auto get_basis() {
        const std::size_t num_col = num_variables();
        if(num_col == 0u) return basis_type{};
        const std::size_t num_row = num_constraints();
        std::vector<int> col_status(num_col), row_status(num_row);
        const auto code =
            COPT->GetBasis(prob, col_status.data(), row_status.data());
        if(code != COPT_RETCODE_INVALID) check(code);
        std::size_t num_basic = 0;
        detail::handle_status_table<basis_variable_status> columns(num_col);
        for(std::size_t j = 0; j < num_col; ++j) {
            if(col_status[j] == COPT_BASIS_BASIC) ++num_basic;
            columns.set(j, _basis_status<basis_variable_status>(col_status[j]));
        }
        detail::handle_status_table<basis_constraint_status> rows(num_row);
        for(std::size_t i = 0; i < num_row; ++i) {
            if(row_status[i] == COPT_BASIS_BASIC) ++num_basic;
            rows.set(i, _basis_status<basis_constraint_status>(row_status[i]));
        }
        // COPT answers COPT_RETCODE_INVALID when it holds no basis
        if(code == COPT_RETCODE_INVALID || num_basic != num_row)
            throw solver_error(
                "mippp: get_basis() found no basis, COPT holds none from its "
                "last solve");
        return basis_type(std::move(columns), std::move(rows));
    }
    // COPT_SetBasis rejects a null array, even for an empty dimension. It
    // drops the solution but not COPT's status: the solution getters throw
    // until the next solve, while get_status() reports the last one.
    template <detail::lp_basis_over<variable, constraint> B>
    void set_basis(const B & basis) {
        const std::size_t num_col = num_variables();
        const std::size_t num_row = num_constraints();
        std::vector<int> col_status(std::max(num_col, std::size_t{1}));
        std::vector<int> row_status(std::max(num_row, std::size_t{1}));
        for(std::size_t j = 0; j < num_col; ++j)
            col_status[j] =
                _basis_code(detail::basis_status_as<basis_variable_status>(
                    basis.get_status(variable(static_cast<int>(j)))));
        for(std::size_t i = 0; i < num_row; ++i)
            row_status[i] =
                _basis_code(detail::basis_status_as<basis_constraint_status>(
                    basis.get_status(constraint(static_cast<int>(i)))));
        check(COPT->SetBasis(prob, col_status.data(), row_status.data()));
    }
};

}  // namespace copt::impl::v1
}  // namespace mippp
