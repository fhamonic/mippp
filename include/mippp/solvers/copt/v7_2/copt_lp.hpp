#pragma once

#include <optional>

#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/copt/v7_2/copt_base.hpp"

namespace mippp {
namespace copt::v7_2 {

class copt_lp : public copt_base {
public:
    [[nodiscard]] explicit copt_lp(const copt_api & api) : copt_base(api) {}

    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////// Solve status ///////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    // clang-format off
private:
    using status_variant = std::variant<
            status::unknown, // default value
            status::optimal,
            status::infeasible,
            status::unbounded,
            status::time_limit,
            status::numerical_failure,
            status::interrupted>;

    status_variant _status;

    status_variant _get_status() {
        using namespace status;
        int status_;
        check(COPT->GetIntAttr(prob, COPT_INTATTR_LPSTATUS, &status_));
        switch(status_) {
            case COPT_LPSTATUS_OPTIMAL: return optimal{};
            case COPT_LPSTATUS_INFEASIBLE: return infeasible{};
            case COPT_LPSTATUS_UNBOUNDED: return unbounded{};
            case COPT_LPSTATUS_TIMEOUT: return time_limit{};
            // case COPT_LPSTATUS_ITERLIMIT: return iteration_limit{};
            case COPT_LPSTATUS_NUMERICAL:
            case COPT_LPSTATUS_IMPRECISE:
            case COPT_LPSTATUS_UNFINISHED: return numerical_failure{};
            case COPT_LPSTATUS_INTERRUPTED: return interrupted{};
            case COPT_LPSTATUS_UNSTARTED: return unknown{};
            default:
                return unknown{};
        }
    }
    // clang-format on
public:
    const status_variant & solve_status() const { return _status; }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////////// Solve //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void solve() {
        check(COPT->SolveLp(prob));
        _status = _get_status();
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
            std::make_unique_for_overwrite<double[]>(num_variables());
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
};

}  // namespace copt::v7_2
}  // namespace mippp
