#pragma once

#include <numeric>
#include <optional>
#include <vector>

#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/xpress/v45_1/xpress_base.hpp"

namespace mippp {
namespace xpress::v45_1 {

class xpress_lp : public xpress_base {
public:
    [[nodiscard]] explicit xpress_lp(const xpress_api & api)
        : xpress_base(api) {}

    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////// Limits //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

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

    status_variant _status;

    status_variant _get_status() {
        using namespace status;
        int stop_status;
        check(XPRS.getintattrib(prob, XPRS_STOPSTATUS, &stop_status));    
        switch(stop_status) {
            case XPRS_STOP_NONE:
            case XPRS_STOP_SOLVECOMPLETE: {
                int lp_status;
                check(XPRS.getintattrib(prob, XPRS_LPSTATUS, &lp_status));    
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
    const status_variant & solve_status() const { return _status; }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////////// Solve //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void solve() {
        check(XPRS.lpoptimize(prob, nullptr));
        _status = _get_status();
    }
    double get_solution_value() {
        double val;
        check(XPRS.getdblattrib(prob, XPRS_LPOBJVAL, &val));
        return objective_offset + val;
    }
    auto get_solution() {
        const auto num_vars = num_variables();
        auto solution = std::make_unique_for_overwrite<double[]>(num_vars);
        check(XPRS.getsolution(prob, nullptr, solution.get(), 0,
                               static_cast<int>(num_vars) - 1));
        return variable_mapping(std::move(solution));
    }
    auto get_dual_solution() {
        const auto num_constrs = num_constraints();
        auto dual_solution =
            std::make_unique_for_overwrite<double[]>(num_constrs);
        check(XPRS.getduals(prob, nullptr, dual_solution.get(), 0,
                            static_cast<int>(num_constrs) - 1));
        return constraint_mapping(std::move(dual_solution));
    }
    auto get_reduced_costs() {
        const auto num_vars = num_variables();
        auto reduced_costs = std::make_unique_for_overwrite<double[]>(num_vars);
        check(XPRS.getredcosts(prob, nullptr, reduced_costs.get(), 0,
                               static_cast<int>(num_vars) - 1));
        return variable_mapping(std::move(reduced_costs));
    }
};

}  // namespace xpress::v45_1
}  // namespace mippp
