#ifndef MIPPP_XPRESS_v45_1_LP_HPP
#define MIPPP_XPRESS_v45_1_LP_HPP

#include <numeric>
#include <optional>
#include <vector>

#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/xpress/v45_1/xpress_base.hpp"

namespace fhamonic::mippp {
namespace xpress::v45_1 {

class xpress_lp : public xpress_base {
private:
    int lp_status;

public:
    [[nodiscard]] explicit xpress_lp(const xpress_api & api)
        : xpress_base(api) {}

    // void set_optimality_tolerance(double tol) {
    //     check(
    //         CPX.setdblparam(env, CPXPARAM_Simplex_Tolerances_Optimality,
    //         tol));
    // }
    // double get_optimality_tolerance() {
    //     double tol;
    //     check(
    //         CPX.getdblparam(env, CPXPARAM_Simplex_Tolerances_Optimality,
    //         &tol));
    //     return tol;
    // }

    // void set_feasibility_tolerance(double tol) {
    //     check(
    //         CPX.setdblparam(env, CPXPARAM_Simplex_Tolerances_Feasibility,
    //         tol));
    // }
    // double get_feasibility_tolerance() {
    //     double tol;
    //     check(CPX.getdblparam(env, CPXPARAM_Simplex_Tolerances_Feasibility,
    //                           &tol));
    //     return tol;
    // }

    void solve() {
        check(XPRS.lpoptimize(prob, NULL));
        check(XPRS.getintattrib(prob, XPRS_LPSTATUS, &lp_status));
    }

    bool proven_optimal() { return lp_status == XPRS_LP_OPTIMAL; }
    bool proven_infeasible() { return lp_status == XPRS_LP_INFEAS; }
    bool proven_unbounded() { return lp_status == XPRS_LP_UNBOUNDED; }

    double get_solution_value() {
        double val;
        check(XPRS.getdblattrib(prob, XPRS_LPOBJVAL, &val));
        return objective_offset + val;
    }
    auto get_solution() {
        const auto num_vars = num_variables();
        auto solution = std::make_unique_for_overwrite<double[]>(num_vars);
        check(XPRS.getsolution(prob, NULL, solution.get(), 0,
                               static_cast<int>(num_vars) - 1));
        return variable_mapping(std::move(solution));
    }
    auto get_dual_solution() {
        const auto num_constrs = num_constraints();
        auto dual_solution =
            std::make_unique_for_overwrite<double[]>(num_constrs);
        check(XPRS.getduals(prob, NULL, dual_solution.get(), 0,
                            static_cast<int>(num_constrs) - 1));
        return constraint_mapping(std::move(dual_solution));
    }
};

}  // namespace xpress::v45_1
}  // namespace fhamonic::mippp

#endif  // MIPPP_XPRESS_v45_1_LP_HPP