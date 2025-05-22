#ifndef MIPPP_CPLEX_v22_12_LP_HPP
#define MIPPP_CPLEX_v22_12_LP_HPP

#include <numeric>
#include <optional>
#include <vector>

#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/cplex/v22_12/cplex_base.hpp"

namespace fhamonic::mippp {
namespace cplex::v22_12 {

class cplex_lp : public cplex_base {
public:
    [[nodiscard]] explicit cplex_lp(const cplex_api & api) : cplex_base(api) {}

    void set_optimality_tolerance(double tol) {
        check(
            CPX.setdblparam(env, CPXPARAM_Simplex_Tolerances_Optimality, tol));
    }
    double get_optimality_tolerance() {
        double tol;
        check(
            CPX.getdblparam(env, CPXPARAM_Simplex_Tolerances_Optimality, &tol));
        return tol;
    }

    void set_feasibility_tolerance(double tol) {
        check(
            CPX.setdblparam(env, CPXPARAM_Simplex_Tolerances_Feasibility, tol));
    }
    double get_feasibility_tolerance() {
        double tol;
        check(CPX.getdblparam(env, CPXPARAM_Simplex_Tolerances_Feasibility,
                              &tol));
        return tol;
    }

    void solve() {
        // if(num_variables() == 0u) {
        //     opt_lp_status.emplace(lp_status::optimal);
        //     return;
        // }
        check(CPX.primopt(env, lp));
    }

    double get_solution_value() {
        double val;
        check(CPX.solution(env, lp, NULL, &val, NULL, NULL, NULL, NULL));
        return val;
    }
    auto get_solution() {
        auto solution =
            std::make_unique_for_overwrite<double[]>(num_variables());
        check(CPX.solution(env, lp, NULL, NULL, solution.get(), NULL, NULL,
                           NULL));
        return variable_mapping(std::move(solution));
    }
    // auto get_dual_solution() {
    //     auto solution =
    //         std::make_unique_for_overwrite<double[]>(num_constraints());
    //     check(CPX.getsolution(task, CPX_SOL_BAS, NULL, NULL, NULL, NULL,
    //     NULL,
    //                           NULL, NULL, solution.get(), NULL, NULL, NULL,
    //                           NULL, NULL));
    //     return constraint_mapping(std::move(solution));
    // }
};

}  // namespace cplex::v22_12
}  // namespace fhamonic::mippp

#endif  // MIPPP_CPLEX_v22_12_LP_HPP