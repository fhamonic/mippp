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
private:
    int opt_lp_status;

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
        check(CPX.primopt(env, lp));
        opt_lp_status = CPX.getstat(env, lp);
    }

private:
    void _refine_lp_status() {
        int tmp_advance_param, tmp_reduce_param;
        check(CPX.getintparam(env, CPXPARAM_Advance, &tmp_advance_param));
        check(CPX.getintparam(env, CPXPARAM_Preprocessing_Reduce,
                              &tmp_reduce_param));
        check(CPX.setintparam(env, CPXPARAM_Advance, 0));
        check(CPX.setintparam(env, CPXPARAM_Preprocessing_Reduce,
                              CPX_PREREDUCE_NOPRIMALORDUAL));
        check(CPX.primopt(env, lp));
        opt_lp_status = CPX.getstat(env, lp);
        check(CPX.setintparam(env, CPXPARAM_Advance, tmp_advance_param));
        check(CPX.setintparam(env, CPXPARAM_Preprocessing_Reduce,
                              tmp_reduce_param));
    }

public:
    bool proven_optimal() {
        return opt_lp_status == CPX_STAT_OPTIMAL ||
               opt_lp_status == CPX_STAT_OPTIMAL_INFEAS ||
               opt_lp_status == CPX_STAT_OPTIMAL_FACE_UNBOUNDED;
    }
    bool proven_infeasible() {
        if(opt_lp_status == CPX_STAT_INForUNBD) _refine_lp_status();
        return opt_lp_status == CPX_STAT_INFEASIBLE;
    }
    bool proven_unbounded() {
        if(opt_lp_status == CPX_STAT_INForUNBD) _refine_lp_status();
        return opt_lp_status == CPX_STAT_UNBOUNDED;
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
    auto get_dual_solution() {
        auto dual_solution =
            std::make_unique_for_overwrite<double[]>(num_constraints());
        check(CPX.solution(env, lp, NULL, NULL, NULL, dual_solution.get(), NULL,
                           NULL));
        return constraint_mapping(std::move(dual_solution));
    }
};

}  // namespace cplex::v22_12
}  // namespace fhamonic::mippp

#endif  // MIPPP_CPLEX_v22_12_LP_HPP