#pragma once

#include <numeric>
#include <optional>
#include <vector>

#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/cplex/v22_1_2/cplex_base.hpp"

namespace mippp {
namespace cplex::v22_1_2 {

class cplex_lp : public cplex_base {
public:
    [[nodiscard]] explicit cplex_lp(const cplex_api & api) : cplex_base(api) {}

    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////// Limits //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void set_time_limit(std::size_t count) {
        check(CPX.setintparam(env, CPXPARAM_Simplex_Limits_Iterations,
                              static_cast<int>(count)));
    }
    auto get_time_limit() {
        int count;
        check(CPX.getintparam(env, CPXPARAM_Simplex_Limits_Iterations, &count));
        return static_cast<std::size_t>(count);
    }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////// Tolerance parameters ///////////////////////////
    ///////////////////////////////////////////////////////////////////////////
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
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////// Solve status ///////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    // clang-format off
private:
    using status_variant = std::variant<
            status::unknown, //  default value
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

    status_variant _status;
    
    status_variant _get_status() {
        using namespace status;
        switch(CPX.getstat(env, lp)) {
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
            // unimplemented limits for now
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
    const status_variant & solve_status() const { return _status; }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////////// Solve //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void solve() {
        check(CPX.primopt(env, lp));
        _status = _get_status();
    }
    void refine_lp_status() {
        if(!is<status::infeasible_or_unbounded>(_status)) return;
        int tmp_advance_param, tmp_reduce_param;
        check(CPX.getintparam(env, CPXPARAM_Advance, &tmp_advance_param));
        check(CPX.getintparam(env, CPXPARAM_Preprocessing_Reduce,
                              &tmp_reduce_param));
        check(CPX.setintparam(env, CPXPARAM_Advance, 0));
        check(CPX.setintparam(env, CPXPARAM_Preprocessing_Reduce,
                              CPX_PREREDUCE_NOPRIMALORDUAL));
        check(CPX.primopt(env, lp));
        _status = _get_status();
        check(CPX.setintparam(env, CPXPARAM_Advance, tmp_advance_param));
        check(CPX.setintparam(env, CPXPARAM_Preprocessing_Reduce,
                              tmp_reduce_param));
        if(is<status::infeasible_or_unbounded>(_status))
            throw std::runtime_error("Failed to refine LP status.");
    }
    double get_solution_value() {
        double val;
        check(CPX.solution(env, lp, nullptr, &val, nullptr, nullptr, nullptr,
                           nullptr));
        return val;
    }
    auto get_solution() {
        auto solution =
            std::make_unique_for_overwrite<double[]>(num_variables());
        check(CPX.solution(env, lp, nullptr, nullptr, solution.get(), nullptr,
                           nullptr, nullptr));
        return variable_mapping(
            [this, solution = std::move(solution)](const variable & v) {
                return *(solution.get() + _native_id(v));
            });
    }
    auto get_dual_solution() {
        auto dual_solution =
            std::make_unique_for_overwrite<double[]>(num_constraints());
        check(CPX.solution(env, lp, nullptr, nullptr, nullptr,
                           dual_solution.get(), nullptr, nullptr));
        return constraint_mapping(std::move(dual_solution));
    }
    auto get_reduced_costs() {
        auto reduced_costs =
            std::make_unique_for_overwrite<double[]>(num_variables());
        check(CPX.solution(env, lp, nullptr, nullptr, nullptr, nullptr, nullptr,
                           reduced_costs.get()));
        return variable_mapping([this, reduced_costs = std::move(
                                           reduced_costs)](const variable & v) {
            return *(reduced_costs.get() + _native_id(v));
        });
    }
};

}  // namespace cplex::v22_1_2
}  // namespace mippp
