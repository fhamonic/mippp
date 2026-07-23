#pragma once

#include "mippp/model_concepts.hpp"

#include "mippp/solvers/gurobi/v12_0/gurobi_base.hpp"

namespace mippp {
namespace gurobi::v12_0 {

class gurobi_lp : public gurobi_base {
public:
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

    status_variant _status;

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
            case GRB_USER_OBJ_LIMIT:
            case GRB_WORK_LIMIT:      return limit_reached{sol_available};
            case GRB_SUBOPTIMAL:      return failed{sol_available};
            case GRB_NUMERIC:         return numerical_failure{sol_available};
            case GRB_INTERRUPTED:     return interrupted{sol_available};
            default:
                return unknown{sol_available}; 
        }// TODO: what for GRB_CUTOFF ?
    }
    // clang-format on
public:
    const status_variant & solve_status() const { return _status; }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////////// Solve //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void solve() {
        check(GRB->optimize(model));
        _status = _get_status();
    }
    void refine_lp_status() {
        if(!is<status::infeasible_or_unbounded>(_status)) return;
        int tmp_dual_reductions;
        check(GRB->getintparam(env, GRB_INT_PAR_DUALREDUCTIONS,
                              &tmp_dual_reductions));
        check(GRB->setintparam(env, GRB_INT_PAR_DUALREDUCTIONS, 0));
        check(GRB->optimize(model));
        _status = _get_status();
        check(GRB->setintparam(env, GRB_INT_PAR_DUALREDUCTIONS,
                              tmp_dual_reductions));
        if(is<status::infeasible_or_unbounded>(_status))
            throw std::runtime_error("Failed to refine LP status.");
    }
    double get_solution_value() {
        double value;
        check(GRB->getdblattr(model, GRB_DBL_ATTR_OBJVAL, &value));
        return value;
    }
    auto get_solution() {
        auto solution =
            std::make_unique_for_overwrite<double[]>(_num_var_native_ids);
        check(GRB->getdblattrarray(model, GRB_DBL_ATTR_X, 0,
                                  static_cast<int>(_num_var_native_ids),
                                  solution.get()));
        return variable_mapping(
            [this, solution = std::move(solution)](const variable & v) {
                return *(solution.get() + _native_id(v));
            });
    }
    auto get_dual_solution() {
        auto num_constrs = num_constraints();
        auto solution = std::make_unique_for_overwrite<double[]>(num_constrs);
        check(GRB->getdblattrarray(model, GRB_DBL_ATTR_PI, 0,
                                  static_cast<int>(num_constrs),
                                  solution.get()));
        return constraint_mapping(std::move(solution));
    }
    auto get_reduced_costs() {
        auto reduced_costs =
            std::make_unique_for_overwrite<double[]>(_num_var_native_ids);
        check(GRB->getdblattrarray(model, GRB_DBL_ATTR_RC, 0,
                                  static_cast<int>(_num_var_native_ids),
                                  reduced_costs.get()));
        return variable_mapping([this, reduced_costs = std::move(
                                           reduced_costs)](const variable & v) {
            return *(reduced_costs.get() + _native_id(v));
        });
    }
};

}  // namespace gurobi::v12_0
}  // namespace mippp
