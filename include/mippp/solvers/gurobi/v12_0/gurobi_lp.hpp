#pragma once

#include <optional>

#include "mippp/model_concepts.hpp"

#include "mippp/solvers/gurobi/v12_0/gurobi_base.hpp"

namespace mippp {
namespace gurobi::v12_0 {

class gurobi_lp : public gurobi_base {
private:
    int lp_status;

    static void check_model_status(int status) {
        if(status <= 5) return;
        throw std::runtime_error(
            "gurobi_lp: solve did not succeeded, status is one of "
            "{GRB_CUTOFF,GRB_ITERATION_LIMIT, GRB_NODE_LIMIT, "
            "GRB_TIME_LIMIT,GRB_SOLUTION_LIMIT, GRB_INTERRUPTED, "
            "GRB_NUMERIC, GRB_SUBOPTIMAL,GRB_INPROGRESS, "
            "GRB_USER_OBJ_LIMIT, GRB_WORK_LIMIT,GRB_MEM_LIMIT}.");
    }

public:
    [[nodiscard]] explicit gurobi_lp(const gurobi_api & api)
        : gurobi_base(api) {}

    void solve() {
        check(GRB.optimize(model));
        check(GRB.getintattr(model, GRB_INT_ATTR_STATUS, &lp_status));
        check_model_status(lp_status);
    }

private:
    void _refine_lp_status() {
        int tmp_dual_reductions;
        check(GRB.getintparam(env, GRB_INT_PAR_DUALREDUCTIONS,
                              &tmp_dual_reductions));
        check(GRB.setintparam(env, GRB_INT_PAR_DUALREDUCTIONS, 0));
        check(GRB.optimize(model));
        check(GRB.getintattr(model, GRB_INT_ATTR_STATUS, &lp_status));
        check(GRB.setintparam(env, GRB_INT_PAR_DUALREDUCTIONS,
                              tmp_dual_reductions));
        check_model_status(lp_status);
    }

public:
    bool proven_optimal() { return lp_status == GRB_OPTIMAL; }
    bool proven_infeasible() {
        if(lp_status == GRB_INF_OR_UNBD) _refine_lp_status();
        return lp_status == GRB_INFEASIBLE;
    }
    bool proven_unbounded() {
        if(lp_status == GRB_INF_OR_UNBD) _refine_lp_status();
        return lp_status == GRB_UNBOUNDED;
    }

    ///////////////////////////////// Limits //////////////////////////////////
    void set_iteration_limit(std::size_t n) {
        check(GRB.setdblparam(env, GRB_DBL_PAR_ITERATIONLIMIT,
                              static_cast<double>(n)));
    }
    std::size_t get_iteration_limit() {
        double n;
        check(GRB.getdblparam(env, GRB_DBL_PAR_ITERATIONLIMIT, &n));
        return static_cast<std::size_t>(n);
    }

    /////////////////////////// Termination reason ////////////////////////////
    // clang-format off
    using termination_reasons = std::variant<
            reason::optimal,
            reason::infeasible_or_unbounded,
            reason::infeasible,
            reason::unbounded,
            reason::interrupted,
            reason::failed,
            reason::numerical_failure,
            reason::limit_reached,
            reason::time_limit,
            reason::iteration_limit,
            reason::memory_limit,
            reason::unknown>;

    termination_reasons termination_reason() {
        using namespace reason;
        int status;
        check(GRB.getintattr(model, GRB_INT_ATTR_STATUS, &status));
        switch(status) {
            case GRB_OPTIMAL:         return optimal{};
            case GRB_INF_OR_UNBD:     return infeasible_or_unbounded{};
            case GRB_INFEASIBLE:      return infeasible{};
            case GRB_UNBOUNDED:       return unbounded{};
        }
        int sol_count;
        check(GRB.getintattr(model, GRB_INT_ATTR_SOLCOUNT, &sol_count));
        const bool sol_available = sol_count > 0;
        switch(status) {
            case GRB_INTERRUPTED:     return interrupted{sol_available};
            case GRB_SUBOPTIMAL:      return failed{sol_available};
            case GRB_NUMERIC:         return numerical_failure{sol_available};
            case GRB_TIME_LIMIT:      return time_limit{sol_available};
            case GRB_ITERATION_LIMIT: return iteration_limit{sol_available};
            case GRB_MEM_LIMIT:       return memory_limit{sol_available};
            case GRB_USER_OBJ_LIMIT:
            case GRB_WORK_LIMIT:      return limit_reached{sol_available};
            default:
                return unknown{sol_available}; 
        }// TODO: what for GRB_CUTOFF ?
    }
    // clang-format on

    //////////////////////////////// Solution /////////////////////////////////
    double get_solution_value() {
        double value;
        check(GRB.getdblattr(model, GRB_DBL_ATTR_OBJVAL, &value));
        return value;
    }
    auto get_solution() {
        auto solution =
            std::make_unique_for_overwrite<double[]>(_num_var_native_ids);
        check(GRB.getdblattrarray(model, GRB_DBL_ATTR_X, 0,
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
        check(GRB.getdblattrarray(model, GRB_DBL_ATTR_PI, 0,
                                  static_cast<int>(num_constrs),
                                  solution.get()));
        return constraint_mapping(std::move(solution));
    }
    auto get_reduced_costs() {
        auto reduced_costs =
            std::make_unique_for_overwrite<double[]>(_num_var_native_ids);
        check(GRB.getdblattrarray(model, GRB_DBL_ATTR_RC, 0,
                                  static_cast<int>(_num_var_native_ids),
                                  reduced_costs.get()));
        return variable_mapping([this, reduced_costs = std::move(
                                           reduced_costs)](const variable & v) {
            return *(reduced_costs.get() + _native_id(v));
        });
    }

    // void set_basic(variable v);
    // void set_non_basic(variable v);

    // void set_basic(constraint v);
    // void set_non_basic(constraint v);
};

}  // namespace gurobi::v12_0
}  // namespace mippp
