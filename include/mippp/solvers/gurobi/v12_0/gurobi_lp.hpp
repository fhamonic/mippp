#ifndef MIPPP_GUROBI_v12_0_LP_HPP
#define MIPPP_GUROBI_v12_0_LP_HPP

#include <optional>

#include "mippp/model_concepts.hpp"

#include "mippp/solvers/gurobi/v12_0/gurobi_base.hpp"

namespace fhamonic::mippp {
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
    bool is_optimal() { return lp_status == GRB_OPTIMAL; }
    bool is_infeasible() {
        if(lp_status == GRB_INF_OR_UNBD) _refine_lp_status();
        return lp_status == GRB_INFEASIBLE;
    }
    bool is_unbounded() {
        if(lp_status == GRB_INF_OR_UNBD) _refine_lp_status();
        return lp_status == GRB_UNBOUNDED;
    }

    double get_solution_value() {
        double value;
        check(GRB.getdblattr(model, GRB_DBL_ATTR_OBJVAL, &value));
        return value;
    }

    auto get_solution() {
        auto num_vars = num_variables();
        auto solution = std::make_unique_for_overwrite<double[]>(num_vars);
        check(GRB.getdblattrarray(model, GRB_DBL_ATTR_X, 0,
                                  static_cast<int>(num_vars), solution.get()));
        return variable_mapping(std::move(solution));
    }
    auto get_dual_solution() {
        auto num_constrs = num_constraints();
        auto solution = std::make_unique_for_overwrite<double[]>(num_constrs);
        check(GRB.getdblattrarray(model, GRB_DBL_ATTR_PI, 0,
                                  static_cast<int>(num_constrs),
                                  solution.get()));
        return constraint_mapping(std::move(solution));
    }

    // void set_basic(variable v);
    // void set_non_basic(variable v);

    // void set_basic(constraint v);
    // void set_non_basic(constraint v);
};

}  // namespace gurobi::v12_0
}  // namespace fhamonic::mippp

#endif  // MIPPP_GUROBI_v12_0_LP_HPP