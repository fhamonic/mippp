#ifndef MIPPP_GUROBI12_lp_HPP
#define MIPPP_GUROBI12_lp_HPP

#include <optional>

#include "mippp/model_concepts.hpp"

#include "mippp/solvers/gurobi/12/gurobi12_base_model.hpp"

namespace fhamonic {
namespace mippp {

class gurobi12_lp : public gurobi12_base_model {
public:
    using variable_id = int;
    using constraint_id = int;
    using scalar = double;
    using variable = model_variable<variable_id, scalar>;
    using constraint = model_constraint<constraint_id, scalar>;

public:
    [[nodiscard]] explicit gurobi12_lp(const gurobi12_api & api)
        : gurobi12_base_model(api) {}

public:
    void solve() {
        check(GRB.optimize(model));
        int status;
        check(GRB.getintattr(model, GRB_INT_ATTR_STATUS, &status));
        switch(status) {
            case GRB_OPTIMAL:
                opt_lp_status.emplace(lp_status::optimal);
                return;
            case GRB_INF_OR_UNBD:
                int dual_reductions;
                check(GRB.getintparam(env, GRB_INT_PAR_DUALREDUCTIONS,
                                      &dual_reductions));
                check(GRB.setintparam(env, GRB_INT_PAR_DUALREDUCTIONS, 0));
                check(GRB.optimize(model));
                check(GRB.getintattr(model, GRB_INT_ATTR_STATUS, &status));
                check(GRB.setintparam(env, GRB_INT_PAR_DUALREDUCTIONS,
                                      dual_reductions));
                switch(status) {
                    case GRB_INFEASIBLE:
                        opt_lp_status.emplace(lp_status::infeasible);
                        return;
                    case GRB_UNBOUNDED:
                        opt_lp_status.emplace(lp_status::unbounded);
                        return;
                    default:
                        throw std::runtime_error(
                            "gurobi12_base_model: Cannot determine if model is "
                            "infeasible or unbounded (status = " +
                            std::to_string(status) + ").");
                }
            default:
                opt_lp_status.reset();
        }
    }
    std::optional<lp_status> get_lp_status() { return opt_lp_status; }

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

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_GUROBI12_lp_HPP