#ifndef MIPPP_HIGHS_v1_10_LP_HPP
#define MIPPP_HIGHS_v1_10_LP_HPP

#include <optional>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/highs/v1_10/highs_base.hpp"

namespace fhamonic::mippp {
namespace highs::v1_10 {

class highs_lp : public highs_base {
private:
    int lp_status;

public:
    [[nodiscard]] explicit highs_lp(const highs_api & api) : highs_base(api) {}

    void solve() {
        if(num_variables() == 0u) {
            lp_status = kHighsModelStatusModelEmpty;
            return;
        }
        check(Highs.run(model));
        lp_status = Highs.getModelStatus(model);
        check_model_status(lp_status);
    }

    // private:
    //     void _refine_lp_status() {
    //         char tmp_presolve[4];
    //         check(Highs.getHighsStringOptionValue(model, "presolve",
    //         tmp_presolve)); check(Highs.setHighsStringOptionValue(model,
    //         "presolve", "off")); check(Highs.run(model)); lp_status =
    //         Highs.getModelStatus(model);
    //         check(Highs.setHighsStringOptionValue(model, "presolve",
    //         tmp_presolve)); check_model_status(lp_status);
    //     }

    // public:
    bool is_optimal() {
        return lp_status == kHighsModelStatusModelEmpty ||
               lp_status == kHighsModelStatusOptimal;
    }
    bool is_infeasible() {
        // if(lp_status == kHighsModelStatusUnboundedOrInfeasible)
        //     _refine_lp_status();
        return lp_status == kHighsModelStatusInfeasible;
    }
    bool is_unbounded() {
        // if(lp_status == kHighsModelStatusUnboundedOrInfeasible)
        //     _refine_lp_status();
        return lp_status == kHighsModelStatusUnbounded;
    }

    double get_solution_value() { return Highs.getObjectiveValue(model); }
    auto get_solution() {
        auto num_vars = num_variables();
        auto solution = std::make_unique_for_overwrite<double[]>(num_vars);
        check(Highs.getSolution(model, solution.get(), NULL, NULL, NULL));
        return variable_mapping(std::move(solution));
    }
    auto get_dual_solution() {
        auto num_constrs = num_constraints();
        auto solution = std::make_unique_for_overwrite<double[]>(num_constrs);
        check(Highs.getSolution(model, NULL, NULL, NULL, solution.get()));
        return constraint_mapping(std::move(solution));
    }
};

}  // namespace highs::v1_10
}  // namespace fhamonic::mippp

#endif  // MIPPP_HIGHS_v1_10_LP_HPP