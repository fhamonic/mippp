#ifndef MIPPP_HIGHS_v1_10_LP_HPP
#define MIPPP_HIGHS_v1_10_LP_HPP

#include <optional>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/highs/v1_10/highs_base_model.hpp"

namespace fhamonic::mippp {
namespace highs::v1_10 {

class highs_lp : public highs_base_model {
private:
    std::optional<lp_status> opt_lp_status;

public:
    [[nodiscard]] explicit highs_lp(const highs_api & api)
        : highs_base_model(api) {}

    void solve() {
        check(Highs.run(model));
        switch(Highs.getModelStatus(model)) {
            case kHighsModelStatusModelEmpty:
            case kHighsModelStatusOptimal:
                opt_lp_status.emplace(lp_status::optimal);
                return;
            // case kHighsModelStatusUnboundedOrInfeasible:
            case kHighsModelStatusInfeasible:
                opt_lp_status.emplace(lp_status::infeasible);
                return;
            case kHighsModelStatusUnbounded:
                opt_lp_status.emplace(lp_status::unbounded);
                return;
            default:
                throw std::runtime_error("highs_lp: error");
        }
    }
    std::optional<lp_status> get_lp_status() { return opt_lp_status; }

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