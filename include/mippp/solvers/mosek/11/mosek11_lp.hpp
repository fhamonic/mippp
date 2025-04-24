#ifndef MIPPP_MOSEK_11_LP_HPP
#define MIPPP_MOSEK_11_LP_HPP

#include <optional>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/mosek/11/mosek11_base_model.hpp"

namespace fhamonic {
namespace mippp {

class mosek11_lp : public mosek11_base_model {
public:
    using variable_id = int;
    using constraint_id = int;
    using scalar = double;
    using variable = model_variable<variable_id, scalar>;
    using constraint = model_constraint<constraint_id, scalar>;

private:
    std::optional<lp_status> opt_lp_status;

public:
    [[nodiscard]] explicit mosek11_lp(const mosek11_api & api)
        : mosek11_base_model(api) {}

    void solve() {
        check(MSK.optimize(task));
    }

    double get_solution_value() {
        double val;
        check(MSK.getprimalobj(task, MSK_SOL_BAS, &val));
        return val;
    }
    auto get_solution() {
        auto solution =
            std::make_unique_for_overwrite<double[]>(num_variables());
        check(MSK.getxx(task, MSK_SOL_BAS, solution.get()));
        return variable_mapping(std::move(solution));
    }
    auto get_dual_solution() {
        auto solution =
            std::make_unique_for_overwrite<double[]>(num_constraints());
        check(MSK.getsolution(task, MSK_SOL_BAS, NULL, NULL, NULL, NULL, NULL,
                              NULL, NULL, solution.get(), NULL, NULL, NULL,
                              NULL, NULL));
        return constraint_mapping(std::move(solution));
    }
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_MOSEK_11_LP_HPP