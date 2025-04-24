#ifndef MIPPP_CPLEX_22_LP_HPP
#define MIPPP_CPLEX_22_LP_HPP

#include <numeric>
#include <optional>
#include <vector>

#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/cplex/22/cplex22_base_model.hpp"

namespace fhamonic {
namespace mippp {

class cplex22_lp : public cplex22_base_model {
public:
    using variable_id = int;
    using constraint_id = int;
    using scalar = double;
    using variable = model_variable<variable_id, scalar>;
    using constraint = model_constraint<constraint_id, scalar>;

    // std::optional<lp_status> opt_lp_status;

public:
    [[nodiscard]] explicit cplex22_lp(const cplex22_api & api)
        : cplex22_base_model(api) {}

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

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_CPLEX_22_LP_HPP