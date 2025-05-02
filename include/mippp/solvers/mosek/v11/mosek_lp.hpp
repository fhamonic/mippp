#ifndef MIPPP_MOSEK_v11_LP_HPP
#define MIPPP_MOSEK_v11_LP_HPP

#include <optional>

#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/mosek/v11/mosek_base_model.hpp"

namespace fhamonic::mippp {
namespace mosek::v11 {

class mosek_lp : public mosek_base_model {
private:
    std::optional<lp_status> opt_lp_status;

public:
    [[nodiscard]] explicit mosek_lp(const mosek_api & api)
        : mosek_base_model(api) {}

    void solve() { check(MSK.optimize(task)); }

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

}  // namespace mosek::v11
}  // namespace fhamonic::mippp

#endif  // MIPPP_MOSEK_v11_LP_HPP