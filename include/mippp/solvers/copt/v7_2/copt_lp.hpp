#ifndef MIPPP_COPT_v7_2_LP_HPP
#define MIPPP_COPT_v7_2_LP_HPP

#include <optional>

#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/copt/v7_2/copt_base.hpp"

namespace fhamonic::mippp {
namespace copt::v7_2 {

class copt_lp : public copt_base {
private:
    std::optional<lp_status> opt_lp_status;

public:
    [[nodiscard]] explicit copt_lp(const copt_api & api) : copt_base(api) {}

    void solve() { check(COPT.SolveLp(prob)); }

    double get_solution_value() {
        double val;
        check(COPT.GetDblAttr(prob, COPT_DBLATTR_LPOBJVAL, &val));
        return val;
    }
    auto get_solution() {
        auto solution =
            std::make_unique_for_overwrite<double[]>(num_variables());
        check(COPT.GetLpSolution(prob, solution.get(), NULL, NULL, NULL));
        return variable_mapping(std::move(solution));
    }
    auto get_dual_solution() {
        auto solution =
            std::make_unique_for_overwrite<double[]>(num_variables());
        check(COPT.GetLpSolution(prob, NULL, NULL, solution.get(), NULL));
        return constraint_mapping(std::move(solution));
    }
};

}  // namespace copt::v7_2
}  // namespace fhamonic::mippp

#endif  // MIPPP_COPT_v7_2_LP_HPP