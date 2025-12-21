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
    int lp_status;

public:
    [[nodiscard]] explicit copt_lp(const copt_api & api) : copt_base(api) {}

    void solve() {
        check(COPT.SolveLp(prob));
        check(COPT.GetIntAttr(prob, COPT_INTATTR_LPSTATUS, &lp_status));
        check_lp_status(lp_status);
    }

    bool proven_optimal() { return lp_status == COPT_LPSTATUS_OPTIMAL; }
    bool proven_infeasible() { return lp_status == COPT_LPSTATUS_INFEASIBLE; }
    bool proven_unbounded() { return lp_status == COPT_LPSTATUS_UNBOUNDED; }

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