#ifndef MIPPP_MOSEK_v11_LP_HPP
#define MIPPP_MOSEK_v11_LP_HPP

#include <iostream>
#include <optional>

#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/mosek/v11/mosek_base.hpp"

namespace fhamonic::mippp {
namespace mosek::v11 {

class mosek_lp : public mosek_base {
private:
    MSKprostae lp_status;

public:
    [[nodiscard]] explicit mosek_lp(const mosek_api & api) : mosek_base(api) {}

    void solve() {
        check(MSK.optimize(task));
        check(MSK.getprosta(task, MSK_SOL_BAS, &lp_status));
        if(lp_status == MSK_PRO_STA_UNKNOWN)
            throw std::runtime_error("Mosek : problem status unknown.");
        if(lp_status == MSK_PRO_STA_ILL_POSED)
            throw std::runtime_error("Mosek : problem ill posed.");
    }

    bool proven_optimal() { return lp_status == MSK_PRO_STA_PRIM_AND_DUAL_FEAS; }
    bool proven_infeasible() {
        return lp_status == MSK_PRO_STA_PRIM_INFEAS ||
               lp_status == MSK_PRO_STA_PRIM_AND_DUAL_INFEAS;
    }
    bool proven_unbounded() { return lp_status == MSK_PRO_STA_DUAL_INFEAS; }

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
        auto dual_solution =
            std::make_unique_for_overwrite<double[]>(num_constraints());
        check(MSK.getsolution(task, MSK_SOL_BAS, NULL, NULL, NULL, NULL, NULL,
                              NULL, NULL, dual_solution.get(), NULL, NULL, NULL,
                              NULL, NULL));
        return constraint_mapping(std::move(dual_solution));
    }
};

}  // namespace mosek::v11
}  // namespace fhamonic::mippp

#endif  // MIPPP_MOSEK_v11_LP_HPP