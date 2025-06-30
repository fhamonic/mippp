#ifndef MIPPP_HIGHS_v1_10_QP_HPP
#define MIPPP_HIGHS_v1_10_QP_HPP

#include <iostream>
#include <map>
#include <optional>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"
#include "mippp/quadratic_expression.hpp"

#include "mippp/solvers/highs/v1_10/highs_base.hpp"

namespace fhamonic::mippp {
namespace highs::v1_10 {

class highs_qp : public highs_base {
private:
    int qp_status;

public:
    [[nodiscard]] explicit highs_qp(const highs_api & api) : highs_base(api) {}

    struct variables_pair_cmp {
        bool operator()(auto && p1, auto && p2) const {
            auto [p1v1, p1v2] = p1;
            if(p1v2 < p1v1) std::swap(p1v1, p1v2);
            auto [p2v1, p2v2] = p2;
            if(p2v2 < p2v1) std::swap(p2v1, p2v2);
            if(p1v1 == p2v1) return p1v2 < p2v2;
            return p1v1 < p2v1;
        }
    };

    template <linear_expression LE>
    void set_objective(LE && le) {
        const auto num_vars = num_variables();
        tmp_scalars.resize(num_vars);
        std::fill(tmp_scalars.begin(), tmp_scalars.end(), 0.0);
        for(auto && [var, coef] : le.linear_terms()) {
            tmp_scalars[var.uid()] += coef;
        }
        check(Highs.changeColsCostByRange(
            model, 0, static_cast<HighsInt>(num_vars) - 1, tmp_scalars.data()));
        set_objective_offset(le.constant());
    }
    template <quadratic_expression QE>
    void set_objective(QE && qe) {
        const auto num_vars = num_variables();
        set_objective(qe.linear_expression());
        std::map<std::pair<variable, variable>, scalar, variables_pair_cmp>
            factorized_terms;
        for(auto && [var1, var2, coef] : qe.quadratic_terms()) {
            factorized_terms[std::make_pair(var1, var2)] += coef;
        }
        tmp_begins.resize(num_vars);
        tmp_indices.resize(0u);
        tmp_scalars.resize(0u);
        HighsInt next_row = 0;
        for(auto && [vars_pair, coef] : factorized_terms) {
            auto && [var1, var2] = vars_pair;
            while(next_row <= var1.id())
                tmp_begins[static_cast<std::size_t>(next_row++)] =
                    static_cast<HighsInt>(tmp_scalars.size());
            tmp_indices.emplace_back(var2.id());
            tmp_scalars.emplace_back(2 * coef);
        }
        while(next_row <= static_cast<int>(num_vars))
            tmp_begins[static_cast<std::size_t>(next_row++)] =
                static_cast<HighsInt>(tmp_scalars.size());
        check(Highs.passHessian(
            model, static_cast<int>(num_vars),
            static_cast<int>(tmp_scalars.size()), kHighsHessianFormatTriangular,
            tmp_begins.data(), tmp_indices.data(), tmp_scalars.data()));
    }

    void solve() {
        if(num_variables() == 0u) {
            add_variable();
        }
        check(Highs.run(model));
        qp_status = Highs.getModelStatus(model);
        check_model_status(qp_status);
    }

    // private:
    //     void _refine_qp_status() {
    //         char tmp_presolve[4];
    //         check(Highs.getHighsStringOptionValue(model, "presolve",
    //         tmp_presolve)); check(Highs.setHighsStringOptionValue(model,
    //         "presolve", "off")); check(Highs.run(model)); qp_status =
    //         Highs.getModelStatus(model);
    //         check(Highs.setHighsStringOptionValue(model, "presolve",
    //         tmp_presolve)); check_model_status(qp_status);
    //     }

    // public:
    bool is_optimal() {
        return qp_status == kHighsModelStatusModelEmpty ||
               qp_status == kHighsModelStatusOptimal;
    }
    bool is_infeasible() {
        // if(qp_status == kHighsModelStatusUnboundedOrInfeasible)
        //     _refine_qp_status();
        return qp_status == kHighsModelStatusInfeasible;
    }
    bool is_unbounded() {
        // if(qp_status == kHighsModelStatusUnboundedOrInfeasible)
        //     _refine_qp_status();
        return qp_status == kHighsModelStatusUnbounded;
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

#endif  // MIPPP_HIGHS_v1_10_QP_HPP