#ifndef MIPPP_HIGHS_LP_MODEL_HPP
#define MIPPP_HIGHS_LP_MODEL_HPP

#include <limits>
// #include <ranges>
#include <optional>
#include <vector>

#include <range/v3/view/iota.hpp>
#include <range/v3/view/span.hpp>
#include <range/v3/view/zip.hpp>

#include "mippp/detail/function_traits.hpp"
#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_variable.hpp"

#include "mippp/api/highs_api.hpp"

namespace fhamonic {
namespace mippp {

class highs_lp_model {
private:
    const highs_api & Highs;
    void * model;
    std::optional<lp_status> opt_lp_status;
    std::vector<int> tmp_variables;
    std::vector<double> tmp_scalars;

public:
    using variable_id = int;
    using scalar = double;
    using variable = model_variable<variable_id, scalar>;
    using constraint = int;

    struct variable_params {
        scalar obj_coef = scalar{0};
        scalar lower_bound = scalar{0};
        scalar upper_bound = std::numeric_limits<scalar>::infinity();
    };

public:
    [[nodiscard]] explicit highs_lp_model(const highs_api & api)
        : Highs(api), model(Highs.create()) {}
    ~highs_lp_model() { Highs.destroy(model); }

private:
    void check(int status) {
        if(status == kHighsStatusError)
            throw std::runtime_error("highs_lp_model: error");
    }

public:
    std::size_t num_variables() {
        return static_cast<std::size_t>(Highs.getNumCol(model));
    }
    std::size_t num_constraints() {
        return static_cast<std::size_t>(Highs.getNumRow(model));
    }
    std::size_t num_entries() {
        return static_cast<std::size_t>(Highs.getNumNz(model));
    }

    void set_maximization() {
        check(Highs.changeObjectiveSense(model, kHighsObjSenseMaximize));
    }
    void set_minimization() {
        check(Highs.changeObjectiveSense(model, kHighsObjSenseMinimize));
    }

    void set_objective_offset(double offset) {
        check(Highs.changeObjectiveOffset(model, offset));
    }
    void set_objective(linear_expression auto && le) {
        auto num_vars = num_variables();
        tmp_scalars.resize(num_vars);
        std::fill(tmp_scalars.begin(), tmp_scalars.end(), 0.0);
        for(auto && [var, coef] : le.linear_terms()) {
            tmp_scalars[static_cast<std::size_t>(var)] = coef;
        }
        check(Highs.changeColsCostByRange(
            model, 0, static_cast<int>(num_vars) - 1, tmp_scalars.data()));
        set_objective_offset(le.constant());
    }
    double get_objective_offset() {
        double offset;
        check(Highs.getObjectiveOffset(model, &offset));
        return offset;
    }

    variable add_variable(
        const variable_params p = {
            .obj_coef = 0,
            .lower_bound = 0,
            .upper_bound = std::numeric_limits<double>::infinity()}) {
        int var_id = static_cast<int>(num_variables());
        check(Highs.addCol(model, p.obj_coef, p.lower_bound, p.upper_bound, 0,
                           NULL, NULL));
        return variable(var_id);
    }
    // variables_range add_variables(std::size_t count,
    //                               const variable_params p);

    constraint add_constraint(linear_constraint auto && lc) {
        int constr_id = static_cast<int>(num_constraints());
        tmp_variables.resize(0);
        tmp_scalars.resize(0);
        for(auto && [var, coef] : lc.expression().linear_terms()) {
            tmp_variables.emplace_back(var);
            tmp_scalars.emplace_back(coef);
        }
        const double b = -lc.expression().constant();
        check(Highs.addRow(
            model,
            (lc.relation() == constraint_relation::less_equal_zero)
                ? -Highs.getInfinity(model)
                : b,
            (lc.relation() == constraint_relation::greater_equal_zero)
                ? Highs.getInfinity(model)
                : b,
            static_cast<int>(tmp_variables.size()), tmp_variables.data(),
            tmp_scalars.data()));
        return constr_id;
    }

    void optimize() {
        // if(num_variables() == 0u) {
        //     opt_lp_status.emplace(lp_status::optimal);
        //     return;
        // }
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
                throw std::runtime_error("highs_lp_model: error");
        }

        // const HighsInt kHighsModelStatusUnboundedOrInfeasible = 9;
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
        return variable_mapping(std::move(solution));
    }
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_HIGHS_LP_MODEL_HPP