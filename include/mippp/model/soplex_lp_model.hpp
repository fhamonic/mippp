#ifndef MIPPP_SOPLEX_LP_MODEL_HPP
#define MIPPP_SOPLEX_LP_MODEL_HPP

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

#include "mippp/api/soplex_api.hpp"

namespace fhamonic {
namespace mippp {

class soplex_lp_model {
private:
    const soplex_api & SoPlex;
    void * model;
    std::optional<lp_status> opt_lp_status;
    double ojective_offset;
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
    [[nodiscard]] explicit soplex_lp_model(const soplex_api & api)
        : SoPlex(api), model(SoPlex.create()), ojective_offset(0.0) {}
    ~soplex_lp_model() { SoPlex.free(model); }

    std::size_t num_variables() {
        return static_cast<std::size_t>(SoPlex.numCols(model));
    }
    std::size_t num_constraints() {
        return static_cast<std::size_t>(SoPlex.numRows(model));
    }
    // std::size_t num_entries() {
    //     return static_cast<std::size_t>(SoPlex.getNumElements(model));
    // }

    void set_maximization() { SoPlex.setIntParam(model, 0, 1); }
    void set_minimization() { SoPlex.setIntParam(model, 0, -1); }

    void set_objective_offset(double constant) { ojective_offset = constant; }
    void set_objective(linear_expression auto && le) {
        auto num_vars = num_variables();
        tmp_scalars.resize(num_vars);
        std::fill(tmp_scalars.begin(), tmp_scalars.end(), 0.0);
        for(auto && [var, coef] : le.linear_terms()) {
            tmp_scalars[static_cast<std::size_t>(var)] = coef;
        }
        SoPlex.changeObjReal(model, tmp_scalars.data(),
                             static_cast<int>(num_vars));
        set_objective_offset(le.constant());
    }
    double get_objective_offset() { return ojective_offset; }

    variable add_variable(
        const variable_params p = {
            .obj_coef = 0,
            .lower_bound = 0,
            .upper_bound = std::numeric_limits<double>::infinity()}) {
        int var_id = static_cast<int>(num_variables());
        SoPlex.addColReal(model, NULL, 0, 0, p.obj_coef, p.lower_bound,
                          p.upper_bound);
        return variable(var_id);
    }
    // variables_range add_variables(std::size_t count,
    //                               const variable_params p);

    constraint add_constraint(linear_constraint auto && lc) {
        int num_nz = 0;
        auto num_constrs = num_constraints();
        tmp_scalars.resize(num_variables());
        std::fill(tmp_scalars.begin(), tmp_scalars.end(), 0.0);
        for(auto && [var, coef] : lc.expression().linear_terms()) {
            tmp_scalars[static_cast<std::size_t>(var)] = coef;
            ++num_nz;
        }
        const double b = -lc.expression().constant();
        SoPlex.addRowReal(
            model, tmp_scalars.data(), static_cast<int>(num_variables()),
            num_nz,
            (lc.relation() == constraint_relation::less_equal_zero)
                ? -std::numeric_limits<double>::infinity()
                : b,
            (lc.relation() == constraint_relation::greater_equal_zero)
                ? std::numeric_limits<double>::infinity()
                : b);
        return static_cast<int>(num_constrs);
    }

    void optimize() {
        if(num_variables() == 0u) {
            opt_lp_status.emplace(lp_status::optimal);
            return;
        }
        SoPlex.optimize(model);
    }

    double get_solution_value() {
        return ojective_offset + SoPlex.objValueReal(model);
    }
    auto get_solution() {
        auto num_vars = num_variables();
        auto solution = std::make_unique_for_overwrite<double[]>(num_vars);
        SoPlex.getPrimalReal(model, solution.get(), static_cast<int>(num_vars));
        return variable_mapping(std::move(solution));
    }
    auto get_dual_solution() {
        auto num_constrs = num_constraints();
        auto solution = std::make_unique_for_overwrite<double[]>(num_constrs);
        SoPlex.getDualReal(model, solution.get(),
                           static_cast<int>(num_constrs));
        return variable_mapping(std::move(solution));
    }
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_SOPLEX_LP_MODEL_HPP