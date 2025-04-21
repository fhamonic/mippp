#ifndef MIPPP_GRB_12_MILP_HPP
#define MIPPP_GRB_12_MILP_HPP

#include <limits>
#include <optional>
#include <vector>

#include <range/v3/view/iota.hpp>
#include <range/v3/view/move.hpp>
#include <range/v3/view/zip.hpp>

#include "mippp/detail/function_traits.hpp"
#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/gurobi/12/grb12_base_model.hpp"

namespace fhamonic {
namespace mippp {

class grb12_milp : public grb12_base_model {
public:
    using variable_id = int;
    using constraint_id = int;
    using scalar = double;
    using variable = model_variable<variable_id, scalar>;
    using constraint = model_constraint<constraint_id, scalar>;

public:
    [[nodiscard]] explicit grb12_milp(const grb12_api & api)
        : grb12_base_model(api) {}

public:
    variable add_integer_variable(const variable_params p = {
                                      .obj_coef = 0,
                                      .lower_bound = 0,
                                      .upper_bound = std::nullopt}) {
        int var_id = static_cast<int>(num_variables());
        _add_variable(p.obj_coef, p.lower_bound.value_or(-GRB_INFINITY),
                      p.upper_bound.value_or(GRB_INFINITY), GRB_INTEGER);
        return variable(var_id);
    }
    variable add_binary_variable() {
        int var_id = static_cast<int>(num_variables());
        _add_variable(0.0, 0.0, 1.0, GRB_BINARY);
        return variable(var_id);
    }
    // auto add_integer_variables(std::size_t count,
    //                    variable_params params = {
    //                        .obj_coef = 0,
    //                        .lower_bound = 0,
    //                        .upper_bound = std::nullopt}) noexcept {
    //     const std::size_t offset = num_variables();
    //     _add_variables(offset, count, params);
    //     return make_variables_range(ranges::view::transform(
    //         ranges::view::iota(static_cast<variable_id>(offset),
    //                            static_cast<variable_id>(offset + count)),
    //         [](auto && i) { return variable{i}; }));
    // }
    // template <typename IL>
    // auto add_integer_variables(std::size_t count, IL && id_lambda,
    //                    variable_params params = {
    //                        .obj_coef = 0,
    //                        .lower_bound = 0,
    //                        .upper_bound = std::nullopt}) noexcept {
    //     const std::size_t offset = num_variables();
    //     _add_variables(offset, count, params);
    //     return make_indexed_variables_range(
    //         typename detail::function_traits<IL>::arg_types(),
    //         ranges::view::transform(
    //             ranges::view::iota(static_cast<variable_id>(offset),
    //                                static_cast<variable_id>(offset + count)),
    //             [](auto && i) { return variable{i}; }),
    //         std::forward<IL>(id_lambda));
    // }

    void set_feasibility_tolerance(double tol) {
        check(GRB.setdblparam(env, GRB_DBL_PAR_FEASIBILITYTOL, tol));
    }
    double get_feasibility_tolerance() {
        double tol;
        check(GRB.getdblparam(env, GRB_DBL_PAR_FEASIBILITYTOL, &tol));
        return tol;
    }
    void set_optimality_tolerance(double tol) {
        check(GRB.setdblparam(env, GRB_DBL_PAR_MIPGAP, tol));
    }
    double get_optimality_tolerance() {
        double tol;
        check(GRB.getdblparam(env, GRB_DBL_PAR_MIPGAP, &tol));
        return tol;
    }

    void optimize() { check(GRB.optimize(model)); }

    double get_solution_value() {
        double value;
        check(GRB.getdblattr(model, GRB_DBL_ATTR_OBJVAL, &value));
        return value;
    }

    auto get_solution() {
        auto num_vars = num_variables();
        auto solution = std::make_unique_for_overwrite<double[]>(num_vars);
        check(GRB.getdblattrarray(model, GRB_DBL_ATTR_X, 0,
                                  static_cast<int>(num_vars), solution.get()));
        return variable_mapping(std::move(solution));
    }
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_GRB_12_MILP_HPP