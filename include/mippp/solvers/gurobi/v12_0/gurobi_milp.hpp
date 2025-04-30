#ifndef MIPPP_GUROBI_v12_0_MILP_HPP
#define MIPPP_GUROBI_v12_0_MILP_HPP

#include <optional>

#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/gurobi/v12_0/gurobi_base_model.hpp"

namespace fhamonic::mippp {
namespace gurobi::v12_0 {

class gurobi_milp : public gurobi_base_model {
public:
    [[nodiscard]] explicit gurobi_milp(const gurobi_api & api)
        : gurobi_base_model(api) {}

    variable add_integer_variable(
        const variable_params params = default_variable_params) {
        int var_id = static_cast<int>(_lazy_num_variables);
        _add_variable(params, GRB_INTEGER);
        return variable(var_id);
    }
    auto add_integer_variables(
        std::size_t count,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = _lazy_num_variables;
        _add_variables(offset, count, params, GRB_INTEGER);
        return _make_variables_range(offset, count);
    }
    template <typename IL>
    auto add_integer_variables(
        std::size_t count, IL && id_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = _lazy_num_variables;
        _add_variables(offset, count, params, GRB_INTEGER);
        return _make_indexed_variables_range(offset, count,
                                             std::forward<IL>(id_lambda));
    }

private:
    inline void _add_binary_variables(const std::size_t & offset,
                                      const std::size_t & count) {
        tmp_types.resize(count);
        std::fill(tmp_types.begin(), tmp_types.end(), GRB_BINARY);
        check(GRB.addvars(model, static_cast<int>(count), 0, NULL, NULL, NULL,
                          NULL, NULL, NULL, tmp_types.data(), NULL));
        _lazy_num_variables += count;
        _var_name_set.resize(offset + count, false);
    }

public:
    variable add_binary_variable() {
        int var_id = static_cast<int>(_lazy_num_variables);
        _add_variable({.obj_coef = 0.0, .lower_bound = 0.0, .upper_bound = 1.0},
                      GRB_BINARY);
        return variable(var_id);
    }
    auto add_binary_variables(std::size_t count) noexcept {
        const std::size_t offset = _lazy_num_variables;
        _add_binary_variables(offset, count);
        return _make_variables_range(offset, count);
    }
    template <typename IL>
    auto add_binary_variables(std::size_t count, IL && id_lambda) noexcept {
        const std::size_t offset = _lazy_num_variables;
        _add_binary_variables(offset, count);
        return _make_indexed_variables_range(offset, count,
                                             std::forward<IL>(id_lambda));
    }

    void set_continuous(variable v) noexcept {
        check(GRB.setcharattrelement(model, GRB_CHAR_ATTR_VTYPE, v.id(),
                                     GRB_CONTINUOUS));
    }
    void set_integer(variable v) noexcept {
        check(GRB.setcharattrelement(model, GRB_CHAR_ATTR_VTYPE, v.id(),
                                     GRB_INTEGER));
    }
    void set_binary(variable v) noexcept {
        check(GRB.setcharattrelement(model, GRB_CHAR_ATTR_VTYPE, v.id(),
                                     GRB_BINARY));
    }

    // add_sos1_constraint
    // add_sos2_constraint
    // add_indicator_constraint

    void set_optimality_tolerance(double tol) {
        check(GRB.setdblparam(env, GRB_DBL_PAR_MIPGAP, tol));
    }
    double get_optimality_tolerance() {
        double tol;
        check(GRB.getdblparam(env, GRB_DBL_PAR_MIPGAP, &tol));
        return tol;
    }

    void solve() { check(GRB.optimize(model)); }

    double get_solution_value() {
        double value;
        check(GRB.getdblattr(model, GRB_DBL_ATTR_OBJVAL, &value));
        return value;
    }

    auto get_solution() {
        auto num_vars = _lazy_num_variables;
        auto solution = std::make_unique_for_overwrite<double[]>(num_vars);
        check(GRB.getdblattrarray(model, GRB_DBL_ATTR_X, 0,
                                  static_cast<int>(num_vars), solution.get()));
        return variable_mapping(std::move(solution));
    }
};

}  // namespace gurobi::v12_0
}  // namespace fhamonic::mippp

#endif  // MIPPP_GUROBI_v12_0_MILP_HPP