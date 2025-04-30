#ifndef MIPPP_MOSEK_v11_MILP_HPP
#define MIPPP_MOSEK_v11_MILP_HPP

#include <optional>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/mosek/v11/mosek_base_model.hpp"

namespace fhamonic::mippp {
namespace mosek::v11 {

class mosek_milp : public mosek_base_model {
private:
    std::optional<lp_status> opt_lp_status;

public:
    [[nodiscard]] explicit mosek_milp(const mosek_api & api)
        : mosek_base_model(api) {
        check(
            MSK.putintparam(task, MSK_IPAR_OPTIMIZER, MSK_OPTIMIZER_MIXED_INT));
    }

    variable add_integer_variable(
        const variable_params params = default_variable_params) {
        int var_id = static_cast<int>(num_variables());
        _add_variable(var_id, params, MSK_VAR_TYPE_INT);
        return variable(var_id);
    }
    auto add_integer_variables(std::size_t count, const variable_params params =
                                                      default_variable_params) {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params, MSK_VAR_TYPE_INT);
        return _make_variables_range(offset, count);
    }
    template <typename IL>
    auto add_integer_variables(
        std::size_t count, IL && id_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params, MSK_VAR_TYPE_INT);
        return _make_indexed_variables_range(offset, count,
                                             std::forward<IL>(id_lambda));
    }
    variable add_binary_variable() {
        int var_id = static_cast<int>(num_variables());
        _add_variable(var_id,
                      variable_params{.lower_bound = 0, .upper_bound = 1},
                      MSK_VAR_TYPE_INT);
        return variable(var_id);
    }
    auto add_binary_variables(std::size_t count) {
        const std::size_t offset = num_variables();
        _add_variables(offset, count,
                       variable_params{.lower_bound = 0, .upper_bound = 1},
                       MSK_VAR_TYPE_INT);
        return _make_variables_range(offset, count);
    }
    template <typename IL>
    auto add_binary_variables(std::size_t count, IL && id_lambda) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count,
                       variable_params{.lower_bound = 0, .upper_bound = 1},
                       MSK_VAR_TYPE_INT);
        return _make_indexed_variables_range(offset, count,
                                             std::forward<IL>(id_lambda));
    }

    void set_continuous(variable v) noexcept {
        check(MSK.putvartype(task, v.id(), MSK_VAR_TYPE_CONT));
    }
    void set_integer(variable v) noexcept {
        check(MSK.putvartype(task, v.id(), MSK_VAR_TYPE_INT));
    }
    void set_binary(variable v) noexcept {
        set_integer(v);
        set_variable_lower_bound(v, 0);
        set_variable_upper_bound(v, 1);
    }

    void solve() { check(MSK.optimize(task)); }

    double get_solution_value() {
        double val = 0.0;
        if(num_variables() > 0)
            check(MSK.getprimalobj(task, MSK_SOL_ITG, &val));
        return val;
    }
    auto get_solution() {
        const auto num_vars = num_variables();
        auto solution = std::make_unique_for_overwrite<double[]>(num_vars);
        if(num_vars > 0) check(MSK.getxx(task, MSK_SOL_ITG, solution.get()));
        return variable_mapping(std::move(solution));
    }
};

}  // namespace mosek::v11
}  // namespace fhamonic::mippp

#endif  // MIPPP_MOSEK_v11_MILP_HPP