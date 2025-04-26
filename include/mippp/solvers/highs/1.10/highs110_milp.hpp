#ifndef MIPPP_HIGHS_110_MILP_HPP
#define MIPPP_HIGHS_110_MILP_HPP

#include <limits>
#include <optional>
#include <vector>

#include <range/v3/view/iota.hpp>
#include <range/v3/view/span.hpp>
#include <range/v3/view/zip.hpp>

#include "mippp/detail/function_traits.hpp"
#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/highs/1.10/highs110_base_model.hpp"

namespace fhamonic {
namespace mippp {

class highs110_milp : public highs110_base_model {
public:
    [[nodiscard]] explicit highs110_milp(const highs110_api & api)
        : highs110_base_model(api) {}

    variable add_integer_variable(
        const variable_params params = default_variable_params) {
        int var_id = static_cast<int>(num_variables());
        _add_variable(var_id, params, kHighsVarTypeInteger);
        return variable(var_id);
    }
    auto add_integer_variables(
        std::size_t count,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params, kHighsVarTypeInteger);
        return _make_variables_range(offset, count);
    }
    template <typename IL>
    auto add_integer_variables(
        std::size_t count, IL && id_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params, kHighsVarTypeInteger);
        return _make_indexed_variables_range(offset, count,
                                             std::forward<IL>(id_lambda));
    }

    variable add_binary_variable() {
        return add_integer_variable(
            {.obj_coef = 0, .lower_bound = 0.0, .upper_bound = 1.0});
    }
    auto add_binary_variables(std::size_t count) noexcept {
        return add_integer_variables(
            count, {.obj_coef = 0, .lower_bound = 0.0, .upper_bound = 1.0});
    }
    template <typename IL>
    auto add_binary_variables(std::size_t count, IL && id_lambda) noexcept {
        return add_integer_variables(
            count, std::forward<IL>(id_lambda),
            {.obj_coef = 0, .lower_bound = 0.0, .upper_bound = 1.0});
    }

    void set_continuous(variable v) noexcept {
        check(
            Highs.changeColIntegrality(model, v.id(), kHighsVarTypeContinuous));
    }
    void set_integer(variable v) noexcept {
        check(Highs.changeColIntegrality(model, v.id(), kHighsVarTypeInteger));
    }
    void set_binary(variable v) noexcept {
        _set_variable_bounds(v, 0.0, 1.0);
        check(Highs.changeColIntegrality(model, v.id(), kHighsVarTypeInteger));
    }

    void solve() { check(Highs.run(model)); }

    double get_solution_value() { return Highs.getObjectiveValue(model); }
    auto get_solution() {
        auto num_vars = num_variables();
        auto solution = std::make_unique_for_overwrite<double[]>(num_vars);
        check(Highs.getSolution(model, solution.get(), NULL, NULL, NULL));
        return variable_mapping(std::move(solution));
    }
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_HIGHS_110_MILP_HPP