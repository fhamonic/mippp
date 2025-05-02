#ifndef MIPPP_COPT_v7_2_MILP_HPP
#define MIPPP_COPT_v7_2_MILP_HPP

#include <numeric>
#include <optional>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/copt/v7_2/copt_base_model.hpp"

namespace fhamonic::mippp {
namespace copt::v7_2 {

class copt_milp : public copt_base_model {
private:
    int _is_mip;

public:
    [[nodiscard]] explicit copt_milp(const copt_api & api)
        : copt_base_model(api) {}

    variable add_integer_variable(
        const variable_params params = default_variable_params) {
        int var_id = static_cast<int>(num_variables());
        _add_variable(params, COPT_INTEGER);
        return variable(var_id);
    }
    auto add_integer_variables(
        std::size_t count,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params, COPT_INTEGER);
        return _make_variables_range(offset, count);
    }
    template <typename IL>
    auto add_integer_variables(
        std::size_t count, IL && id_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params, COPT_INTEGER);
        return _make_indexed_variables_range(offset, count,
                                             std::forward<IL>(id_lambda));
    }

private:
    inline void _add_binary_variables(const std::size_t & offset,
                                      const std::size_t & count) {
        tmp_scalars.resize(2 * count);
        std::fill(tmp_scalars.begin(),
                  tmp_scalars.begin() + static_cast<std::ptrdiff_t>(count), 0);
        std::fill(tmp_scalars.begin() + static_cast<std::ptrdiff_t>(count),
                  tmp_scalars.end(), 1);
        tmp_types.resize(count);
        std::fill(tmp_types.begin(), tmp_types.end(), COPT_BINARY);
        check(COPT.AddCols(
            prob, static_cast<int>(count), tmp_scalars.data(), NULL, NULL, NULL,
            NULL, tmp_types.data(), NULL,
            NULL /*tmp_scalars.data() + static_cast<std::ptrdiff_t>(count)*/,
            NULL));
    }

public:
    variable add_binary_variable() {
        int var_id = static_cast<int>(num_variables());
        _add_variable({.obj_coef = 0.0, .lower_bound = 0.0, .upper_bound = 1.0},
                      COPT_BINARY);
        return variable(var_id);
    }
    auto add_binary_variables(std::size_t count) noexcept {
        const std::size_t offset = num_variables();
        _add_binary_variables(offset, count);
        return _make_variables_range(offset, count);
    }
    template <typename IL>
    auto add_binary_variables(std::size_t count, IL && id_lambda) noexcept {
        const std::size_t offset = num_variables();
        _add_binary_variables(offset, count);
        return _make_indexed_variables_range(offset, count,
                                             std::forward<IL>(id_lambda));
    }

    void set_continuous(variable v) noexcept {
        int var_id = v.id();
        char type = COPT_CONTINUOUS;
        check(COPT.SetColType(prob, 1, &var_id, &type));
    }
    void set_integer(variable v) noexcept {
        int var_id = v.id();
        char type = COPT_INTEGER;
        check(COPT.SetColType(prob, 1, &var_id, &type));
    }
    void set_binary(variable v) noexcept {
        int var_id = v.id();
        char type = COPT_BINARY;
        check(COPT.SetColType(prob, 1, &var_id, &type));
        // char lu[2] = {'L', 'U'};
        // double bd[2] = {0.0, 1.0};
        // check(COPT.chgbds(env, lp, 2, &var_id, lu, bd));
    }

    void solve() {
        check(COPT.GetIntAttr(prob, COPT_INTATTR_ISMIP, &_is_mip));
        if(_is_mip)
            check(COPT.Solve(prob));
        else
            check(COPT.SolveLp(prob));
    }

    double get_solution_value() {
        double val;
        if(_is_mip)
            check(COPT.GetDblAttr(prob, COPT_DBLATTR_BESTOBJ, &val));
        else
            check(COPT.GetDblAttr(prob, COPT_DBLATTR_LPOBJVAL, &val));
        return val;
    }
    auto get_solution() {
        auto solution =
            std::make_unique_for_overwrite<double[]>(num_variables());
        if(_is_mip)
            check(COPT.GetSolution(prob, solution.get()));
        else
            check(COPT.GetLpSolution(prob, solution.get(), NULL, NULL, NULL));
        return variable_mapping(std::move(solution));
    }
};

}  // namespace copt::v7_2
}  // namespace fhamonic::mippp

#endif  // MIPPP_COPT_v7_2_MILP_HPP