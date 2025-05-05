#ifndef MIPPP_CPLEX_v22_12_MILP_HPP
#define MIPPP_CPLEX_v22_12_MILP_HPP

#include <numeric>
#include <optional>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/cplex/v22_12/cplex_base.hpp"

namespace fhamonic::mippp {
namespace cplex::v22_12 {

class cplex_milp : public cplex_base {
public:
    [[nodiscard]] explicit cplex_milp(const cplex_api & api)
        : cplex_base(api) {}

    variable add_integer_variable(
        const variable_params params = default_variable_params) {
        int var_id = static_cast<int>(num_variables());
        _add_variable(params, CPX_INTEGER);
        return variable(var_id);
    }
    auto add_integer_variables(
        std::size_t count,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params, CPX_INTEGER);
        return _make_variables_range(offset, count);
    }
    template <typename IL>
    auto add_integer_variables(
        std::size_t count, IL && id_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params, CPX_INTEGER);
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
        std::fill(tmp_types.begin(), tmp_types.end(), CPX_BINARY);
        check(CPX.newcols(
            env, lp, static_cast<int>(count), NULL, tmp_scalars.data(),
            tmp_scalars.data() + static_cast<std::ptrdiff_t>(count),
            tmp_types.data(), NULL));
    }

public:
    variable add_binary_variable() {
        int var_id = static_cast<int>(num_variables());
        _add_variable({.obj_coef = 0.0, .lower_bound = 0.0, .upper_bound = 1.0},
                      CPX_BINARY);
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
        char type = CPX_CONTINUOUS;
        check(CPX.chgctype(env, lp, 1, &var_id, &type));
    }
    void set_integer(variable v) noexcept {
        int var_id = v.id();
        char type = CPX_INTEGER;
        check(CPX.chgctype(env, lp, 1, &var_id, &type));
    }
    void set_binary(variable v) noexcept {
        int var_id = v.id();
        char type = CPX_BINARY;
        check(CPX.chgctype(env, lp, 1, &var_id, &type));
        char lu[2] = {'L', 'U'};
        double bd[2] = {0.0, 1.0};
        check(CPX.chgbds(env, lp, 2, &var_id, lu, bd));
    }

    void solve() {
        int probtype = CPX.getprobtype(env, lp);
        switch(probtype) {
            case CPXPROB_MILP:
                check(CPX.mipopt(env, lp));
                return;
            case CPXPROB_LP:
                check(CPX.lpopt(env, lp));
                return;
            default:
                throw std::runtime_error("cplex_milp: unknowned problem type " +
                                         std::to_string(probtype));
        }
    }

    double get_solution_value() {
        double val;
        check(CPX.solution(env, lp, NULL, &val, NULL, NULL, NULL, NULL));
        return val;
    }
    auto get_solution() {
        auto solution =
            std::make_unique_for_overwrite<double[]>(num_variables());
        check(CPX.solution(env, lp, NULL, NULL, solution.get(), NULL, NULL,
                           NULL));
        return variable_mapping(std::move(solution));
    }
    // auto get_dual_solution() {
    //     auto solution =
    //         std::make_unique_for_overwrite<double[]>(num_constraints());
    //     check(CPX.getsolution(task, CPX_SOL_BAS, NULL, NULL, NULL, NULL,
    //     NULL,
    //                           NULL, NULL, solution.get(), NULL, NULL, NULL,
    //                           NULL, NULL));
    //     return constraint_mapping(std::move(solution));
    // }
};

}  // namespace cplex::v22_12
}  // namespace fhamonic::mippp

#endif  // MIPPP_CPLEX_v22_12_MILP_HPP