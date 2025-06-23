#ifndef MIPPP_XPRESS_v45_1_MILP_HPP
#define MIPPP_XPRESS_v45_1_MILP_HPP

#include <numeric>
#include <optional>
#include <vector>

#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/xpress/v45_1/xpress_base.hpp"

namespace fhamonic::mippp {
namespace xpress::v45_1 {

class xpress_milp : public xpress_base {
private:
    int lp_status;

public:
    [[nodiscard]] explicit xpress_milp(const xpress_api & api)
        : xpress_base(api) {}

    variable add_integer_variable(
        const variable_params params = default_variable_params) {
        int var_id = static_cast<int>(num_variables());
        _add_variable(params, 'I');
        return variable(var_id);
    }
    auto add_integer_variables(
        std::size_t count,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params, 'I');
        return _make_variables_range(offset, count);
    }
    template <typename IL>
    auto add_integer_variables(
        std::size_t count, IL && id_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params, 'I');
        return _make_indexed_variables_range(offset, count,
                                             std::forward<IL>(id_lambda));
    }

    variable add_binary_variable() {
        int var_id = static_cast<int>(num_variables());
        _add_variable({}, 'B');
        return variable(var_id);
    }
    auto add_binary_variables(std::size_t count) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, {}, 'B');
        return _make_variables_range(offset, count);
    }
    template <typename IL>
    auto add_binary_variables(std::size_t count, IL && id_lambda) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, {}, 'B');
        return _make_indexed_variables_range(offset, count,
                                             std::forward<IL>(id_lambda));
    }

    void set_continuous(variable v) noexcept {
        int var_id = v.id();
        char type = 'C';
        check(XPRS.chgcoltype(prob, 1, &var_id, &type));
    }
    void set_integer(variable v) noexcept {
        int var_id = v.id();
        char type = 'I';
        check(XPRS.chgcoltype(prob, 1, &var_id, &type));
    }
    void set_binary(variable v) noexcept {
        int var_id = v.id();
        char type = 'B';
        check(XPRS.chgcoltype(prob, 1, &var_id, &type));
    }

    // void set_optimality_tolerance(double tol) {
    //     check(
    //         CPX.setdblparam(env, CPXPARAM_Simplex_Tolerances_Optimality,
    //         tol));
    // }
    // double get_optimality_tolerance() {
    //     double tol;
    //     check(
    //         CPX.getdblparam(env, CPXPARAM_Simplex_Tolerances_Optimality,
    //         &tol));
    //     return tol;
    // }

    // void set_feasibility_tolerance(double tol) {
    //     check(
    //         CPX.setdblparam(env, CPXPARAM_Simplex_Tolerances_Feasibility,
    //         tol));
    // }
    // double get_feasibility_tolerance() {
    //     double tol;
    //     check(CPX.getdblparam(env, CPXPARAM_Simplex_Tolerances_Feasibility,
    //                           &tol));
    //     return tol;
    // }

    void solve() { check(XPRS.mipoptimize(prob, NULL)); }

    double get_solution_value() {
        double val;
        check(XPRS.getdblattrib(prob, XPRS_MIPOBJVAL, &val));
        return objective_offset + val;
    }
    auto get_solution() {
        const auto num_vars = num_variables();
        auto solution = std::make_unique_for_overwrite<double[]>(num_vars);
        check(XPRS.getsolution(prob, NULL, solution.get(), 0,
                               static_cast<int>(num_vars) - 1));
        return variable_mapping(std::move(solution));
    }
};

}  // namespace xpress::v45_1
}  // namespace fhamonic::mippp

#endif  // MIPPP_XPRESS_v45_1_MILP_HPP