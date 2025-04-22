#ifndef MIPPP_CPLEX_22_LP_HPP
#define MIPPP_CPLEX_22_LP_HPP

#include <limits>
#include <numeric>
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

#include "mippp/solvers/cplex/22/cplex22_api.hpp"

namespace fhamonic {
namespace mippp {

class cplex22_lp {
private:
    const cplex22_api & CPX;
    CPXCENVptr env;
    CPXLPptr lp;

    // std::optional<lp_status> opt_lp_status;
    std::vector<char> tmp_type;
    std::vector<int> tmp_variables;
    std::vector<double> tmp_scalars;

    void check(int error) const {
        if(error == 0) return;
        throw std::runtime_error("CPLEX: error");
    }

    static constexpr char constraint_relation_to_cplex_sense(
        constraint_relation rel) {
        if(rel == constraint_relation::less_equal_zero) return 'L';
        if(rel == constraint_relation::equal_zero) return 'E';
        return 'G';
    }
    // static constexpr constraint_relation mosek_sense_to_constraint_relation(
    //     CPXboundkeye sense) {
    //     if(sense == CPX_BK_UP) return constraint_relation::less_equal_zero;
    //     if(sense == CPX_BK_FX) return constraint_relation::equal_zero;
    //     return constraint_relation::greater_equal_zero;
    // }

public:
    using variable_id = int;
    using constraint_id = int;
    using scalar = double;
    using variable = model_variable<variable_id, scalar>;
    using constraint = model_constraint<constraint_id, scalar>;

    struct variable_params {
        scalar obj_coef = scalar{0};
        std::optional<scalar> lower_bound = 0.0;
        std::optional<scalar> upper_bound = std::nullopt;
    };

public:
    [[nodiscard]] explicit cplex22_lp(const cplex22_api & api)
        : CPX(api)
        , env(CPX.openCPLEX(NULL))
        , lp(CPX.createprob(env, NULL, NULL)) {}
    ~cplex22_lp() { check(CPX.freeprob(env, &lp)); }

    std::size_t num_variables() {
        return static_cast<std::size_t>(CPX.getnumcols(env, lp));
    }
    std::size_t num_constraints() {
        return static_cast<std::size_t>(CPX.getnumrows(env, lp));
    }
    std::size_t num_entries() {
        return static_cast<std::size_t>(CPX.getnumnz(env, lp));
    }

    void set_maximization() { check(CPX.chgobjsen(env, lp, CPX_MAX)); }
    void set_minimization() { check(CPX.chgobjsen(env, lp, CPX_MIN)); }

    void set_objective_offset(double constant) {
        check(CPX.chgobjoffset(env, lp, constant));
    }

    void set_objective(linear_expression auto && le) {
        tmp_variables.resize(0);
        tmp_scalars.resize(0);
        for(auto && [var, coef] : le.linear_terms()) {
            tmp_variables.emplace_back(var.id());
            tmp_scalars.emplace_back(coef);
        }
        check(CPX.chgobj(env, lp, static_cast<int>(tmp_variables.size()),
                         tmp_variables.data(), tmp_scalars.data()));
        set_objective_offset(le.constant());
    }

    double get_objective_offset() {
        double objective_offset;
        check(CPX.getobjoffset(env, lp, &objective_offset));
        return objective_offset;
    }

    variable add_variable(const variable_params p = {
                              .obj_coef = 0,
                              .lower_bound = 0,
                              .upper_bound = std::nullopt}) {
        int var_id = static_cast<int>(num_variables());
        const double lb = p.lower_bound.value_or(-CPX_INFBOUND);
        const double ub = p.upper_bound.value_or(CPX_INFBOUND);
        check(CPX.newcols(env, lp, 1, &p.obj_coef, &lb, &ub, NULL, NULL));
        return variable(var_id);
    }

private:
    void _add_variables(std::size_t offset, std::size_t count,
                        const variable_params & params) {
        check(CPX.newcols(env, lp, static_cast<int>(count), NULL, NULL, NULL,
                          NULL, NULL));
        bool done_iota = false;
        if(auto obj = params.obj_coef; obj != 0.0) {
            if(!done_iota) {
                tmp_variables.resize(count);
                std::iota(tmp_variables.begin(), tmp_variables.end(),
                          static_cast<variable_id>(offset));
                done_iota = true;
            }
            tmp_scalars.resize(count);
            std::fill(tmp_scalars.begin(), tmp_scalars.end(), obj);
            CPX.chgobj(env, lp, static_cast<int>(count), tmp_variables.data(),
                       tmp_scalars.data());
        }
        if(auto lb = params.lower_bound.value_or(-CPX_INFBOUND); lb != 0.0) {
            if(!done_iota) {
                tmp_variables.resize(count);
                std::iota(tmp_variables.begin(), tmp_variables.end(),
                          static_cast<variable_id>(offset));
                done_iota = true;
            }
            tmp_type.resize(count);
            std::fill(tmp_type.begin(), tmp_type.end(), 'L');
            tmp_scalars.resize(count);
            std::fill(tmp_scalars.begin(), tmp_scalars.end(), lb);
            CPX.chgbds(env, lp, static_cast<int>(count), tmp_variables.data(),
                       tmp_type.data(), tmp_scalars.data());
        }
        if(auto ub = params.upper_bound.value_or(CPX_INFBOUND);
           ub != CPX_INFBOUND) {
            if(!done_iota) {
                tmp_variables.resize(count);
                std::iota(tmp_variables.begin(), tmp_variables.end(),
                          static_cast<variable_id>(offset));
                done_iota = true;
            }
            tmp_type.resize(count);
            std::fill(tmp_type.begin(), tmp_type.end(), 'U');
            tmp_scalars.resize(count);
            std::fill(tmp_scalars.begin(), tmp_scalars.end(), ub);
            CPX.chgbds(env, lp, static_cast<int>(count), tmp_variables.data(),
                       tmp_type.data(), tmp_scalars.data());
        }
    }

public:
    auto add_variables(std::size_t count,
                       variable_params params = {
                           .obj_coef = 0,
                           .lower_bound = 0,
                           .upper_bound = std::nullopt}) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params);
        return make_variables_range(ranges::view::transform(
            ranges::view::iota(static_cast<variable_id>(offset),
                               static_cast<variable_id>(offset + count)),
            [](auto && i) { return variable{i}; }));
    }
    template <typename IL>
    auto add_variables(std::size_t count, IL && id_lambda,
                       variable_params params = {
                           .obj_coef = 0,
                           .lower_bound = 0,
                           .upper_bound = std::nullopt}) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params);
        return make_indexed_variables_range(
            typename detail::function_traits<IL>::arg_types(),
            ranges::view::transform(
                ranges::view::iota(static_cast<variable_id>(offset),
                                   static_cast<variable_id>(offset + count)),
                [](auto && i) { return variable{i}; }),
            std::forward<IL>(id_lambda));
    }

    constraint add_constraint(linear_constraint auto && lc) {
        auto constr_id = static_cast<int>(num_constraints());
        tmp_variables.resize(0);
        tmp_scalars.resize(0);
        for(auto && [var, coef] : lc.expression().linear_terms()) {
            tmp_variables.emplace_back(var.id());
            tmp_scalars.emplace_back(coef);
        }
        const double b = -lc.expression().constant();
        const char sense = constraint_relation_to_cplex_sense(lc.relation());
        CPX.addrows(env, lp, 0, 1, static_cast<int>(), &b, &sense, NULL,
                    tmp_variables.data(), tmp_scalars.data(), NULL, NULL);
        return constraint(constr_id);
    }

    void solve() {
        // if(num_variables() == 0u) {
        //     opt_lp_status.emplace(lp_status::optimal);
        //     return;
        // }
        check(CPX.primopt(env, lp));
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

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_CPLEX_22_LP_HPP