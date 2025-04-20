#ifndef MIPPP_HIGHS_110_lp_HPP
#define MIPPP_HIGHS_110_lp_HPP

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

#include "mippp/solvers/highs/1.10/highs110_api.hpp"

namespace fhamonic {
namespace mippp {

class highs110_lp {
public:
    using variable_id = int;
    using constraint_id = int;
    using scalar = double;
    using variable = model_variable<variable_id, scalar>;
    using constraint = model_constraint<constraint_id, scalar>;

    struct variable_params {
        scalar obj_coef = 0.0;
        std::optional<scalar> lower_bound = 0.0;
        std::optional<scalar> upper_bound = std::nullopt;
    };

private:
    const highs110_api & Highs;
    void * model;
    std::optional<lp_status> opt_lp_status;

    std::vector<std::pair<constraint_id, unsigned int>>
        tmp_constraint_entry_cache;
    std::vector<int> tmp_variables;
    std::vector<double> tmp_scalars;

public:
    [[nodiscard]] explicit highs110_lp(const highs110_api & api)
        : Highs(api), model(Highs.create()) {}
    ~highs110_lp() { Highs.destroy(model); }

private:
    void check(int status) {
        if(status == kHighsStatusError)
            throw std::runtime_error("highs110_lp: error");
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
            tmp_scalars[var.uid()] += coef;
        }
        check(Highs.changeColsCostByRange(
            model, 0, static_cast<int>(num_vars) - 1, tmp_scalars.data()));
        set_objective_offset(le.constant());
    }
    // void add_objective(linear_expression auto && le) {
    //     auto num_vars = num_variables();
    //     tmp_scalars.resize(num_vars);
    //     Highs.getColsByRange(model, 0, static_cast<int>(num_vars) - 1, NULL,
    //                          tmp_scalars.data(), NULL, NULL, NULL, NULL,
    //                          NULL, NULL);
    //     for(auto && [var, coef] : le.linear_terms()) {
    //         tmp_scalars[var.uid()] += coef;
    //     }
    //     check(Highs.changeColsCostByRange(
    //         model, 0, static_cast<int>(num_vars) - 1, tmp_scalars.data()));
    //     set_objective_offset(le.constant());
    // }
    double get_objective_offset() {
        double offset;
        check(Highs.getObjectiveOffset(model, &offset));
        return offset;
    }

    variable add_variable(const variable_params p = {
                              .obj_coef = 0,
                              .lower_bound = 0,
                              .upper_bound = std::nullopt}) {
        int var_id = static_cast<int>(num_variables());
        check(Highs.addCol(model, p.obj_coef,
                           p.lower_bound.value_or(-Highs.getInfinity(model)),
                           p.upper_bound.value_or(Highs.getInfinity(model)), 0,
                           NULL, NULL));
        return variable(var_id);
    }

private:
    void _add_variables(std::size_t offset, std::size_t count,
                        const variable_params & params) {
        const auto diff_count = static_cast<std::ptrdiff_t>(count);
        tmp_scalars.resize(3 * count);
        std::fill(tmp_scalars.begin(), tmp_scalars.begin() + diff_count,
                  params.obj_coef);
        std::fill(tmp_scalars.begin() + diff_count,
                  tmp_scalars.begin() + 2 * diff_count,
                  params.lower_bound.value_or(-Highs.getInfinity(model)));
        std::fill(tmp_scalars.begin() + 2 * diff_count, tmp_scalars.end(),
                  params.upper_bound.value_or(Highs.getInfinity(model)));
        Highs.addCols(model, static_cast<HighsInt>(count), tmp_scalars.data(),
                      tmp_scalars.data() + diff_count,
                      tmp_scalars.data() + 2 * diff_count, 0, NULL, NULL, NULL);
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
        int constr_id = static_cast<int>(num_constraints());
        tmp_constraint_entry_cache.resize(num_variables());
        tmp_variables.resize(0);
        tmp_scalars.resize(0);
        for(auto && [var, coef] : lc.expression().linear_terms()) {
            auto & p = tmp_constraint_entry_cache[var.uid()];
            if(p.first == constr_id + 1) {
                tmp_scalars[p.second] += coef;
                continue;
            }
            p = std::make_pair(constr_id + 1, tmp_variables.size());
            tmp_variables.emplace_back(var.id());
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
        return constraint(constr_id);
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
                throw std::runtime_error("highs110_lp: error");
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
        return constraint_mapping(std::move(solution));
    }
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_HIGHS_110_lp_HPP