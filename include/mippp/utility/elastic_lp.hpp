#pragma once

#include <cmath>
#include <cstddef>
#include <functional>
#include <optional>
#include <span>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "mippp/algorithm/elasticity_filter.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/utility/linear_iis_model.hpp"

namespace mippp::iis::detail {

// Keep construction and solution interpretation shared by the cold and retained
// model paths. They must differ only in how solver state survives between calls.
template <typename Model, typename Scalar>
auto build_elastic_lp(Model & model,
    std::size_t num_variables,
    const std::vector<linear_inequality<Scalar>> & inequalities,
    std::span<const std::size_t> hard) {
    using Variable = model_variable_t<Model>;
    if(model.num_variables() || model.num_constraints())
        throw std::invalid_argument(nonempty_model_message(model.num_variables(), model.num_constraints()));
    model.set_minimization();
    std::vector<Variable> variables;
    variables.reserve(num_variables);
    for(std::size_t i = 0; i < num_variables; ++i)
        variables.push_back(model.add_variable(
            {.obj_coef = Scalar{0}, .lower_bound = std::nullopt,
             .upper_bound = std::nullopt}));
    std::vector<bool> fixed(inequalities.size(), false);
    for(auto id : hard) fixed[id] = true;
    std::vector<std::pair<std::size_t, Variable>> slacks;
    slacks.reserve(inequalities.size() - hard.size());
    // Constraint insertion consumes the view; retain only the scratch capacity
    // between rows, including room for the optional elastic slack coefficient.
    std::vector<std::pair<Variable, Scalar>> terms;
    for(std::size_t id = 0; id < inequalities.size(); ++id) {
        const auto & row = inequalities[id];
        terms.clear();
        terms.reserve(row.terms.size() + !fixed[id]);
        for(const auto & [column, coefficient] : row.terms)
            terms.emplace_back(variables[column], coefficient);
        if(!fixed[id]) {
            auto slack = model.add_variable(
                {.obj_coef = Scalar{1}, .lower_bound = Scalar{0},
                 .upper_bound = std::nullopt});
            // Ax + s >= lower; Ax - s <= upper. Unit penalties minimize
            // total violation. Variable bounds are rows here too, so every
            // finite bound remains eligible for inclusion in the conflict.
            terms.emplace_back(slack, row.lower ? Scalar{1} : Scalar{-1});
            slacks.emplace_back(id, slack);
        }
        auto expression = linear_expression_view(terms, Scalar{0});
        if(row.lower)
            model.add_constraint(operators::operator>=(expression, row.rhs));
        else model.add_constraint(operators::operator<=(expression, row.rhs));
    }
    if(model.num_variables() == 0)
        model.add_variable({.obj_coef = Scalar{0}, .lower_bound = Scalar{0},
                            .upper_bound = Scalar{0}});
    return slacks;
}

template <typename Model, typename Scalar>
elastic_trial read_elastic_trial(
    Model & model,
    const std::vector<std::pair<std::size_t, model_variable_t<Model>>> & slacks,
    Scalar tolerance, work_statistics & stats, const std::vector<bool> * fixed = nullptr) {
    const auto status_value = model.solve_status();
    record_solver_issue(status_value, stats);
    const auto state = classify_feasibility(status_value);
    if(state != feasibility::feasible) return {state, {}};
    // A stopped solve with a feasible incumbent is useful to deletion, but
    // its arbitrary slacks can make this heuristic expensive. Require an LP
    // optimum here; the caller can spend the remaining budget on deletion.
    const bool optimal = std::visit([](const auto & value) {
        return std::derived_from<std::decay_t<decltype(value)>, status::optimal>;
    }, status_value);
    if(!optimal) return {};
    auto solution = model.get_solution();
    elastic_trial answer{feasibility::feasible, {}};
    for(const auto & [id, slack] : slacks) {
        // A retained model still has columns for fixed slacks. Their values
        // must never be reported as new violations, even with solver noise.
        if(fixed && (*fixed)[id]) continue;
        const auto value = solution[slack];
        if(!std::isfinite(value) || value < -tolerance) {
            stats.last_solver_issue = solve_issue::numerical;
            return {};
        }
        if(value > tolerance) answer.violated.push_back(id);
    }
    return answer;
}

template <typename Factory, typename Scalar>
elastic_trial solve_elastic_lp(
    std::size_t num_variables,
    const std::vector<linear_inequality<Scalar>> & inequalities,
    std::span<const std::size_t> hard, Factory & factory, Scalar tolerance,
    const options & limits, work_statistics & stats) {
    auto model = std::invoke(factory);
    auto slacks = build_elastic_lp(model, num_variables, inequalities, hard);
    stats.model_built(std::max(num_variables + slacks.size(), std::size_t{1}), inequalities.size());
    if(!prepare_iis_solve(model, limits, stats)) return {};
    ++stats.solver_runs;
    model.solve();
    return read_elastic_trial(model, slacks, tolerance, stats);
}

// Keeping rows and columns in place lets the solver reuse its optimization
// state. Actual reuse depends on the solver and algorithm; a MIP wrapper may
// restart internally even though this model contains only continuous variables.
template <typename Model, typename Scalar>
    requires has_modifiable_variables_bounds<Model>
class elastic_lp_workspace {
    Model model_;
    std::vector<std::pair<std::size_t, model_variable_t<Model>>> slacks_;
    std::vector<bool> fixed_;
    Scalar tolerance_;
    options limits_;
    std::size_t solve_count_ = 0;
    work_statistics & stats_;

public:
    template <typename Factory>
    elastic_lp_workspace(std::size_t num_variables,
                         const std::vector<linear_inequality<Scalar>> & inequalities,
                         Factory & factory, Scalar tolerance, options limits, work_statistics & stats)
        : model_(std::invoke(factory))
        , slacks_(build_elastic_lp(model_, num_variables, inequalities, {}))
        , fixed_(inequalities.size(), false)
        , tolerance_(tolerance)
        , limits_(limits), stats_(stats) {
        stats_.model_built(std::max(num_variables + slacks_.size(), std::size_t{1}), inequalities.size());
    }

    elastic_trial operator()(std::span<const std::size_t> hard) {
        // elasticity_filter only adds to the enforced set. Tightening an
        // upper bound to zero leaves matrix coefficients, objective and basis
        // dimensions intact. No infinity conversion or basis remapping is needed.
        // Construction uses an empty hard set, so every ID has a slack at the
        // same index; unlike the cold path, no slack columns are omitted.
        for(auto id : hard) {
            if(!fixed_[id]) {
                model_.set_variable_upper_bound(slacks_[id].second, Scalar{0});
                ++stats_.bound_updates;
                fixed_[id] = true;
            }
        }
        if(!prepare_iis_solve(model_, limits_, stats_)) return {};
        ++stats_.solver_runs;
        model_.solve();
        ++solve_count_;
        return read_elastic_trial(model_, slacks_, tolerance_, stats_, &fixed_);
    }

    // Counts re-solves on the same model, not successful basis imports or saved
    // simplex iterations: those are solver-specific and not exposed uniformly.
    [[nodiscard]] std::size_t reoptimizations() const noexcept {
        return solve_count_ == 0 ? 0 : solve_count_ - 1;
    }
};

}  // namespace mippp::iis::detail
