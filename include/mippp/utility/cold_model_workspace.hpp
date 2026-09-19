#pragma once
#include <functional>
#include <span>
#include <utility>
#include <vector>
#include "mippp/utility/prepared_linear_system.hpp"

namespace mippp::iis::detail {

// Build a fresh model for each check, reusing temporary buffer capacity.
// The observer can read a certificate before the model is destroyed, but must
// not run the optimizer again: all solves count toward the shared budget.
template <typename Factory, std::floating_point Scalar, domain Mode = domain::original>
class cold_model_workspace {
    using Model = std::invoke_result_t<Factory &>;
    using Variable = model_variable_t<Model>;
    const prepared_linear_system<Scalar> & prepared_;
    Factory & factory_;
    const options & limits_;
    work_statistics & stats_;
    std::vector<model_variable_params_t<Model>> params_;
    std::vector<Variable> variables_;
    std::vector<std::pair<Variable, Scalar>> terms_;
public:
    cold_model_workspace(const prepared_linear_system<Scalar> & prepared,
                         Factory & factory, const options & limits, work_statistics & stats)
        : prepared_(prepared), factory_(factory), limits_(limits), stats_(stats) {}

    template <typename Observer>
        requires std::invocable<Observer &, Model &, std::span<const std::size_t>>
    feasibility operator()(std::span<const std::size_t> active, Observer && observe) {
        const auto & system = prepared_.system;
        const auto & candidates = prepared_.candidates;
        auto model = std::invoke(factory_);
        // Keep capacity across calls, but destroy handles before their model,
        // including on early returns and exceptions. Backends need not provide
        // trivial handle destructors just to benefit from scratch reuse.
        struct clear_handles {
            std::vector<Variable> & variables_;
            std::vector<std::pair<Variable, Scalar>> & terms_;
            ~clear_handles() { terms_.clear(); variables_.clear(); }
        } cleanup{variables_, terms_};
        if(model.num_variables() != 0 || model.num_constraints() != 0)
            throw std::invalid_argument(nonempty_model_message(model.num_variables(), model.num_constraints()));
        model.set_minimization();
        params_.resize(system.variables.size());
        for(auto & p : params_) {
            p = model_variable_params_t<Model>{};
            // A backend's default x >= 0 must not survive removal of a bound.
            // Start free, then install only the currently active bound sides.
            p.obj_coef = Scalar{0};
            p.lower_bound = std::nullopt;
            p.upper_bound = std::nullopt;
        }
        for(auto id : active) {
            const auto [kind, index] = candidates[id];
            if(kind == member_kind::variable_lower)
                params_[index].lower_bound = system.variables[index].lower;
            if(kind == member_kind::variable_upper)
                params_[index].upper_bound = system.variables[index].upper;
        }
        // Some APIs reject inconsistent column bounds when loading the model.
        // This is already an exact infeasibility certificate for this subset.
        for(const auto & p : params_)
            if(p.lower_bound && p.upper_bound &&
               *p.lower_bound > *p.upper_bound) {
                stats_.model_built(0, 0);
                ++stats_.bound_only_proofs;
                return feasibility::infeasible;
            }
        variables_.reserve(params_.size());
        for(std::size_t i = 0; i < params_.size(); ++i) {
            // Keep every variable, even if no active row mentions it. Types
            // belong to the fixed background and column indices remain stable.
            if constexpr(milp_model<Model> && Mode == domain::original) {
                if(system.variables[i].integer) {
                    variables_.push_back(model.add_integer_variable(params_[i]));
                    continue;
                }
            }
            variables_.push_back(model.add_variable(params_[i]));
        }
        // Some backends leave zero-column models unsolved. A fixed dummy
        // column preserves feasibility, including empty contradictory rows.
        if(variables_.empty())
            model.add_variable({.obj_coef = Scalar{0},
                                .lower_bound = Scalar{0},
                                .upper_bound = Scalar{0}});
        std::size_t row_count = 0;
        for(auto id : active) {
            const auto [kind, index] = candidates[id];
            if(kind != member_kind::row_lower && kind != member_kind::row_upper)
                continue;
            ++row_count;
            const auto & row = system.rows[index];
            // add_constraint consumes the borrowed expression synchronously.
            // Clear handles and coefficients before building the next row.
            terms_.clear();
            terms_.reserve(row.terms.size());
            for(const auto & [column, value] : row.terms)
                terms_.emplace_back(variables_[column], value);
            auto expression = linear_expression_view(terms_, Scalar{0});
            // Two one-sided rows work even on backends without ranged-row
            // support. The expression borrows terms_ only until add_constraint
            // has consumed it; no borrowed view is kept across oracle calls.
            if(kind == member_kind::row_lower)
                model.add_constraint(operators::operator>=(expression, *row.lower));
            else model.add_constraint(operators::operator<=(expression, *row.upper));
        }
        stats_.model_built(std::max(params_.size(), std::size_t{1}), row_count);
        if(!detail::prepare_iis_solve(model, limits_, stats_)) return feasibility::unknown;
        ++stats_.solver_runs;
        model.solve();
        const auto status = model.solve_status();
        record_solver_issue(status, stats_);
        const auto state = detail::classify_feasibility(status);

        if(state == feasibility::infeasible) std::invoke(observe, model, active);
        return state;
    }
};
} // namespace mippp::iis::detail
