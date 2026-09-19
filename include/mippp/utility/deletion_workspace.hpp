#pragma once

#include <algorithm>
#include <cstddef>
#include <functional>
#include <optional>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

#include "mippp/utility/linear_iis_model.hpp"

namespace mippp::iis::detail {

template <typename Model>
concept has_deletion_updates = has_modifiable_variables_bounds<Model> &&
                               has_readable_variables_bounds<Model>;

// A feasibility model, not an elastic objective: every candidate has a
// zero-cost continuous slack. Fixed at zero it enforces the original side;
// otherwise it removes that side exactly (no finite big-M). Dimensions stay
// fixed so the backend may retain a basis, but basis reuse is not guaranteed.
template <typename Model, typename Scalar>
class deletion_workspace {
    Model model_; // outlives all stored variable handles
    std::vector<model_variable_t<Model>> slacks_;
    std::vector<Scalar> released_upper_;
    std::vector<bool> active_, requested_;
    options limits_;
    std::size_t solve_count_ = 0;
    work_statistics & stats_;

public:
    template <typename Factory>
        requires has_deletion_updates<Model>
    deletion_workspace(const std::vector<bool> & integer,
                       const std::vector<linear_inequality<Scalar>> & inequalities,
                       Factory & factory, options limits, work_statistics & stats)
        : model_(std::invoke(factory)), active_(inequalities.size(), false),
          requested_(inequalities.size(), false), limits_(limits), stats_(stats) {
        if(model_.num_variables() || model_.num_constraints())
            throw std::invalid_argument(nonempty_model_message(model_.num_variables(), model_.num_constraints()));
        model_.set_minimization();
        using Variable = model_variable_t<Model>;
        std::vector<Variable> variables;
        variables.reserve(integer.size());
        for(bool integral : integer) {
            model_variable_params_t<Model> p{};
            p.obj_coef = Scalar{0};
            p.lower_bound = p.upper_bound = std::nullopt;
            if constexpr(milp_model<Model>) {
                if(integral) {
                    variables.push_back(model_.add_integer_variable(p));
                    continue;
                }
            }
            variables.push_back(model_.add_variable(p));
        }
        slacks_.reserve(inequalities.size());
        released_upper_.reserve(inequalities.size());
        std::vector<std::pair<Variable, Scalar>> terms;
        for(const auto & row : inequalities) {
            const auto slack = model_.add_variable({.obj_coef = Scalar{0},
                .lower_bound = Scalar{0}, .upper_bound = std::nullopt});
            slacks_.push_back(slack);
            // Read the backend's own infinity representation rather than
            // assuming IEEE infinity or hard-coding a solver-specific value.
            released_upper_.push_back(model_.get_variable_upper_bound(slack));
            terms.clear();
            terms.reserve(row.terms.size() + 1);
            for(const auto & [column, value] : row.terms)
                terms.emplace_back(variables[column], value);
            terms.emplace_back(slack, row.lower ? Scalar{1} : Scalar{-1});
            auto expression = linear_expression_view(terms, Scalar{0});
            if(row.lower) model_.add_constraint(operators::operator>=(expression, row.rhs));
            else model_.add_constraint(operators::operator<=(expression, row.rhs));
        }
        if(model_.num_variables() == 0)
            model_.add_variable({.obj_coef = Scalar{0}, .lower_bound = Scalar{0},
                                 .upper_bound = Scalar{0}});
        stats_.model_built(std::max(integer.size() + inequalities.size(), std::size_t{1}),
                           inequalities.size());
    }

    feasibility operator()(std::span<const std::size_t> active) {
        std::fill(requested_.begin(), requested_.end(), false);
        for(auto id : active) requested_[id] = true;
        // A failed or inconclusive deletion restores candidates; a rejected
        // seed restores the full set. Both tighten and release bounds as
        // needed, updating only those that changed.
        for(std::size_t id = 0; id < slacks_.size(); ++id) {
            if(active_[id] == requested_[id]) continue;
            model_.set_variable_upper_bound(slacks_[id],
                requested_[id] ? Scalar{0} : released_upper_[id]);
            ++stats_.bound_updates;
            active_[id] = requested_[id];
        }
        if(!prepare_iis_solve(model_, limits_, stats_)) return feasibility::unknown;
        ++stats_.solver_runs;
        model_.solve();
        ++solve_count_;
        const auto status = model_.solve_status();
        record_solver_issue(status, stats_);
        return classify_feasibility(status);
    }

    [[nodiscard]] std::size_t reoptimizations() const noexcept {
        return solve_count_ ? solve_count_ - 1 : 0;
    }
};

} // namespace mippp::iis::detail
