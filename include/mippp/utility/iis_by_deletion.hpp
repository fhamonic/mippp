#pragma once

#include <algorithm>
#include <chrono>
#include <concepts>
#include <cstddef>
#include <exception>
#include <numeric>
#include <optional>
#include <ranges>
#include <span>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

#include "mippp/detail/handle_status_table.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/quadratic_expression.hpp"
#include "mippp/utility/deletion_filter.hpp"
#include "mippp/utility/iis_outcome.hpp"
#include "mippp/utility/iis_snapshot.hpp"
#include "mippp/utility/status.hpp"
#include "mippp/utility/variant.hpp"

namespace mippp {

namespace detail {

// A model whose rows the filter writes through their sense and rhs: it reads
// and writes the sense, writes the rhs, and has neither row-bound setters nor
// ranged rows. Each of its rows has one side or the two equal sides of an ==
// row, and so does every state a row passes through while its sides are
// relaxed and restored, so one sense and one rhs express each of them. Gurobi
// is such a model: its ranges add a slack column rather than a second side.
// has_ranged_constraints alone does not exclude the others, since it means
// that a ranged row can be added, and HiGHS and CPLEX, which add none, range
// a row through their row-bound setters.
template <typename M>
concept iis_rows_as_sense_and_rhs =
    has_readable_constraint_sense<M> && has_modifiable_constraint_sense<M> &&
    has_modifiable_constraint_rhs<M> && !has_modifiable_constraint_bounds<M> &&
    !has_ranged_constraints<M>;

}  // namespace detail

// The models the deletion filter can run on in place: every finite variable
// bound and row side is relaxed to the backend's infinity() and restored, so
// both halves of each pair are needed, and the objective is zeroed for the
// trials and written back from a copy, the Hessian included on a qp_model.
// Row sides are written through the row-bound setters, or, on a model without
// them and without ranged rows, through the rhs, and through the sense when a
// row changes it.
template <typename M>
concept iis_by_deletion_model =
    lp_model<M> && has_enumerable_variables<M> &&
    has_enumerable_constraints<M> && has_readable_variable_bounds<M> &&
    has_modifiable_variable_bounds<M> && has_readable_constraint_bounds<M> &&
    (has_modifiable_constraint_bounds<M> ||
     detail::iis_rows_as_sense_and_rhs<M>) &&
    has_readable_objective<M> && has_status_reset<M> &&
    (!qp_model<M> || has_readable_quadratic_objective<M>);

template <iis_by_deletion_model M>
using iis_by_deletion_t =
    iis_snapshot<model_variable_t<M>, model_constraint_t<M>,
                 detail::iis_sided_status, detail::iis_sided_status,
                 deletion_filter_outcome>;

namespace detail {

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// Candidates ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// Candidate k is variable_sides[k] below variable_sides.size(), and
// row_sides[k - variable_sides.size()] from there on.
template <typename M>
struct iis_deletion_candidates {
    using scalar = model_scalar_t<M>;
    template <typename Handle>
    struct side {
        Handle handle;
        scalar value;
        bool lower;
    };
    std::vector<side<model_variable_t<M>>> variable_sides;
    std::vector<side<model_constraint_t<M>>> row_sides;
    // max uid + 1 over the enumerated handles: ids are not dense once a
    // backend recycles or remaps them, so the enumeration size is not enough
    std::size_t variable_id_bound = 0;
    std::size_t row_id_bound = 0;

    [[nodiscard]] std::size_t size() const noexcept {
        return variable_sides.size() + row_sides.size();
    }
};

template <iis_by_deletion_model M, std::ranges::input_range Variables>
[[nodiscard]] iis_deletion_candidates<M> iis_enumerate_deletion_candidates(
    M & model, Variables && variables) {
    iis_deletion_candidates<M> candidates;
    const auto infinity = model.infinity();
    // a NaN side fails both tests, so it is never a candidate
    for(auto v : variables) {
        candidates.variable_id_bound =
            std::max(candidates.variable_id_bound, v.uid() + 1);
        const auto lb = model.get_variable_lower_bound(v);
        if(lb > -infinity) candidates.variable_sides.push_back({v, lb, true});
        const auto ub = model.get_variable_upper_bound(v);
        if(ub < infinity) candidates.variable_sides.push_back({v, ub, false});
    }
    for(auto c : model.constraints()) {
        candidates.row_id_bound =
            std::max(candidates.row_id_bound, c.uid() + 1);
        const auto lb = model.get_constraint_lower_bound(c);
        if(lb > -infinity) candidates.row_sides.push_back({c, lb, true});
        const auto ub = model.get_constraint_upper_bound(c);
        if(ub < infinity) candidates.row_sides.push_back({c, ub, false});
    }
    return candidates;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////// Classifier //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// Read under a zero objective, which cannot be unbounded: a solver that still
// says infeasible_or_unbounded has proven infeasibility, and one that says
// unbounded has failed. failed keeps its inconclusive reading whatever its
// solution flag, since its point is not trusted, and an unscaled
// infeasibility is not a proof of feasibility.
template <variant_of<status::any> Status>
[[nodiscard]] constexpr deletion_verdict classify_deletion_trial(
    const Status & s) noexcept {
    return std::visit(
        []<typename Tag>([[maybe_unused]] const Tag & tag) {
            if constexpr(std::derived_from<Tag, status::infeasible> ||
                         std::same_as<Tag, status::infeasible_or_unbounded>)
                return deletion_verdict::infeasible;
            else if constexpr(std::derived_from<
                                  Tag, status::optimal_infeasible_unscaled> ||
                              std::derived_from<Tag, status::failed> ||
                              std::derived_from<Tag, status::unbounded>)
                return deletion_verdict::inconclusive;
            else
                return tag.solution_available ? deletion_verdict::feasible
                                              : deletion_verdict::inconclusive;
        },
        s);
}

///////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// Guard /////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// The engine's oracle over one model: it saves what the trials change at
// construction, writes nothing until the first trial, and restore() puts
// everything back. Restoring applies the full candidate set, so one code path
// serves both the trials and the exit.
template <iis_by_deletion_model M, typename Clock = std::chrono::steady_clock>
class iis_deletion_guard {
private:
    using scalar = model_scalar_t<M>;
    using variable = model_variable_t<M>;
    using seconds = std::chrono::duration<double>;

    M & _model;
    const iis_deletion_candidates<M> & _candidates;
    const deletion_budget<Clock> & _budget;

    // one flag per candidate: true while the side is known to hold its
    // saved value
    std::vector<char> _at_saved_value;

    // get_objective() is a view over the solver's live coefficients on
    // several backends: a saved view would read back the zeros written for
    // the trials, so the terms are copied. The offset is read separately
    // because the view's constant may be a compile-time zero.
    decltype(materialize(std::declval<M &>().get_objective())) _objective;
    scalar _offset;
    std::vector<std::tuple<variable, variable, scalar>> _quadratic_terms;

    // engaged by the first trial when a deadline is forwarded
    std::optional<seconds> _saved_time_limit;

    bool _mutated = false;
    bool _solved = false;
    bool _restored = false;

    template <typename Side>
    [[nodiscard]] scalar _side_value(const Side & s, bool relaxed) const {
        if(!relaxed) return s.value;
        return s.lower ? -_model.infinity() : _model.infinity();
    }
    // A model without row-bound setters cannot write one side of a row, so
    // the row's whole state is computed: side r as asked, the other as its
    // candidate's state has it, or infinite when it is no candidate. The
    // sense is written only when that state needs another one, so a one-sided
    // row is relaxed and restored through its rhs alone, and a row left with
    // no side keeps its sense under an infinite rhs, an == row becoming a <=
    // row. The two sides of a row are consecutive candidates, and the flag of
    // side r is not read, since the callers update it on either side of the
    // write.
    void _write_row(std::size_t r, bool relaxed)
        requires detail::iis_rows_as_sense_and_rhs<M>
    {
        const auto & rows = _candidates.row_sides;
        const std::size_t offset = _candidates.variable_sides.size();
        const scalar infinity = _model.infinity();
        scalar lower = -infinity, upper = infinity;
        const auto take = [&](std::size_t i, bool is_relaxed) {
            (rows[i].lower ? lower : upper) = _side_value(rows[i], is_relaxed);
        };
        take(r, relaxed);
        if(r > 0 && rows[r - 1].handle == rows[r].handle)
            take(r - 1, !_at_saved_value[offset + r - 1]);
        if(r + 1 < rows.size() && rows[r + 1].handle == rows[r].handle)
            take(r + 1, !_at_saved_value[offset + r + 1]);
        const auto handle = rows[r].handle;
        const constraint_sense current = _model.get_constraint_sense(handle);
        constraint_sense sense = constraint_sense::less_equal;
        scalar rhs = infinity;
        // two finite sides are the equal sides of an == row: without ranged
        // rows, no other row has them
        if(lower > -infinity && upper < infinity) {
            sense = constraint_sense::equal;
            rhs = lower;
        } else if(lower > -infinity) {
            sense = constraint_sense::greater_equal;
            rhs = lower;
        } else if(upper < infinity) {
            rhs = upper;
        } else if(current == constraint_sense::greater_equal) {
            sense = constraint_sense::greater_equal;
            rhs = -infinity;
        }
        if(sense != current) _model.set_constraint_sense(handle, sense);
        _model.set_constraint_rhs(handle, rhs);
    }
    void _write_side(std::size_t k, bool relaxed) {
        const std::size_t num_variable_sides =
            _candidates.variable_sides.size();
        if(k < num_variable_sides) {
            const auto & s = _candidates.variable_sides[k];
            const scalar value = _side_value(s, relaxed);
            if(s.lower)
                _model.set_variable_lower_bound(s.handle, value);
            else
                _model.set_variable_upper_bound(s.handle, value);
        } else if constexpr(has_modifiable_constraint_bounds<M>) {
            const auto & s = _candidates.row_sides[k - num_variable_sides];
            const scalar value = _side_value(s, relaxed);
            if(s.lower)
                _model.set_constraint_lower_bound(s.handle, value);
            else
                _model.set_constraint_upper_bound(s.handle, value);
        } else {
            _write_row(k - num_variable_sides, relaxed);
        }
    }
    // Only the sides whose state differs from the wanted one are written: the
    // single pass moves one or two sides per trial.
    void _apply(std::span<const std::size_t> active) {
        std::vector<char> wanted(_candidates.size(), char{0});
        for(const std::size_t k : active) wanted[k] = 1;
        for(std::size_t k = 0; k < wanted.size(); ++k) {
            if(wanted[k] == _at_saved_value[k]) continue;
            // cleared before a relaxing write and set after a restoring one:
            // a throwing setter must leave the side marked for the restore
            if(wanted[k]) {
                _write_side(k, false);
                _at_saved_value[k] = 1;
            } else {
                _at_saved_value[k] = 0;
                _write_side(k, true);
            }
        }
    }

public:
    iis_deletion_guard(M & model, const iis_deletion_candidates<M> & candidates,
                       const deletion_budget<Clock> & budget)
        : _model(model)
        , _candidates(candidates)
        , _budget(budget)
        , _at_saved_value(candidates.size(), char{1})
        , _objective(materialize(model.get_objective()))
        , _offset(model.get_objective_offset()) {
        if constexpr(qp_model<M>) {
            auto qe = model.get_quadratic_objective();
            for(auto && [v1, v2, c] : qe.quadratic_terms())
                _quadratic_terms.emplace_back(v1, v2, c);
        }
    }
    iis_deletion_guard(const iis_deletion_guard &) = delete;
    iis_deletion_guard & operator=(const iis_deletion_guard &) = delete;
    // The normal path calls restore() first, so this only runs after an
    // exception, where a second error could not be reported without
    // terminating the program.
    ~iis_deletion_guard() noexcept {
        if(_restored) return;
        try {
            restore();
        } catch(...) {
        }
    }

    deletion_verdict operator()(std::span<const std::size_t> active) {
        if(!_mutated) {
            if constexpr(has_time_limit<M>) {
                if(_budget.deadline != Clock::time_point::max())
                    _saved_time_limit = seconds(_model.get_time_limit());
            }
            // set before the write, so that a throwing set_objective is
            // still followed by the restore of the objective
            _mutated = true;
            _model.set_objective(empty_linear_expression<variable, scalar>);
        }
        _apply(active);
        if constexpr(has_time_limit<M>) {
            if(_saved_time_limit) {
                const seconds remaining(_budget.deadline - Clock::now());
                // the engine stops at the deadline between trials, so this is
                // the window in which a non-positive limit would be written,
                // which some setters reject
                if(remaining <= seconds::zero())
                    return deletion_verdict::inconclusive;
                // std::min(a, b) is b < a ? b : a, so a NaN saved limit
                // forwards the remaining time
                _model.set_time_limit(std::min(remaining, *_saved_time_limit));
            }
        }
        // before solve(): a throwing solve may have left a partial state
        _solved = true;
        _model.solve();
        return classify_deletion_trial(_model.get_status());
    }

    // Every item is attempted, and the first error is rethrown once all of
    // them ran. The status reset comes first because it cannot throw and must
    // not be skipped by an item that does.
    void restore() {
        std::exception_ptr first;
        auto attempt = [&first](auto && item) {
            try {
                item();
            } catch(...) {
                if(!first) first = std::current_exception();
            }
        };
        if(_solved) _model.reset_status();
        if(_mutated) {
            for(std::size_t k = 0; k < _at_saved_value.size(); ++k) {
                if(_at_saved_value[k]) continue;
                attempt([&, k] {
                    _write_side(k, false);
                    _at_saved_value[k] = 1;
                });
            }
            attempt([&] {
                if constexpr(qp_model<M>) {
                    // set_objective would clear the Hessian: the quadratic
                    // setter is the one call that restores both parts
                    _model.set_quadratic_objective(quadratic_expression_view(
                        std::views::all(_quadratic_terms), _objective));
                } else {
                    _model.set_objective(_objective);
                }
            });
            attempt([&] { _model.set_objective_offset(_offset); });
            if constexpr(has_time_limit<M>) {
                if(_saved_time_limit)
                    attempt([&] { _model.set_time_limit(*_saved_time_limit); });
            }
        }
        _restored = true;
        if(first) std::rethrow_exception(first);
    }
};

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Prechecks //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// The first variable whose bounds cross, then the first row whose sides
// cross, as the indices of its two candidate sides. Crossed sides are both
// finite, hence both candidates, and adjacent since lower precedes upper.
template <typename M>
[[nodiscard]] std::optional<std::pair<std::size_t, std::size_t>>
iis_first_crossed_pair(const iis_deletion_candidates<M> & candidates) {
    const auto scan = [](const auto & sides, std::size_t offset)
        -> std::optional<std::pair<std::size_t, std::size_t>> {
        for(std::size_t k = 0; k + 1 < sides.size(); ++k) {
            if(sides[k].lower && !sides[k + 1].lower &&
               sides[k].handle == sides[k + 1].handle &&
               sides[k].value > sides[k + 1].value)
                return std::pair{offset + k, offset + k + 1};
        }
        return std::nullopt;
    };
    if(const auto pair = scan(candidates.variable_sides, 0)) return pair;
    return scan(candidates.row_sides, candidates.variable_sides.size());
}

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// Fold /////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <iis_by_deletion_model M>
[[nodiscard]] iis_by_deletion_t<M> iis_fold_deletion_answer(
    const iis_deletion_candidates<M> & candidates,
    deletion_filter_result answer) {
    handle_status_table<iis_sided_status> variables(
        candidates.variable_id_bound);
    handle_status_table<iis_sided_status> rows(candidates.row_id_bound);
    // a side joins the sides its entity already holds, in whatever order
    // the members come
    const auto mark = [](auto & table, const auto & s) {
        const std::size_t id = s.handle.uid();
        const iis_sides held = iis_status::sides_of(table.get(id));
        table.set(id,
                  iis_flagged_status<iis_sided_status>(
                      held.lower || s.lower, held.upper || !s.lower, false));
    };
    const std::size_t num_variable_sides = candidates.variable_sides.size();
    for(const std::size_t k : answer.members) {
        if(k < num_variable_sides)
            mark(variables, candidates.variable_sides[k]);
        else
            mark(rows, candidates.row_sides[k - num_variable_sides]);
    }
    return iis_by_deletion_t<M>(std::move(variables), std::move(rows),
                                answer.outcome);
}

}  // namespace detail

///////////////////////////////////////////////////////////////////////////////
////////////////////////////// The free function //////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// Analyzes the model as it is, in place, and returns the snapshot by value.
// The model comes back with its data restored; after any trial solved,
// get_status() reports unknown, since the solver then holds a trial's
// solution. A NaN or negative limits.time_limit throws std::invalid_argument
// before the model is read.
template <iis_by_deletion_model M>
[[nodiscard]] iis_by_deletion_t<M> compute_iis_by_deletion(
    M & model, const iis_limits & limits = {}) {
    using clock = std::chrono::steady_clock;
    using scalar = model_scalar_t<M>;
    const detail::deletion_budget<clock> budget =
        detail::make_deletion_budget(limits);
    auto variables = model.variables();
    // tested on the variables: a free variable has no side
    const bool column_less = std::ranges::empty(variables);
    const auto candidates =
        detail::iis_enumerate_deletion_candidates(model, variables);
    if(column_less) {
        // Every activity is 0, and solvers answer unknown without columns:
        // the first side that 0 violates is the whole explanation, a row's
        // lower side winning when 0 violates both, since it comes first. The
        // comparison is exact.
        deletion_filter_result answer{{}, iis_outcome::feasible{}};
        for(std::size_t k = 0; k < candidates.row_sides.size(); ++k) {
            const auto & s = candidates.row_sides[k];
            if(s.lower ? s.value > scalar{0} : s.value < scalar{0}) {
                answer = {{k}, iis_outcome::irreducible{}};
                break;
            }
        }
        return detail::iis_fold_deletion_answer(candidates, std::move(answer));
    }
    detail::iis_deletion_guard<M, clock> guard(model, candidates, budget);
    detail::deletion_state state;
    if(const auto pair = detail::iis_first_crossed_pair(candidates)) {
        // the pair alone proves infeasibility, so the initial trial is
        // skipped and each side is tested without the other
        state.members = {pair->first, pair->second};
        state.proven = true;
    } else {
        state.members.assign(candidates.size(), std::size_t{0});
        std::iota(state.members.begin(), state.members.end(), std::size_t{0});
    }
    auto answer = detail::run_deletion_filter(std::move(state), guard, budget);
    guard.restore();
    return detail::iis_fold_deletion_answer(candidates, std::move(answer));
}

}  // namespace mippp
