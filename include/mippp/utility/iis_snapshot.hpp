#pragma once

#include <concepts>
#include <cstddef>
#include <utility>
#include <variant>

#include "mippp/detail/handle_status_table.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/utility/iis_outcome.hpp"
#include "mippp/utility/variant.hpp"

namespace mippp {

// Taken once and keyed by handle id, it describes the model as it was: a
// handle created afterwards may reuse the id of a removed entity, and then
// reads that entity's entry.
template <typename Variable, typename Constraint, typename VariableStatus,
          typename ConstraintStatus, typename Outcome>
    requires(!std::same_as<Variable, Constraint>) &&
            lp_iis_status<VariableStatus> && lp_iis_status<ConstraintStatus> &&
            lp_iis_outcome<Outcome> &&
            std::same_as<std::variant_alternative_t<0, VariableStatus>,
                         iis_status::absent> &&
            std::same_as<std::variant_alternative_t<0, ConstraintStatus>,
                         iis_status::absent> &&
            std::same_as<std::variant_alternative_t<0, Outcome>,
                         iis_outcome::incomplete>
class iis_snapshot {
private:
    detail::handle_status_table<VariableStatus> _variables;
    detail::handle_status_table<ConstraintStatus> _constraints;
    std::size_t _num_variable_members;
    std::size_t _num_constraint_members;
    Outcome _outcome;

public:
    iis_snapshot(detail::handle_status_table<VariableStatus> variables,
                 detail::handle_status_table<ConstraintStatus> constraints,
                 Outcome outcome)
        : _variables(std::move(variables))
        , _constraints(std::move(constraints))
        , _num_variable_members(
              _variables.template count_a<iis_status::member>())
        , _num_constraint_members(
              _constraints.template count_a<iis_status::member>())
        , _outcome(outcome) {}

    [[nodiscard]] VariableStatus get_status(Variable v) const noexcept {
        return _variables.get(v.uid());
    }
    [[nodiscard]] ConstraintStatus get_status(Constraint c) const noexcept {
        return _constraints.get(c.uid());
    }

    // irreducible with no member means that the background alone, such as
    // integrality, is infeasible, never that the model is feasible
    [[nodiscard]] Outcome get_outcome() const noexcept { return _outcome; }

    [[nodiscard]] std::size_t num_variable_members() const noexcept {
        return _num_variable_members;
    }
    [[nodiscard]] std::size_t num_constraint_members() const noexcept {
        return _num_constraint_members;
    }
};

namespace detail {

// The answer of a routine that names the side of every member, variables and
// rows alike: the deletion filter always knows which side it relaxed, and the
// native routines that report one flag per bound decode to the same four tags.
using iis_sided_status =
    std::variant<iis_status::absent, iis_status::member_lower,
                 iis_status::member_upper, iis_status::member_both>;

// The answer of a routine that flags one side where a member needs both:
// such a member is reported whole, the others by their sides.
using iis_whole_or_sided_status =
    std::variant<iis_status::absent, iis_status::member,
                 iis_status::member_lower, iis_status::member_upper,
                 iis_status::member_both>;

// The answer of a routine that flags a row's membership only: an inequality
// row has one side to name, and an equality or ranged row is reported whole
// rather than with a side the routine never named.
using iis_whole_or_one_side_status =
    std::variant<iis_status::absent, iis_status::member,
                 iis_status::member_lower, iis_status::member_upper>;

// The tag of a row that such a routine flags, from the native sense the
// backend spells LessEqual and GreaterEqual.
template <auto LessEqual, auto GreaterEqual, typename Sense>
[[nodiscard]] constexpr iis_whole_or_one_side_status iis_row_status_by_sense(
    Sense sense) noexcept {
    if(sense == LessEqual)
        return iis_whole_or_one_side_status(
            std::in_place_type<iis_status::member_upper>);
    if(sense == GreaterEqual)
        return iis_whole_or_one_side_status(
            std::in_place_type<iis_status::member_lower>);
    return iis_whole_or_one_side_status(std::in_place_type<iis_status::member>);
}

// The tag of a member flagged on at least one side. whole asks for a bare
// member, which a Status without that alternative answers by the sides.
template <typename Status>
[[nodiscard]] constexpr Status iis_flagged_status(bool lower, bool upper,
                                                  bool whole) noexcept {
    if(lower && upper)
        return Status(std::in_place_type<iis_status::member_both>);
    if constexpr(variant_with_alternative<Status, iis_status::member>) {
        if(whole) return Status(std::in_place_type<iis_status::member>);
    }
    if(lower) return Status(std::in_place_type<iis_status::member_lower>);
    return Status(std::in_place_type<iis_status::member_upper>);
}

// A native routine's answer as its decoder builds it: members are flagged
// into the two tables, then finish() names how the call ended.
template <typename Snapshot>
class iis_answer;

template <typename Variable, typename Constraint, typename VariableStatus,
          typename ConstraintStatus, typename Outcome>
class iis_answer<iis_snapshot<Variable, Constraint, VariableStatus,
                              ConstraintStatus, Outcome>> {
public:
    using snapshot = iis_snapshot<Variable, Constraint, VariableStatus,
                                  ConstraintStatus, Outcome>;

    // written directly where a tag does not come from side flags, as for a
    // row named by its sense
    handle_status_table<VariableStatus> variables;
    handle_status_table<ConstraintStatus> constraints;

    iis_answer(std::size_t variable_id_bound, std::size_t constraint_id_bound)
        : variables(variable_id_bound), constraints(constraint_id_bound) {}

    // Flagged on neither side, the entity is left unwritten, absent unless
    // written before: iis_flagged_status would make it a member.
    void flag_variable(std::size_t id, bool lower, bool upper,
                       bool whole = false) {
        if(lower || upper)
            variables.set(
                id, iis_flagged_status<VariableStatus>(lower, upper, whole));
    }
    void flag_constraint(std::size_t id, bool lower, bool upper,
                         bool whole = false) {
        if(lower || upper)
            constraints.set(
                id, iis_flagged_status<ConstraintStatus>(lower, upper, whole));
    }

    // hands the tables over: one call per answer
    [[nodiscard]] snapshot finish(Outcome outcome) {
        return snapshot(std::move(variables), std::move(constraints), outcome);
    }
};

}  // namespace detail

}  // namespace mippp
