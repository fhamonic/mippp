#pragma once

#include <concepts>
#include <cstddef>
#include <optional>
#include <utility>
#include <variant>

#include "mippp/detail/handle_status_table.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/utility/iis_outcome.hpp"

namespace mippp {

// Taken once and keyed by handle id, it describes the model as it was: a
// handle created afterwards may reuse the id of a removed entity, and then
// reads that entity's entry.
template <typename Variable, typename Constraint, typename VariableStatus,
          typename ConstraintStatus>
    requires(!std::same_as<Variable, Constraint>) &&
            lp_iis_status<VariableStatus> && lp_iis_status<ConstraintStatus> &&
            std::same_as<std::variant_alternative_t<0, VariableStatus>,
                         iis_status::absent> &&
            std::same_as<std::variant_alternative_t<0, ConstraintStatus>,
                         iis_status::absent>
class iis_snapshot {
private:
    detail::handle_status_table<VariableStatus> _variables;
    detail::handle_status_table<ConstraintStatus> _constraints;
    std::size_t _num_variable_members;
    std::size_t _num_constraint_members;
    iis_outcome _outcome;
    std::optional<iis_reason> _reason;

public:
    iis_snapshot(detail::handle_status_table<VariableStatus> variables,
                 detail::handle_status_table<ConstraintStatus> constraints,
                 iis_outcome outcome,
                 std::optional<iis_reason> reason = std::nullopt)
        : _variables(std::move(variables))
        , _constraints(std::move(constraints))
        , _num_variable_members(
              _variables.template count_a<iis_status::member>())
        , _num_constraint_members(
              _constraints.template count_a<iis_status::member>())
        , _outcome(outcome)
        , _reason(reason) {}

    [[nodiscard]] VariableStatus get_status(Variable v) const noexcept {
        return _variables.get(v.uid());
    }
    [[nodiscard]] ConstraintStatus get_status(Constraint c) const noexcept {
        return _constraints.get(c.uid());
    }

    // irreducible with no member means that the background alone, such as
    // integrality, is infeasible, never that the model is feasible
    [[nodiscard]] iis_outcome get_outcome() const noexcept { return _outcome; }
    [[nodiscard]] std::optional<iis_reason> get_reason() const noexcept {
        return _reason;
    }

    [[nodiscard]] std::size_t num_variable_members() const noexcept {
        return _num_variable_members;
    }
    [[nodiscard]] std::size_t num_constraint_members() const noexcept {
        return _num_constraint_members;
    }
};

}  // namespace mippp
