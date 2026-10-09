#pragma once

#include <concepts>
#include <cstddef>
#include <utility>
#include <variant>

#include "mippp/detail/handle_status_table.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/utility/variant.hpp"

namespace mippp {

namespace detail {

template <typename Tag>
concept basis_tag = std::derived_from<Tag, basis_status::basic> ||
                    std::derived_from<Tag, basis_status::nonbasic>;

template <typename Status, typename Tag, typename Candidate,
          typename... Candidates>
[[nodiscard]] constexpr Status basis_nearest_listed_status() noexcept {
    if constexpr(std::derived_from<Tag, Candidate> &&
                 variant_with_alternative<Status, Candidate>)
        return Status(std::in_place_type<Candidate>);
    else
        return basis_nearest_listed_status<Status, Tag, Candidates...>();
}

// Tag itself when Status lists it, else the most derived basis_status tag
// that Status lists and Tag derives from: a refinement of the caller's that
// Status lists is never the fallback. Status must list basic and nonbasic.
template <typename Status, basis_tag Tag>
[[nodiscard]] constexpr Status basis_stored_status(Tag) noexcept {
    if constexpr(variant_with_alternative<Status, Tag>)
        return Status(std::in_place_type<Tag>);
    else
        return basis_nearest_listed_status<
            Status, Tag, basis_status::nonbasic_free, basis_status::superbasic,
            basis_status::nonbasic_fixed, basis_status::nonbasic_at_lower_bound,
            basis_status::nonbasic_at_upper_bound,
            basis_status::nonbasic_at_bound, basis_status::nonbasic,
            basis_status::basic>();
}

// What a backend writes, whatever the status type of the basis it is given.
enum class basis_kind : unsigned char {
    basic,
    nonbasic,
    nonbasic_at_lower_bound,
    nonbasic_at_upper_bound,
    nonbasic_fixed,
    superbasic,
    nonbasic_free
};

// Decided by derivation, most derived first, so a refinement tag writes as
// the tag it refines and nonbasic_at_bound as bare nonbasic. An alternative
// derived from neither basic nor nonbasic writes as Default, the status of
// an entity the basis does not cover.
template <basis_kind Default, lp_basis_status Status>
[[nodiscard]] constexpr basis_kind basis_kind_of(
    const Status & status) noexcept {
    return std::visit(
        []<typename Tag>(const Tag &) -> basis_kind {
            if constexpr(std::derived_from<Tag, basis_status::nonbasic_free>)
                return basis_kind::nonbasic_free;
            else if constexpr(std::derived_from<Tag, basis_status::superbasic>)
                return basis_kind::superbasic;
            else if constexpr(std::derived_from<Tag,
                                                basis_status::nonbasic_fixed>)
                return basis_kind::nonbasic_fixed;
            else if constexpr(std::derived_from<
                                  Tag, basis_status::nonbasic_at_lower_bound>)
                return basis_kind::nonbasic_at_lower_bound;
            else if constexpr(std::derived_from<
                                  Tag, basis_status::nonbasic_at_upper_bound>)
                return basis_kind::nonbasic_at_upper_bound;
            else if constexpr(std::derived_from<Tag, basis_status::nonbasic>)
                return basis_kind::nonbasic;
            else if constexpr(std::derived_from<Tag, basis_status::basic>)
                return basis_kind::basic;
            else
                return Default;
        },
        status);
}

// The sources a backend's set_basis accepts: lp_basis over its own handles,
// which does not need the model type to be complete.
template <typename B, typename Variable, typename Constraint>
concept lp_basis_over = requires(const B & basis, Variable v, Constraint c) {
    { basis.get_status(v) } -> lp_basis_status;
    { basis.get_status(c) } -> lp_basis_status;
};

template <typename B, typename Variable>
[[nodiscard]] constexpr basis_kind variable_basis_kind(const B & basis,
                                                       Variable v) {
    return basis_kind_of<basis_kind::nonbasic>(basis.get_status(v));
}
template <typename B, typename Constraint>
[[nodiscard]] constexpr basis_kind constraint_basis_kind(const B & basis,
                                                         Constraint c) {
    return basis_kind_of<basis_kind::basic>(basis.get_status(c));
}

}  // namespace detail

// Taken once and keyed by handle id, it describes the model as it was: a
// handle created afterwards may reuse the id of a removed entity, and then
// reads that entity's entry. An id it does not cover reads alternative 0, a
// column nonbasic and a row basic, which keeps a basis square when entities
// are added; a default snapshot is the slack basis.
template <typename Variable, typename Constraint, typename VariableStatus,
          typename ConstraintStatus>
    requires(!std::same_as<Variable, Constraint>) &&
            lp_basis_status<VariableStatus> &&
            lp_basis_status<ConstraintStatus> &&
            std::same_as<std::variant_alternative_t<0, VariableStatus>,
                         basis_status::nonbasic> &&
            variant_with_alternative<VariableStatus, basis_status::basic> &&
            std::same_as<std::variant_alternative_t<0, ConstraintStatus>,
                         basis_status::basic> &&
            variant_with_alternative<ConstraintStatus, basis_status::nonbasic>
class basis_snapshot {
private:
    detail::handle_status_table<VariableStatus> _variables;
    detail::handle_status_table<ConstraintStatus> _constraints;

public:
    basis_snapshot() = default;
    basis_snapshot(detail::handle_status_table<VariableStatus> variables,
                   detail::handle_status_table<ConstraintStatus> constraints)
        : _variables(std::move(variables))
        , _constraints(std::move(constraints)) {}

    [[nodiscard]] VariableStatus get_status(Variable v) const noexcept {
        return _variables.get(v.uid());
    }
    [[nodiscard]] ConstraintStatus get_status(Constraint c) const noexcept {
        return _constraints.get(c.uid());
    }

    // A tag the status does not list is stored as the most derived
    // basis_status tag it lists: what the backend could write of it.
    template <detail::basis_tag Tag>
    void set_status(Variable v, Tag tag) {
        _variables.grow_and_set(
            v.uid(), detail::basis_stored_status<VariableStatus>(tag));
    }
    template <detail::basis_tag Tag>
    void set_status(Constraint c, Tag tag) {
        _constraints.grow_and_set(
            c.uid(), detail::basis_stored_status<ConstraintStatus>(tag));
    }
};

}  // namespace mippp
