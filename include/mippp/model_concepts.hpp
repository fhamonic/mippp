#pragma once

#include <chrono>
#include <concepts>
#include <cstddef>
#include <initializer_list>
#include <iterator>
#include <optional>
#include <ranges>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <variant>

#include "mippp/detail/variadic_helper.hpp"
#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/quadratic_expression.hpp"
#include "mippp/utility/keys_view.hpp"
#include "mippp/utility/memory_size.hpp"
#include "mippp/utility/status.hpp"
#include "mippp/utility/variant.hpp"

namespace mippp {

template <typename M>
using model_variable_t =
    std::decay_t<decltype(std::declval<M &>().add_variable())>;

template <typename M>
using model_scalar_t = linear_expression_scalar_t<model_variable_t<M>>;

template <typename M>
using model_variable_params_t =
    std::remove_cvref_t<decltype(M::default_variable_params)>;

///////////////////////////////////////////////////////////////////////////////
////////////////////////// Dummy types for concepts ///////////////////////////
///////////////////////////////////////////////////////////////////////////////

namespace archetype {
// Key ranges stay forward: add_variables/add_constraints require it.
template <typename T>
auto range() {
    return std::views::empty<T>;
}

// Term ranges do not. `xsum` builds a join over a transform, which is
// input-only, not sized and not common -- so this, not `range()`, is the
// weakest thing a backend must accept. Using the stronger `range()` here let
// `lp_model<M>` be true for a model that cannot consume a real xsum.
template <typename T>
struct term_range {
    struct iterator {
        using value_type = T;
        using difference_type = std::ptrdiff_t;
        T operator*() const;
        iterator & operator++();
        void operator++(int);
        bool operator==(std::default_sentinel_t) const;
    };
    iterator begin() const;
    std::default_sentinel_t end() const;
};

template <typename M>
struct linear_expression {
    auto linear_terms() const {
        return term_range<std::pair<model_variable_t<M>, model_scalar_t<M>>>();
    }
    auto constant() const { return model_scalar_t<M>{}; }
};
template <typename M>
struct linear_constraint {
    auto linear_terms() const {
        return term_range<std::pair<model_variable_t<M>, model_scalar_t<M>>>();
    }
    auto sense() const { return constraint_sense::equal; }
    auto rhs() const { return model_scalar_t<M>{}; }
};
template <typename M>
struct quadratic_expression {
    auto quadratic_terms() const {
        return term_range<std::tuple<model_variable_t<M>, model_variable_t<M>,
                                     model_scalar_t<M>>>();
    }
    auto linear_part() const { return archetype::linear_expression<M>(); }
};
struct any_type {};
constexpr bool operator<(any_type, any_type) { return bool{}; }
struct any_variable {
    auto linear_terms() const {
        return range<std::pair<any_variable, any_type>>();
    }
    auto constant() const { return any_type{}; }
};
// the smallest surface the model_*_t aliases deduce from
struct model {
    any_variable add_variable();
    template <typename LC>
    any_type add_constraint(LC &&);
    static constexpr any_type default_variable_params{};
};
}  // namespace archetype

template <typename M>
using model_constraint_t =
    std::decay_t<decltype(std::declval<M &>().add_constraint(
        std::declval<archetype::linear_constraint<M>>()))>;

static_assert(
    linear_expression<archetype::linear_expression<archetype::model>>);
static_assert(
    linear_constraint<archetype::linear_constraint<archetype::model>>);
static_assert(
    quadratic_expression<archetype::quadratic_expression<archetype::model>>);
static_assert(
    std::same_as<model_variable_t<archetype::model>, archetype::any_variable>);
static_assert(
    std::same_as<model_constraint_t<archetype::model>, archetype::any_type>);
static_assert(std::same_as<model_variable_params_t<archetype::model>,
                           archetype::any_type>);

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// Model ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// clang-format off
template <typename VP, typename T>
concept variable_params = requires(VP & vparams, model_scalar_t<T> s) {
    { vparams.obj_coef } -> std::same_as<model_scalar_t<T> &>;
    { vparams.lower_bound } -> std::same_as<std::optional<model_scalar_t<T>> &>;
    { vparams.upper_bound } -> std::same_as<std::optional<model_scalar_t<T>> &>;
    { vparams = {.obj_coef = s, .lower_bound = s, .upper_bound = s} };
};

template <typename VR, typename T>
concept variables_range =
    std::ranges::random_access_range<VR> &&
    std::same_as<std::ranges::range_value_t<VR>, model_variable_t<T>>;

template <typename CR, typename T>
concept constraints_range =
    std::ranges::random_access_range<CR> &&
    std::same_as<std::ranges::range_value_t<CR>, model_constraint_t<T>>;

struct distinct_variables_t { explicit distinct_variables_t() = default; };
inline constexpr distinct_variables_t distinct_variables {};

template <typename T>
concept lp_model =
    linear_expression<model_variable_t<T>> &&
    variable_params<model_variable_params_t<T>, T> &&
    requires(T & model, model_variable_params_t<T> vpars, model_scalar_t<T> s) {
        { model.set_maximization() };
        { model.set_minimization() };

        { model.add_variable() } -> std::same_as<model_variable_t<T>>;
        { model.add_variable(vpars) } -> std::same_as<model_variable_t<T>>;
        { model.add_variables(std::size_t{1u}) } -> variables_range<T>;
        { model.add_variables(std::size_t{1u}, vpars) } -> variables_range<T>;
        { model.add_variables(std::size_t{1u},
                              [](archetype::any_type) { return 0; }) } 
                -> variables_range<T>;
        { model.add_variables(std::size_t{1u}, 
                              [](archetype::any_type) { return 0; }, vpars) }
                -> variables_range<T>;
        { model.add_variables(archetype::range<archetype::any_type>()) }
                -> variables_range<T>;
        { model.add_variables(archetype::range<archetype::any_type>(), vpars) }
                -> variables_range<T>;

        { model.set_objective_offset(s) };
        { model.set_objective(archetype::linear_expression<T>()) };
        { model.set_objective(distinct_variables,
                              archetype::linear_expression<T>()) };

        { model.add_constraint(archetype::linear_constraint<T>()) }
                -> std::same_as<model_constraint_t<T>>;
        { model.add_constraint(distinct_variables,
                               archetype::linear_constraint<T>()) } 
                -> std::same_as<model_constraint_t<T>>;
        { model.add_constraints(archetype::range<archetype::any_type>(),
                                [](archetype::any_type) {
                                    return archetype::linear_constraint<T>();
                                }) } -> constraints_range<T>;
        { model.add_constraints(distinct_variables,
                                archetype::range<archetype::any_type>(),
                                [](archetype::any_type) {
                                    return archetype::linear_constraint<T>();
                                }) } -> constraints_range<T>;
                                
        { model.num_variables() } -> std::same_as<std::size_t>;
        { model.num_constraints() } -> std::same_as<std::size_t>;
        { model.infinity() } -> std::same_as<model_scalar_t<T>>;
        { model.is_infinite(s) } -> std::same_as<bool>;
        { model.solve() };
        { model.get_status() } -> variant_of<status::any>;
        { model.get_solution_value() } -> std::same_as<model_scalar_t<T>>;
        { model.get_solution() }
                -> input_mapping_of<model_variable_t<T>, model_scalar_t<T>>;
    } && variant_with_alternative<model_status_t<T>, status::unknown>
      && variant_containing_a<model_status_t<T>, status::optimal>;

// set_objective stays linear on every model: a quadratic objective has its
// own setter so that the linear one always replaces the whole objective
template <typename T>
concept qp_model = lp_model<T> && requires(T & model) {
    { model.set_quadratic_objective(archetype::quadratic_expression<T>()) };
    { model.set_quadratic_objective(distinct_variables,
                                    archetype::quadratic_expression<T>()) };
};

template <typename T>
concept milp_model =
    lp_model<T> && requires(T & model, model_variable_t<T> v,
                            model_variable_params_t<T> vparams) {
    { model.add_integer_variable() } -> std::same_as<model_variable_t<T>>;
    { model.add_integer_variable(vparams) }
            -> std::same_as<model_variable_t<T>>;
    { model.add_integer_variables(std::size_t{1u}) } -> variables_range<T>;
    { model.add_integer_variables(std::size_t{1u}, vparams) }
            -> variables_range<T>;
    { model.add_integer_variables(std::size_t{1u}, 
                                  [](archetype::any_type) { return 0; }) }
            -> variables_range<T>;
    { model.add_integer_variables(std::size_t{1u}, 
                                  [](archetype::any_type) { return 0; },
                                  vparams) }
            -> variables_range<T>;
    { model.add_integer_variables(archetype::range<archetype::any_type>()) }
            -> variables_range<T>;
    { model.add_integer_variables(archetype::range<archetype::any_type>(),
                                  vparams) }
            -> variables_range<T>;

    { model.add_binary_variable() } -> std::same_as<model_variable_t<T>>;
    { model.add_binary_variables(std::size_t{1u}) } -> variables_range<T>;
    { model.add_binary_variables(std::size_t{1u}, 
                                 [](archetype::any_type) { return 0; }) }
            -> variables_range<T>;
    { model.add_binary_variables(archetype::range<archetype::any_type>()) }
            -> variables_range<T>;

    { model.set_continuous(v) };
    { model.set_integer(v) };
    { model.set_binary(v) };
};
template <typename T>
concept has_num_nonzeros = requires(T & model) {
    { model.num_nonzeros() } -> std::same_as<std::size_t>;
};

template <typename T>
concept has_lp_status =
    variant_containing_a<model_status_t<T>, status::infeasible> &&
    variant_containing_a<model_status_t<T>, status::unbounded>;

template <typename T>
concept has_refinable_lp_status =
    variant_with_alternative<model_status_t<T>,
                             status::infeasible_or_unbounded> &&
    requires(T & model) { model.refine_lp_status(); };

template <typename T, typename M = T>
concept has_dual_solution = requires(T & model) {
    { model.get_dual_solution() }
            -> input_mapping_of<model_constraint_t<M>, model_scalar_t<M>>;
};

template <typename T, typename M = T>
concept has_reduced_costs = requires(T & model) {
    { model.get_reduced_costs() }
            -> input_mapping_of<model_variable_t<M>, model_scalar_t<M>>;
};
// clang-format on
///////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// Limits ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename T>
concept has_time_limit = requires(T & model, std::chrono::seconds s) {
    { model.set_time_limit(s) };
    { model.get_time_limit() } -> std::common_with<std::chrono::seconds>;
    { model.get_status() } -> variant_containing_a<status::time_limit>;
};
// clang-format off
template <typename T>
concept has_iteration_limit = requires(T & model, std::size_t n) {
    { model.set_iteration_limit(n) };
    { model.get_iteration_limit() } -> std::same_as<std::size_t>;
    { model.get_status() }
            -> variant_containing_a<status::iteration_limit>;
};

template <typename T>
concept has_node_limit = requires(T & model, std::size_t n) {
    { model.set_node_limit(n) };
    { model.get_node_limit() } -> std::same_as<std::size_t>;
    { model.get_status() } -> variant_containing_a<status::node_limit>;
};

template <typename T>
concept has_solution_limit = requires(T & model, std::size_t n) {
    { model.set_solution_limit(n) };
    { model.get_solution_limit() } -> std::same_as<std::size_t>;
    { model.get_status() }
            -> variant_containing_a<status::solution_limit>;
};

template <typename T>
concept has_memory_limit = requires(T & model) {
    { model.set_memory_limit(mebibytes{128u}) };
    { model.set_memory_limit(gigabytes{4u}) };  // any memory_size<>
    { model.get_memory_limit() } -> std::common_with<bytes>;
    { model.get_status() }
            -> variant_containing_a<status::memory_limit>;
};

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// Names ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename T>
concept has_named_variables = requires(T & model, model_variable_t<T> v,
             model_variable_params_t<T> vparams, std::string name) {
        { model.set_variable_name(v, name) };
        { model.get_variable_name(v) } -> std::convertible_to<std::string>;

        { model.add_named_variable(name) } -> std::same_as<model_variable_t<T>>;
        { model.add_named_variable(name, vparams) } 
                -> std::same_as<model_variable_t<T>>;
        { model.add_variables(named(archetype::range<archetype::any_type>(),
                [](archetype::any_type) -> std::string { return ""; }))
        } -> variables_range<T>;
        { model.add_variables(named(archetype::range<archetype::any_type>(),
                [](archetype::any_type) -> std::string { return ""; }), vparams)
        } -> variables_range<T>;
        { model.add_named_variables(
                std::size_t{1u}, [](archetype::any_type) { return 0; },
                [](archetype::any_type) -> std::string { return ""; })
        } -> variables_range<T>;
        { model.add_named_variables(
                std::size_t{1u}, [](archetype::any_type) { return 0; },
                [](archetype::any_type) -> std::string { return ""; }, vparams)
        } -> variables_range<T>;
    };
// clang-format on
template <typename T>
concept has_named_constraints =
    requires(T & model, model_constraint_t<T> c, std::string name) {
        { model.set_constraint_name(c, name) };
        { model.get_constraint_name(c) } -> std::convertible_to<std::string>;
    };

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Objective //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename T>
using model_objective_expression_t =
    decltype(std::declval<T &>().get_objective());
// clang-format off
template <typename T>
concept has_readable_objective = requires(T & model, model_variable_t<T> v) {
        { model.get_objective_offset() } -> std::same_as<model_scalar_t<T>>;
        { model.get_objective_coefficient(v) }
                -> std::same_as<model_scalar_t<T>>;
        { model.get_objective() } -> linear_expression;
    } && std::same_as<linear_expression_variable_t<model_objective_expression_t<T>>,
                      model_variable_t<T>>
      && std::same_as<linear_expression_scalar_t<model_objective_expression_t<T>>,
                      model_scalar_t<T>>;
// clang-format on
template <typename T>
concept has_modifiable_objective =
    requires(T & model, model_variable_t<T> v, model_scalar_t<T> s) {
        { model.set_objective_coefficient(v, s) };
        { model.add_to_objective(archetype::linear_expression<T>()) };
        {
            model.add_to_objective(distinct_variables,
                                   archetype::linear_expression<T>())
        };
    };

// get_objective() reads the linear part on any model, quadratic ones
// included; get_quadratic_objective() reads the whole objective
template <typename T>
using model_quadratic_objective_expression_t =
    decltype(std::declval<T &>().get_quadratic_objective());
// clang-format off
template <typename T>
concept has_readable_quadratic_objective =
    has_readable_objective<T> && requires(T & model) {
        { model.get_quadratic_objective() } -> quadratic_expression;
    } && std::same_as<quadratic_expression_variable_t<
                          model_quadratic_objective_expression_t<T>>,
                      model_variable_t<T>>
      && std::same_as<quadratic_expression_scalar_t<
                          model_quadratic_objective_expression_t<T>>,
                      model_scalar_t<T>>;
// clang-format on

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Variables //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// clang-format off
template <typename T, typename M = T>
concept has_readable_variable_bounds =
    requires(T & model, model_variable_t<M> v) {
        { model.get_variable_lower_bound(v) }
                -> std::same_as<model_scalar_t<M>>;
        { model.get_variable_upper_bound(v) } 
                -> std::same_as<model_scalar_t<M>>;
    };
// clang-format on
template <typename T, typename M = T>
concept has_modifiable_variable_bounds =
    requires(T & model, model_variable_t<M> v, model_scalar_t<M> s) {
        { model.set_variable_lower_bound(v, s) };
        { model.set_variable_upper_bound(v, s) };
    };

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////// Constraints /////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename T, typename M = T>
using model_constraint_lhs_range_t =
    decltype(std::declval<T &>().get_constraint_lhs(
        std::declval<model_constraint_t<M> &>()));

template <typename T, typename M = T>
concept has_readable_constraint_lhs =
    requires(T & model, model_constraint_t<M> c) {
        { model.get_constraint_lhs(c) } -> std::ranges::range;
    } &&
    linear_term<
        std::ranges::range_value_t<model_constraint_lhs_range_t<T, M>>> &&
    std::same_as<model_variable_t<M>,
                 linear_term_variable_t<std::ranges::range_value_t<
                     model_constraint_lhs_range_t<T, M>>>> &&
    std::same_as<model_scalar_t<M>,
                 linear_term_scalar_t<std::ranges::range_value_t<
                     model_constraint_lhs_range_t<T, M>>>>;
// clang-format off
template <typename T>
concept has_modifiable_constraint_lhs =
    requires(T & model, model_constraint_t<T> c) {
        { model.set_constraint_lhs(
                c, archetype::range<
                       std::pair<model_variable_t<T>, model_scalar_t<T>>>()) };
    };
// clang-format on
template <typename T, typename M = T>
concept has_readable_constraint_sense =
    requires(T & model, model_constraint_t<M> c) {
        { model.get_constraint_sense(c) } -> std::same_as<constraint_sense>;
    };

template <typename T>
concept has_modifiable_constraint_sense =
    requires(T & model, model_constraint_t<T> c) {
        { model.set_constraint_sense(c, constraint_sense::equal) };
    };

template <typename T, typename M = T>
concept has_readable_constraint_rhs =
    requires(T & model, model_constraint_t<M> c) {
        { model.get_constraint_rhs(c) } -> std::same_as<model_scalar_t<M>>;
    };

// Defined on every row, unlike get_constraint_sense/rhs which have no
// meaning on a ranged row (Clp and Cbc throw there).
// clang-format off
template <typename T, typename M = T>
concept has_readable_constraint_bounds =
    requires(T & model, model_constraint_t<M> c) {
        { model.get_constraint_lower_bound(c) }
                -> std::same_as<model_scalar_t<M>>;
        { model.get_constraint_upper_bound(c) }
                -> std::same_as<model_scalar_t<M>>;
    };
// clang-format on

template <typename T>
concept has_modifiable_constraint_rhs =
    requires(T & model, model_constraint_t<T> c, model_scalar_t<T> s) {
        { model.set_constraint_rhs(c, s) };
    };

template <typename T, typename M = T>
concept has_readable_constraints =
    has_readable_constraint_lhs<T, M> && has_readable_constraint_sense<T, M> &&
    has_readable_constraint_rhs<T, M> &&
    requires(T & model, model_constraint_t<M> c) {
        { model.get_constraint(c) } -> linear_constraint;
    };

///////////////////////////////////////////////////////////////////////////////
///////////////////////////// Special constraints /////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// clang-format off
// Unconstrained return types: backends can return a dedicated handle or void.
template <typename T, typename M = T>
concept has_lazy_constraints = requires(T & handle) {
    { handle.add_lazy_constraint(archetype::linear_constraint<M>()) };
    { handle.add_lazy_constraint(distinct_variables,
                                 archetype::linear_constraint<M>()) };
};

template <typename T>
concept has_sos1_constraints = requires(
    T & model, std::initializer_list<model_variable_t<T>> init_variables) {
    model.add_sos1_constraint(archetype::range<model_variable_t<T>>());
    model.add_sos1_constraint(init_variables);
};

template <typename T>
concept has_sos2_constraints = requires(
    T & model, std::initializer_list<model_variable_t<T>> init_variables) {
    model.add_sos2_constraint(archetype::range<model_variable_t<T>>());
    model.add_sos2_constraint(init_variables);
};

template <typename T>
concept has_indicator_constraints = requires(T & model, model_variable_t<T> v) {
    model.add_indicator_constraint(v, true, archetype::linear_constraint<T>());
    model.add_indicator_constraint(distinct_variables, v, true,
                                   archetype::linear_constraint<T>());
};

// lb <= expression <= ub as one row: the usual handle comes back, but only
// has_readable_constraint_bounds can read such a row back
template <typename T>
concept has_ranged_constraints = requires(T & model, model_scalar_t<T> s) {
    { model.add_ranged_constraint(archetype::linear_expression<T>(), s, s) }
            -> std::same_as<model_constraint_t<T>>;
    { model.add_ranged_constraint(distinct_variables,
                                  archetype::linear_expression<T>(), s, s) }
            -> std::same_as<model_constraint_t<T>>;
};

///////////////////////////////////////////////////////////////////////////////
////////////////////////////// Column generation //////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename T>
concept has_column_generation = requires(
    T & model, model_variable_params_t<T> vparams, model_scalar_t<T> s,
    std::initializer_list<std::pair<model_constraint_t<T>, model_scalar_t<T>>>
        init_entries) {
    { model.add_column(archetype::range<
                std::pair<model_constraint_t<T>, model_scalar_t<T>>>()) }
            -> std::same_as<model_variable_t<T>>;
    { model.add_column(archetype::range<
                std::pair<model_constraint_t<T>, model_scalar_t<T>>>(),
                       vparams) }
            -> std::same_as<model_variable_t<T>>;
    { model.add_column(init_entries) } -> std::same_as<model_variable_t<T>>;
    { model.add_column(init_entries,
                       vparams) }
            -> std::same_as<model_variable_t<T>>;
};
// clang-format on
template <typename T>
concept has_remove_variable = requires(T & model, model_variable_t<T> v) {
    { model.remove_variable(v) };
    { model.remove_variables(archetype::range<model_variable_t<T>>()) };
};

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// Basis ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

namespace basis_status {
struct basic {};
struct nonbasic {};
struct nonbasic_free : nonbasic {};
struct nonbasic_at_bound : nonbasic {};
struct nonbasic_at_lower_bound : nonbasic_at_bound {};
struct nonbasic_at_upper_bound : nonbasic_at_bound {};
struct nonbasic_fixed : nonbasic_at_bound {};
}  // namespace basis_status

template <typename S>
concept lp_basis_status = variant_containing_a<S, basis_status::basic> &&
                          variant_containing_a<S, basis_status::nonbasic>;

template <typename B, typename T>
concept lp_basis =
    requires(B & basis, model_variable_t<T> v, model_constraint_t<T> c) {
        { basis.get_status(v) } -> lp_basis_status;
        { basis.get_status(c) } -> lp_basis_status;
    };

template <typename T>
using model_basis_t = std::decay_t<decltype(std::declval<T &>().get_basis())>;

template <typename T>
concept has_lp_basis = lp_basis<model_basis_t<T>, T>;

template <typename T>
concept has_modifiable_lp_basis =
    has_lp_basis<T> &&
    requires(T & model, model_basis_t<T> b, model_variable_t<T> v,
             model_constraint_t<T> c, model_scalar_t<T> s) {
        // variables
        { b.set_basic(v) };
        { b.set_nonbasic(v, s) };  // snaps to lower/upper/free within tolerance
        { b.set_status(v, basis_status::basic{}) };
        { b.set_status(v, basis_status::nonbasic_free{}) };
        { b.set_status(v, basis_status::nonbasic_at_lower_bound{}) };
        { b.set_status(v, basis_status::nonbasic_at_upper_bound{}) };
        { b.set_status(v, basis_status::nonbasic_fixed{}) };
        // constraints
        { b.set_basic(c) };
        { b.set_nonbasic(c, s) };  // s: row activity (a·x) to compare w/ bounds
        { b.set_status(c, basis_status::basic{}) };
        { b.set_status(c, basis_status::nonbasic_free{}) };
        { b.set_status(c, basis_status::nonbasic_at_lower_bound{}) };
        { b.set_status(c, basis_status::nonbasic_at_upper_bound{}) };
        { b.set_status(c, basis_status::nonbasic_fixed{}) };
    };

template <typename T>
using model_variable_basis_status_t =
    std::decay_t<decltype(std::declval<T>().get_basis().get_status(
        std::declval<model_variable_t<T>>()))>;

template <typename T>
using model_constraint_basis_status_t =
    std::decay_t<decltype(std::declval<T>().get_basis().get_status(
        std::declval<model_constraint_t<T>>()))>;

template <typename T>
concept has_lp_basis_warm_start =
    has_lp_basis<T> &&
    requires(T & model, model_basis_t<T> b) { model.set_basis(b); };

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// MIP start //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// clang-format off
template <typename T>
concept has_mip_start = requires(
    T & model,
    std::initializer_list<std::pair<model_variable_t<T>, model_scalar_t<T>>>
        init_entries) {
    { model.add_mip_start(archetype::range<std::pair<model_variable_t<T>, 
                                                     model_scalar_t<T>>>()) };
    { model.add_mip_start(init_entries) };
};

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Callbacks //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename T>
using candidate_solution_callback_handle_t =
    typename T::candidate_solution_callback_handle;

template <typename T>
concept has_candidate_solution_callback =
    requires(T & model, candidate_solution_callback_handle_t<T> & handle) {
        { model.set_candidate_solution_callback(
                [](candidate_solution_callback_handle_t<T> &) {}) };
        { handle.get_solution_value() } -> std::same_as<model_scalar_t<T>>;
        { handle.get_solution() }
                -> input_mapping_of<model_variable_t<T>, model_scalar_t<T>>;
    };

// on the candidate-solution handle: discards the candidate without cutting
template <typename T>
concept has_candidate_solution_rejection = requires(T & handle) {
    { handle.reject_solution() };
};

template <typename T>
using node_relaxation_callback_handle_t =
    typename T::node_relaxation_callback_handle;

template <typename T>
concept has_node_relaxation_callback = requires(T & model) {
    { model.set_node_relaxation_callback(
            [](node_relaxation_callback_handle_t<T> &) {}) };
};
// clang-format on
///////////////////////////////////////////////////////////////////////////////
//////////////////////////// Tolerance parameters /////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename T>
concept has_feasibility_tolerance = requires(T & model, model_scalar_t<T> s) {
    { model.set_feasibility_tolerance(s) };
    { model.get_feasibility_tolerance() } -> std::same_as<model_scalar_t<T>>;
};

template <typename T>
concept has_optimality_tolerance = requires(T & model, model_scalar_t<T> s) {
    { model.set_optimality_tolerance(s) };
    { model.get_optimality_tolerance() } -> std::same_as<model_scalar_t<T>>;
};

template <typename T>
concept has_integrality_tolerance = requires(T & model, model_scalar_t<T> s) {
    { model.set_integrality_tolerance(s) };
    { model.get_integrality_tolerance() } -> std::same_as<model_scalar_t<T>>;
};

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Verbosity //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// Models are quiet until set_verbose(true), which prints the solver's log on
// stdout.
template <typename T>
concept has_verbosity = requires(T & model, bool verbose) {
    { model.set_verbose(verbose) };
    { model.is_verbose() } -> std::same_as<bool>;
};

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// Escape hatch /////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename T>
concept has_native_handles =
    requires(T & model, model_variable_t<T> v, model_constraint_t<T> c) {
        model.native_api();
        model.native_model();
        model.native_id(v);
        model.native_id(c);
    };

}  // namespace mippp
