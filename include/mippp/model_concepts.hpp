#pragma once

#include <chrono>
#include <concepts>
#include <cstddef>
#include <initializer_list>
#include <optional>
#include <ranges>
#include <string>
#include <tuple>
#include <utility>
#include <variant>

#include "mippp/detail/variadic_helper.hpp"
#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/quadratic_expression.hpp"
#include "mippp/utility/memory_size.hpp"
#include "mippp/utility/solve_status.hpp"

namespace mippp {

template <typename M>
using model_variable_t = typename M::variable;

template <typename M>
using model_variable_params_t = typename M::variable_params;

template <typename M>
using model_scalar_t = typename M::scalar;

template <typename M>
using model_constraint_t = typename M::constraint;

///////////////////////////////////////////////////////////////////////////////
////////////////////////// Dummy types for concepts ///////////////////////////
///////////////////////////////////////////////////////////////////////////////

namespace archetype {
template <typename T>
auto range() {
    return std::views::empty<T>;
}
template <typename M>
struct linear_expression {
    auto linear_terms() const {
        return range<std::pair<model_variable_t<M>, model_scalar_t<M>>>();
    }
    auto constant() const { return model_scalar_t<M>{}; }
};
template <typename M>
struct linear_constraint {
    auto linear_terms() const {
        return range<std::pair<model_variable_t<M>, model_scalar_t<M>>>();
    }
    auto sense() const { return constraint_sense::equal; }
    auto rhs() const { return model_scalar_t<M>{}; }
};
template <typename M>
struct quadratic_expression {
    auto quadratic_terms() const {
        return range<std::tuple<model_variable_t<M>, model_variable_t<M>,
                                model_scalar_t<M>>>();
    }
    auto linear_part() const { return archetype::linear_expression<M>(); }
};
struct any_type {};
constexpr bool operator<(any_type, any_type) { return bool{}; }
struct model {
    using variable = any_type;
    using variable_params = any_type;
    using scalar = any_type;
    using constraint = any_type;
};
}  // namespace archetype
static_assert(
    linear_expression<archetype::linear_expression<archetype::model>>);
static_assert(
    linear_constraint<archetype::linear_constraint<archetype::model>>);
static_assert(
    quadratic_expression<archetype::quadratic_expression<archetype::model>>);

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
                                }) } -> std::ranges::range;
        { model.add_constraints(distinct_variables,
                                archetype::range<archetype::any_type>(),
                                [](archetype::any_type) {
                                    return archetype::linear_constraint<T>();
                                }) } -> std::ranges::range;
                                
        { model.num_variables() } -> std::same_as<std::size_t>;
        { model.num_constraints() } -> std::same_as<std::size_t>;
        { model.solve() };
        { model.solve_status() } -> variant_of<status::any>;
        { model.get_solution_value() } -> std::same_as<model_scalar_t<T>>;
        { model.get_solution() }
                -> input_mapping_of<model_variable_t<T>, model_scalar_t<T>>;
    } && variant_with_alternative<model_solve_status_t<T>, status::unknown>
      && variant_containing_a<model_solve_status_t<T>, status::optimal>;

template <typename T>
concept qp_model = lp_model<T> && requires(T & model) {
    { model.set_objective(archetype::quadratic_expression<T>()) };
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

    { model.add_binary_variable() } -> std::same_as<model_variable_t<T>>;
    { model.add_binary_variables(std::size_t{1u}) } -> variables_range<T>;
    { model.add_binary_variables(std::size_t{1u}, 
                                 [](archetype::any_type) { return 0; }) }
            -> variables_range<T>;

    { model.set_continuous(v) };
    { model.set_integer(v) };
    { model.set_binary(v) };
};
template <typename T>
concept sized_model = requires(T & model) {
    { model.num_entries() } -> std::same_as<std::size_t>;
};

template <typename T>
concept has_lp_status =
    variant_containing_a<model_solve_status_t<T>, status::infeasible> &&
    variant_containing_a<model_solve_status_t<T>, status::unbounded>;

template <typename T>
concept has_refinable_lp_status =
    variant_with_alternative<model_solve_status_t<T>,
                             status::infeasible_or_unbounded> &&
    requires(T & model) { model.refine_lp_status(); };

template <typename T>
concept has_dual_solution = requires(T & model) {
    { model.get_dual_solution() }
            -> input_mapping_of<model_constraint_t<T>, model_scalar_t<T>>;
};

template <typename T>
concept has_reduced_costs = requires(T & model, model_variable_t<T> v) {
    { model.get_reduced_costs() }
            -> input_mapping_of<model_variable_t<T>, model_scalar_t<T>>;
};
// clang-format on
///////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// Limits ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename T>
concept has_time_limit = requires(T & model, std::chrono::seconds s) {
    { model.set_time_limit(s) };
    { model.get_time_limit() } -> std::common_with<std::chrono::seconds>;
    { model.solve_status() } -> variant_containing_a<status::time_limit>;
};
// clang-format off
template <typename T>
concept has_iteration_limit = requires(T & model, std::size_t n) {
    { model.set_iteration_limit(n) };
    { model.get_iteration_limit() } -> std::same_as<std::size_t>;
    { model.solve_status() } 
            -> variant_containing_a<status::iteration_limit>;
};

template <typename T>
concept has_node_limit = requires(T & model, std::size_t n) {
    { model.set_node_limit(n) };
    { model.get_node_limit() } -> std::same_as<std::size_t>;
    { model.solve_status() } -> variant_containing_a<status::node_limit>;
};

template <typename T>
concept has_solution_limit = requires(T & model, std::size_t n) {
    { model.set_solution_limit(n) };
    { model.get_solution_limit() } -> std::same_as<std::size_t>;
    { model.solve_status() }
            -> variant_containing_a<status::solution_limit>;
};

template <typename T>
concept has_memory_limit = requires(T & model) {
    { model.set_memory_limit(mebibytes{128u}) };
    { model.set_memory_limit(gigabytes{4u}) };  // any memory_size<>
    { model.get_memory_limit() } -> std::common_with<bytes>;
    { model.solve_status() }
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
        { model.add_named_variables(
                std::size_t{1u}, [](std::size_t) -> std::string { return ""; })
        } -> variables_range<T>;
        { model.add_named_variables(
                std::size_t{1u}, [](std::size_t) -> std::string { return ""; },
                vparams)
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
using objective_expression_t = decltype(std::declval<T &>().get_objective());
// clang-format off
template <typename T>
concept has_readable_objective = requires(T & model, model_variable_t<T> v) {
        { model.get_objective_offset() } -> std::same_as<model_scalar_t<T>>;
        { model.get_objective_coefficient(v) }
                -> std::same_as<model_scalar_t<T>>;
        { model.get_objective() } -> linear_expression;
    } && std::same_as<linear_expression_variable_t<objective_expression_t<T>>,
                      model_variable_t<T>>
      && std::same_as<linear_expression_scalar_t<objective_expression_t<T>>,
                      model_scalar_t<T>>;
// clang-format on
template <typename T>
concept has_modifiable_objective =
    requires(T & model, model_variable_t<T> v, model_scalar_t<T> s) {
        { model.set_objective_coefficient(v, s) };
        { model.add_objective(archetype::linear_expression<T>()) };
    };

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Variables //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// clang-format off
template <typename T>
concept has_readable_variables_bounds =
    requires(T & model, model_variable_t<T> v) {
        { model.get_variable_lower_bound(v) }
                -> std::same_as<model_scalar_t<T>>;
        { model.get_variable_upper_bound(v) } 
                -> std::same_as<model_scalar_t<T>>;
    };
// clang-format on
template <typename T>
concept has_modifiable_variables_bounds =
    requires(T & model, model_variable_t<T> v, model_scalar_t<T> s) {
        { model.set_variable_lower_bound(v, s) };
        { model.set_variable_upper_bound(v, s) };
    };

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////// Constraints /////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename T>
using constraint_lhs_range_t = decltype(std::declval<T &>().get_constraint_lhs(
    std::declval<model_constraint_t<T> &>()));

template <typename T>
concept has_readable_constraint_lhs =
    requires(T & model, model_constraint_t<T> c) {
        { model.get_constraint_lhs(c) } -> std::ranges::range;
    } && linear_term<std::ranges::range_value_t<constraint_lhs_range_t<T>>> &&
    std::same_as<model_variable_t<T>,
                 linear_term_variable_t<
                     std::ranges::range_value_t<constraint_lhs_range_t<T>>>> &&
    std::same_as<model_scalar_t<T>,
                 linear_term_scalar_t<
                     std::ranges::range_value_t<constraint_lhs_range_t<T>>>>;
// clang-format off
template <typename T>
concept has_modifiable_constraint_lhs =
    requires(T & model, model_constraint_t<T> c) {
        { model.set_constraint_lhs(
                c, archetype::range<
                       std::pair<model_variable_t<T>, model_scalar_t<T>>>()) };
    };
// clang-format on
template <typename T>
concept has_readable_constraint_sense =
    requires(T & model, model_constraint_t<T> c) {
        { model.get_constraint_sense(c) } -> std::same_as<constraint_sense>;
    };

template <typename T>
concept has_modifiable_constraint_sense =
    requires(T & model, model_constraint_t<T> c) {
        { model.set_constraint_sense(c, constraint_sense::equal) };
    };

template <typename T>
concept has_readable_constraint_rhs =
    requires(T & model, model_constraint_t<T> c) {
        { model.get_constraint_rhs(c) } -> std::same_as<model_scalar_t<T>>;
    };

template <typename T>
concept has_modifiable_constraint_rhs =
    requires(T & model, model_constraint_t<T> c, model_scalar_t<T> s) {
        { model.set_constraint_rhs(c, s) };
    };

template <typename T>
concept has_readable_constraints =
    has_readable_constraint_lhs<T> && has_readable_constraint_sense<T> &&
    has_readable_constraint_rhs<T> &&
    requires(T & model, model_constraint_t<T> c) {
        { model.get_constraint(c) } -> linear_constraint;
    };

///////////////////////////////////////////////////////////////////////////////
///////////////////////////// Special constraints /////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// clang-format off
template <typename T>
concept has_sos1_constraints = requires(
    T & model, std::initializer_list<model_variable_t<T>> init_variables) {
    { model.add_sos1_constraint(archetype::range<model_variable_t<T>>()) }
            -> std::same_as<model_constraint_t<T>>;
    { model.add_sos1_constraint(init_variables) }
            -> std::same_as<model_constraint_t<T>>;
};

template <typename T>
concept has_sos2_constraints = requires(
    T & model, std::initializer_list<model_variable_t<T>> init_variables) {
    { model.add_sos2_constraint(archetype::range<model_variable_t<T>>()) }
            -> std::same_as<model_constraint_t<T>>;
    { model.add_sos2_constraint(init_variables) }
            -> std::same_as<model_constraint_t<T>>;
};

template <typename T>
concept has_indicator_constraints = requires(T & model, model_variable_t<T> v) {
    { model.add_indicator_constraint(v, true, 
                                     archetype::linear_constraint<T>()) }
            -> std::same_as<model_constraint_t<T>>;
};

///////////////////////////////////////////////////////////////////////////////
////////////////////////////// Column generation //////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename T>
concept has_add_column = requires(
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
///////////////////////////////// Warm start //////////////////////////////////
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

// clang-format off
template <typename S>
concept lp_basis_status = requires(const S & status) {
    { std::visit(detail::overloaded{
                       [](basis_status::basic) {},
                       [](basis_status::nonbasic) {}
                 },
                 status) };
};
// clang-format on
template <typename B, typename T>
concept lp_basis =
    requires(B & basis, model_variable_t<T> v, model_constraint_t<T> c) {
        // variables
        { basis.is_basic(v) } -> std::same_as<bool>;
        { basis.get_status(v) } -> lp_basis_status;
        // constraints
        { basis.is_basic(c) } -> std::same_as<bool>;
        { basis.get_status(c) } -> lp_basis_status;
    };

template <typename T>
using model_basis_t = std::decay_t<decltype(std::declval<T &>().get_basis())>;

template <typename T>
concept has_lp_basis = lp_basis<model_basis_t<T>, T>;

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
    requires(T & model, model_basis_t<T> b, model_variable_t<T> v,
             model_constraint_t<T> c, model_scalar_t<T> s) {
        { model.set_basis(b) };
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
concept has_candidate_solution_callback = requires(T & model) {
    { model.set_candidate_solution_callback(
            [](candidate_solution_callback_handle_t<T> &) {}) };
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
}  // namespace mippp
