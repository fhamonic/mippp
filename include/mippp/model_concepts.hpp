#ifndef MIPPP_MODEL_CONCEPTS_HPP
#define MIPPP_MODEL_CONCEPTS_HPP

#include <chrono>
#include <concepts>
#include <optional>
#include <ranges>
#include <string>
#include <variant>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_entities.hpp"
#include "mippp/utility/variadic_helper.hpp"

namespace mippp {

namespace detail {

///////////////////////////////////////////////////////////////////////////////
////////////////////////// Dummy types for concepts ///////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename T>
auto dummy_range() {
    return std::views::empty<T>;
};

template <typename M>
struct dummy_linear_expression {
    auto linear_terms() const {
        return dummy_range<
            std::pair<typename M::variable, typename M::scalar>>();
    }
    auto constant() const { return typename M::scalar{}; }
};
template <typename M>
struct dummy_linear_constraint {
    auto linear_terms() const {
        return dummy_range<
            std::pair<typename M::variable, typename M::scalar>>();
    }
    auto sense() const { return constraint_sense::equal; }
    auto rhs() const { return typename M::scalar{0}; }
};

template <typename M>
struct dummy_quadratic_expression {
    auto quadratic_terms() const {
        return dummy_range<std::tuple<
            typename M::variable, typename M::variable, typename M::scalar>>();
    }
    auto linear_expression() const { return dummy_linear_expression<M>(); }
};

struct dummy_type {};

constexpr bool operator<(dummy_type, dummy_type) { return true; }

}  // namespace detail

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// Model ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename M>
using model_variable_t = typename M::variable;

template <typename M>
using model_constraint_t = typename M::constraint;

template <typename M, typename K>
using model_constraints_range_t =
    decltype(std::declval<M>().add_constraints(detail::dummy_range<K>(), [](K) {
        return detail::dummy_linear_constraint<M>();
    }));

// clang-format off
template <typename T>
concept lp_model = requires(T & model, T::variable v, 
                    T::variable_params vparams, T::constraint c, T::scalar s) {
    { model.set_maximization() };
    { model.set_minimization() };

    { model.add_variable() } -> std::same_as<typename T::variable>;
    { model.add_variable({.obj_coef = s, .lower_bound = s, .upper_bound = s}) }
            -> std::same_as<typename T::variable>;    
    { model.add_variables(std::size_t{1u}) } ->std::ranges::random_access_range;
    { model.add_variables(std::size_t{1u},
                        {.obj_coef = s, .lower_bound = s, .upper_bound = s}) }
            ->std::ranges::random_access_range;
    { model.add_variables(std::size_t{1u}, [](detail::dummy_type) { return 0; }) }
            ->std::ranges::random_access_range;
    { model.add_variables(std::size_t{1u}, [](detail::dummy_type) { return 0; },
                         {.obj_coef = s, .lower_bound = s, .upper_bound = s}) }
            ->std::ranges::random_access_range;

    { model.set_objective_offset(0.0) };
    { model.set_objective(detail::dummy_linear_expression<T>()) };

    { model.add_constraint(detail::dummy_linear_constraint<T>()) }
            -> std::same_as<typename T::constraint>;
    { model.add_constraints(detail::dummy_range<detail::dummy_type>(),
                              [](detail::dummy_type) {
                                  return detail::dummy_linear_constraint<T>();
                              }) } 
            -> std::ranges::range;

    { model.num_variables() } -> std::same_as<std::size_t>;
    { model.num_constraints() } -> std::same_as<std::size_t>;

    { model.solve() };
    { model.get_solution_value() } -> std::same_as<typename T::scalar>;
    { model.get_solution()[v] } -> std::convertible_to<typename T::scalar>;
};


template <typename T>
concept qp_model = lp_model<T> && requires(T & model) {
    { model.set_objective(detail::dummy_quadratic_expression<T>()) };
};

template <typename T>
concept milp_model = lp_model<T> && requires(T & model, T::variable v,
                            T::variable_params vparams, T::scalar s) {
    { model.add_integer_variable() } -> std::same_as<typename T::variable>;
    { model.add_integer_variable(
                        {.obj_coef = s, .lower_bound = s, .upper_bound = s}) }
            -> std::same_as<typename T::variable>;
    { model.add_integer_variables(std::size_t{1u}) }
            ->std::ranges::random_access_range;
    { model.add_integer_variables(std::size_t{1u},
                        {.obj_coef = s, .lower_bound = s, .upper_bound = s}) }
            ->std::ranges::random_access_range;
    { model.add_integer_variables(std::size_t{1u}, 
                                  [](detail::dummy_type) { return 0; }) }
            ->std::ranges::random_access_range;
    { model.add_integer_variables(std::size_t{1u}, 
                        [](detail::dummy_type) { return 0; },
                        {.obj_coef = s, .lower_bound = s, .upper_bound = s}) }
            ->std::ranges::random_access_range;

    { model.add_binary_variable() } -> std::same_as<typename T::variable>;
    { model.add_binary_variables(std::size_t{1u}) }
            ->std::ranges::random_access_range;
    { model.add_binary_variables(std::size_t{1u}, 
                                 [](detail::dummy_type) { return 0; }) }
            ->std::ranges::random_access_range;

    { model.set_continuous(v) };
    { model.set_integer(v) };
    { model.set_binary(v) };
};

template <typename T>
concept sized_model = lp_model<T> && requires(T & model) {
    { model.num_entries() } -> std::same_as<std::size_t>;
};

template <typename T>
concept has_lp_status = lp_model<T> && requires(T & model) {
    { model.proven_optimal() } -> std::same_as<bool>;
    { model.proven_infeasible() } -> std::same_as<bool>;
    { model.proven_unbounded() } -> std::same_as<bool>;
};

template <typename T>
concept has_dual_solution = requires(T & model, T::constraint c) {
    { model.get_dual_solution()[c] } -> std::convertible_to<typename T::scalar>;
};

template <typename T>
concept has_reduced_costs = requires(T & model, typename T::variable v) {
    { model.get_reduced_costs()[v] } -> std::convertible_to<typename T::scalar>;
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////// Termination reasons /////////////////////////////
///////////////////////////////////////////////////////////////////////////////

namespace reason {
struct completed {};
struct optimal : completed {};
struct infeasible_or_unbounded : completed {};
struct infeasible : infeasible_or_unbounded {};
struct unbounded : infeasible_or_unbounded {};
struct stopped {};
struct interrupted : stopped {};
struct numerical_failure : stopped {};
struct reached_limit : stopped {};
struct time_limit : reached_limit {};
struct node_limit : reached_limit {};
struct unknown : stopped {};
}  // namespace reason

template <typename T>
using model_termination_reason_t =
    std::decay_t<decltype(std::declval<T>().termination_reason())>;

template <typename T>
concept has_termination_reason = requires(T & model) {
    { std::visit(detail::overloaded{
                           [](reason::completed) {},
                           [](reason::stopped) {}
                       },
                       model.termination_reason()) };
    };

template <typename T>
concept has_time_limit =
    requires(T & model, std::chrono::seconds s) {
        { model.set_time_limit(s) };
        { model.get_time_limit() } -> std::common_with<std::chrono::seconds>;
    } && (!has_termination_reason<T> ||
          detail::variant_contains_v<reason::time_limit,
                                     model_termination_reason_t<T>>);

template <typename T>
concept has_node_limit = requires(T & model, std::size_t n) {
    { model.set_node_limit(n) };
    { model.get_node_limit() } -> std::same_as<std::size_t>;
    } && (!has_termination_reason<T> ||
          detail::variant_contains_v<reason::node_limit, 
                                     model_termination_reason_t<T>>);

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// Names ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename T>
concept has_named_variables = lp_model<T> && requires(T & model, T::variable v,
                    T::variable_params vparams, T::scalar s, std::string nm) {
    { model.set_variable_name(v, nm) };
    { model.get_variable_name(v) };

    { model.add_named_variable(nm) } -> std::same_as<typename T::variable>;
    { model.add_named_variable(nm,
                        {.obj_coef = s, .lower_bound = s, .upper_bound = s}) }
            -> std::same_as<typename T::variable>;
    { model.add_named_variables(std::size_t{1u},
                        [](std::size_t) -> std::string { return ""; }) }
            ->std::ranges::random_access_range;
    { model.add_named_variables(std::size_t{1u},
                        [](std::size_t) -> std::string { return ""; },
                        {.obj_coef = s, .lower_bound = s, .upper_bound = s}) }
            ->std::ranges::random_access_range;  
    { model.add_named_variables(std::size_t{1u},
                        [](detail::dummy_type) { return 0; },
                        [](detail::dummy_type) -> std::string { return ""; }) }
            ->std::ranges::random_access_range;
    { model.add_named_variables(std::size_t{1u},
                        [](detail::dummy_type) { return 0; },
                        [](detail::dummy_type) -> std::string { return ""; },
                        {.obj_coef = s, .lower_bound = s, .upper_bound = s}) }
            ->std::ranges::random_access_range; 
};

template <typename T>
concept has_named_constraints =
        requires(T & model, T::constraint v, std::string s) {
    { model.set_constraint_name(v, s) };
    { model.get_constraint_name(v) };
};

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Objective //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename T>
using objective_expression_t = decltype(std::declval<T &>().get_objective());

template <typename T>
concept has_readable_objective = requires(T & model, T::variable v) {
        { model.get_objective_offset() }
                -> std::same_as<typename T::scalar>;
        { model.get_objective_coefficient(v) }
                -> std::same_as<typename T::scalar>;
        { model.get_objective() } -> linear_expression;
    } && std::same_as<std::decay_t<
                linear_expression_variable_t<objective_expression_t<T>>>,
                typename T::variable>
      && std::same_as<std::decay_t<
                linear_expression_scalar_t<objective_expression_t<T>>>,
                typename T::scalar>;

template <typename T>
concept has_modifiable_objective =
        requires(T & model, T::variable v, T::scalar s) {
    { model.set_objective_coefficient(v, s) };
    { model.add_objective(detail::dummy_linear_expression<T>()) };
};
    
///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Variables //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
       
template <typename T>
concept has_readable_variables_bounds =
        requires(T & model, T::variable v) {
    { model.get_variable_lower_bound(v) }
            -> std::same_as<typename T::scalar>;
    { model.get_variable_upper_bound(v) }
            -> std::same_as<typename T::scalar>;
};

template <typename T>
concept has_modifiable_variables_bounds =
        requires(T & model, T::variable v, T::scalar s) {
    { model.set_variable_lower_bound(v, s) };
    { model.set_variable_upper_bound(v, s) };
};
    
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////// Constraints /////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename T>
using constraint_lhs_range_t = decltype(std::declval<T &>().get_constraint_lhs(
    std::declval<typename T::constraint &>()));

template <typename T>
concept has_readable_constraint_lhs = requires(T & model, T::constraint c) {
        { model.get_constraint_lhs(c) } ->std::ranges::range;
    } && std::same_as<
            std::decay_t<std::ranges::range_value_t<constraint_lhs_range_t<T>>>,
            std::pair<typename T::variable, typename T::scalar>>;

template <typename T>
concept has_modifiable_constraint_lhs = requires(T & model, T::constraint c) {
    { model.set_constraint_lhs(c, detail::dummy_range<
                   std::pair<typename T::variable, typename T::scalar>>()) };
};

template <typename T>
concept has_readable_constraint_sense = requires(T & model, T::constraint c) {
    { model.get_constraint_sense(c) } -> std::same_as<constraint_sense>;
};

template <typename T>
concept has_modifiable_constraint_sense = requires(T & model, T::constraint c) {
    { model.set_constraint_sense(c, constraint_sense::equal) };
};

template <typename T>
concept has_readable_constraint_rhs = requires(T & model, T::constraint c) {
    { model.get_constraint_rhs(c) } -> std::same_as<typename T::scalar>;
};

template <typename T>
concept has_modifiable_constraint_rhs =
    requires(T & model, T::constraint c, T::scalar s) {
        { model.set_constraint_rhs(c, s) };
    };

template <typename T>
concept has_readable_constraints =
    has_readable_constraint_lhs<T> && has_readable_constraint_sense<T> &&
    has_readable_constraint_rhs<T> && requires(T & model, T::constraint c) {
        { model.get_constraint(c) } -> linear_constraint;
    };

///////////////////////////////////////////////////////////////////////////////
///////////////////////////// Special constraints /////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename T>
concept has_sos1_constraints = requires(
    T & model, std::initializer_list<typename T::variable> init_variables) {
    { model.add_sos1_constraint(detail::dummy_range<typename T::variable>()) }
            -> std::same_as<typename T::constraint>;
    { model.add_sos1_constraint(init_variables) }
            -> std::same_as<typename T::constraint>;
};

template <typename T>
concept has_sos2_constraints = requires(
    T & model, std::initializer_list<typename T::variable> init_variables) {
    { model.add_sos2_constraint(detail::dummy_range<typename T::variable>()) }
            -> std::same_as<typename T::constraint>;
    { model.add_sos2_constraint(init_variables) }
            -> std::same_as<typename T::constraint>;
};

template <typename T>
concept has_indicator_constraints = requires(T & model, T::variable v) {
    { model.add_indicator_constraint(v, true, 
                                     detail::dummy_linear_constraint<T>()) }
            -> std::same_as<typename T::constraint>;
};

///////////////////////////////////////////////////////////////////////////////
////////////////////////////// Column generation //////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename T>
concept has_add_column = requires(
    T & model, T::scalar s,
    std::initializer_list<std::pair<typename T::constraint, typename T::scalar>>
        init_entries) {
    { model.add_column(detail::dummy_range<
                std::pair<typename T::constraint, typename T::scalar>>()) }
            -> std::same_as<typename T::variable>;
    { model.add_column(detail::dummy_range<
                std::pair<typename T::constraint, typename T::scalar>>(),
                       {.obj_coef = s, .lower_bound = s, .upper_bound = s}) }
            -> std::same_as<typename T::variable>;
    { model.add_column(init_entries) } -> std::same_as<typename T::variable>;
    { model.add_column(init_entries,
                       {.obj_coef = s, .lower_bound = s, .upper_bound = s}) }
            -> std::same_as<typename T::variable>;
};

template <typename T>
concept has_remove_variable = requires(T & model, T::variable v) {
    { model.remove_variable(v) };
    { model.remove_variables(detail::dummy_range<typename T::variable>()) };
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

template <typename S>
concept lp_basis_status = requires(const S & status) {
    { std::visit(detail::overloaded{
                       [](basis_status::basic) {},
                       [](basis_status::nonbasic) {}
                   },
                   status) };
};

template <typename T>
using model_basis_t = std::decay_t<decltype(std::declval<T>().get_basis())>;

template <typename T>
concept has_lp_basis =
    requires(T & model, typename T::basis b, T::variable v, T::constraint c) {
        { model.get_basis() } -> std::same_as<typename T::basis>;
        // variables
        { b.is_basic(v) } -> std::same_as<bool>;
        { b.get_status(v) } -> lp_basis_status;
        // constraints
        { b.is_basic(c) } -> std::same_as<bool>;
        { b.get_status(c) } -> lp_basis_status;
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
    has_lp_basis<T> && requires(T & model, typename T::basis b, T::variable v,
                                T::constraint c, T::scalar s) {
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

template <typename T>
concept has_mip_start =
        requires(T & model,
    std::initializer_list<std::pair<typename T::variable, typename T::scalar>>
        init_entries) {
    { model.add_mip_start(detail::dummy_range<
                std::pair<typename T::variable, typename T::scalar>>()) };
    { model.add_mip_start(init_entries) };
};

// template <typename T>
// concept has_multiple_mip_starts =
//         requires(T & model,
//     std::initializer_list<std::pair<typename T::variable, typename T::scalar>>
//         init_entries, std::size_t i) {
//     { model.add_mip_start(detail::dummy_range<
//                 std::pair<typename T::variable, typename T::scalar>>()) } -> std::same_as<std::size_t>;
//     { model.add_mip_start(init_entries) } -> std::same_as<std::size_t>;
//     { model.num_mip_starts() } -> std::same_as<std::size_t>;
//     { model.set_mip_start(i, detail::dummy_range<
//                 std::pair<typename T::variable, typename T::scalar>>()) };
//     { model.set_mip_start(i, init_entries) };
// };

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Callbacks //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename T>
concept has_candidate_solution_callback =
        requires(T & model, T::candidate_solution_callback_handle) {
    { model.set_candidate_solution_callback(
        [](T::candidate_solution_callback_handle &) {}) };
};

// template <typename T>
// concept has_node_relaxation_callback =
//     requires(T & model, T::node_relaxation_callback_handle) {
//         { model.set_node_relaxation_callback(
//             [](T::node_relaxation_callback_handle & h) {}) };
//     };

///////////////////////////////////////////////////////////////////////////////
//////////////////////////// Tolerance parameters /////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename T>
concept has_feasibility_tolerance =
        requires(T & model, T::variable v, T::constraint c, T::scalar s) {
    { model.get_feasibility_tolerance() }
            -> std::same_as<typename T::scalar>;
    { model.set_feasibility_tolerance(s) };
};

template <typename T>
concept has_optimality_tolerance =
        requires(T & model, T::variable v, T::constraint c, T::scalar s) {
    { model.get_optimality_tolerance() }
            -> std::same_as<typename T::scalar>;
    { model.set_optimality_tolerance(s) };
};

template <typename T>
concept has_integrality_tolerance =
        requires(T & model, T::variable v, T::constraint c, T::scalar s) {
    { model.get_integrality_tolerance() } 
            -> std::same_as<typename T::scalar>;
    { model.set_integrality_tolerance(s) };
};
// clang-format on

}  // namespace mippp

#endif  // MIPPP_MODEL_CONCEPTS_HPP