#ifndef MIPPP_MODEL_CONCEPTS_HPP
#define MIPPP_MODEL_CONCEPTS_HPP

#include <concepts>
#include <optional>
#include <string>

#include <range/v3/view/single.hpp>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_entities.hpp"

namespace fhamonic::mippp {

namespace detail {

///////////////////////////////////////////////////////////////////////////////
////////////////////////// Dummy types for concepts ///////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename T>
auto dummy_range() {
    return ranges::views::empty<T>;
};

template <typename M>
struct dummy_expression {
    auto linear_terms() const {
        return dummy_range<
            std::pair<typename M::variable, typename M::scalar>>();
    }
    auto constant() const { return typename M::scalar{}; }
};
template <typename M>
struct dummy_constraint {
    auto linear_terms() const {
        return dummy_range<
            std::pair<typename M::variable, typename M::scalar>>();
    }
    auto sense() const { return constraint_sense::equal; }
    auto rhs() const { return typename M::scalar{0}; }
};
struct dummy_type {};

constexpr bool operator<(dummy_type, dummy_type) { return true; }

}  // namespace detail

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// Model ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// clang-format off
template <typename T>
concept lp_model = requires(T & model, T::variable v, 
                    T::variable_params vparams, T::constraint c, T::scalar s) {
    { model.set_maximization() };
    { model.set_minimization() };

    { model.add_variable() } -> std::same_as<typename T::variable>;
    { model.add_variable({.obj_coef = s, .lower_bound = s, .upper_bound = s}) }
            -> std::same_as<typename T::variable>;    
    { model.add_variables(std::size_t{1u}) } -> ranges::random_access_range;
    { model.add_variables(std::size_t{1u},
                        {.obj_coef = s, .lower_bound = s, .upper_bound = s}) }
            -> ranges::random_access_range;
    { model.add_variables(std::size_t{1u}, [](detail::dummy_type) { return 0; }) }
            -> ranges::random_access_range;
    { model.add_variables(std::size_t{1u}, [](detail::dummy_type) { return 0; },
                         {.obj_coef = s, .lower_bound = s, .upper_bound = s}) }
            -> ranges::random_access_range;

    { model.set_objective_offset(0.0) };
    { model.set_objective(detail::dummy_expression<T>()) };
    { model.add_constraint(detail::dummy_constraint<T>()) }
            -> std::same_as<typename T::constraint>;
    { model.add_constraints(detail::dummy_range<detail::dummy_type>(),
            [](detail::dummy_type) { return detail::dummy_constraint<T>(); }) };

    { model.num_variables() } -> std::same_as<std::size_t>;
    { model.num_constraints() } -> std::same_as<std::size_t>;

    { model.solve() };
    { model.get_solution_value() } -> std::same_as<typename T::scalar>;
    { model.get_solution() } /*-> input_mapping<typename T::variable>*/;
};

template <typename T>
concept milp_model = lp_model<T> && requires(T & model, T::variable v,
                            T::variable_params vparams, T::scalar s) {
    { model.add_integer_variable() } -> std::same_as<typename T::variable>;
    { model.add_integer_variable(
                        {.obj_coef = s, .lower_bound = s, .upper_bound = s}) }
            -> std::same_as<typename T::variable>;
    { model.add_integer_variables(std::size_t{1u}) }
            -> ranges::random_access_range;
    { model.add_integer_variables(std::size_t{1u},
                        {.obj_coef = s, .lower_bound = s, .upper_bound = s}) }
            -> ranges::random_access_range;
    { model.add_integer_variables(std::size_t{1u}, 
                                  [](detail::dummy_type) { return 0; }) }
            -> ranges::random_access_range;
    { model.add_integer_variables(std::size_t{1u}, 
                        [](detail::dummy_type) { return 0; },
                        {.obj_coef = s, .lower_bound = s, .upper_bound = s}) }
            -> ranges::random_access_range;

    { model.add_binary_variable() } -> std::same_as<typename T::variable>;
    { model.add_binary_variables(std::size_t{1u}) }
            -> ranges::random_access_range;
    { model.add_binary_variables(std::size_t{1u}, 
                                 [](detail::dummy_type) { return 0; }) }
            -> ranges::random_access_range;

    { model.set_continuous(v) };
    { model.set_integer(v) };
    { model.set_binary(v) };
};

template <typename T>
concept sized_model = lp_model<T> && requires(T & model) {
    { model.num_entries() } -> std::same_as<std::size_t>;
};

template <typename T>
concept has_lp_status =
    lp_model<T> &&
    requires(T & model) {
        { model.is_optimal() } -> std::same_as<bool>;
        { model.is_infeasible() } -> std::same_as<bool>;
        { model.is_unbounded() } -> std::same_as<bool>;
    };

template <typename T>
concept has_dual_solution = requires(T & model) {
    { model.get_dual_solution() } /*-> input_mapping<typename T::constraint>*/;
};

// template <typename T>
// concept has_milp_status =
//     milp_model<T> &&
//     requires(T & model, T::variable v, T::variable_id vid,
//              T::variable_params vparams, T::constraint c, T::scalar s) {
//         { model.get_milp_status() };
//     };


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
            -> ranges::random_access_range;
    { model.add_named_variables(std::size_t{1u},
                        [](std::size_t) -> std::string { return ""; },
                        {.obj_coef = s, .lower_bound = s, .upper_bound = s}) }
            -> ranges::random_access_range;  
    { model.add_named_variables(std::size_t{1u},
                        [](detail::dummy_type) { return 0; },
                        [](detail::dummy_type) -> std::string { return ""; }) }
            -> ranges::random_access_range;
    { model.add_named_variables(std::size_t{1u},
                        [](detail::dummy_type) { return 0; },
                        [](detail::dummy_type) -> std::string { return ""; },
                        {.obj_coef = s, .lower_bound = s, .upper_bound = s}) }
            -> ranges::random_access_range; 
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
concept has_readable_objective =
    requires(T & model, T::variable v) {
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
        { model.add_objective(detail::dummy_expression<T>()) };
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
concept has_readable_constraint_lhs =
    requires(T & model, T::constraint c) {
        { model.get_constraint_lhs(c) } -> ranges::range;
    } && std::same_as<
            std::decay_t<ranges::value_type_t<constraint_lhs_range_t<T>>>,
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
////////////////////////////// Column generation //////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename T>
concept has_add_column = requires(
    T & model, T::constraint c, T::scalar s,
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
concept has_column_deletion = requires(T & model, T::variable v) {
    { model.remove_column(v) };
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////// Warm start //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

enum basis_status : int {
    basic = 0,
    nonbasic_at_lb = 1,
    nonbasic_at_ub = 2,
    nonbasic_free = 3
};

template <typename T>
concept has_lp_basis_warm_start =
    requires(T & model, T::variable v, T::constraint c, T::scalar s) {
        { model.get_variable_basis_status(v) } -> std::same_as<basis_status>;
        { model.set_variable_basis_status(v, basis_status::basic) };

        { model.get_constraint_basis_status(c) } -> std::same_as<basis_status>;
        { model.set_constraint_basis_status(c, basis_status::basic) };
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
    { model.add_indicator_constraint(v, true, detail::dummy_constraint<T>()) }
            -> std::same_as<typename T::constraint>;
};

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Callbacks //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename T>
concept has_candidate_solution_callback =
    requires(T & model, T::candidate_solution_callback_handle) {
        { model.set_candidate_solution_callback(
            [](T::candidate_solution_callback_handle & h) {}) };
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

}  // namespace fhamonic::mippp

#endif  // MIPPP_MODEL_CONCEPTS_HPP