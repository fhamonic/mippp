#ifndef MIPPP_MODEL_CONCEPTS_HPP
#define MIPPP_MODEL_CONCEPTS_HPP

#include <concepts>
#include <optional>

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

enum lp_status : char { optimal = 0, infeasible = 1, unbounded = 2 };

// clang-format off
template <typename T>
concept lp_model = requires(T & model, T::variable v, T::variable_id vid,
                            T::variable_params vparams, T::constraint c,
                            T::scalar s) {
    { model.set_maximization() };
    { model.set_minimization() };

    { model.add_variable() } -> std::convertible_to<typename T::variable>;
    { model.add_variable({.obj_coef = s, .lower_bound = s, .upper_bound = s}) }
            -> std::convertible_to<typename T::variable>;    
    { model.add_variables(std::size_t{1u}) } -> ranges::random_access_range;
    { model.add_variables(std::size_t{1u},
                        {.obj_coef = s, .lower_bound = s, .upper_bound = s}) }
            -> ranges::random_access_range;
    { model.add_variables(std::size_t{1u}, [](detail::dummy_type) { return 0; }) }
            -> ranges::random_access_range;
    { model.add_variables(std::size_t{1u}, [](detail::dummy_type) { return 0; },
                         {.obj_coef = s, .lower_bound = s, .upper_bound = s}) }
            -> ranges::random_access_range;

    { model.set_objective(detail::dummy_expression<T>()) };
    { model.add_constraint(detail::dummy_constraint<T>()) }
            -> std::convertible_to<typename T::constraint>;
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
    { model.add_integer_variable() } -> std::convertible_to<typename T::variable>;
    { model.add_integer_variable({.obj_coef = s, .lower_bound = s, .upper_bound = s}) }
            -> std::convertible_to<typename T::variable>;
    { model.add_integer_variables(std::size_t{1u}) } -> ranges::random_access_range;
    { model.add_integer_variables(std::size_t{1u},
                        {.obj_coef = s, .lower_bound = s, .upper_bound = s}) }
            -> ranges::random_access_range;
    { model.add_integer_variables(std::size_t{1u}, [](detail::dummy_type) { return 0; }) }
            -> ranges::random_access_range;
    { model.add_integer_variables(std::size_t{1u}, [](detail::dummy_type) { return 0; },
                            {.obj_coef = s, .lower_bound = s, .upper_bound = s}) }
            -> ranges::random_access_range;

    { model.add_binary_variable() } -> std::convertible_to<typename T::variable>;
    { model.add_binary_variables(std::size_t{1u}) } -> ranges::random_access_range;
    { model.add_binary_variables(std::size_t{1u}, [](detail::dummy_type) { return 0; }) }
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
    requires(T & model, T::variable v, T::variable_id vid,
             T::variable_params vparams, T::constraint c, T::scalar s) {
        { model.get_lp_status() }
                -> std::same_as<std::optional<lp_status>>;
    };

template <typename T>
concept has_dual_solution = requires(T & model) {
    { model.get_dual_solution() } /*-> input_mapping<typename T::constraint>*/;
};

template <typename T>
concept has_milp_status =
    milp_model<T> &&
    requires(T & model, T::variable v, T::variable_id vid,
             T::variable_params vparams, T::constraint c, T::scalar s) {
        { model.get_milp_status() };
    };

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Objective //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename T>
concept has_readable_objective =
    requires(T & model, T::variable v) {
        { model.get_objective_coefficient(v) }
                -> std::convertible_to<typename T::scalar>;
        { model.get_objective() } -> linear_expression;
    };

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
                -> std::convertible_to<typename T::scalar>;
        { model.get_variable_upper_bound(v) }
                -> std::convertible_to<typename T::scalar>;
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
concept has_readable_constraint_lhs =
    requires(T & model, T::constraint c) {
        { model.get_constraint_lhs(c) } -> ranges::range;
    };

template <typename T>
concept has_modifiable_constraint_lhs = requires(T & model, T::constraint c) {
    {
        model.set_constraint_lhs(c,
                                 detail::dummy_range<std::pair<typename T::variable,
                                                          typename T::scalar>>())
    };
};

template <typename T>
concept has_readable_constraint_sense =
    requires(T & model, T::constraint c) {
        { model.get_constraint_sense(c) } -> std::same_as<constraint_sense>;
    };

template <typename T>
concept has_modifiable_constraint_sense =
    requires(T & model, T::constraint c) {
        { model.set_constraint_sense(c, constraint_sense::equal) };
    };        
                   
template <typename T>
concept has_readable_constraint_rhs =
    requires(T & model, T::constraint c) {
        { model.get_constraint_rhs(c) }
                -> std::convertible_to<typename T::scalar>;
    };

template <typename T>
concept has_modifiable_constraint_rhs =
    requires(T & model, T::constraint c, T::scalar s) {
        { model.set_constraint_rhs(c, s) };
    };

template <typename T>
concept has_readable_constraints =
    has_readable_constraint_lhs<T> &&
    has_readable_constraint_sense<T> &&
    has_readable_constraint_rhs<T> &&
    requires(T & model, T::constraint c) {
        { model.get_constraint(c) } -> linear_constraint;
    };
    
///////////////////////////////////////////////////////////////////////////////
////////////////////////////// Column generation //////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename T>
concept has_column_generation =
    requires(T & model, T::constraint c, T::scalar s) {
        {
            model.add_column(detail::dummy_range<std::pair<typename T::constraint,
                                                      typename T::scalar>>())
        } -> std::same_as<typename T::variable>;
    };

template <typename T>
concept has_column_deletion =
    requires(T & model, T::variable v) {
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
concept has_lp_basis_warm_start = lp_model<T> &&
    requires(T & model, T::variable v, T::constraint c, T::scalar s) {
        { model.get_variable_basis_status(v) }
                -> std::same_as<basis_status>;
        { model.set_variable_basis_status(v, basis_status::basic) };

        { model.get_constraint_basis_status(c) }
                -> std::same_as<basis_status>;
        { model.set_constraint_basis_status(c, basis_status::basic) };
    };

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// MILP features ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename T>
concept has_sos1_constraints = milp_model<T> &&
    requires(T & model) {
        { model.add_sos1_constraint(detail::dummy_range<typename T::variable>()) };
    };

template <typename T>
concept has_sos2_constraints = milp_model<T> &&
    requires(T & model) {
        { model.add_sos2_constraint(detail::dummy_range<typename T::variable>()) };
    };

template <typename T>
concept has_indicator_constraints = milp_model<T> &&
    requires(T & model, T::variable v) {
        { model.add_indicator_constraint(v, detail::dummy_constraint<T>()) };
    };

///////////////////////////////////////////////////////////////////////////////
//////////////////////////// Tolerance parameters /////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename T>
concept has_feasibility_tolerance =
    requires(T & model, T::variable v, T::constraint c, T::scalar s) {
        { model.get_feasibility_tolerance() }
                -> std::convertible_to<typename T::scalar>;
        { model.set_feasibility_tolerance(s) };
    };

template <typename T>
concept has_optimality_tolerance =
    requires(T & model, T::variable v, T::constraint c, T::scalar s) {
        { model.get_optimality_tolerance() }
                -> std::convertible_to<typename T::scalar>;
        { model.set_optimality_tolerance(s) };
    };

template <typename T>
concept has_integrality_tolerance =
    requires(T & model, T::variable v, T::constraint c, T::scalar s) {
        { model.get_integrality_tolerance() }
                -> std::convertible_to<typename T::scalar>;
        { model.set_integrality_tolerance(s) };
    };
// clang-format on

}  // namespace fhamonic::mippp

#endif  // MIPPP_MODEL_CONCEPTS_HPP