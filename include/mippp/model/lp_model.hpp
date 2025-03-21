#ifndef MIPPP_LP_MODEL_HPP
#define MIPPP_LP_MODEL_HPP

#include <concepts>

#include <range/v3/view/single.hpp>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_variable.hpp"

namespace fhamonic {
namespace mippp {

namespace detail {

template <typename M>
struct dummy_expression {
    auto linear_terms() const {
        return ranges::views::single(
            std::make_pair(typename M::variable_id{}, typename M::scalar{}));
    }
    auto constant() const { return typename M::scalar{}; }
};
template <typename M>
struct dummy_constraint {
    auto expression() const { return dummy_expression<M>(); }
    auto relation() const { return constraint_relation::equal_zero; }
};
struct dummy_type {};

}  // namespace detail

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
    // { model.add_variables(std::size_t{1u}, [](dummy_type) { return 0; },
    //                      {.obj_coef = s, .lower_bound = s, .upper_bound = s}) }
    //         -> std::invocable<detail::dummy_type>;

    { model.set_objective(detail::dummy_expression<T>()) };
    { model.add_constraint(detail::dummy_constraint<T>()) }
            -> std::convertible_to<typename T::constraint>;

    { model.num_variables() } -> std::same_as<std::size_t>;
    { model.num_constraints() } -> std::same_as<std::size_t>;
    { model.num_entries() } -> std::same_as<std::size_t>;

    { model.optimize() };

    { model.get_objective_value() } -> std::same_as<typename T::scalar>;
    { model.get_primal_solution() } /*-> input_mapping<typename T::variable>*/;
    { model.get_dual_solution() } /*-> input_mapping<typename T::constraint>*/;
};

template <typename T>
concept updatable_lp_model =
    lp_model<T> &&
    requires(T & model, T::variable v, T::constraint c, T::scalar s) {
        { model.set_objective_coefficient(v, s) };
        { model.get_objective_coefficient(v) }
                -> std::convertible_to<typename T::scalar>;
        { model.add_objective(detail::dummy_expression<T>()) };
        { model.get_objective() } -> linear_expression;

        { model.set_variable_lower_bound(v, s) };
        { model.get_variable_lower_bound(v) }
                -> std::convertible_to<typename T::scalar>;
        { model.set_variable_upper_bound(v, s) };
        { model.get_variable_upper_bound(v) }
                -> std::convertible_to<typename T::scalar>;

        { model.set_constraint_sense(c, constraint_relation::equal_zero) };
        { model.get_constraint_sense(c) } -> std::same_as<constraint_relation>;
        { model.set_constraint_rhs(c, s) };
        { model.get_constraint_rhs(c) }
                -> std::convertible_to<typename T::scalar>;
        // { model.get_constraint_lhs(c) } -> ranges::range;
        // { model.get_constraint(c) } -> linear_constraint;
    };



template <typename T>
concept has_column_generation =
    requires(T & model, T::constraint c, T::scalar s) {
        { model.add_column(std::vector<std::pair<typename T::constraint, typename T::scalar>>()) }
                 -> std::same_as<typename T::variable>;
    };

template <typename T>
concept has_column_deletion =
    requires(T & model, T::variable v) {
        { model.remove_column(v) };
    };

template <typename T>
concept has_feasability_tolerance =
    requires(T & model, T::variable v, T::constraint c, T::scalar s) {
        { model.get_feasability_tolerance() }
                -> std::convertible_to<typename T::scalar>;
        { model.set_feasability_tolerance(s) };
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

enum basis_status : int {
    basic = 0,
    nonbasic_at_lb = 1,
    nonbasic_at_ub = 2,
    nonbasic_free = 3
};

template <typename T>
concept has_lp_basis_warm_start =
    lp_model<T> &&
    requires(T & model, T::variable v, T::constraint c, T::scalar s) {
        { model.get_variable_basis_status(v) }
                -> std::same_as<basis_status>;
        { model.set_variable_basis_status(v, basis_status::basic) };

        { model.get_constraint_basis_status(c) }
                -> std::same_as<basis_status>;
        { model.set_constraint_basis_status(c, basis_status::basic) };
    };

// clang-format on

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_LP_MODEL_HPP