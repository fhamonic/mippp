#pragma once

#include <algorithm>
#include <cstddef>
#include <limits>
#include <memory>
#include <optional>
#include <ranges>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "mippp/detail/invoke_key.hpp"
#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/model_base.hpp"
#include "mippp/solvers/soplex/impl/v1/soplex_api.hpp"

namespace mippp {
namespace soplex::impl::v1 {

class soplex_lp : protected model_base<int, double> {
private:
    // soplex::infinity, which the C interface does not export
    static constexpr double _infinity = 1e100;

protected:
    using variable_id = int;
    using constraint_id = int;

public:
    using model_base<int, double>::default_variable_params;
    double infinity() const noexcept { return _infinity; }
    using model_base<int, double>::is_infinite;

private:
    const soplex_api * SoPlex;
    void * model;
    double objective_offset;

public:
    [[nodiscard]] soplex_lp() : soplex_lp(soplex_api::load()) {}
    [[nodiscard]] explicit soplex_lp(const soplex_api & api)
        : SoPlex(&api), model(SoPlex->create()), objective_offset(0.0) {}
    ~soplex_lp() {
        if(model) SoPlex->free(model);
    }

    constexpr soplex_lp(const soplex_lp &) = delete;
    constexpr soplex_lp(soplex_lp && other) noexcept
        : model_base<int, double>(std::move(other))
        , SoPlex(other.SoPlex)
        , model(other.model)
        , objective_offset(other.objective_offset)
        , _status(other._status) {
        other.model = nullptr;
    }

    constexpr soplex_lp & operator=(const soplex_lp &) = delete;
    constexpr soplex_lp & operator=(soplex_lp && other) = delete;

    std::size_t num_variables() {
        return static_cast<std::size_t>(SoPlex->numCols(model));
    }
    std::size_t num_constraints() {
        return static_cast<std::size_t>(SoPlex->numRows(model));
    }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////// Native handles /////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
public:
    const soplex_api & native_api() const noexcept { return *SoPlex; }
    void * native_model() const noexcept { return model; }
    int native_id(variable v) const noexcept { return v.id(); }
    int native_id(constraint c) const noexcept { return c.id(); }

public:
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// Objective ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void set_maximization() { SoPlex->setIntParam(model, 0, 1); }
    void set_minimization() { SoPlex->setIntParam(model, 0, -1); }

    void set_objective_offset(double constant) { objective_offset = constant; }
    void set_objective(linear_expression auto && le) {
        auto num_vars = num_variables();
        tmp_scalars.resize(num_vars);
        std::fill(tmp_scalars.begin(), tmp_scalars.end(), 0.0);
        for(auto && [var, coef] : le.linear_terms()) {
            tmp_scalars[var.uid()] += coef;
        }
        SoPlex->changeObjReal(model, tmp_scalars.data(),
                              static_cast<int>(num_vars));
        set_objective_offset(le.constant());
    }
    template <linear_expression LE>
    void set_objective(distinct_variables_t, LE && le) {
        set_objective(std::forward<LE>(le));
    }
    double get_objective_offset() { return objective_offset; }
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// Variables ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    inline void _add_var(const variable_params & params) {
        SoPlex->addColReal(model, nullptr, 0, 0, params.obj_coef,
                           params.lower_bound.value_or(-_infinity),
                           params.upper_bound.value_or(_infinity));
    }

private:
public:
    friend model_base<int, double>;
    using model_base<int, double>::add_variable;
    using model_base<int, double>::add_variables;
    using model_base<int, double>::add_named_variable;
    using model_base<int, double>::add_named_variables;

private:
    std::size_t _new_variables(std::size_t count,
                               const variable_params & params, variable_kind) {
        const std::size_t offset = num_variables();
        for(std::size_t i = 0; i < count; ++i) _add_var(params);
        return offset;
    }

private:
    template <typename ER>
    inline variable _add_column(ER && entries, const variable_params & params) {
        const auto num_vars = num_variables();
        tmp_scalars.resize(num_constraints());
        std::fill(tmp_scalars.begin(), tmp_scalars.end(), 0.0);
        int num_nz = 0;
        for(auto && [constr, coef] : entries) {
            if(coef == 0) continue;
            tmp_scalars[constr.uid()] += static_cast<scalar>(coef);
            num_nz += (tmp_scalars[constr.uid()] != 0) ? 1 : -1;
        }
        SoPlex->addColReal(model, tmp_scalars.data(), num_nz,
                           static_cast<int>(num_vars), params.obj_coef,
                           params.lower_bound.value_or(-_infinity),
                           params.upper_bound.value_or(_infinity));
        return variable(static_cast<int>(num_vars));
    }

public:
    template <std::ranges::range ER>
    variable add_column(
        ER && entries, const variable_params params = default_variable_params) {
        return _add_column(entries, params);
    }
    variable add_column(
        std::initializer_list<std::pair<constraint, scalar>> entries,
        const variable_params params = default_variable_params) {
        return _add_column(entries, params);
    }
    ///////////////////////////////////////////////////////////////////////////
    /////////////////////////////// Constraints ///////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    // the dense row merges a repeat for free; the tagged form still asserts
    // it so that code developed here does not abort on GLPK
    template <bool distinct>
    void _add_constraint(linear_constraint auto && lc) {
        int num_nz = 0;
        std::fill(tmp_scalars.begin(), tmp_scalars.end(), 0.0);
        if constexpr(distinct) _begin_distinct_check();
        for(auto && [var, coef] : lc.linear_terms()) {
            if constexpr(distinct) _check_distinct(var.id());
            if(coef == 0) continue;
            tmp_scalars[var.uid()] += coef;
            num_nz += (tmp_scalars[var.uid()] != 0) ? 1 : -1;
        }
        const double b = lc.rhs();
        SoPlex->addRowReal(model, tmp_scalars.data(),
                           static_cast<int>(num_variables()), num_nz,
                           (lc.sense() == constraint_sense::less_equal)
                               ? -std::numeric_limits<double>::infinity()
                               : b,
                           (lc.sense() == constraint_sense::greater_equal)
                               ? std::numeric_limits<double>::infinity()
                               : b);
    }

public:
    template <linear_constraint LC>
    constraint add_constraint(LC && lc) {
        constraint_id constr_id = static_cast<constraint_id>(num_constraints());
        tmp_scalars.resize(num_variables());
        _add_constraint<false>(std::forward<LC>(lc));
        return constraint(constr_id);
    }
    template <linear_constraint LC>
    constraint add_constraint(distinct_variables_t, LC && lc) {
        constraint_id constr_id = static_cast<constraint_id>(num_constraints());
        tmp_scalars.resize(num_variables());
        _add_constraint<true>(std::forward<LC>(lc));
        return constraint(constr_id);
    }

private:
    template <bool distinct, typename Key, typename LastConstrLambda>
        requires linear_constraint<
            detail::key_invoke_result_t<LastConstrLambda &, const Key &>>
    void _add_first_valued_constraint(const Key & key,
                                      LastConstrLambda & lc_lambda) {
        _add_constraint<distinct>(detail::invoke_key(lc_lambda, key));
    }
    template <bool distinct, typename Key, typename OptConstrLambda,
              typename... Tail>
        requires detail::optional_type<detail::key_invoke_result_t<
                     OptConstrLambda &, const Key &>> &&
                 linear_constraint<
                     detail::optional_type_value_t<detail::key_invoke_result_t<
                         OptConstrLambda &, const Key &>>>
    void _add_first_valued_constraint(const Key & key,
                                      OptConstrLambda & opt_lc_lambda,
                                      Tail &... tail) {
        if(const auto & opt_lc = detail::invoke_key(opt_lc_lambda, key)) {
            _add_constraint<distinct>(opt_lc.value());
            return;
        }
        _add_first_valued_constraint<distinct>(key, tail...);
    }
    template <bool distinct, std::ranges::range IR, typename... CL>
    auto _add_constraints(IR && keys, CL &... constraint_lambdas) {
        tmp_scalars.resize(num_variables());
        const int offset = static_cast<int>(num_constraints());
        int constr_id = offset;
        for(auto && key : keys) {
            _add_first_valued_constraint<distinct>(key, constraint_lambdas...);
            ++constr_id;
        }
        return detail::keyed_entities(
            *this, keys, detail::set_constraint_name,
            entity_range(constraint{offset},
                         static_cast<std::size_t>(constr_id - offset)));
    }

public:
    template <std::ranges::range IR, typename... CL>
    auto add_constraints(IR && keys, CL &&... constraint_lambdas) {
        return _add_constraints<false>(std::forward<IR>(keys),
                                       constraint_lambdas...);
    }
    template <std::ranges::range IR, typename... CL>
    auto add_constraints(distinct_variables_t, IR && keys,
                         CL &&... constraint_lambdas) {
        return _add_constraints<true>(std::forward<IR>(keys),
                                      constraint_lambdas...);
    }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////// Solve status ///////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    // clang-format off
private:
    using status_variant = std::variant<
            status::unknown,
            status::optimal,
            status::optimal_infeasible_unscaled,
            status::infeasible_or_unbounded,
            status::infeasible,
            status::unbounded,
            status::failed>;

    status_variant _status = status::unknown{};
    // clang-format on
public:
    const status_variant & get_status() const { return _status; }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////////// Solve //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void solve() {
        using namespace status;
        if(num_variables() == 0u) {
            _status = status::unknown{};
            return;
        }
        switch(SoPlex->optimize(model)) {
            case OPTIMAL:
                _status.emplace<optimal>();
                return;
            case OPTIMAL_UNSCALED_VIOLATIONS:
                _status.emplace<optimal_infeasible_unscaled>();
                return;
            case INForUNBD:
                _status.emplace<infeasible_or_unbounded>();
                return;
            case INFEASIBLE:
                _status.emplace<infeasible>();
                return;
            case UNBOUNDED:
                _status.emplace<unbounded>();
                return;
            case ERROR_:
            case NO_RATIOTESTER:
            case NO_PRICER:
            case NO_SOLVER:
            case NOT_INIT:
            case ABORT_CYCLING:
            case ABORT_TIME:
            case ABORT_ITER:
            case ABORT_VALUE:
                _status.emplace<failed>();
                return;
            case SINGULAR:
            case NO_PROBLEM:
            case REGULAR:
            case RUNNING:
            case UNKNOWN:
            default:
                _status.emplace<unknown>();
        }
    }
    double get_solution_value() {
        return objective_offset + SoPlex->objValueReal(model);
    }
    auto get_solution() {
        auto num_vars = num_variables();
        auto solution = std::make_unique_for_overwrite<double[]>(num_vars);
        SoPlex->getPrimalReal(model, solution.get(),
                              static_cast<int>(num_vars));
        return variable_mapping(std::move(solution));
    }
    auto get_dual_solution() {
        auto num_constrs = num_constraints();
        auto solution = std::make_unique_for_overwrite<double[]>(num_constrs);
        SoPlex->getDualReal(model, solution.get(),
                            static_cast<int>(num_constrs));
        return constraint_mapping(std::move(solution));
    }
    // Copy the original-row Farkas multipliers before the model is destroyed.
    // Both runtime capability and certificate availability are optional. This
    // accessor never reoptimizes, changes presolve, or consumes a hidden solve.
    std::optional<std::vector<scalar>> get_infeasibility_ray() {
        if(!std::holds_alternative<status::infeasible>(_status) ||
           !SoPlex->hasDualFarkas || !SoPlex->getDualFarkasReal ||
           !SoPlex->hasDualFarkas(model))
            return std::nullopt;
        std::vector<scalar> ray(num_constraints());
        if(!SoPlex->getDualFarkasReal(model, ray.data(),
                                      static_cast<int>(ray.size())))
            return std::nullopt;
        return ray;
    }
};

}  // namespace soplex::impl::v1
}  // namespace mippp
