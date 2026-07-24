#pragma once

#include <limits>
#include <optional>
#include <ranges>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/soplex/v6/soplex_api.hpp"

namespace mippp {
namespace soplex::v6 {

class soplex_lp {
public:
    using variable_id = int;
    using constraint_id = int;
    using scalar = double;
    using variable = model_variable<variable_id, scalar>;
    using constraint = model_constraint<constraint_id>;
    template <typename Map>
    struct variable_mapping : entity_mapping<variable, Map> {
        variable_mapping(Map && t)
            : entity_mapping<variable, Map>(std::move(t)) {}
    };
    template <typename Map>
    struct constraint_mapping : entity_mapping<constraint, Map> {
        constraint_mapping(Map && t)
            : entity_mapping<constraint, Map>(std::move(t)) {}
    };

    struct variable_params {
        scalar obj_coef = scalar{0};
        std::optional<scalar> lower_bound = std::nullopt;
        std::optional<scalar> upper_bound = std::nullopt;
    };

    static constexpr variable_params default_variable_params = {
        .obj_coef = 0, .lower_bound = 0, .upper_bound = std::nullopt};

private:
    const soplex_api * SoPlex;
    void * model;
    double objective_offset;
    std::vector<double> tmp_scalars;

public:
    [[nodiscard]] explicit soplex_lp(const soplex_api & api)
        : SoPlex(&api), model(SoPlex->create()), objective_offset(0.0) {}
    ~soplex_lp() {
        if(model) SoPlex->free(model);
    }

    constexpr soplex_lp(const soplex_lp &) = delete;
    constexpr soplex_lp(soplex_lp && other) noexcept
        : SoPlex(other.SoPlex)
        , model(other.model)
        , objective_offset(other.objective_offset)
        , tmp_scalars(std::move(other.tmp_scalars)) {
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
    // std::size_t num_entries() {
    //     return static_cast<std::size_t>(SoPlex->getNumElements(model));
    // }
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
                           params.lower_bound.value_or(-1e100),
                           params.upper_bound.value_or(1e100));
    }

public:
    variable add_variable(
        const variable_params params = default_variable_params) {
        int var_id = static_cast<int>(num_variables());
        _add_var(params);
        return variable(var_id);
    }
    auto add_variables(std::size_t count,
                       const variable_params params = default_variable_params) {
        const std::size_t offset = num_variables();
        for(std::size_t i = 0; i < count; ++i) _add_var(params);
        return variables_view(
            std::from_range,
            std::views::transform(
                std::views::iota(static_cast<variable_id>(offset),
                                 static_cast<variable_id>(offset + count)),
                [](auto && i) { return variable{i}; }));
    }
    template <typename IL>
    auto add_variables(
        std::size_t count, IL && id_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = num_variables();
        for(std::size_t i = 0; i < count; ++i) _add_var(params);
        return variables_view(
            typename detail::function_traits<IL>::arg_types(),
            std::views::transform(
                std::views::iota(static_cast<variable_id>(offset),
                                 static_cast<variable_id>(offset + count)),
                [](auto && i) { return variable{i}; }),
            std::forward<IL>(id_lambda));
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
                           params.lower_bound.value_or(-1e100),
                           params.upper_bound.value_or(1e100));
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
    void _add_constraint(linear_constraint auto && lc) {
        int num_nz = 0;
        std::fill(tmp_scalars.begin(), tmp_scalars.end(), 0.0);
        for(auto && [var, coef] : lc.linear_terms()) {
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
        _add_constraint(std::forward<LC>(lc));
        return constraint(constr_id);
    }
    template <linear_constraint LC>
    constraint add_constraint(distinct_variables_t, LC && lc) {
        return add_constraint(std::forward<LC>(lc));
    }

private:
    template <typename Key, typename LastConstrLambda>
        requires linear_constraint<std::invoke_result_t<LastConstrLambda, Key>>
    void _add_first_valued_constraint(const Key & key,
                                      LastConstrLambda & lc_lambda) {
        _add_constraint(lc_lambda(key));
    }
    template <typename Key, typename OptConstrLambda, typename... Tail>
        requires detail::optional_type<
                     std::invoke_result_t<OptConstrLambda, Key>> &&
                 linear_constraint<detail::optional_type_value_t<
                     std::invoke_result_t<OptConstrLambda, Key>>>
    void _add_first_valued_constraint(const Key & key,
                                      OptConstrLambda & opt_lc_lambda,
                                      Tail &... tail) {
        if(const auto & opt_lc = opt_lc_lambda(key)) {
            _add_constraint(opt_lc.value());
            return;
        }
        _add_first_valued_constraint(key, tail...);
    }

public:
    template <std::ranges::range IR, typename... CL>
    auto add_constraints(IR && keys, CL &&... constraint_lambdas) {
        tmp_scalars.resize(num_variables());
        const int offset = static_cast<int>(num_constraints());
        int constr_id = offset;
        for(auto && key : keys) {
            _add_first_valued_constraint(key, constraint_lambdas...);
            ++constr_id;
        }
        return constraints_range(
            std::forward<IR>(keys),
            std::views::transform(std::views::iota(offset, constr_id),
                                  [](auto && i) { return constraint{i}; }));
    }
    template <std::ranges::range IR, typename... CL>
    auto add_constraints(distinct_variables_t, IR && keys,
                         CL &&... constraint_lambdas) {
        return add_constraints(std::forward<IR>(keys),
                              std::forward<CL>(constraint_lambdas)...);
    }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////// Solve status ///////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    // clang-format off
private:
    using status_variant = std::variant<
            status::unknown,  // default value
            status::optimal,
            status::optimal_infeasible_unscaled,
            status::infeasible_or_unbounded,
            status::infeasible,
            status::unbounded,
            status::failed>;

    status_variant _status;
    // clang-format on
public:
    const status_variant & solve_status() const { return _status; }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////////// Solve //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void solve() {
        using namespace status;
        if(num_variables() == 0u) {
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
};

}  // namespace soplex::v6
}  // namespace mippp
