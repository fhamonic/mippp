#pragma once

#include <algorithm>
#include <cstring>
#include <ranges>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "mippp/detail/invoke_key.hpp"
#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/clp/v1_17/clp_api.hpp"
#include "mippp/solvers/model_base.hpp"

namespace mippp {
namespace clp::v1_17 {

class clp_lp : protected model_base<int, double> {
private:
    const clp_api * Clp;
    Clp_Simplex * model;

    std::vector<index> tmp_begins;
    std::vector<scalar> tmp_lower_bounds;
    std::vector<scalar> tmp_upper_bounds;

    std::vector<int> _free_variable_ids;

public:
    // the anchor model_variable_params_t deduces from
    using model_base<int, double>::default_variable_params;

    [[nodiscard]] clp_lp() : clp_lp(clp_api::load()) {}
    [[nodiscard]] explicit clp_lp(const clp_api & api)
        : model_base<int, double>(), Clp(&api), model(Clp->newModel()) {}
    ~clp_lp() {
        if(model) Clp->deleteModel(model);
    }

    constexpr clp_lp(const clp_lp &) = delete;
    constexpr clp_lp(clp_lp && other) noexcept
        : model_base<int, double>(std::move(other))
        , Clp(other.Clp)
        , model(other.model)
        , tmp_begins(std::move(other.tmp_begins))
        , tmp_lower_bounds(std::move(other.tmp_lower_bounds))
        , tmp_upper_bounds(std::move(other.tmp_upper_bounds))
        , _free_variable_ids(std::move(other._free_variable_ids)) {
        other.model = nullptr;
    }

    constexpr clp_lp & operator=(const clp_lp &) = delete;
    constexpr clp_lp & operator=(clp_lp && other) = delete;

    std::size_t num_variables() {
        return static_cast<std::size_t>(Clp->getNumCols(model)) -
               _free_variable_ids.size();
    }
    std::size_t num_constraints() {
        return static_cast<std::size_t>(Clp->getNumRows(model));
    }
    std::size_t num_entries() {
        return static_cast<std::size_t>(Clp->getNumElements(model));
    }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////// Native handles /////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
public:
    const clp_api & native_api() const noexcept { return *Clp; }
    Clp_Simplex * native_model() const noexcept { return model; }
    int native_id(variable v) const noexcept { return v.id(); }
    int native_id(constraint c) const noexcept { return c.id(); }

public:
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// Objective ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void set_maximization() { Clp->setObjSense(model, -1); }
    void set_minimization() { Clp->setObjSense(model, 1); }

    void set_objective_offset(scalar constant) {
        Clp->setObjectiveOffset(model, -constant);
    }
    void set_objective(linear_expression auto && le) {
        auto num_vars = num_native_ids_variables();
        scalar * objective = Clp->objective(model);
        std::fill(objective, objective + num_vars, 0.0);
        for(auto && [var, coef] : le.linear_terms()) {
            objective[var.id()] += coef;
        }
        set_objective_offset(le.constant());
    }
    template <linear_expression LE>
    void set_objective(distinct_variables_t, LE && le) {
        set_objective(std::forward<LE>(le));
    }
    void add_objective(linear_expression auto && le) {
        scalar * objective = Clp->objective(model);
        for(auto && [var, coef] : le.linear_terms()) {
            objective[var.id()] += coef;
        }
        set_objective_offset(get_objective_offset() + le.constant());
    }
    scalar get_objective_offset() { return -Clp->objectiveOffset(model); }
    auto get_objective() {
        auto num_vars = num_native_ids_variables();
        const scalar * objective = Clp->objective(model);
        return linear_expression_view(
            std::views::transform(
                std::views::iota(index{0}, static_cast<index>(num_vars)),
                [coefs = objective](auto i) {
                    return std::make_pair(variable(i), coefs[i]);
                }),
            get_objective_offset());
    }
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// Variables ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    std::size_t num_native_ids_variables() {
        return static_cast<std::size_t>(Clp->getNumCols(model));
    }
    std::size_t _add_variables(std::size_t count,
                               const variable_params & params) {
        const std::size_t offset = num_native_ids_variables();
        Clp->addColumns(model, static_cast<int>(count), nullptr, nullptr,
                        nullptr, nullptr, nullptr, nullptr);
        if(auto obj = params.obj_coef; obj != 0.0) {
            scalar * objective = Clp->objective(model);
            std::fill(objective + offset, objective + offset + count, obj);
        }
        if(auto lb = params.lower_bound.value_or(-COIN_DBL_MAX); lb != 0.0) {
            scalar * lower_bounds = Clp->columnLower(model);
            std::fill(lower_bounds + offset, lower_bounds + offset + count, lb);
        }
        if(auto ub = params.upper_bound.value_or(COIN_DBL_MAX);
           ub != COIN_DBL_MAX) {
            scalar * upper_bounds = Clp->columnUpper(model);
            std::fill(upper_bounds + offset, upper_bounds + offset + count, ub);
        }
        return offset;
    }

    inline variable _recycle_variable(const variable_params & params,
                                      const char * name_str = "") {
        variable v{_free_variable_ids.back()};
        _free_variable_ids.pop_back();
        set_objective_coefficient(v, params.obj_coef);
        if(double lb = params.lower_bound.value_or(-COIN_DBL_MAX); lb != 0.0)
            set_variable_lower_bound(v, lb);
        if(double ub = params.upper_bound.value_or(COIN_DBL_MAX); ub != 0.0)
            set_variable_upper_bound(v, ub);
        set_variable_name(v, name_str);
        return v;
    }

public:
    variable add_variable(
        const variable_params params = default_variable_params) {
        if(!_free_variable_ids.empty()) {
            return _recycle_variable(params);
        }
        index var_id = static_cast<index>(num_native_ids_variables());
        const auto lb = params.lower_bound.value_or(-COIN_DBL_MAX);
        const auto ub = params.upper_bound.value_or(COIN_DBL_MAX);
        Clp->addColumns(model, 1, &lb, &ub, &params.obj_coef, nullptr, nullptr,
                        nullptr);
        return variable(var_id);
    }
    friend model_base<int, double>;
    using model_base<int, double>::add_variables;
    using model_base<int, double>::add_named_variable;
    using model_base<int, double>::add_named_variables;

private:
    std::size_t _new_variables(std::size_t count,
                               const variable_params & params, variable_kind) {
        return _add_variables(count, params);
    }

private:
    template <typename ER>
    inline variable _add_column(ER && entries, const variable_params & params) {
        if(!_free_variable_ids.empty()) {
            variable v = _recycle_variable(params);
            for(auto && [constr, coef] : entries) {
                Clp->modifyCoefficient(model, constr.id(), v.id(),
                                       static_cast<double>(coef), false);
            }
            return v;
        }
        _reset_cache();
        _register_constraints_entries<true>(entries);
        const int var_id = static_cast<int>(num_native_ids_variables());
        const auto lb = params.lower_bound.value_or(-COIN_DBL_MAX);
        const auto ub = params.upper_bound.value_or(COIN_DBL_MAX);
        index starts[2] = {0, static_cast<index>(tmp_indices.size())};
        Clp->addColumns(model, 1, &lb, &ub, &params.obj_coef, starts,
                        tmp_indices.data(), tmp_scalars.data());
        return variable(var_id);
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

    void remove_variable(variable v) {
        set_objective_coefficient(v, 0);
        set_variable_lower_bound(v, 0);
        set_variable_upper_bound(v, 0);
        const int * col_starts = Clp->getVectorStarts(model);
        const int * row_indices = Clp->getIndices(model);

        const int * begin = row_indices + col_starts[v.id()];
        const int * end = row_indices + col_starts[v.id() + 1];
        for(const int * it = begin; it != end; ++it) {
            Clp->modifyCoefficient(model, *it, v.id(), 0.0, false);
        }

        _free_variable_ids.emplace_back(v.id());
    }
    template <std::ranges::range VR>
    void remove_variables(VR && variables) {
        for(auto && v : variables) remove_variable(v);
    }

    void set_objective_coefficient(variable v, scalar c) {
        Clp->objective(model)[v.id()] = c;
    }
    void set_variable_lower_bound(variable v, scalar lb) {
        Clp->columnLower(model)[v.id()] = lb;
    }
    void set_variable_upper_bound(variable v, scalar ub) {
        Clp->columnUpper(model)[v.id()] = ub;
    }
    void set_variable_name(variable v, const char * name) {
        Clp->setColumnName(model, v.id(), const_cast<char *>(name));
    }
    void set_variable_name(variable v, std::string name) {
        set_variable_name(v, name.c_str());
    }

    scalar get_objective_coefficient(variable v) {
        return Clp->objective(model)[v.id()];
    }
    scalar get_variable_lower_bound(variable v) {
        return Clp->columnLower(model)[v.id()];
    }
    scalar get_variable_upper_bound(variable v) {
        return Clp->columnUpper(model)[v.id()];
    }
    std::string get_variable_name(variable v) {
        auto max_length = static_cast<std::size_t>(Clp->lengthNames(model));
        std::string name(max_length, '\0');
        Clp->columnName(model, v.id(), name.data());
        name.resize(std::strlen(name.c_str()));
        return name;
    }
    ///////////////////////////////////////////////////////////////////////////
    /////////////////////////////// Constraints ///////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    constraint add_constraint(linear_constraint auto && lc) {
        int constr_id = static_cast<int>(num_constraints());
        _reset_cache();
        _register_variables_entries<true>(lc.linear_terms());
        const scalar b = lc.rhs();
        index starts[2] = {0, static_cast<index>(tmp_indices.size())};
        Clp->addRows(
            model, 1,
            (lc.sense() == constraint_sense::less_equal) ? nullptr : &b,
            (lc.sense() == constraint_sense::greater_equal) ? nullptr : &b,
            starts, tmp_indices.data(), tmp_scalars.data());
        return constraint(constr_id);
    }
    template <linear_constraint LC>
    constraint add_constraint(distinct_variables_t, LC && lc) {
        return add_constraint(std::forward<LC>(lc));
    }

private:
    template <linear_constraint LC>
    void _register_constraint(LC && lc) {
        tmp_begins.emplace_back(static_cast<index>(tmp_indices.size()));
        const scalar b = lc.rhs();
        tmp_lower_bounds.emplace_back(
            (lc.sense() == constraint_sense::less_equal) ? -COIN_DBL_MAX : b);
        tmp_upper_bounds.emplace_back(
            (lc.sense() == constraint_sense::greater_equal) ? COIN_DBL_MAX : b);
        for(auto && [var, coef] : lc.linear_terms()) {
            tmp_indices.emplace_back(var.id());
            tmp_scalars.emplace_back(coef);
        }
    }
    template <typename Key, typename LastConstrLambda>
        requires linear_constraint<
            detail::key_invoke_result_t<LastConstrLambda &, const Key &>>
    void _register_first_valued_constraint(const Key & key,
                                           LastConstrLambda & lc_lambda) {
        _register_constraint(detail::invoke_key(lc_lambda, key));
    }
    template <typename Key, typename OptConstrLambda, typename... Tail>
        requires detail::optional_type<detail::key_invoke_result_t<
                     OptConstrLambda &, const Key &>> &&
                 linear_constraint<
                     detail::optional_type_value_t<detail::key_invoke_result_t<
                         OptConstrLambda &, const Key &>>>
    void _register_first_valued_constraint(const Key & key,
                                           OptConstrLambda & opt_lc_lambda,
                                           Tail &... tail) {
        if(const auto & opt_lc = detail::invoke_key(opt_lc_lambda, key)) {
            _register_constraint(opt_lc.value());
            return;
        }
        _register_first_valued_constraint(key, tail...);
    }

public:
    template <std::ranges::range IR, typename... CL>
    auto add_constraints(IR && keys, CL &&... constraint_lambdas) {
        tmp_begins.resize(0);
        tmp_indices.resize(0);
        tmp_scalars.resize(0);
        tmp_lower_bounds.resize(0);
        tmp_upper_bounds.resize(0);
        const index offset = static_cast<index>(num_constraints());
        index constr_id = offset;
        for(auto && key : keys) {
            _register_first_valued_constraint(key, constraint_lambdas...);
            ++constr_id;
        }
        tmp_begins.emplace_back(static_cast<index>(tmp_indices.size()));
        Clp->addRows(model, static_cast<int>(tmp_begins.size()) - 1,
                     tmp_lower_bounds.data(), tmp_upper_bounds.data(),
                     tmp_begins.data(), tmp_indices.data(), tmp_scalars.data());
        return detail::keyed_entities(
            *this, keys, detail::set_constraint_name,
            entity_range(constraint{offset},
                         static_cast<std::size_t>(constr_id - offset)));
    }
    template <std::ranges::range IR, typename... CL>
    auto add_constraints(distinct_variables_t, IR && keys,
                         CL &&... constraint_lambdas) {
        return add_constraints(std::forward<IR>(keys),
                               std::forward<CL>(constraint_lambdas)...);
    }

    void set_constraint_rhs(constraint constr, scalar rhs) {
        switch(get_constraint_sense(constr)) {
            case constraint_sense::equal:
                Clp->rowLower(model)[constr.id()] =
                    Clp->rowUpper(model)[constr.id()] = rhs;
                return;
            case constraint_sense::less_equal:
                Clp->rowUpper(model)[constr.id()] = rhs;
                return;
            case constraint_sense::greater_equal:
                Clp->rowLower(model)[constr.id()] = rhs;
                return;
        }
    }
    void set_constraint_sense(constraint constr, constraint_sense r) {
        constraint_sense old_r = get_constraint_sense(constr);
        scalar old_rhs = get_constraint_rhs(constr);
        if(old_r == r) return;
        switch(r) {
            case constraint_sense::equal:
                Clp->rowLower(model)[constr.id()] =
                    Clp->rowUpper(model)[constr.id()] = old_rhs;
                return;
            case constraint_sense::less_equal:
                Clp->rowLower(model)[constr.id()] = -COIN_DBL_MAX;
                Clp->rowUpper(model)[constr.id()] = old_rhs;
                return;
            case constraint_sense::greater_equal:
                Clp->rowLower(model)[constr.id()] = old_rhs;
                Clp->rowUpper(model)[constr.id()] = COIN_DBL_MAX;
                return;
        }
    }
    constraint add_ranged_constraint(linear_expression auto && le, scalar lb,
                                     scalar ub) {
        index constr_id = static_cast<index>(num_constraints());
        tmp_indices.resize(0);
        tmp_scalars.resize(0);
        for(auto && [var, coef] : le.linear_terms()) {
            tmp_indices.emplace_back(var.id());
            tmp_scalars.emplace_back(coef);
        }
        lb -= le.constant();
        ub -= le.constant();
        index starts[2] = {0, static_cast<index>(tmp_indices.size())};
        Clp->addRows(model, 1, &lb, &ub, starts, tmp_indices.data(),
                     tmp_scalars.data());
        return constraint(constr_id);
    }
    constraint add_ranged_constraint(distinct_variables_t,
                                     linear_expression auto && le, scalar lb,
                                     scalar ub) {
        return add_ranged_constraint(le, lb, ub);
    }
    void set_constraint_name(constraint constr, const std::string & name) {
        Clp->setRowName(model, constr.id(), const_cast<char *>(name.c_str()));
    }

    // No get_constraint_lhs / get_constraint: the Clp C API exposes the
    // matrix column-major only.
    scalar get_constraint_rhs(constraint constr) {
        if(get_constraint_sense(constr) == constraint_sense::greater_equal)
            return Clp->rowLower(model)[constr.id()];
        return Clp->rowUpper(model)[constr.id()];
    }
    constraint_sense get_constraint_sense(constraint constr) {
        const scalar lb = Clp->rowLower(model)[constr.id()];
        const scalar ub = Clp->rowUpper(model)[constr.id()];
        if(lb == ub) return constraint_sense::equal;
        if(lb == -COIN_DBL_MAX) return constraint_sense::less_equal;
        if(ub == COIN_DBL_MAX) return constraint_sense::greater_equal;
        throw std::runtime_error(
            "Tried to get the sense of a ranged constraint");
    }
    scalar get_constraint_lower_bound(constraint constr) {
        return Clp->rowLower(model)[constr.id()];
    }
    scalar get_constraint_upper_bound(constraint constr) {
        return Clp->rowUpper(model)[constr.id()];
    }
    auto get_constraint_name(constraint constr) {
        auto max_length = static_cast<std::size_t>(Clp->lengthNames(model));
        std::string name(max_length, '\0');
        Clp->rowName(model, constr.id(), name.data());
        name.resize(std::strlen(name.c_str()));
        return name;
    }

    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////// Tolerance parameters ///////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void set_feasibility_tolerance(scalar tol) {
        Clp->setPrimalTolerance(model, tol);
    }
    scalar get_feasibility_tolerance() { return Clp->primalTolerance(model); }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////// Solve status ///////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    // clang-format off
private:
    using status_variant = std::variant<
            status::unknown,
            status::optimal,
            status::infeasible,
            status::unbounded>;

    status_variant _status = status::unknown{};

    status_variant _get_status() {
        using namespace status;
        switch(Clp->status(model)) {
            case 0: return optimal{};
            case 1: return infeasible{};
            case 2: return unbounded{};
            default:
                return unknown{};
        }
    }
    // clang-format on
public:
    const status_variant & solve_status() const { return _status; }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////////// Solve //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void solve() {
        if(num_variables() == 0u) {
            _status = status::unknown{};
            return;
        }
        // Clp keeps the scale factors of the previous solve as long as the
        // dimensions match, and an empty column with an infinite bound range
        // gets a factor around 1e22: a finite range set since then scales to
        // a "tiny gap" and the column is silently fixed. Dropping the factors
        // forces a recompute and keeps the warm-start basis.
        const int scaling_mode = Clp->scalingFlag(model);
        Clp->scaling(model, 0);
        Clp->scaling(model, scaling_mode);
        Clp->primal(model, 0);
        _status = _get_status();
    }
    scalar get_solution_value() { return Clp->getObjValue(model); }
    auto get_solution() {
        return variable_mapping(Clp->primalColumnSolution(model));
    }
    auto get_dual_solution() {
        return constraint_mapping(Clp->dualRowSolution(model));
    }
    auto get_reduced_costs() {
        return variable_mapping(Clp->dualColumnSolution(model));
    }
};

}  // namespace clp::v1_17
}  // namespace mippp
