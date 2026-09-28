#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <limits>
#include <memory>
#include <optional>
#include <ranges>
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

#include "mippp/solvers/model_base.hpp"
#include "mippp/solvers/soplex/impl/v1/soplex_api.hpp"

namespace mippp {
namespace soplex::impl::v1 {

class soplex_lp : protected model_base<int, double> {
private:
    // soplex::infinity, which the C interface does not export
    static constexpr double _infinity = 1e100;

    // Sides are written and read within [-infinity(), infinity()]: SoPlex
    // reports a bounded LP unbounded when a column side is IEEE infinity, and
    // add_constraint writes the absent side of a row as IEEE infinity.
    static double _clamp_side(double value) noexcept {
        return std::clamp(value, -_infinity, _infinity);
    }

    template <typename F>
    static F & _optional(F * function, const char * name) {
        if(function == nullptr)
            throw solver_error((std::string(name) + " not available.").c_str());
        return *function;
    }

protected:
    using variable_id = int;
    using constraint_id = int;

public:
    using model_base<int, double>::default_variable_params;
    using model_base<int, double>::variables;
    using model_base<int, double>::constraints;
    double infinity() const noexcept { return _infinity; }
    using model_base<int, double>::is_infinite;

private:
    const soplex_api * SoPlex;
    void * model;
    double objective_offset;
    std::chrono::duration<double> _time_limit;
    // What was written, read back from here: once a solve has scaled the LP
    // in place, SoPlex_getLowerReal and SoPlex_getUpperReal unscale the
    // infinite sides too, and a -1e100 bound reads back as -1.2e96. The copy
    // is authoritative: a solve that reloads the LP writes it back into
    // SoPlex, so a column bound changed through native_model() is reverted.
    std::vector<double> _lower_bounds;
    std::vector<double> _upper_bounds;

public:
    [[nodiscard]] soplex_lp() : soplex_lp(soplex_api::load()) {}
    [[nodiscard]] explicit soplex_lp(const soplex_api & api)
        : SoPlex(&api)
        , model(SoPlex->create())
        , objective_offset(0.0)
        , _time_limit(_infinity) {
        SoPlex->setIntParam(model, SOPLEX_VERBOSITY, SOPLEX_VERBOSITY_ERROR);
        // SoPlex maximizes by default, every other backend minimizes
        set_minimization();
    }
    ~soplex_lp() {
        if(model) SoPlex->free(model);
    }

    constexpr soplex_lp(const soplex_lp &) = delete;
    constexpr soplex_lp(soplex_lp && other) noexcept
        : model_base<int, double>(std::move(other))
        , SoPlex(other.SoPlex)
        , model(other.model)
        , objective_offset(other.objective_offset)
        , _time_limit(other._time_limit)
        , _lower_bounds(std::move(other._lower_bounds))
        , _upper_bounds(std::move(other._upper_bounds))
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
    void set_maximization() {
        SoPlex->setIntParam(model, SOPLEX_OBJSENSE, SOPLEX_OBJSENSE_MAXIMIZE);
    }
    void set_minimization() {
        SoPlex->setIntParam(model, SOPLEX_OBJSENSE, SOPLEX_OBJSENSE_MINIMIZE);
    }

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
    // the C interface reads the objective only as a whole vector
    double get_objective_coefficient(variable v) {
        auto & get_obj = _optional(SoPlex->getObjReal, "SoPlex_getObjReal");
        std::vector<double> coefs(num_variables());
        get_obj(model, coefs.data(), static_cast<int>(coefs.size()));
        return coefs[static_cast<std::size_t>(v.id())];
    }
    auto get_objective() {
        auto & get_obj = _optional(SoPlex->getObjReal, "SoPlex_getObjReal");
        const auto num_vars = num_variables();
        auto coefs = std::make_shared_for_overwrite<double[]>(num_vars);
        if(num_vars > 0u)
            get_obj(model, coefs.get(), static_cast<int>(num_vars));
        return linear_expression_view(
            std::views::transform(
                std::views::iota(index{0}, static_cast<index>(num_vars)),
                [coefs = std::move(coefs)](auto i) {
                    return std::make_pair(variable(i), coefs[i]);
                }),
            get_objective_offset());
    }
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// Variables ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    // entries holds one coefficient per row, num_nz of them nonzero
    void _add_soplex_column(double * entries, int num_rows, int num_nz,
                            const variable_params & params) {
        const double lb = _clamp_side(params.lower_bound.value_or(-_infinity));
        const double ub = _clamp_side(params.upper_bound.value_or(_infinity));
        SoPlex->addColReal(model, entries, num_rows, num_nz, params.obj_coef,
                           lb, ub);
        _lower_bounds.push_back(lb);
        _upper_bounds.push_back(ub);
    }
    inline void _add_var(const variable_params & params) {
        _add_soplex_column(nullptr, 0, 0, params);
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
        _add_soplex_column(tmp_scalars.data(),
                           static_cast<int>(num_constraints()), num_nz, params);
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

    double get_variable_lower_bound(variable v) {
        return _lower_bounds[static_cast<std::size_t>(v.id())];
    }
    double get_variable_upper_bound(variable v) {
        return _upper_bounds[static_cast<std::size_t>(v.id())];
    }
    void set_variable_lower_bound(variable v, double lb) {
        lb = _clamp_side(lb);
        _optional(SoPlex->changeVarLowerReal, "SoPlex_changeVarLowerReal")(
            model, v.id(), lb);
        _lower_bounds[static_cast<std::size_t>(v.id())] = lb;
    }
    void set_variable_upper_bound(variable v, double ub) {
        // SoPlex 6.0 has SoPlex_changeVarUpperReal, but there it moves the
        // lower bound, so the setter stands or falls with its lower twin
        if(SoPlex->changeVarLowerReal == nullptr)
            throw solver_error("SoPlex_changeVarUpperReal not available.");
        ub = _clamp_side(ub);
        _optional(SoPlex->changeVarUpperReal, "SoPlex_changeVarUpperReal")(
            model, v.id(), ub);
        _upper_bounds[static_cast<std::size_t>(v.id())] = ub;
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
    double get_constraint_lower_bound(constraint constr) {
        double lb, ub;
        _optional(SoPlex->getRowBoundsReal, "SoPlex_getRowBoundsReal")(
            model, constr.id(), &lb, &ub);
        return _clamp_side(lb);
    }
    double get_constraint_upper_bound(constraint constr) {
        double lb, ub;
        _optional(SoPlex->getRowBoundsReal, "SoPlex_getRowBoundsReal")(
            model, constr.id(), &lb, &ub);
        return _clamp_side(ub);
    }
    void set_constraint_lower_bound(constraint constr, double lb) {
        _optional(SoPlex->changeRowLhsReal, "SoPlex_changeRowLhsReal")(
            model, constr.id(), _clamp_side(lb));
    }
    void set_constraint_upper_bound(constraint constr, double ub) {
        _optional(SoPlex->changeRowRhsReal, "SoPlex_changeRowRhsReal")(
            model, constr.id(), _clamp_side(ub));
    }
    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////// Limits //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    // The C interface of SoPlex has no getter for real parameters, so the
    // limit is read back from the copy kept here.
    void set_time_limit(std::chrono::duration<double> t) {
        auto & set_real_param =
            _optional(SoPlex->setRealParam, "SoPlex_setRealParam");
        // SoPlex silently keeps its previous limit when given a value outside
        // [0, 1e100], so the copy read back would disagree with the solver.
        if(t.count() < 0) throw solver_error("soplex_lp: negative time limit");
        const double seconds = std::min(t.count(), _infinity);
        set_real_param(model, SOPLEX_TIMELIMIT, seconds);
        _time_limit = std::chrono::duration<double>(seconds);
    }
    auto get_time_limit() { return _time_limit; }
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// Verbosity ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void set_verbose(bool verbose) {
        SoPlex->setIntParam(
            model, SOPLEX_VERBOSITY,
            verbose ? SOPLEX_VERBOSITY_NORMAL : SOPLEX_VERBOSITY_ERROR);
    }
    bool is_verbose() {
        return SoPlex->getIntParam(model, SOPLEX_VERBOSITY) !=
               SOPLEX_VERBOSITY_ERROR;
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
            status::time_limit,
            status::failed>;

    status_variant _status = status::unknown{};
    // clang-format on
public:
    const status_variant & get_status() const { return _status; }
    void reset_status() noexcept { _status = status::unknown{}; }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////////// Solve //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    // A warm start leaves a nonbasic column or row on the side it sat on, and
    // SoPlex does not check that side against the other one: once they cross,
    // it reports optimal at a point outside them.
    bool _has_crossed_sides() {
        for(std::size_t j = 0; j < _lower_bounds.size(); ++j)
            if(_lower_bounds[j] > _upper_bounds[j]) return true;
        if(SoPlex->getRowBoundsReal == nullptr) return false;
        const int num_rows = SoPlex->numRows(model);
        for(int i = 0; i < num_rows; ++i) {
            double lb, ub;
            SoPlex->getRowBoundsReal(model, i, &lb, &ub);
            if(lb > ub) return true;
        }
        return false;
    }
    // Freeing the active side of a row leaves it nonbasic and free, and a warm
    // start from such a basis fails: SoPlex reports running after an internal
    // XLEAVE04 exception, or a wrong optimum. The C interface cannot drop a
    // basis, so the LP is loaded again, without one.
    bool _has_nonbasic_free_row() {
        if(SoPlex->basisRowStatus == nullptr) return false;
        const int num_rows = SoPlex->numRows(model);
        for(int i = 0; i < num_rows; ++i)
            if(SoPlex->basisRowStatus(model, i) == SOPLEX_BASIS_ZERO)
                return true;
        return false;
    }
    void _reload_without_basis() {
        auto & get_obj = _optional(SoPlex->getObjReal, "SoPlex_getObjReal");
        auto & get_row =
            _optional(SoPlex->getRowVectorReal, "SoPlex_getRowVectorReal");
        auto & get_row_bounds =
            _optional(SoPlex->getRowBoundsReal, "SoPlex_getRowBoundsReal");
        auto & clear_lp = _optional(SoPlex->clearLPReal, "SoPlex_clearLPReal");
        const int num_cols = SoPlex->numCols(model);
        const int num_rows = SoPlex->numRows(model);
        const auto num_cols_size = static_cast<std::size_t>(num_cols);
        const auto num_rows_size = static_cast<std::size_t>(num_rows);
        std::vector<double> objective(num_cols_size);
        if(num_cols > 0) get_obj(model, objective.data(), num_cols);
        std::vector<double> lhs(num_rows_size), rhs(num_rows_size);
        std::vector<std::size_t> row_begins(num_rows_size + 1u, 0u);
        std::vector<long> indices;
        std::vector<double> coefs;
        std::vector<long> row_indices(num_cols_size);
        std::vector<double> row_coefs(num_cols_size);
        for(int i = 0; i < num_rows; ++i) {
            const auto row = static_cast<std::size_t>(i);
            get_row_bounds(model, i, &lhs[row], &rhs[row]);
            int num_nz = 0;
            get_row(model, i, &num_nz, row_indices.data(), row_coefs.data());
            indices.insert(indices.end(), row_indices.begin(),
                           row_indices.begin() + num_nz);
            coefs.insert(coefs.end(), row_coefs.begin(),
                         row_coefs.begin() + num_nz);
            row_begins[row + 1u] = indices.size();
        }
        const int sense = SoPlex->getIntParam(model, SOPLEX_OBJSENSE);
        clear_lp(model);
        // the cleared LP maximizes, and setting a parameter to the value it
        // holds is ignored
        if(sense != SOPLEX_OBJSENSE_MAXIMIZE) {
            SoPlex->setIntParam(model, SOPLEX_OBJSENSE,
                                SOPLEX_OBJSENSE_MAXIMIZE);
            SoPlex->setIntParam(model, SOPLEX_OBJSENSE, sense);
        }
        for(std::size_t j = 0; j < num_cols_size; ++j)
            SoPlex->addColReal(model, nullptr, 0, 0, objective[j],
                               _lower_bounds[j], _upper_bounds[j]);
        std::vector<double> dense_row(num_cols_size, 0.0);
        for(std::size_t row = 0; row < num_rows_size; ++row) {
            for(std::size_t k = row_begins[row]; k < row_begins[row + 1u]; ++k)
                dense_row[static_cast<std::size_t>(indices[k])] = coefs[k];
            SoPlex->addRowReal(
                model, dense_row.data(), num_cols,
                static_cast<int>(row_begins[row + 1u] - row_begins[row]),
                lhs[row], rhs[row]);
            for(std::size_t k = row_begins[row]; k < row_begins[row + 1u]; ++k)
                dense_row[static_cast<std::size_t>(indices[k])] = 0.0;
        }
    }

public:
    void solve() {
        using namespace status;
        if(num_variables() == 0u) {
            _status = status::unknown{};
            return;
        }
        if(_has_crossed_sides()) {
            _status.emplace<infeasible>();
            return;
        }
        if(_has_nonbasic_free_row()) _reload_without_basis();
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
            case ABORT_TIME:
                _status.emplace<time_limit>();
                return;
            case ABORT_CYCLING:
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

}  // namespace soplex::impl::v1
}  // namespace mippp
