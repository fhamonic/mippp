#pragma once

// dumb_lp: a trivially correct but inefficient LP model implementation.
//
// It stores the whole model in plain, obvious data structures (one struct per
// column, one std::map per row) so that every setter/getter of the common
// model concepts of "mippp/model_concepts.hpp" is implemented in the most
// straightforward way possible. It is meant to be used as a reference
// implementation to cross-check the solver-specific implementations in fuzzy
// tests, not to be fast.
//
// solve() rebuilds the whole problem from scratch, loads it into Clp with
// loadProblem, solves it with primal and caches the solution
// (primalColumnSolution), the dual solution and the status.

#include <algorithm>
#include <map>
#include <ranges>
#include <string>
#include <utility>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/clp/v1_17/clp_api.hpp"
#include "mippp/solvers/model_base.hpp"

namespace mippp {
// nested in clp::v1_17 so that CoinBigIndex resolves whether the real Clp
// headers are included or the declarations of clp_api.hpp are used
namespace clp::v1_17 {

class dumb_lp : public model_base<int, double> {
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

    static constexpr scalar infinity = std::numeric_limits<double>::max();

private:
    struct column_data {
        scalar obj_coef = 0.0;
        scalar lower_bound = -infinity;
        scalar upper_bound = infinity;
        std::string name;
        bool removed = false;
    };
    struct row_data {
        std::map<variable_id, scalar> coefs;
        constraint_sense sense = constraint_sense::equal;
        scalar rhs = 0.0;
        std::string name;
    };

    using status_variant = std::variant<status::unknown, status::optimal,
                                        status::infeasible, status::unbounded>;

    const clp_api & Clp;

    bool _maximize = false;
    scalar _obj_offset = 0.0;
    std::vector<column_data> _cols;
    std::vector<row_data> _rows;
    std::vector<variable_id> _free_variable_ids;
    scalar _feasibility_tolerance = 1e-7;

    status_variant _status;
    scalar _solution_value = 0.0;
    std::vector<scalar> _primal_solution;
    std::vector<scalar> _dual_solution;

public:
    [[nodiscard]] explicit dumb_lp(const clp_api & api)
        : model_base<int, double>(), Clp(api) {}

    std::size_t num_variables() {
        return _cols.size() - _free_variable_ids.size();
    }
    std::size_t num_constraints() { return _rows.size(); }
    std::size_t num_entries() {
        std::size_t count = 0;
        for(const row_data & row : _rows)
            for(const auto & [var_id, coef] : row.coefs)
                if(coef != 0.0) ++count;
        return count;
    }

    void set_maximization() { _maximize = true; }
    void set_minimization() { _maximize = false; }

    ////////////////////////////////////////////////////////////////////////////
    // Objective
    ////////////////////////////////////////////////////////////////////////////

    void set_objective_offset(scalar constant) { _obj_offset = constant; }
    scalar get_objective_offset() { return _obj_offset; }

    void set_objective(linear_expression auto && le) {
        for(column_data & col : _cols) col.obj_coef = 0.0;
        add_objective(le);
        set_objective_offset(le.constant());
    }
    void add_objective(linear_expression auto && le) {
        for(auto && [var, coef] : le.linear_terms())
            _cols[var.uid()].obj_coef += coef;
        set_objective_offset(get_objective_offset() + le.constant());
    }
    void set_objective_coefficient(variable v, scalar c) {
        _cols[v.uid()].obj_coef = c;
    }
    scalar get_objective_coefficient(variable v) {
        return _cols[v.uid()].obj_coef;
    }
    auto get_objective() {
        return linear_expression_view(
            std::views::transform(
                std::views::filter(
                    std::views::iota(variable_id{0},
                                     static_cast<variable_id>(_cols.size())),
                    [this](variable_id i) {
                        return !_cols[static_cast<std::size_t>(i)].removed;
                    }),
                [this](variable_id i) {
                    return std::make_pair(
                        variable(i),
                        _cols[static_cast<std::size_t>(i)].obj_coef);
                }),
            get_objective_offset());
    }

    ////////////////////////////////////////////////////////////////////////////
    // Variables
    ////////////////////////////////////////////////////////////////////////////

private:
    variable _new_variable(const variable_params & params,
                           const std::string & name) {
        column_data col{.obj_coef = params.obj_coef,
                        .lower_bound = params.lower_bound.value_or(-infinity),
                        .upper_bound = params.upper_bound.value_or(infinity),
                        .name = name,
                        .removed = false};
        if(!_free_variable_ids.empty()) {
            const variable_id var_id = _free_variable_ids.back();
            _free_variable_ids.pop_back();
            _cols[static_cast<std::size_t>(var_id)] = std::move(col);
            return variable(var_id);
        }
        _cols.emplace_back(std::move(col));
        return variable(static_cast<variable_id>(_cols.size() - 1));
    }
    std::size_t _append_variables(std::size_t count,
                                  const variable_params & params) {
        const std::size_t offset = _cols.size();
        for(std::size_t i = 0; i < count; ++i)
            _cols.emplace_back(column_data{
                .obj_coef = params.obj_coef,
                .lower_bound = params.lower_bound.value_or(-infinity),
                .upper_bound = params.upper_bound.value_or(infinity),
                .name = {},
                .removed = false});
        return offset;
    }

public:
    variable add_variable(
        const variable_params params = default_variable_params) {
        return _new_variable(params, "");
    }
    auto add_variables(std::size_t count,
                       variable_params params = default_variable_params) {
        const std::size_t offset = _append_variables(count, params);
        return _make_variables_view(offset, count);
    }
    template <typename IL>
    auto add_variables(std::size_t count, IL && id_lambda,
                       variable_params params = default_variable_params) {
        const std::size_t offset = _append_variables(count, params);
        return _make_indexed_variables_view(offset, count,
                                            std::forward<IL>(id_lambda));
    }

    variable add_named_variable(
        const std::string & name,
        const variable_params params = default_variable_params) {
        return _new_variable(params, name);
    }
    template <typename NL>
    auto add_named_variables(std::size_t count, NL && name_lambda,
                             variable_params params = default_variable_params) {
        const std::size_t offset = _append_variables(count, params);
        return _make_named_variables_view(offset, count,
                                          std::forward<NL>(name_lambda), this);
    }
    template <typename IL, typename NL>
    auto add_named_variables(std::size_t count, IL && id_lambda,
                             NL && name_lambda,
                             variable_params params = default_variable_params) {
        const std::size_t offset = _append_variables(count, params);
        return _make_indexed_named_variables_view(
            offset, count, std::forward<IL>(id_lambda),
            std::forward<NL>(name_lambda), this);
    }

private:
    template <typename ER>
    variable _add_column(ER && entries, const variable_params & params) {
        const variable v = _new_variable(params, "");
        for(auto && [constr, coef] : entries)
            _rows[constr.uid()].coefs[v.id()] += static_cast<scalar>(coef);
        return v;
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
        _cols[v.uid()] = column_data{.obj_coef = 0.0,
                                     .lower_bound = 0.0,
                                     .upper_bound = 0.0,
                                     .name = {},
                                     .removed = true};
        for(row_data & row : _rows) row.coefs.erase(v.id());
        _free_variable_ids.emplace_back(v.id());
    }
    template <std::ranges::range VR>
    void remove_variables(VR && variables) {
        for(auto && v : variables) remove_variable(v);
    }

    void set_variable_lower_bound(variable v, scalar lb) {
        _cols[v.uid()].lower_bound = lb;
    }
    void set_variable_upper_bound(variable v, scalar ub) {
        _cols[v.uid()].upper_bound = ub;
    }
    scalar get_variable_lower_bound(variable v) {
        return _cols[v.uid()].lower_bound;
    }
    scalar get_variable_upper_bound(variable v) {
        return _cols[v.uid()].upper_bound;
    }
    void set_variable_name(variable v, const std::string & name) {
        _cols[v.uid()].name = name;
    }
    std::string get_variable_name(variable v) { return _cols[v.uid()].name; }

    ////////////////////////////////////////////////////////////////////////////
    // Constraints
    ////////////////////////////////////////////////////////////////////////////

    constraint add_constraint(linear_constraint auto && lc) {
        const constraint_id constr_id =
            static_cast<constraint_id>(_rows.size());
        row_data row;
        for(auto && [var, coef] : lc.linear_terms())
            row.coefs[var.id()] += coef;
        row.sense = lc.sense();
        row.rhs = lc.rhs();
        _rows.emplace_back(std::move(row));
        return constraint(constr_id);
    }

private:
    template <typename Key, typename LastConstrLambda>
        requires linear_constraint<std::invoke_result_t<LastConstrLambda, Key>>
    void _add_first_valued_constraint(const Key & key,
                                      const LastConstrLambda & lc_lambda) {
        add_constraint(lc_lambda(key));
    }
    template <typename Key, typename OptConstrLambda, typename... Tail>
        requires detail::optional_type<
                     std::invoke_result_t<OptConstrLambda, Key>> &&
                 linear_constraint<detail::optional_type_value_t<
                     std::invoke_result_t<OptConstrLambda, Key>>>
    void _add_first_valued_constraint(const Key & key,
                                      const OptConstrLambda & opt_lc_lambda,
                                      const Tail &... tail) {
        if(const auto & opt_lc = opt_lc_lambda(key)) {
            add_constraint(opt_lc.value());
            return;
        }
        _add_first_valued_constraint(key, tail...);
    }

public:
    template <std::ranges::range IR, typename... CL>
    auto add_constraints(IR && keys, CL... constraint_lambdas) {
        const constraint_id offset = static_cast<constraint_id>(_rows.size());
        constraint_id constr_id = offset;
        for(auto && key : keys) {
            _add_first_valued_constraint(key, constraint_lambdas...);
            ++constr_id;
        }
        return constraints_range(
            std::forward<IR>(keys),
            std::views::transform(std::views::iota(offset, constr_id),
                                  [](auto && i) { return constraint{i}; }));
    }

    template <std::ranges::range ER>
    void set_constraint_lhs(constraint c, ER && entries) {
        auto & coefs = _rows[c.uid()].coefs;
        coefs.clear();
        for(auto && [var, coef] : entries) coefs[var.id()] += coef;
    }
    void set_constraint_lhs(
        constraint c,
        std::initializer_list<std::pair<variable, scalar>> entries) {
        set_constraint_lhs(c, std::views::all(entries));
    }
    void set_constraint_sense(constraint c, constraint_sense s) {
        _rows[c.uid()].sense = s;
    }
    void set_constraint_rhs(constraint c, scalar rhs) {
        _rows[c.uid()].rhs = rhs;
    }
    auto get_constraint_lhs(constraint c) {
        return std::views::transform(
            std::views::filter(
                _rows[c.uid()].coefs,
                [](const auto & entry) { return entry.second != 0.0; }),
            [](const auto & entry) {
                return std::make_pair(variable(entry.first), entry.second);
            });
    }
    constraint_sense get_constraint_sense(constraint c) {
        return _rows[c.uid()].sense;
    }
    scalar get_constraint_rhs(constraint c) { return _rows[c.uid()].rhs; }
    auto get_constraint(constraint c) {
        return linear_constraint_view(
            linear_expression_view(get_constraint_lhs(c),
                                   -get_constraint_rhs(c)),
            get_constraint_sense(c));
    }
    void set_constraint_name(constraint c, const std::string & name) {
        _rows[c.uid()].name = name;
    }
    std::string get_constraint_name(constraint c) {
        return _rows[c.uid()].name;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Parameters
    ////////////////////////////////////////////////////////////////////////////

    void set_feasibility_tolerance(scalar tol) { _feasibility_tolerance = tol; }
    scalar get_feasibility_tolerance() { return _feasibility_tolerance; }

    ////////////////////////////////////////////////////////////////////////////
    // Solve
    ////////////////////////////////////////////////////////////////////////////

private:
    static bool _violated_by_zero_lhs(const row_data & row) {
        switch(row.sense) {
            case constraint_sense::equal:
                return row.rhs != 0.0;
            case constraint_sense::less_equal:
                return row.rhs < 0.0;
            case constraint_sense::greater_equal:
                return row.rhs > 0.0;
        }
        return false;
    }

public:
    const status_variant & solve_status() const { return _status; }
    void solve() {
        const std::size_t num_cols = _cols.size();
        const std::size_t num_rows = _rows.size();
        _primal_solution.assign(num_cols, 0.0);
        _dual_solution.assign(num_rows, 0.0);
        _solution_value = _obj_offset;
        if(num_cols == 0u) {
            // no column was ever created, so every row lhs evaluates to 0
            _status.emplace<status::optimal>();
            for(const row_data & row : _rows) {
                if(_violated_by_zero_lhs(row)) {
                    _status.emplace<status::infeasible>();
                    break;
                }
            }
            return;
        }
        // column-major matrix
        std::vector<CoinBigIndex> col_starts;
        std::vector<int> row_indices;
        std::vector<scalar> values;
        for(std::size_t j = 0; j < num_cols; ++j) {
            col_starts.emplace_back(
                static_cast<CoinBigIndex>(row_indices.size()));
            for(std::size_t i = 0; i < num_rows; ++i) {
                const auto & coefs = _rows[i].coefs;
                const auto it = coefs.find(static_cast<variable_id>(j));
                if(it == coefs.end() || it->second == 0.0) continue;
                row_indices.emplace_back(static_cast<int>(i));
                values.emplace_back(it->second);
            }
        }
        col_starts.emplace_back(static_cast<CoinBigIndex>(row_indices.size()));
        // column bounds and objective
        std::vector<scalar> col_lb, col_ub, obj;
        for(const column_data & col : _cols) {
            col_lb.emplace_back(col.lower_bound);
            col_ub.emplace_back(col.upper_bound);
            obj.emplace_back(col.obj_coef);
        }
        // row bounds
        std::vector<scalar> row_lb, row_ub;
        for(const row_data & row : _rows) {
            row_lb.emplace_back(row.sense == constraint_sense::less_equal
                                    ? -infinity
                                    : row.rhs);
            row_ub.emplace_back(row.sense == constraint_sense::greater_equal
                                    ? infinity
                                    : row.rhs);
        }
        auto * model = Clp.newModel();
        Clp.loadProblem(
            model, static_cast<int>(num_cols), static_cast<int>(num_rows),
            col_starts.data(), row_indices.data(), values.data(), col_lb.data(),
            col_ub.data(), obj.data(), row_lb.data(), row_ub.data());
        Clp.setObjSense(model, _maximize ? -1 : 1);
        Clp.setPrimalTolerance(model, _feasibility_tolerance);
        Clp.primal(model, 0);
        switch(Clp.status(model)) {
            case 0:
                _status.emplace<status::optimal>();
                break;
            case 1:
                _status.emplace<status::infeasible>();
                break;
            case 2:
                _status.emplace<status::unbounded>();
                break;
            default:
                _status.emplace<status::unknown>();
        }
        const scalar * primal = Clp.primalColumnSolution(model);
        std::copy(primal, primal + num_cols, _primal_solution.begin());
        const scalar * dual = Clp.dualRowSolution(model);
        std::copy(dual, dual + num_rows, _dual_solution.begin());
        for(std::size_t j = 0; j < num_cols; ++j)
            _solution_value += _cols[j].obj_coef * _primal_solution[j];
        Clp.deleteModel(model);
    }

    scalar get_solution_value() { return _solution_value; }

    auto get_solution() {
        return variable_mapping(std::vector<scalar>(_primal_solution));
    }
    auto get_dual_solution() {
        return constraint_mapping(std::vector<scalar>(_dual_solution));
    }
};

}  // namespace clp::v1_17

using clp_api = clp::v1_17::clp_api;
using dumb_lp = clp::v1_17::dumb_lp;

static_assert(lp_model<dumb_lp>);
static_assert(sized_model<dumb_lp>);
static_assert(has_lp_status<dumb_lp>);
static_assert(has_dual_solution<dumb_lp>);
static_assert(has_named_variables<dumb_lp>);
static_assert(has_named_constraints<dumb_lp>);
static_assert(has_readable_objective<dumb_lp>);
static_assert(has_modifiable_objective<dumb_lp>);
static_assert(has_readable_variables_bounds<dumb_lp>);
static_assert(has_modifiable_variables_bounds<dumb_lp>);
static_assert(has_readable_constraints<dumb_lp>);
static_assert(has_modifiable_constraint_lhs<dumb_lp>);
static_assert(has_modifiable_constraint_sense<dumb_lp>);
static_assert(has_modifiable_constraint_rhs<dumb_lp>);
static_assert(has_add_column<dumb_lp>);
static_assert(has_remove_variable<dumb_lp>);
static_assert(has_feasibility_tolerance<dumb_lp>);

}  // namespace mippp
