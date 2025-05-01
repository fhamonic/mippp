#ifndef MIPPP_CLP_v1_17_LP_HPP
#define MIPPP_CLP_v1_17_LP_HPP

#include <algorithm>
#include <cstring>
#include <limits>
#include <optional>
#include <ostream>
#include <sstream>
#include <string_view>
#include <vector>

#include <range/v3/view/iota.hpp>
#include <range/v3/view/span.hpp>
#include <range/v3/view/zip.hpp>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/clp/v1_17/clp_api.hpp"

namespace fhamonic::mippp {
namespace clp::v1_17 {

class clp_lp {
public:
    using index = CoinBigIndex;
    using variable_id = int;
    using constraint_id = int;
    using scalar = double;
    using variable = model_variable<variable_id, scalar>;
    using constraint = model_constraint<constraint_id>;
    template <typename Map>
    using variable_mapping = entity_mapping<variable, Map>;
    template <typename Map>
    using constraint_mapping = entity_mapping<constraint, Map>;

    struct variable_params {
        scalar obj_coef = scalar{0};
        std::optional<scalar> lower_bound = std::nullopt;
        std::optional<scalar> upper_bound = std::nullopt;
    };

    static constexpr variable_params default_variable_params = {
        .obj_coef = 0, .lower_bound = 0, .upper_bound = std::nullopt};

private:
    const clp_api & Clp;
    Clp_Simplex * model;
    std::optional<lp_status> opt_lp_status;

    // static constexpr char constraint_sense_to_gurobi_sense(
    //     constraint_sense rel) {
    //     if(rel == constraint_sense::less_equal) return
    //     Clp_LESS_EQUAL; if(rel == constraint_sense::equal) return
    //     Clp_EQUAL; return Clp_GREATER_EQUAL;
    // }
    // static constexpr constraint_sense gurobi_sense_to_constraint_sense(
    //     char sense) {
    //     if(sense == Clp_LESS_EQUAL) return
    //     constraint_sense::less_equal; if(sense == Clp_EQUAL) return
    //     constraint_sense::equal; return
    //     constraint_sense::greater_equal;
    // }

    std::vector<index> tmp_indices;
    std::vector<variable_id> tmp_variables;
    std::vector<scalar> tmp_scalars;
    std::vector<scalar> tmp_lower_bounds;
    std::vector<scalar> tmp_upper_bounds;

public:
    [[nodiscard]] explicit clp_lp(const clp_api & api)
        : Clp(api), model(Clp.newModel()) {}
    ~clp_lp() { Clp.deleteModel(model); }

    std::size_t num_variables() {
        return static_cast<std::size_t>(Clp.getNumCols(model));
    }
    std::size_t num_constraints() {
        return static_cast<std::size_t>(Clp.getNumRows(model));
    }
    std::size_t num_entries() {
        return static_cast<std::size_t>(Clp.getNumElements(model));
    }

    void set_maximization() { Clp.setObjSense(model, -1); }
    void set_minimization() { Clp.setObjSense(model, 1); }

    void set_objective_offset(scalar constant) {
        Clp.setObjectiveOffset(model, constant);
    }
    void set_objective(linear_expression auto && le) {
        auto num_vars = num_variables();
        scalar * objective = Clp.objective(model);
        std::fill(objective, objective + num_vars, 0.0);
        for(auto && [var, coef] : le.linear_terms()) {
            objective[var.id()] += coef;
        }
        set_objective_offset(le.constant());
    }
    void add_objective(linear_expression auto && le) {
        scalar * objective = Clp.objective(model);
        for(auto && [var, coef] : le.linear_terms()) {
            objective[var.id()] += coef;
        }
        set_objective_offset(get_objective_offset() + le.constant());
    }
    scalar get_objective_offset() { return Clp.objectiveOffset(model); }
    auto get_objective() {
        auto num_vars = num_variables();
        const scalar * objective = Clp.objective(model);
        return linear_expression_view(
            ranges::view::zip(
                ranges::view::iota(variable_id{0},
                                   static_cast<variable_id>(num_vars)),
                ranges::span(objective, objective + num_vars)),
            get_objective_offset());
    }

    variable add_variable(const variable_params p = default_variable_params) {
        variable_id var_id = static_cast<variable_id>(num_variables());
        const auto lb = p.lower_bound.value_or(-COIN_DBL_MAX);
        const auto ub = p.upper_bound.value_or(COIN_DBL_MAX);
        Clp.addColumns(model, 1, &lb, &ub, &p.obj_coef, NULL, NULL, NULL);
        return variable(var_id);
    }

private:
    void _add_variables(std::size_t offset, std::size_t count,
                        const variable_params & params) {
        Clp.addColumns(model, static_cast<int>(count), NULL, NULL, NULL, NULL,
                       NULL, NULL);
        if(auto obj = params.obj_coef; obj != 0.0) {
            scalar * objective = Clp.objective(model);
            std::fill(objective + offset, objective + offset + count, obj);
        }
        if(auto lb = params.lower_bound.value_or(-COIN_DBL_MAX); lb != 0.0) {
            scalar * lower_bounds = Clp.columnLower(model);
            std::fill(lower_bounds + offset, lower_bounds + offset + count, lb);
        }
        if(auto ub = params.upper_bound.value_or(COIN_DBL_MAX);
           ub != COIN_DBL_MAX) {
            scalar * upper_bounds = Clp.columnUpper(model);
            std::fill(upper_bounds + offset, upper_bounds + offset + count, ub);
        }
    }

public:
    auto add_variables(
        std::size_t count,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params);
        return make_variables_range(ranges::view::transform(
            ranges::view::iota(static_cast<variable_id>(offset),
                               static_cast<variable_id>(offset + count)),
            [](auto && i) { return variable{i}; }));
    }
    template <typename IL>
    auto add_variables(
        std::size_t count, IL && id_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params);
        return make_indexed_variables_range(
            typename detail::function_traits<IL>::arg_types(),
            ranges::view::transform(
                ranges::view::iota(static_cast<variable_id>(offset),
                                   static_cast<variable_id>(offset + count)),
                [](auto && i) { return variable{i}; }),
            std::forward<IL>(id_lambda));
    }

    void set_objective_coefficient(variable v, scalar c) {
        Clp.objective(model)[v.id()] = c;
    }
    void set_variable_lower_bound(variable v, scalar lb) {
        Clp.columnLower(model)[v.id()] = lb;
    }
    void set_variable_upper_bound(variable v, scalar ub) {
        Clp.columnUpper(model)[v.id()] = ub;
    }
    void set_variable_name(variable v, std::string name) {
        Clp.setColumnName(model, v.id(), const_cast<char *>(name.c_str()));
    }

    scalar get_objective_coefficient(variable v) {
        return Clp.objective(model)[v.id()];
    }
    scalar get_variable_lower_bound(variable v) {
        return Clp.columnLower(model)[v.id()];
    }
    scalar get_variable_upper_bound(variable v) {
        return Clp.columnUpper(model)[v.id()];
    }
    auto get_variable_name(variable v) {
        auto max_length = static_cast<std::size_t>(Clp.lengthNames(model));
        std::string name(max_length, '\0');
        Clp.columnName(model, v.id(), name.data());
        name.resize(std::strlen(name.c_str()));
        return name;
    }

    constraint add_constraint(linear_constraint auto && lc) {
        int constr_id = static_cast<int>(num_constraints());
        tmp_variables.resize(0);
        tmp_scalars.resize(0);
        for(auto && [var, coef] : lc.linear_terms()) {
            tmp_variables.emplace_back(var.id());
            tmp_scalars.emplace_back(coef);
        }
        const scalar b = lc.rhs();
        index starts[2] = {0, static_cast<index>(tmp_variables.size())};
        Clp.addRows(model, 1,
                    (lc.sense() == constraint_sense::less_equal) ? NULL : &b,
                    (lc.sense() == constraint_sense::greater_equal) ? NULL : &b,
                    starts, tmp_variables.data(), tmp_scalars.data());
        return constraint(constr_id);
    }

private:
    template <linear_constraint LC>
    void _register_constraint(const constraint_id & constr_id, const LC & lc) {
        tmp_indices.emplace_back(static_cast<index>(tmp_variables.size()));
        const scalar b = lc.rhs();
        tmp_lower_bounds.emplace_back(
            (lc.sense() == constraint_sense::less_equal) ? -COIN_DBL_MAX : b);
        tmp_upper_bounds.emplace_back(
            (lc.sense() == constraint_sense::greater_equal) ? COIN_DBL_MAX : b);
        for(auto && [var, coef] : lc.linear_terms()) {
            tmp_variables.emplace_back(var.id());
            tmp_scalars.emplace_back(coef);
        }
    }
    template <typename Key, typename LastConstrLambda>
        requires linear_constraint<std::invoke_result_t<LastConstrLambda, Key>>
    void _register_first_valued_constraint(const constraint_id & constr_id,
                                           const Key & key,
                                           const LastConstrLambda & lc_lambda) {
        _register_constraint(constr_id, lc_lambda(key));
    }
    template <typename Key, typename OptConstrLambda, typename... Tail>
        requires detail::optional_type<
                     std::invoke_result_t<OptConstrLambda, Key>> &&
                 linear_constraint<detail::optional_type_value_t<
                     std::invoke_result_t<OptConstrLambda, Key>>>
    void _register_first_valued_constraint(
        const constraint_id & constr_id, const Key & key,
        const OptConstrLambda & opt_lc_lambda, const Tail &... tail) {
        if(const auto & opt_lc = opt_lc_lambda(key)) {
            _register_constraint(constr_id, opt_lc.value());
            return;
        }
        _register_first_valued_constraint(constr_id, key, tail...);
    }

public:
    template <ranges::range IR, typename... CL>
    auto add_constraints(IR && keys, CL... constraint_lambdas) {
        tmp_indices.resize(0);
        tmp_variables.resize(0);
        tmp_scalars.resize(0);
        tmp_lower_bounds.resize(0);
        tmp_upper_bounds.resize(0);
        const constraint_id offset =
            static_cast<constraint_id>(num_constraints());
        constraint_id constr_id = offset;
        for(auto && key : keys) {
            _register_first_valued_constraint(constr_id, key,
                                              constraint_lambdas...);
            ++constr_id;
        }
        tmp_indices.emplace_back(static_cast<index>(tmp_variables.size()));
        Clp.addRows(model, static_cast<int>(tmp_indices.size()) - 1,
                    tmp_lower_bounds.data(), tmp_upper_bounds.data(),
                    tmp_indices.data(), tmp_variables.data(),
                    tmp_scalars.data());
        return constraints_range(
            keys,
            ranges::view::transform(ranges::view::iota(offset, constr_id),
                                    [](auto && i) { return constraint{i}; }));
    }

    void set_constraint_rhs(constraint constr, scalar rhs) {
        if(get_constraint_sense(constr) == constraint_sense::greater_equal) {
            Clp.rowLower(model)[constr.id()] = rhs;
            return;
        }
        Clp.rowUpper(model)[constr.id()] = rhs;
    }
    void set_constraint_sense(constraint constr, constraint_sense r) {
        constraint_sense old_r = get_constraint_sense(constr);
        scalar old_rhs = get_constraint_rhs(constr);
        if(old_r == r) return;
        switch(r) {
            case constraint_sense::equal:
                Clp.rowLower(model)[constr.id()] =
                    Clp.rowUpper(model)[constr.id()] = old_rhs;
                return;
            case constraint_sense::less_equal:
                Clp.rowLower(model)[constr.id()] = -COIN_DBL_MAX;
                Clp.rowUpper(model)[constr.id()] = old_rhs;
                return;
            case constraint_sense::greater_equal:
                Clp.rowLower(model)[constr.id()] = old_rhs;
                Clp.rowUpper(model)[constr.id()] = COIN_DBL_MAX;
                return;
        }
    }
    constraint add_ranged_constraint(linear_expression auto && le, scalar lb,
                                     scalar ub) {
        constraint_id constr_id = static_cast<constraint_id>(num_constraints());
        tmp_variables.resize(0);
        tmp_scalars.resize(0);
        for(auto && [var, coef] : le.linear_terms()) {
            tmp_variables.emplace_back(var.id());
            tmp_scalars.emplace_back(coef);
        }
        lb -= le.constant();
        ub -= le.constant();
        index starts[2] = {0, static_cast<index>(tmp_variables.size())};
        Clp.addRows(model, 1, &lb, &ub, starts, tmp_variables.data(),
                    tmp_scalars.data());
        return constraint(constr_id);
    }
    void set_constraint_name(constraint constr, auto && name) {
        Clp.setRowName(model, constr.id(), const_cast<char *>(name.c_str()));
    }

    // auto get_constraint_lhs(constraint constr) const;
    scalar get_constraint_rhs(constraint constr) {
        if(get_constraint_sense(constr) == constraint_sense::greater_equal)
            return Clp.rowLower(model)[constr.id()];
        return Clp.rowUpper(model)[constr.id()];
    }
    constraint_sense get_constraint_sense(constraint constr) {
        const scalar lb = Clp.rowLower(model)[constr.id()];
        const scalar ub = Clp.rowUpper(model)[constr.id()];
        if(lb == ub) return constraint_sense::equal;
        if(lb == -COIN_DBL_MAX) return constraint_sense::less_equal;
        if(ub == COIN_DBL_MAX) return constraint_sense::greater_equal;
        throw std::runtime_error(
            "Tried to get the sense of a ranged constraint");
    }
    auto get_constraint_name(constraint constr) {
        auto max_length = static_cast<std::size_t>(Clp.lengthNames(model));
        std::string name(max_length, '\0');
        Clp.rowName(model, constr.id(), name.data());
        name.resize(std::strlen(name.c_str()));
        return name;
    }
    // auto get_constraint(const constraint constr) const;

    // void set_basic(variable v);
    // void set_non_basic(variable v);

    // void set_basic(constraint v);
    // void set_non_basic(constraint v);

    void set_feasibility_tolerance(scalar tol) {
        Clp.setPrimalTolerance(model, tol);
    }
    scalar get_feasibility_tolerance() { return Clp.primalTolerance(model); }

    void solve() {
        if(num_variables() == 0u) {
            opt_lp_status.emplace(lp_status::optimal);
            return;
        }
        Clp.primal(model, 0);
        switch(Clp.status(model)) {
            case 0:
                opt_lp_status.emplace(lp_status::optimal);
                return;
            case 1:
                opt_lp_status.emplace(lp_status::infeasible);
                return;
            case 2:
                opt_lp_status.emplace(lp_status::unbounded);
                return;
            default:
                opt_lp_status.reset();
        }
    }
    std::optional<lp_status> get_lp_status() const { return opt_lp_status; }

    scalar get_solution_value() { return Clp.getObjValue(model); }

    auto get_solution() {
        return variable_mapping(Clp.primalColumnSolution(model));
    }
    auto get_dual_solution() {
        return constraint_mapping(Clp.dualRowSolution(model));
    }
};

}  // namespace clp::v1_17
}  // namespace fhamonic::mippp

#endif  // MIPPP_CLP_v1_17_LP_HPP