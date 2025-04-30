#ifndef MIPPP_CBC_v2_10_12_MILP_HPP
#define MIPPP_CBC_v2_10_12_MILP_HPP

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
#include "mippp/model_entities.hpp"

#include "mippp/solvers/cbc/v2_10_12/cbc_api.hpp"

namespace fhamonic::mippp {
namespace cbc::v2_10_12 {

class cbc_milp {
public:
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
    const cbc_api & Cbc;
    Cbc_Model * model;
    double objective_offset;
    double feasibility_tol;

    static constexpr char constraint_sense_to_cbc_sense(constraint_sense rel) {
        if(rel == constraint_sense::less_equal) return 'L';
        if(rel == constraint_sense::equal) return 'E';
        return 'G';
    }
    // static constexpr constraint_sense cbc_sense_to_constraint_sense(
    //     char sense) {
    //     if(sense == Clp_LESS_EQUAL) return
    //     constraint_sense::less_equal; if(sense == Clp_EQUAL) return
    //     constraint_sense::equal; return
    //     constraint_sense::greater_equal;
    // }

    std::size_t _lazy_num_variables;
    std::size_t _lazy_num_constraints;

    std::vector<std::pair<constraint_id, unsigned int>>
        tmp_constraint_entry_cache;
    std::vector<int> tmp_variables;
    std::vector<double> tmp_scalars;

public:
    [[nodiscard]] explicit cbc_milp(const cbc_api & api)
        : Cbc(api)
        , model(Cbc.newModel())
        , objective_offset(0.0)
        , feasibility_tol(1e-4)
        , _lazy_num_variables(0)
        , _lazy_num_constraints(0) {}
    ~cbc_milp() { Cbc.deleteModel(model); }

    std::size_t num_variables() {
        if(static_cast<std::size_t>(Cbc.getNumCols(model)) !=
           _lazy_num_variables)
            throw std::runtime_error(
                "cbc_milp : _lazy_num_variables differs from Cbc one.");
        return _lazy_num_variables;
    }
    std::size_t num_constraints() {
        if(static_cast<std::size_t>(Cbc.getNumRows(model)) !=
           _lazy_num_constraints)
            throw std::runtime_error(
                "cbc_milp : _lazy_num_constraints differs from Cbc one.");
        return _lazy_num_constraints;
    }
    std::size_t num_entries() {
        return static_cast<std::size_t>(Cbc.getNumElements(model));
    }

    void set_maximization() { Cbc.setObjSense(model, -1); }
    void set_minimization() { Cbc.setObjSense(model, 1); }

    void set_objective_offset(double offset) { objective_offset = offset; }
    void set_objective(linear_expression auto && le) {
        for(auto && v :
            std::views::iota(0, static_cast<int>(_lazy_num_variables))) {
            Cbc.setObjCoeff(model, v, 0.0);
        }
        for(auto && [var, coef] : le.linear_terms()) {
            set_objective_coefficient(var,
                                      get_objective_coefficient(var) + coef);
        }
        set_objective_offset(le.constant());
    }
    void add_objective(linear_expression auto && le) {
        for(auto && [var, coef] : le.linear_terms()) {
            set_objective_coefficient(var,
                                      get_objective_coefficient(var) + coef);
        }
        set_objective_offset(get_objective_offset() + le.constant());
    }
    double get_objective_offset() { return objective_offset; }
    auto get_objective() {
        return linear_expression_view(
            ranges::view::transform(
                ranges::view::iota(0, static_cast<int>(_lazy_num_variables)),
                [this](auto && v) {
                    return std::make_pair(v, Cbc.getObjCoefficients(model)[v]);
                }),
            get_objective_offset());
    }

private:
    inline void _add_var(const variable_params & p, const bool is_integer) {
        Cbc.addCol(model, "", p.lower_bound.value_or(-COIN_DBL_MAX),
                   p.upper_bound.value_or(COIN_DBL_MAX), p.obj_coef, is_integer,
                   0, NULL, NULL);
        ++_lazy_num_variables;
    }
    inline auto _make_variables_range(const std::size_t & offset,
                                      const std::size_t & count) {
        return make_variables_range(ranges::view::transform(
            ranges::view::iota(static_cast<variable_id>(offset),
                               static_cast<variable_id>(offset + count)),
            [](auto && i) { return variable{i}; }));
    }
    template <typename IL>
    inline auto _make_indexed_variables_range(const std::size_t & offset,
                                              const std::size_t & count,
                                              IL && id_lambda) {
        return make_indexed_variables_range(
            typename detail::function_traits<IL>::arg_types(),
            ranges::view::transform(
                ranges::view::iota(static_cast<variable_id>(offset),
                                   static_cast<variable_id>(offset + count)),
                [](auto && i) { return variable{i}; }),
            std::forward<IL>(id_lambda));
    }

public:
    variable add_variable(
        const variable_params params = default_variable_params) {
        int var_id = static_cast<int>(_lazy_num_variables);
        _add_var(params, false);
        return variable(var_id);
    }
    auto add_variables(std::size_t count,
                       const variable_params params = default_variable_params) {
        const std::size_t offset = _lazy_num_variables;
        for(std::size_t i = 0; i < count; ++i) _add_var(params, false);
        return _make_variables_range(offset, count);
    }
    template <typename IL>
    auto add_variables(
        std::size_t count, IL && id_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = _lazy_num_variables;
        for(std::size_t i = 0; i < count; ++i) _add_var(params, false);
        return _make_indexed_variables_range(offset, count,
                                             std::forward<IL>(id_lambda));
    }
    variable add_integer_variable(
        const variable_params params = default_variable_params) {
        int var_id = static_cast<int>(_lazy_num_variables);
        _add_var(params, true);
        return variable(var_id);
    }
    auto add_integer_variables(std::size_t count, const variable_params params =
                                                      default_variable_params) {
        const std::size_t offset = _lazy_num_variables;
        for(std::size_t i = 0; i < count; ++i) _add_var(params, true);
        return _make_variables_range(offset, count);
    }
    template <typename IL>
    auto add_integer_variables(
        std::size_t count, IL && id_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = _lazy_num_variables;
        for(std::size_t i = 0; i < count; ++i) _add_var(params, true);
        return _make_indexed_variables_range(offset, count,
                                             std::forward<IL>(id_lambda));
    }
    variable add_binary_variable() {
        int var_id = static_cast<int>(_lazy_num_variables);
        _add_var(variable_params{.lower_bound = 0, .upper_bound = 1}, true);
        return variable(var_id);
    }
    auto add_binary_variables(std::size_t count) {
        const std::size_t offset = _lazy_num_variables;
        for(std::size_t i = 0; i < count; ++i)
            _add_var(variable_params{.lower_bound = 0, .upper_bound = 1}, true);
        return _make_variables_range(offset, count);
    }
    template <typename IL>
    auto add_binary_variables(std::size_t count, IL && id_lambda) noexcept {
        const std::size_t offset = _lazy_num_variables;
        for(std::size_t i = 0; i < count; ++i)
            _add_var(variable_params{.lower_bound = 0, .upper_bound = 1}, true);
        return _make_indexed_variables_range(offset, count,
                                             std::forward<IL>(id_lambda));
    }

    void set_continuous(variable v) noexcept {
        Cbc.setContinuous(model, v.id());
    }
    void set_integer(variable v) noexcept { Cbc.setInteger(model, v.id()); }
    void set_binary(variable v) noexcept {
        set_integer(v);
        set_variable_lower_bound(v, 0);
        set_variable_upper_bound(v, 1);
    }

    void set_objective_coefficient(variable v, double c) {
        Cbc.setObjCoeff(model, v.id(), c);
    }
    void set_variable_lower_bound(variable v, double lb) {
        Cbc.setColLower(model, v.id(), lb);
    }
    void set_variable_upper_bound(variable v, double ub) {
        Cbc.setColUpper(model, v.id(), ub);
    }
    void set_variable_name(variable v, std::string name) {
        Cbc.setColName(model, v.id(), const_cast<char *>(name.c_str()));
    }

    double get_objective_coefficient(variable v) {
        return Cbc.getObjCoefficients(model)[v.id()];
    }
    double get_variable_lower_bound(variable v) {
        return Cbc.getColLower(model)[v.id()];
    }
    double get_variable_upper_bound(variable v) {
        return Cbc.getColUpper(model)[v.id()];
    }
    auto get_variable_name(variable v) {
        auto max_length = Cbc.maxNameLength(model);
        std::string name(max_length, '\0');
        Cbc.getColName(model, v.id(), name.data(), max_length);
        name.resize(std::strlen(name.c_str()));
        return name;
    }

private:
    template <linear_constraint LC>
    void _add_constraint(const int & constr_id, const LC & lc) {
        tmp_variables.resize(0);
        tmp_scalars.resize(0);
        for(auto && [var, coef] : lc.linear_terms()) {
            auto & p = tmp_constraint_entry_cache[var.uid()];
            if(p.first == constr_id + 1) {
                tmp_scalars[p.second] += coef;
                continue;
            }
            p = std::make_pair(constr_id + 1, tmp_variables.size());
            tmp_variables.emplace_back(var.id());
            tmp_scalars.emplace_back(coef);
        }
        Cbc.addRow(model, "", static_cast<int>(tmp_variables.size()),
                   tmp_variables.data(), tmp_scalars.data(),
                   constraint_sense_to_cbc_sense(lc.sense()), lc.rhs());
        ++_lazy_num_constraints;
    }

public:
    constraint add_constraint(linear_constraint auto && lc) {
        tmp_constraint_entry_cache.resize(_lazy_num_variables);
        int constr_id = static_cast<int>(_lazy_num_constraints);
        _add_constraint(constr_id, lc);
        return constraint(constr_id);
    }

private:
    template <typename Key, typename LastConstrLambda>
        requires linear_constraint<std::invoke_result_t<LastConstrLambda, Key>>
    void _add_first_valued_constraint(const int & constr_id, const Key & key,
                                      const LastConstrLambda & lc_lambda) {
        _add_constraint(constr_id, lc_lambda(key));
    }
    template <typename Key, typename OptConstrLambda, typename... Tail>
        requires detail::optional_type<
                     std::invoke_result_t<OptConstrLambda, Key>> &&
                 linear_constraint<detail::optional_type_value_t<
                     std::invoke_result_t<OptConstrLambda, Key>>>
    void _add_first_valued_constraint(const int & constr_id, const Key & key,
                                      const OptConstrLambda & opt_lc_lambda,
                                      const Tail &... tail) {
        if(const auto & opt_lc = opt_lc_lambda(key)) {
            _add_constraint(constr_id, opt_lc.value());
            return;
        }
        _add_first_valued_constraint(constr_id, key, tail...);
    }

public:
    template <ranges::range IR, typename... CL>
    auto add_constraints(IR && keys, CL... constraint_lambdas) {
        tmp_constraint_entry_cache.resize(_lazy_num_variables);
        const int offset = static_cast<int>(_lazy_num_constraints);
        int constr_id = offset;
        for(auto && key : keys) {
            _add_first_valued_constraint(constr_id, key, constraint_lambdas...);
            ++constr_id;
        }
        return constraints_range(
            keys,
            ranges::view::transform(ranges::view::iota(offset, constr_id),
                                    [](auto && i) { return constraint{i}; }));
    }

    // void set_constraint_rhs(constraint constr, double rhs) {
    // if(get_constraint_sense(constr) ==
    // constraint_sense::greater_equal) {
    //     Clp.rowLower(model)[constr] = rhs;
    //     return;
    // }
    // Clp.rowUpper(model)[constr] = rhs;
    // }
    // void set_constraint_sense(constraint constr, constraint_sense r) {
    // constraint_sense old_r = get_constraint_sense(constr);
    // double old_rhs = get_constraint_rhs(constr);
    // if(old_r == r) return;
    // switch(r) {
    //     case constraint_sense::equal:
    //         Clp.rowLower(model)[constr] = Clp.rowUpper(model)[constr] =
    //         old_rhs; return;
    //     case constraint_sense::less_equal:
    //         Clp.rowLower(model)[constr] = -COIN_DBL_MAX;
    //         Clp.rowUpper(model)[constr] = old_rhs;
    //         return;
    //     case constraint_sense::greater_equal:
    //         Clp.rowLower(model)[constr] = old_rhs;
    //         Clp.rowUpper(model)[constr] = COIN_DBL_MAX;
    //         return;
    // }
    // }
    constraint add_ranged_constraint(linear_expression auto && le, double lb,
                                     double ub) {
        const int constr_id = static_cast<int>(_lazy_num_constraints);
        tmp_variables.resize(0);
        tmp_scalars.resize(0);
        for(auto && [var, coef] : le.linear_terms()) {
            tmp_variables.emplace_back(var.id());
            tmp_scalars.emplace_back(coef);
        }
        const double c = le.constant();
        Cbc.addRow(model, "", static_cast<int>(tmp_variables.size()),
                   tmp_variables.data(), tmp_scalars.data(), 'L', ub - c);
        Cbc.setRowLower(model, constr_id, lb - c);
        ++_lazy_num_constraints;
        return constraint(constr_id);
    }
    // void set_constraint_name(constraint constr, auto && name);

    // auto get_constraint_lexpr(constraint constr) const;
    constraint_sense get_constraint_sense(constraint constr) {
        const double lb = Cbc.getRowLower(model)[constr.id()];
        const double ub = Cbc.getRowUpper(model)[constr.id()];
        if(lb == ub) return constraint_sense::equal;
        if(lb == -COIN_DBL_MAX) return constraint_sense::less_equal;
        if(ub == COIN_DBL_MAX) return constraint_sense::greater_equal;
        throw std::runtime_error(
            "Tried to get the sense of a ranged constraint");
    }
    double get_constraint_rhs(constraint constr) {
        return Cbc.getRowRHS(model, constr.id());
    }
    auto get_constraint_name(constraint constr) {
        auto max_length = Cbc.maxNameLength(model);
        std::string name(max_length, '\0');
        Cbc.getRowName(model, constr.id(), name.data(), max_length);
        name.resize(std::strlen(name.c_str()));
        return name;
    }
    // auto get_constraint(const constraint constr) const;

    void set_feasibility_tolerance(double tol) {
        feasibility_tol = tol;
        auto tol_s = std::to_string(tol);
        Cbc.setParameter(model, "-primalTolerance=", tol_s.c_str());
        Cbc.setParameter(model, "-dualTolerance=", tol_s.c_str());
    }
    double get_feasibility_tolerance() { return feasibility_tol; }

    void set_optimality_tolerance(double tol) {
        Cbc.setAllowableFractionGap(model, tol);
    }
    double get_optimality_tolerance() {
        return Cbc.getAllowableFractionGap(model);
    }

    void solve() {
        if(_lazy_num_constraints == 0u) return;
        Cbc.solve(model);
    }

    double get_solution_value() {
        return objective_offset +
               (_lazy_num_variables == 0u ? 0.0 : Cbc.getObjValue(model));
    }

    auto get_solution() { return variable_mapping(Cbc.getColSolution(model)); }
};

}  // namespace cbc::v2_10_12
}  // namespace fhamonic::mippp

#endif  // MIPPP_CBC_v2_10_12_MILP_HPP