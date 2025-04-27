#ifndef MIPPP_HIGHS_110_BASE_MODEL_HPP
#define MIPPP_HIGHS_110_BASE_MODEL_HPP

#include <limits>
#include <optional>
#include <vector>

#include <range/v3/view/iota.hpp>
#include <range/v3/view/span.hpp>
#include <range/v3/view/zip.hpp>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/highs/1.10/highs110_api.hpp"

namespace fhamonic {
namespace mippp {

class highs110_base_model {
public:
    using variable_id = HighsInt;
    using constraint_id = HighsInt;
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

protected:
    const highs110_api & Highs;
    void * model;

    std::vector<std::pair<constraint_id, unsigned int>>
        tmp_constraint_entry_cache;
    std::vector<HighsInt> tmp_indices;
    std::vector<int> tmp_variables;
    std::vector<double> tmp_scalars;
    std::vector<double> tmp_lower_bounds;
    std::vector<double> tmp_upper_bounds;

public:
    [[nodiscard]] explicit highs110_base_model(const highs110_api & api)
        : Highs(api), model(Highs.create()) {}
    ~highs110_base_model() { Highs.destroy(model); }

protected:
    void check(int status) {
        if(status == kHighsStatusError)
            throw std::runtime_error("highs110_base_model: error");
    }

public:
    std::size_t num_variables() {
        return static_cast<std::size_t>(Highs.getNumCol(model));
    }
    std::size_t num_constraints() {
        return static_cast<std::size_t>(Highs.getNumRow(model));
    }
    std::size_t num_entries() {
        return static_cast<std::size_t>(Highs.getNumNz(model));
    }

    void set_maximization() {
        check(Highs.changeObjectiveSense(model, kHighsObjSenseMaximize));
    }
    void set_minimization() {
        check(Highs.changeObjectiveSense(model, kHighsObjSenseMinimize));
    }

    void set_objective_offset(double offset) {
        check(Highs.changeObjectiveOffset(model, offset));
    }
    void set_objective(linear_expression auto && le) {
        auto num_vars = num_variables();
        tmp_scalars.resize(num_vars);
        std::fill(tmp_scalars.begin(), tmp_scalars.end(), 0.0);
        for(auto && [var, coef] : le.linear_terms()) {
            tmp_scalars[var.uid()] += coef;
        }
        check(Highs.changeColsCostByRange(
            model, 0, static_cast<HighsInt>(num_vars) - 1, tmp_scalars.data()));
        set_objective_offset(le.constant());
    }
    // void add_objective(linear_expression auto && le) {
    //     auto num_vars = num_variables();
    //     tmp_scalars.resize(num_vars);
    //     Highs.getColsByRange(model, 0, static_cast<HighsInt>(num_vars) - 1,
    //     NULL,
    //                          tmp_scalars.data(), NULL, NULL, NULL, NULL,
    //                          NULL, NULL);
    //     for(auto && [var, coef] : le.linear_terms()) {
    //         tmp_scalars[var.uid()] += coef;
    //     }
    //     check(Highs.changeColsCostByRange(
    //         model, 0, static_cast<HighsInt>(num_vars) - 1,
    //         tmp_scalars.data()));
    //     set_objective_offset(le.constant());
    // }
    double get_objective_offset() {
        double offset;
        check(Highs.getObjectiveOffset(model, &offset));
        return offset;
    }

protected:
    void _add_variable(const HighsInt & var_id, const variable_params & params,
                       int type) {
        check(
            Highs.addCol(model, params.obj_coef,
                         params.lower_bound.value_or(-Highs.getInfinity(model)),
                         params.upper_bound.value_or(Highs.getInfinity(model)),
                         0, NULL, NULL));
        if(type != kHighsVarTypeContinuous)
            check(Highs.changeColIntegrality(model, var_id, type));
    }
    void _add_variables(std::size_t offset, std::size_t count,
                        const variable_params & params, int type) {
        const auto diff_count = static_cast<std::ptrdiff_t>(count);
        tmp_scalars.resize(3 * count);
        std::fill(tmp_scalars.begin(), tmp_scalars.begin() + diff_count,
                  params.obj_coef);
        std::fill(tmp_scalars.begin() + diff_count,
                  tmp_scalars.begin() + 2 * diff_count,
                  params.lower_bound.value_or(-Highs.getInfinity(model)));
        std::fill(tmp_scalars.begin() + 2 * diff_count, tmp_scalars.end(),
                  params.upper_bound.value_or(Highs.getInfinity(model)));
        Highs.addCols(model, static_cast<HighsInt>(count), tmp_scalars.data(),
                      tmp_scalars.data() + diff_count,
                      tmp_scalars.data() + 2 * diff_count, 0, NULL, NULL, NULL);
        if(type != kHighsVarTypeContinuous) {
            tmp_variables.resize(count);
            std::fill(tmp_variables.begin(), tmp_variables.end(), type);
            check(Highs.changeColsIntegralityByRange(
                model, static_cast<HighsInt>(offset),
                static_cast<HighsInt>(count - 1), tmp_variables.data()));
        }
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
        HighsInt var_id = static_cast<HighsInt>(num_variables());
        _add_variable(var_id, params, kHighsVarTypeContinuous);
        return variable(var_id);
    }
    auto add_variables(
        std::size_t count,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params, kHighsVarTypeContinuous);
        return _make_variables_range(offset, count);
    }
    template <typename IL>
    auto add_variables(
        std::size_t count, IL && id_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params, kHighsVarTypeContinuous);
        return _make_indexed_variables_range(offset, count,
                                             std::forward<IL>(id_lambda));
    }

protected:
    void _set_variable_bounds(variable v, double lb, double ub) {
        check(Highs.changeColBounds(model, v.id(), lb, ub));
    }

public:
    // void set_objective_coefficient(variable v, double c) {
    //     Clp.objective(model)[v.id()] = c;
    // }
    // void set_variable_lower_bound(variable v, double lb) {
    //     Highs.changeColBounds(model, )
    //     Clp.columnLower(model)[v.id()] = lb;
    // }
    // void set_variable_upper_bound(variable v, double ub) {
    //     Clp.columnUpper(model)[v.id()] = ub;
    // }
    // void set_variable_name(variable v, std::string name) {
    //     Clp.setColumnName(model, v.id(), const_cast<char *>(name.c_str()));
    // }

    // double get_objective_coefficient(variable v) {
    //     return Clp.objective(model)[v.id()];
    // }
    // double get_variable_lower_bound(variable v) {
    //     return Clp.columnLower(model)[v.id()];
    // }
    // double get_variable_upper_bound(variable v) {
    //     return Clp.columnUpper(model)[v.id()];
    // }

    constraint add_constraint(linear_constraint auto && lc) {
        HighsInt constr_id = static_cast<HighsInt>(num_constraints());
        tmp_constraint_entry_cache.resize(num_variables());
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
        const double b = lc.rhs();
        check(Highs.addRow(model,
                           (lc.sense() == constraint_sense::less_equal)
                               ? -Highs.getInfinity(model)
                               : b,
                           (lc.sense() == constraint_sense::greater_equal)
                               ? Highs.getInfinity(model)
                               : b,
                           static_cast<HighsInt>(tmp_variables.size()),
                           tmp_variables.data(), tmp_scalars.data()));
        return constraint(constr_id);
    }

private:
    template <linear_constraint LC>
    void _register_constraint(const HighsInt & constr_id, const LC & lc) {
        tmp_indices.emplace_back(static_cast<HighsInt>(tmp_variables.size()));
        const double b = lc.rhs();
        tmp_lower_bounds.emplace_back(
            (lc.sense() == constraint_sense::less_equal)
                ? -Highs.getInfinity(model)
                : b);
        tmp_upper_bounds.emplace_back(
            (lc.sense() == constraint_sense::greater_equal)
                ? Highs.getInfinity(model)
                : b);
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
    }
    template <typename Key, typename LastConstrLambda>
        requires linear_constraint<std::invoke_result_t<LastConstrLambda, Key>>
    void _register_first_valued_constraint(const HighsInt & constr_id,
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
        const HighsInt & constr_id, const Key & key,
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
        tmp_constraint_entry_cache.resize(num_variables());
        tmp_indices.resize(0);
        tmp_variables.resize(0);
        tmp_scalars.resize(0);
        tmp_lower_bounds.resize(0);
        tmp_upper_bounds.resize(0);
        const HighsInt offset = static_cast<HighsInt>(num_constraints());
        HighsInt constr_id = offset;
        for(auto && key : keys) {
            _register_first_valued_constraint(constr_id, key,
                                              constraint_lambdas...);
            ++constr_id;
        }
        check(Highs.addRows(model, static_cast<HighsInt>(tmp_indices.size()),
                            tmp_lower_bounds.data(), tmp_upper_bounds.data(),
                            static_cast<HighsInt>(tmp_variables.size()),
                            tmp_indices.data(), tmp_variables.data(),
                            tmp_scalars.data()));
        return constraints_range(
            keys,
            ranges::view::transform(ranges::view::iota(offset, constr_id),
                                    [](auto && i) { return constraint{i}; }));
    }
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_HIGHS_110_BASE_MODEL_HPP