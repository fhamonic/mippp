#ifndef MIPPP_HIGHS_v1_10_BASE_HPP
#define MIPPP_HIGHS_v1_10_BASE_HPP

#include <limits>
#include <memory>
#include <optional>
#include <vector>

#include <range/v3/view/iota.hpp>
#include <range/v3/view/span.hpp>
#include <range/v3/view/zip.hpp>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/highs/v1_10/highs_api.hpp"
#include "mippp/solvers/model_base.hpp"

namespace fhamonic::mippp {
namespace highs::v1_10 {

class highs_base : public model_base<int, double> {
public:
    using index = HighsInt;
    using variable_id = HighsInt;
    using constraint_id = HighsInt;
    using scalar = double;
    using variable = model_variable<variable_id, scalar>;
    using constraint = model_constraint<constraint_id>;
    template <typename Map>
    using variable_mapping = entity_mapping<variable, Map>;
    template <typename Map>
    using constraint_mapping = entity_mapping<constraint, Map>;

protected:
    const highs_api & Highs;
    void * model;

    std::vector<index> tmp_begins;
    std::vector<scalar> tmp_lower_bounds;
    std::vector<scalar> tmp_upper_bounds;

public:
    [[nodiscard]] explicit highs_base(const highs_api & api)
        : model_base<int, double>(), Highs(api), model(Highs.create()) {}
    ~highs_base() { Highs.destroy(model); }

protected:
    void check(int status) {
        if(status == kHighsStatusError)
            throw std::runtime_error("highs_base: error");
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

    void set_objective_offset(scalar offset) {
        check(Highs.changeObjectiveOffset(model, offset));
    }
    void set_objective(linear_expression auto && le) {
        const auto num_vars = num_variables();
        tmp_scalars.resize(num_vars);
        std::fill(tmp_scalars.begin(), tmp_scalars.end(), 0.0);
        for(auto && [var, coef] : le.linear_terms()) {
            tmp_scalars[var.uid()] += coef;
        }
        check(Highs.changeColsCostByRange(
            model, 0, static_cast<HighsInt>(num_vars) - 1, tmp_scalars.data()));
        set_objective_offset(le.constant());
    }
    void add_objective(linear_expression auto && le) {
        auto num_vars = num_variables();
        tmp_scalars.resize(num_vars);
        int dummy_int;
        Highs.getColsByRange(model, 0, static_cast<HighsInt>(num_vars) - 1,
                             &dummy_int, tmp_scalars.data(), NULL, NULL,
                             &dummy_int, NULL, NULL, NULL);
        for(auto && [var, coef] : le.linear_terms()) {
            tmp_scalars[var.uid()] += coef;
        }
        check(Highs.changeColsCostByRange(
            model, 0, static_cast<HighsInt>(num_vars) - 1, tmp_scalars.data()));
        set_objective_offset(get_objective_offset() + le.constant());
    }
    scalar get_objective_offset() {
        scalar offset;
        check(Highs.getObjectiveOffset(model, &offset));
        return offset;
    }
    auto get_objective() {
        const auto num_vars = num_variables();
        auto coefs = std::make_shared_for_overwrite<double[]>(num_vars);
        int dummy_int;
        check(Highs.getColsByRange(
            model, 0, static_cast<HighsInt>(num_vars) - 1, &dummy_int,
            coefs.get(), NULL, NULL, &dummy_int, NULL, NULL, NULL));
        return linear_expression_view(
            ranges::view::transform(
                ranges::view::iota(variable_id{0},
                                   static_cast<variable_id>(num_vars)),
                [coefs = std::move(coefs)](auto i) {
                    return std::make_pair(variable(i), coefs[i]);
                }),
            get_objective_offset());
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
            tmp_indices.resize(count);
            std::fill(tmp_indices.begin(), tmp_indices.end(), type);
            check(Highs.changeColsIntegralityByRange(
                model, static_cast<HighsInt>(offset),
                static_cast<HighsInt>(count - 1), tmp_indices.data()));
        }
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
    void _set_variable_bounds(variable v, scalar lb, scalar ub) {
        check(Highs.changeColBounds(model, v.id(), lb, ub));
    }

public:
    void set_objective_coefficient(variable v, scalar c) {
        check(Highs.changeColCost(model, v.id(), c));
    }
    void set_variable_lower_bound(variable v, scalar lb) {
        _set_variable_bounds(v, lb, get_variable_upper_bound(v));
    }
    void set_variable_upper_bound(variable v, scalar ub) {
        _set_variable_bounds(v, get_variable_lower_bound(v), ub);
    }

    scalar get_objective_coefficient(variable v) {
        scalar coef;
        index dummy_int;
        scalar dummy_dbl;
        check(Highs.getColsByRange(model, v.id(), v.id(), &dummy_int, &coef,
                                   &dummy_dbl, &dummy_dbl, &dummy_int, NULL,
                                   NULL, NULL));
        return coef;
    }
    scalar get_variable_lower_bound(variable v) {
        scalar lb;
        int dummy_int;
        double dummy_dbl;
        check(Highs.getColsByRange(model, v.id(), v.id(), &dummy_int,
                                   &dummy_dbl, &lb, &dummy_dbl, &dummy_int,
                                   NULL, NULL, NULL));
        return lb;
    }
    scalar get_variable_upper_bound(variable v) {
        scalar ub;
        int dummy_int;
        double dummy_dbl;
        check(Highs.getColsByRange(model, v.id(), v.id(), &dummy_int,
                                   &dummy_dbl, &dummy_dbl, &ub, &dummy_int,
                                   NULL, NULL, NULL));
        return ub;
    }

    constraint add_constraint(linear_constraint auto && lc) {
        HighsInt constr_id = static_cast<HighsInt>(num_constraints());
        _reset_cache(num_variables());
        _register_entries(lc.linear_terms());
        const scalar b = lc.rhs();
        check(Highs.addRow(model,
                           (lc.sense() == constraint_sense::less_equal)
                               ? -Highs.getInfinity(model)
                               : b,
                           (lc.sense() == constraint_sense::greater_equal)
                               ? Highs.getInfinity(model)
                               : b,
                           static_cast<HighsInt>(tmp_indices.size()),
                           tmp_indices.data(), tmp_scalars.data()));
        return constraint(constr_id);
    }

private:
    template <linear_constraint LC>
    void _register_constraint(const HighsInt & constr_id, const LC & lc) {
        tmp_begins.emplace_back(static_cast<HighsInt>(tmp_indices.size()));
        const scalar b = lc.rhs();
        tmp_lower_bounds.emplace_back(
            (lc.sense() == constraint_sense::less_equal)
                ? -Highs.getInfinity(model)
                : b);
        tmp_upper_bounds.emplace_back(
            (lc.sense() == constraint_sense::greater_equal)
                ? Highs.getInfinity(model)
                : b);
        _register_entries(lc.linear_terms());
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
        _reset_cache(num_variables());
        tmp_begins.resize(0);
        tmp_lower_bounds.resize(0);
        tmp_upper_bounds.resize(0);
        const HighsInt offset = static_cast<HighsInt>(num_constraints());
        HighsInt constr_id = offset;
        for(auto && key : keys) {
            _register_first_valued_constraint(constr_id, key,
                                              constraint_lambdas...);
            ++constr_id;
        }
        check(Highs.addRows(model, static_cast<HighsInt>(tmp_begins.size()),
                            tmp_lower_bounds.data(), tmp_upper_bounds.data(),
                            static_cast<HighsInt>(tmp_indices.size()),
                            tmp_begins.data(), tmp_indices.data(),
                            tmp_scalars.data()));
        return constraints_range(
            keys,
            ranges::view::transform(ranges::view::iota(offset, constr_id),
                                    [](auto && i) { return constraint{i}; }));
    }
};

}  // namespace highs::v1_10
}  // namespace fhamonic::mippp

#endif  // MIPPP_HIGHS_v1_10_BASE_HPP