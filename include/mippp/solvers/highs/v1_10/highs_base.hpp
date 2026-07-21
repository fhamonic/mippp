#pragma once

#include <cstring>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
#include <ranges>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/highs/v1_10/highs_api.hpp"
#include "mippp/solvers/remapping_model_base.hpp"

namespace mippp {
namespace highs::v1_10 {

class highs_base : public remapping_model_base<int, double> {
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

    void check(const int status) { Highs._check(status); }

    std::vector<index> tmp_begins;
    std::vector<scalar> tmp_lower_bounds;
    std::vector<scalar> tmp_upper_bounds;

public:
    [[nodiscard]] explicit highs_base(const highs_api & api)
        : remapping_model_base<int, double>()
        , Highs(api)
        , model(Highs.create()) {}
    ~highs_base() {
        if(model) Highs.destroy(model);
    }

    constexpr highs_base(const highs_base &) = delete;
    constexpr highs_base(highs_base && other) noexcept
        : remapping_model_base<int, double>(std::move(other))
        , Highs(other.Highs)
        , model(other.model)
        , tmp_begins(std::move(other.tmp_begins))
        , tmp_lower_bounds(std::move(other.tmp_lower_bounds))
        , tmp_upper_bounds(std::move(other.tmp_upper_bounds)) {
        other.model = nullptr;
    }

    constexpr highs_base & operator=(const highs_base &) = delete;
    constexpr highs_base & operator=(highs_base && other) = delete;

protected:
    std::size_t _num_var_native_ids() {
        return static_cast<std::size_t>(Highs.getNumCol(model));
    }
    int _new_var_native_id() {
        if(_remap_ids) _extend_handle_ids_map(1);
        return static_cast<int>(num_variables());
    }

    void _lazily_remove_variables() {
        if(_var_handles_to_delete.empty()) return;
        tmp_indices.resize(0);
        for(const variable & var : _var_handles_to_delete)
            tmp_indices.emplace_back(_native_id(var));
        std::ranges::sort(tmp_indices);

        const std::size_t old_num_native_ids = _num_var_native_ids();
        check(Highs.deleteColsBySet(model, static_cast<int>(tmp_indices.size()),
                                    tmp_indices.data()));

        const std::size_t new_num_native_ids =
            old_num_native_ids - tmp_indices.size();
        // // Skips remapping if all deletiond are the native ids tail
        if(_remap_ids ||
           static_cast<std::size_t>(tmp_indices.front()) < new_num_native_ids) {
            if(!_remap_ids) {
                _native_ids_map.resize(old_num_native_ids);
                _handle_ids_map.resize(old_num_native_ids);
                std::ranges::iota(_native_ids_map, 0);
                std::ranges::iota(_handle_ids_map, 0);
                _remap_ids = true;
            }

            std::size_t offset = 0;
            for(int old_native_id :
                std::views::iota(0, static_cast<int>(old_num_native_ids))) {
                if(offset < tmp_indices.size() &&
                   old_native_id == tmp_indices[offset]) {
                    ++offset;
                    continue;
                }
                const int handle_id =
                    _handle_ids_map[static_cast<std::size_t>(old_native_id)];
                const int new_native_id =
                    old_native_id - static_cast<int>(offset);
                _handle_ids_map[static_cast<std::size_t>(new_native_id)] =
                    handle_id;
                _native_ids_map[static_cast<std::size_t>(handle_id)] =
                    new_native_id;
            }
            _shrink_handle_ids_map(_var_handles_to_delete.size());
#if defined(__cpp_lib_containers_ranges)
            _free_var_handles.append_range(_var_handles_to_delete);
#else
            _free_var_handles.insert(_free_var_handles.end(),
                                     _var_handles_to_delete.cbegin(),
                                     _var_handles_to_delete.cend());
#endif
        }
        _var_handles_to_delete.clear();
    }

public:
    std::size_t num_variables() {
        return _num_var_native_ids() - _var_handles_to_delete.size();
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
    template <linear_expression LE>
    void set_objective(LE && le) {
        const auto num_vars = _num_var_native_ids();
        tmp_scalars.resize(num_vars);
        std::fill(tmp_scalars.begin(), tmp_scalars.end(), 0.0);
        for(auto && [var, coef] : le.linear_terms()) {
            tmp_scalars[static_cast<std::size_t>(_native_id(var))] += coef;
        }
        check(Highs.changeColsCostByRange(
            model, 0, static_cast<HighsInt>(num_vars) - 1, tmp_scalars.data()));
        set_objective_offset(le.constant());
    }
    void add_objective(linear_expression auto && le) {
        _reset_cache(_num_var_native_ids());
        _register_entries(le.linear_terms());
        const std::size_t num_entries = tmp_indices.size();
        std::ranges::sort(std::views::zip(tmp_indices, tmp_scalars),
                          [](const auto & e1, const auto & e2) {
                              return std::get<0>(e1) < std::get<0>(e2);
                          });
        tmp_scalars.resize(2 * num_entries);
        int dummy_int;
        check(Highs.getColsBySet(
            model, static_cast<HighsInt>(num_entries), tmp_indices.data(),
            &dummy_int, tmp_scalars.data() + num_entries, nullptr, nullptr,
            &dummy_int, nullptr, nullptr, nullptr));

        for(std::size_t i = 0; i < num_entries; ++i) {
            tmp_scalars[i] += tmp_scalars[num_entries + i];
        }
        check(
            Highs.changeColsCostBySet(model, static_cast<HighsInt>(num_entries),
                                      tmp_indices.data(), tmp_scalars.data()));
        set_objective_offset(get_objective_offset() + le.constant());
    }
    scalar get_objective_offset() {
        scalar offset;
        check(Highs.getObjectiveOffset(model, &offset));
        return offset;
    }
    auto get_objective() {
        const auto num_vars = _num_var_native_ids();
        auto coefs = std::make_shared_for_overwrite<double[]>(num_vars);
        int dummy_int;
        check(Highs.getColsByRange(model, 0,
                                   static_cast<HighsInt>(num_vars) - 1,
                                   &dummy_int, coefs.get(), nullptr, nullptr,
                                   &dummy_int, nullptr, nullptr, nullptr));
        return linear_expression_view(
            std::views::transform(
                std::views::iota(variable_id{0},
                                 static_cast<variable_id>(num_vars)),
                [this, coefs = std::move(coefs)](auto i) {
                    return std::make_pair(_var_handle(i), coefs[i]);
                }),
            get_objective_offset());
    }

protected:
    variable _add_variable(const variable_params & params, int type) {
        HighsInt var_id = _new_var_native_id();
        check(
            Highs.addCol(model, params.obj_coef,
                         params.lower_bound.value_or(-Highs.getInfinity(model)),
                         params.upper_bound.value_or(Highs.getInfinity(model)),
                         0, nullptr, nullptr));
        if(type != kHighsVarTypeContinuous)
            check(Highs.changeColIntegrality(model, var_id, type));
        return _new_var_handle(var_id);
    }
    std::size_t _add_variables(std::size_t count,
                               const variable_params & params, int type) {
        if(_remap_ids) _extend_handle_ids_map(count);
        const std::size_t offset = _num_var_native_ids();
        const std::size_t handle_ids_begin =
            _new_var_handle_range(offset, count);

        const auto diff_count = static_cast<std::ptrdiff_t>(count);
        tmp_scalars.resize(3 * count);
        std::fill(tmp_scalars.begin(), tmp_scalars.begin() + diff_count,
                  params.obj_coef);
        std::fill(tmp_scalars.begin() + diff_count,
                  tmp_scalars.begin() + 2 * diff_count,
                  params.lower_bound.value_or(-Highs.getInfinity(model)));
        std::fill(tmp_scalars.begin() + 2 * diff_count, tmp_scalars.end(),
                  params.upper_bound.value_or(Highs.getInfinity(model)));
        check(Highs.addCols(model, static_cast<HighsInt>(count),
                            tmp_scalars.data(), tmp_scalars.data() + diff_count,
                            tmp_scalars.data() + 2 * diff_count, 0, nullptr,
                            nullptr, nullptr));
        if(type != kHighsVarTypeContinuous) {
            tmp_indices.resize(count);
            std::fill(tmp_indices.begin(), tmp_indices.end(), type);
            check(Highs.changeColsIntegralityByRange(
                model, static_cast<HighsInt>(offset),
                static_cast<HighsInt>(offset + count - 1), tmp_indices.data()));
        }
        return handle_ids_begin;
    }

public:
    variable add_variable(
        const variable_params params = default_variable_params) {
        return _add_variable(params, kHighsVarTypeContinuous);
    }
    auto add_variables(
        std::size_t count,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset =
            _add_variables(count, params, kHighsVarTypeContinuous);
        return _make_variables_view(offset, count);
    }
    template <typename IL>
    auto add_variables(
        std::size_t count, IL && id_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset =
            _add_variables(count, params, kHighsVarTypeContinuous);
        return _make_indexed_variables_view(offset, count,
                                             std::forward<IL>(id_lambda));
    }

    variable add_named_variable(
        const std::string & name,
        const variable_params params = default_variable_params) {
        variable v = add_variable(params);
        set_variable_name(v, name);
        return v;
    }
    template <typename NL>
    auto add_named_variables(
        std::size_t count, NL && name_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset =
            _add_variables(count, params, kHighsVarTypeContinuous);
        return _make_named_variables_view(offset, count,
                                           std::forward<NL>(name_lambda), this);
    }
    template <typename IL, typename NL>
    auto add_named_variables(
        std::size_t count, IL && id_lambda, NL && name_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset =
            _add_variables(count, params, kHighsVarTypeContinuous);
        return _make_indexed_named_variables_view(
            offset, count, std::forward<IL>(id_lambda),
            std::forward<NL>(name_lambda), this);
    }

private:
    template <typename ER>
    inline variable _add_column(ER && entries, const variable_params & params) {
        const int var_id = _new_var_native_id();
        _reset_raw_cache();
        _register_raw_entries(entries);
        check(
            Highs.addCol(model, params.obj_coef,
                         params.lower_bound.value_or(-Highs.getInfinity(model)),
                         params.upper_bound.value_or(Highs.getInfinity(model)),
                         static_cast<HighsInt>(tmp_indices.size()),
                         tmp_indices.data(), tmp_scalars.data()));
        return _new_var_handle(var_id);
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
        _var_handles_to_delete.emplace_back(v);
        _lazily_remove_variables();
    }
    template <std::ranges::range VR>
    void remove_variables(VR && variables) {
#if defined(__cpp_lib_containers_ranges)
        _var_handles_to_delete.append_range(variables);
#else
        _var_handles_to_delete.insert(_var_handles_to_delete.end(),
                                      variables.cbegin(), variables.cend());
#endif
        _lazily_remove_variables();
    }

protected:
    void _set_variable_bounds(variable v, scalar lb, scalar ub) {
        check(Highs.changeColBounds(model, _native_id(v), lb, ub));
    }

public:
    void set_objective_coefficient(variable v, scalar c) {
        check(Highs.changeColCost(model, _native_id(v), c));
    }
    void set_variable_lower_bound(variable v, scalar lb) {
        _set_variable_bounds(v, lb, get_variable_upper_bound(v));
    }
    void set_variable_upper_bound(variable v, scalar ub) {
        _set_variable_bounds(v, get_variable_lower_bound(v), ub);
    }
    void set_variable_name(variable v, const std::string & name) {
        check(Highs.passColName(model, _native_id(v), name.c_str()));
    }

    scalar get_objective_coefficient(variable v) {
        scalar coef;
        index dummy_int;
        scalar dummy_dbl;
        const int native_id = _native_id(v);
        check(Highs.getColsByRange(model, native_id, native_id, &dummy_int,
                                   &coef, &dummy_dbl, &dummy_dbl, &dummy_int,
                                   nullptr, nullptr, nullptr));
        return coef;
    }
    scalar get_variable_lower_bound(variable v) {
        scalar lb;
        int dummy_int;
        double dummy_dbl;
        const int native_id = _native_id(v);
        check(Highs.getColsByRange(model, native_id, native_id, &dummy_int,
                                   &dummy_dbl, &lb, &dummy_dbl, &dummy_int,
                                   nullptr, nullptr, nullptr));
        return lb;
    }
    scalar get_variable_upper_bound(variable v) {
        scalar ub;
        int dummy_int;
        double dummy_dbl;
        const int native_id = _native_id(v);
        check(Highs.getColsByRange(model, native_id, native_id, &dummy_int,
                                   &dummy_dbl, &dummy_dbl, &ub, &dummy_int,
                                   nullptr, nullptr, nullptr));
        return ub;
    }
    std::string get_variable_name(variable v) {
        std::string name(kHighsMaximumStringLength, '\0');
        check(Highs.getColName(model, _native_id(v), name.data()));
        name.resize(std::strlen(name.data()));
        name.shrink_to_fit();
        return name;
    }

    constraint add_constraint(linear_constraint auto && lc) {
        const HighsInt constr_id = static_cast<HighsInt>(num_constraints());
        _reset_cache(_num_var_native_ids());
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
    void _register_constraint(LC && lc) {
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
    void _register_first_valued_constraint(const Key & key,
                                           const LastConstrLambda & lc_lambda) {
        _register_constraint(lc_lambda(key));
    }
    template <typename Key, typename OptConstrLambda, typename... Tail>
        requires detail::optional_type<
                     std::invoke_result_t<OptConstrLambda, Key>> &&
                 linear_constraint<detail::optional_type_value_t<
                     std::invoke_result_t<OptConstrLambda, Key>>>
    void _register_first_valued_constraint(
        const Key & key, const OptConstrLambda & opt_lc_lambda,
        const Tail &... tail) {
        if(const auto & opt_lc = opt_lc_lambda(key)) {
            _register_constraint(opt_lc.value());
            return;
        }
        _register_first_valued_constraint(key, tail...);
    }

public:
    template <std::ranges::range IR, typename... CL>
    auto add_constraints(IR && keys, CL... constraint_lambdas) {
        _reset_cache(_num_var_native_ids());
        tmp_begins.resize(0);
        tmp_lower_bounds.resize(0);
        tmp_upper_bounds.resize(0);
        const HighsInt offset = static_cast<HighsInt>(num_constraints());
        HighsInt constr_id = offset;
        for(auto && key : keys) {
            _register_first_valued_constraint(key, constraint_lambdas...);
            ++constr_id;
        }
        check(Highs.addRows(model, static_cast<HighsInt>(tmp_begins.size()),
                            tmp_lower_bounds.data(), tmp_upper_bounds.data(),
                            static_cast<HighsInt>(tmp_indices.size()),
                            tmp_begins.data(), tmp_indices.data(),
                            tmp_scalars.data()));
        return constraints_range(
            std::forward<IR>(keys),
            std::views::transform(std::views::iota(offset, constr_id),
                                  [](auto && i) { return constraint{i}; }));
    }

private:
    std::pair<double, double> _row_bounds(const constraint & constr) {
        double lower, upper;
        int dummy_int;
        check(Highs.getRowsByRange(model, constr.id(), constr.id(), &dummy_int,
                                   &lower, &upper, &dummy_int, nullptr, nullptr,
                                   nullptr));
        return std::make_pair(lower, upper);
    }
    auto _row_lhs_bounds(const constraint & constr) {
        int dummy_int, num_nz;
        check(Highs.getRowsByRange(model, constr.id(), constr.id(), &dummy_int,
                                   nullptr, nullptr, &num_nz, nullptr, nullptr,
                                   nullptr));
        double lower, upper;
        auto indices = std::make_shared_for_overwrite<int[]>(
            static_cast<std::size_t>(num_nz));
        auto coefs = std::make_shared_for_overwrite<double[]>(
            static_cast<std::size_t>(num_nz));
        check(Highs.getRowsByRange(model, constr.id(), constr.id(), &dummy_int,
                                   &lower, &upper, &num_nz, &dummy_int,
                                   indices.get(), coefs.get()));
        return std::make_tuple(
            std::views::transform(std::views::iota(0, num_nz),
                                  [this, indices = std::move(indices),
                                   coefs = std::move(coefs)](int i) {
                                      return std::make_pair(
                                          _var_handle(indices.get()[i]),
                                          coefs.get()[i]);
                                  }),
            lower, upper);
    }
    double _bounds_to_rhs(const double & lower, const double & upper) {
        return (lower == -Highs.getInfinity(model)) ? upper : lower;
    }
    constraint_sense _bounds_to_constraint_sense(const double & lower,
                                                 const double & upper) {
        if(lower == upper) return constraint_sense::equal;
        if(lower == -Highs.getInfinity(model))
            return constraint_sense::less_equal;
        return constraint_sense::greater_equal;
    }

public:
    void set_constraint_rhs(constraint constr, double rhs) {
        auto [lower, upper] = _row_bounds(constr);
        switch(_bounds_to_constraint_sense(lower, upper)) {
            case constraint_sense::equal:
                lower = upper = rhs;
                break;
            case constraint_sense::less_equal:
                upper = rhs;
                break;
            case constraint_sense::greater_equal:
                lower = rhs;
                break;
        }
        check(Highs.changeRowBounds(model, constr.id(), lower, upper));
    }
    void set_constraint_sense(constraint constr, constraint_sense new_sense) {
        auto [lower, upper] = _row_bounds(constr);
        constraint_sense old_sense = _bounds_to_constraint_sense(lower, upper);
        if(old_sense == new_sense) return;
        const double rhs = _bounds_to_rhs(lower, upper);
        switch(new_sense) {
            case constraint_sense::equal:
                lower = upper = rhs;
                break;
            case constraint_sense::less_equal:
                lower = -Highs.getInfinity(model);
                upper = rhs;
                break;
            case constraint_sense::greater_equal:
                lower = rhs;
                upper = Highs.getInfinity(model);
                break;
        }
        check(Highs.changeRowBounds(model, constr.id(), lower, upper));
    }
    void set_constraint_name(constraint constr, std::string name) {
        check(Highs.passRowName(model, constr.id(), name.c_str()));
    }

    auto get_constraint_lhs(constraint constr) {
        auto [lhs, lower, upper] = _row_lhs_bounds(constr);
        return lhs;
    }
    double get_constraint_rhs(constraint constr) {
        auto [lower, upper] = _row_bounds(constr);
        return _bounds_to_rhs(lower, upper);
    }
    constraint_sense get_constraint_sense(constraint constr) {
        auto [lower, upper] = _row_bounds(constr);
        return _bounds_to_constraint_sense(lower, upper);
    }
    auto get_constraint(constraint constr) {
        auto [lhs, lower, upper] = _row_lhs_bounds(constr);
        return linear_constraint_view(
            linear_expression_view(std::move(lhs),
                                   -_bounds_to_rhs(lower, upper)),
            _bounds_to_constraint_sense(lower, upper));
    }
    auto get_constraint_name(constraint constr) {
        char name[kHighsMaximumStringLength];
        check(Highs.getRowName(model, constr.id(), name));
        return std::string(name);
    }

    ///////////////////////////////// Limits //////////////////////////////////
    void set_time_limit(std::chrono::duration<double> t) {
        check(Highs.setDoubleOptionValue(model, "time_limit", t.count()));
    }
    auto get_time_limit() {
        double t;
        check(Highs.getDoubleOptionValue(model, "time_limit", &t));
        return std::chrono::duration<double>(t);
    }

protected:
    static void check_model_status(int status) {
        if(status >= 7 && status <= 10) return;
        if(status == kHighsModelStatusNotset)
            throw std::runtime_error("highs_lp: kHighsModelStatusNotset.");
        if(status == kHighsModelStatusLoadError)
            throw std::runtime_error("highs_lp: kHighsModelStatusLoadError.");
        if(status == kHighsModelStatusModelError)
            throw std::runtime_error("highs_lp: kHighsModelStatusModelError.");
        if(status == kHighsModelStatusPresolveError)
            throw std::runtime_error(
                "highs_lp: kHighsModelStatusPresolveError.");
        if(status == kHighsModelStatusSolveError)
            throw std::runtime_error("highs_lp: kHighsModelStatusSolveError.");
        if(status == kHighsModelStatusPostsolveError)
            throw std::runtime_error(
                "highs_lp: kHighsModelStatusPostsolveError.");
        if(status == kHighsModelStatusModelEmpty)
            throw std::runtime_error("highs_lp: kHighsModelStatusModelEmpty.");
        if(status == kHighsModelStatusObjectiveBound)
            throw std::runtime_error(
                "highs_lp: kHighsModelStatusObjectiveBound.");
        if(status == kHighsModelStatusObjectiveTarget)
            throw std::runtime_error(
                "highs_lp: kHighsModelStatusObjectiveTarget.");
        if(status == kHighsModelStatusTimeLimit)
            throw std::runtime_error("highs_lp: kHighsModelStatusTimeLimit.");
        if(status == kHighsModelStatusIterationLimit)
            throw std::runtime_error(
                "highs_lp: kHighsModelStatusIterationLimit.");
        if(status == kHighsModelStatusUnknown)
            throw std::runtime_error("highs_lp: kHighsModelStatusUnknown.");
        if(status == kHighsModelStatusSolutionLimit)
            throw std::runtime_error(
                "highs_lp: kHighsModelStatusSolutionLimit.");
        if(status == kHighsModelStatusInterrupt)
            throw std::runtime_error("highs_lp: kHighsModelStatusInterrupt.");
    }
};

}  // namespace highs::v1_10
}  // namespace mippp
