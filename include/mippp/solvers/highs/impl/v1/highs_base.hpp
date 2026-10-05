#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstring>
#include <limits>
#include <memory>
#include <numeric>
#include <ranges>
#include <stdexcept>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "mippp/detail/handle_status_table.hpp"
#include "mippp/detail/iis_arithmetic.hpp"
#include "mippp/detail/invoke_key.hpp"
#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"
#include "mippp/utility/iis_outcome.hpp"
#include "mippp/utility/iis_snapshot.hpp"
#include "mippp/utility/solver_exceptions.hpp"
#include "mippp/utility/solver_version.hpp"

#include "mippp/solvers/highs/impl/v1/highs_api.hpp"
#include "mippp/solvers/remapping_model_base.hpp"

namespace mippp {
namespace highs::impl::v1 {

class highs_base : protected remapping_model_base<int, double> {
protected:
    const highs_api * Highs;
    void * model;

    void check(const int status) { Highs->_check(status); }

    std::vector<index> tmp_begins;
    std::vector<scalar> tmp_lower_bounds;
    std::vector<scalar> tmp_upper_bounds;

public:
    // the anchor model_variable_params_t deduces from
    using remapping_model_base<int, double>::default_variable_params;
    std::vector<variable> variables() {
        return _live_variables(num_variables());
    }
    using model_base<int, double>::constraints;
    double infinity() const noexcept { return Highs->getInfinity(model); }
    using remapping_model_base<int, double>::is_infinite;

    [[nodiscard]] explicit highs_base(const highs_api & api)
        : remapping_model_base<int, double>()
        , Highs(&api)
        , model(Highs->create()) {
        check(Highs->setBoolOptionValue(model, "output_flag", false));
    }
    ~highs_base() {
        if(model) Highs->destroy(model);
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

public:
    std::size_t num_variables() {
        return _num_var_native_ids() - _var_handles_to_delete.size();
    }
    std::size_t num_constraints() {
        return static_cast<std::size_t>(Highs->getNumRow(model));
    }
    std::size_t num_nonzeros() {
        return static_cast<std::size_t>(Highs->getNumNz(model));
    }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////// Native handles /////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
public:
    const highs_api & native_api() const noexcept { return *Highs; }
    void * native_model() const noexcept { return model; }
    int native_id(variable v) const noexcept { return _native_id(v); }
    int native_id(constraint c) const noexcept { return c.id(); }

public:
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// Objective ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void set_maximization() {
        check(Highs->changeObjectiveSense(model, kHighsObjSenseMaximize));
    }
    void set_minimization() {
        check(Highs->changeObjectiveSense(model, kHighsObjSenseMinimize));
    }

    void set_objective_offset(scalar offset) {
        check(Highs->changeObjectiveOffset(model, offset));
    }
    template <linear_expression LE>
    void set_objective(LE && le) {
        const auto num_vars = _num_var_native_ids();
        tmp_scalars.resize(num_vars);
        std::fill(tmp_scalars.begin(), tmp_scalars.end(), 0.0);
        for(auto && [var, coef] : le.linear_terms()) {
            tmp_scalars[static_cast<std::size_t>(_native_id(var))] += coef;
        }
        check(Highs->changeColsCostByRange(
            model, 0, static_cast<HighsInt>(num_vars) - 1, tmp_scalars.data()));
        set_objective_offset(le.constant());
    }
    template <linear_expression LE>
    void set_objective(distinct_variables_t, LE && le) {
        set_objective(std::forward<LE>(le));
    }

private:
    template <bool distinct, linear_expression LE>
    void _add_to_objective(LE && le) {
        if constexpr(!distinct) _prepare_coalescing(_num_var_native_ids());
        _reset_cache();
        _register_variables_entries<distinct>(le.linear_terms());
        const std::size_t num_nonzeros = tmp_indices.size();
        std::ranges::sort(std::views::zip(tmp_indices, tmp_scalars),
                          [](const auto & e1, const auto & e2) {
                              return std::get<0>(e1) < std::get<0>(e2);
                          });
        tmp_scalars.resize(2 * num_nonzeros);
        int dummy_int;
        check(Highs->getColsBySet(
            model, static_cast<HighsInt>(num_nonzeros), tmp_indices.data(),
            &dummy_int, tmp_scalars.data() + num_nonzeros, nullptr, nullptr,
            &dummy_int, nullptr, nullptr, nullptr));

        for(std::size_t i = 0; i < num_nonzeros; ++i) {
            tmp_scalars[i] += tmp_scalars[num_nonzeros + i];
        }
        check(Highs->changeColsCostBySet(
            model, static_cast<HighsInt>(num_nonzeros), tmp_indices.data(),
            tmp_scalars.data()));
        set_objective_offset(get_objective_offset() + le.constant());
    }

public:
    template <linear_expression LE>
    void add_to_objective(LE && le) {
        _add_to_objective<false>(std::forward<LE>(le));
    }
    template <linear_expression LE>
    void add_to_objective(distinct_variables_t, LE && le) {
        _add_to_objective<true>(std::forward<LE>(le));
    }

    scalar get_objective_offset() {
        scalar offset;
        check(Highs->getObjectiveOffset(model, &offset));
        return offset;
    }
    auto get_objective() {
        const auto num_vars = _num_var_native_ids();
        auto coefs = std::make_shared_for_overwrite<double[]>(num_vars);
        int dummy_int;
        check(Highs->getColsByRange(model, 0,
                                    static_cast<HighsInt>(num_vars) - 1,
                                    &dummy_int, coefs.get(), nullptr, nullptr,
                                    &dummy_int, nullptr, nullptr, nullptr));
        return linear_expression_view(
            std::views::transform(
                std::views::iota(index{0}, static_cast<index>(num_vars)),
                [this, coefs = std::move(coefs)](auto i) {
                    return std::make_pair(_var_handle(i), coefs[i]);
                }),
            get_objective_offset());
    }
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// Variables ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
protected:
    std::size_t _num_var_native_ids() {
        return static_cast<std::size_t>(Highs->getNumCol(model));
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
        check(Highs->deleteColsBySet(
            model, static_cast<int>(tmp_indices.size()), tmp_indices.data()));

        const std::size_t new_num_native_ids =
            old_num_native_ids - tmp_indices.size();
        // Deleting only the tail of the native ids leaves every surviving id
        // in place: no remap table is needed and _remap_ids stays false.
        if(_remap_ids ||
           static_cast<std::size_t>(tmp_indices.front()) < new_num_native_ids) {
            if(!_remap_ids) {
                _native_ids_map.resize(old_num_native_ids);
                _handle_ids_map.resize(old_num_native_ids);
                std::iota(_native_ids_map.begin(), _native_ids_map.end(), 0);
                std::iota(_handle_ids_map.begin(), _handle_ids_map.end(), 0);
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

protected:
    std::size_t _add_variables(std::size_t count,
                               const variable_params & params, int type) {
        if(_remap_ids) _extend_handle_ids_map(count);
        const std::size_t offset = _num_var_native_ids();
        const std::size_t handle_ids_begin =
            _new_var_handle_range(offset, count);

        const auto diff_count = static_cast<std::ptrdiff_t>(count);
        // not resize(3 * count) then fill: GCC 14 -O3 then reports a null
        // dereference inside vector::resize, a false positive
        tmp_scalars.reserve(3 * count);
        tmp_scalars.assign(count, params.obj_coef);
        tmp_scalars.insert(
            tmp_scalars.end(), count,
            params.lower_bound.value_or(-Highs->getInfinity(model)));
        tmp_scalars.insert(
            tmp_scalars.end(), count,
            params.upper_bound.value_or(Highs->getInfinity(model)));
        check(Highs->addCols(
            model, static_cast<HighsInt>(count), tmp_scalars.data(),
            tmp_scalars.data() + diff_count,
            tmp_scalars.data() + 2 * diff_count, 0, nullptr, nullptr, nullptr));
        if(type != kHighsVarTypeContinuous) {
            tmp_indices.resize(count);
            std::fill(tmp_indices.begin(), tmp_indices.end(), type);
            check(Highs->changeColsIntegralityByRange(
                model, static_cast<HighsInt>(offset),
                static_cast<HighsInt>(offset + count - 1), tmp_indices.data()));
        }
        return handle_ids_begin;
    }

public:
    friend model_base<int, double>;
    using model_base<int, double>::add_variable;
    using model_base<int, double>::add_variables;
    using model_base<int, double>::add_named_variable;
    using model_base<int, double>::add_named_variables;

private:
    variable _add_variable(const variable_params & params, int type) {
        HighsInt var_id = _new_var_native_id();
        check(Highs->addCol(
            model, params.obj_coef,
            params.lower_bound.value_or(-Highs->getInfinity(model)),
            params.upper_bound.value_or(Highs->getInfinity(model)), 0, nullptr,
            nullptr));
        if(type != kHighsVarTypeContinuous)
            check(Highs->changeColIntegrality(model, var_id, type));
        return _new_var_handle(var_id);
    }
    variable _new_variable(const variable_params & params, variable_kind kind) {
        return _add_variable(params, kind == variable_kind::continuous
                                         ? kHighsVarTypeContinuous
                                         : kHighsVarTypeInteger);
    }
    std::size_t _new_variables(std::size_t count,
                               const variable_params & params,
                               variable_kind kind) {
        return _add_variables(count, params,
                              kind == variable_kind::continuous
                                  ? kHighsVarTypeContinuous
                                  : kHighsVarTypeInteger);
    }

private:
    template <typename ER>
    inline variable _add_column(ER && entries, const variable_params & params) {
        const int var_id = _new_var_native_id();
        _reset_cache();
        _register_constraints_entries<true>(entries);
        check(Highs->addCol(
            model, params.obj_coef,
            params.lower_bound.value_or(-Highs->getInfinity(model)),
            params.upper_bound.value_or(Highs->getInfinity(model)),
            static_cast<HighsInt>(tmp_indices.size()), tmp_indices.data(),
            tmp_scalars.data()));
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
        check(Highs->changeColBounds(model, _native_id(v), lb, ub));
    }

public:
    void set_objective_coefficient(variable v, scalar c) {
        check(Highs->changeColCost(model, _native_id(v), c));
    }
    void set_variable_lower_bound(variable v, scalar lb) {
        _set_variable_bounds(v, lb, get_variable_upper_bound(v));
    }
    void set_variable_upper_bound(variable v, scalar ub) {
        _set_variable_bounds(v, get_variable_lower_bound(v), ub);
    }
    void set_variable_name(variable v, const std::string & name) {
        check(Highs->passColName(model, _native_id(v), name.c_str()));
    }

    scalar get_objective_coefficient(variable v) {
        scalar coef;
        index dummy_int;
        scalar dummy_dbl;
        const int native_id = _native_id(v);
        check(Highs->getColsByRange(model, native_id, native_id, &dummy_int,
                                    &coef, &dummy_dbl, &dummy_dbl, &dummy_int,
                                    nullptr, nullptr, nullptr));
        return coef;
    }
    scalar get_variable_lower_bound(variable v) {
        scalar lb;
        int dummy_int;
        double dummy_dbl;
        const int native_id = _native_id(v);
        check(Highs->getColsByRange(model, native_id, native_id, &dummy_int,
                                    &dummy_dbl, &lb, &dummy_dbl, &dummy_int,
                                    nullptr, nullptr, nullptr));
        return lb;
    }
    scalar get_variable_upper_bound(variable v) {
        scalar ub;
        int dummy_int;
        double dummy_dbl;
        const int native_id = _native_id(v);
        check(Highs->getColsByRange(model, native_id, native_id, &dummy_int,
                                    &dummy_dbl, &dummy_dbl, &ub, &dummy_int,
                                    nullptr, nullptr, nullptr));
        return ub;
    }
    std::string get_variable_name(variable v) {
        std::string name(kHighsMaximumStringLength, '\0');
        check(Highs->getColName(model, _native_id(v), name.data()));
        name.resize(std::strlen(name.data()));
        name.shrink_to_fit();
        return name;
    }
    ///////////////////////////////////////////////////////////////////////////
    /////////////////////////////// Constraints ///////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    template <bool distinct, linear_constraint LC>
    constraint _add_constraint(LC && lc) {
        const HighsInt constr_id = static_cast<HighsInt>(num_constraints());
        if constexpr(!distinct) _prepare_coalescing(_num_var_native_ids());
        _reset_cache();
        _register_variables_entries<distinct>(lc.linear_terms());
        const scalar b = lc.rhs();
        check(Highs->addRow(model,
                            (lc.sense() == constraint_sense::less_equal)
                                ? -Highs->getInfinity(model)
                                : b,
                            (lc.sense() == constraint_sense::greater_equal)
                                ? Highs->getInfinity(model)
                                : b,
                            static_cast<HighsInt>(tmp_indices.size()),
                            tmp_indices.data(), tmp_scalars.data()));
        return constraint(constr_id);
    }

public:
    template <linear_constraint LC>
    constraint add_constraint(LC && lc) {
        return _add_constraint<false>(std::forward<LC>(lc));
    }
    template <linear_constraint LC>
    constraint add_constraint(distinct_variables_t, LC && lc) {
        return _add_constraint<true>(std::forward<LC>(lc));
    }

private:
    template <bool distinct, linear_constraint LC>
    void _register_constraint(LC && lc) {
        tmp_begins.emplace_back(static_cast<HighsInt>(tmp_indices.size()));
        const scalar b = lc.rhs();
        tmp_lower_bounds.emplace_back(
            (lc.sense() == constraint_sense::less_equal)
                ? -Highs->getInfinity(model)
                : b);
        tmp_upper_bounds.emplace_back(
            (lc.sense() == constraint_sense::greater_equal)
                ? Highs->getInfinity(model)
                : b);
        _register_variables_entries<distinct>(lc.linear_terms());
    }
    template <bool distinct, typename Key, typename LastConstrLambda>
        requires linear_constraint<
            detail::key_invoke_result_t<LastConstrLambda &, const Key &>>
    void _register_first_valued_constraint(const Key & key,
                                           LastConstrLambda & lc_lambda) {
        _register_constraint<distinct>(detail::invoke_key(lc_lambda, key));
    }
    template <bool distinct, typename Key, typename OptConstrLambda,
              typename... Tail>
        requires detail::optional_type<detail::key_invoke_result_t<
                     OptConstrLambda &, const Key &>> &&
                 linear_constraint<
                     detail::optional_type_value_t<detail::key_invoke_result_t<
                         OptConstrLambda &, const Key &>>>
    void _register_first_valued_constraint(const Key & key,
                                           OptConstrLambda & opt_lc_lambda,
                                           Tail &... tail) {
        if(const auto & opt_lc = detail::invoke_key(opt_lc_lambda, key)) {
            _register_constraint<distinct>(opt_lc.value());
            return;
        }
        _register_first_valued_constraint<distinct>(key, tail...);
    }
    template <bool distinct, std::ranges::range IR, typename... CL>
    auto _add_constraints(IR && keys, CL &... constraint_lambdas) {
        if constexpr(!distinct) _prepare_coalescing(_num_var_native_ids());
        _reset_cache();
        tmp_begins.resize(0);
        tmp_lower_bounds.resize(0);
        tmp_upper_bounds.resize(0);
        const HighsInt offset = static_cast<HighsInt>(num_constraints());
        HighsInt constr_id = offset;
        for(auto && key : keys) {
            _register_first_valued_constraint<distinct>(key,
                                                        constraint_lambdas...);
            ++constr_id;
        }
        check(Highs->addRows(model, static_cast<HighsInt>(tmp_begins.size()),
                             tmp_lower_bounds.data(), tmp_upper_bounds.data(),
                             static_cast<HighsInt>(tmp_indices.size()),
                             tmp_begins.data(), tmp_indices.data(),
                             tmp_scalars.data()));
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

private:
    std::pair<double, double> _row_bounds(const constraint & constr) {
        double lower, upper;
        int dummy_int;
        check(Highs->getRowsByRange(model, constr.id(), constr.id(), &dummy_int,
                                    &lower, &upper, &dummy_int, nullptr,
                                    nullptr, nullptr));
        return std::make_pair(lower, upper);
    }
    auto _row_lhs_bounds(const constraint & constr) {
        int dummy_int, num_nz;
        check(Highs->getRowsByRange(model, constr.id(), constr.id(), &dummy_int,
                                    nullptr, nullptr, &num_nz, nullptr, nullptr,
                                    nullptr));
        double lower, upper;
        auto indices = std::make_shared_for_overwrite<int[]>(
            static_cast<std::size_t>(num_nz));
        auto coefs = std::make_shared_for_overwrite<double[]>(
            static_cast<std::size_t>(num_nz));
        check(Highs->getRowsByRange(model, constr.id(), constr.id(), &dummy_int,
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
    constraint_sense _bounds_to_constraint_sense(const double & lower,
                                                 const double & upper) {
        if(lower == upper) return constraint_sense::equal;
        if(lower == -Highs->getInfinity(model))
            return constraint_sense::less_equal;
        if(upper == Highs->getInfinity(model))
            return constraint_sense::greater_equal;
        throw std::runtime_error(
            "Tried to get the sense of a ranged constraint");
    }
    double _bounds_to_rhs(const double & lower, const double & upper) {
        return _bounds_to_constraint_sense(lower, upper) ==
                       constraint_sense::greater_equal
                   ? lower
                   : upper;
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
        check(Highs->changeRowBounds(model, constr.id(), lower, upper));
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
                lower = -Highs->getInfinity(model);
                upper = rhs;
                break;
            case constraint_sense::greater_equal:
                lower = rhs;
                upper = Highs->getInfinity(model);
                break;
        }
        check(Highs->changeRowBounds(model, constr.id(), lower, upper));
    }
    void set_constraint_lower_bound(constraint constr, scalar lb) {
        check(Highs->changeRowBounds(model, constr.id(), lb,
                                     _row_bounds(constr).second));
    }
    void set_constraint_upper_bound(constraint constr, scalar ub) {
        check(Highs->changeRowBounds(model, constr.id(),
                                     _row_bounds(constr).first, ub));
    }
    void set_constraint_name(constraint constr, std::string name) {
        check(Highs->passRowName(model, constr.id(), name.c_str()));
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
    double get_constraint_lower_bound(constraint constr) {
        return _row_bounds(constr).first;
    }
    double get_constraint_upper_bound(constraint constr) {
        return _row_bounds(constr).second;
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
        check(Highs->getRowName(model, constr.id(), name));
        return std::string(name);
    }

    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////// Limits //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void set_time_limit(std::chrono::duration<double> t) {
        check(Highs->setDoubleOptionValue(model, "time_limit", t.count()));
    }
    auto get_time_limit() {
        double t;
        check(Highs->getDoubleOptionValue(model, "time_limit", &t));
        return std::chrono::duration<double>(t);
    }

protected:
    // Highs_run does not reset the clock its time limit is read on, so the
    // limit would otherwise bound the model's cumulative solve time: a
    // modified model that already spent longer than the limit in HiGHS
    // stops before its first iteration.
    void _run() {
        check(Highs->zeroAllClocks(model));
        check(Highs->run(model));
    }

public:
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// Verbosity ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void set_verbose(bool verbose) {
        check(Highs->setBoolOptionValue(model, "output_flag", verbose));
    }
    bool is_verbose() {
        HighsInt verbose;
        check(Highs->getBoolOptionValue(model, "output_flag", &verbose));
        return verbose != 0;
    }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////////// IIS ////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    // Spelled here rather than taken from the HiGHS header: the header a
    // user includes may be any release, and the status numbering changed at
    // 1.13, so only the loaded library's regime counts, and the floor below
    // leaves one.
    static constexpr HighsInt _iis_bound_free = 1, _iis_bound_lower = 2,
                              _iis_bound_upper = 3, _iis_bound_boxed = 4;
    static constexpr HighsInt _iis_status_not_in_conflict = -1,
                              _iis_status_maybe_in_conflict = 0;
    // the default Light strategy finds trivial conflicts only
    static constexpr HighsInt _iis_strategy_full = 6;
    // the release from which the status numbering is stable and
    // iis_time_limit exists
    static constexpr solver_version _iis_native_floor{1, 14, 0};

    // Every option is written back before the first error is raised, so a
    // rejected write cannot leave the other one set; a constructor that
    // throws runs no destructor, so it writes back itself.
    class iis_option_guard {
    private:
        const highs_api & _api;
        void * _model;
        HighsInt _strategy;
        double _time_limit;
        bool _restored = false;

        int _write_back() const noexcept {
            const int strategy_status =
                _api.setIntOptionValue(_model, "iis_strategy", _strategy);
            const int time_limit_status = _api.setDoubleOptionValue(
                _model, "iis_time_limit", _time_limit);
            return strategy_status == kHighsStatusError ? strategy_status
                                                        : time_limit_status;
        }

    public:
        iis_option_guard(const highs_api & api, void * model, HighsInt strategy,
                         double time_limit)
            : _api(api), _model(model) {
            _api._check(
                _api.getIntOptionValue(_model, "iis_strategy", &_strategy));
            _api._check(_api.getDoubleOptionValue(_model, "iis_time_limit",
                                                  &_time_limit));
            try {
                _api._check(
                    _api.setIntOptionValue(_model, "iis_strategy", strategy));
                _api._check(_api.setDoubleOptionValue(_model, "iis_time_limit",
                                                      time_limit));
            } catch(...) {
                (void)_write_back();
                throw;
            }
        }
        iis_option_guard(const iis_option_guard &) = delete;
        iis_option_guard & operator=(const iis_option_guard &) = delete;

        void restore() {
            _restored = true;
            _api._check(_write_back());
        }
        // values read back moments ago: the writes cannot be rejected
        ~iis_option_guard() {
            if(!_restored) (void)_write_back();
        }
    };

    // HiGHS answers a row whose sides cross as boxed before it reads the
    // row's terms. Without terms the activity is 0, which violates one of two
    // crossed sides, and that side alone is the IIS.
    HighsInt _iis_row_bound(HighsInt row, HighsInt bound) {
        if(bound != _iis_bound_boxed) return bound;
        const auto [lower, upper] = _row_bounds(constraint(row));
        if(!(lower > upper)) return bound;
        int dummy_int, num_nz;
        check(Highs->getRowsByRange(model, row, row, &dummy_int, nullptr,
                                    nullptr, &num_nz, nullptr, nullptr,
                                    nullptr));
        if(num_nz != 0) return bound;
        const auto violated_lower =
            detail::iis_side_violated_by_zero(lower, upper);
        if(!violated_lower) return bound;
        return *violated_lower ? _iis_bound_lower : _iis_bound_upper;
    }

protected:
    using iis_outcome_type =
        std::variant<iis_outcome::incomplete, iis_outcome::irreducible,
                     iis_outcome::feasible, iis_outcome::time_limit>;
    using iis_snapshot_type =
        iis_snapshot<variable, constraint, iis_sided_status, iis_sided_status,
                     iis_outcome_type>;

    iis_snapshot_type _compute_iis() {
        const auto loaded = Highs->library_version();
        if(loaded && *loaded < _iis_native_floor)
            throw solver_error(
                detail::concat_str("mippp: compute_iis() needs HiGHS ",
                                   to_string(_iis_native_floor),
                                   " or later, the loaded library '",
                                   Highs->library_path().string(), "' reports ",
                                   to_string(*loaded))
                    .c_str());
        if(Highs->getIis == nullptr)
            throw solver_error(
                detail::concat_str(
                    "mippp: compute_iis() needs Highs_getIis, which the "
                    "loaded library '",
                    Highs->library_path().string(), "' (reporting ",
                    loaded ? to_string(*loaded) : std::string("no version"),
                    ") does not export")
                    .c_str());

        const std::size_t num_col = _num_var_native_ids();
        const std::size_t num_row = num_constraints();
        detail::handle_status_table<iis_sided_status> variable_table(
            _handle_id_bound(num_col));
        detail::handle_status_table<iis_sided_status> constraint_table(num_row);

        // HiGHS ignores time_limit during the search and reads iis_time_limit
        // instead: the model's limit is copied there for this call only
        const double budget = get_time_limit().count();
        iis_option_guard guard(*Highs, model, _iis_strategy_full, budget);

        // HiGHS stores the matrix row-wise once added rows bring more
        // nonzeros than it holds, and its check of an answer made of one row
        // and no column then reads past the end of an array it built for a
        // column-wise matrix. Deleting the empty column range turns the
        // matrix column-wise and drops the presolve and ray records, which
        // the search does not read, but leaves the data, the Hessian, the
        // basis and the model status alone. An all-zero mask would rewrite
        // them in place and fail an assertion of HiGHS on a QP's Hessian.
        check(Highs->deleteColsByRange(model, 0, -1));

        // HiGHS overwrites every entry on the returns decoded below, so the
        // fill is only a default; NotInConflict rather than zero because zero
        // is the maybe code.
        HighsInt iis_num_col = 0, iis_num_row = 0;
        std::vector<HighsInt> col_index(num_col), row_index(num_row),
            col_bound(num_col), row_bound(num_row),
            col_status(num_col, _iis_status_not_in_conflict),
            row_status(num_row, _iis_status_not_in_conflict);

        // The C API exposes no IIS status: the time measured around the call
        // is the only signal that tells a limit stop from a failure.
        const auto start = std::chrono::steady_clock::now();
        const int code =
            Highs->getIis(model, &iis_num_col, &iis_num_row, col_index.data(),
                          row_index.data(), col_bound.data(), row_bound.data(),
                          col_status.data(), row_status.data());
        const double elapsed = std::chrono::duration<double>(
                                   std::chrono::steady_clock::now() - start)
                                   .count();
        // never reached under an infinite budget, always under a zero one
        const bool out_of_time = elapsed >= budget;
        const auto short_of = [&](bool conflict) -> iis_outcome_type {
            if(out_of_time) return iis_outcome::time_limit(conflict);
            return iis_outcome::incomplete(conflict);
        };
        guard.restore();

        if(code == kHighsStatusError) {
            if(out_of_time)
                return iis_snapshot_type(std::move(variable_table),
                                         std::move(constraint_table),
                                         iis_outcome::time_limit{});
            throw solver_error(
                detail::concat_str(
                    "mippp: Highs_getIis failed with model status ",
                    std::to_string(Highs->getModelStatus(model)))
                    .c_str());
        }
        // A warning is either a stop after the elasticity filter, whose set
        // is the whole model flagged maybe, or a failed post-check, after
        // which HiGHS copies its own emptied status vectors into the arrays
        // (an out-of-bounds read on its side that no fill here can absorb):
        // neither is an answer, and the two cannot be told apart.
        if(code == kHighsStatusWarning)
            return iis_snapshot_type(std::move(variable_table),
                                     std::move(constraint_table),
                                     short_of(false));

        std::size_t num_members = 0;
        bool maybe = false;
        const auto decode = [&](HighsInt bound, HighsInt status, auto & table,
                                std::size_t id) {
            if(bound == _iis_bound_free) return;
            table.set(id, detail::iis_flagged_status<iis_sided_status>(
                              bound != _iis_bound_upper,
                              bound != _iis_bound_lower, false));
            ++num_members;
            maybe |= (status == _iis_status_maybe_in_conflict);
        };
        for(std::size_t k = 0; k < static_cast<std::size_t>(iis_num_col); ++k) {
            const HighsInt col = col_index[k];
            decode(col_bound[k], col_status[static_cast<std::size_t>(col)],
                   variable_table, _var_handle(col).uid());
        }
        for(std::size_t k = 0; k < static_cast<std::size_t>(iis_num_row); ++k) {
            const auto row = static_cast<std::size_t>(row_index[k]);
            decode(_iis_row_bound(row_index[k], row_bound[k]), row_status[row],
                   constraint_table, row);
        }

        // Members, not listed entries: a listing made only of free-bound
        // columns would otherwise claim that the background is infeasible.
        if(num_members == 0) {
            const int model_status = Highs->getModelStatus(model);
            const bool solved = model_status == kHighsModelStatusOptimal ||
                                model_status == kHighsModelStatusUnbounded;
            return iis_snapshot_type(
                std::move(variable_table), std::move(constraint_table),
                solved ? iis_outcome_type(iis_outcome::feasible{})
                       : short_of(false));
        }
        if(!maybe)
            return iis_snapshot_type(std::move(variable_table),
                                     std::move(constraint_table),
                                     iis_outcome::irreducible{});
        return iis_snapshot_type(std::move(variable_table),
                                 std::move(constraint_table), short_of(true));
    }
};

}  // namespace highs::impl::v1
}  // namespace mippp
