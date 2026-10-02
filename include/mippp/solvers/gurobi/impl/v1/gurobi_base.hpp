#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <memory>
#include <numeric>
#include <optional>
#include <ranges>
#include <ratio>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "mippp/detail/handle_status_table.hpp"
#include "mippp/detail/invoke_key.hpp"
#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"
#include "mippp/utility/iis_outcome.hpp"
#include "mippp/utility/iis_snapshot.hpp"

#include "mippp/solvers/gurobi/impl/v1/gurobi_api.hpp"
#include "mippp/solvers/remapping_model_base.hpp"

namespace mippp {
namespace gurobi::impl::v1 {

class gurobi_base : protected remapping_model_base<int, double> {
protected:
    const gurobi_api * GRB;
    GRBenv * env;
    GRBmodel * model;

    void check(const int error) { GRB->_check(env, error); }
    static constexpr char constraint_sense_to_gurobi_sense(
        constraint_sense rel) {
        if(rel == constraint_sense::less_equal) return GRB_LESS_EQUAL;
        if(rel == constraint_sense::equal) return GRB_EQUAL;
        return GRB_GREATER_EQUAL;
    }
    static constexpr constraint_sense gurobi_sense_to_constraint_sense(
        char sense) {
        if(sense == GRB_LESS_EQUAL) return constraint_sense::less_equal;
        if(sense == GRB_EQUAL) return constraint_sense::equal;
        return constraint_sense::greater_equal;
    }

    std::size_t _num_var_native_ids;
    std::size_t _lazy_num_constraints;

    std::vector<int> tmp_begins;
    std::vector<char> tmp_types;
    std::vector<double> tmp_rhs;

public:
    // the anchor model_variable_params_t deduces from
    using remapping_model_base<int, double>::default_variable_params;
    // From the counters, not num_variables() and num_constraints(): their
    // GRBupdatemodel would flush the queued changes at every listing.
    std::vector<variable> variables() {
        return _live_variables(_num_var_native_ids);
    }
    auto constraints() {
        return entity_range(constraint(0), _lazy_num_constraints);
    }
    double infinity() const noexcept { return GRB_INFINITY; }
    using remapping_model_base<int, double>::is_infinite;

    [[nodiscard]] explicit gurobi_base(const gurobi_api & api)
        : remapping_model_base<int, double>()
        , GRB(&api)
        , env(GRB->_empty_env())
        , model(nullptr)
        , _num_var_native_ids(0)
        , _lazy_num_constraints(0) {
        // a constructor that throws runs no destructor
        try {
            // before GRBstartenv, which otherwise prints the licence banner
            check(GRB->setintparam(env, GRB_INT_PAR_OUTPUTFLAG, 0));
            check(GRB->startenv(env));
            check(GRB->newmodel(env, &model, "GUROBI", 0, nullptr, nullptr,
                                nullptr, nullptr, nullptr));
        } catch(...) {
            GRB->freeenv(env);
            throw;
        }
        GRB->freeenv(env);
        env = GRB->getenv(model);
        if(env == nullptr) {
            static_cast<void>(GRB->freemodel(model));
            throw std::runtime_error(
                "gurobi_base: Could not retrieve model environment.");
        }
    }
    // check() would throw out of the destructor and terminate the program
    ~gurobi_base() {
        if(model) static_cast<void>(GRB->freemodel(model));
    }

    constexpr gurobi_base(const gurobi_base &) = delete;
    constexpr gurobi_base(gurobi_base && other) noexcept
        : remapping_model_base<int, double>(std::move(other))
        , GRB(other.GRB)
        , env(other.env)
        , model(other.model)
        , _num_var_native_ids(other._num_var_native_ids)
        , _lazy_num_constraints(other._lazy_num_constraints)
        , tmp_begins(std::move(other.tmp_begins))
        , tmp_types(std::move(other.tmp_types))
        , tmp_rhs(std::move(other.tmp_rhs)) {
        other.model = nullptr;
        other.env = nullptr;
    }

    constexpr gurobi_base & operator=(const gurobi_base &) = delete;
    constexpr gurobi_base & operator=(gurobi_base && other) = delete;

protected:
    void update_gurobi_model() { check(GRB->updatemodel(model)); }

    int _new_var_native_id() {
        if(_remap_ids) _extend_handle_ids_map(1);
        return static_cast<int>(_num_var_native_ids++);
    }

    void _lazily_remove_variables() {
        if(_var_handles_to_delete.empty()) return;
        update_gurobi_model();
        tmp_indices.resize(0);
        for(const variable & var : _var_handles_to_delete)
            tmp_indices.emplace_back(_native_id(var));
        std::ranges::sort(tmp_indices);
        check(GRB->delvars(model, static_cast<int>(tmp_indices.size()),
                           tmp_indices.data()));

        update_gurobi_model();

        const std::size_t new_num_native_ids =
            _num_var_native_ids - tmp_indices.size();
        // Deleting only the tail of the native ids leaves every surviving id
        // in place: no remap table is needed and _remap_ids stays false.
        if(_remap_ids ||
           static_cast<std::size_t>(tmp_indices.front()) < new_num_native_ids) {
            if(!_remap_ids) {
                _native_ids_map.resize(_num_var_native_ids);
                _handle_ids_map.resize(_num_var_native_ids);
                std::iota(_native_ids_map.begin(), _native_ids_map.end(), 0);
                std::iota(_handle_ids_map.begin(), _handle_ids_map.end(), 0);
                _remap_ids = true;
            }

            std::size_t offset = 0;
            for(int old_native_id :
                std::views::iota(0, static_cast<int>(_num_var_native_ids))) {
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
            _free_var_handles.insert(_free_var_handles.end(),
                                     _var_handles_to_delete.cbegin(),
                                     _var_handles_to_delete.cend());
        }
        _num_var_native_ids = new_num_native_ids;
        _var_handles_to_delete.clear();
    }

public:
    std::size_t num_variables() {
        int num;
        update_gurobi_model();
        check(GRB->getintattr(model, GRB_INT_ATTR_NUMVARS, &num));
        if(static_cast<std::size_t>(num) != _num_var_native_ids)
            throw std::runtime_error(
                "gurobi_base: _num_var_native_ids differs from gurobi "
                "one.");
        return _num_var_native_ids - _var_handles_to_delete.size();
    }
    std::size_t num_constraints() {
        int num;
        update_gurobi_model();
        check(GRB->getintattr(model, GRB_INT_ATTR_NUMCONSTRS, &num));
        if(static_cast<std::size_t>(num) != _lazy_num_constraints)
            throw std::runtime_error(
                "gurobi_base: _lazy_num_constraints differs from "
                "gurobi "
                "one.");
        return _lazy_num_constraints;
    }
    std::size_t num_nonzeros() {
        int num;
        update_gurobi_model();
        check(GRB->getintattr(model, GRB_INT_ATTR_NUMNZS, &num));
        return static_cast<std::size_t>(num);
    }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////// Native handles /////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
public:
    const gurobi_api & native_api() const noexcept { return *GRB; }
    std::pair<GRBenv *, GRBmodel *> native_model() const noexcept {
        return {env, model};
    }
    int native_id(variable v) const noexcept { return _native_id(v); }
    int native_id(constraint c) const noexcept { return c.id(); }

public:
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// Objective ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void set_maximization() {
        check(GRB->setintattr(model, GRB_INT_ATTR_MODELSENSE, GRB_MAXIMIZE));
    }
    void set_minimization() {
        check(GRB->setintattr(model, GRB_INT_ATTR_MODELSENSE, GRB_MINIMIZE));
    }

    void set_objective_offset(double constant) {
        check(GRB->setdblattr(model, GRB_DBL_ATTR_OBJCON, constant));
    }
    void set_objective(linear_expression auto && le) {
        tmp_scalars.resize(_num_var_native_ids);
        std::fill(tmp_scalars.begin(), tmp_scalars.end(), 0.0);
        for(auto && [var, coef] : le.linear_terms()) {
            tmp_scalars[static_cast<std::size_t>(_native_id(var))] += coef;
        }
        check(GRB->setdblattrarray(model, GRB_DBL_ATTR_OBJ, 0,
                                   static_cast<int>(_num_var_native_ids),
                                   tmp_scalars.data()));
        set_objective_offset(le.constant());
    }
    template <linear_expression LE>
    void set_objective(distinct_variables_t, LE && le) {
        set_objective(std::forward<LE>(le));
    }

private:
    template <bool distinct, linear_expression LE>
    void _add_to_objective(LE && le) {
        if constexpr(!distinct) _prepare_coalescing(_num_var_native_ids);
        _reset_cache();
        _register_variables_entries<distinct>(le.linear_terms());
        for(auto && [native_id, coef] :
            std::views::zip(tmp_indices, tmp_scalars)) {
            coef += get_objective_coefficient(_var_handle(native_id));
        }
        check(GRB->setdblattrlist(model, GRB_DBL_ATTR_OBJ,
                                  static_cast<int>(tmp_indices.size()),
                                  tmp_indices.data(), tmp_scalars.data()));
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
    double get_objective_offset() {
        double constant;
        update_gurobi_model();
        check(GRB->getdblattr(model, GRB_DBL_ATTR_OBJCON, &constant));
        return constant;
    }
    auto get_objective() {
        auto coefs =
            std::make_shared_for_overwrite<double[]>(_num_var_native_ids);
        update_gurobi_model();
        check(GRB->getdblattrarray(model, GRB_DBL_ATTR_OBJ, 0,
                                   static_cast<int>(_num_var_native_ids),
                                   coefs.get()));
        return linear_expression_view(
            std::views::transform(
                std::views::iota(0, static_cast<int>(_num_var_native_ids)),
                [this, coefs = std::move(coefs)](auto && i) {
                    return std::make_pair(_var_handle(i), coefs[i]);
                }),
            get_objective_offset());
    }
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// Variables ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
protected:
    inline std::size_t _add_variables(const std::size_t & count,
                                      const variable_params & params,
                                      const char & type) {
        if(_remap_ids) _extend_handle_ids_map(count);
        const int new_native_ids_begin = static_cast<int>(_num_var_native_ids);
        const std::size_t handle_ids_begin =
            _new_var_handle_range(_num_var_native_ids, count);
        check(GRB->addvars(model, static_cast<int>(count), 0, nullptr, nullptr,
                           nullptr, nullptr, nullptr, nullptr, nullptr,
                           nullptr));
        if(double obj = params.obj_coef; obj != 0.0) {
            tmp_scalars.resize(count);
            std::fill(tmp_scalars.begin(), tmp_scalars.end(), obj);
            check(GRB->setdblattrarray(
                model, GRB_DBL_ATTR_OBJ, new_native_ids_begin,
                static_cast<int>(count), tmp_scalars.data()));
        }
        if(double lb = params.lower_bound.value_or(-GRB_INFINITY); lb != 0.0) {
            tmp_scalars.resize(count);
            std::fill(tmp_scalars.begin(), tmp_scalars.end(), lb);
            check(GRB->setdblattrarray(
                model, GRB_DBL_ATTR_LB, new_native_ids_begin,
                static_cast<int>(count), tmp_scalars.data()));
        }
        if(double ub = params.upper_bound.value_or(GRB_INFINITY);
           ub != GRB_INFINITY) {
            tmp_scalars.resize(count);
            std::fill(tmp_scalars.begin(), tmp_scalars.end(), ub);
            check(GRB->setdblattrarray(
                model, GRB_DBL_ATTR_UB, new_native_ids_begin,
                static_cast<int>(count), tmp_scalars.data()));
        }
        if(type != GRB_CONTINUOUS) {
            tmp_types.resize(count);
            std::fill(tmp_types.begin(), tmp_types.end(), type);
            check(GRB->setcharattrarray(
                model, GRB_CHAR_ATTR_VTYPE, new_native_ids_begin,
                static_cast<int>(count), tmp_types.data()));
        }
        _num_var_native_ids += count;
        return handle_ids_begin;
    }

public:
    friend model_base<int, double>;
    using model_base<int, double>::add_variable;
    using model_base<int, double>::add_variables;
    using model_base<int, double>::add_named_variable;
    using model_base<int, double>::add_named_variables;

private:
    inline variable _add_variable(const variable_params & params,
                                  const char & type, const char * name_str) {
        check(GRB->addvar(model, 0, nullptr, nullptr, params.obj_coef,
                          params.lower_bound.value_or(-GRB_INFINITY),
                          params.upper_bound.value_or(GRB_INFINITY), type,
                          name_str));
        return _new_var_handle(_new_var_native_id());
    }
    variable _new_variable(const variable_params & params, variable_kind kind) {
        return _add_variable(params,
                             kind == variable_kind::continuous ? GRB_CONTINUOUS
                             : kind == variable_kind::integer  ? GRB_INTEGER
                                                               : GRB_BINARY,
                             nullptr);
    }
    std::size_t _new_variables(std::size_t count,
                               const variable_params & params,
                               variable_kind kind) {
        return _add_variables(count, params,
                              kind == variable_kind::continuous ? GRB_CONTINUOUS
                              : kind == variable_kind::integer  ? GRB_INTEGER
                                                                : GRB_BINARY);
    }

private:
    template <typename ER>
    inline variable _add_column(ER && entries, const variable_params & params,
                                const char & type) {
        _reset_cache();
        _register_constraints_entries<true>(entries);
        check(GRB->addvar(
            model, static_cast<int>(tmp_indices.size()), tmp_indices.data(),
            tmp_scalars.data(), params.obj_coef,
            params.lower_bound.value_or(-GRB_INFINITY),
            params.upper_bound.value_or(GRB_INFINITY), type, nullptr));
        return _new_var_handle(_new_var_native_id());
    }

public:
    template <std::ranges::range ER>
    variable add_column(
        ER && entries, const variable_params params = default_variable_params) {
        return _add_column(std::forward<ER>(entries), params, GRB_CONTINUOUS);
    }
    variable add_column(
        std::initializer_list<std::pair<constraint, scalar>> entries,
        const variable_params params = default_variable_params) {
        return _add_column(entries, params, GRB_CONTINUOUS);
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

    void set_objective_coefficient(variable v, double c) {
        check(
            GRB->setdblattrelement(model, GRB_DBL_ATTR_OBJ, _native_id(v), c));
    }
    void set_variable_lower_bound(variable v, double lb) {
        check(
            GRB->setdblattrelement(model, GRB_DBL_ATTR_LB, _native_id(v), lb));
    }
    void set_variable_upper_bound(variable v, double ub) {
        check(
            GRB->setdblattrelement(model, GRB_DBL_ATTR_UB, _native_id(v), ub));
    }
    void set_variable_name(variable v, const char * name_ptr) {
        check(GRB->setstrattrelement(model, GRB_STR_ATTR_VARNAME, _native_id(v),
                                     name_ptr));
    }
    void set_variable_name(variable v, const std::string & name) {
        set_variable_name(v, name.c_str());
    }

    double get_objective_coefficient(variable v) {
        double coef;
        update_gurobi_model();
        check(GRB->getdblattrelement(model, GRB_DBL_ATTR_OBJ, _native_id(v),
                                     &coef));
        return coef;
    }
    double get_variable_lower_bound(variable v) {
        double lb;
        update_gurobi_model();
        check(
            GRB->getdblattrelement(model, GRB_DBL_ATTR_LB, _native_id(v), &lb));
        return lb;
    }
    double get_variable_upper_bound(variable v) {
        double ub;
        update_gurobi_model();
        check(
            GRB->getdblattrelement(model, GRB_DBL_ATTR_UB, _native_id(v), &ub));
        return ub;
    }
    std::string get_variable_name(variable v) {
        char * name;
        update_gurobi_model();
        check(GRB->getstrattrelement(model, GRB_STR_ATTR_VARNAME, _native_id(v),
                                     &name));
        return std::string(name);
    }
    ///////////////////////////////////////////////////////////////////////////
    /////////////////////////////// Constraints ///////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    template <bool distinct, linear_constraint LC>
    constraint _add_constraint(LC && lc) {
        const int constr_id = static_cast<int>(_lazy_num_constraints++);
        if constexpr(!distinct) _prepare_coalescing(_num_var_native_ids);
        _reset_cache();
        _register_variables_entries<distinct>(lc.linear_terms());
        check(GRB->addconstr(model, static_cast<int>(tmp_indices.size()),
                             tmp_indices.data(), tmp_scalars.data(),
                             constraint_sense_to_gurobi_sense(lc.sense()),
                             lc.rhs(), nullptr));
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
        ++register_count;
        tmp_begins.emplace_back(static_cast<int>(tmp_indices.size()));
        tmp_types.emplace_back(constraint_sense_to_gurobi_sense(lc.sense()));
        tmp_rhs.emplace_back(lc.rhs());
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
        if constexpr(!distinct) _prepare_coalescing(_num_var_native_ids);
        _reset_cache();
        tmp_begins.resize(0);
        tmp_types.resize(0);
        tmp_rhs.resize(0);
        const int offset = static_cast<int>(_lazy_num_constraints);
        int constr_id = offset;
        for(auto && key : keys) {
            _register_first_valued_constraint<distinct>(key,
                                                        constraint_lambdas...);
            ++constr_id;
        }
        check(GRB->addconstrs(
            model, constr_id - offset, static_cast<int>(tmp_indices.size()),
            tmp_begins.data(), tmp_indices.data(), tmp_scalars.data(),
            tmp_types.data(), tmp_rhs.data(), nullptr));
        _lazy_num_constraints += static_cast<std::size_t>(constr_id - offset);
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

    void set_constraint_rhs(constraint constr, double rhs) {
        check(
            GRB->setdblattrelement(model, GRB_DBL_ATTR_RHS, constr.id(), rhs));
    }
    void set_constraint_sense(constraint constr, constraint_sense r) {
        check(GRB->setcharattrelement(model, GRB_CHAR_ATTR_SENSE, constr.id(),
                                      constraint_sense_to_gurobi_sense(r)));
    }
    auto get_constraint_lhs(constraint constr) {
        int num_nz, beg;
        update_gurobi_model();
        check(GRB->getconstrs(model, &num_nz, nullptr, nullptr, nullptr,
                              constr.id(), 1));
        auto indices = std::make_shared_for_overwrite<int[]>(
            static_cast<std::size_t>(num_nz));
        auto coefs = std::make_shared_for_overwrite<double[]>(
            static_cast<std::size_t>(num_nz));
        check(GRB->getconstrs(model, &num_nz, &beg, indices.get(), coefs.get(),
                              constr.id(), 1));
        return std::views::transform(
            std::views::iota(0, num_nz), [this, indices = std::move(indices),
                                          coefs = std::move(coefs)](int i) {
                return std::make_pair(_var_handle(indices.get()[i]),
                                      coefs.get()[i]);
            });
    }
    double get_constraint_rhs(constraint constr) {
        double rhs;
        update_gurobi_model();
        check(
            GRB->getdblattrelement(model, GRB_DBL_ATTR_RHS, constr.id(), &rhs));
        return rhs;
    }
    constraint_sense get_constraint_sense(constraint constr) {
        char sense;
        update_gurobi_model();
        check(GRB->getcharattrelement(model, GRB_CHAR_ATTR_SENSE, constr.id(),
                                      &sense));
        return gurobi_sense_to_constraint_sense(sense);
    }
    double get_constraint_lower_bound(constraint constr) {
        if(get_constraint_sense(constr) == constraint_sense::less_equal)
            return -infinity();
        return get_constraint_rhs(constr);
    }
    double get_constraint_upper_bound(constraint constr) {
        if(get_constraint_sense(constr) == constraint_sense::greater_equal)
            return infinity();
        return get_constraint_rhs(constr);
    }
    auto get_constraint(constraint constr) {
        return linear_constraint_view(
            linear_expression_view(get_constraint_lhs(constr),
                                   -get_constraint_rhs(constr)),
            get_constraint_sense(constr));
    }

    void set_constraint_name(constraint constr, const char * name_ptr) {
        check(GRB->setstrattrelement(model, GRB_STR_ATTR_CONSTRNAME,
                                     constr.id(), name_ptr));
    }
    void set_constraint_name(constraint constr, const std::string & name) {
        set_constraint_name(constr, name.c_str());
    }
    auto get_constraint_name(constraint constr) {
        char * name;
        update_gurobi_model();
        check(GRB->getstrattrelement(model, GRB_STR_ATTR_CONSTRNAME,
                                     constr.id(), &name));
        return std::string(name);
    }

    void set_feasibility_tolerance(double tol) {
        check(GRB->setdblparam(env, GRB_DBL_PAR_FEASIBILITYTOL, tol));
    }
    double get_feasibility_tolerance() {
        double tol;
        check(GRB->getdblparam(env, GRB_DBL_PAR_FEASIBILITYTOL, &tol));
        return tol;
    }

    ///////////////////////////////// Limits //////////////////////////////////
    void set_time_limit(std::chrono::duration<double> t) {
        check(GRB->setdblparam(env, GRB_DBL_PAR_TIMELIMIT, t.count()));
    }
    auto get_time_limit() {
        double t;
        check(GRB->getdblparam(env, GRB_DBL_PAR_TIMELIMIT, &t));
        return std::chrono::duration<double>(t);
    }

    void set_memory_limit(memory_size<double, std::giga> gb) {
        check(GRB->setdblparam(env, GRB_DBL_PAR_SOFTMEMLIMIT, gb.count()));
    }
    auto get_memory_limit() {
        double gb;
        check(GRB->getdblparam(env, GRB_DBL_PAR_SOFTMEMLIMIT, &gb));
        return memory_size<double, std::giga>(gb);
    }

    //////////////////////////////// Verbosity ////////////////////////////////
    void set_verbose(bool verbose) {
        check(GRB->setintparam(env, GRB_INT_PAR_OUTPUTFLAG, verbose));
    }
    bool is_verbose() {
        int verbose;
        check(GRB->getintparam(env, GRB_INT_PAR_OUTPUTFLAG, &verbose));
        return verbose != 0;
    }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////////// IIS ////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    // SOS, quadratic and general constraints have no handle to report, so
    // they are forced into the IIS for the call: the routine then never tries
    // to remove one, and the rows and bounds it reports conflict against them
    // as fixed background. The forcing attributes are the user's, and a
    // value they set must survive the call. Every write here is a model
    // modification, which Gurobi answers by discarding the held solution and
    // the IIS attributes at the next update: the routine reads its answer
    // before the restore, and a model without special constraints is left
    // untouched.
    class iis_force_guard {
    private:
        const gurobi_api & _api;
        GRBenv * _env;
        GRBmodel * _model;
        std::vector<int> _sos_force, _qconstr_force, _genconstr_force;
        bool _restored = false;

        void _check(const int error) const { _api._check(_env, error); }
        bool _forced() const noexcept {
            return !_sos_force.empty() || !_qconstr_force.empty() ||
                   !_genconstr_force.empty();
        }

        std::vector<int> _save_and_force(const char * count_attr,
                                         const char * force_attr) {
            int count;
            _check(_api.getintattr(_model, count_attr, &count));
            std::vector<int> saved(static_cast<std::size_t>(count));
            if(count == 0) return saved;
            _check(_api.getintattrarray(_model, force_attr, 0, count,
                                        saved.data()));
            std::vector<int> forced(static_cast<std::size_t>(count), 1);
            _check(_api.setintattrarray(_model, force_attr, 0, count,
                                        forced.data()));
            return saved;
        }
        int _write_back(const char * force_attr,
                        std::vector<int> & saved) noexcept {
            if(saved.empty()) return 0;
            return _api.setintattrarray(_model, force_attr, 0,
                                        static_cast<int>(saved.size()),
                                        saved.data());
        }
        // every attribute is written back before the first error is
        // returned, so a rejected write cannot leave the others forced
        int _write_back_all() noexcept {
            int first_error = 0;
            for(const int error :
                {_write_back(GRB_INT_ATTR_IIS_SOSFORCE, _sos_force),
                 _write_back(GRB_INT_ATTR_IIS_QCONSTRFORCE, _qconstr_force),
                 _write_back(GRB_INT_ATTR_IIS_GENCONSTRFORCE, _genconstr_force),
                 _api.updatemodel(_model)}) {
                if(first_error == 0) first_error = error;
            }
            return first_error;
        }

    public:
        iis_force_guard(const gurobi_api & api, GRBenv * env, GRBmodel * model)
            : _api(api), _env(env), _model(model) {
            // a constructor that throws runs no destructor
            try {
                _sos_force = _save_and_force(GRB_INT_ATTR_NUMSOS,
                                             GRB_INT_ATTR_IIS_SOSFORCE);
                _qconstr_force = _save_and_force(GRB_INT_ATTR_NUMQCONSTRS,
                                                 GRB_INT_ATTR_IIS_QCONSTRFORCE);
                _genconstr_force =
                    _save_and_force(GRB_INT_ATTR_NUMGENCONSTRS,
                                    GRB_INT_ATTR_IIS_GENCONSTRFORCE);
                // attribute writes are queued until an update
                if(_forced()) _check(_api.updatemodel(_model));
            } catch(...) {
                (void)_write_back_all();
                throw;
            }
        }
        iis_force_guard(const iis_force_guard &) = delete;
        iis_force_guard & operator=(const iis_force_guard &) = delete;

        void restore() {
            _restored = true;
            if(_forced()) _check(_write_back_all());
        }
        // values read back moments ago: the writes cannot be rejected
        ~iis_force_guard() {
            if(!_restored && _forced()) (void)_write_back_all();
        }
    };

protected:
    // IISConstr flags a row's membership, never a side
    using iis_snapshot_type =
        iis_snapshot<variable, constraint, iis_sided_status,
                     detail::iis_whole_or_one_side_status>;

    iis_snapshot_type _compute_iis() {
        _lazily_remove_variables();
        update_gurobi_model();
        const std::size_t num_col = _num_var_native_ids;
        const std::size_t num_row = num_constraints();
        detail::handle_status_table<iis_sided_status> variable_table(
            _handle_id_bound(num_col));
        detail::handle_status_table<detail::iis_whole_or_one_side_status>
            constraint_table(num_row);

        // GRBcomputeIIS reads TimeLimit itself and returns 0 on a stop, and
        // IISMinimal is 0 after numerical trouble as well as after a stop:
        // the time measured around the call is what tells the two apart.
        const double budget = get_time_limit().count();
        iis_force_guard guard(*GRB, env, model);
        const auto start = std::chrono::steady_clock::now();
        const int code = GRB->computeIIS(model);
        const double elapsed = std::chrono::duration<double>(
                                   std::chrono::steady_clock::now() - start)
                                   .count();
        // never reached under the default 1e100, always under a zero limit
        const std::optional<iis_reason> stop_reason =
            elapsed >= budget ? std::optional(iis_reason::time_limit)
                              : std::nullopt;

        // A stop before any subsystem was found, whatever the limit, leaves
        // the IIS attributes unset and returns 0: asking for one of them is
        // the only way to know.
        int minimal = 0;
        bool answered = false;
        std::vector<int> lower_in_iis(num_col), upper_in_iis(num_col),
            row_in_iis(num_row);
        std::vector<char> senses(num_row);
        if(code == 0) {
            const int minimal_code =
                GRB->getintattr(model, GRB_INT_ATTR_IIS_MINIMAL, &minimal);
            if(minimal_code != GRB_ERROR_DATA_NOT_AVAILABLE) {
                check(minimal_code);
                answered = true;
                // an empty vector has no buffer to hand over
                if(num_col > 0) {
                    check(GRB->getintattrarray(model, GRB_INT_ATTR_IIS_LB, 0,
                                               static_cast<int>(num_col),
                                               lower_in_iis.data()));
                    check(GRB->getintattrarray(model, GRB_INT_ATTR_IIS_UB, 0,
                                               static_cast<int>(num_col),
                                               upper_in_iis.data()));
                }
                if(num_row > 0) {
                    check(GRB->getintattrarray(model, GRB_INT_ATTR_IIS_CONSTR,
                                               0, static_cast<int>(num_row),
                                               row_in_iis.data()));
                    check(GRB->getcharattrarray(model, GRB_CHAR_ATTR_SENSE, 0,
                                                static_cast<int>(num_row),
                                                senses.data()));
                }
            }
        }
        guard.restore();

        if(code == GRB_ERROR_IIS_NOT_INFEASIBLE)
            return iis_snapshot_type(std::move(variable_table),
                                     std::move(constraint_table),
                                     iis_outcome::feasible);
        check(code);
        if(!answered)
            return iis_snapshot_type(std::move(variable_table),
                                     std::move(constraint_table),
                                     iis_outcome::undetermined, stop_reason);

        for(std::size_t j = 0; j < num_col; ++j) {
            const bool lower = lower_in_iis[j] != 0;
            const bool upper = upper_in_iis[j] != 0;
            if(!lower && !upper) continue;
            variable_table.set(_var_handle(static_cast<int>(j)).uid(),
                               detail::iis_flagged_status<iis_sided_status>(
                                   lower, upper, false));
        }
        for(std::size_t i = 0; i < num_row; ++i) {
            if(row_in_iis[i] == 0) continue;
            constraint_table.set(
                i,
                detail::iis_row_status_by_sense<GRB_LESS_EQUAL,
                                                GRB_GREATER_EQUAL>(senses[i]));
        }

        // An answer made of forced elements alone has no member and is
        // still an IIS: the background alone is infeasible.
        if(minimal != 0)
            return iis_snapshot_type(std::move(variable_table),
                                     std::move(constraint_table),
                                     iis_outcome::irreducible);
        return iis_snapshot_type(std::move(variable_table),
                                 std::move(constraint_table),
                                 iis_outcome::not_proven_minimal, stop_reason);
    }
};

}  // namespace gurobi::impl::v1
}  // namespace mippp
