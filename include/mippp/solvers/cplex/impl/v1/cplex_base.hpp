#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <memory>
#include <numeric>
#include <optional>
#include <ranges>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "mippp/detail/handle_guard.hpp"
#include "mippp/detail/handle_status_table.hpp"
#include "mippp/detail/invoke_key.hpp"
#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"
#include "mippp/utility/iis_outcome.hpp"
#include "mippp/utility/iis_snapshot.hpp"
#include "mippp/utility/solver_exceptions.hpp"
#include "mippp/utility/variant.hpp"

#include "mippp/solvers/cplex/impl/v1/cplex_api.hpp"
#include "mippp/solvers/remapping_model_base.hpp"

namespace mippp {
namespace cplex::impl::v1 {

struct cplex_handle_release {
    static auto free_problem(const auto & api, CPXENVptr env, CPXLPptr & lp) {
        return api.freeprob(env, &lp);
    }
    static auto free_env(const auto & api, CPXENVptr & env) {
        return api.closeCPLEX(&env);
    }
};

class cplex_base : protected remapping_model_base<int, double> {
protected:
    const cplex_api * CPX;
    CPXENVptr env;
    CPXLPptr lp;
    // declared after env and lp, which it releases
    detail::handle_guard<cplex_api, CPXENVptr, CPXLPptr, cplex_handle_release>
        handle_guard;

    std::vector<int> tmp_begins;
    std::vector<char> tmp_types;
    std::vector<double> tmp_rhs;

    void check(const int error) { CPX->_check(env, error); }
    static constexpr char constraint_sense_to_cplex_sense(
        constraint_sense rel) {
        if(rel == constraint_sense::less_equal) return 'L';
        if(rel == constraint_sense::equal) return 'E';
        return 'G';
    }
    static constexpr constraint_sense cplex_sense_to_constraint_sense(
        char sense) {
        if(sense == 'L') return constraint_sense::less_equal;
        if(sense == 'E') return constraint_sense::equal;
        return constraint_sense::greater_equal;
    }

public:
    // the anchor model_variable_params_t deduces from
    using remapping_model_base<int, double>::default_variable_params;
    std::vector<variable> variables() {
        return _live_variables(num_variables());
    }
    using model_base<int, double>::constraints;
    double infinity() const noexcept { return CPX_INFBOUND; }
    using remapping_model_base<int, double>::is_infinite;

    [[nodiscard]] explicit cplex_base(const cplex_api & api)
        : remapping_model_base<int, double>()
        , CPX(&api)
        , env(nullptr)
        , lp(nullptr)
        , handle_guard(api, env, lp) {
        env = CPX->_create_env();
        lp = CPX->_create_prob(env);
    }

    constexpr cplex_base(const cplex_base &) = delete;
    cplex_base(cplex_base && other) noexcept
        : remapping_model_base<int, double>(std::move(other))
        , CPX(other.CPX)
        , env(other.env)
        , lp(other.lp)
        , handle_guard(*CPX, env, lp)
        , tmp_begins(std::move(other.tmp_begins))
        , tmp_types(std::move(other.tmp_types))
        , tmp_rhs(std::move(other.tmp_rhs))
        , _conflict_preference(other._conflict_preference) {
        other.env = nullptr;
        other.lp = nullptr;
    }

    constexpr cplex_base & operator=(const cplex_base &) = delete;
    constexpr cplex_base & operator=(cplex_base && other) = delete;

public:
    std::size_t num_variables() {
        return static_cast<std::size_t>(CPX->getnumcols(env, lp)) -
               _var_handles_to_delete.size();
    }
    std::size_t num_constraints() {
        return static_cast<std::size_t>(CPX->getnumrows(env, lp));
    }
    std::size_t num_nonzeros() {
        return static_cast<std::size_t>(CPX->getnumnz(env, lp));
    }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////// Native handles /////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
public:
    const cplex_api & native_api() const noexcept { return *CPX; }
    std::pair<CPXENVptr, CPXLPptr> native_model() const noexcept {
        return {env, lp};
    }
    int native_id(variable v) const noexcept { return _native_id(v); }
    int native_id(constraint c) const noexcept { return c.id(); }

public:
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// Objective ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void set_maximization() { check(CPX->chgobjsen(env, lp, CPX_MAX)); }
    void set_minimization() { check(CPX->chgobjsen(env, lp, CPX_MIN)); }

    void set_objective_offset(double constant) {
        check(CPX->chgobjoffset(env, lp, constant));
    }
    void set_objective(linear_expression auto && le) {
        const std::size_t num_vars = _num_var_native_ids();
        tmp_indices.resize(num_vars);
        std::iota(tmp_indices.begin(), tmp_indices.end(), 0);
        tmp_scalars.resize(num_vars);
        std::fill(tmp_scalars.begin(), tmp_scalars.end(), 0.0);
        for(auto && [var, coef] : le.linear_terms()) {
            tmp_scalars[static_cast<std::size_t>(_native_id(var))] += coef;
        }
        check(CPX->chgobj(env, lp, static_cast<int>(num_vars),
                          tmp_indices.data(), tmp_scalars.data()));
        set_objective_offset(le.constant());
    }
    template <linear_expression LE>
    void set_objective(distinct_variables_t, LE && le) {
        set_objective(std::forward<LE>(le));
    }
    void add_to_objective(linear_expression auto && le) {
        const std::size_t num_vars = _num_var_native_ids();
        tmp_indices.resize(num_vars);
        std::iota(tmp_indices.begin(), tmp_indices.end(), 0);
        tmp_scalars.resize(num_vars);
        check(CPX->getobj(env, lp, tmp_scalars.data(), 0,
                          static_cast<int>(num_vars) - 1));
        for(auto && [var, coef] : le.linear_terms()) {
            tmp_scalars[static_cast<std::size_t>(_native_id(var))] += coef;
        }
        check(CPX->chgobj(env, lp, static_cast<int>(num_vars),
                          tmp_indices.data(), tmp_scalars.data()));
        set_objective_offset(get_objective_offset() + le.constant());
    }
    template <linear_expression LE>
    void add_to_objective(distinct_variables_t, LE && le) {
        add_to_objective(std::forward<LE>(le));
    }
    double get_objective_offset() {
        double objective_offset;
        check(CPX->getobjoffset(env, lp, &objective_offset));
        return objective_offset;
    }
    auto get_objective() {
        const std::size_t num_vars = _num_var_native_ids();
        auto coefs = std::make_shared_for_overwrite<double[]>(num_vars);
        check(CPX->getobj(env, lp, coefs.get(), 0,
                          static_cast<int>(num_vars) - 1));
        return linear_expression_view(
            std::views::transform(
                std::views::iota(0, static_cast<int>(num_vars)),
                [this, coefs = std::move(coefs)](auto i) {
                    return std::make_pair(_var_handle(i), coefs[i]);
                }),
            get_objective_offset());
    }
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// Variables ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
protected:
    int _new_var_native_id() {
        if(_remap_ids) _extend_handle_ids_map(1);
        return CPX->getnumcols(env, lp);
    }
    std::size_t _num_var_native_ids() {
        return static_cast<std::size_t>(CPX->getnumcols(env, lp));
    }
    void _lazily_remove_variables() {
        if(_var_handles_to_delete.empty()) return;
        const std::size_t old_num_variables =
            static_cast<std::size_t>(CPX->getnumcols(env, lp));
        tmp_indices.resize(old_num_variables);
        std::fill(tmp_indices.begin(), tmp_indices.end(), 0);
        for(const variable & var : _var_handles_to_delete) {
            tmp_indices[static_cast<std::size_t>(_native_id(var))] = 1;
        }
        check(CPX->delsetcols(env, lp, tmp_indices.data()));

        if(!_remap_ids) {
            _native_ids_map.resize(old_num_variables);
            _handle_ids_map.resize(old_num_variables);
            std::iota(_native_ids_map.begin(), _native_ids_map.end(), 0);
            std::iota(_handle_ids_map.begin(), _handle_ids_map.end(), 0);
            _remap_ids = true;
        }
        for(std::size_t old_native_id :
            std::views::iota(std::size_t{0}, old_num_variables)) {
            const int new_native_id = tmp_indices[old_native_id];
            if(new_native_id == -1) continue;
            _native_ids_map[static_cast<std::size_t>(
                _handle_ids_map[old_native_id])] = new_native_id;
            // delsetcols yields new_native_id <= old_native_id, so the slot
            // written here was already read: iterating in decreasing order
            // would read overwritten handles.
            _handle_ids_map[static_cast<std::size_t>(new_native_id)] =
                _handle_ids_map[old_native_id];
        }
        _shrink_handle_ids_map(_var_handles_to_delete.size());
#if defined(__cpp_lib_containers_ranges)
        _free_var_handles.append_range(_var_handles_to_delete);
#else
        _free_var_handles.insert(_free_var_handles.end(),
                                 _var_handles_to_delete.cbegin(),
                                 _var_handles_to_delete.cend());
#endif
        _var_handles_to_delete.clear();
    }

protected:
    std::size_t _add_variables(std::size_t count,
                               const variable_params & params, char type) {
        if(_remap_ids) _extend_handle_ids_map(count);
        const std::size_t handle_ids_begin =
            _new_var_handle_range(_num_var_native_ids(), count);
        std::optional<std::size_t> dbl_offset_1, dbl_offset_2, dbl_offset_3;
        tmp_scalars.resize(0u);
        if(params.obj_coef != 0.0) {
            dbl_offset_1.emplace(tmp_scalars.size());
            tmp_scalars.resize(tmp_scalars.size() + count, params.obj_coef);
        }
        if(auto lb = params.lower_bound.value_or(-CPX_INFBOUND); lb != 0.0) {
            dbl_offset_2.emplace(tmp_scalars.size());
            tmp_scalars.resize(tmp_scalars.size() + count, lb);
        }
        if(auto ub = params.upper_bound.value_or(CPX_INFBOUND);
           ub != CPX_INFBOUND) {
            dbl_offset_3.emplace(tmp_scalars.size());
            tmp_scalars.resize(tmp_scalars.size() + count, ub);
        }
        if(type != CPX_CONTINUOUS) {
            tmp_types.resize(count);
            std::fill(tmp_types.begin(), tmp_types.end(), type);
        }
        check(CPX->newcols(
            env, lp, static_cast<int>(count),
            dbl_offset_1.has_value()
                ? (tmp_scalars.data() +
                   static_cast<std::ptrdiff_t>(dbl_offset_1.value()))
                : nullptr,
            dbl_offset_2.has_value()
                ? (tmp_scalars.data() +
                   static_cast<std::ptrdiff_t>(dbl_offset_2.value()))
                : nullptr,
            dbl_offset_3.has_value()
                ? (tmp_scalars.data() +
                   static_cast<std::ptrdiff_t>(dbl_offset_3.value()))
                : nullptr,
            (type != CPX_CONTINUOUS) ? tmp_types.data() : nullptr, nullptr));
        return handle_ids_begin;
    }

public:
    friend model_base<int, double>;
    using model_base<int, double>::add_variable;
    using model_base<int, double>::add_variables;
    using model_base<int, double>::add_named_variable;
    using model_base<int, double>::add_named_variables;

private:
    variable _add_variable(const variable_params & params, char type,
                           char * name = nullptr) {
        const int var_id = _new_var_native_id();
        const double lb = params.lower_bound.value_or(-CPX_INFBOUND);
        const double ub = params.upper_bound.value_or(CPX_INFBOUND);
        check(CPX->newcols(env, lp, 1, &params.obj_coef, &lb, &ub,
                           (type != CPX_CONTINUOUS) ? &type : nullptr, &name));
        return _new_var_handle(var_id);
    }
    variable _new_variable(const variable_params & params, variable_kind kind) {
        return _add_variable(params,
                             kind == variable_kind::continuous ? CPX_CONTINUOUS
                             : kind == variable_kind::integer  ? CPX_INTEGER
                                                               : CPX_BINARY);
    }
    std::size_t _new_variables(std::size_t count,
                               const variable_params & params,
                               variable_kind kind) {
        return _add_variables(count, params,
                              kind == variable_kind::continuous ? CPX_CONTINUOUS
                              : kind == variable_kind::integer  ? CPX_INTEGER
                                                                : CPX_BINARY);
    }

private:
    template <typename ER>
    inline variable _add_column(ER && entries, const variable_params & params) {
        const int var_id = _new_var_native_id();
        const int cmatbeg = 0;
        _reset_cache();
        _register_constraints_entries<true>(entries);
        const double lb = params.lower_bound.value_or(-CPX_INFBOUND);
        const double ub = params.upper_bound.value_or(CPX_INFBOUND);
        check(CPX->addcols(env, lp, 1, static_cast<int>(tmp_indices.size()),
                           &params.obj_coef, &cmatbeg, tmp_indices.data(),
                           tmp_scalars.data(), &lb, &ub, nullptr));
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

    void set_objective_coefficient(variable v, double c) {
        int var_id = _native_id(v);
        check(CPX->chgobj(env, lp, 1, &var_id, &c));
    }
    void set_variable_lower_bound(variable v, double lb) {
        int var_id = _native_id(v);
        char lu = 'L';
        check(CPX->chgbds(env, lp, 1, &var_id, &lu, &lb));
    }
    void set_variable_upper_bound(variable v, double ub) {
        int var_id = _native_id(v);
        char lu = 'U';
        check(CPX->chgbds(env, lp, 1, &var_id, &lu, &ub));
    }
    void set_variable_name(variable v, const std::string & name) {
        int var_id = _native_id(v);
        char * col_name = const_cast<char *>(name.c_str());
        check(CPX->chgcolname(env, lp, 1, &var_id, &col_name));
    }

    double get_objective_coefficient(variable v) {
        const int var_id = _native_id(v);
        double coef;
        check(CPX->getobj(env, lp, &coef, var_id, var_id));
        return coef;
    }
    double get_variable_lower_bound(variable v) {
        const int var_id = _native_id(v);
        double b;
        check(CPX->getlb(env, lp, &b, var_id, var_id));
        return b;
    }
    double get_variable_upper_bound(variable v) {
        const int var_id = _native_id(v);
        double b;
        check(CPX->getub(env, lp, &b, var_id, var_id));
        return b;
    }
    std::string get_variable_name(variable v) {
        const int var_id = _native_id(v);
        std::string name;
        name.resize(name.capacity());
        char * subptr;
        int surplus = 0;
        const int status = CPX->getcolname(env, lp, &subptr, name.data(),
                                           static_cast<int>(name.size()),
                                           &surplus, var_id, var_id);
        // 1219 = CPXERR_NO_NAMES: nothing in the problem has been named, so
        // the call leaves surplus untouched -- without this the resize below
        // would hand back name.size() - 1 bytes of the uninitialized buffer.
        if(status == 1219) return std::string();
        // 1207 = CPXERR_NEGATIVE_SURPLUS: the buffer was too small, and
        // surplus says by how much (it is negative, so this subtraction
        // grows the string).
        if(status == 1207) {
            name.resize(name.size() - static_cast<std::size_t>(surplus));
            check(CPX->getcolname(env, lp, &subptr, name.data(),
                                  static_cast<int>(name.size()), &surplus,
                                  var_id, var_id));
        } else {
            check(status);
        }
        name.resize(name.size() - static_cast<std::size_t>(surplus + 1));
        return name;
    }
    ///////////////////////////////////////////////////////////////////////////
    /////////////////////////////// Constraints ///////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    template <bool distinct, linear_constraint LC>
    constraint _add_constraint(LC && lc) {
        int constr_id = static_cast<int>(num_constraints());
        if constexpr(!distinct) _prepare_coalescing(_num_var_native_ids());
        _reset_cache();
        _register_variables_entries<distinct>(lc.linear_terms());
        int matbegin = 0;
        const double b = lc.rhs();
        const char sense = constraint_sense_to_cplex_sense(lc.sense());
        check(CPX->addrows(env, lp, 0, 1, static_cast<int>(tmp_indices.size()),
                           &b, &sense, &matbegin, tmp_indices.data(),
                           tmp_scalars.data(), nullptr, nullptr));
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
        tmp_begins.emplace_back(static_cast<int>(tmp_indices.size()));
        tmp_types.emplace_back(constraint_sense_to_cplex_sense(lc.sense()));
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
        if constexpr(!distinct) _prepare_coalescing(_num_var_native_ids());
        _reset_cache();
        tmp_begins.resize(0);
        tmp_types.resize(0);
        tmp_rhs.resize(0);
        const int offset = static_cast<int>(num_constraints());
        int constr_id = offset;
        for(auto && key : keys) {
            _register_first_valued_constraint<distinct>(key,
                                                        constraint_lambdas...);
            ++constr_id;
        }
        check(CPX->addrows(env, lp, 0, static_cast<int>(tmp_begins.size()),
                           static_cast<int>(tmp_indices.size()), tmp_rhs.data(),
                           tmp_types.data(), tmp_begins.data(),
                           tmp_indices.data(), tmp_scalars.data(), nullptr,
                           nullptr));
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
    char _row_sense(int row) {
        char sense;
        check(CPX->getsense(env, lp, &sense, row, row));
        return sense;
    }
    // 'R' spans rhs to rhs + rngval, the range value being of either sign
    std::pair<double, double> _row_sides(int row) {
        const char sense = _row_sense(row);
        double rhs;
        check(CPX->getrhs(env, lp, &rhs, row, row));
        if(sense == 'L') return {-CPX_INFBOUND, rhs};
        if(sense == 'G') return {rhs, CPX_INFBOUND};
        if(sense == 'E') return {rhs, rhs};
        double range;
        check(CPX->getrngval(env, lp, &range, row, row));
        if(range >= 0.) return {rhs, rhs + range};
        return {rhs + range, rhs};
    }
    // An 'R' row is a right-hand side and a width, so crossed sides have no
    // representation: written as they are, they would read back swapped and
    // the solver would see a satisfiable range. An infinite side cannot live
    // on an 'R' row either (CPLEX clamps the range and the row turns
    // infeasible), so one infinite side gives an inequality and two a <= row
    // with an infinite right-hand side, which CPLEX ignores.
    void _set_row_sides(int row, double lower, double upper) {
        if(lower > upper)
            throw std::invalid_argument(
                "cplex: a row's sides cannot cross, CPLEX stores a ranged row "
                "as a right-hand side and a range width");
        const bool lower_finite = lower > -CPX_INFBOUND;
        const bool upper_finite = upper < CPX_INFBOUND;
        char sense;
        double rhs = lower;
        if(lower_finite && upper_finite) {
            sense = lower == upper ? 'E' : 'R';
        } else if(lower_finite) {
            sense = 'G';
        } else {
            sense = 'L';
            rhs = upper;
        }
        check(CPX->chgsense(env, lp, 1, &row, &sense));
        check(CPX->chgrhs(env, lp, 1, &row, &rhs));
        // the switch to 'R' does not always zero the range value left by an
        // earlier switch away from it (measured), so the width is written
        // every time
        if(sense == 'R') {
            const double range = upper - lower;
            check(CPX->chgrngval(env, lp, 1, &row, &range));
        }
    }

public:
    void set_constraint_rhs(constraint constr, double rhs) {
        int constr_id = constr.id();
        check(CPX->chgrhs(env, lp, 1, &constr_id, &rhs));
    }
    void set_constraint_sense(constraint constr, constraint_sense r) {
        int constr_id = constr.id();
        char sense = constraint_sense_to_cplex_sense(r);
        check(CPX->chgsense(env, lp, 1, &constr_id, &sense));
    }
    void set_constraint_lower_bound(constraint constr, double lb) {
        _set_row_sides(constr.id(), lb, _row_sides(constr.id()).second);
    }
    void set_constraint_upper_bound(constraint constr, double ub) {
        _set_row_sides(constr.id(), _row_sides(constr.id()).first, ub);
    }

    auto get_constraint_lhs(constraint constr) {
        int placeholder, surplus, beg;
        if(int error =
               CPX->getrows(env, lp, &placeholder, nullptr, nullptr, nullptr, 0,
                            &surplus, constr.id(), constr.id());
           error != 1207 && error != 0)
            throw std::runtime_error("CPLEX: error " + std::to_string(error));

        const int num_nz = -surplus;
        auto indices = std::make_shared_for_overwrite<int[]>(
            static_cast<std::size_t>(num_nz));
        auto coefs = std::make_shared_for_overwrite<double[]>(
            static_cast<std::size_t>(num_nz));
        check(CPX->getrows(env, lp, &placeholder, &beg, indices.get(),
                           coefs.get(), num_nz, &surplus, constr.id(),
                           constr.id()));
        return std::views::transform(
            std::views::iota(0, num_nz), [this, indices = std::move(indices),
                                          coefs = std::move(coefs)](int i) {
                return std::make_pair(_var_handle(indices.get()[i]),
                                      coefs.get()[i]);
            });
    }
    double get_constraint_rhs(constraint constr) {
        double rhs;
        check(CPX->getrhs(env, lp, &rhs, constr.id(), constr.id()));
        return rhs;
    }
    constraint_sense get_constraint_sense(constraint constr) {
        return cplex_sense_to_constraint_sense(_row_sense(constr.id()));
    }
    double get_constraint_lower_bound(constraint constr) {
        return _row_sides(constr.id()).first;
    }
    double get_constraint_upper_bound(constraint constr) {
        return _row_sides(constr.id()).second;
    }
    auto get_constraint(constraint constr) {
        return linear_constraint_view(
            linear_expression_view(get_constraint_lhs(constr),
                                   -get_constraint_rhs(constr)),
            get_constraint_sense(constr));
    }
    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////// Limits //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void set_time_limit(std::chrono::duration<double> t) {
        // The ceiling is 1e75, the fresh value, not infinity() (CPX_INFBOUND).
        check(CPX->setdblparam(env, CPXPARAM_TimeLimit,
                               std::min(t.count(), 1e75)));
    }
    auto get_time_limit() {
        double t;
        check(CPX->getdblparam(env, CPXPARAM_TimeLimit, &t));
        return std::chrono::duration<double>(t);
    }
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// Verbosity ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void set_verbose(bool verbose) {
        check(CPX->setintparam(env, CPXPARAM_ScreenOutput, verbose));
    }
    bool is_verbose() {
        int verbose;
        check(CPX->getintparam(env, CPXPARAM_ScreenOutput, &verbose));
        return verbose != 0;
    }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////////// IIS ////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    // The refiner keeps its state on an unchanged problem: after a stop, a
    // call with the same groups resumes it and, when the stop came before its
    // first iteration, ends "feasible" on an infeasible model (measured on
    // 22.1.1 and 22.1.2). A preference value no earlier call used makes the
    // call a fresh refinement, and one equal value on every group leaves the
    // refiner's choices alone: only relative preferences count, 0 forces a
    // group in and a negative value leaves it out.
    double _conflict_preference = 0.;

protected:
    // Membership only: the refiner sides a bound by its group and names a
    // row whole, so an inequality row takes the side of its sense and an
    // equality or ranged row stays a plain member.
    using iis_row_status =
        std::variant<iis_status::absent, iis_status::member,
                     iis_status::member_lower, iis_status::member_upper>;
    using iis_snapshot_type =
        iis_snapshot<variable, constraint, iis_sided_status, iis_row_status>;

    iis_snapshot_type _compute_iis() {
        const std::size_t num_col = _num_var_native_ids();
        const std::size_t num_row = num_constraints();
        detail::handle_status_table<iis_sided_status> variable_table(
            _handle_id_bound(num_col));
        detail::handle_status_table<iis_row_status> constraint_table(num_row);

        std::vector<double> lower(num_col), upper(num_col);
        if(num_col > 0) {
            const int last = static_cast<int>(num_col) - 1;
            check(CPX->getlb(env, lp, lower.data(), 0, last));
            check(CPX->getub(env, lp, upper.data(), 0, last));
        }
        std::vector<char> senses(num_row);
        std::vector<double> rhs(num_row);
        if(num_row > 0) {
            const int last = static_cast<int>(num_row) - 1;
            check(CPX->getsense(env, lp, senses.data(), 0, last));
            check(CPX->getrhs(env, lp, rhs.data(), 0, last));
        }
        // One group per finite bound side and per row with a finite side.
        // Indicators and SOS are left out on purpose: an ungrouped constraint
        // stays in every subproblem without analysis (documented), which
        // makes it background.
        std::vector<int> group_indices;
        std::vector<char> group_types;
        group_indices.reserve(2 * num_col + num_row);
        group_types.reserve(2 * num_col + num_row);
        for(std::size_t j = 0; j < num_col; ++j) {
            if(lower[j] > -CPX_INFBOUND) {
                group_indices.push_back(static_cast<int>(j));
                group_types.push_back(static_cast<char>(CPX_CON_LOWER_BOUND));
            }
            if(upper[j] < CPX_INFBOUND) {
                group_indices.push_back(static_cast<int>(j));
                group_types.push_back(static_cast<char>(CPX_CON_UPPER_BOUND));
            }
        }
        for(std::size_t i = 0; i < num_row; ++i) {
            if((senses[i] == 'L' && rhs[i] >= CPX_INFBOUND) ||
               (senses[i] == 'G' && rhs[i] <= -CPX_INFBOUND))
                continue;
            group_indices.push_back(static_cast<int>(i));
            group_types.push_back(static_cast<char>(CPX_CON_LINEAR));
        }
        const int num_groups = static_cast<int>(group_indices.size());
        std::vector<int> group_begins(group_indices.size());
        std::iota(group_begins.begin(), group_begins.end(), 0);
        const std::vector<double> group_preferences(group_indices.size(),
                                                    ++_conflict_preference);

        check(CPX->refineconflictext(
            env, lp, num_groups, num_groups, group_preferences.data(),
            group_begins.data(), group_indices.data(), group_types.data()));
        const int conflict_status = CPX->getstat(env, lp);
        if(conflict_status == CPX_STAT_CONFLICT_FEASIBLE)
            return iis_snapshot_type(std::move(variable_table),
                                     std::move(constraint_table),
                                     iis_outcome::feasible);
        // A stopped refinement flags every group possible until its first
        // subproblem completes, on a feasible model too, and a node-limit
        // stop excluded groups whose subproblem had only hit the limit (both
        // measured): the flags of a stop prove nothing, so no subsystem is
        // reported.
        if(conflict_status >= CPX_STAT_CONFLICT_ABORT_CONTRADICTION &&
           conflict_status <= CPX_STAT_CONFLICT_ABORT_DETTIME_LIM)
            return iis_snapshot_type(
                std::move(variable_table), std::move(constraint_table),
                iis_outcome::undetermined,
                conflict_status == CPX_STAT_CONFLICT_ABORT_TIME_LIM
                    ? std::optional(iis_reason::time_limit)
                    : std::nullopt);
        if(conflict_status != CPX_STAT_CONFLICT_MINIMAL)
            throw solver_error(
                ("mippp: CPXrefineconflictext left the unexpected status " +
                 std::to_string(conflict_status))
                    .c_str());

        std::vector<int> group_flags(group_indices.size());
        if(num_groups > 0)
            check(CPX->getconflictext(env, lp, group_flags.data(), 0,
                                      num_groups - 1));
        bool proven = true;
        for(std::size_t k = 0; k < group_indices.size(); ++k) {
            const int flag = group_flags[k];
            if(flag == CPX_CONFLICT_EXCLUDED) continue;
            if(flag < CPX_CONFLICT_MEMBER) proven = false;
            const int native_index = group_indices[k];
            if(group_types[k] == CPX_CON_LINEAR) {
                const auto row = static_cast<std::size_t>(native_index);
                constraint_table.set(
                    row, senses[row] == 'L'
                             ? iis_row_status{iis_status::member_upper{}}
                         : senses[row] == 'G'
                             ? iis_row_status{iis_status::member_lower{}}
                             : iis_row_status{iis_status::member{}});
                continue;
            }
            // the lower group of a column precedes its upper group
            const std::size_t id = _var_handle(native_index).uid();
            const bool other_side_flagged =
                !is<iis_status::absent>(variable_table.get(id));
            variable_table.set(
                id, other_side_flagged
                        ? iis_sided_status{iis_status::member_both{}}
                    : group_types[k] == CPX_CON_LOWER_BOUND
                        ? iis_sided_status{iis_status::member_lower{}}
                        : iis_sided_status{iis_status::member_upper{}});
        }
        return iis_snapshot_type(std::move(variable_table),
                                 std::move(constraint_table),
                                 proven ? iis_outcome::irreducible
                                        : iis_outcome::not_proven_minimal);
    }
};

}  // namespace cplex::impl::v1
}  // namespace mippp
