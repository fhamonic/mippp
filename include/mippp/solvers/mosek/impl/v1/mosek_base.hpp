#pragma once

#include <algorithm>
#include <cstddef>
#include <memory>
#include <numeric>
#include <optional>
#include <ranges>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "mippp/detail/invoke_key.hpp"
#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/model_base.hpp"
#include "mippp/solvers/mosek/impl/v1/mosek_api.hpp"

namespace mippp {
namespace mosek::impl::v1 {

class mosek_base : protected model_base<int, double> {
protected:
    const mosek_api * MSK;
    MSKenv_t env;
    MSKtask_t task;

    std::vector<index> tmp_begins;
    std::vector<MSKboundkeye> tmp_boundkeye;
    std::vector<scalar> tmp_rhs;
    std::vector<MSKvariabletypee> tmp_vartype;

    void check(const MSKrescodee error) const { MSK->_check(error); }
    static constexpr MSKboundkeye constraint_sense_to_mosek_sense(
        constraint_sense rel) {
        if(rel == constraint_sense::less_equal) return MSK_BK_UP;
        if(rel == constraint_sense::equal) return MSK_BK_FX;
        return MSK_BK_LO;
    }
    static constexpr constraint_sense mosek_sense_to_constraint_sense(
        MSKboundkeye sense) {
        if(sense == MSK_BK_UP) return constraint_sense::less_equal;
        if(sense == MSK_BK_FX) return constraint_sense::equal;
        return constraint_sense::greater_equal;
    }

public:
    // the anchor model_variable_params_t deduces from
    using model_base<int, double>::default_variable_params;
    double infinity() const noexcept { return MSK_INFINITY; }
    using model_base<int, double>::is_infinite;

    [[nodiscard]] explicit mosek_base(const mosek_api & api)
        : model_base<int, double>(), MSK(&api), env(nullptr), task(nullptr) {
        check(MSK->makeenv(&env, nullptr));
        check(MSK->makeemptytask(env, &task));
    }
    ~mosek_base() {
        if(task) check(MSK->deletetask(&task));
        if(env) check(MSK->deleteenv(&env));
    }

    constexpr mosek_base(const mosek_base &) = delete;
    constexpr mosek_base(mosek_base && other) noexcept
        : model_base<int, double>(std::move(other))
        , MSK(other.MSK)
        , env(other.env)
        , task(other.task)
        , tmp_begins(std::move(other.tmp_begins))
        , tmp_boundkeye(std::move(other.tmp_boundkeye))
        , tmp_rhs(std::move(other.tmp_rhs))
        , tmp_vartype(std::move(other.tmp_vartype)) {
        other.task = nullptr;
        other.env = nullptr;
    }

    constexpr mosek_base & operator=(const mosek_base &) = delete;
    constexpr mosek_base & operator=(mosek_base && other) = delete;

    std::size_t num_variables() {
        MSKint32t num;
        check(MSK->getnumvar(task, &num));
        return static_cast<std::size_t>(num);
    }
    std::size_t num_constraints() {
        MSKint32t num;
        check(MSK->getnumcon(task, &num));
        return static_cast<std::size_t>(num);
    }
    std::size_t num_nonzeros() {
        MSKint32t num;
        check(MSK->getnumanz(task, &num));
        return static_cast<std::size_t>(num);
    }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////// Native handles /////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
public:
    const mosek_api & native_api() const noexcept { return *MSK; }
    std::pair<MSKenv_t, MSKtask_t> native_model() const noexcept {
        return {env, task};
    }
    int native_id(variable v) const noexcept { return v.id(); }
    int native_id(constraint c) const noexcept { return c.id(); }

public:
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// Objective ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void set_maximization() {
        check(MSK->putobjsense(task, MSK_OBJECTIVE_SENSE_MAXIMIZE));
    }
    void set_minimization() {
        check(MSK->putobjsense(task, MSK_OBJECTIVE_SENSE_MINIMIZE));
    }

    void set_objective_offset(scalar constant) {
        check(MSK->putcfix(task, constant));
    }
    void set_objective(linear_expression auto && le) {
        auto num_vars = num_variables();
        tmp_scalars.resize(num_vars);
        std::fill(tmp_scalars.begin(), tmp_scalars.end(), 0.0);
        for(auto && [var, coef] : le.linear_terms()) {
            tmp_scalars[var.uid()] += coef;
        }
        check(MSK->putcslice(task, 0, static_cast<index>(num_vars),
                             tmp_scalars.data()));
        set_objective_offset(le.constant());
    }
    template <linear_expression LE>
    void set_objective(distinct_variables_t, LE && le) {
        set_objective(std::forward<LE>(le));
    }
    void add_to_objective(linear_expression auto && le) {
        auto num_vars = num_variables();
        tmp_scalars.resize(num_vars);
        check(MSK->getc(task, tmp_scalars.data()));
        for(auto && [var, coef] : le.linear_terms()) {
            tmp_scalars[var.uid()] += coef;
        }
        check(MSK->putcslice(task, 0, static_cast<index>(num_vars),
                             tmp_scalars.data()));
        set_objective_offset(get_objective_offset() + le.constant());
    }
    template <linear_expression LE>
    void add_to_objective(distinct_variables_t, LE && le) {
        add_to_objective(std::forward<LE>(le));
    }

    scalar get_objective_offset() {
        scalar objective_offset;
        check(MSK->getcfix(task, &objective_offset));
        return objective_offset;
    }
    auto get_objective() {
        const auto num_vars = num_variables();
        auto coefs = std::make_shared_for_overwrite<double[]>(num_vars);
        check(MSK->getc(task, coefs.get()));
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
protected:
    void _add_variable(const int & var_id, const variable_params & params,
                       const MSKvariabletypee type) {
        check(MSK->appendvars(task, 1));
        MSKboundkeye boundkey = MSK_BK_FR;
        scalar lb = params.lower_bound.value_or(-MSK_INFINITY);
        scalar ub = params.upper_bound.value_or(+MSK_INFINITY);
        if(params.lower_bound.has_value() && params.upper_bound.has_value()) {
            boundkey = (lb == ub) ? MSK_BK_FX : MSK_BK_RA;
        } else if(params.lower_bound.has_value()) {
            boundkey = MSK_BK_LO;
        } else if(params.upper_bound.has_value()) {
            boundkey = MSK_BK_UP;
        }
        check(MSK->putvarbound(task, var_id, boundkey, lb, ub));
        check(MSK->putcj(task, var_id, params.obj_coef));

        if(type != MSK_VAR_TYPE_CONT) {
            check(MSK->putvartype(task, var_id, type));
        }
    }
    void _add_variables(std::size_t offset, std::size_t count,
                        const variable_params & params,
                        const MSKvariabletypee type) {
        check(MSK->appendvars(task, static_cast<int>(count)));
        if(auto obj = params.obj_coef; obj != 0.0) {
            tmp_scalars.resize(count);
            std::fill(tmp_scalars.begin(), tmp_scalars.end(), obj);
            check(MSK->putcslice(task, static_cast<index>(offset),
                                 static_cast<index>(offset + count),
                                 tmp_scalars.data()));
        }
        MSKboundkeye boundkey = MSK_BK_FR;
        scalar lb = params.lower_bound.value_or(-MSK_INFINITY);
        scalar ub = params.upper_bound.value_or(+MSK_INFINITY);
        if(params.lower_bound.has_value() && params.upper_bound.has_value()) {
            boundkey = (lb == ub) ? MSK_BK_FX : MSK_BK_RA;
        } else if(params.lower_bound.has_value()) {
            boundkey = MSK_BK_LO;
        } else if(params.upper_bound.has_value()) {
            boundkey = MSK_BK_UP;
        }
        check(MSK->putvarboundsliceconst(task, static_cast<index>(offset),
                                         static_cast<index>(offset + count),
                                         boundkey, lb, ub));

        if(type != MSK_VAR_TYPE_CONT) {
            tmp_indices.resize(count);
            std::iota(tmp_indices.begin(), tmp_indices.end(),
                      static_cast<index>(offset));
            tmp_vartype.resize(count);
            std::fill(tmp_vartype.begin(), tmp_vartype.end(), type);
            check(MSK->putvartypelist(task, static_cast<index>(count),
                                      tmp_indices.data(), tmp_vartype.data()));
        }
    }

public:
    friend model_base<int, double>;
    using model_base<int, double>::add_variable;
    using model_base<int, double>::add_variables;
    using model_base<int, double>::add_named_variable;
    using model_base<int, double>::add_named_variables;

private:
    std::size_t _new_variables(std::size_t count,
                               const variable_params & params,
                               variable_kind kind) {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params,
                       kind == variable_kind::continuous ? MSK_VAR_TYPE_CONT
                                                         : MSK_VAR_TYPE_INT);
        return offset;
    }

private:
    template <typename ER>
    inline variable _add_column(ER && entries, const variable_params & params) {
        const int var_id = static_cast<int>(num_variables());
        _add_variable(var_id, params, MSK_VAR_TYPE_CONT);
        _reset_cache();
        _register_constraints_entries<true>(entries);
        check(MSK->putacol(task, var_id, static_cast<int>(tmp_indices.size()),
                           tmp_indices.data(), tmp_scalars.data()));
        return variable(var_id);
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

    void set_objective_coefficient(variable v, scalar c) {
        check(MSK->putcj(task, v.id(), c));
    }
    void set_variable_lower_bound(variable v, scalar lb) {
        check(MSK->chgvarbound(task, v.id(), 1, 1, lb));
    }
    void set_variable_upper_bound(variable v, scalar ub) {
        check(MSK->chgvarbound(task, v.id(), 0, 1, ub));
    }
    void set_variable_name(variable v, const std::string & name) {
        check(MSK->putvarname(task, v.id(), name.c_str()));
    }

    scalar get_objective_coefficient(variable v) {
        scalar coef;
        check(MSK->getcj(task, v.id(), &coef));
        return coef;
    }
    scalar get_variable_lower_bound(variable v) {
        MSKboundkeye boundkey;
        scalar lb, ub;
        check(MSK->getvarbound(task, v.id(), &boundkey, &lb, &ub));
        return lb;
    }
    scalar get_variable_upper_bound(variable v) {
        MSKboundkeye boundkey;
        scalar lb, ub;
        check(MSK->getvarbound(task, v.id(), &boundkey, &lb, &ub));
        return ub;
    }
    std::string get_variable_name(variable v) {
        MSKint32t len;
        check(MSK->getvarnamelen(task, v.id(), &len));
        std::string name(static_cast<std::size_t>(len + 1), '\0');
        check(MSK->getvarname(task, v.id(), len + 1, name.data()));
        name.pop_back();
        return name;
    }
    ///////////////////////////////////////////////////////////////////////////
    /////////////////////////////// Constraints ///////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    template <bool distinct, linear_constraint LC>
    constraint _add_constraint(LC && lc) {
        auto constr_id = static_cast<index>(num_constraints());
        check(MSK->appendcons(task, 1));
        if constexpr(!distinct) _prepare_coalescing(num_variables());
        _reset_cache();
        _register_variables_entries<distinct>(lc.linear_terms());
        check(MSK->putarow(task, constr_id,
                           static_cast<index>(tmp_indices.size()),
                           tmp_indices.data(), tmp_scalars.data()));
        const scalar b = lc.rhs();
        check(MSK->putconbound(task, constr_id,
                               constraint_sense_to_mosek_sense(lc.sense()), b,
                               b));
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
        tmp_begins.emplace_back(static_cast<index>(tmp_indices.size()));
        tmp_boundkeye.emplace_back(constraint_sense_to_mosek_sense(lc.sense()));
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
        if constexpr(!distinct) _prepare_coalescing(num_variables());
        _reset_cache();
        tmp_begins.resize(0);
        tmp_boundkeye.resize(0);
        tmp_rhs.resize(0);
        const index offset = static_cast<index>(num_constraints());
        index constr_id = offset;
        for(auto && key : keys) {
            _register_first_valued_constraint<distinct>(key,
                                                        constraint_lambdas...);
            ++constr_id;
        }
        check(MSK->appendcons(task, constr_id - offset));
        tmp_begins.emplace_back(static_cast<index>(tmp_indices.size()));
        check(MSK->putarowslice(task, offset, constr_id, tmp_begins.data(),
                                tmp_begins.data() + 1, tmp_indices.data(),
                                tmp_scalars.data()));
        check(MSK->putconboundslice(task, offset, constr_id,
                                    tmp_boundkeye.data(), tmp_rhs.data(),
                                    tmp_rhs.data()));
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
};

}  // namespace mosek::impl::v1
}  // namespace mippp
