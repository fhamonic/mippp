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

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/copt/v7_2/copt_api.hpp"
#include "mippp/solvers/model_base.hpp"

namespace mippp {
namespace copt::v7_2 {

class copt_base : protected model_base<int, double> {
protected:
    const copt_api * COPT;
    copt_env * env;
    copt_prob * prob;

    std::vector<index> tmp_begins;
    std::vector<char> tmp_types;
    std::vector<scalar> tmp_rhs;

    void check(const ret_code error) { COPT->_check(env, error); }
    static constexpr char constraint_sense_to_copt_sense(constraint_sense rel) {
        if(rel == constraint_sense::less_equal) return COPT_LESS_EQUAL;
        if(rel == constraint_sense::equal) return COPT_EQUAL;
        return COPT_GREATER_EQUAL;
    }
    static constexpr constraint_sense copt_sense_to_constraint_sense(
        const char sense) {
        if(sense == COPT_LESS_EQUAL) return constraint_sense::less_equal;
        if(sense == COPT_EQUAL) return constraint_sense::equal;
        return constraint_sense::greater_equal;
    }

public:
    // the anchor model_variable_params_t deduces from
    using model_base<int, double>::default_variable_params;

    [[nodiscard]] explicit copt_base(const copt_api & api)
        : model_base<int, double>(), COPT(&api), env(nullptr), prob(nullptr) {
        check(COPT->CreateEnv(&env));
        check(COPT->CreateProb(env, &prob));
    }
    ~copt_base() {
        if(prob) check(COPT->DeleteProb(&prob));
        if(env) check(COPT->DeleteEnv(&env));
    }

    constexpr copt_base(const copt_base &) = delete;
    constexpr copt_base(copt_base && other) noexcept
        : model_base<int, double>(std::move(other))
        , COPT(other.COPT)
        , env(other.env)
        , prob(other.prob)
        , tmp_begins(std::move(other.tmp_begins))
        , tmp_types(std::move(other.tmp_types))
        , tmp_rhs(std::move(other.tmp_rhs)) {
        other.env = nullptr;
        other.prob = nullptr;
    }

    constexpr copt_base & operator=(const copt_base &) = delete;
    constexpr copt_base & operator=(copt_base && other) = delete;

    std::size_t num_variables() {
        int num;
        check(COPT->GetIntAttr(prob, COPT_INTATTR_COLS, &num));
        return static_cast<std::size_t>(num);
    }
    std::size_t num_constraints() {
        int num;
        check(COPT->GetIntAttr(prob, COPT_INTATTR_ROWS, &num));
        return static_cast<std::size_t>(num);
    }
    std::size_t num_entries() {
        int num;
        check(COPT->GetIntAttr(prob, COPT_INTATTR_ELEMS, &num));
        return static_cast<std::size_t>(num);
    }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////// Native handles /////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
public:
    std::pair<copt_env *, copt_prob *> native_model() const noexcept {
        return {env, prob};
    }
    int native_id(variable v) const noexcept { return v.id(); }
    int native_id(constraint c) const noexcept { return c.id(); }

public:
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// Objective ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void set_maximization() { check(COPT->SetObjSense(prob, COPT_MAXIMIZE)); }
    void set_minimization() { check(COPT->SetObjSense(prob, COPT_MINIMIZE)); }

    void set_objective_offset(scalar constant) {
        check(COPT->SetObjConst(prob, constant));
    }

private:
    template <bool distinct, linear_expression LE>
    void _set_objective(LE && le) {
        if constexpr(!distinct) _prepare_coalescing(num_variables());
        _reset_cache();
        _register_variables_entries<distinct>(le.linear_terms());
        check(COPT->ReplaceColObj(prob, static_cast<int>(tmp_indices.size()),
                                  tmp_indices.data(), tmp_scalars.data()));
        set_objective_offset(le.constant());
    }

public:
    template <linear_expression LE>
    void set_objective(LE && le) {
        _set_objective<false>(std::forward<LE>(le));
    }
    template <linear_expression LE>
    void set_objective(distinct_variables_t, LE && le) {
        _set_objective<true>(std::forward<LE>(le));
    }
    void add_objective(linear_expression auto && le) {
        const auto num_vars = num_variables();
        tmp_indices.resize(num_vars);
        std::iota(tmp_indices.begin(), tmp_indices.end(), 0);
        tmp_scalars.resize(num_vars);
        check(COPT->GetColInfo(prob, COPT_DBLINFO_OBJ,
                               static_cast<int>(num_vars), tmp_indices.data(),
                               tmp_scalars.data()));
        for(auto && [var, coef] : le.linear_terms()) {
            tmp_scalars[var.uid()] += coef;
        }
        check(COPT->ReplaceColObj(prob, static_cast<int>(tmp_indices.size()),
                                  tmp_indices.data(), tmp_scalars.data()));
        set_objective_offset(get_objective_offset() + le.constant());
    }
    scalar get_objective_offset() {
        scalar objective_offset;
        check(COPT->GetDblAttr(prob, COPT_DBLATTR_OBJCONST, &objective_offset));
        return objective_offset;
    }
    auto get_objective() {
        const auto num_vars = num_variables();
        auto coefs = std::make_shared_for_overwrite<double[]>(num_vars);
        tmp_indices.resize(num_vars);
        std::iota(tmp_indices.begin(), tmp_indices.end(), 0);
        check(COPT->GetColInfo(prob, COPT_DBLINFO_OBJ,
                               static_cast<int>(num_vars), tmp_indices.data(),
                               coefs.get()));
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
    variable _add_variable(const variable_params & params, const char type,
                           const char * name = nullptr) {
        int var_id = static_cast<int>(num_variables());
        check(COPT->AddCol(prob, params.obj_coef, 0, nullptr, nullptr, type,
                           params.lower_bound.value_or(-COPT_INFINITY),
                           params.upper_bound.value_or(+COPT_INFINITY), name));
        return variable(var_id);
    }
    void _add_variables(std::size_t count, const variable_params & params,
                        const char type) {
        std::optional<std::size_t> dbl_offset_1, dbl_offset_2;
        tmp_scalars.resize(count);
        std::fill(tmp_scalars.begin(), tmp_scalars.end(), params.obj_coef);
        if(auto lb = params.lower_bound.value_or(-COPT_INFINITY); lb != 0.0) {
            dbl_offset_1.emplace(tmp_scalars.size());
            tmp_scalars.resize(tmp_scalars.size() + count, lb);
        }
        if(auto ub = params.upper_bound.value_or(COPT_INFINITY);
           ub < COPT_INFINITY) {
            dbl_offset_2.emplace(tmp_scalars.size());
            tmp_scalars.resize(tmp_scalars.size() + count, ub);
        }
        tmp_types.resize(count);
        std::fill(tmp_types.begin(), tmp_types.end(), type);
        check(COPT->AddCols(
            prob, static_cast<int>(count), tmp_scalars.data(), nullptr, nullptr,
            nullptr, nullptr, tmp_types.data(),
            dbl_offset_1.has_value()
                ? (tmp_scalars.data() +
                   static_cast<std::ptrdiff_t>(dbl_offset_1.value()))
                : nullptr,
            dbl_offset_2.has_value()
                ? (tmp_scalars.data() +
                   static_cast<std::ptrdiff_t>(dbl_offset_2.value()))
                : nullptr,
            nullptr));
    }

public:
    variable add_variable(
        const variable_params params = default_variable_params) {
        return _add_variable(params, COPT_CONTINUOUS);
    }
    auto add_variables(std::size_t count,
                       variable_params params = default_variable_params) {
        const std::size_t offset = num_variables();
        _add_variables(count, params, COPT_CONTINUOUS);
        return _make_variables_view(offset, count);
    }
    template <typename IL>
    auto add_variables(std::size_t count, IL && id_lambda,
                       variable_params params = default_variable_params) {
        const std::size_t offset = num_variables();
        _add_variables(count, params, COPT_CONTINUOUS);
        return _make_indexed_variables_view(offset, count,
                                            std::forward<IL>(id_lambda));
    }

    variable add_named_variable(
        const std::string & name,
        const variable_params params = default_variable_params) {
        return _add_variable(params, COPT_CONTINUOUS, name.c_str());
    }
    template <typename NL>
    auto add_named_variables(std::size_t count, NL && name_lambda,
                             variable_params params = default_variable_params) {
        const std::size_t offset = num_variables();
        _add_variables(count, params, COPT_CONTINUOUS);
        return _make_named_variables_view(offset, count,
                                          std::forward<NL>(name_lambda), this);
    }
    template <typename IL, typename NL>
    auto add_named_variables(std::size_t count, IL && id_lambda,
                             NL && name_lambda,
                             variable_params params = default_variable_params) {
        const std::size_t offset = num_variables();
        _add_variables(count, params, COPT_CONTINUOUS);
        return _make_indexed_named_variables_view(
            offset, count, std::forward<IL>(id_lambda),
            std::forward<NL>(name_lambda), this);
    }

private:
    template <typename ER>
    inline variable _add_column(ER && entries, const variable_params & params,
                                const char & type) {
        const int var_id = static_cast<int>(num_variables());
        _reset_cache();
        _register_constraints_entries<true>(entries);
        check(COPT->AddCol(
            prob, params.obj_coef, static_cast<int>(tmp_indices.size()),
            tmp_indices.data(), tmp_scalars.data(), type,
            params.lower_bound.value_or(-COPT_INFINITY),
            params.upper_bound.value_or(+COPT_INFINITY), nullptr));
        return variable(var_id);
    }

public:
    template <std::ranges::range ER>
    variable add_column(
        ER && entries, const variable_params params = default_variable_params) {
        return _add_column(entries, params, COPT_CONTINUOUS);
    }
    variable add_column(
        std::initializer_list<std::pair<constraint, scalar>> entries,
        const variable_params params = default_variable_params) {
        return _add_column(entries, params, COPT_CONTINUOUS);
    }

    void set_objective_coefficient(variable v, scalar c) {
        const int id = v.id();
        check(COPT->SetColObj(prob, 1, &id, &c));
    }
    void set_variable_lower_bound(variable v, scalar lb) {
        const int id = v.id();
        check(COPT->SetColLower(prob, 1, &id, &lb));
    }
    void set_variable_upper_bound(variable v, scalar ub) {
        const int id = v.id();
        check(COPT->SetColUpper(prob, 1, &id, &ub));
    }
    void set_variable_name(variable v, const std::string & name) {
        const int id = v.id();
        const char * c_str = name.c_str();
        check(COPT->SetColNames(prob, 1, &id, &c_str));
    }

    scalar get_objective_coefficient(variable v) {
        scalar coef;
        const int id = v.id();
        check(COPT->GetColInfo(prob, COPT_DBLINFO_OBJ, 1, &id, &coef));
        return coef;
    }
    scalar get_variable_lower_bound(variable v) {
        scalar lb;
        const int id = v.id();
        check(COPT->GetColInfo(prob, COPT_DBLINFO_LB, 1, &id, &lb));
        return lb;
    }
    scalar get_variable_upper_bound(variable v) {
        scalar ub;
        const int id = v.id();
        check(COPT->GetColInfo(prob, COPT_DBLINFO_UB, 1, &id, &ub));
        return ub;
    }
    std::string get_variable_name(variable v) {
        int size;
        check(COPT->GetColName(prob, v.id(), nullptr, 0, &size));
        std::string name(static_cast<std::size_t>(size), '\0');
        check(COPT->GetColName(prob, v.id(), name.data(), size, nullptr));
        name.pop_back();
        return name;
    }

private:
    template <bool distinct, linear_constraint LC>
    constraint _add_constraint(LC && lc) {
        auto constr_id = static_cast<index>(num_constraints());
        if constexpr(!distinct) _prepare_coalescing(num_variables());
        _reset_cache();
        _register_variables_entries<distinct>(lc.linear_terms());
        const scalar b = lc.rhs();
        check(COPT->AddRow(prob, static_cast<int>(tmp_indices.size()),
                           tmp_indices.data(), tmp_scalars.data(),
                           constraint_sense_to_copt_sense(lc.sense()), b,
                           COPT_INFINITY, nullptr));
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
        tmp_types.emplace_back(constraint_sense_to_copt_sense(lc.sense()));
        tmp_rhs.emplace_back(lc.rhs());
        _register_variables_entries<distinct>(lc.linear_terms());
    }
    template <bool distinct, typename Key, typename LastConstrLambda>
        requires linear_constraint<std::invoke_result_t<LastConstrLambda, Key>>
    void _register_first_valued_constraint(const Key & key,
                                           LastConstrLambda & lc_lambda) {
        _register_constraint<distinct>(lc_lambda(key));
    }
    template <bool distinct, typename Key, typename OptConstrLambda,
              typename... Tail>
        requires detail::optional_type<
                     std::invoke_result_t<OptConstrLambda, Key>> &&
                 linear_constraint<detail::optional_type_value_t<
                     std::invoke_result_t<OptConstrLambda, Key>>>
    void _register_first_valued_constraint(const Key & key,
                                           OptConstrLambda & opt_lc_lambda,
                                           Tail &... tail) {
        if(const auto & opt_lc = opt_lc_lambda(key)) {
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
        tmp_types.resize(0);
        tmp_rhs.resize(0);
        const index offset = static_cast<index>(num_constraints());
        index count = 0;
        for(auto && key : keys) {
            _register_first_valued_constraint<distinct>(key,
                                                        constraint_lambdas...);
            ++count;
        }
        tmp_begins.emplace_back(static_cast<index>(tmp_indices.size()));
        check(COPT->AddRows(prob, static_cast<int>(tmp_rhs.size()),
                            tmp_begins.data(), nullptr, tmp_indices.data(),
                            tmp_scalars.data(), tmp_types.data(),
                            tmp_rhs.data(), nullptr, nullptr));
        return constraints_range(
            std::forward<IR>(keys),
            std::views::transform(std::views::iota(offset, offset + count),
                                  [](auto && i) { return constraint{i}; }));
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

}  // namespace copt::v7_2
}  // namespace mippp
