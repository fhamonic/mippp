#pragma once

#include <memory>
#include <numeric>
#include <optional>
#include <ranges>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/model_base.hpp"
#include "mippp/solvers/xpress/v45_1/xpress_api.hpp"

namespace mippp {
namespace xpress::v45_1 {

class xpress_base : public model_base<int, double> {
public:
    using variable_id = int;
    using constraint_id = int;
    using scalar = double;
    using variable = model_variable<variable_id, scalar>;
    using constraint = model_constraint<constraint_id>;
    template <typename Map>
    struct variable_mapping : entity_mapping<variable, Map> {
        variable_mapping(Map && t)
            : entity_mapping<variable, Map>(std::move(t)) {}
    };
    template <typename Map>
    struct constraint_mapping : entity_mapping<constraint, Map> {
        constraint_mapping(Map && t)
            : entity_mapping<constraint, Map>(std::move(t)) {}
    };

protected:
    const xpress_api * XPRS;
    XPRSprob prob;
    double objective_offset;

    std::vector<int> tmp_begins;
    std::vector<char> tmp_types;
    std::vector<double> tmp_rhs;

    void check(const int error) { XPRS->_check(prob, error); }
    static constexpr char constraint_sense_to_xpress_sense(
        constraint_sense rel) {
        if(rel == constraint_sense::less_equal) return 'L';
        if(rel == constraint_sense::equal) return 'E';
        return 'G';
    }
    // static constexpr constraint_sense mosek_sense_to_constraint_sense(
    //     CPXboundkeye sense) {
    //     if(sense == CPX_BK_UP) return constraint_sense::less_equal;
    //     if(sense == CPX_BK_FX) return constraint_sense::equal;
    //     return constraint_sense::greater_equal;
    // }

public:
    [[nodiscard]] explicit xpress_base(const xpress_api & api)
        : model_base<int, double>(), XPRS(&api), objective_offset(0.0) {
        check(XPRS->createprob(&prob));
    }
    ~xpress_base() {
        if(!prob) return;
        check(XPRS->destroyprob(prob));
    }

    constexpr xpress_base(const xpress_base &) = delete;
    constexpr xpress_base(xpress_base && other) noexcept
        : model_base<int, double>(std::move(other))
        , XPRS(other.XPRS)
        , prob(other.prob)
        , objective_offset(other.objective_offset)
        , tmp_begins(std::move(other.tmp_begins))
        , tmp_types(std::move(other.tmp_types))
        , tmp_rhs(std::move(other.tmp_rhs)) {
        other.prob = nullptr;
    }

    constexpr xpress_base & operator=(const xpress_base &) = delete;
    constexpr xpress_base & operator=(xpress_base && other) = delete;

    std::size_t num_variables() {
        int num_vars;
        check(XPRS->getintattrib(prob, XPRS_COLS, &num_vars));
        return static_cast<std::size_t>(num_vars);
    }
    std::size_t num_constraints() {
        int num_constrs;
        check(XPRS->getintattrib(prob, XPRS_ROWS, &num_constrs));
        return static_cast<std::size_t>(num_constrs);
    }
    std::size_t num_entries() {
        int num_entries;
        check(XPRS->getintattrib(prob, XPRS_ELEMS, &num_entries));
        return static_cast<std::size_t>(num_entries);
    }
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// Objective ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void set_maximization() {
        check(XPRS->chgobjsense(prob, XPRS_OBJ_MAXIMIZE));
    }
    void set_minimization() {
        check(XPRS->chgobjsense(prob, XPRS_OBJ_MINIMIZE));
    }

    void set_objective_offset(double constant) { objective_offset = constant; }

    void set_objective(linear_expression auto && le) {
        auto num_vars = num_variables();
        tmp_indices.resize(num_vars);
        std::iota(tmp_indices.begin(), tmp_indices.end(), 0);
        tmp_scalars.resize(num_vars);
        std::fill(tmp_scalars.begin(), tmp_scalars.end(), 0.0);
        for(auto && [var, coef] : le.linear_terms()) {
            tmp_scalars[var.uid()] += coef;
        }
        check(XPRS->chgobj(prob, static_cast<int>(num_vars), tmp_indices.data(),
                           tmp_scalars.data()));
        set_objective_offset(le.constant());
    }
    template <linear_expression LE>
    void set_objective(distinct_variables_t, LE && le) {
        set_objective(std::forward<LE>(le));
    }
    void add_objective(linear_expression auto && le) {
        auto num_vars = num_variables();
        tmp_indices.resize(num_vars);
        std::iota(tmp_indices.begin(), tmp_indices.end(), 0);
        tmp_scalars.resize(num_vars);
        std::fill(tmp_scalars.begin(), tmp_scalars.end(), 0.0);
        check(XPRS->getobj(prob, tmp_scalars.data(), 0,
                           static_cast<int>(num_vars) - 1));
        for(auto && [var, coef] : le.linear_terms()) {
            tmp_scalars[var.uid()] += coef;
        }
        check(XPRS->chgobj(prob, static_cast<int>(num_vars), tmp_indices.data(),
                           tmp_scalars.data()));
        set_objective_offset(get_objective_offset() + le.constant());
    }

    double get_objective_offset() { return objective_offset; }
    auto get_objective() {
        const auto num_vars = num_variables();
        auto coefs = std::make_shared_for_overwrite<double[]>(num_vars);
        check(
            XPRS->getobj(prob, coefs.get(), 0, static_cast<int>(num_vars) - 1));
        return linear_expression_view(
            std::views::transform(
                std::views::iota(variable_id{0},
                                 static_cast<variable_id>(num_vars)),
                [coefs = std::move(coefs)](auto i) {
                    return std::make_pair(variable(i), coefs[i]);
                }),
            get_objective_offset());
    }
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// Variables ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
protected:
    variable _add_variable(const variable_params & params, char type) {
        const int var_id = static_cast<int>(num_variables());
        const double lb = params.lower_bound.value_or(XPRS_MINUSINFINITY);
        const double ub = params.upper_bound.value_or(XPRS_PLUSINFINITY);
        check(XPRS->addcols(prob, 1, 0, &params.obj_coef, nullptr, nullptr,
                            nullptr, &lb, &ub));
        if(type != 'C') check(XPRS->chgcoltype(prob, 1, &var_id, &type));
        return variable(var_id);
    }
    void _add_variables(std::size_t offset, std::size_t count,
                        const variable_params & params, char type) {
        std::optional<std::size_t> dbl_offset_1, dbl_offset_2, dbl_offset_3;
        tmp_scalars.resize(0u);
        if(params.obj_coef != 0.0) {
            dbl_offset_1.emplace(tmp_scalars.size());
            tmp_scalars.resize(tmp_scalars.size() + count, params.obj_coef);
        }
        if(auto lb = params.lower_bound.value_or(XPRS_MINUSINFINITY);
           lb != 0.0) {
            dbl_offset_2.emplace(tmp_scalars.size());
            tmp_scalars.resize(tmp_scalars.size() + count, lb);
        }
        if(auto ub = params.upper_bound.value_or(XPRS_PLUSINFINITY);
           ub < XPRS_PLUSINFINITY) {
            dbl_offset_3.emplace(tmp_scalars.size());
            tmp_scalars.resize(tmp_scalars.size() + count, ub);
        }
        check(XPRS->addcols(
            prob, static_cast<int>(count), 0,
            dbl_offset_1.has_value()
                ? (tmp_scalars.data() +
                   static_cast<std::ptrdiff_t>(dbl_offset_1.value()))
                : nullptr,
            nullptr, nullptr, nullptr,
            dbl_offset_2.has_value()
                ? (tmp_scalars.data() +
                   static_cast<std::ptrdiff_t>(dbl_offset_2.value()))
                : nullptr,
            dbl_offset_3.has_value()
                ? (tmp_scalars.data() +
                   static_cast<std::ptrdiff_t>(dbl_offset_3.value()))
                : nullptr));

        if(type != 'C') {
            tmp_indices.resize(count);
            std::iota(tmp_indices.begin(), tmp_indices.end(), offset);
            tmp_types.resize(count);
            std::fill(tmp_types.begin(), tmp_types.end(), type);
            check(XPRS->chgcoltype(prob, static_cast<int>(count),
                                   tmp_indices.data(), tmp_types.data()));
        }
    }

public:
    variable add_variable(
        const variable_params params = default_variable_params) {
        return _add_variable(params, 'C');
    }
    auto add_variables(std::size_t count,
                       variable_params params = {
                           .obj_coef = 0,
                           .lower_bound = 0,
                           .upper_bound = std::nullopt}) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params, 'C');
        return _make_variables_view(offset, count);
    }
    template <typename IL>
    auto add_variables(std::size_t count, IL && id_lambda,
                       variable_params params = {
                           .obj_coef = 0,
                           .lower_bound = 0,
                           .upper_bound = std::nullopt}) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params, 'C');
        return _make_indexed_variables_view(offset, count,
                                            std::forward<IL>(id_lambda));
    }

    variable add_named_variable(
        const std::string & name,
        const variable_params params = default_variable_params) {
        auto var = _add_variable(params, 'C');
        set_variable_name(var, name);
        return var;
    }
    template <typename NL>
    auto add_named_variables(
        std::size_t count, NL && name_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params, 'C');
        return _make_named_variables_view(offset, count,
                                          std::forward<NL>(name_lambda), this);
    }
    template <typename IL, typename NL>
    auto add_named_variables(
        std::size_t count, IL && id_lambda, NL && name_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params, 'C');
        return _make_indexed_named_variables_view(
            offset, count, std::forward<IL>(id_lambda),
            std::forward<NL>(name_lambda), this);
    }

private:
    template <typename ER>
    inline variable _add_column(ER && entries, const variable_params & params) {
        const int var_id = static_cast<int>(num_variables());
        const int cmatbeg = 0;
        _reset_raw_cache();
        _register_raw_entries(entries);
        const double lb = params.lower_bound.value_or(XPRS_MINUSINFINITY);
        const double ub = params.upper_bound.value_or(XPRS_PLUSINFINITY);
        check(XPRS->addcols(prob, 1, static_cast<int>(tmp_indices.size()),
                            &params.obj_coef, &cmatbeg, tmp_indices.data(),
                            tmp_scalars.data(), &lb, &ub));
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

    void set_objective_coefficient(variable v, double c) {
        int var_id = v.id();
        check(XPRS->chgobj(prob, 1, &var_id, &c));
    }
    void set_variable_lower_bound(variable v, double lb) noexcept {
        int var_id = v.id();
        char bt = 'L';
        check(XPRS->chgbounds(prob, 1, &var_id, &bt, &lb));
    }
    void set_variable_upper_bound(variable v, double ub) noexcept {
        int var_id = v.id();
        char bt = 'U';
        check(XPRS->chgbounds(prob, 1, &var_id, &bt, &ub));
    }
    void set_variable_name(variable v, const std::string & name) noexcept {
        check(XPRS->addnames(prob, XPRS_NAMES_COLUMN, name.data(), v.id(),
                             v.id()));
    }

    double get_objective_coefficient(variable v) {
        double coef;
        check(XPRS->getobj(prob, &coef, v.id(), v.id()));
        return coef;
    }
    double get_variable_lower_bound(variable v) noexcept {
        double b;
        check(XPRS->getlb(prob, &b, v.id(), v.id()));
        return b;
    }
    double get_variable_upper_bound(variable v) noexcept {
        double b;
        check(XPRS->getub(prob, &b, v.id(), v.id()));
        return b;
    }
    std::string get_variable_name(variable v) noexcept {
        int nbytes;
        check(XPRS->getnamelist(prob, XPRS_NAMES_COLUMN, nullptr, 0, &nbytes,
                                v.id(), v.id()));
        std::string name(static_cast<std::size_t>(nbytes - 1), '\0');
        check(XPRS->getnamelist(prob, XPRS_NAMES_COLUMN, name.data(), nbytes,
                                nullptr, v.id(), v.id()));
        return name;
    }
    ///////////////////////////////////////////////////////////////////////////
    /////////////////////////////// Constraints ///////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    template <bool distinct, linear_constraint LC>
    constraint _add_constraint(LC && lc) {
        int constr_id = static_cast<int>(num_constraints());
        if constexpr(distinct) {
            _reset_raw_cache();
            _register_raw_entries(lc.linear_terms());
        } else {
            _reset_cache(num_variables());
            _register_entries(lc.linear_terms());
        }
        int matbegin = 0;
        const double b = lc.rhs();
        const char sense = constraint_sense_to_xpress_sense(lc.sense());
        check(XPRS->addrows(prob, 1, static_cast<int>(tmp_indices.size()),
                            &sense, &b, nullptr, &matbegin, tmp_indices.data(),
                            tmp_scalars.data()));
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
        tmp_types.emplace_back(constraint_sense_to_xpress_sense(lc.sense()));
        tmp_rhs.emplace_back(lc.rhs());
        if constexpr(distinct) {
            _register_raw_entries(lc.linear_terms());
        } else {
            _register_entries(lc.linear_terms());
        }
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
    auto _add_constraints(IR && keys, CL &&... constraint_lambdas) {
        if constexpr(distinct) {
            _reset_raw_cache();
        } else {
            _reset_cache(num_variables());
        }
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
        check(XPRS->addrows(prob, static_cast<int>(tmp_begins.size()),
                            static_cast<int>(tmp_indices.size()),
                            tmp_types.data(), tmp_rhs.data(), nullptr,
                            tmp_begins.data(), tmp_indices.data(),
                            tmp_scalars.data()));
        return constraints_range(
            std::forward<IR>(keys),
            std::views::transform(std::views::iota(offset, constr_id),
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
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////// Tolerance parameters ///////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void set_feasibility_tolerance(double tol) {
        check(XPRS->setdblcontrol(prob, XPRS_FEASTOL, tol));
    }
    double get_feasibility_tolerance() {
        double tol;
        check(XPRS->getdblcontrol(prob, XPRS_FEASTOL, &tol));
        return tol;
    }
    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////// Limits //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void set_time_limit(std::chrono::duration<double> t) {
        check(XPRS->setdblcontrol(prob, XPRS_TIMELIMIT, t.count()));
    }
    auto get_time_limit() {
        double t;
        check(XPRS->getdblcontrol(prob, XPRS_TIMELIMIT, &t));
        return std::chrono::duration<double>(t);
    }
};

}  // namespace xpress::v45_1
}  // namespace mippp
