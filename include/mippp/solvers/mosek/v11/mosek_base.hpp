#ifndef MIPPP_MOSEK_v11_BASE_MODEL_HPP
#define MIPPP_MOSEK_v11_BASE_MODEL_HPP

#include <filesystem>
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
#include "mippp/solvers/mosek/v11/mosek_api.hpp"

namespace fhamonic::mippp {
namespace mosek::v11 {

class mosek_base : public model_base<int, double> {
public:
    using indice = MSKint32t;
    using variable_id = MSKint32t;
    using constraint_id = MSKint32t;
    using scalar = MSKrealt;
    using variable = model_variable<variable_id, scalar>;
    using constraint = model_constraint<constraint_id>;
    template <typename Map>
    using variable_mapping = entity_mapping<variable, Map>;
    template <typename Map>
    using constraint_mapping = entity_mapping<constraint, Map>;

protected:
    const mosek_api & MSK;
    MSKenv_t env;
    MSKtask_t task;

    std::vector<indice> tmp_begins;
    std::vector<MSKboundkeye> tmp_boundkeye;
    std::vector<scalar> tmp_rhs;
    std::vector<MSKvariabletypee> tmp_vartype;

    void check(MSKrescodee error) const {
        if(error == 0) return;
        char str[MSK_MAX_STR_LEN];
        MSK.getcodedesc(error, NULL, str);
        throw std::runtime_error(str);
    }

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
    [[nodiscard]] explicit mosek_base(const mosek_api & api)
        : model_base<int, double>(), MSK(api), env(NULL), task(NULL) {
        check(MSK.makeenv(
            &env, (std::filesystem::temp_directory_path() / "mosek_").c_str()));
        check(MSK.makeemptytask(env, &task));
    }
    ~mosek_base() {
        check(MSK.deletetask(&task));
        check(MSK.deleteenv(&env));
    }

    std::size_t num_variables() {
        MSKint32t num;
        check(MSK.getnumvar(task, &num));
        return static_cast<std::size_t>(num);
    }
    std::size_t num_constraints() {
        MSKint32t num;
        check(MSK.getnumcon(task, &num));
        return static_cast<std::size_t>(num);
    }
    std::size_t num_entries() {
        MSKint32t num;
        check(MSK.getnumanz(task, &num));
        return static_cast<std::size_t>(num);
    }

    void set_maximization() {
        check(MSK.putobjsense(task, MSK_OBJECTIVE_SENSE_MAXIMIZE));
    }
    void set_minimization() {
        check(MSK.putobjsense(task, MSK_OBJECTIVE_SENSE_MINIMIZE));
    }

    void set_objective_offset(scalar constant) {
        check(MSK.putcfix(task, constant));
    }
    void set_objective(linear_expression auto && le) {
        auto num_vars = num_variables();
        tmp_scalars.resize(num_vars);
        std::fill(tmp_scalars.begin(), tmp_scalars.end(), 0.0);
        for(auto && [var, coef] : le.linear_terms()) {
            tmp_scalars[var.uid()] += coef;
        }
        check(MSK.putcslice(task, 0, static_cast<indice>(num_vars),
                            tmp_scalars.data()));
        set_objective_offset(le.constant());
    }
    void add_objective(linear_expression auto && le) {
        auto num_vars = num_variables();
        tmp_scalars.resize(num_vars);
        check(MSK.getc(task, tmp_scalars.data()));
        for(auto && [var, coef] : le.linear_terms()) {
            tmp_scalars[var.uid()] += coef;
        }
        check(MSK.putcslice(task, 0, static_cast<indice>(num_vars),
                            tmp_scalars.data()));
        set_objective_offset(get_objective_offset() + le.constant());
    }

    scalar get_objective_offset() {
        scalar objective_offset;
        check(MSK.getcfix(task, &objective_offset));
        return objective_offset;
    }
    auto get_objective() {
        const auto num_vars = num_variables();
        auto coefs = std::make_shared_for_overwrite<double[]>(num_vars);
        check(MSK.getc(task, coefs.get()));
        return linear_expression_view(
           std::views::transform(
               std::views::iota(variable_id{0},
                                   static_cast<variable_id>(num_vars)),
                [coefs = std::move(coefs)](auto i) {
                    return std::make_pair(variable(i), coefs[i]);
                }),
            get_objective_offset());
    }

protected:
    void _add_variable(const int & var_id, const variable_params & params,
                       const MSKvariabletypee type) {
        check(MSK.appendvars(task, 1));
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
        check(MSK.putvarbound(task, var_id, boundkey, lb, ub));
        check(MSK.putcj(task, var_id, params.obj_coef));

        if(type != MSK_VAR_TYPE_CONT) {
            check(MSK.putvartype(task, var_id, type));
        }
    }
    void _add_variables(std::size_t offset, std::size_t count,
                        const variable_params & params,
                        const MSKvariabletypee type) {
        check(MSK.appendvars(task, static_cast<int>(count)));
        if(auto obj = params.obj_coef; obj != 0.0) {
            tmp_scalars.resize(count);
            std::fill(tmp_scalars.begin(), tmp_scalars.end(), obj);
            check(MSK.putcslice(task, static_cast<variable_id>(offset),
                                static_cast<variable_id>(offset + count),
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
        check(MSK.putvarboundsliceconst(
            task, static_cast<variable_id>(offset),
            static_cast<variable_id>(offset + count), boundkey, lb, ub));

        if(type != MSK_VAR_TYPE_CONT) {
            tmp_indices.resize(count);
            std::iota(tmp_indices.begin(), tmp_indices.end(),
                      static_cast<variable_id>(offset));
            tmp_vartype.resize(count);
            std::fill(tmp_vartype.begin(), tmp_vartype.end(), type);
            check(MSK.putvartypelist(task, static_cast<indice>(count),
                                     tmp_indices.data(), tmp_vartype.data()));
        }
    }

public:
    variable add_variable(
        const variable_params params = default_variable_params) {
        int var_id = static_cast<int>(num_variables());
        _add_variable(var_id, params, MSK_VAR_TYPE_CONT);
        return variable(var_id);
    }
    auto add_variables(
        std::size_t count,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params, MSK_VAR_TYPE_CONT);
        return _make_variables_range(offset, count);
    }
    template <typename IL>
    auto add_variables(
        std::size_t count, IL && id_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params, MSK_VAR_TYPE_CONT);
        return _make_indexed_variables_range(offset, count,
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
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params, MSK_VAR_TYPE_CONT);
        return _make_named_variables_range(offset, count,
                                           std::forward<NL>(name_lambda), this);
    }
    template <typename IL, typename NL>
    auto add_named_variables(
        std::size_t count, IL && id_lambda, NL && name_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params, MSK_VAR_TYPE_CONT);
        return _make_indexed_named_variables_range(
            offset, count, std::forward<IL>(id_lambda),
            std::forward<NL>(name_lambda), this);
    }

private:
    template <typename ER>
    inline variable _add_column(ER && entries, const variable_params & params) {
        const int var_id = static_cast<int>(num_variables());
        _add_variable(var_id, params, MSK_VAR_TYPE_CONT);
        _reset_cache(num_constraints());
        _register_raw_entries(entries);
        check(MSK.putacol(task, var_id, static_cast<int>(tmp_indices.size()),
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
        check(MSK.putcj(task, v.id(), c));
    }
    void set_variable_lower_bound(variable v, scalar lb) {
        check(MSK.chgvarbound(task, v.id(), 1, 1, lb));
    }
    void set_variable_upper_bound(variable v, scalar ub) {
        check(MSK.chgvarbound(task, v.id(), 0, 1, ub));
    }
    void set_variable_name(variable v, const std::string & name) {
        check(MSK.putvarname(task, v.id(), name.c_str()));
    }

    scalar get_objective_coefficient(variable v) {
        scalar coef;
        check(MSK.getcj(task, v.id(), &coef));
        return coef;
    }
    scalar get_variable_lower_bound(variable v) {
        MSKboundkeye boundkey;
        scalar lb, ub;
        check(MSK.getvarbound(task, v.id(), &boundkey, &lb, &ub));
        return lb;
    }
    scalar get_variable_upper_bound(variable v) {
        MSKboundkeye boundkey;
        scalar lb, ub;
        check(MSK.getvarbound(task, v.id(), &boundkey, &lb, &ub));
        return ub;
    }
    std::string get_variable_name(variable v) {
        MSKint32t len;
        check(MSK.getvarnamelen(task, v.id(), &len));
        std::string name(static_cast<std::size_t>(len + 1), '\0');
        check(MSK.getvarname(task, v.id(), len + 1, name.data()));
        name.pop_back();
        return name;
    }

    constraint add_constraint(linear_constraint auto && lc) {
        auto constr_id = static_cast<constraint_id>(num_constraints());
        check(MSK.appendcons(task, 1));
        _reset_cache(num_variables());
        _register_entries(lc.linear_terms());
        check(MSK.putarow(task, constr_id,
                          static_cast<indice>(tmp_indices.size()),
                          tmp_indices.data(), tmp_scalars.data()));
        const scalar b = lc.rhs();
        check(MSK.putconbound(task, constr_id,
                              constraint_sense_to_mosek_sense(lc.sense()), b,
                              b));
        return constraint(constr_id);
    }

private:
    template <linear_constraint LC>
    void _register_constraint(const int & constr_id, const LC & lc) {
        tmp_begins.emplace_back(static_cast<indice>(tmp_indices.size()));
        tmp_boundkeye.emplace_back(constraint_sense_to_mosek_sense(lc.sense()));
        tmp_rhs.emplace_back(lc.rhs());
        _register_entries(lc.linear_terms());
    }
    template <typename Key, typename LastConstrLambda>
        requires linear_constraint<std::invoke_result_t<LastConstrLambda, Key>>
    void _register_first_valued_constraint(const int & constr_id,
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
        const int & constr_id, const Key & key,
        const OptConstrLambda & opt_lc_lambda, const Tail &... tail) {
        if(const auto & opt_lc = opt_lc_lambda(key)) {
            _register_constraint(constr_id, opt_lc.value());
            return;
        }
        _register_first_valued_constraint(constr_id, key, tail...);
    }

public:
    template <std::ranges::range IR, typename... CL>
    auto add_constraints(IR && keys, CL... constraint_lambdas) {
        _reset_cache(num_variables());
        tmp_begins.resize(0);
        tmp_boundkeye.resize(0);
        tmp_rhs.resize(0);
        const indice offset = static_cast<indice>(num_constraints());
        indice constr_id = offset;
        for(auto && key : keys) {
            _register_first_valued_constraint(constr_id, key,
                                              constraint_lambdas...);
            ++constr_id;
        }
        check(MSK.appendcons(task, constr_id - offset));
        tmp_begins.emplace_back(static_cast<indice>(tmp_indices.size()));
        check(MSK.putarowslice(task, offset, constr_id, tmp_begins.data(),
                               tmp_begins.data() + 1, tmp_indices.data(),
                               tmp_scalars.data()));
        check(MSK.putconboundslice(task, offset, constr_id,
                                   tmp_boundkeye.data(), tmp_rhs.data(),
                                   tmp_rhs.data()));
        return constraints_range(
            keys,
           std::views::transform(std::views::iota(offset, constr_id),
                                    [](auto && i) { return constraint{i}; }));
    }
};

}  // namespace mosek::v11
}  // namespace fhamonic::mippp

#endif  // MIPPP_MOSEK_v11_BASE_MODEL_HPP