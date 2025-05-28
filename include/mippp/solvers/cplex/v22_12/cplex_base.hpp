#ifndef MIPPP_CPLEX_v22_12_BASE_MODEL_HPP
#define MIPPP_CPLEX_v22_12_BASE_MODEL_HPP

#include <memory>
#include <numeric>
#include <optional>
#include <vector>

#include <range/v3/view/iota.hpp>
#include <range/v3/view/span.hpp>
#include <range/v3/view/zip.hpp>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/cplex/v22_12/cplex_api.hpp"
#include "mippp/solvers/model_base.hpp"

namespace fhamonic::mippp {
namespace cplex::v22_12 {

class cplex_base : public model_base<int, double> {
public:
    using variable_id = int;
    using constraint_id = int;
    using scalar = double;
    using variable = model_variable<variable_id, scalar>;
    using constraint = model_constraint<constraint_id>;
    template <typename Map>
    using variable_mapping = entity_mapping<variable, Map>;
    template <typename Map>
    using constraint_mapping = entity_mapping<constraint, Map>;

protected:
    int cplex_status;
    const cplex_api & CPX;
    CPXENVptr env;
    CPXLPptr lp;

    std::vector<int> tmp_begins;
    std::vector<char> tmp_types;
    std::vector<double> tmp_rhs;

    static void check(int error) {
        if(error == 0) return;
        throw std::runtime_error("CPLEX: error " + std::to_string(error));
    }

    static constexpr char constraint_sense_to_cplex_sense(
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
    [[nodiscard]] explicit cplex_base(const cplex_api & api)
        : model_base<int, double>()
        , CPX(api)
        , env(CPX.openCPLEX(&cplex_status))
        , lp(CPX.createprob(env, &cplex_status, "cplex_base")) {}
    ~cplex_base() { check(CPX.freeprob(env, &lp)); }

    std::size_t num_variables() {
        return static_cast<std::size_t>(CPX.getnumcols(env, lp));
    }
    std::size_t num_constraints() {
        return static_cast<std::size_t>(CPX.getnumrows(env, lp));
    }
    std::size_t num_entries() {
        return static_cast<std::size_t>(CPX.getnumnz(env, lp));
    }

    void set_maximization() { check(CPX.chgobjsen(env, lp, CPX_MAX)); }
    void set_minimization() { check(CPX.chgobjsen(env, lp, CPX_MIN)); }

    void set_objective_offset(double constant) {
        check(CPX.chgobjoffset(env, lp, constant));
    }

    void set_objective(linear_expression auto && le) {
        auto num_vars = num_variables();
        tmp_indices.resize(num_vars);
        std::iota(tmp_indices.begin(), tmp_indices.end(), 0);
        tmp_scalars.resize(num_vars);
        std::fill(tmp_scalars.begin(), tmp_scalars.end(), 0.0);
        for(auto && [var, coef] : le.linear_terms()) {
            tmp_scalars[var.uid()] += coef;
        }
        check(CPX.chgobj(env, lp, static_cast<int>(num_vars),
                         tmp_indices.data(), tmp_scalars.data()));
        set_objective_offset(le.constant());
    }
    void add_objective(linear_expression auto && le) {
        auto num_vars = num_variables();
        tmp_indices.resize(num_vars);
        std::iota(tmp_indices.begin(), tmp_indices.end(), 0);
        tmp_scalars.resize(num_vars);
        check(CPX.getobj(env, lp, tmp_scalars.data(), 0,
                         static_cast<int>(num_vars) - 1));
        for(auto && [var, coef] : le.linear_terms()) {
            tmp_scalars[var.uid()] += coef;
        }
        check(CPX.chgobj(env, lp, static_cast<int>(num_vars),
                         tmp_indices.data(), tmp_scalars.data()));
        set_objective_offset(get_objective_offset() + le.constant());
    }

    double get_objective_offset() {
        double objective_offset;
        check(CPX.getobjoffset(env, lp, &objective_offset));
        return objective_offset;
    }
    auto get_objective() {
        const auto num_vars = num_variables();
        auto coefs = std::make_shared_for_overwrite<double[]>(num_vars);
        check(CPX.getobj(env, lp, coefs.get(), 0,
                         static_cast<int>(num_vars) - 1));
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
    variable _add_variable(const variable_params & params, char type) {
        const int var_id = static_cast<int>(num_variables());
        const double lb = params.lower_bound.value_or(-CPX_INFBOUND);
        const double ub = params.upper_bound.value_or(CPX_INFBOUND);
        check(CPX.newcols(env, lp, 1, &params.obj_coef, &lb, &ub,
                          (type != CPX_CONTINUOUS) ? &type : NULL, NULL));
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
        if(auto lb = params.lower_bound.value_or(-CPX_INFBOUND); lb != 0.0) {
            dbl_offset_2.emplace(tmp_scalars.size());
            tmp_scalars.resize(tmp_scalars.size() + count, lb);
        }
        if(auto ub = params.upper_bound.value_or(CPX_INFBOUND); ub != 0.0) {
            dbl_offset_3.emplace(tmp_scalars.size());
            tmp_scalars.resize(tmp_scalars.size() + count, ub);
        }

        if(type != CPX_CONTINUOUS) {
            tmp_types.resize(count);
            std::fill(tmp_types.begin(), tmp_types.end(), type);
        }

        check(CPX.newcols(
            env, lp, static_cast<int>(count),
            dbl_offset_1.has_value()
                ? (tmp_scalars.data() +
                   static_cast<std::ptrdiff_t>(dbl_offset_1.value()))
                : NULL,
            dbl_offset_2.has_value()
                ? (tmp_scalars.data() +
                   static_cast<std::ptrdiff_t>(dbl_offset_2.value()))
                : NULL,
            dbl_offset_3.has_value()
                ? (tmp_scalars.data() +
                   static_cast<std::ptrdiff_t>(dbl_offset_3.value()))
                : NULL,
            (type != CPX_CONTINUOUS) ? tmp_types.data() : NULL, NULL));
    }

public:
    variable add_variable(
        const variable_params params = default_variable_params) {
        return _add_variable(params, CPX_CONTINUOUS);
    }
    auto add_variables(std::size_t count,
                       variable_params params = {
                           .obj_coef = 0,
                           .lower_bound = 0,
                           .upper_bound = std::nullopt}) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params, CPX_CONTINUOUS);
        return _make_variables_range(offset, count);
    }
    template <typename IL>
    auto add_variables(std::size_t count, IL && id_lambda,
                       variable_params params = {
                           .obj_coef = 0,
                           .lower_bound = 0,
                           .upper_bound = std::nullopt}) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params, CPX_CONTINUOUS);
        return _make_indexed_variables_range(offset, count,
                                             std::forward<IL>(id_lambda));
    }

private:
    template <typename ER>
    inline variable _add_column(ER && entries, const variable_params & params) {
        const int var_id = static_cast<int>(num_variables());
        const int cmatbeg = 0;
        _reset_cache(num_constraints());
        _register_raw_entries(entries);
        const double lb = params.lower_bound.value_or(-CPX_INFBOUND);
        const double ub = params.upper_bound.value_or(CPX_INFBOUND);
        check(CPX.addcols(env, lp, 1, static_cast<int>(tmp_indices.size()),
                          &params.obj_coef, &cmatbeg, tmp_indices.data(),
                          tmp_scalars.data(), &lb, &ub, NULL));
        return variable(var_id);
    }

public:
    template <ranges::range ER>
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
        check(CPX.chgobj(env, lp, 1, &var_id, &c));
    }
    void set_variable_lower_bound(variable v, double lb) noexcept {
        int var_id = v.id();
        char lu = 'L';
        check(CPX.chgbds(env, lp, 1, &var_id, &lu, &lb));
    }
    void set_variable_upper_bound(variable v, double ub) noexcept {
        int var_id = v.id();
        char lu = 'U';
        check(CPX.chgbds(env, lp, 1, &var_id, &lu, &ub));
    }

    double get_objective_coefficient(variable v) {
        double coef;
        check(CPX.getobj(env, lp, &coef, v.id(), v.id()));
        return coef;
    }
    double get_variable_lower_bound(variable v) noexcept {
        double b;
        check(CPX.getlb(env, lp, &b, v.id(), v.id()));
        return b;
    }
    double get_variable_upper_bound(variable v) noexcept {
        double b;
        check(CPX.getub(env, lp, &b, v.id(), v.id()));
        return b;
    }

    constraint add_constraint(linear_constraint auto && lc) {
        int constr_id = static_cast<int>(num_constraints());
        _reset_cache(num_variables());
        _register_entries(lc.linear_terms());
        int matbegin = 0;
        const double b = lc.rhs();
        const char sense = constraint_sense_to_cplex_sense(lc.sense());
        check(CPX.addrows(env, lp, 0, 1, static_cast<int>(tmp_indices.size()),
                          &b, &sense, &matbegin, tmp_indices.data(),
                          tmp_scalars.data(), NULL, NULL));
        return constraint(constr_id);
    }

private:
    template <linear_constraint LC>
    void _register_constraint(const int & constr_id, const LC & lc) {
        tmp_begins.emplace_back(static_cast<int>(tmp_indices.size()));
        tmp_types.emplace_back(constraint_sense_to_cplex_sense(lc.sense()));
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
    template <ranges::range IR, typename... CL>
    auto add_constraints(IR && keys, CL... constraint_lambdas) {
        _reset_cache(num_variables());
        tmp_begins.resize(0);
        tmp_types.resize(0);
        tmp_rhs.resize(0);
        const int offset = static_cast<int>(num_constraints());
        int constr_id = offset;
        for(auto && key : keys) {
            _register_first_valued_constraint(constr_id, key,
                                              constraint_lambdas...);
            ++constr_id;
        }
        check(CPX.addrows(env, lp, 0, static_cast<int>(tmp_begins.size()),
                          static_cast<int>(tmp_indices.size()), tmp_rhs.data(),
                          tmp_types.data(), tmp_begins.data(),
                          tmp_indices.data(), tmp_scalars.data(), NULL, NULL));
        return constraints_range(
            keys,
            ranges::view::transform(ranges::view::iota(offset, constr_id),
                                    [](auto && i) { return constraint{i}; }));
    }
};

}  // namespace cplex::v22_12
}  // namespace fhamonic::mippp

#endif  // MIPPP_CPLEX_v22_12_BASE_MODEL_HPP