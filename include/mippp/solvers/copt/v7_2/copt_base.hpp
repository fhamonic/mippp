#ifndef MIPPP_COPT_v7_2_BASE_HPP
#define MIPPP_COPT_v7_2_BASE_HPP

#include <memory>
#include <numeric>
#include <optional>
#include <vector>

#include <range/v3/view/iota.hpp>
#include <range/v3/view/transform.hpp>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/copt/v7_2/copt_api.hpp"
#include "mippp/solvers/model_base.hpp"

namespace fhamonic::mippp {
namespace copt::v7_2 {

class copt_base : public model_base<int, double> {
public:
    using indice = int;
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
    const copt_api & COPT;
    copt_env * env;
    copt_prob * prob;

    std::vector<indice> tmp_begins;
    std::vector<char> tmp_types;
    std::vector<scalar> tmp_rhs;

    static void check(ret_code error) {
        if(error == COPT_RETCODE_OK) return;
        throw std::runtime_error("copt_base error");
    }

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
    [[nodiscard]] explicit copt_base(const copt_api & api)
        : model_base<int, double>(), COPT(api), env(NULL), prob(NULL) {
        COPT.CreateEnv(&env);
        COPT.CreateProb(env, &prob);
    }
    ~copt_base() {
        COPT.DeleteProb(&prob);
        COPT.DeleteEnv(&env);
    }

    std::size_t num_variables() {
        int num;
        check(COPT.GetIntAttr(prob, COPT_INTATTR_COLS, &num));
        return static_cast<std::size_t>(num);
    }
    std::size_t num_constraints() {
        int num;
        check(COPT.GetIntAttr(prob, COPT_INTATTR_ROWS, &num));
        return static_cast<std::size_t>(num);
    }
    std::size_t num_entries() {
        int num;
        check(COPT.GetIntAttr(prob, COPT_INTATTR_ELEMS, &num));
        return static_cast<std::size_t>(num);
    }

    void set_maximization() { check(COPT.SetObjSense(prob, COPT_MAXIMIZE)); }
    void set_minimization() { check(COPT.SetObjSense(prob, COPT_MINIMIZE)); }

    void set_objective_offset(scalar constant) {
        check(COPT.SetObjConst(prob, constant));
    }
    void set_objective(linear_expression auto && le) {
        _reset_cache(num_variables());
        _register_entries(le.linear_terms());
        check(COPT.ReplaceColObj(prob, static_cast<int>(tmp_indices.size()),
                                 tmp_indices.data(), tmp_scalars.data()));
        set_objective_offset(le.constant());
    }

    scalar get_objective_offset() {
        scalar objective_offset;
        check(COPT.GetDblAttr(prob, COPT_DBLATTR_OBJCONST, &objective_offset));
        return objective_offset;
    }
    auto get_objective() {
        const auto num_vars = num_variables();
        auto coefs = std::make_shared_for_overwrite<double[]>(num_vars);
        tmp_indices.resize(num_vars);
        std::iota(tmp_indices.begin(), tmp_indices.end(), 0);
        check(COPT.GetColInfo(prob, COPT_DBLINFO_OBJ,
                              static_cast<int>(num_vars), tmp_indices.data(),
                              coefs.get()));
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
    void _add_variable(const variable_params & params, const char type) {
        check(COPT.AddCol(prob, params.obj_coef, 0, NULL, NULL, type,
                          params.lower_bound.value_or(-COPT_INFINITY),
                          params.upper_bound.value_or(+COPT_INFINITY), NULL));
    }
    void _add_variables(std::size_t offset, std::size_t count,
                        const variable_params & params, const char type) {
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
        check(COPT.AddCols(
            prob, static_cast<int>(count), tmp_scalars.data(), NULL, NULL, NULL,
            NULL, tmp_types.data(),
            dbl_offset_1.has_value()
                ? (tmp_scalars.data() +
                   static_cast<std::ptrdiff_t>(dbl_offset_1.value()))
                : NULL,
            dbl_offset_2.has_value()
                ? (tmp_scalars.data() +
                   static_cast<std::ptrdiff_t>(dbl_offset_2.value()))
                : NULL,
            NULL));
    }

public:
    variable add_variable(
        const variable_params params = default_variable_params) {
        int var_id = static_cast<int>(num_variables());
        _add_variable(params, COPT_CONTINUOUS);
        return variable(var_id);
    }
    auto add_variables(
        std::size_t count,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params, COPT_CONTINUOUS);
        return _make_variables_range(offset, count);
    }
    template <typename IL>
    auto add_variables(
        std::size_t count, IL && id_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params, COPT_CONTINUOUS);
        return _make_indexed_variables_range(offset, count,
                                             std::forward<IL>(id_lambda));
    }

private:
    template <typename ER>
    inline variable _add_column(ER && entries, const variable_params & params,
                                const char & type) {
        const int var_id = static_cast<int>(num_variables());
        _reset_cache(num_constraints());
        _register_raw_entries(entries);
        check(COPT.AddCol(prob, params.obj_coef,
                          static_cast<int>(tmp_indices.size()),
                          tmp_indices.data(), tmp_scalars.data(), type,
                          params.lower_bound.value_or(-COPT_INFINITY),
                          params.upper_bound.value_or(+COPT_INFINITY), NULL));
        return variable(var_id);
    }

public:
    template <ranges::range ER>
    variable add_column(
        ER && entries, const variable_params params = default_variable_params) {
        return _add_column(entries, params, COPT_CONTINUOUS);
    }
    template <typename E>
    variable add_column(
        std::initializer_list<E> entries,
        const variable_params params = default_variable_params) {
        return _add_column(entries, params, COPT_CONTINUOUS);
    }

    void set_objective_coefficient(variable v, scalar c) {
        const int id = v.id();
        check(COPT.SetColObj(prob, 1, &id, &c));
    }
    void set_variable_lower_bound(variable v, scalar lb) {
        const int id = v.id();
        check(COPT.SetColLower(prob, 1, &id, &lb));
    }
    void set_variable_upper_bound(variable v, scalar ub) {
        const int id = v.id();
        check(COPT.SetColUpper(prob, 1, &id, &ub));
    }

    scalar get_objective_coefficient(variable v) {
        scalar coef;
        const int id = v.id();
        check(COPT.GetColInfo(prob, COPT_DBLINFO_OBJ, 1, &id, &coef));
        return coef;
    }
    scalar get_variable_lower_bound(variable v) {
        scalar lb;
        const int id = v.id();
        check(COPT.GetColInfo(prob, COPT_DBLINFO_LB, 1, &id, &lb));
        return lb;
    }
    scalar get_variable_upper_bound(variable v) {
        scalar ub;
        const int id = v.id();
        check(COPT.GetColInfo(prob, COPT_DBLINFO_UB, 1, &id, &ub));
        return ub;
    }

    constraint add_constraint(linear_constraint auto && lc) {
        auto constr_id = static_cast<constraint_id>(num_constraints());
        _reset_cache(num_variables());
        _register_entries(lc.linear_terms());
        const scalar b = lc.rhs();
        check(COPT.AddRow(prob, static_cast<int>(tmp_indices.size()),
                          tmp_indices.data(), tmp_scalars.data(),
                          constraint_sense_to_copt_sense(lc.sense()), b,
                          COPT_INFINITY, NULL));
        return constraint(constr_id);
    }

private:
    template <linear_constraint LC>
    void _register_constraint(const int & constr_id, const LC & lc) {
        tmp_begins.emplace_back(static_cast<indice>(tmp_indices.size()));
        tmp_types.emplace_back(constraint_sense_to_copt_sense(lc.sense()));
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
        const indice offset = static_cast<indice>(num_constraints());
        indice constr_id = offset;
        for(auto && key : keys) {
            _register_first_valued_constraint(constr_id, key,
                                              constraint_lambdas...);
            ++constr_id;
        }
        tmp_begins.emplace_back(static_cast<indice>(tmp_indices.size()));
        check(COPT.AddRows(prob, static_cast<int>(tmp_rhs.size()),
                           tmp_begins.data(), NULL, tmp_indices.data(),
                           tmp_scalars.data(), tmp_types.data(), tmp_rhs.data(),
                           NULL, NULL));
        return constraints_range(
            keys,
            ranges::view::transform(ranges::view::iota(offset, constr_id),
                                    [](auto && i) { return constraint{i}; }));
    }
};

}  // namespace copt::v7_2
}  // namespace fhamonic::mippp

#endif  // MIPPP_COPT_v7_2_BASE_HPP