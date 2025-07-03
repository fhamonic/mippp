#ifndef MIPPP_GLPK_v5_BASE_MODEL_HPP
#define MIPPP_GLPK_v5_BASE_MODEL_HPP

#include <cmath>
#include <optional>
#include <ranges>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/glpk/v5/glpk_api.hpp"
#include "mippp/solvers/model_base.hpp"

namespace fhamonic::mippp {
namespace glpk::v5 {

class glpk_base : public model_base<int, double> {
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
    const glpk_api & glp;
    glp_prob * model;
    double objective_offset;

    static constexpr int constraint_sense_to_glp_row_type(
        constraint_sense rel) {
        if(rel == constraint_sense::less_equal) return GLP_UP;
        if(rel == constraint_sense::equal) return GLP_FX;
        return GLP_LO;
    }
    static constexpr constraint_sense glp_row_type_to_constraint_sense(
        int type) {
        if(type == GLP_UP) return constraint_sense::less_equal;
        if(type == GLP_FX) return constraint_sense::equal;
        if(type == GLP_LO) return constraint_sense::greater_equal;
        throw std::runtime_error("glpk_base: Cannot convert row type '" +
                                 std::to_string(type) +
                                 "' to constraint_sense.");
    }

    void _reset_cache(std::size_t num_variables) {
        tmp_entry_index_cache.resize(num_variables);
        tmp_indices.resize(1);
        tmp_scalars.resize(1);
    }

    template <std::ranges::range Entries>
    void _register_entries(Entries && entries) {
        ++register_count;
        for(auto && [entity, coef] : entries) {
            auto & p = tmp_entry_index_cache[entity.uid()];
            if(p.first == register_count) {
                tmp_scalars[p.second] += static_cast<scalar>(coef);
                continue;
            }
            p = std::make_pair(register_count, tmp_indices.size());
            tmp_indices.emplace_back(entity.id() + 1);
            tmp_scalars.emplace_back(coef);
        }
    }

public:
    [[nodiscard]] explicit glpk_base(const glpk_api & api)
        : model_base<int, double>()
        , glp(api)
        , model(glp.create_prob())
        , objective_offset(0.0) {}
    ~glpk_base() { glp.delete_prob(model); }

    std::size_t num_variables() {
        return static_cast<std::size_t>(glp.get_num_cols(model));
    }
    std::size_t num_constraints() {
        return static_cast<std::size_t>(glp.get_num_rows(model));
    }
    std::size_t num_entries() {
        return static_cast<std::size_t>(glp.get_num_nz(model));
    }

    void set_maximization() { glp.set_obj_dir(model, GLP_MAX); }
    void set_minimization() { glp.set_obj_dir(model, GLP_MIN); }

    void set_objective_offset(double constant) { objective_offset = constant; }
    void set_objective(linear_expression auto && le) {
        auto num_vars = static_cast<int>(num_variables());
        for(int var = 0; var < num_vars; ++var) {
            glp.set_obj_coef(model, var + 1, 0.0);
        }
        for(auto && [var, coef] : le.linear_terms()) {
            glp.set_obj_coef(model, var.id() + 1,
                             glp.get_obj_coef(model, var.id() + 1) + coef);
        }
        set_objective_offset(le.constant());
    }
    void add_objective(linear_expression auto && le) {
        for(auto && [var, coef] : le.linear_terms()) {
            glp.set_obj_coef(model, var.id() + 1,
                             glp.get_obj_coef(model, var.id() + 1) + coef);
        }
        set_objective_offset(get_objective_offset() + le.constant());
    }
    double get_objective_offset() { return objective_offset; }
    auto get_objective() {
        return linear_expression_view(
           std::views::transform(
               std::views::iota(variable_id{0},
                                   static_cast<variable_id>(num_variables())),
                [this](auto i) {
                    return std::make_pair(variable(i),
                                          glp.get_obj_coef(model, i + 1));
                }),
            get_objective_offset());
    }

protected:
    inline void _add_variable(const int & var_id,
                              const variable_params & params, int type) {
        glp.add_cols(model, 1);
        if(params.obj_coef != 0.0) {
            glp.set_obj_coef(model, var_id + 1, params.obj_coef);
        }
        if(params.lower_bound.has_value() && params.upper_bound.has_value()) {
            double lb = params.lower_bound.value();
            double ub = params.upper_bound.value();
            glp.set_col_bnds(model, var_id + 1, (lb == ub) ? GLP_FX : GLP_DB,
                             lb, ub);
        } else if(params.lower_bound.has_value()) {
            glp.set_col_bnds(model, var_id + 1, GLP_LO,
                             params.lower_bound.value(), 0.0);
        } else if(params.upper_bound.has_value()) {
            glp.set_col_bnds(model, var_id + 1, GLP_UP, 0.0,
                             params.upper_bound.value());
        } else {
            glp.set_col_bnds(model, var_id + 1, GLP_FR, 0.0, 0.0);
        }
        if(type != GLP_CV) {
            glp.set_col_kind(model, var_id + 1, type);
        }
    }
    inline void _add_variables(std::size_t offset, std::size_t count,
                               const variable_params & params, int type) {
        if(count == 0u) return;
        glp.add_cols(model, static_cast<int>(count));
        if(auto obj = params.obj_coef; obj != 0.0) {
            for(std::size_t i = offset + 1; i <= offset + count; ++i)
                glp.set_obj_coef(model, static_cast<int>(i), obj);
        }
        if(params.lower_bound.has_value() && params.upper_bound.has_value()) {
            double lb = params.lower_bound.value();
            double ub = params.upper_bound.value();
            for(std::size_t i = offset + 1; i <= offset + count; ++i)
                glp.set_col_bnds(model, static_cast<int>(i),
                                 (lb == ub) ? GLP_FX : GLP_DB, lb, ub);
        } else if(params.lower_bound.has_value()) {
            for(std::size_t i = offset + 1; i <= offset + count; ++i)
                glp.set_col_bnds(model, static_cast<int>(i), GLP_LO,
                                 params.lower_bound.value(), 0.0);
        } else if(params.upper_bound.has_value()) {
            for(std::size_t i = offset + 1; i <= offset + count; ++i)
                glp.set_col_bnds(model, static_cast<int>(i), GLP_UP, 0.0,
                                 params.upper_bound.value());
        } else {
            for(std::size_t i = offset + 1; i <= offset + count; ++i)
                glp.set_col_bnds(model, static_cast<int>(i), GLP_FR, 0.0, 0.0);
        }
        if(type != GLP_CV) {
            for(std::size_t i = offset + 1; i <= offset + count; ++i)
                glp.set_col_kind(model, static_cast<int>(i), type);
        }
    }

public:
    variable add_variable(
        const variable_params params = default_variable_params) {
        int var_id = static_cast<int>(num_variables());
        _add_variable(var_id, params, GLP_CV);
        return variable(var_id);
    }
    auto add_variables(
        std::size_t count,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params, GLP_CV);
        return _make_variables_range(offset, count);
    }
    template <typename IL>
    auto add_variables(
        std::size_t count, IL && id_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params, GLP_CV);
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
        _add_variables(offset, count, params, GLP_CV);
        return _make_named_variables_range(offset, count,
                                           std::forward<NL>(name_lambda), this);
    }
    template <typename IL, typename NL>
    auto add_named_variables(
        std::size_t count, IL && id_lambda, NL && name_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params, GLP_CV);
        return _make_indexed_named_variables_range(
            offset, count, std::forward<IL>(id_lambda),
            std::forward<NL>(name_lambda), this);
    }

private:
    template <typename ER>
    inline variable _add_column(ER && entries, const variable_params & params) {
        const int var_id = static_cast<int>(num_variables());
        _add_variable(var_id, params, GLP_CV);
        _reset_cache(num_constraints());
        _register_entries(entries);
        glp.set_mat_col(model, var_id + 1,
                        static_cast<int>(tmp_indices.size()) - 1,
                        tmp_indices.data(), tmp_scalars.data());
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
        glp.set_obj_coef(model, v.id() + 1, c);
    }
    void set_variable_lower_bound(variable v, double lb) {
        glp.set_col_bnds(model, v.id() + 1, GLP_DB, lb,
                         get_variable_upper_bound(v));
    }
    void set_variable_upper_bound(variable v, double ub) {
        glp.set_col_bnds(model, v.id() + 1, GLP_DB, get_variable_lower_bound(v),
                         ub);
    }
    void set_variable_name(variable v, const std::string & name) {
        glp.set_col_name(model, v.id() + 1, name.c_str());
    }

    double get_objective_coefficient(variable v) {
        return glp.get_obj_coef(model, v.id() + 1);
    }
    double get_variable_lower_bound(variable v) {
        return glp.get_col_lb(model, v.id() + 1);
    }
    double get_variable_upper_bound(variable v) {
        return glp.get_col_ub(model, v.id() + 1);
    }
    std::string get_variable_name(variable v) {
        return std::string(glp.get_col_name(model, v.id() + 1));
    }

private:
    template <linear_constraint LC>
    void _add_constraint(const int & constr_id, const LC & lc) {
        glp.add_rows(model, 1);
        tmp_indices.resize(1);
        tmp_scalars.resize(1);
        _register_entries(lc.linear_terms());
        glp.set_mat_row(model, constr_id + 1,
                        static_cast<int>(tmp_indices.size()) - 1,
                        tmp_indices.data(), tmp_scalars.data());
        const double b = lc.rhs();
        glp.set_row_bnds(model, constr_id + 1,
                         constraint_sense_to_glp_row_type(lc.sense()), b, b);
    }

public:
    constraint add_constraint(linear_constraint auto && lc) {
        tmp_entry_index_cache.resize(num_variables());
        int constr_id = static_cast<int>(num_constraints());
        _add_constraint(constr_id, lc);
        return constraint(constr_id);
    }

private:
    template <typename Key, typename LastConstrLambda>
        requires linear_constraint<std::invoke_result_t<LastConstrLambda, Key>>
    void _add_first_valued_constraint(const int & constr_id, const Key & key,
                                      const LastConstrLambda & lc_lambda) {
        _add_constraint(constr_id, lc_lambda(key));
    }
    template <typename Key, typename OptConstrLambda, typename... Tail>
        requires detail::optional_type<
                     std::invoke_result_t<OptConstrLambda, Key>> &&
                 linear_constraint<detail::optional_type_value_t<
                     std::invoke_result_t<OptConstrLambda, Key>>>
    void _add_first_valued_constraint(const int & constr_id, const Key & key,
                                      const OptConstrLambda & opt_lc_lambda,
                                      const Tail &... tail) {
        if(const auto & opt_lc = opt_lc_lambda(key)) {
            _add_constraint(constr_id, opt_lc.value());
            return;
        }
        _add_first_valued_constraint(constr_id, key, tail...);
    }

public:
    template <std::ranges::range IR, typename... CL>
    auto add_constraints(IR && keys, CL... constraint_lambdas) {
        tmp_entry_index_cache.resize(num_variables());
        const int offset = static_cast<int>(num_constraints());
        int constr_id = offset;
        for(auto && key : keys) {
            _add_first_valued_constraint(constr_id, key, constraint_lambdas...);
            ++constr_id;
        }
        return constraints_range(
            keys,
           std::views::transform(std::views::iota(offset, constr_id),
                                    [](auto && i) { return constraint{i}; }));
    }
};

}  // namespace glpk::v5
}  // namespace fhamonic::mippp

#endif  // MIPPP_GLPK_v5_BASE_MODEL_HPP