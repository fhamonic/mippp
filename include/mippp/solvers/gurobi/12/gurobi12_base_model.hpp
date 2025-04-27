#ifndef MIPPP_GUROBI12_BASE_MODEL_HPP
#define MIPPP_GUROBI12_BASE_MODEL_HPP

#include <limits>
#include <optional>
#include <vector>

#include <range/v3/algorithm/sort.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/move.hpp>
#include <range/v3/view/zip.hpp>

#include "mippp/detail/optional_helper.hpp"
#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/gurobi/12/gurobi12_api.hpp"

namespace fhamonic {
namespace mippp {

class gurobi12_base_model {
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

    struct variable_params {
        scalar obj_coef = scalar{0};
        std::optional<scalar> lower_bound = std::nullopt;
        std::optional<scalar> upper_bound = std::nullopt;
    };

    static constexpr variable_params default_variable_params = {
        .obj_coef = 0, .lower_bound = 0, .upper_bound = std::nullopt};

protected:
    const gurobi12_api & GRB;
    GRBenv * env;
    GRBmodel * model;

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

    std::vector<std::pair<constraint_id, unsigned int>>
        tmp_constraint_entry_cache;
    std::vector<int> tmp_indices;
    std::vector<int> tmp_variables;
    std::vector<double> tmp_scalars;
    std::vector<char> tmp_types;
    std::vector<double> tmp_rhs;

    std::vector<bool> _var_name_set;

public:
    [[nodiscard]] explicit gurobi12_base_model(const gurobi12_api & api)
        : GRB(api) {
        check(GRB.emptyenvinternal(&env, GRB_VERSION_MAJOR, GRB_VERSION_MINOR,
                                   GRB_VERSION_TECHNICAL));
        check(GRB.startenv(env));
        check(GRB.newmodel(env, &model, "GUROBI", 0, NULL, NULL, NULL, NULL,
                           NULL));
        GRB.freeenv(env);
        env = GRB.getenv(model);
        if(env == NULL)
            throw std::runtime_error(
                "gurobi12_base_model: Could not retrieve model environement.");
    }
    ~gurobi12_base_model() { check(GRB.freemodel(model)); };

protected:
    void check(int error) {
        if(error)
            throw std::runtime_error(std::to_string(error) + ':' +
                                     GRB.geterrormsg(env));
    }
    void update_gurobi_model() { check(GRB.updatemodel(model)); }

public:
    std::size_t num_variables() {
        int num;
        update_gurobi_model();
        check(GRB.getintattr(model, GRB_INT_ATTR_NUMVARS, &num));
        return static_cast<std::size_t>(num);
    }
    std::size_t num_constraints() {
        int num;
        update_gurobi_model();
        check(GRB.getintattr(model, GRB_INT_ATTR_NUMCONSTRS, &num));
        return static_cast<std::size_t>(num);
    }
    std::size_t num_entries() {
        int num;
        update_gurobi_model();
        check(GRB.getintattr(model, GRB_INT_ATTR_NUMNZS, &num));
        return static_cast<std::size_t>(num);
    }

    void set_maximization() {
        check(GRB.setintattr(model, GRB_INT_ATTR_MODELSENSE, GRB_MAXIMIZE));
    }
    void set_minimization() {
        check(GRB.setintattr(model, GRB_INT_ATTR_MODELSENSE, GRB_MINIMIZE));
    }

    void set_objective_offset(double constant) {
        check(GRB.setdblattr(model, GRB_DBL_ATTR_OBJCON, constant));
    }
    void set_objective(linear_expression auto && le) {
        auto num_vars = num_variables();
        tmp_scalars.resize(num_vars);
        std::fill(tmp_scalars.begin(), tmp_scalars.end(), 0.0);
        for(auto && [var, coef] : le.linear_terms()) {
            tmp_scalars[var.uid()] += coef;
        }
        check(GRB.setdblattrarray(model, GRB_DBL_ATTR_OBJ, 0,
                                  static_cast<int>(num_vars),
                                  tmp_scalars.data()));
        set_objective_offset(le.constant());
    }
    void add_objective(linear_expression auto && le) {
        tmp_variables.resize(0);
        tmp_scalars.resize(0);
        for(auto && [var, coef] : le.linear_terms()) {
            tmp_variables.emplace_back(var.id());
            tmp_scalars.emplace_back(get_objective_coefficient(var) + coef);
        }
        check(GRB.setdblattrlist(model, GRB_DBL_ATTR_OBJ,
                                 static_cast<int>(tmp_variables.size()),
                                 tmp_variables.data(), tmp_scalars.data()));
        set_objective_offset(get_objective_offset() + le.constant());
    }
    double get_objective_offset() {
        double constant;
        update_gurobi_model();
        check(GRB.getdblattr(model, GRB_DBL_ATTR_OBJCON, &constant));
        return constant;
    }
    auto get_objective() {
        auto num_vars = num_variables();
        std::vector<double> coefs(num_vars);
        update_gurobi_model();
        check(GRB.getdblattrarray(model, GRB_DBL_ATTR_OBJ, 0,
                                  static_cast<int>(num_vars), coefs.data()));
        return linear_expression_view(
            ranges::view::zip(ranges::view::iota(0, static_cast<int>(num_vars)),
                              ranges::view::move(coefs)),
            get_objective_offset());
    }

protected:
    inline void _add_variable(const variable_params & params,
                              const char & type) {
        check(GRB.addvar(model, 0, NULL, NULL, params.obj_coef,
                         params.lower_bound.value_or(-GRB_INFINITY),
                         params.upper_bound.value_or(GRB_INFINITY), type,
                         NULL));
        _var_name_set.push_back(false);
    }
    inline void _add_variables(const std::size_t & offset,
                               const std::size_t & count,
                               const variable_params & params,
                               const char & type) {
        check(GRB.addvars(model, static_cast<int>(count), 0, NULL, NULL, NULL,
                          NULL, NULL, NULL, NULL, NULL));
        if(double obj = params.obj_coef; obj != 0.0) {
            tmp_scalars.resize(count);
            std::fill(tmp_scalars.begin(), tmp_scalars.end(), obj);
            check(GRB.setdblattrarray(
                model, GRB_DBL_ATTR_OBJ, static_cast<int>(offset),
                static_cast<int>(count), tmp_scalars.data()));
        }
        if(double lb = params.lower_bound.value_or(-GRB_INFINITY); lb != 0.0) {
            tmp_scalars.resize(count);
            std::fill(tmp_scalars.begin(), tmp_scalars.end(), lb);
            check(GRB.setdblattrarray(
                model, GRB_DBL_ATTR_LB, static_cast<int>(offset),
                static_cast<int>(count), tmp_scalars.data()));
        }
        if(double ub = params.upper_bound.value_or(GRB_INFINITY);
           ub != GRB_INFINITY) {
            tmp_scalars.resize(count);
            std::fill(tmp_scalars.begin(), tmp_scalars.end(), ub);
            check(GRB.setdblattrarray(
                model, GRB_DBL_ATTR_UB, static_cast<int>(offset),
                static_cast<int>(count), tmp_scalars.data()));
        }
        if(type != GRB_CONTINUOUS) {
            tmp_types.resize(count);
            std::fill(tmp_types.begin(), tmp_types.end(), type);
            check(GRB.setcharattrarray(
                model, GRB_CHAR_ATTR_VTYPE, static_cast<int>(offset),
                static_cast<int>(count), tmp_types.data()));
        }
        _var_name_set.resize(offset + count, false);
    }
    inline auto _make_variables_range(const std::size_t & offset,
                                      const std::size_t & count) {
        return make_variables_range(ranges::view::transform(
            ranges::view::iota(static_cast<variable_id>(offset),
                               static_cast<variable_id>(offset + count)),
            [](auto && i) { return variable{i}; }));
    }
    template <typename IL>
    inline auto _make_indexed_variables_range(const std::size_t & offset,
                                              const std::size_t & count,
                                              IL && id_lambda) {
        return make_indexed_variables_range(
            typename detail::function_traits<IL>::arg_types(),
            ranges::view::transform(
                ranges::view::iota(static_cast<variable_id>(offset),
                                   static_cast<variable_id>(offset + count)),
                [](auto && i) { return variable{i}; }),
            std::forward<IL>(id_lambda));
    }

public:
    variable add_variable(
        const variable_params params = default_variable_params) {
        int var_id = static_cast<int>(num_variables());
        _add_variable(params, GRB_CONTINUOUS);
        return variable(var_id);
    }
    auto add_variables(
        std::size_t count,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params, GRB_CONTINUOUS);
        return _make_variables_range(offset, count);
    }
    template <typename IL>
    auto add_variables(
        std::size_t count, IL && id_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params, GRB_CONTINUOUS);
        return _make_indexed_variables_range(offset, count,
                                             std::forward<IL>(id_lambda));
    }
    // template <typename IL, typename NL>
    // auto add_variables(std::size_t count, IL && id_lambda, NL && name_lambda,
    //                    variable_params params = default_variable_params)
    //                    noexcept {
    //     const std::size_t offset = num_variables();
    //     _add_variables(offset, count, params);
    //     return make_lazily_named_variables_range(
    //         typename detail::function_traits<IL>::arg_types(),
    //         ranges::view::transform(ranges::view::iota(offset, offset +
    //         count),
    //                                 [](auto && i) { return variable{i}; }),
    //         std::forward<IL>(id_lambda),
    //         [this, name_lambda = std::forward<NL>(name_lambda)](
    //             const variable var, Args... args) mutable {
    //             if(_var_name_set[static_cast<std::size_t>(var.id())]) return;
    //             set_variable_name(var, name_lambda(args...));
    //         });
    // }

    void set_objective_coefficient(variable v, double c) {
        check(GRB.setdblattrelement(model, GRB_DBL_ATTR_OBJ, v.id(), c));
    }
    void set_variable_lower_bound(variable v, double lb) {
        check(GRB.setdblattrelement(model, GRB_DBL_ATTR_LB, v.id(), lb));
    }
    void set_variable_upper_bound(variable v, double ub) {
        check(GRB.setdblattrelement(model, GRB_DBL_ATTR_UB, v.id(), ub));
    }
    void set_variable_name(variable v, std::string name) {
        check(GRB.setstrattrelement(model, GRB_STR_ATTR_VARNAME, v.id(),
                                    name.c_str()));
        _var_name_set[static_cast<std::size_t>(v.id())] = true;
    }

    double get_objective_coefficient(variable v) {
        double coef;
        update_gurobi_model();
        check(GRB.getdblattrelement(model, GRB_DBL_ATTR_OBJ, v.id(), &coef));
        return coef;
    }
    double get_variable_lower_bound(variable v) {
        double lb;
        update_gurobi_model();
        check(GRB.getdblattrelement(model, GRB_DBL_ATTR_LB, v.id(), &lb));
        return lb;
    }
    double get_variable_upper_bound(variable v) {
        double ub;
        update_gurobi_model();
        check(GRB.getdblattrelement(model, GRB_DBL_ATTR_UB, v.id(), &ub));
        return ub;
    }
    auto get_variable_name(variable v) {
        char * name;
        update_gurobi_model();
        check(
            GRB.getstrattrelement(model, GRB_STR_ATTR_VARNAME, v.id(), &name));
        return std::string(name);
    }

    constraint add_constraint(linear_constraint auto && lc) {
        int constr_id = static_cast<int>(num_constraints());
        tmp_constraint_entry_cache.resize(num_variables());
        tmp_variables.resize(0);
        tmp_scalars.resize(0);
        for(auto && [var, coef] : lc.linear_terms()) {
            auto & p = tmp_constraint_entry_cache[var.uid()];
            if(p.first == constr_id + 1) {
                tmp_scalars[p.second] += coef;
                continue;
            }
            p = std::make_pair(constr_id + 1, tmp_variables.size());
            tmp_variables.emplace_back(var.id());
            tmp_scalars.emplace_back(coef);
        }
        check(GRB.addconstr(model, static_cast<int>(tmp_variables.size()),
                            tmp_variables.data(), tmp_scalars.data(),
                            constraint_sense_to_gurobi_sense(lc.sense()),
                            lc.rhs(), NULL));
        return constraint(constr_id);
    }

private:
    template <linear_constraint LC>
    void register_constraint(const int & constr_id, const LC & lc) {
        tmp_indices.emplace_back(static_cast<int>(tmp_variables.size()));
        tmp_types.emplace_back(
            constraint_sense_to_gurobi_sense(lc.sense()));
        tmp_rhs.emplace_back(lc.rhs());
        for(auto && [var, coef] : lc.linear_terms()) {
            auto & p = tmp_constraint_entry_cache[var.uid()];
            if(p.first == constr_id + 1) {
                tmp_scalars[p.second] += coef;
                continue;
            }
            p = std::make_pair(constr_id + 1, tmp_variables.size());
            tmp_variables.emplace_back(var.id());
            tmp_scalars.emplace_back(coef);
        }
    }
    template <typename Key, typename LastConstrLambda>
        requires linear_constraint<std::invoke_result_t<LastConstrLambda, Key>>
    void register_first_valued_constraint(const int & constr_id,
                                          const Key & key,
                                          const LastConstrLambda & lc_lambda) {
        register_constraint(constr_id, lc_lambda(key));
    }
    template <typename Key, typename OptConstrLambda, typename... Tail>
        requires detail::optional_type<
                     std::invoke_result_t<OptConstrLambda, Key>> &&
                 linear_constraint<detail::optional_type_value_t<
                     std::invoke_result_t<OptConstrLambda, Key>>>
    void register_first_valued_constraint(const int & constr_id,
                                          const Key & key,
                                          const OptConstrLambda & opt_lc_lambda,
                                          const Tail &... tail) {
        if(const auto & opt_lc = opt_lc_lambda(key)) {
            register_constraint(constr_id, opt_lc.value());
            return;
        }
        register_first_valued_constraint(constr_id, key, tail...);
    }

public:
    template <ranges::range IR, typename... CL>
    auto add_constraints(IR && keys, CL... constraint_lambdas) {
        tmp_constraint_entry_cache.resize(num_variables());
        tmp_indices.resize(0);
        tmp_variables.resize(0);
        tmp_scalars.resize(0);
        tmp_types.resize(0);
        tmp_rhs.resize(0);
        const int offset = static_cast<int>(num_constraints());
        int constr_id = offset;
        for(auto && key : keys) {
            register_first_valued_constraint(constr_id, key,
                                             constraint_lambdas...);
            ++constr_id;
        }
        check(GRB.addconstrs(
            model, constr_id - offset, static_cast<int>(tmp_variables.size()),
            tmp_indices.data(), tmp_variables.data(), tmp_scalars.data(),
            tmp_types.data(), tmp_rhs.data(), NULL));
        return constraints_range(
            keys,
            ranges::view::transform(ranges::view::iota(offset, constr_id),
                                    [](auto && i) { return constraint{i}; }));
    }

    void set_constraint_rhs(constraint constr, double rhs) {
        check(GRB.setdblattrelement(model, GRB_DBL_ATTR_RHS, constr.id(), rhs));
    }
    void set_constraint_sense(constraint constr, constraint_sense r) {
        check(GRB.setcharattrelement(model, GRB_CHAR_ATTR_SENSE, constr.id(),
                                     constraint_sense_to_gurobi_sense(r)));
    }
    // adds an equality constraint with a slack variable bounded in [0, ub-lb]
    constraint add_ranged_constraint(linear_expression auto && le, double lb,
                                     double ub) {
        int constr_id = static_cast<int>(num_constraints());
        tmp_constraint_entry_cache.resize(num_variables());
        tmp_variables.resize(0);
        tmp_scalars.resize(0);
        for(auto && [var, coef] : le.linear_terms()) {
            auto & p = tmp_constraint_entry_cache[var.uid()];
            if(p.first == constr_id + 1) {
                tmp_scalars[p.second] += coef;
                continue;
            }
            p = std::make_pair(constr_id + 1, tmp_variables.size());
            tmp_variables.emplace_back(var.id());
            tmp_scalars.emplace_back(coef);
        }
        check(GRB.addrangeconstr(model, static_cast<int>(tmp_variables.size()),
                                 tmp_variables.data(), tmp_scalars.data(), lb,
                                 ub, NULL));
        return constraint(constr_id);
    }
    // void set_constraint_name(constraint constr, auto && name);

    // auto get_constraint_lhs(constraint constr) {}
    double get_constraint_rhs(constraint constr) {
        double rhs;
        update_gurobi_model();
        check(
            GRB.getdblattrelement(model, GRB_DBL_ATTR_RHS, constr.id(), &rhs));
        return rhs;
    }
    constraint_sense get_constraint_sense(constraint constr) {
        char sense;
        update_gurobi_model();
        check(GRB.getcharattrelement(model, GRB_CHAR_ATTR_SENSE, constr.id(),
                                     &sense));
        return gurobi_sense_to_constraint_sense(sense);
    }
    // auto get_constraint(const constraint constr) {}
    auto get_constraint_name(constraint constr) {
        char * name;
        update_gurobi_model();
        check(GRB.getstrattrelement(model, GRB_STR_ATTR_CONSTRNAME, constr.id(),
                                    &name));
        return std::string(name);
    }

    void set_feasibility_tolerance(double tol) {
        check(GRB.setdblparam(env, GRB_DBL_PAR_FEASIBILITYTOL, tol));
    }
    double get_feasibility_tolerance() {
        double tol;
        check(GRB.getdblparam(env, GRB_DBL_PAR_FEASIBILITYTOL, &tol));
        return tol;
    }
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_GUROBI12_BASE_MODEL_HPP