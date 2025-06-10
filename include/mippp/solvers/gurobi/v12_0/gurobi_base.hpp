#ifndef MIPPP_GUROBI_v12_0_BASE_HPP
#define MIPPP_GUROBI_v12_0_BASE_HPP

#include <memory>
#include <optional>
#include <vector>

#include <range/v3/algorithm/sort.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/move.hpp>
#include <range/v3/view/zip.hpp>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/gurobi/v12_0/gurobi_api.hpp"
#include "mippp/solvers/model_base.hpp"

namespace fhamonic::mippp {
namespace gurobi::v12_0 {

class gurobi_base : public model_base<int, double> {
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
    const gurobi_api & GRB;
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

    std::size_t _lazy_num_variables;
    std::size_t _lazy_num_constraints;

    std::vector<int> tmp_begins;
    std::vector<char> tmp_types;
    std::vector<double> tmp_rhs;

    std::vector<bool> _var_name_set;

public:
    [[nodiscard]] explicit gurobi_base(const gurobi_api & api)
        : model_base<int, double>()
        , GRB(api)
        , _lazy_num_variables(0)
        , _lazy_num_constraints(0) {
        check(GRB.emptyenvinternal(&env, GRB_VERSION_MAJOR, GRB_VERSION_MINOR,
                                   GRB_VERSION_TECHNICAL));
        check(GRB.startenv(env));
        check(GRB.newmodel(env, &model, "GUROBI", 0, NULL, NULL, NULL, NULL,
                           NULL));
        GRB.freeenv(env);
        env = GRB.getenv(model);
        if(env == NULL)
            throw std::runtime_error(
                "gurobi_base: Could not retrieve model environement.");
    }
    ~gurobi_base() { check(GRB.freemodel(model)); };

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
        if(static_cast<std::size_t>(num) != _lazy_num_variables)
            throw std::runtime_error(
                "gurobi_base: _lazy_num_variables differs from gurobi "
                "one.");
        return _lazy_num_variables;
    }
    std::size_t num_constraints() {
        int num;
        update_gurobi_model();
        check(GRB.getintattr(model, GRB_INT_ATTR_NUMCONSTRS, &num));
        if(static_cast<std::size_t>(num) != _lazy_num_constraints)
            throw std::runtime_error(
                "gurobi_base: _lazy_num_constraints differs from "
                "gurobi "
                "one.");
        return _lazy_num_constraints;
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
        tmp_indices.resize(0);
        tmp_scalars.resize(0);
        for(auto && [var, coef] : le.linear_terms()) {
            tmp_indices.emplace_back(var.id());
            tmp_scalars.emplace_back(get_objective_coefficient(var) + coef);
        }
        check(GRB.setdblattrlist(model, GRB_DBL_ATTR_OBJ,
                                 static_cast<int>(tmp_indices.size()),
                                 tmp_indices.data(), tmp_scalars.data()));
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
        auto coefs = std::make_shared_for_overwrite<double[]>(num_vars);
        update_gurobi_model();
        check(GRB.getdblattrarray(model, GRB_DBL_ATTR_OBJ, 0,
                                  static_cast<int>(num_vars), coefs.get()));
        return linear_expression_view(
            ranges::view::transform(
                ranges::view::iota(0, static_cast<int>(num_vars)),
                [coefs = std::move(coefs)](auto && i) {
                    return std::make_pair(variable(i), coefs[i]);
                }),
            get_objective_offset());
    }

protected:
    inline variable _add_variable(const variable_params & params,
                                  const char & type, const char * name_str) {
        check(GRB.addvar(model, 0, NULL, NULL, params.obj_coef,
                         params.lower_bound.value_or(-GRB_INFINITY),
                         params.upper_bound.value_or(GRB_INFINITY), type,
                         name_str));
        return variable(static_cast<int>(_lazy_num_variables++));
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
        _lazy_num_variables += count;
    }

public:
    variable add_variable(
        const variable_params params = default_variable_params) {
        return _add_variable(params, GRB_CONTINUOUS, NULL);
    }
    auto add_variables(
        std::size_t count,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = _lazy_num_variables;
        _add_variables(offset, count, params, GRB_CONTINUOUS);
        return _make_variables_range(offset, count);
    }
    template <typename IL>
    auto add_variables(
        std::size_t count, IL && id_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = _lazy_num_variables;
        _add_variables(offset, count, params, GRB_CONTINUOUS);
        return _make_indexed_variables_range(offset, count,
                                             std::forward<IL>(id_lambda));
    }

public:
    variable add_named_variable(
        const std::string & name,
        const variable_params params = default_variable_params) {
        return _add_variable(params, GRB_CONTINUOUS, name.c_str());
    }
    template <typename NL>
    auto add_named_variables(
        std::size_t count, NL && name_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = _lazy_num_variables;
        _add_variables(offset, count, params, GRB_CONTINUOUS);
        return _make_named_variables_range(offset, count,
                                           std::forward<NL>(name_lambda),
                                           this);
    }
    template <typename IL, typename NL>
    auto add_named_variables(
        std::size_t count, IL && id_lambda, NL && name_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = _lazy_num_variables;
        _add_variables(offset, count, params, GRB_CONTINUOUS);
        return _make_indexed_named_variables_range(
            offset, count, std::forward<IL>(id_lambda),
            std::forward<NL>(name_lambda), this);
    }

private:
    template <typename ER>
    inline variable _add_column(ER && entries, const variable_params & params,
                                const char & type) {
        _reset_cache(_lazy_num_constraints);
        _register_raw_entries(entries);
        check(
            GRB.addvar(model, static_cast<int>(tmp_indices.size()),
                       tmp_indices.data(), tmp_scalars.data(), params.obj_coef,
                       params.lower_bound.value_or(-GRB_INFINITY),
                       params.upper_bound.value_or(GRB_INFINITY), type, NULL));
        return variable(static_cast<int>(_lazy_num_variables++));
    }

public:
    template <ranges::range ER>
    variable add_column(
        ER && entries, const variable_params params = default_variable_params) {
        return _add_column(entries, params, GRB_CONTINUOUS);
    }
    variable add_column(
        std::initializer_list<std::pair<constraint, scalar>> entries,
        const variable_params params = default_variable_params) {
        return _add_column(entries, params, GRB_CONTINUOUS);
    }

    void set_objective_coefficient(variable v, double c) {
        check(GRB.setdblattrelement(model, GRB_DBL_ATTR_OBJ, v.id(), c));
    }
    void set_variable_lower_bound(variable v, double lb) {
        check(GRB.setdblattrelement(model, GRB_DBL_ATTR_LB, v.id(), lb));
    }
    void set_variable_upper_bound(variable v, double ub) {
        check(GRB.setdblattrelement(model, GRB_DBL_ATTR_UB, v.id(), ub));
    }
    void set_variable_name(variable v, const std::string & name) {
        check(GRB.setstrattrelement(model, GRB_STR_ATTR_VARNAME, v.id(),
                                    name.c_str()));
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
        const int constr_id = static_cast<int>(_lazy_num_constraints++);
        _reset_cache(_lazy_num_variables);
        _register_entries(lc.linear_terms());
        check(GRB.addconstr(model, static_cast<int>(tmp_indices.size()),
                            tmp_indices.data(), tmp_scalars.data(),
                            constraint_sense_to_gurobi_sense(lc.sense()),
                            lc.rhs(), NULL));
        return constraint(constr_id);
    }

private:
    template <linear_constraint LC>
    void _register_constraint(const LC & lc) {
        ++register_count;
        tmp_begins.emplace_back(static_cast<int>(tmp_indices.size()));
        tmp_types.emplace_back(constraint_sense_to_gurobi_sense(lc.sense()));
        tmp_rhs.emplace_back(lc.rhs());
        _register_entries(lc.linear_terms());
    }
    template <typename Key, typename LastConstrLambda>
        requires linear_constraint<std::invoke_result_t<LastConstrLambda, Key>>
    void _register_first_valued_constraint(const Key & key,
                                           const LastConstrLambda & lc_lambda) {
        _register_constraint(lc_lambda(key));
    }
    template <typename Key, typename OptConstrLambda, typename... Tail>
        requires detail::optional_type<
                     std::invoke_result_t<OptConstrLambda, Key>> &&
                 linear_constraint<detail::optional_type_value_t<
                     std::invoke_result_t<OptConstrLambda, Key>>>
    void _register_first_valued_constraint(
        const Key & key, const OptConstrLambda & opt_lc_lambda,
        const Tail &... tail) {
        if(const auto & opt_lc = opt_lc_lambda(key)) {
            _register_constraint(opt_lc.value());
            return;
        }
        _register_first_valued_constraint(key, tail...);
    }

public:
    template <ranges::range IR, typename... CL>
    auto add_constraints(IR && keys, CL... constraint_lambdas) {
        _reset_cache(_lazy_num_variables);
        tmp_begins.resize(0);
        tmp_types.resize(0);
        tmp_rhs.resize(0);
        const int offset = static_cast<int>(_lazy_num_constraints);
        int constr_id = offset;
        for(auto && key : keys) {
            _register_first_valued_constraint(key, constraint_lambdas...);
            ++constr_id;
        }
        check(GRB.addconstrs(
            model, constr_id - offset, static_cast<int>(tmp_indices.size()),
            tmp_begins.data(), tmp_indices.data(), tmp_scalars.data(),
            tmp_types.data(), tmp_rhs.data(), NULL));
        _lazy_num_constraints += static_cast<std::size_t>(constr_id - offset);
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
        int constr_id = static_cast<int>(_lazy_num_constraints++);
        _reset_cache(_lazy_num_variables);
        _register_entries(le.linear_terms());
        check(GRB.addrangeconstr(model, static_cast<int>(tmp_indices.size()),
                                 tmp_indices.data(), tmp_scalars.data(), lb, ub,
                                 NULL));
        ++_lazy_num_variables;  // added_slack variable
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

}  // namespace gurobi::v12_0
}  // namespace fhamonic::mippp

#endif  // MIPPP_GUROBI_v12_0_BASE_HPP