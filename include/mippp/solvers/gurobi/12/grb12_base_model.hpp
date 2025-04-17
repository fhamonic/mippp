#ifndef MIPPP_GRB12_BASE_MODEL_HPP
#define MIPPP_GRB12_BASE_MODEL_HPP

#include <limits>
#include <optional>
#include <vector>

#include <range/v3/algorithm/sort.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/move.hpp>
#include <range/v3/view/zip.hpp>

#include "mippp/detail/function_traits.hpp"
#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_variable.hpp"

#include "mippp/solvers/gurobi/12/grb12_api.hpp"

namespace fhamonic {
namespace mippp {

class grb12_base_model {
public:
    using variable_id = int;
    using scalar = double;
    using variable = model_variable<variable_id, scalar>;
    using constraint = int;

    struct variable_params {
        scalar obj_coef = scalar{0};
        std::optional<scalar> lower_bound = scalar{0};
        std::optional<scalar> upper_bound = std::nullopt;
    };

protected:
    const grb12_api & GRB;
    GRBenv * env;
    GRBmodel * model;
    std::optional<lp_status> opt_lp_status;

    static constexpr char constraint_relation_to_grb_sense(
        constraint_relation rel) {
        if(rel == constraint_relation::less_equal_zero) return GRB_LESS_EQUAL;
        if(rel == constraint_relation::equal_zero) return GRB_EQUAL;
        return GRB_GREATER_EQUAL;
    }
    static constexpr constraint_relation grb_sense_to_constraint_relation(
        char sense) {
        if(sense == GRB_LESS_EQUAL) return constraint_relation::less_equal_zero;
        if(sense == GRB_EQUAL) return constraint_relation::equal_zero;
        return constraint_relation::greater_equal_zero;
    }

    std::vector<std::pair<constraint, unsigned int>> tmp_constraint_entry_cache;
    std::vector<int> tmp_variables;
    std::vector<double> tmp_scalars;

public:
    [[nodiscard]] explicit grb12_base_model(const grb12_api & api) : GRB(api) {
        check(GRB.emptyenvinternal(&env, GRB_VERSION_MAJOR, GRB_VERSION_MINOR,
                                   GRB_VERSION_TECHNICAL));
        check(GRB.startenv(env));
        check(GRB.newmodel(env, &model, "GUROBI", 0, NULL, NULL, NULL, NULL,
                           NULL));
        GRB.freeenv(env);
        env = GRB.getenv(model);
        if(env == NULL)
            throw std::runtime_error(
                "grb12_base_model: Could not retrieve model environement.");
    }
    ~grb12_base_model() { check(GRB.freemodel(model)); };

protected:
    void check(int error) {
        if(error)
            throw std::runtime_error(std::to_string(error) + ':' +
                                     GRB.geterrormsg(env));
    }
    void update_grb_model() { check(GRB.updatemodel(model)); }

public:
    std::size_t num_variables() {
        int num;
        update_grb_model();
        check(GRB.getintattr(model, GRB_INT_ATTR_NUMVARS, &num));
        return static_cast<std::size_t>(num);
    }
    std::size_t num_constraints() {
        int num;
        update_grb_model();
        check(GRB.getintattr(model, GRB_INT_ATTR_NUMCONSTRS, &num));
        return static_cast<std::size_t>(num);
    }
    std::size_t num_entries() {
        int num;
        update_grb_model();
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
            tmp_scalars[static_cast<std::size_t>(var)] += coef;
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
            tmp_variables.emplace_back(var);
            tmp_scalars.emplace_back(get_objective_coefficient(var) + coef);
        }
        check(GRB.setdblattrlist(model, GRB_DBL_ATTR_OBJ,
                                 static_cast<int>(tmp_variables.size()),
                                 tmp_variables.data(), tmp_scalars.data()));
        set_objective_offset(get_objective_offset() + le.constant());
    }
    double get_objective_offset() {
        double constant;
        update_grb_model();
        check(GRB.getdblattr(model, GRB_DBL_ATTR_OBJCON, &constant));
        return constant;
    }
    auto get_objective() {
        auto num_vars = num_variables();
        std::vector<double> coefs(num_vars);
        update_grb_model();
        check(GRB.getdblattrarray(model, GRB_DBL_ATTR_OBJ, 0,
                                  static_cast<int>(num_vars), coefs.data()));
        return linear_expression_view(
            ranges::view::zip(ranges::view::iota(0, static_cast<int>(num_vars)),
                              ranges::view::move(coefs)),
            get_objective_offset());
    }

    variable add_variable(
        const variable_params p = {
            .obj_coef = 0,
            .lower_bound = 0,
            .upper_bound = std::numeric_limits<double>::infinity()}) {
        int var_id = static_cast<int>(num_variables());
        check(GRB.addvar(model, 0, NULL, NULL, p.obj_coef,
                         p.lower_bound.value_or(-GRB_INFINITY),
                         p.upper_bound.value_or(GRB_INFINITY), GRB_CONTINUOUS,
                         NULL));
        return variable(var_id);
    }

// private:
//     template <typename IL, typename NL, typename... Args>
//         requires std::convertible_to<
//             typename detail::function_traits<IL>::result_type, variable_id>
//     auto _add_variables(detail::pack<Args...>, std::size_t count,
//                         IL && id_lambda, NL && name_lambda,
//                         variable_params params = {}) noexcept {
//         const std::size_t offset = num_variables();

//         check(GRB.addvars(model, static_cast<int>(count), 0, NULL, NULL, NULL,
//                           NULL, NULL, NULL, NULL, NULL));
//         if(auto obj = params.obj_coef; obj != 0.0) {
//             tmp_scalars.resize(count);
//             std::fill(tmp_scalars.begin(), tmp_scalars.end(), obj);
//             check(GRB.setdblattrarray(model, GRB_DBL_ATTR_OBJ,
//                                       static_cast<int>(offset),
//                                       static_cast<int>(count), NULL));
//         }
//         if(auto lb = params.lower_bound.value_or(-GRB_INFINITY); lb != 0.0) {
//             tmp_scalars.resize(count);
//             std::fill(tmp_scalars.begin(), tmp_scalars.end(), lb);
//             check(GRB.setdblattrarray(model, GRB_DBL_ATTR_LB,
//                                       static_cast<int>(offset),
//                                       static_cast<int>(count), NULL));
//         }
//         if(auto ub = params.upper_bound.value_or(GRB_INFINITY); ub != 0.0) {
//             tmp_scalars.resize(count);
//             std::fill(tmp_scalars.begin(), tmp_scalars.end(), ub);
//             check(GRB.setdblattrarray(model, GRB_DBL_ATTR_UB,
//                                       static_cast<int>(offset),
//                                       static_cast<int>(count), NULL));
//         }

//         return variables_range(
//             typename detail::function_traits<IL>::arg_types(), offset, count,
//             std::forward<IL>(id_lambda),
//             [this, name_lambda = std::forward<NL>(name_lambda)](
//                 const variable_id var_num, Args... args) mutable {
//                 if(_col_name[static_cast<std::size_t>(var_num)].has_value())
//                     return;
//                 _col_name[static_cast<std::size_t>(var_num)].emplace(
//                     name_lambda(args...));
//             });
//     }

public:
    // template <typename IL>
    //     requires std::convertible_to<
    //         typename detail::function_traits<IL>::result_type, variable_id>
    // auto add_variables(std::size_t count, IL && id_lambda,
    //                    variable_params params = {}) noexcept {
    //     return _add_variables(typename
    //     detail::function_traits<IL>::arg_types(),
    //                           count, std::forward<IL>(id_lambda), params);
    // }
    // template <typename IL, typename NL>
    //     requires std::convertible_to<
    //         typename detail::function_traits<IL>::result_type, variable_id>
    // auto add_variables(std::size_t count, IL && id_lambda, NL && name_lambda,
    //                    variable_params params = {}) noexcept {
    //     return _add_variables(typename
    //     detail::function_traits<IL>::arg_types(),
    //                           count, std::forward<IL>(id_lambda),
    //                           std::forward<NL>(name_lambda), params);
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
    }

    double get_objective_coefficient(variable v) {
        double coef;
        update_grb_model();
        check(GRB.getdblattrelement(model, GRB_DBL_ATTR_OBJ, v.id(), &coef));
        return coef;
    }
    double get_variable_lower_bound(variable v) {
        double lb;
        update_grb_model();
        check(GRB.getdblattrelement(model, GRB_DBL_ATTR_LB, v.id(), &lb));
        return lb;
    }
    double get_variable_upper_bound(variable v) {
        double ub;
        update_grb_model();
        check(GRB.getdblattrelement(model, GRB_DBL_ATTR_UB, v.id(), &ub));
        return ub;
    }
    auto get_variable_name(variable v) {
        char * name;
        update_grb_model();
        check(
            GRB.getstrattrelement(model, GRB_STR_ATTR_VARNAME, v.id(), &name));
        return std::string(name);
    }

    constraint add_constraint(linear_constraint auto && lc) {
        int constr_id = static_cast<int>(num_constraints());
        tmp_constraint_entry_cache.resize(num_variables());
        tmp_variables.resize(0);
        tmp_scalars.resize(0);
        for(auto && [var, coef] : lc.expression().linear_terms()) {
            auto & p =
                tmp_constraint_entry_cache[static_cast<std::size_t>(var)];
            if(p.first == constr_id + 1) {
                tmp_scalars[p.second] += coef;
                continue;
            }
            p = std::make_pair(constr_id + 1, tmp_variables.size());
            tmp_variables.emplace_back(var);
            tmp_scalars.emplace_back(coef);
        }
        check(GRB.addconstr(model, static_cast<int>(tmp_variables.size()),
                            tmp_variables.data(), tmp_scalars.data(),
                            constraint_relation_to_grb_sense(lc.relation()),
                            -lc.expression().constant(), NULL));
        return constr_id;
    }
    void set_constraint_rhs(constraint c, double rhs) {
        check(GRB.setdblattrelement(model, GRB_DBL_ATTR_RHS, c, rhs));
    }
    void set_constraint_sense(constraint c, constraint_relation r) {
        check(GRB.setcharattrelement(model, GRB_CHAR_ATTR_SENSE, c,
                                     constraint_relation_to_grb_sense(r)));
    }
    // adds an equality constraint with a slack variable bounded in [0, ub-lb]
    constraint add_ranged_constraint(linear_expression auto && le, double lb,
                                     double ub) {
        int constr_id = static_cast<int>(num_constraints());
        tmp_variables.resize(0);
        tmp_scalars.resize(0);
        for(auto && [var, coef] : le.linear_terms()) {
            tmp_variables.emplace_back(var);
            tmp_scalars.emplace_back(coef);
        }
        check(GRB.addrangeconstr(model, static_cast<int>(tmp_variables.size()),
                                 tmp_variables.data(), tmp_scalars.data(), lb,
                                 ub, NULL));
        return constr_id;
    }
    // void set_constraint_name(constraint c, auto && name);

    // auto get_constraint_lhs(constraint c) {}
    double get_constraint_rhs(constraint c) {
        double rhs;
        update_grb_model();
        check(GRB.getdblattrelement(model, GRB_DBL_ATTR_RHS, c, &rhs));
        return rhs;
    }
    constraint_relation get_constraint_sense(constraint c) {
        char sense;
        update_grb_model();
        check(GRB.getcharattrelement(model, GRB_CHAR_ATTR_SENSE, c, &sense));
        return grb_sense_to_constraint_relation(sense);
    }
    // auto get_constraint(const constraint c) {}
    auto get_constraint_name(constraint c) {
        char * name;
        update_grb_model();
        check(GRB.getstrattrelement(model, GRB_STR_ATTR_CONSTRNAME, c, &name));
        return std::string(name);
    }

    // void set_basic(variable v);
    // void set_non_basic(variable v);

    // void set_basic(constraint v);
    // void set_non_basic(constraint v);

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

#endif  // MIPPP_GRB12_BASE_MODEL_HPP