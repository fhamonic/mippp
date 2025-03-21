#ifndef MIPPP_GRB_MILP_MODEL_HPP
#define MIPPP_GRB_MILP_MODEL_HPP

#include <limits>
#include <ostream>
// #include <ranges>
#include <sstream>
#include <string_view>
#include <vector>

#include <range/v3/view/iota.hpp>
#include <range/v3/view/move.hpp>
#include <range/v3/view/zip.hpp>

#include "mippp/detail/function_traits.hpp"
#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_variable.hpp"

#include "mippp/api/grb_api.hpp"

namespace fhamonic {
namespace mippp {

class grb_milp_model {
private:
    const grb_api & GRB;
    GRBenv * env;
    GRBmodel * model;

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

    std::vector<int> tmp_variables;
    std::vector<double> tmp_scalars;

public:
    using variable_id = int;
    using scalar = double;
    using variable = model_variable<variable_id, scalar>;
    using constraint = int;

    struct variable_params {
        scalar obj_coef = scalar{0};
        scalar lower_bound = scalar{0};
        scalar upper_bound = std::numeric_limits<scalar>::infinity();
    };

public:
    [[nodiscard]] explicit grb_milp_model(const grb_api & api) : GRB(api) {
        check(GRB.emptyenvinternal(&env, GRB_VERSION_MAJOR, GRB_VERSION_MINOR,
                                   GRB_VERSION_TECHNICAL));
        check(GRB.startenv(env));
        check(GRB.newmodel(env, &model, "PL", 0, NULL, NULL, NULL, NULL, NULL));
    }
    ~grb_milp_model() {
        check(GRB.freemodel(model));
        GRB.freeenv(env);
    };

private:
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
        tmp_scalars.resize(0);
        tmp_scalars.resize(num_vars, 0.0);
        check(GRB.getdblattrarray(model, GRB_DBL_ATTR_OBJ, 0,
                                  static_cast<int>(num_vars), tmp_scalars.data()));
        tmp_variables.resize(0);
        tmp_scalars.resize(0);
        for(auto && [var, coef] : le.linear_terms()) {
            tmp_variables.emplace_back(var);
            tmp_scalars.emplace_back(coef);
        }
        check(GRB.setdblattrlist(model, GRB_DBL_ATTR_OBJ,
                                 static_cast<int>(tmp_variables.size()),
                                 tmp_variables.data(), tmp_scalars.data()));
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
        check(GRB.addvar(model, 0, NULL, NULL, p.obj_coef, p.lower_bound,
                         p.upper_bound, GRB_CONTINUOUS, NULL));
        return variable(var_id);
    }
    // variables_range add_variables(std::size_t count,
    //                               const variable_params p);

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
        tmp_variables.resize(0);
        tmp_scalars.resize(0);
        for(auto && [var, coef] : lc.expression().linear_terms()) {
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

    void optimize() { check(GRB.optimize(model)); }

    double get_objective_value() {
        double value;
        check(GRB.getdblattr(model, GRB_DBL_ATTR_OBJVAL, &value));
        return value;
    }

    template <typename Arr>
    class variable_mapping {
    private:
        Arr arr;

    public:
        variable_mapping(Arr && t) : arr(std::move(t)) {}

        double operator[](int i) const { return arr[i]; }
        double operator[](model_variable<int, double> x) const {
            return arr[static_cast<std::size_t>(x.id())];
        }
    };
    auto get_primal_solution() {
        auto num_vars = num_variables();
        auto solution = std::make_unique_for_overwrite<double[]>(num_vars);
        check(GRB.getdblattrarray(model, GRB_DBL_ATTR_X, 0,
                                  static_cast<int>(num_vars), solution.get()));
        return variable_mapping(std::move(solution));
    }
    auto get_dual_solution() {
        auto num_constrs = num_constraints();
        auto solution = std::make_unique_for_overwrite<double[]>(num_constrs);
        check(GRB.getdblattrarray(model, GRB_DBL_ATTR_PI, 0,
                                  static_cast<int>(num_constrs),
                                  solution.get()));
        return solution;
    }
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_GRB_MILP_MODEL_HPP