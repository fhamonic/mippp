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

#include "mippp/grb_api.hpp"

namespace fhamonic {
namespace mippp {

class grb_milp_model {
private:
    const grb_api & GRB;
    GRBenv * env;
    GRBmodel * model;

    void check(int error) {
        if(error)
            throw std::runtime_error(std::to_string(error) + ':' +
                                     GRB.geterrormsg(env));
    }
    void update_grb_model() { check(GRB.updatemodel(model)); }
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

    std::vector<int> tmp_indices;
    std::vector<double> tmp_scalars;

public:
    using variable_id_t = int;
    using scalar_t = double;
    using variable = model_variable<variable_id_t, scalar_t>;
    using constraint = int;

    struct lp_variable_params {
        scalar_t obj_coef = scalar_t{0};
        scalar_t lower_bound = scalar_t{0};
        scalar_t upper_bound = std::numeric_limits<scalar_t>::infinity();
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

    std::size_t num_variables() /*const*/ {
        int num;
        update_grb_model();
        check(GRB.getintattr(model, GRB_INT_ATTR_NUMVARS, &num));
        return static_cast<std::size_t>(num);
    }
    std::size_t num_constraints() /*const*/ {
        int num;
        update_grb_model();
        check(GRB.getintattr(model, GRB_INT_ATTR_NUMCONSTRS, &num));
        return static_cast<std::size_t>(num);
    }
    std::size_t num_entries() /*const*/ {
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

    void set_objective_constant(double constant) {
        check(GRB.setdblattr(model, GRB_DBL_ATTR_OBJCON, constant));
    }
    void set_objective(linear_expression auto && le) {
        tmp_indices.resize(0);
        tmp_scalars.resize(0);
        for(auto && [var, coef] : le.linear_terms()) {
            tmp_indices.emplace_back(var);
            tmp_scalars.emplace_back(coef);
        }
        check(GRB.setdblattrlist(model, GRB_DBL_ATTR_OBJ,
                                 static_cast<int>(tmp_indices.size()),
                                 tmp_indices.data(), tmp_scalars.data()));
        set_objective_constant(le.constant());
    }
    double get_objective_constant() /*const*/ {
        double constant;
        update_grb_model();
        check(GRB.getdblattr(model, GRB_DBL_ATTR_OBJCON, &constant));
        return constant;
    }
    auto get_objective() /*const*/ {
        auto num_vars = num_variables();
        std::vector<double> coefs(num_vars);
        update_grb_model();
        check(GRB.getdblattrarray(model, GRB_DBL_ATTR_OBJ, 0,
                                  static_cast<int>(num_vars), coefs.data()));
        return linear_expression_view(
            ranges::view::zip(ranges::view::iota(0, static_cast<int>(num_vars)),
                              ranges::view::move(coefs)),
            get_objective_constant());
    }

    variable add_variable(
        const lp_variable_params p = {
            .obj_coef = 0,
            .lower_bound = 0,
            .upper_bound = std::numeric_limits<double>::infinity()}) {
        int var_id = static_cast<int>(num_variables());
        std::cout << "#vars = " << var_id << std::endl;
        check(GRB.addvar(model, 0, NULL, NULL, p.obj_coef, p.lower_bound,
                         p.upper_bound, GRB_CONTINUOUS, NULL));
        return variable(var_id);
    }
    // variables_range add_variables(std::size_t count,
    //                               const lp_variable_params p);

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

    double get_objective_coefficient(variable v) /*const*/ {
        double coef;
        update_grb_model();
        check(GRB.getdblattrelement(model, GRB_DBL_ATTR_OBJ, v.id(), &coef));
        return coef;
    }
    double get_variable_lower_bound(variable v) /*const*/ {
        double lb;
        update_grb_model();
        check(GRB.getdblattrelement(model, GRB_DBL_ATTR_LB, v.id(), &lb));
        return lb;
    }
    double get_variable_upper_bound(variable v) /*const*/ {
        double ub;
        update_grb_model();
        check(GRB.getdblattrelement(model, GRB_DBL_ATTR_UB, v.id(), &ub));
        return ub;
    }
    auto get_variable_name(variable v) /*const*/ {
        char * name;
        update_grb_model();
        check(
            GRB.getstrattrelement(model, GRB_STR_ATTR_VARNAME, v.id(), &name));
        return std::string(name);
    }

    constraint add_constraint(linear_constraint auto && lc) {
        int constr_id = static_cast<int>(num_constraints());
        tmp_indices.resize(0);
        tmp_scalars.resize(0);
        for(auto && [var, coef] : lc.expression().linear_terms()) {
            tmp_indices.emplace_back(var);
            tmp_scalars.emplace_back(coef);
            std::cout << var << " " << coef << std::endl;
        }
        check(GRB.addconstr(model, static_cast<int>(tmp_indices.size()),
                            tmp_indices.data(), tmp_scalars.data(),
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
    // void set_constraint_name(constraint c, auto && name);

    // auto get_constraint_lhs(constraint c) const;
    double get_constraint_rhs(constraint c) /*const*/ {
        double rhs;
        update_grb_model();
        check(GRB.getdblattrelement(model, GRB_DBL_ATTR_RHS, c, &rhs));
        return rhs;
    }
    constraint_relation get_constraint_sense(constraint c) /*const*/ {
        char sense;
        update_grb_model();
        check(GRB.getcharattrelement(model, GRB_CHAR_ATTR_SENSE, c, &sense));
        return grb_sense_to_constraint_relation(sense);
    }
    auto get_constraint_name(constraint c) /*const*/ {
        char * name;
        update_grb_model();
        check(GRB.getstrattrelement(model, GRB_STR_ATTR_CONSTRNAME, c, &name));
        return std::string(name);
    }
    // auto get_constraint(const constraint c) const;

    // void set_basic(variable v);
    // void set_non_basic(variable v);

    // void set_basic(constraint v);
    // void set_non_basic(constraint v);

    void set_feasibility_tolerance(double tol) {
        check(GRB.setdblparam(env, GRB_DBL_PAR_FEASIBILITYTOL, tol));
    }
    double get_feasibility_tolerance() /*const*/ {
        double tol;
        check(GRB.getdblparam(env, GRB_DBL_PAR_FEASIBILITYTOL, &tol));
        return tol;
    }

    void optimize() { check(GRB.optimize(model)); }

    double get_objective_value() /*const*/ {
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
    auto get_primal_solution() /*const*/ {
        auto num_vars = num_variables();
        auto solution = std::make_unique_for_overwrite<double[]>(num_vars);
        check(GRB.getdblattrarray(model, GRB_DBL_ATTR_X, 0,
                                  static_cast<int>(num_vars), solution.get()));
        return variable_mapping(std::move(solution));
    }
    auto get_dual_solution() /*const*/ {
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