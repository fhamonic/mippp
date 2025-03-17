#ifndef MIPPP_CBC_MILP_MODEL_HPP
#define MIPPP_CBC_MILP_MODEL_HPP

#include <algorithm>
#include <limits>
#include <ostream>
// #include <ranges>
#include <sstream>
#include <string_view>
#include <vector>

#include <range/v3/view/iota.hpp>
#include <range/v3/view/span.hpp>
#include <range/v3/view/zip.hpp>

#include "mippp/detail/function_traits.hpp"
#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_variable.hpp"

#include "mippp/clp_api.hpp"

namespace fhamonic {
namespace mippp {

class clp_milp_model {
private:
    const clp_api & Clp;
    Clp_Simplex * model;

    // static constexpr char constraint_relation_to_grb_sense(
    //     constraint_relation rel) {
    //     if(rel == constraint_relation::less_equal_zero) return
    //     Clp_LESS_EQUAL; if(rel == constraint_relation::equal_zero) return
    //     Clp_EQUAL; return Clp_GREATER_EQUAL;
    // }
    // static constexpr constraint_relation grb_sense_to_constraint_relation(
    //     char sense) {
    //     if(sense == Clp_LESS_EQUAL) return
    //     constraint_relation::less_equal_zero; if(sense == Clp_EQUAL) return
    //     constraint_relation::equal_zero; return
    //     constraint_relation::greater_equal_zero;
    // }

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
    [[nodiscard]] explicit clp_milp_model(const clp_api & api)
        : Clp(api), model(Clp.newModel()) {}
    [[nodiscard]] ~clp_milp_model() { Clp.deleteModel(model); }

    std::size_t num_variables() /*const*/ {
        return static_cast<std::size_t>(Clp.getNumCols(model));
    }
    std::size_t num_constraints() /*const*/ {
        return static_cast<std::size_t>(Clp.getNumRows(model));
    }
    std::size_t num_entries() /*const*/ {
        return static_cast<std::size_t>(Clp.getNumElements(model));
    }

    void set_maximization() { Clp.setObjSense(model, -1); }
    void set_minimization() { Clp.setObjSense(model, 1); }

    void set_objective_constant(double constant) {
        Clp.setObjectiveOffset(model, constant);
    }
    void set_objective(linear_expression auto && le) {
        auto num_vars = num_variables();
        double * objective = Clp.objective(model);
        std::fill(objective, objective + num_variables, 0.0);
        for(auto && [var, coef] : le.linear_terms()) {
            objective[var] = coef;
        }
        set_objective_constant(le.constant());
    }
    double get_objective_constant() /*const*/ {
        return Clp.objectiveOffset(model);
    }
    auto get_objective() /*const*/ {
        auto num_vars = num_variables();
        const double * objective = Clp.objective(model);
        return linear_expression_view(
            ranges::view::zip(ranges::view::iota(0, static_cast<int>(num_vars)),
                              ranges::span(objective, objective + num_vars)),
            get_objective_constant());
    }

    variable add_variable(
        const lp_variable_params p = {
            .obj_coef = 0,
            .lower_bound = 0,
            .upper_bound = std::numeric_limits<double>::infinity()}) {
        int var_id = static_cast<int>(num_variables());
        Clp.addColumns(model, 1, &p.lower_bound, &p.upper_bound, &p.obj_coef,
                       NULL, NULL, NULL);
        return variable(var_id);
    }
    // variables_range add_variables(std::size_t count,
    //                               const lp_variable_params p);

    void set_objective_coefficient(variable v, double c) {
        Clp.objective(model)[v.id()] = c;
    }
    void set_variable_lower_bound(variable v, double lb) {
        Clp.columnLower(model)[v.id()] = lb;
    }
    void set_variable_upper_bound(variable v, double ub) {
        Clp.columnUpper(model)[v.id()] = ub;
    }
    void set_variable_name(variable v, std::string name) {
        Clp.setColumnName(model, v.id(), const_cast<char *>(name.c_str()));
    }

    double get_objective_coefficient(variable v) /*const*/ {
        return Clp.objective(model)[v.id()];
    }
    double get_variable_lower_bound(variable v) /*const*/ {
        return Clp.columnLower(model)[v.id()];
    }
    double get_variable_upper_bound(variable v) /*const*/ {
        return Clp.columnUpper(model)[v.id()];
    }
    auto get_variable_name(variable v) /*const*/ {
        char * name;
        Clp.columnName(model, v.id(), name);
        return std::string(name);
    }

    // constraint add_constraint(linear_constraint auto && lc) {
    //     int constr_id = static_cast<int>(num_constraints());
    //     tmp_indices.resize(0);
    //     tmp_scalars.resize(0);
    //     for(auto && [var, coef] : lc.expression().linear_terms()) {
    //         tmp_indices.emplace_back(var);
    //         tmp_scalars.emplace_back(coef);
    //         std::cout << var << " " << coef << std::endl;
    //     }
    //     check(Clp.addconstr(model, static_cast<int>(tmp_indices.size()),
    //                         tmp_indices.data(), tmp_scalars.data(),
    //                         constraint_relation_to_grb_sense(lc.relation()),
    //                         -lc.expression().constant(), NULL));
    //     return constr_id;
    // }
    // void set_constraint_rhs(constraint c, double rhs) {
    //     check(Clp.setdblattrelement(model, Clp_DBL_ATTR_RHS, c, rhs));
    // }
    // void set_constraint_sense(constraint c, constraint_relation r) {
    //     check(Clp.setcharattrelement(model, Clp_CHAR_ATTR_SENSE, c,
    //                                  constraint_relation_to_grb_sense(r)));
    // }
    // void set_constraint_name(constraint c, auto && name);

    // auto get_constraint_lhs(constraint c) const;
    // double get_constraint_rhs(constraint c) /*const*/ {
    //     double rhs;
    //     update_grb_model();
    //     check(Clp.getdblattrelement(model, Clp_DBL_ATTR_RHS, c, &rhs));
    //     return rhs;
    // }
    // constraint_relation get_constraint_sense(constraint c) /*const*/ {
    //     char sense;
    //     update_grb_model();
    //     check(Clp.getcharattrelement(model, Clp_CHAR_ATTR_SENSE, c, &sense));
    //     return grb_sense_to_constraint_relation(sense);
    // }
    // auto get_constraint_name(constraint c) /*const*/ {
    //     char * name;
    //     update_grb_model();
    //     check(Clp.getstrattrelement(model, Clp_STR_ATTR_CONSTRNAME, c,
    //     &name)); return std::string(name);
    // }
    // auto get_constraint(const constraint c) const;

    // void set_basic(variable v);
    // void set_non_basic(variable v);

    // void set_basic(constraint v);
    // void set_non_basic(constraint v);

    void set_feasibility_tolerance(double tol) {
        Clp.setPrimalTolerance(model, tol);
    }
    double get_feasibility_tolerance() /*const*/ {
        return Clp.primalTolerance(model);
    }

    void optimize() { Clp.primal(model, 0); }

    double get_objective_value() /*const*/ { return Clp.getObjValue(model); }

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
        return variable_mapping(Clp.primalColumnSolution(model));
    }
    auto get_dual_solution() /*const*/ { return Clp.primalRowSolution(model); }
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_CBC_MILP_MODEL_HPP