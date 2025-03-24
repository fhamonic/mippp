#ifndef MIPPP_CLP_LP_MODEL_HPP
#define MIPPP_CLP_LP_MODEL_HPP

#include <algorithm>
#include <cstring>
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
#include "mippp/model_concepts.hpp"
#include "mippp/model_variable.hpp"

#include "mippp/api/clp_api.hpp"

namespace fhamonic {
namespace mippp {

class clp_lp_model {
private:
    const clp_api & Clp;
    Clp_Simplex * model;
    std::optional<lp_status> opt_lp_status;

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

    // std::vector<CoinBigIndex> tmp_indices;
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
    [[nodiscard]] explicit clp_lp_model(const clp_api & api)
        : Clp(api), model(Clp.newModel()) {}
    ~clp_lp_model() { Clp.deleteModel(model); }

    std::size_t num_variables() {
        return static_cast<std::size_t>(Clp.getNumCols(model));
    }
    std::size_t num_constraints() {
        return static_cast<std::size_t>(Clp.getNumRows(model));
    }
    std::size_t num_entries() {
        return static_cast<std::size_t>(Clp.getNumElements(model));
    }

    void set_maximization() { Clp.setObjSense(model, -1); }
    void set_minimization() { Clp.setObjSense(model, 1); }

    void set_objective_offset(double constant) {
        Clp.setObjectiveOffset(model, constant);
    }
    void set_objective(linear_expression auto && le) {
        auto num_vars = num_variables();
        double * objective = Clp.objective(model);
        std::fill(objective, objective + num_vars, 0.0);
        for(auto && [var, coef] : le.linear_terms()) {
            objective[var] = coef;
        }
        set_objective_offset(le.constant());
    }
    void add_objective(linear_expression auto && le) {
        double * objective = Clp.objective(model);
        for(auto && [var, coef] : le.linear_terms()) {
            objective[var] += coef;
        }
        set_objective_offset(get_objective_offset() + le.constant());
    }
    double get_objective_offset() { return Clp.objectiveOffset(model); }
    auto get_objective() {
        auto num_vars = num_variables();
        const double * objective = Clp.objective(model);
        return linear_expression_view(
            ranges::view::zip(ranges::view::iota(0, static_cast<int>(num_vars)),
                              ranges::span(objective, objective + num_vars)),
            get_objective_offset());
    }

    variable add_variable(
        const variable_params p = {
            .obj_coef = 0,
            .lower_bound = 0,
            .upper_bound = std::numeric_limits<double>::infinity()}) {
        int var_id = static_cast<int>(num_variables());
        Clp.addColumns(model, 1, &p.lower_bound, &p.upper_bound, &p.obj_coef,
                       NULL, NULL, NULL);
        return variable(var_id);
    }
    // variables_range add_variables(std::size_t count,
    //                               const variable_params p);

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

    double get_objective_coefficient(variable v) {
        return Clp.objective(model)[v.id()];
    }
    double get_variable_lower_bound(variable v) {
        return Clp.columnLower(model)[v.id()];
    }
    double get_variable_upper_bound(variable v) {
        return Clp.columnUpper(model)[v.id()];
    }
    auto get_variable_name(variable v) {
        auto max_length = static_cast<std::size_t>(Clp.lengthNames(model));
        std::string name(max_length, '\0');
        Clp.columnName(model, v.id(), name.data());
        name.resize(std::strlen(name.c_str()));
        return name;
    }

    constraint add_constraint(linear_constraint auto && lc) {
        int constr_id = static_cast<int>(num_constraints());
        tmp_variables.resize(0);
        tmp_scalars.resize(0);
        for(auto && [var, coef] : lc.expression().linear_terms()) {
            tmp_variables.emplace_back(var);
            tmp_scalars.emplace_back(coef);
        }
        const double b = -lc.expression().constant();
        CoinBigIndex starts[2] = {
            0, static_cast<CoinBigIndex>(tmp_variables.size())};
        Clp.addRows(
            model, 1,
            (lc.relation() == constraint_relation::less_equal_zero) ? NULL : &b,
            (lc.relation() == constraint_relation::greater_equal_zero) ? NULL
                                                                       : &b,
            starts, tmp_variables.data(), tmp_scalars.data());
        return constr_id;
    }
    void set_constraint_rhs(constraint c, double rhs) {
        if(get_constraint_sense(c) == constraint_relation::greater_equal_zero) {
            Clp.rowLower(model)[c] = rhs;
            return;
        }
        Clp.rowUpper(model)[c] = rhs;
    }
    void set_constraint_sense(constraint c, constraint_relation r) {
        constraint_relation old_r = get_constraint_sense(c);
        double old_rhs = get_constraint_rhs(c);
        if(old_r == r) return;
        switch(r) {
            case constraint_relation::equal_zero:
                Clp.rowLower(model)[c] = Clp.rowUpper(model)[c] = old_rhs;
                return;
            case constraint_relation::less_equal_zero:
                Clp.rowLower(model)[c] = -COIN_DBL_MAX;
                Clp.rowUpper(model)[c] = old_rhs;
                return;
            case constraint_relation::greater_equal_zero:
                Clp.rowLower(model)[c] = old_rhs;
                Clp.rowUpper(model)[c] = COIN_DBL_MAX;
                return;
        }
    }
    constraint add_ranged_constraint(linear_expression auto && le, double lb,
                                     double ub) {
        int constr_id = static_cast<int>(num_constraints());
        tmp_variables.resize(0);
        tmp_scalars.resize(0);
        for(auto && [var, coef] : le.linear_terms()) {
            tmp_variables.emplace_back(var);
            tmp_scalars.emplace_back(coef);
        }
        lb -= le.constant();
        ub -= le.constant();
        CoinBigIndex starts[2] = {
            0, static_cast<CoinBigIndex>(tmp_variables.size())};
        Clp.addRows(model, 1, &lb, &ub, starts, tmp_variables.data(),
                    tmp_scalars.data());
        return constr_id;
    }
    void set_constraint_name(constraint c, auto && name) {
        Clp.setRowName(model, c, const_cast<char *>(name.c_str()));
    }

    // auto get_constraint_lhs(constraint c) const;
    double get_constraint_rhs(constraint c) {
        if(get_constraint_sense(c) == constraint_relation::greater_equal_zero)
            return Clp.rowLower(model)[c];
        return Clp.rowUpper(model)[c];
    }
    constraint_relation get_constraint_sense(constraint c) {
        const double lb = Clp.rowLower(model)[c];
        const double ub = Clp.rowUpper(model)[c];
        if(lb == ub) return constraint_relation::equal_zero;
        if(lb == -COIN_DBL_MAX) return constraint_relation::less_equal_zero;
        if(ub == COIN_DBL_MAX) return constraint_relation::greater_equal_zero;
        throw std::runtime_error(
            "Tried to get the sense of a ranged constraint");
    }
    auto get_constraint_name(constraint c) {
        auto max_length = static_cast<std::size_t>(Clp.lengthNames(model));
        std::string name(max_length, '\0');
        Clp.rowName(model, c, name.data());
        name.resize(std::strlen(name.c_str()));
        return name;
    }
    // auto get_constraint(const constraint c) const;

    // void set_basic(variable v);
    // void set_non_basic(variable v);

    // void set_basic(constraint v);
    // void set_non_basic(constraint v);

    void set_feasibility_tolerance(double tol) {
        Clp.setPrimalTolerance(model, tol);
    }
    double get_feasibility_tolerance() { return Clp.primalTolerance(model); }

    void optimize() {
        if(num_variables() == 0u) {
            opt_lp_status.emplace(lp_status::optimal);
            return;
        }
        Clp.initialSolve(model);
        // Clp.primal(model, 0);
        switch(Clp.status(model)) {
            case 0:
                opt_lp_status.emplace(lp_status::optimal);
                return;
            case 1:
                opt_lp_status.emplace(lp_status::infeasible);
                return;
            case 2:
                opt_lp_status.emplace(lp_status::unbounded);
                return;
            default:
                opt_lp_status.reset();
        }
    }
    std::optional<lp_status> get_lp_status() const { return opt_lp_status; }

    double get_solution_value() { return Clp.getObjValue(model); }

    auto get_solution() {
        return variable_mapping(Clp.primalColumnSolution(model));
    }
    auto get_dual_solution() { return Clp.dualRowSolution(model); }
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_CLP_LP_MODEL_HPP