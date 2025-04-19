#ifndef MIPPP_CBC_210_milp_HPP
#define MIPPP_CBC_210_milp_HPP

#include <algorithm>
#include <cstring>
#include <limits>
#include <ostream>
#include <sstream>
#include <string_view>
#include <vector>

#include <range/v3/view/iota.hpp>
#include <range/v3/view/span.hpp>
#include <range/v3/view/zip.hpp>

#include "mippp/detail/function_traits.hpp"
#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/cbc/2.10/cbc210_api.hpp"

namespace fhamonic {
namespace mippp {

class cbc210_milp {
public:
    using variable_id = int;
    using constraint_id = int;
    using scalar = double;
    using variable = model_variable<variable_id, scalar>;
    using constraint = model_constraint<constraint_id, scalar>;

    struct variable_params {
        scalar obj_coef = scalar{0};
        scalar lower_bound = scalar{0};
        scalar upper_bound = std::numeric_limits<scalar>::infinity();
    };

private:
    const cbc210_api & Cbc;
    Cbc_Model * model;

    static constexpr char constraint_relation_to_cbc_sense(
        constraint_relation rel) {
        if(rel == constraint_relation::less_equal_zero) return 'L';
        if(rel == constraint_relation::equal_zero) return 'E';
        return 'G';
    }
    // static constexpr constraint_relation cbc_sense_to_constraint_relation(
    //     char sense) {
    //     if(sense == Clp_LESS_EQUAL) return
    //     constraint_relation::less_equal_zero; if(sense == Clp_EQUAL) return
    //     constraint_relation::equal_zero; return
    //     constraint_relation::greater_equal_zero;
    // }

    double objective_offset;
    std::vector<std::pair<constraint_id, unsigned int>>
        tmp_constraint_entry_cache;
    std::vector<int> tmp_variables;
    std::vector<double> tmp_scalars;

public:
    [[nodiscard]] explicit cbc210_milp(const cbc210_api & api)
        : Cbc(api), model(Cbc.newModel()), objective_offset(0.0) {}
    ~cbc210_milp() { Cbc.deleteModel(model); }

    std::size_t num_variables() {
        return static_cast<std::size_t>(Cbc.getNumCols(model));
    }
    std::size_t num_constraints() {
        return static_cast<std::size_t>(Cbc.getNumRows(model));
    }
    std::size_t num_entries() {
        return static_cast<std::size_t>(Cbc.getNumElements(model));
    }

    void set_maximization() { Cbc.setObjSense(model, -1); }
    void set_minimization() { Cbc.setObjSense(model, 1); }

    void set_objective_offset(double offset) { objective_offset = offset; }
    void set_objective(linear_expression auto && le) {
        for(auto && v :
            std::views::iota(0, static_cast<int>(num_variables()))) {
            Cbc.setObjCoeff(model, v, 0.0);
        }
        for(auto && [var, coef] : le.linear_terms()) {
            set_objective_coefficient(var,
                                      get_objective_coefficient(var) + coef);
        }
        set_objective_offset(le.constant());
    }
    void add_objective(linear_expression auto && le) {
        for(auto && [var, coef] : le.linear_terms()) {
            set_objective_coefficient(var,
                                      get_objective_coefficient(var) + coef);
        }
        set_objective_offset(get_objective_offset() + le.constant());
    }
    double get_objective_offset() { return objective_offset; }
    auto get_objective() {
        return linear_expression_view(
            ranges::view::transform(
                ranges::view::iota(0, static_cast<int>(num_variables())),
                [this](auto && v) {
                    return std::make_pair(v, Cbc.getObjCoefficients(model)[v]);
                }),
            get_objective_offset());
    }

    variable add_variable(
        const variable_params p = {
            .obj_coef = 0,
            .lower_bound = 0,
            .upper_bound = std::numeric_limits<double>::infinity()}) {
        int var_id = static_cast<int>(num_variables());
        Cbc.addCol(model, "", p.lower_bound, p.upper_bound, p.obj_coef, false,
                   0, NULL, NULL);
        return variable(var_id);
    }
    // variables_range add_variables(std::size_t count,
    //                               const variable_params p);

    void set_objective_coefficient(variable v, double c) {
        Cbc.setObjCoeff(model, v.id(), c);
    }
    void set_variable_lower_bound(variable v, double lb) {
        Cbc.setColLower(model, v.id(), lb);
    }
    void set_variable_upper_bound(variable v, double ub) {
        Cbc.setColUpper(model, v.id(), ub);
    }
    void set_variable_name(variable v, std::string name) {
        Cbc.setColName(model, v.id(), const_cast<char *>(name.c_str()));
    }

    double get_objective_coefficient(variable v) {
        return Cbc.getObjCoefficients(model)[v.id()];
    }
    double get_variable_lower_bound(variable v) {
        return Cbc.getColLower(model)[v.id()];
    }
    double get_variable_upper_bound(variable v) {
        return Cbc.getColUpper(model)[v.id()];
    }
    auto get_variable_name(variable v) {
        auto max_length = Cbc.maxNameLength(model);
        std::string name(max_length, '\0');
        Cbc.getColName(model, v.id(), name.data(), max_length);
        name.resize(std::strlen(name.c_str()));
        return name;
    }

    constraint add_constraint(linear_constraint auto && lc) {
        int constr_id = static_cast<int>(num_constraints());
        tmp_constraint_entry_cache.resize(num_variables());
        tmp_variables.resize(0);
        tmp_scalars.resize(0);
        for(auto && [var, coef] : lc.expression().linear_terms()) {
            auto & p = tmp_constraint_entry_cache[var.uid()];
            if(p.first == constr_id + 1) {
                tmp_scalars[p.second] += coef;
                continue;
            }
            p = std::make_pair(constr_id + 1, tmp_variables.size());
            tmp_variables.emplace_back(var.id());
            tmp_scalars.emplace_back(coef);
        }
        Cbc.addRow(model, "", static_cast<int>(tmp_variables.size()),
                   tmp_variables.data(), tmp_scalars.data(),
                   constraint_relation_to_cbc_sense(lc.relation()),
                   -lc.expression().constant());
        return constraint(constr_id);
    }
    // void set_constraint_rhs(constraint constr, double rhs) {
    // if(get_constraint_sense(constr) ==
    // constraint_relation::greater_equal_zero) {
    //     Clp.rowLower(model)[constr] = rhs;
    //     return;
    // }
    // Clp.rowUpper(model)[constr] = rhs;
    // }
    // void set_constraint_sense(constraint constr, constraint_relation r) {
    // constraint_relation old_r = get_constraint_sense(constr);
    // double old_rhs = get_constraint_rhs(constr);
    // if(old_r == r) return;
    // switch(r) {
    //     case constraint_relation::equal_zero:
    //         Clp.rowLower(model)[constr] = Clp.rowUpper(model)[constr] =
    //         old_rhs; return;
    //     case constraint_relation::less_equal_zero:
    //         Clp.rowLower(model)[constr] = -COIN_DBL_MAX;
    //         Clp.rowUpper(model)[constr] = old_rhs;
    //         return;
    //     case constraint_relation::greater_equal_zero:
    //         Clp.rowLower(model)[constr] = old_rhs;
    //         Clp.rowUpper(model)[constr] = COIN_DBL_MAX;
    //         return;
    // }
    // }
    constraint add_ranged_constraint(linear_expression auto && le, double lb,
                                     double ub) {
        const int constr_id = static_cast<int>(num_constraints());
        tmp_variables.resize(0);
        tmp_scalars.resize(0);
        for(auto && [var, coef] : le.linear_terms()) {
            tmp_variables.emplace_back(var.id());
            tmp_scalars.emplace_back(coef);
        }
        const double c = le.constant();
        Cbc.addRow(model, "", static_cast<int>(tmp_variables.size()),
                   tmp_variables.data(), tmp_scalars.data(), 'L', ub - c);
        Cbc.setRowLower(model, constr_id, lb - c);
        return constraint(constr_id);
    }
    // void set_constraint_name(constraint constr, auto && name);

    // auto get_constraint_lhs(constraint constr) const;
    double get_constraint_rhs(constraint constr) {
        return Cbc.getRowRHS(model, constr.id());
    }
    constraint_relation get_constraint_sense(constraint constr) {
        const double lb = Cbc.getRowLower(model)[constr.id()];
        const double ub = Cbc.getRowUpper(model)[constr.id()];
        if(lb == ub) return constraint_relation::equal_zero;
        if(lb == -COIN_DBL_MAX) return constraint_relation::less_equal_zero;
        if(ub == COIN_DBL_MAX) return constraint_relation::greater_equal_zero;
        throw std::runtime_error(
            "Tried to get the sense of a ranged constraint");
    }
    auto get_constraint_name(constraint constr) {
        auto max_length = Cbc.maxNameLength(model);
        std::string name(max_length, '\0');
        Cbc.getRowName(model, constr.id(), name.data(), max_length);
        name.resize(std::strlen(name.c_str()));
        return name;
    }
    // auto get_constraint(const constraint constr) const;

    void set_feasibility_tolerance(double tol) {
        auto tol_s = std::to_string(tol);
        Cbc.setParameter(model, "-primalTolerance=", tol_s.c_str());
        Cbc.setParameter(model, "-dualTolerance=", tol_s.c_str());
    }
    // double get_feasibility_tolerance() { return Clp.primalTolerance(model); }

    void set_optimality_tolerance(double tol) {
        Cbc.setAllowableGap(model, tol);
    }
    double set_optimality_tolerance() { return Cbc.getAllowableGap(model); }

    void optimize() {
        if(num_constraints() == 0u) {
            return;
        }
        Cbc.solve(model);
    }

    double get_solution_value() {
        return objective_offset + Cbc.getObjValue(model);
    }

    auto get_solution() { return variable_mapping(Cbc.getColSolution(model)); }
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_CBC_210_milp_HPP