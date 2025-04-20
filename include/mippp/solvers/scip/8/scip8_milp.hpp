#ifndef MIPPP_SCIP_8_milp_HPP
#define MIPPP_SCIP_8_milp_HPP

#include <limits>
// #include <ranges>
#include <optional>
#include <vector>

#include <range/v3/view/iota.hpp>
#include <range/v3/view/move.hpp>
#include <range/v3/view/zip.hpp>

#include "mippp/detail/function_traits.hpp"
#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/scip/8/scip8_api.hpp"

namespace fhamonic {
namespace mippp {

// https://github.com/scipopt/SCIPpp/blob/main/source/model.cpp

class scip8_milp {
private:
    const scip8_api & SCIP;
    struct Scip * model;
    std::vector<SCIP_VAR *> variables;
    std::vector<SCIP_CONS *> constraints;
    std::optional<lp_status> opt_lp_status;

public:
    using variable_id = int;
    using constraint_id = int;
    using scalar = double;
    using variable = model_variable<variable_id, scalar>;
    using constraint = model_constraint<constraint_id, scalar>;

    struct variable_params {
        scalar obj_coef = 0.0;
        std::optional<scalar> lower_bound = 0.0;
        std::optional<scalar> upper_bound = std::nullopt;
    };

public:
    [[nodiscard]] explicit scip8_milp(const scip8_api & api) : SCIP(api) {
        SCIP.create(&model);
        SCIP.includeDefaultPlugins(model);
        SCIP.createProbBasic(model, "MILP");
    }
    ~scip8_milp() {
        for(auto & var : variables) {
            SCIP.releaseVar(model, &var);
        }
        for(auto & cons : constraints) {
            SCIP.releaseCons(model, &cons);
        }
        SCIP.free(&model);
    };

private:
    static constexpr const char * error_messages[] = {
        "unspecified error",          // SCIP_ERROR
        "insufficient memory error",  // SCIP_NOMEMORY
        "read error",                 // SCIP_READERROR
        "write error",                // SCIP_WRITEERROR
        "file not found error",       // SCIP_NOFILE
        "cannot create file",         // SCIP_FILECREATEERROR
        "error in LP solver",         // SCIP_LPERROR
        "no problem exists",          // SCIP_NOPROBLEM
        "method cannot be called at this time in solution process",  // SCIP_INVALIDCALL
        "error in input data",                     // SCIP_INVALIDDATA
        "method returned an invalid result code",  // SCIP_INVALIDRESULT
        "a required plugin was not found",         // SCIP_PLUGINNOTFOUND
        "the parameter with the given name was not found",  // SCIP_PARAMETERUNKNOWN
        "the parameter is not of the expected type",  // SCIP_PARAMETERWRONGTYPE
        "the value is invalid for the given parameter",  // SCIP_PARAMETERWRONGVAL
        "the given key is already existing in table",  // SCIP_KEYALREADYEXISTING
        "maximal branching depth level exceeded",      // SCIP_MAXDEPTHLEVEL
        "no branching could be created",               // SCIP_BRANCHERROR
        "function not implemented"};                   // SCIP_NOTIMPLEMENTED

    int check(int retval) {
        if(retval > 0) return retval;
        throw std::runtime_error(std::string("scip8_milp: ") +
                                 error_messages[-retval]);
    }

public:
    std::size_t num_variables() { return variables.size(); }
    std::size_t num_constraints() { return constraints.size(); }
    std::size_t num_entries() {
        return static_cast<std::size_t>(SCIP.getNNZs(model));
    }

    void set_maximization() {
        check(SCIP.setObjsense(model, SCIP_OBJSENSE_MAXIMIZE));
    }
    void set_minimization() {
        check(SCIP.setObjsense(model, SCIP_OBJSENSE_MINIMIZE));
    }

    void set_objective_offset(double offset) {
        check(SCIP.addOrigObjoffset(model,
                                    offset - SCIP.getOrigObjoffset(model)));
    }
    void set_objective(linear_expression auto && le) {
        for(auto && var : variables) {
            check(SCIP.chgVarObj(model, var, 0.0));
        }
        for(auto && [var_, coef] : le.linear_terms()) {
            const auto & var = variables[var_.uid()];
            check(SCIP.chgVarObj(model, var, SCIP.varGetObj(var) + coef));
        }
        set_objective_offset(le.constant());
    }
    void add_objective(linear_expression auto && le) {
        for(auto && [var_, coef] : le.linear_terms()) {
            const auto & var = variables[var_.uid()];
            check(SCIP.chgVarObj(model, var, SCIP.varGetObj(var) + coef));
        }
        set_objective_offset(get_objective_offset() + le.constant());
    }
    double get_objective_offset() { return SCIP.getOrigObjoffset(model); }
    // auto get_objective() {
    //     auto num_vars = num_variables();
    //     std::vector<double> coefs(num_vars);
    //     update_scip_model();
    //     check(SCIP.getdblattrarray(model, SCIP_DBL_ATTR_OBJ, 0,
    //                                static_cast<int>(num_vars),
    //                                coefs.data()));
    //     return linear_expression_view(
    //         ranges::view::zip(ranges::view::iota(0,
    //         static_cast<int>(num_vars)),
    //                           ranges::view::move(coefs)),
    //         get_objective_offset());
    // }

private:
    void _add_var(const variable_params & params) {
        SCIP_VAR * var = NULL;
        check(SCIP.createVarBasic(
            model, &var, "", params.lower_bound.value_or(-SCIP.infinity(model)),
            params.upper_bound.value_or(SCIP.infinity(model)), params.obj_coef,
            SCIP_VARTYPE_CONTINUOUS));
        check(SCIP.addVar(model, var));
        variables.emplace_back(var);
    }

public:
    variable add_variable(const variable_params params = {
                              .obj_coef = 0,
                              .lower_bound = 0,
                              .upper_bound = std::nullopt}) {
        int var_id = static_cast<int>(num_variables());
        _add_var(params);
        return variable(var_id);
    }
    auto add_variables(std::size_t count,
                       variable_params params = {
                           .obj_coef = 0,
                           .lower_bound = 0,
                           .upper_bound = std::nullopt}) noexcept {
        const std::size_t offset = num_variables();
        for(std::size_t i = 0; i < count; ++i) _add_var(params);
        return make_variables_range(ranges::view::transform(
            ranges::view::iota(static_cast<variable_id>(offset),
                               static_cast<variable_id>(offset + count)),
            [](auto && i) { return variable{i}; }));
    }
    template <typename IL>
    auto add_variables(std::size_t count, IL && id_lambda,
                       variable_params params = {
                           .obj_coef = 0,
                           .lower_bound = 0,
                           .upper_bound = std::nullopt}) noexcept {
        const std::size_t offset = num_variables();
        for(std::size_t i = 0; i < count; ++i) _add_var(params);
        return make_indexed_variables_range(
            typename detail::function_traits<IL>::arg_types(),
            ranges::view::transform(
                ranges::view::iota(static_cast<variable_id>(offset),
                                   static_cast<variable_id>(offset + count)),
                [](auto && i) { return variable{i}; }),
            std::forward<IL>(id_lambda));
    }

    void set_objective_coefficient(variable v, double c) {
        check(SCIP.chgVarObj(model, variables[static_cast<std::size_t>(v.id())],
                             c));
    }
    void set_variable_lower_bound(variable v, double lb) {
        check(SCIP.chgVarLb(model, variables[static_cast<std::size_t>(v.id())],
                            lb));
    }
    void set_variable_upper_bound(variable v, double ub) {
        check(SCIP.chgVarUb(model, variables[static_cast<std::size_t>(v.id())],
                            ub));
    }
    // void set_variable_name(variable v, std::string name) {
    //     check(SCIP.setstrattrelement(model, SCIP_STR_ATTR_VARNAME, v.id(),
    //                                  name.c_str()));
    // }

    double get_objective_coefficient(variable v) {
        return SCIP.varGetObj(variables[static_cast<std::size_t>(v.id())]);
    }
    // double get_variable_lower_bound(variable v) {
    //     return SCIP.varGetLb(variables[static_cast<std::size_t>(v.id())]);
    //     SCIPgetVarUb
    // }
    // double get_variable_upper_bound(variable v) {
    //     return SCIP.varGetUb(variables[static_cast<std::size_t>(v.id())]);
    // }
    // auto get_variable_name(variable v) {
    //     char * name;
    //     update_scip_model();
    //     check(SCIP.getstrattrelement(model, SCIP_STR_ATTR_VARNAME, v.id(),
    //                                  &name));
    //     return std::string(name);
    // }

    constraint add_constraint(linear_constraint auto && lc) {
        int constr_id = static_cast<int>(num_constraints());
        SCIP_CONS * constr = NULL;
        const double b = -lc.expression().constant();
        check(SCIP.createConsBasicLinear(
            model, &constr, "", 0, NULL, NULL,
            (lc.relation() == constraint_relation::less_equal_zero)
                ? -SCIP.infinity(model)
                : b,
            (lc.relation() == constraint_relation::greater_equal_zero)
                ? SCIP.infinity(model)
                : b));
        constraints.emplace_back(constr);
        for(auto && [var_, coef] : lc.expression().linear_terms()) {
            check(
                SCIP.addCoefLinear(model, constr, variables[var_.uid()], coef));
        }
        check(SCIP.addCons(model, constr));
        return constraint(constr_id);
    }
    // void set_constraint_rhs(constraint c, double rhs) {
    //     check(SCIP.setdblattrelement(model, SCIP_DBL_ATTR_RHS, c, rhs));
    // }
    // void set_constraint_sense(constraint c, constraint_relation r) {
    //     check(SCIP.setcharattrelement(model, SCIP_CHAR_ATTR_SENSE, c,
    //                                   constraint_relation_to_scip_sense(r)));
    // }
    // constraint add_ranged_constraint(linear_expression auto && le, double lb,
    //                                  double ub) {
    //     int constr_id = static_cast<int>(num_constraints());
    //     tmp_variables.resize(0);
    //     tmp_scalars.resize(0);
    //     for(auto && [var, coef] : le.linear_terms()) {
    //         tmp_variables.emplace_back(var);
    //         tmp_scalars.emplace_back(coef);
    //     }
    //     check(SCIP.addrangeconstr(model,
    //     static_cast<int>(tmp_variables.size()),
    //                               tmp_variables.data(), tmp_scalars.data(),
    //                               lb, ub, NULL));
    //     return constr_id;
    // }
    // void set_constraint_name(constraint c, auto && name);

    // auto get_constraint_lhs(constraint c) {}
    // double get_constraint_rhs(constraint c) {
    //     double rhs;
    //     update_scip_model();
    //     check(SCIP.getdblattrelement(model, SCIP_DBL_ATTR_RHS, c, &rhs));
    //     return rhs;
    // }
    // constraint_relation get_constraint_sense(constraint c) {
    //     char sense;
    //     update_scip_model();
    //     check(SCIP.getcharattrelement(model, SCIP_CHAR_ATTR_SENSE, c,
    //     &sense)); return scip_sense_to_constraint_relation(sense);
    // }
    // // auto get_constraint(const constraint c) {}
    // auto get_constraint_name(constraint c) {
    //     char * name;
    //     update_scip_model();
    //     check(
    //         SCIP.getstrattrelement(model, SCIP_STR_ATTR_CONSTRNAME, c,
    //         &name));
    //     return std::string(name);
    // }

    void set_feasibility_tolerance(double tol) {
        check(SCIPsetRealParam(model, "numerics/feastol", tol));
    }
    double get_feasibility_tolerance() {
        double tol;
        check(SCIPgetRealParam(model, "numerics/feastol", &tol));
        return tol;
    }

    void set_optimality_tolerance(double tol) {
        check(SCIPsetRealParam(model, "limits/gap", tol));
    }
    double get_optimality_tolerance() {
        double tol;
        check(SCIPgetRealParam(model, "limits/gap", &tol));
        return tol;
    }

    void optimize() { check(SCIP.solve(model)); }

    double get_solution_value() { return SCIP.getPrimalbound(model); }

    auto get_solution() {
        auto num_vars = num_variables();
        auto solution = std::make_unique_for_overwrite<double[]>(num_vars);
        SCIP_SOL * sol = SCIP.getBestSol(model);
        for(std::size_t i = 0; i < variables.size(); ++i) {
            solution[i] = SCIP.getSolVal(model, sol, variables[i]);
        }
        return variable_mapping(std::move(solution));
    }
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_SCIP_8_milp_HPP