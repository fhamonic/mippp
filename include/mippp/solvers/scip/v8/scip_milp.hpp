#ifndef MIPPP_SCIP_v8_MILP_HPP
#define MIPPP_SCIP_v8_MILP_HPP

#include <limits>
#include <optional>
#include <ranges>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/scip/v8/scip_api.hpp"

namespace fhamonic::mippp {
namespace scip::v8 {

class scip_milp {
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

private:
    const scip_api & SCIP;
    struct Scip * model;
    std::vector<SCIP_VAR *> variables;
    std::vector<SCIP_CONS *> constraints;

public:
    [[nodiscard]] explicit scip_milp(const scip_api & api) : SCIP(api) {
        SCIP.create(&model);
        SCIP.includeDefaultPlugins(model);
        SCIP.createProbBasic(model, "MILP");
    }
    ~scip_milp() {
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
        throw std::runtime_error(std::string("scip_milp: ") +
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
    auto get_objective() {
        return linear_expression_view(
           std::views::transform(
               std::views::iota(variable_id{0},
                                   static_cast<variable_id>(num_variables())),
                [this](auto i) {
                    return std::make_pair(
                        variable(i),
                        SCIP.varGetObj(variables[static_cast<std::size_t>(i)]));
                }),
            get_objective_offset());
    }

private:
    void _add_variable(const variable_params & params, SCIP_VARTYPE type,
                       const char * name = "") {
        SCIP_VAR * var = NULL;
        check(SCIP.createVarBasic(
            model, &var, name,
            params.lower_bound.value_or(-SCIP.infinity(model)),
            params.upper_bound.value_or(SCIP.infinity(model)), params.obj_coef,
            type));
        check(SCIP.addVar(model, var));
        variables.emplace_back(var);
    }

    inline auto _make_variables_range(const std::size_t & offset,
                                      const std::size_t & count) {
        return variables_range(std::from_range_t{}, std::views::transform(
           std::views::iota(static_cast<variable_id>(offset),
                               static_cast<variable_id>(offset + count)),
            [](auto && i) { return variable{i}; }));
    }
    template <typename IL>
    inline auto _make_indexed_variables_range(const std::size_t & offset,
                                              const std::size_t & count,
                                              IL && id_lambda) {
        return variables_range(
            typename detail::function_traits<IL>::arg_types(),
           std::views::transform(
               std::views::iota(static_cast<variable_id>(offset),
                                   static_cast<variable_id>(offset + count)),
                [](auto && i) { return variable{i}; }),
            std::forward<IL>(id_lambda));
    }
    template <typename IL, typename NL>
    inline auto _make_indexed_named_variables_range(const std::size_t & offset,
                                                    const std::size_t & count,
                                                    IL && id_lambda,
                                                    NL && name_lambda) {
        return lazily_named_variables_range(
            typename detail::function_traits<IL>::arg_types(),
           std::views::transform(
               std::views::iota(static_cast<variable_id>(offset),
                                   static_cast<variable_id>(offset + count)),
                [](auto && i) { return variable{i}; }),
            std::forward<IL>(id_lambda), std::forward<NL>(name_lambda), this);
    }

public:
    variable add_variable(
        const variable_params params = default_variable_params) {
        int var_id = static_cast<int>(num_variables());
        _add_variable(params, SCIP_VARTYPE_CONTINUOUS);
        return variable(var_id);
    }
    auto add_variables(
        std::size_t count,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = num_variables();
        for(std::size_t i = 0; i < count; ++i)
            _add_variable(params, SCIP_VARTYPE_CONTINUOUS);
        return _make_variables_range(offset, count);
    }
    template <typename IL>
    auto add_variables(
        std::size_t count, IL && id_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = num_variables();
        for(std::size_t i = 0; i < count; ++i)
            _add_variable(params, SCIP_VARTYPE_CONTINUOUS);
        return _make_indexed_variables_range(offset, count,
                                             std::forward<IL>(id_lambda));
    }

    variable add_integer_variable(
        const variable_params params = default_variable_params) {
        int var_id = static_cast<int>(num_variables());
        _add_variable(params, SCIP_VARTYPE_INTEGER);
        return variable(var_id);
    }
    auto add_integer_variables(
        std::size_t count,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = num_variables();
        for(std::size_t i = 0; i < count; ++i)
            _add_variable(params, SCIP_VARTYPE_INTEGER);
        return _make_variables_range(offset, count);
    }
    template <typename IL>
    auto add_integer_variables(
        std::size_t count, IL && id_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = num_variables();
        for(std::size_t i = 0; i < count; ++i)
            _add_variable(params, SCIP_VARTYPE_INTEGER);
        return _make_indexed_variables_range(offset, count,
                                             std::forward<IL>(id_lambda));
    }

    variable add_binary_variable() {
        int var_id = static_cast<int>(num_variables());
        _add_variable({.obj_coef = 0.0, .lower_bound = 0.0, .upper_bound = 1.0},
                      SCIP_VARTYPE_BINARY);
        return variable(var_id);
    }
    auto add_binary_variables(std::size_t count) noexcept {
        const std::size_t offset = num_variables();
        for(std::size_t i = 0; i < count; ++i)
            _add_variable(
                {.obj_coef = 0.0, .lower_bound = 0.0, .upper_bound = 1.0},
                SCIP_VARTYPE_BINARY);
        return _make_variables_range(offset, count);
    }
    template <typename IL>
    auto add_binary_variables(std::size_t count, IL && id_lambda) noexcept {
        const std::size_t offset = num_variables();
        for(std::size_t i = 0; i < count; ++i)
            _add_variable(
                {.obj_coef = 0.0, .lower_bound = 0.0, .upper_bound = 1.0},
                SCIP_VARTYPE_BINARY);
        return _make_indexed_variables_range(offset, count,
                                             std::forward<IL>(id_lambda));
    }

    variable add_named_variable(
        const std::string & name,
        const variable_params params = default_variable_params) {
        int var_id = static_cast<int>(num_variables());
        _add_variable(params, SCIP_VARTYPE_CONTINUOUS, name.c_str());
        return variable(var_id);
    }
    template <typename NL>
    auto add_named_variables(
        std::size_t count, NL && name_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = num_variables();
        for(std::size_t i = 0; i < count; ++i)
            _add_variable(params, SCIP_VARTYPE_CONTINUOUS,
                          name_lambda(i).c_str());
        return _make_variables_range(offset, count);
    }
    template <typename IL, typename NL>
    auto add_named_variables(
        std::size_t count, IL && id_lambda, NL && name_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = num_variables();
        for(std::size_t i = 0; i < count; ++i)
            _add_variable(params, SCIP_VARTYPE_CONTINUOUS);
        return _make_indexed_named_variables_range(
            offset, count, std::forward<IL>(id_lambda),
            std::forward<NL>(name_lambda));
    }

    void set_continuous(variable v) noexcept {
        unsigned int infeas;
        check(SCIP.chgVarType(model, variables[v.uid()],
                              SCIP_VARTYPE_CONTINUOUS, &infeas));
    }
    void set_integer(variable v) noexcept {
        unsigned int infeas;
        check(SCIP.chgVarType(model, variables[v.uid()], SCIP_VARTYPE_INTEGER,
                              &infeas));
    }
    void set_binary(variable v) noexcept {
        set_variable_lower_bound(v, 0);
        set_variable_upper_bound(v, 1);
        unsigned int infeas;
        check(SCIP.chgVarType(model, variables[v.uid()], SCIP_VARTYPE_BINARY,
                              &infeas));
    }

    void set_objective_coefficient(variable v, double c) {
        check(SCIP.chgVarObj(model, variables[v.uid()], c));
    }
    void set_variable_lower_bound(variable v, double lb) {
        check(SCIP.chgVarLb(model, variables[v.uid()], lb));
    }
    void set_variable_upper_bound(variable v, double ub) {
        check(SCIP.chgVarUb(model, variables[v.uid()], ub));
    }
    void set_variable_name(variable v, std::string name) {
        check(SCIP.chgVarName(model, variables[v.uid()], name.c_str()));
    }

    double get_objective_coefficient(variable v) {
        return SCIP.varGetObj(variables[v.uid()]);
    }
    double get_variable_lower_bound(variable v) {
        return SCIP.varGetLbGlobal(variables[v.uid()]);
    }
    double get_variable_upper_bound(variable v) {
        return SCIP.varGetUbGlobal(variables[v.uid()]);
    }
    std::string get_variable_name(variable v) {
        return std::string(SCIP.varGetName(variables[v.uid()]));
    }

private:
    SCIP_CONS * _add_constraint(linear_constraint auto && lc) {
        SCIP_CONS * constr = NULL;
        const double b = lc.rhs();
        check(SCIP.createConsBasicLinear(
            model, &constr, "", 0, NULL, NULL,
            (lc.sense() == constraint_sense::less_equal) ? -SCIP.infinity(model)
                                                         : b,
            (lc.sense() == constraint_sense::greater_equal)
                ? SCIP.infinity(model)
                : b));
        for(auto && [var, coef] : lc.linear_terms()) {
            check(
                SCIP.addCoefLinear(model, constr, variables[var.uid()], coef));
        }
        check(SCIP.addCons(model, constr));
        return constr;
    }

public:
    constraint add_constraint(linear_constraint auto && lc) {
        constraint_id constr_id = static_cast<constraint_id>(num_constraints());
        constraints.emplace_back(_add_constraint(lc));
        return constraint(constr_id);
    }

private:
    template <typename Key, typename LastConstrLambda>
        requires linear_constraint<std::invoke_result_t<LastConstrLambda, Key>>
    SCIP_CONS * _add_first_valued_constraint(
        const Key & key, const LastConstrLambda & lc_lambda) {
        return _add_constraint(lc_lambda(key));
    }
    template <typename Key, typename OptConstrLambda, typename... Tail>
        requires detail::optional_type<
                     std::invoke_result_t<OptConstrLambda, Key>> &&
                 linear_constraint<detail::optional_type_value_t<
                     std::invoke_result_t<OptConstrLambda, Key>>>
    SCIP_CONS * _add_first_valued_constraint(
        const Key & key, const OptConstrLambda & opt_lc_lambda,
        const Tail &... tail) {
        if(const auto & opt_lc = opt_lc_lambda(key)) {
            return _add_constraint(opt_lc.value());
        }
        return _add_first_valued_constraint(key, tail...);
    }

public:
    template <std::ranges::range IR, typename... CL>
    auto add_constraints(IR && keys, CL... constraint_lambdas) {
        const constraint_id offset =
            static_cast<constraint_id>(num_constraints());
        constraint_id constr_id = offset;
        for(auto && key : keys) {
            constraints.emplace_back(
                _add_first_valued_constraint(key, constraint_lambdas...));
            ++constr_id;
        }
        return constraints_range(
            keys,
           std::views::transform(std::views::iota(offset, constr_id),
                                    [](auto && i) { return constraint{i}; }));
    }

    // void set_constraint_rhs(constraint c, double rhs) {
    //     check(SCIP.setdblattrelement(model, SCIP_DBL_ATTR_RHS, c, rhs));
    // }
    // void set_constraint_sense(constraint c, constraint_sense r) {
    //     check(SCIP.setcharattrelement(model, SCIP_CHAR_ATTR_SENSE, c,
    //                                   constraint_sense_to_scip_sense(r)));
    // }
    // constraint add_ranged_constraint(linear_expression auto && le, double lb,
    //                                  double ub) {
    //     int constr_id = static_cast<int>(num_constraints());
    //     tmp_indices.resize(0);
    //     tmp_scalars.resize(0);
    //     for(auto && [var, coef] : le.linear_terms()) {
    //         tmp_indices.emplace_back(var);
    //         tmp_scalars.emplace_back(coef);
    //     }
    //     check(SCIP.addrangeconstr(model,
    //     static_cast<int>(tmp_indices.size()),
    //                               tmp_indices.data(), tmp_scalars.data(),
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
    // constraint_sense get_constraint_sense(constraint c) {
    //     char sense;
    //     update_scip_model();
    //     check(SCIP.getcharattrelement(model, SCIP_CHAR_ATTR_SENSE, c,
    //     &sense)); return scip_sense_to_constraint_sense(sense);
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

    void solve() { check(SCIP.solve(model)); }

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

}  // namespace scip::v8
}  // namespace fhamonic::mippp

#endif  // MIPPP_SCIP_v8_MILP_HPP