#pragma once

#include <functional>
#include <limits>
#include <optional>
#include <ranges>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/scip/v8/scip_api.hpp"

#include <print>

namespace mippp {
namespace scip::v8 {

class scip_milp {
public:
    using variable_id = int;
    using constraint_id = int;
    using scalar = double;
    using variable = model_variable<variable_id, scalar>;
    using constraint = model_constraint<constraint_id>;
    template <typename Map>
    struct variable_mapping : entity_mapping<variable, Map> {
        variable_mapping(Map && t)
            : entity_mapping<variable, Map>(std::move(t)) {}
    };
    template <typename Map>
    struct constraint_mapping : entity_mapping<constraint, Map> {
        constraint_mapping(Map && t)
            : entity_mapping<constraint, Map>(std::move(t)) {}
    };

    struct variable_params {
        scalar obj_coef = scalar{0};
        std::optional<scalar> lower_bound = std::nullopt;
        std::optional<scalar> upper_bound = std::nullopt;
    };

    static constexpr variable_params default_variable_params = {
        .obj_coef = 0, .lower_bound = 0, .upper_bound = std::nullopt};

protected:
    const scip_api * SCIP;
    struct Scip * model;
    std::vector<SCIP_VAR *> variables;
    std::vector<SCIP_CONS *> constraints;

public:
    [[nodiscard]] explicit scip_milp(const scip_api & api) : SCIP(&api) {
        SCIP->create(&model);
        SCIP->includeDefaultPlugins(model);
        SCIP->createProbBasic(model, "MILP");
    }
    ~scip_milp() {
        if(!model) return;
        for(auto & var : variables) {
            SCIP->releaseVar(model, &var);
        }
        for(auto & cons : constraints) {
            SCIP->releaseCons(model, &cons);
        }
        SCIP->free(&model);
    }

    constexpr scip_milp(const scip_milp &) = delete;
    scip_milp(scip_milp && other) noexcept
        : SCIP(other.SCIP)
        , model(other.model)
        , variables(std::move(other.variables))
        , constraints(std::move(other.constraints)) {
        other.model = nullptr;
    }

    constexpr scip_milp & operator=(const scip_milp &) = delete;
    constexpr scip_milp & operator=(scip_milp && other) = delete;

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

    static constexpr void check(int retval) {
        if(retval > 0) return;
        throw std::runtime_error(std::string("scip_milp: ") +
                                 error_messages[-retval]);
    }

public:
    std::size_t num_variables() { return variables.size(); }
    std::size_t num_constraints() { return constraints.size(); }
    std::size_t num_entries() {
        return static_cast<std::size_t>(SCIP->getNNZs(model));
    }
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// Objective ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void set_maximization() {
        check(SCIP->setObjsense(model, SCIP_OBJSENSE_MAXIMIZE));
    }
    void set_minimization() {
        check(SCIP->setObjsense(model, SCIP_OBJSENSE_MINIMIZE));
    }

    void set_objective_offset(double offset) {
        check(SCIP->addOrigObjoffset(model,
                                     offset - SCIP->getOrigObjoffset(model)));
    }
    void set_objective(linear_expression auto && le) {
        for(auto && var : variables) {
            check(SCIP->chgVarObj(model, var, 0.0));
        }
        for(auto && [var_, coef] : le.linear_terms()) {
            const auto & var = variables[var_.uid()];
            check(SCIP->chgVarObj(model, var, SCIP->varGetObj(var) + coef));
        }
        set_objective_offset(le.constant());
    }
    template <linear_expression LE>
    void set_objective(distinct_variables_t, LE && le) {
        for(auto && var : variables) {
            check(SCIP->chgVarObj(model, var, 0.0));
        }
        for(auto && [var_, coef] : le.linear_terms()) {
            const auto & var = variables[var_.uid()];
            check(SCIP->chgVarObj(model, var, coef));
        }
        set_objective_offset(le.constant());
    }
    void add_objective(linear_expression auto && le) {
        for(auto && [var_, coef] : le.linear_terms()) {
            const auto & var = variables[var_.uid()];
            check(SCIP->chgVarObj(model, var, SCIP->varGetObj(var) + coef));
        }
        set_objective_offset(get_objective_offset() + le.constant());
    }
    double get_objective_offset() { return SCIP->getOrigObjoffset(model); }
    auto get_objective() {
        return linear_expression_view(
            std::views::transform(
                std::views::iota(variable_id{0},
                                 static_cast<variable_id>(num_variables())),
                [this](auto i) {
                    return std::make_pair(
                        variable(i),
                        SCIP->varGetObj(
                            variables[static_cast<std::size_t>(i)]));
                }),
            get_objective_offset());
    }
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// Variables ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    void _add_variable(const variable_params & params, SCIP_VARTYPE type,
                       const char * name = "") {
        SCIP_VAR * var = nullptr;
        check(SCIP->createVarBasic(
            model, &var, name,
            params.lower_bound.value_or(-SCIP->infinity(model)),
            params.upper_bound.value_or(SCIP->infinity(model)), params.obj_coef,
            type));
        check(SCIP->addVar(model, var));
        variables.emplace_back(var);
    }

    inline auto _make_variables_view(const std::size_t & offset,
                                     const std::size_t & count) {
        return variables_view(
            std::from_range,
            std::views::transform(
                std::views::iota(static_cast<variable_id>(offset),
                                 static_cast<variable_id>(offset + count)),
                [](auto && i) { return variable{i}; }));
    }
    template <typename IL>
    inline auto _make_indexed_variables_view(const std::size_t & offset,
                                             const std::size_t & count,
                                             IL && id_lambda) {
        return variables_view(
            typename detail::function_traits<IL>::arg_types(),
            std::views::transform(
                std::views::iota(static_cast<variable_id>(offset),
                                 static_cast<variable_id>(offset + count)),
                [](auto && i) { return variable{i}; }),
            std::forward<IL>(id_lambda));
    }
    template <typename IL, typename NL>
    inline auto _make_indexed_named_variables_view(const std::size_t & offset,
                                                   const std::size_t & count,
                                                   IL && id_lambda,
                                                   NL && name_lambda) {
        return lazily_named_variables_view(
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
        return _make_variables_view(offset, count);
    }
    template <typename IL>
    auto add_variables(
        std::size_t count, IL && id_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = num_variables();
        for(std::size_t i = 0; i < count; ++i)
            _add_variable(params, SCIP_VARTYPE_CONTINUOUS);
        return _make_indexed_variables_view(offset, count,
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
        return _make_variables_view(offset, count);
    }
    template <typename IL>
    auto add_integer_variables(
        std::size_t count, IL && id_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = num_variables();
        for(std::size_t i = 0; i < count; ++i)
            _add_variable(params, SCIP_VARTYPE_INTEGER);
        return _make_indexed_variables_view(offset, count,
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
        return _make_variables_view(offset, count);
    }
    template <typename IL>
    auto add_binary_variables(std::size_t count, IL && id_lambda) noexcept {
        const std::size_t offset = num_variables();
        for(std::size_t i = 0; i < count; ++i)
            _add_variable(
                {.obj_coef = 0.0, .lower_bound = 0.0, .upper_bound = 1.0},
                SCIP_VARTYPE_BINARY);
        return _make_indexed_variables_view(offset, count,
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
        return _make_variables_view(offset, count);
    }
    template <typename IL, typename NL>
    auto add_named_variables(
        std::size_t count, IL && id_lambda, NL && name_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = num_variables();
        for(std::size_t i = 0; i < count; ++i)
            _add_variable(params, SCIP_VARTYPE_CONTINUOUS);
        return _make_indexed_named_variables_view(
            offset, count, std::forward<IL>(id_lambda),
            std::forward<NL>(name_lambda));
    }

    void set_continuous(variable v) noexcept {
        unsigned int infeas;
        check(SCIP->chgVarType(model, variables[v.uid()],
                               SCIP_VARTYPE_CONTINUOUS, &infeas));
    }
    void set_integer(variable v) noexcept {
        unsigned int infeas;
        check(SCIP->chgVarType(model, variables[v.uid()], SCIP_VARTYPE_INTEGER,
                               &infeas));
    }
    void set_binary(variable v) noexcept {
        set_variable_lower_bound(v, 0);
        set_variable_upper_bound(v, 1);
        unsigned int infeas;
        check(SCIP->chgVarType(model, variables[v.uid()], SCIP_VARTYPE_BINARY,
                               &infeas));
    }
    void set_objective_coefficient(variable v, double c) {
        check(SCIP->chgVarObj(model, variables[v.uid()], c));
    }
    void set_variable_lower_bound(variable v, double lb) {
        check(SCIP->chgVarLb(model, variables[v.uid()], lb));
    }
    void set_variable_upper_bound(variable v, double ub) {
        check(SCIP->chgVarUb(model, variables[v.uid()], ub));
    }
    void set_variable_name(variable v, std::string name) {
        check(SCIP->chgVarName(model, variables[v.uid()], name.c_str()));
    }
    double get_objective_coefficient(variable v) {
        return SCIP->varGetObj(variables[v.uid()]);
    }
    double get_variable_lower_bound(variable v) {
        return SCIP->varGetLbGlobal(variables[v.uid()]);
    }
    double get_variable_upper_bound(variable v) {
        return SCIP->varGetUbGlobal(variables[v.uid()]);
    }
    std::string get_variable_name(variable v) {
        return std::string(SCIP->varGetName(variables[v.uid()]));
    }
    ///////////////////////////////////////////////////////////////////////////
    /////////////////////////////// Constraints ///////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    SCIP_CONS * _add_constraint(linear_constraint auto && lc) {
        SCIP_CONS * constr = nullptr;
        const double b = lc.rhs();
        check(SCIP->createConsBasicLinear(
            model, &constr, "", 0, nullptr, nullptr,
            (lc.sense() == constraint_sense::less_equal)
                ? -SCIP->infinity(model)
                : b,
            (lc.sense() == constraint_sense::greater_equal)
                ? SCIP->infinity(model)
                : b));
        for(auto && [var, coef] : lc.linear_terms()) {
            check(
                SCIP->addCoefLinear(model, constr, variables[var.uid()], coef));
        }
        check(SCIP->addCons(model, constr));
        return constr;
    }

public:
    constraint add_constraint(linear_constraint auto && lc) {
        constraint_id constr_id = static_cast<constraint_id>(num_constraints());
        constraints.emplace_back(_add_constraint(lc));
        return constraint(constr_id);
    }
    template <linear_constraint LC>
    constraint add_constraint(distinct_variables_t, LC && lc) {
        return add_constraint(std::forward<LC>(lc));
    }

private:
    template <typename Key, typename LastConstrLambda>
        requires linear_constraint<std::invoke_result_t<LastConstrLambda, Key>>
    SCIP_CONS * _add_first_valued_constraint(const Key & key,
                                             LastConstrLambda & lc_lambda) {
        return _add_constraint(lc_lambda(key));
    }
    template <typename Key, typename OptConstrLambda, typename... Tail>
        requires detail::optional_type<
                     std::invoke_result_t<OptConstrLambda, Key>> &&
                 linear_constraint<detail::optional_type_value_t<
                     std::invoke_result_t<OptConstrLambda, Key>>>
    SCIP_CONS * _add_first_valued_constraint(const Key & key,
                                             OptConstrLambda & opt_lc_lambda,
                                             Tail &... tail) {
        if(const auto & opt_lc = opt_lc_lambda(key)) {
            return _add_constraint(opt_lc.value());
        }
        return _add_first_valued_constraint(key, tail...);
    }

public:
    template <std::ranges::range IR, typename... CL>
    auto add_constraints(IR && keys, CL &&... constraint_lambdas) {
        const constraint_id offset =
            static_cast<constraint_id>(num_constraints());
        constraint_id constr_id = offset;
        for(auto && key : keys) {
            constraints.emplace_back(
                _add_first_valued_constraint(key, constraint_lambdas...));
            ++constr_id;
        }
        return constraints_range(
            std::forward<IR>(keys),
            std::views::transform(std::views::iota(offset, constr_id),
                                  [](auto && i) { return constraint{i}; }));
    }
    template <std::ranges::range IR, typename... CL>
    auto add_constraints(distinct_variables_t, IR && keys,
                         CL &&... constraint_lambdas) {
        return add_constraints(std::forward<IR>(keys),
                               std::forward<CL>(constraint_lambdas)...);
    }

    // void set_constraint_rhs(constraint c, double rhs) {
    //     check(SCIP->setdblattrelement(model, SCIP_DBL_ATTR_RHS, c, rhs));
    // }
    // void set_constraint_sense(constraint c, constraint_sense r) {
    //     check(SCIP->setcharattrelement(model, SCIP_CHAR_ATTR_SENSE, c,
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
    //     check(SCIP->addrangeconstr(model,
    //     static_cast<int>(tmp_indices.size()),
    //                               tmp_indices.data(), tmp_scalars.data(),
    //                               lb, ub, nullptr));
    //     return constr_id;
    // }
    // void set_constraint_name(constraint c, auto && name);

    // auto get_constraint_lhs(constraint c) {}
    // double get_constraint_rhs(constraint c) {
    //     double rhs;
    //     update_scip_model();
    //     check(SCIP->getdblattrelement(model, SCIP_DBL_ATTR_RHS, c, &rhs));
    //     return rhs;
    // }
    // constraint_sense get_constraint_sense(constraint c) {
    //     char sense;
    //     update_scip_model();
    //     check(SCIP->getcharattrelement(model, SCIP_CHAR_ATTR_SENSE, c,
    //     &sense)); return scip_sense_to_constraint_sense(sense);
    // }
    // // auto get_constraint(const constraint c) {}
    // auto get_constraint_name(constraint c) {
    //     char * name;
    //     update_scip_model();
    //     check(
    //         SCIP->getstrattrelement(model, SCIP_STR_ATTR_CONSTRNAME, c,
    //         &name));
    //     return std::string(name);
    // }

    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// Callbacks ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    class callback_handle_base {
    protected:
        const scip_api * SCIP;
        scip_milp & milp;
        SCIP_RESULT * result;

    public:
        callback_handle_base(const scip_api * api, scip_milp & milp_,
                             SCIP_RESULT * result_)
            : SCIP(api), milp(milp_), result(result_) {}

        std::size_t num_variables() { return milp.num_variables(); }
    };

public:
    class candidate_solution_callback_handle : public callback_handle_base {
    public:
        candidate_solution_callback_handle(const scip_api * api,
                                           scip_milp & milp_,
                                           SCIP_RESULT * result_)
            : callback_handle_base(api, milp_, result_) {
            *result = SCIP_FEASIBLE;
        }

        void reject_solution() { *result = SCIP_INFEASIBLE; }
        void add_lazy_constraint(linear_constraint auto && lc) {
            milp.add_constraint(lc);
            *result = SCIP_CONSADDED;
        }
        // double get_solution_value() {
        //     double obj;
        //     check(SCIP->callbackgetcandidatepoint(context, nullptr, 0, 0,
        //     &obj)); return obj;
        // }
        auto get_solution() {
            auto num_vars = num_variables();
            auto solution = std::make_unique_for_overwrite<double[]>(num_vars);
            SCIP_SOL * sol = SCIP->getBestSol(milp.model);
            SCIP->getSolVals(milp.model, sol, static_cast<int>(num_vars),
                             milp.variables.data(), solution.get());
            return variable_mapping(std::move(solution));
        }
    };

private:
    SCIP_CONSHDLR * candidate_solution_constraint_handler;
    std::function<void(candidate_solution_callback_handle &)>
        candidate_solution_callback;

    static SCIP_RETCODE candidate_solution_callback_fun(
        [[maybe_unused]] struct Scip * scip,
        [[maybe_unused]] SCIP_CONSHDLR * conshdlr,
        [[maybe_unused]] SCIP_CONS ** conss, [[maybe_unused]] int nconss,
        [[maybe_unused]] int nusefulconss,
        [[maybe_unused]] SCIP_Bool solinfeasible,
        [[maybe_unused]] SCIP_RESULT * result) {
        std::println("*ptr = {:p}",
                     *static_cast<void **>(conshdlr->conshdlrdata));

        // auto * model = *(static_cast<scip_milp **>(conshdlr->conshdlrdata));
        // candidate_solution_callback_handle handle(model->SCIP, *model,
        // result); model->candidate_solution_callback(handle);
        return SCIP_OKAY;
    }

public:
    template <typename F>
    void set_candidate_solution_callback(F && f) {
        candidate_solution_callback = std::forward<F>(f);

        auto ptr = static_cast<scip_milp **>(malloc(sizeof(scip_milp **)));
        *ptr = this;

        std::println("this = {:p}", static_cast<void *>(this));

        check(SCIP->includeConshdlrBasic(
            model, &candidate_solution_constraint_handler,
            "candidate_solution_callback", "candidate_solution_callback", -1,
            -1, -1, false, nullptr, nullptr, nullptr, nullptr,
            reinterpret_cast<SCIP_CONSHDLRDATA *>(ptr)));
    }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////// Tolerance parameters ///////////////////////////
    ///////////////////////////////////////////////////////////////////////////
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
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////// Solve status ///////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    // clang-format off
private:
    using status_variant = std::variant<
            status::unknown,
            status::optimal,
            status::infeasible_or_unbounded,
            status::infeasible,
            status::unbounded,
            status::limit_reached,
            status::time_limit,
            status::memory_limit,
            status::node_limit, 
            status::solution_limit,
            status::failed,
            status::numerical_failure,
            status::interrupted>;

    status_variant _status;

    status_variant _get_status() {
        using namespace status;
        switch(SCIP->getStatus(model)) {
            case SCIP_STATUS_OPTIMAL:       return optimal{};
            case SCIP_STATUS_INFORUNBD:     return infeasible_or_unbounded{};
            case SCIP_STATUS_INFEASIBLE:    return infeasible{};
            case SCIP_STATUS_UNBOUNDED:     return unbounded{};
            case SCIP_STATUS_TIMELIMIT:     return time_limit{};
            case SCIP_STATUS_MEMLIMIT:      return memory_limit{};
            case SCIP_STATUS_NODELIMIT:     return node_limit{};
            case SCIP_STATUS_SOLLIMIT:      return solution_limit{};
            case SCIP_STATUS_TOTALNODELIMIT:
            case SCIP_STATUS_GAPLIMIT:
            case SCIP_STATUS_PRIMALLIMIT:
            case SCIP_STATUS_DUALLIMIT:
            case SCIP_STATUS_BESTSOLLIMIT:
            case SCIP_STATUS_RESTARTLIMIT:   return limit_reached{};
            case SCIP_STATUS_STALLNODELIMIT: return numerical_failure{};
            case SCIP_STATUS_TERMINATE:
            case SCIP_STATUS_USERINTERRUPT:  return interrupted{};
            case SCIP_STATUS_UNKNOWN:
            default:
                return unknown{};
        }
    }
    // clang-format on
public:
    const status_variant & solve_status() const { return _status; }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////////// Solve //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void solve() {
        check(SCIP->solve(model));
        _status = _get_status();
    }
    double get_solution_value() { return SCIP->getPrimalbound(model); }
    auto get_solution() {
        auto num_vars = num_variables();
        auto solution = std::make_unique_for_overwrite<double[]>(num_vars);
        SCIP_SOL * sol = SCIP->getBestSol(model);
        check(SCIP->getSolVals(model, sol, static_cast<int>(num_vars),
                               variables.data(), solution.get()));
        return variable_mapping(std::move(solution));
    }
};

}  // namespace scip::v8
}  // namespace mippp
