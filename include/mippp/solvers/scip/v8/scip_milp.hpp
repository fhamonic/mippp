#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <ranges>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/scip/v8/scip_api.hpp"

namespace mippp {
namespace scip::v8 {

class scip_milp {
protected:
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

public:
    static constexpr variable_params default_variable_params = {
        .obj_coef = 0, .lower_bound = 0, .upper_bound = std::nullopt};

protected:
    const scip_api * SCIP;
    struct Scip * model;
    std::vector<SCIP_VAR *> variables;
    std::vector<SCIP_CONS *> constraints;

    std::vector<std::pair<unsigned int, unsigned int>> tmp_entry_index_cache;
    std::vector<SCIP_VAR *> tmp_vars;
    std::vector<SCIP_Real> tmp_reals;
    unsigned int register_count;
    bool _solved = false;

    void _prepare_coalescing(const std::size_t ids_end) {
        tmp_entry_index_cache.resize(ids_end);
    }
    void _reset_cache() {
        tmp_vars.resize(0);
        tmp_reals.resize(0);
    }
    template <bool distinct, std::ranges::range Entries>
    void _register_variables_entries(Entries && entries) {
        if constexpr(distinct) {
            for(auto && [entity, coef] : entries) {
                const int id = entity.id();
                tmp_vars.emplace_back(*(variables.data() + id));
                tmp_reals.emplace_back(coef);
            }
        } else {
            ++register_count;
            for(auto && [entity, coef] : entries) {
                const int id = entity.id();
                auto & p = *(tmp_entry_index_cache.data() + id);
                if(p.first == register_count) {
                    tmp_reals[p.second] += static_cast<scalar>(coef);
                    continue;
                }
                p = std::make_pair(register_count, tmp_vars.size());
                tmp_vars.emplace_back(*(variables.data() + id));
                tmp_reals.emplace_back(coef);
            }
        }
    }

public:
    [[nodiscard]] scip_milp() : scip_milp(scip_api::load()) {}
    [[nodiscard]] explicit scip_milp(const scip_api & api)
        : SCIP(&api), register_count(0) {
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
        , constraints(std::move(other.constraints))
        , register_count(other.register_count)
        , _solved(other._solved) {
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
    ////////////////////////////// Native handles /////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
public:
    const scip_api & native_api() const noexcept { return *SCIP; }
    struct Scip * native_model() const noexcept { return model; }
    SCIP_VAR * native_id(variable v) const noexcept {
        return variables[v.uid()];
    }
    SCIP_CONS * native_id(constraint c) const noexcept {
        return constraints[c.uid()];
    }

public:
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// Objective ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    // SCIP rejects every modification once a solve has run (stage SOLVED):
    // drop the transformed problem, which sends it back to stage PROBLEM and
    // keeps the found solutions in the original space. Called by every
    // mutator; a solve followed by another solve resumes the search instead.
    void _free_transform() {
        if(!_solved) return;
        check(SCIP->freeTransform(model));
        _solved = false;
    }

    void set_maximization() {
        _free_transform();
        check(SCIP->setObjsense(model, SCIP_OBJSENSE_MAXIMIZE));
    }
    void set_minimization() {
        _free_transform();
        check(SCIP->setObjsense(model, SCIP_OBJSENSE_MINIMIZE));
    }

    void set_objective_offset(double offset) {
        _free_transform();
        check(SCIP->addOrigObjoffset(model,
                                     offset - SCIP->getOrigObjoffset(model)));
    }
    void set_objective(linear_expression auto && le) {
        _free_transform();
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
        _free_transform();
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
        _free_transform();
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
        _free_transform();
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
    auto add_variables(std::size_t count,
                       variable_params params = default_variable_params) {
        const std::size_t offset = num_variables();
        for(std::size_t i = 0; i < count; ++i)
            _add_variable(params, SCIP_VARTYPE_CONTINUOUS);
        return _make_variables_view(offset, count);
    }
    template <typename IL>
    auto add_variables(std::size_t count, IL && id_lambda,
                       variable_params params = default_variable_params) {
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
        std::size_t count, variable_params params = default_variable_params) {
        const std::size_t offset = num_variables();
        for(std::size_t i = 0; i < count; ++i)
            _add_variable(params, SCIP_VARTYPE_INTEGER);
        return _make_variables_view(offset, count);
    }
    template <typename IL>
    auto add_integer_variables(
        std::size_t count, IL && id_lambda,
        variable_params params = default_variable_params) {
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
    auto add_binary_variables(std::size_t count) {
        const std::size_t offset = num_variables();
        for(std::size_t i = 0; i < count; ++i)
            _add_variable(
                {.obj_coef = 0.0, .lower_bound = 0.0, .upper_bound = 1.0},
                SCIP_VARTYPE_BINARY);
        return _make_variables_view(offset, count);
    }
    template <typename IL>
    auto add_binary_variables(std::size_t count, IL && id_lambda) {
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
    auto add_named_variables(std::size_t count, NL && name_lambda,
                             variable_params params = default_variable_params) {
        const std::size_t offset = num_variables();
        for(std::size_t i = 0; i < count; ++i)
            _add_variable(params, SCIP_VARTYPE_CONTINUOUS,
                          name_lambda(i).c_str());
        return _make_variables_view(offset, count);
    }
    template <typename IL, typename NL>
    auto add_named_variables(std::size_t count, IL && id_lambda,
                             NL && name_lambda,
                             variable_params params = default_variable_params) {
        const std::size_t offset = num_variables();
        for(std::size_t i = 0; i < count; ++i)
            _add_variable(params, SCIP_VARTYPE_CONTINUOUS);
        return _make_indexed_named_variables_view(
            offset, count, std::forward<IL>(id_lambda),
            std::forward<NL>(name_lambda));
    }

    void set_continuous(variable v) {
        _free_transform();
        unsigned int infeas;
        check(SCIP->chgVarType(model, variables[v.uid()],
                               SCIP_VARTYPE_CONTINUOUS, &infeas));
    }
    void set_integer(variable v) {
        _free_transform();
        unsigned int infeas;
        check(SCIP->chgVarType(model, variables[v.uid()], SCIP_VARTYPE_INTEGER,
                               &infeas));
    }
    void set_binary(variable v) {
        _free_transform();
        set_variable_lower_bound(v, 0);
        set_variable_upper_bound(v, 1);
        unsigned int infeas;
        check(SCIP->chgVarType(model, variables[v.uid()], SCIP_VARTYPE_BINARY,
                               &infeas));
    }
    void set_objective_coefficient(variable v, double c) {
        _free_transform();
        check(SCIP->chgVarObj(model, variables[v.uid()], c));
    }
    void set_variable_lower_bound(variable v, double lb) {
        _free_transform();
        check(SCIP->chgVarLb(model, variables[v.uid()], lb));
    }
    void set_variable_upper_bound(variable v, double ub) {
        _free_transform();
        check(SCIP->chgVarUb(model, variables[v.uid()], ub));
    }
    void set_variable_name(variable v, std::string name) {
        _free_transform();
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
    template <bool distinct>
    SCIP_CONS * _add_constraint(linear_constraint auto && lc) {
        _free_transform();
        SCIP_CONS * constr = nullptr;
        const double b = lc.rhs();
        _reset_cache();
        _register_variables_entries<distinct>(lc.linear_terms());
        check(SCIP->createConsBasicLinear(
            model, &constr, "", static_cast<int>(tmp_vars.size()),
            tmp_vars.data(), tmp_reals.data(),
            (lc.sense() == constraint_sense::less_equal)
                ? -SCIP->infinity(model)
                : b,
            (lc.sense() == constraint_sense::greater_equal)
                ? SCIP->infinity(model)
                : b));
        check(SCIP->addCons(model, constr));
        return constr;
    }

public:
    template <linear_constraint LC>
    constraint add_constraint(LC && lc) {
        constraint_id constr_id = static_cast<constraint_id>(num_constraints());
        _prepare_coalescing(num_variables());
        constraints.emplace_back(_add_constraint<false>(std::forward<LC>(lc)));
        return constraint(constr_id);
    }
    template <linear_constraint LC>
    constraint add_constraint(distinct_variables_t, LC && lc) {
        constraint_id constr_id = static_cast<constraint_id>(num_constraints());
        constraints.emplace_back(_add_constraint<true>(std::forward<LC>(lc)));
        return constraint(constr_id);
    }

private:
    template <bool distinct, typename Key, typename LastConstrLambda>
        requires linear_constraint<std::invoke_result_t<LastConstrLambda, Key>>
    SCIP_CONS * _add_first_valued_constraint(const Key & key,
                                             LastConstrLambda & lc_lambda) {
        return _add_constraint<distinct>(lc_lambda(key));
    }
    template <bool distinct, typename Key, typename OptConstrLambda,
              typename... Tail>
        requires detail::optional_type<
                     std::invoke_result_t<OptConstrLambda, Key>> &&
                 linear_constraint<detail::optional_type_value_t<
                     std::invoke_result_t<OptConstrLambda, Key>>>
    SCIP_CONS * _add_first_valued_constraint(const Key & key,
                                             OptConstrLambda & opt_lc_lambda,
                                             Tail &... tail) {
        if(const auto & opt_lc = opt_lc_lambda(key)) {
            return _add_constraint<distinct>(opt_lc.value());
        }
        return _add_first_valued_constraint<distinct>(key, tail...);
    }

    template <bool distinct, std::ranges::range IR, typename... CL>
    auto _add_constraints(IR && keys, CL &&... constraint_lambdas) {
        if constexpr(!distinct) _prepare_coalescing(num_variables());
        const constraint_id offset =
            static_cast<constraint_id>(num_constraints());
        constraint_id constr_id = offset;
        for(auto && key : keys) {
            constraints.emplace_back(_add_first_valued_constraint<distinct>(
                key, constraint_lambdas...));
            ++constr_id;
        }
        return constraints_range(
            std::forward<IR>(keys),
            std::views::transform(std::views::iota(offset, constr_id),
                                  [](auto && i) { return constraint{i}; }));
    }

public:
    template <std::ranges::range IR, typename... CL>
    auto add_constraints(IR && keys, CL &&... constraint_lambdas) {
        return _add_constraints<false>(std::forward<IR>(keys),
                                       std::forward<CL>(constraint_lambdas)...);
    }
    template <std::ranges::range IR, typename... CL>
    auto add_constraints(distinct_variables_t, IR && keys,
                         CL &&... constraint_lambdas) {
        return _add_constraints<true>(std::forward<IR>(keys),
                                      std::forward<CL>(constraint_lambdas)...);
    }

    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////// Tolerance parameters ///////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void set_feasibility_tolerance(double tol) {
        check(SCIP->setRealParam(model, "numerics/feastol", tol));
    }
    double get_feasibility_tolerance() {
        double tol;
        check(SCIP->getRealParam(model, "numerics/feastol", &tol));
        return tol;
    }
    void set_optimality_tolerance(double tol) {
        check(SCIP->setRealParam(model, "limits/gap", tol));
    }
    double get_optimality_tolerance() {
        double tol;
        check(SCIP->getRealParam(model, "limits/gap", &tol));
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

    status_variant _status = status::unknown{};

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
        _solved = true;
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
