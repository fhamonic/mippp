#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <memory>
#include <numeric>
#include <optional>
#include <ranges>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "mippp/detail/handle_guard.hpp"
#include "mippp/detail/iis_arithmetic.hpp"
#include "mippp/detail/invoke_key.hpp"
#include "mippp/detail/restore_guard.hpp"
#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"
#include "mippp/utility/iis_snapshot.hpp"
#include "mippp/utility/variant.hpp"

#include "mippp/solvers/copt/impl/v1/copt_api.hpp"
#include "mippp/solvers/model_base.hpp"

namespace mippp {
namespace copt::impl::v1 {

struct copt_handle_release {
    static auto free_problem(const auto & api, copt_env *, copt_prob *& prob) {
        return api.DeleteProb(&prob);
    }
    static auto free_env(const auto & api, copt_env *& env) {
        return api.DeleteEnv(&env);
    }
};

class copt_base : protected model_base<int, double> {
protected:
    const copt_api * COPT;
    copt_env * env;
    copt_prob * prob;
    // declared after env and prob, which it releases
    detail::handle_guard<copt_api, copt_env *, copt_prob *, copt_handle_release>
        handle_guard;

    std::vector<index> tmp_begins;
    std::vector<char> tmp_types;
    std::vector<scalar> tmp_rhs;

    void check(const ret_code error) { COPT->_check(env, error); }
    static constexpr char constraint_sense_to_copt_sense(constraint_sense rel) {
        if(rel == constraint_sense::less_equal) return COPT_LESS_EQUAL;
        if(rel == constraint_sense::equal) return COPT_EQUAL;
        return COPT_GREATER_EQUAL;
    }
    static constexpr constraint_sense copt_sense_to_constraint_sense(
        const char sense) {
        if(sense == COPT_LESS_EQUAL) return constraint_sense::less_equal;
        if(sense == COPT_EQUAL) return constraint_sense::equal;
        return constraint_sense::greater_equal;
    }

public:
    // the anchor model_variable_params_t deduces from
    using model_base<int, double>::default_variable_params;
    using model_base<int, double>::variables;
    using model_base<int, double>::constraints;
    double infinity() const noexcept { return COPT_INFINITY; }
    using model_base<int, double>::is_infinite;

    [[nodiscard]] explicit copt_base(const copt_api & api)
        : model_base<int, double>()
        , COPT(&api)
        , env(nullptr)
        , prob(nullptr)
        , handle_guard(api, env, prob) {
        check(COPT->CreateEnv(&env));
        check(COPT->CreateProb(env, &prob));
        check(COPT->SetIntParam(prob, COPT_INTPARAM_LOGGING, 0));
    }

    constexpr copt_base(const copt_base &) = delete;
    copt_base(copt_base && other) noexcept
        : model_base<int, double>(std::move(other))
        , COPT(other.COPT)
        , env(other.env)
        , prob(other.prob)
        , handle_guard(*COPT, env, prob)
        , tmp_begins(std::move(other.tmp_begins))
        , tmp_types(std::move(other.tmp_types))
        , tmp_rhs(std::move(other.tmp_rhs)) {
        other.env = nullptr;
        other.prob = nullptr;
    }

    constexpr copt_base & operator=(const copt_base &) = delete;
    constexpr copt_base & operator=(copt_base && other) = delete;

    std::size_t num_variables() {
        int num;
        check(COPT->GetIntAttr(prob, COPT_INTATTR_COLS, &num));
        return static_cast<std::size_t>(num);
    }
    std::size_t num_constraints() {
        int num;
        check(COPT->GetIntAttr(prob, COPT_INTATTR_ROWS, &num));
        return static_cast<std::size_t>(num);
    }
    std::size_t num_nonzeros() {
        int num;
        check(COPT->GetIntAttr(prob, COPT_INTATTR_ELEMS, &num));
        return static_cast<std::size_t>(num);
    }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////// Native handles /////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
public:
    const copt_api & native_api() const noexcept { return *COPT; }
    std::pair<copt_env *, copt_prob *> native_model() const noexcept {
        return {env, prob};
    }
    int native_id(variable v) const noexcept { return v.id(); }
    int native_id(constraint c) const noexcept { return c.id(); }

public:
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// Objective ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void set_maximization() { check(COPT->SetObjSense(prob, COPT_MAXIMIZE)); }
    void set_minimization() { check(COPT->SetObjSense(prob, COPT_MINIMIZE)); }

    void set_objective_offset(scalar constant) {
        check(COPT->SetObjConst(prob, constant));
    }

private:
    template <bool distinct, linear_expression LE>
    void _set_objective(LE && le) {
        if constexpr(!distinct) _prepare_coalescing(num_variables());
        _reset_cache();
        _register_variables_entries<distinct>(le.linear_terms());
        check(COPT->ReplaceColObj(prob, static_cast<int>(tmp_indices.size()),
                                  tmp_indices.data(), tmp_scalars.data()));
        set_objective_offset(le.constant());
    }

public:
    template <linear_expression LE>
    void set_objective(LE && le) {
        _set_objective<false>(std::forward<LE>(le));
    }
    template <linear_expression LE>
    void set_objective(distinct_variables_t, LE && le) {
        _set_objective<true>(std::forward<LE>(le));
    }
    void add_to_objective(linear_expression auto && le) {
        const auto num_vars = num_variables();
        tmp_indices.resize(num_vars);
        std::iota(tmp_indices.begin(), tmp_indices.end(), 0);
        tmp_scalars.resize(num_vars);
        check(COPT->GetColInfo(prob, COPT_DBLINFO_OBJ,
                               static_cast<int>(num_vars), tmp_indices.data(),
                               tmp_scalars.data()));
        for(auto && [var, coef] : le.linear_terms()) {
            tmp_scalars[var.uid()] += coef;
        }
        check(COPT->ReplaceColObj(prob, static_cast<int>(tmp_indices.size()),
                                  tmp_indices.data(), tmp_scalars.data()));
        set_objective_offset(get_objective_offset() + le.constant());
    }
    template <linear_expression LE>
    void add_to_objective(distinct_variables_t, LE && le) {
        add_to_objective(std::forward<LE>(le));
    }
    scalar get_objective_offset() {
        scalar objective_offset;
        check(COPT->GetDblAttr(prob, COPT_DBLATTR_OBJCONST, &objective_offset));
        return objective_offset;
    }
    auto get_objective() {
        const auto num_vars = num_variables();
        auto coefs = std::make_shared_for_overwrite<double[]>(num_vars);
        tmp_indices.resize(num_vars);
        std::iota(tmp_indices.begin(), tmp_indices.end(), 0);
        check(COPT->GetColInfo(prob, COPT_DBLINFO_OBJ,
                               static_cast<int>(num_vars), tmp_indices.data(),
                               coefs.get()));
        return linear_expression_view(
            std::views::transform(
                std::views::iota(index{0}, static_cast<index>(num_vars)),
                [coefs = std::move(coefs)](auto i) {
                    return std::make_pair(variable(i), coefs[i]);
                }),
            get_objective_offset());
    }
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// Variables ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
protected:
    void _add_variables(std::size_t count, const variable_params & params,
                        const char type) {
        std::optional<std::size_t> dbl_offset_1, dbl_offset_2;
        tmp_scalars.resize(count);
        std::fill(tmp_scalars.begin(), tmp_scalars.end(), params.obj_coef);
        if(auto lb = params.lower_bound.value_or(-COPT_INFINITY); lb != 0.0) {
            dbl_offset_1.emplace(tmp_scalars.size());
            tmp_scalars.resize(tmp_scalars.size() + count, lb);
        }
        if(auto ub = params.upper_bound.value_or(COPT_INFINITY);
           ub < COPT_INFINITY) {
            dbl_offset_2.emplace(tmp_scalars.size());
            tmp_scalars.resize(tmp_scalars.size() + count, ub);
        }
        tmp_types.resize(count);
        std::fill(tmp_types.begin(), tmp_types.end(), type);
        check(COPT->AddCols(
            prob, static_cast<int>(count), tmp_scalars.data(), nullptr, nullptr,
            nullptr, nullptr, tmp_types.data(),
            dbl_offset_1.has_value()
                ? (tmp_scalars.data() +
                   static_cast<std::ptrdiff_t>(dbl_offset_1.value()))
                : nullptr,
            dbl_offset_2.has_value()
                ? (tmp_scalars.data() +
                   static_cast<std::ptrdiff_t>(dbl_offset_2.value()))
                : nullptr,
            nullptr));
    }

public:
    friend model_base<int, double>;
    using model_base<int, double>::add_variable;
    using model_base<int, double>::add_variables;
    using model_base<int, double>::add_named_variable;
    using model_base<int, double>::add_named_variables;

private:
    std::size_t _new_variables(std::size_t count,
                               const variable_params & params,
                               variable_kind kind) {
        const std::size_t offset = num_variables();
        _add_variables(count, params,
                       kind == variable_kind::continuous ? COPT_CONTINUOUS
                       : kind == variable_kind::integer  ? COPT_INTEGER
                                                         : COPT_BINARY);
        return offset;
    }

private:
    template <typename ER>
    inline variable _add_column(ER && entries, const variable_params & params,
                                const char & type) {
        const int var_id = static_cast<int>(num_variables());
        _reset_cache();
        _register_constraints_entries<true>(entries);
        check(COPT->AddCol(
            prob, params.obj_coef, static_cast<int>(tmp_indices.size()),
            tmp_indices.data(), tmp_scalars.data(), type,
            params.lower_bound.value_or(-COPT_INFINITY),
            params.upper_bound.value_or(+COPT_INFINITY), nullptr));
        return variable(var_id);
    }

public:
    template <std::ranges::range ER>
    variable add_column(
        ER && entries, const variable_params params = default_variable_params) {
        return _add_column(entries, params, COPT_CONTINUOUS);
    }
    variable add_column(
        std::initializer_list<std::pair<constraint, scalar>> entries,
        const variable_params params = default_variable_params) {
        return _add_column(entries, params, COPT_CONTINUOUS);
    }

    void set_objective_coefficient(variable v, scalar c) {
        const int id = v.id();
        check(COPT->SetColObj(prob, 1, &id, &c));
    }
    void set_variable_lower_bound(variable v, scalar lb) {
        const int id = v.id();
        check(COPT->SetColLower(prob, 1, &id, &lb));
    }
    void set_variable_upper_bound(variable v, scalar ub) {
        const int id = v.id();
        check(COPT->SetColUpper(prob, 1, &id, &ub));
    }
    void set_variable_name(variable v, const std::string & name) {
        const int id = v.id();
        const char * c_str = name.c_str();
        check(COPT->SetColNames(prob, 1, &id, &c_str));
    }

    scalar get_objective_coefficient(variable v) {
        scalar coef;
        const int id = v.id();
        check(COPT->GetColInfo(prob, COPT_DBLINFO_OBJ, 1, &id, &coef));
        return coef;
    }
    scalar get_variable_lower_bound(variable v) {
        scalar lb;
        const int id = v.id();
        check(COPT->GetColInfo(prob, COPT_DBLINFO_LB, 1, &id, &lb));
        return lb;
    }
    scalar get_variable_upper_bound(variable v) {
        scalar ub;
        const int id = v.id();
        check(COPT->GetColInfo(prob, COPT_DBLINFO_UB, 1, &id, &ub));
        return ub;
    }
    std::string get_variable_name(variable v) {
        int size;
        check(COPT->GetColName(prob, v.id(), nullptr, 0, &size));
        std::string name(static_cast<std::size_t>(size), '\0');
        check(COPT->GetColName(prob, v.id(), name.data(), size, nullptr));
        name.pop_back();
        return name;
    }

private:
    template <bool distinct, linear_constraint LC>
    constraint _add_constraint(LC && lc) {
        auto constr_id = static_cast<index>(num_constraints());
        if constexpr(!distinct) _prepare_coalescing(num_variables());
        _reset_cache();
        _register_variables_entries<distinct>(lc.linear_terms());
        const scalar b = lc.rhs();
        check(COPT->AddRow(prob, static_cast<int>(tmp_indices.size()),
                           tmp_indices.data(), tmp_scalars.data(),
                           constraint_sense_to_copt_sense(lc.sense()), b,
                           COPT_INFINITY, nullptr));
        return constraint(constr_id);
    }

public:
    template <linear_constraint LC>
    constraint add_constraint(LC && lc) {
        return _add_constraint<false>(std::forward<LC>(lc));
    }
    template <linear_constraint LC>
    constraint add_constraint(distinct_variables_t, LC && lc) {
        return _add_constraint<true>(std::forward<LC>(lc));
    }

private:
    template <bool distinct, linear_constraint LC>
    void _register_constraint(LC && lc) {
        tmp_begins.emplace_back(static_cast<index>(tmp_indices.size()));
        tmp_types.emplace_back(constraint_sense_to_copt_sense(lc.sense()));
        tmp_rhs.emplace_back(lc.rhs());
        _register_variables_entries<distinct>(lc.linear_terms());
    }
    template <bool distinct, typename Key, typename LastConstrLambda>
        requires linear_constraint<
            detail::key_invoke_result_t<LastConstrLambda &, const Key &>>
    void _register_first_valued_constraint(const Key & key,
                                           LastConstrLambda & lc_lambda) {
        _register_constraint<distinct>(detail::invoke_key(lc_lambda, key));
    }
    template <bool distinct, typename Key, typename OptConstrLambda,
              typename... Tail>
        requires detail::optional_type<detail::key_invoke_result_t<
                     OptConstrLambda &, const Key &>> &&
                 linear_constraint<
                     detail::optional_type_value_t<detail::key_invoke_result_t<
                         OptConstrLambda &, const Key &>>>
    void _register_first_valued_constraint(const Key & key,
                                           OptConstrLambda & opt_lc_lambda,
                                           Tail &... tail) {
        if(const auto & opt_lc = detail::invoke_key(opt_lc_lambda, key)) {
            _register_constraint<distinct>(opt_lc.value());
            return;
        }
        _register_first_valued_constraint<distinct>(key, tail...);
    }

    template <bool distinct, std::ranges::range IR, typename... CL>
    auto _add_constraints(IR && keys, CL &... constraint_lambdas) {
        if constexpr(!distinct) _prepare_coalescing(num_variables());
        _reset_cache();
        tmp_begins.resize(0);
        tmp_types.resize(0);
        tmp_rhs.resize(0);
        const index offset = static_cast<index>(num_constraints());
        index count = 0;
        for(auto && key : keys) {
            _register_first_valued_constraint<distinct>(key,
                                                        constraint_lambdas...);
            ++count;
        }
        tmp_begins.emplace_back(static_cast<index>(tmp_indices.size()));
        check(COPT->AddRows(prob, static_cast<int>(tmp_rhs.size()),
                            tmp_begins.data(), nullptr, tmp_indices.data(),
                            tmp_scalars.data(), tmp_types.data(),
                            tmp_rhs.data(), nullptr, nullptr));
        return detail::keyed_entities(
            *this, keys, detail::set_constraint_name,
            entity_range(constraint{offset}, static_cast<std::size_t>(count)));
    }

public:
    template <std::ranges::range IR, typename... CL>
    auto add_constraints(IR && keys, CL &&... constraint_lambdas) {
        return _add_constraints<false>(std::forward<IR>(keys),
                                       constraint_lambdas...);
    }
    template <std::ranges::range IR, typename... CL>
    auto add_constraints(distinct_variables_t, IR && keys,
                         CL &&... constraint_lambdas) {
        return _add_constraints<true>(std::forward<IR>(keys),
                                      constraint_lambdas...);
    }
    scalar get_constraint_lower_bound(constraint constr) {
        scalar lb;
        const int id = constr.id();
        check(COPT->GetRowInfo(prob, COPT_DBLINFO_LB, 1, &id, &lb));
        return lb;
    }
    scalar get_constraint_upper_bound(constraint constr) {
        scalar ub;
        const int id = constr.id();
        check(COPT->GetRowInfo(prob, COPT_DBLINFO_UB, 1, &id, &ub));
        return ub;
    }
    // COPT stores a row's two sides: a side moved to a finite value turns a
    // one-sided or equality row into a ranged one in place, and a side set to
    // infinity() frees it.
    void set_constraint_lower_bound(constraint constr, scalar lb) {
        const int id = constr.id();
        check(COPT->SetRowLower(prob, 1, &id, &lb));
    }
    void set_constraint_upper_bound(constraint constr, scalar ub) {
        const int id = constr.id();
        check(COPT->SetRowUpper(prob, 1, &id, &ub));
    }

    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////// Limits //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void set_time_limit(std::chrono::duration<double> t) {
        check(COPT->SetDblParam(prob, COPT_DBLPARAM_TIMELIMIT, t.count()));
    }
    auto get_time_limit() {
        double t;
        check(COPT->GetDblParam(prob, COPT_DBLPARAM_TIMELIMIT, &t));
        return std::chrono::duration<double>(t);
    }

    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// Verbosity ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void set_verbose(bool verbose) {
        check(COPT->SetIntParam(prob, COPT_INTPARAM_LOGGING, verbose));
    }
    bool is_verbose() {
        int verbose;
        check(COPT->GetIntParam(prob, COPT_INTPARAM_LOGGING, &verbose));
        return verbose != 0;
    }

    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////////// IIS ////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    // The routine reads TimeLimit, and the solve that confirms feasibility (an
    // LP, after the routine) or infeasibility (a MIP, before it) shares that
    // budget with it: the remainder is written for the second call only.
    template <typename Call>
    void _iis_within_time_limit(double remaining, Call && call) {
        double time_limit;
        check(COPT->GetDblParam(prob, COPT_DBLPARAM_TIMELIMIT, &time_limit));
        check(COPT->SetDblParam(prob, COPT_DBLPARAM_TIMELIMIT, remaining));
        detail::restore_guard restore_time_limit([&] {
            check(COPT->SetDblParam(prob, COPT_DBLPARAM_TIMELIMIT, time_limit));
        });
        std::forward<Call>(call)();
        restore_time_limit.restore();
    }

    // COPT keeps the bounds a user moves outside [0, 1] on a binary column
    // and calls the model infeasible, as the arithmetic assumes.
    std::optional<detail::iis_self_infeasible_column_at>
    _self_infeasible_column(int num_col) {
        if(num_col == 0) return std::nullopt;
        const auto count = static_cast<std::size_t>(num_col);
        std::vector<double> lower(count), upper(count);
        std::vector<char> types(count);
        check(COPT->GetColInfo(prob, COPT_DBLINFO_LB, num_col, nullptr,
                               lower.data()));
        check(COPT->GetColInfo(prob, COPT_DBLINFO_UB, num_col, nullptr,
                               upper.data()));
        check(COPT->GetColType(prob, num_col, nullptr, types.data()));
        return detail::iis_first_self_infeasible_column<
            COPT_CONTINUOUS, COPT_INTEGER, COPT_BINARY>(lower, upper, types);
    }

    // The solve that tells a feasible MIP apart is stopped at its first
    // incumbent: the answer is then known and the routine is not called. The
    // MIPSOL context would stop before the candidate is committed, leaving
    // HasMipSol at 0.
    static int _iis_interrupt_callback(copt_prob * prob, void *, int,
                                       void * model) {
        (void)static_cast<copt_base *>(model)->COPT->Interrupt(prob);
        return 0;
    }

protected:
    // node_limit is reachable on copt_milp only, from the solve that precedes
    // the search
    using iis_outcome_type =
        std::variant<iis_outcome::incomplete, iis_outcome::irreducible,
                     iis_outcome::feasible, iis_outcome::interrupted,
                     iis_outcome::time_limit, iis_outcome::node_limit>;

    template <typename VariableStatus, typename ConstraintStatus>
    iis_snapshot<variable, constraint, VariableStatus, ConstraintStatus,
                 iis_outcome_type>
    _compute_iis(const bool mip) {
        const int num_col = static_cast<int>(num_variables());
        const int num_row = static_cast<int>(num_constraints());
        detail::iis_answer<iis_snapshot<variable, constraint, VariableStatus,
                                        ConstraintStatus, iis_outcome_type>>
            answer(static_cast<std::size_t>(num_col),
                   static_cast<std::size_t>(num_row));
        const auto single_column = [&](const auto & col) {
            answer.flag_variable(col.index, col.sides.lower, col.sides.upper);
            return answer.finish(iis_outcome::irreducible{});
        };

        // The routine hands back its previous answer, and calls a model whose
        // LP status is optimal feasible, until a row or column is added or
        // the solution is reset: the reset drops the held solution too.
        check(COPT->Reset(prob, 0));

        // The routine leaks memory on a model without columns (measured on
        // 8.0.5), whose rows are constants: the first side that 0 violates
        // is the whole explanation, and none violated means feasible.
        if(num_col == 0) {
            const auto side =
                detail::iis_column_less_precheck(*this, constraints());
            if(!side) return answer.finish(iis_outcome::feasible{});
            answer.flag_constraint(side->first.uid(), side->second,
                                   !side->second);
            return answer.finish(iis_outcome::irreducible{});
        }

        const double budget = get_time_limit().count();
        const auto start = std::chrono::steady_clock::now();
        const auto elapsed = [&] {
            return std::chrono::duration<double>(
                       std::chrono::steady_clock::now() - start)
                .count();
        };

        // COPT's MIP path takes the model it is given for infeasible and
        // flags the whole model as its answer: only a solve tells a feasible
        // MIP apart. The user's callback is detached by the caller, so the
        // slot is free for the interrupt.
        if(mip) {
            check(COPT->SetCallback(prob, _iis_interrupt_callback,
                                    COPT_CBCONTEXT_INCUMBENT, this));
            const ret_code solve_code = COPT->Solve(prob);
            const ret_code detach_code =
                COPT->SetCallback(prob, nullptr, 0, nullptr);
            check(solve_code);
            check(detach_code);
            int mip_status, has_sol;
            check(COPT->GetIntAttr(prob, COPT_INTATTR_MIPSTATUS, &mip_status));
            check(COPT->GetIntAttr(prob, COPT_INTATTR_HASMIPSOL, &has_sol));
            switch(mip_status) {
                case COPT_MIPSTATUS_INFEASIBLE:
                    break;
                case COPT_MIPSTATUS_OPTIMAL:
                case COPT_MIPSTATUS_UNBOUNDED:
                    return answer.finish(iis_outcome::feasible{});
                default:
                    // an incumbent found before a stop settles the question,
                    // and the interrupt above always leaves one
                    if(has_sol) return answer.finish(iis_outcome::feasible{});
                    switch(mip_status) {
                        case COPT_MIPSTATUS_TIMEOUT:
                            return answer.finish(iis_outcome::time_limit{});
                        case COPT_MIPSTATUS_NODELIMIT:
                            return answer.finish(iis_outcome::node_limit{});
                        case COPT_MIPSTATUS_INTERRUPTED:
                            return answer.finish(iis_outcome::interrupted{});
                        default:
                            return answer.finish(iis_outcome::incomplete{});
                    }
            }
        }

        // The routine reads past its arrays on a model without rows, SOS or
        // indicators whose bounds are feasible as an LP, so such a model is
        // answered from its columns, the only place a conflict can be.
        int num_sos, num_indicators;
        check(COPT->GetIntAttr(prob, COPT_INTATTR_SOSS, &num_sos));
        check(COPT->GetIntAttr(prob, COPT_INTATTR_INDICATORS, &num_indicators));
        if(num_row == 0 && num_sos == 0 && num_indicators == 0) {
            if(const auto col = _self_infeasible_column(num_col))
                return single_column(*col);
            if(mip)
                throw solver_error(
                    "mippp: COPT_Solve found a model without rows infeasible, "
                    "but every column's bounds admit a value");
            return answer.finish(iis_outcome::feasible{});
        }

        ret_code code = COPT_RETCODE_OK;
        if(mip)
            _iis_within_time_limit(std::max(0., budget - elapsed()),
                                   [&] { code = COPT->ComputeIIS(prob); });
        else
            code = COPT->ComputeIIS(prob);
        // never reached under an infinite budget, always under a zero one
        const bool out_of_time = elapsed() >= budget;
        const auto short_of = [&](bool conflict) -> iis_outcome_type {
            if(out_of_time) return iis_outcome::time_limit(conflict);
            return iis_outcome::incomplete(conflict);
        };

        // the generic code is the routine's word for a feasible model
        if(code == COPT_RETCODE_INVALID) {
            if(mip) {
                // the routine takes an integer column whose interval holds
                // no integer for feasible
                if(const auto col = _self_infeasible_column(num_col))
                    return single_column(*col);
                throw solver_error(
                    "mippp: COPT_ComputeIIS reports as feasible a model "
                    "COPT_Solve found infeasible");
            }
            // the routine leaves the model unstarted, so a solve confirms
            int lp_status;
            _iis_within_time_limit(std::max(0., budget - elapsed()),
                                   [&] { check(COPT->SolveLp(prob)); });
            check(COPT->GetIntAttr(prob, COPT_INTATTR_LPSTATUS, &lp_status));
            switch(lp_status) {
                case COPT_LPSTATUS_OPTIMAL:
                case COPT_LPSTATUS_UNBOUNDED:
                    return answer.finish(iis_outcome::feasible{});
                case COPT_LPSTATUS_INFEASIBLE:
                    throw solver_error(
                        "mippp: COPT_ComputeIIS reports as feasible a model "
                        "COPT_SolveLp found infeasible");
                case COPT_LPSTATUS_TIMEOUT:
                    return answer.finish(iis_outcome::time_limit{});
                case COPT_LPSTATUS_INTERRUPTED:
                    return answer.finish(iis_outcome::interrupted{});
                default:
                    return answer.finish(iis_outcome::incomplete{});
            }
        }
        check(code);

        int has_iis;
        check(COPT->GetIntAttr(prob, COPT_INTATTR_HASIIS, &has_iis));
        if(!has_iis) {
            if(out_of_time) return answer.finish(iis_outcome::time_limit{});
            throw solver_error(
                "mippp: COPT_ComputeIIS returned without an IIS");
        }
        int iis_rows, iis_cols, iis_sos, iis_indicators, is_minimal;
        check(COPT->GetIntAttr(prob, COPT_INTATTR_IISROWS, &iis_rows));
        check(COPT->GetIntAttr(prob, COPT_INTATTR_IISCOLS, &iis_cols));
        check(COPT->GetIntAttr(prob, COPT_INTATTR_IISSOSS, &iis_sos));
        check(COPT->GetIntAttr(prob, COPT_INTATTR_IISINDICATORS,
                               &iis_indicators));
        check(COPT->GetIntAttr(prob, COPT_INTATTR_ISMINIIS, &is_minimal));

        const auto row_count = static_cast<std::size_t>(num_row);
        const auto col_count = static_cast<std::size_t>(num_col);
        std::vector<int> row_lower_flag(row_count), row_upper_flag(row_count),
            col_lower_flag(col_count), col_upper_flag(col_count);
        if(num_row > 0) {
            check(COPT->GetRowLowerIIS(prob, num_row, nullptr,
                                       row_lower_flag.data()));
            check(COPT->GetRowUpperIIS(prob, num_row, nullptr,
                                       row_upper_flag.data()));
        }
        if(num_col > 0) {
            check(COPT->GetColLowerIIS(prob, num_col, nullptr,
                                       col_lower_flag.data()));
            check(COPT->GetColUpperIIS(prob, num_col, nullptr,
                                       col_upper_flag.data()));
        }
        int flagged_rows = 0, flagged_cols = 0, flagged_bounds = 0;
        for(std::size_t i = 0; i < row_count; ++i)
            flagged_rows += (row_lower_flag[i] != 0 || row_upper_flag[i] != 0);
        for(std::size_t j = 0; j < col_count; ++j) {
            flagged_cols += (col_lower_flag[j] != 0 || col_upper_flag[j] != 0);
            flagged_bounds +=
                (col_lower_flag[j] != 0) + (col_upper_flag[j] != 0);
        }
        // A stop can leave the counts at the whole model while the flags
        // name a few entities: neither is then an answer. IISCols is
        // documented as a number of bounds but 8.0.5 counts columns: either
        // agreement is accepted.
        if(flagged_rows != iis_rows ||
           (flagged_cols != iis_cols && flagged_bounds != iis_cols))
            return answer.finish(short_of(false));
        // Without a special constraint as sole member, an empty answer is
        // the routine's word for an integer column whose interval holds no
        // integer, which it never names.
        if(flagged_rows + flagged_cols == 0 && iis_sos + iis_indicators == 0) {
            if(const auto col = _self_infeasible_column(num_col))
                return single_column(*col);
            return answer.finish(short_of(false));
        }

        // On a MIP, COPT flags one side of an equality row and one bound of a
        // two-bounded column, continuous or not, where both are needed: those
        // members are reported whole.
        std::vector<double> row_lower(row_count), row_upper(row_count),
            col_lower(col_count), col_upper(col_count);
        if(mip) {
            if(num_row > 0) {
                check(COPT->GetRowInfo(prob, COPT_DBLINFO_LB, num_row, nullptr,
                                       row_lower.data()));
                check(COPT->GetRowInfo(prob, COPT_DBLINFO_UB, num_row, nullptr,
                                       row_upper.data()));
            }
            if(num_col > 0) {
                check(COPT->GetColInfo(prob, COPT_DBLINFO_LB, num_col, nullptr,
                                       col_lower.data()));
                check(COPT->GetColInfo(prob, COPT_DBLINFO_UB, num_col, nullptr,
                                       col_upper.data()));
            }
        }
        for(std::size_t i = 0; i < row_count; ++i)
            answer.flag_constraint(i, row_lower_flag[i] != 0,
                                   row_upper_flag[i] != 0,
                                   mip && !is_infinite(row_lower[i]) &&
                                       !is_infinite(row_upper[i]));
        for(std::size_t j = 0; j < col_count; ++j)
            answer.flag_variable(j, col_lower_flag[j] != 0,
                                 col_upper_flag[j] != 0,
                                 mip && !is_infinite(col_lower[j]) &&
                                     !is_infinite(col_upper[j]));
        // The routine takes native SOS and indicators for candidates, so
        // IsMinIIS is minimality against the ones it names: against the whole
        // background only when it names all of them, or no linear member.
        const bool every_special_named =
            iis_sos == num_sos && iis_indicators == num_indicators;
        if(is_minimal &&
           (every_special_named || flagged_rows + flagged_cols == 0))
            return answer.finish(iis_outcome::irreducible{});
        return answer.finish(short_of(true));
    }
};

}  // namespace copt::impl::v1
}  // namespace mippp
