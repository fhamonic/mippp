#pragma once

#include <algorithm>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <numeric>
#include <ranges>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include "mippp/detail/restore_guard.hpp"
#include "mippp/detail/row_preparation.hpp"
#include "mippp/linear_constraint.hpp"
#include "mippp/model_entities.hpp"
#include "mippp/utility/memory_size.hpp"
#include "mippp/utility/status.hpp"

#include "mippp/solvers/cplex/impl/v1/cplex_base.hpp"

namespace mippp {
namespace cplex::impl::v1 {

class cplex_milp : public cplex_base {
public:
    [[nodiscard]] cplex_milp() : cplex_milp(cplex_api::load()) {}
    [[nodiscard]] explicit cplex_milp(const cplex_api & api)
        : cplex_base(api) {}

    using model_base<int, double>::add_integer_variable;
    using model_base<int, double>::add_integer_variables;
    using model_base<int, double>::add_binary_variable;
    using model_base<int, double>::add_binary_variables;

private:
    inline std::size_t _add_binary_variables(const std::size_t & count) {
        if(_remap_ids) _extend_handle_ids_map(count);
        const std::size_t handle_ids_begin =
            _new_var_handle_range(_num_var_native_ids(), count);
        tmp_scalars.resize(2 * count);
        std::fill(tmp_scalars.begin(),
                  tmp_scalars.begin() + static_cast<std::ptrdiff_t>(count), 0);
        std::fill(tmp_scalars.begin() + static_cast<std::ptrdiff_t>(count),
                  tmp_scalars.end(), 1);
        tmp_types.resize(count);
        std::fill(tmp_types.begin(), tmp_types.end(), CPX_BINARY);
        check(CPX->newcols(
            env, lp, static_cast<int>(count), nullptr, tmp_scalars.data(),
            tmp_scalars.data() + static_cast<std::ptrdiff_t>(count),
            tmp_types.data(), nullptr));
        return handle_ids_begin;
    }

public:
    void set_continuous(variable v) {
        _invalidate_results();
        int var_id = _native_id(v);
        char type = CPX_CONTINUOUS;
        check(CPX->chgctype(env, lp, 1, &var_id, &type));
    }
    void set_integer(variable v) {
        _invalidate_results();
        int var_id = _native_id(v);
        char type = CPX_INTEGER;
        check(CPX->chgctype(env, lp, 1, &var_id, &type));
    }
    void set_binary(variable v) {
        _invalidate_results();
        int var_id = _native_id(v);
        char type = CPX_BINARY;
        check(CPX->chgctype(env, lp, 1, &var_id, &type));
        int ids[2] = {var_id, var_id};
        char lu[2] = {'L', 'U'};
        double bd[2] = {0.0, 1.0};
        check(CPX->chgbds(env, lp, 2, ids, lu, bd));
    }
    ///////////////////////////////////////////////////////////////////////////
    /////////////////////////// Special constraints ///////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    template <bool distinct, linear_constraint LC>
    void _add_indicator_constraint(variable x, bool val, LC && lc) {
        if constexpr(!distinct) _prepare_coalescing(_num_var_native_ids());
        _reset_cache();
        _register_variables_entries<distinct>(lc.linear_terms());
        check(CPX->addindconstr(env, lp, _native_id(x), static_cast<int>(!val),
                                static_cast<int>(tmp_indices.size()), lc.rhs(),
                                constraint_sense_to_cplex_sense(lc.sense()),
                                tmp_indices.data(), tmp_scalars.data(),
                                nullptr));
    }

public:
    template <linear_constraint LC>
    void add_indicator_constraint(variable x, bool val, LC && lc) {
        _invalidate_results();
        _add_indicator_constraint<false>(x, val, std::forward<LC>(lc));
    }
    template <linear_constraint LC>
    void add_indicator_constraint(distinct_variables_t, variable x, bool val,
                                  LC && lc) {
        _invalidate_results();
        _add_indicator_constraint<true>(x, val, std::forward<LC>(lc));
    }
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// Callbacks ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    // What the callbacks of one solve read. CPLEX documents its query
    // routines (CPXgetnumcols, CPXgetobj) as undefined inside a callback, so
    // solve() reads these before CPXmipopt.
    struct callback_context {
        cplex_milp * model;
        std::size_t num_columns;
        // CPXcallbackpostheursoln wants the value of the point it is given:
        // read only when a callback that can post one is set.
        std::vector<double> objective;
        double objective_offset;
        // EpRHS: what CPLEX lets a row be violated by
        double feasibility_tolerance;
        // per CPLEX thread, the node and the point of its last
        // node-relaxation call
        struct relaxation_call {
            CPXLONG node = -1;
            std::vector<double> point;
        };
        std::unordered_map<int, relaxation_call> last_relaxation;
        // CPLEX runs CANDIDATE on several threads at once, and beside
        // GLOBAL_PROGRESS: without it, two callbacks sharing one separator
        // race. Here and not in a static, which a solve nested in a callback
        // would deadlock on.
        std::mutex mutex;
    };

    class callback_handle_base : protected model_base<int, double> {
    protected:
        const cplex_api * CPX;
        CPXCALLBACKCONTEXTptr context;
        callback_context & solve_context;
        cplex_milp * model;

        void cbcheck(int error) {
            if(error == 0) return;
            CPX->callbackabort(context);
        }

        callback_handle_base(const cplex_api * api,
                             CPXCALLBACKCONTEXTptr context_,
                             callback_context & solve_context_)
            : model_base<int, double>()
            , CPX(api)
            , context(context_)
            , solve_context(solve_context_)
            , model(solve_context_.model) {}

    public:
        std::size_t num_variables() { return solve_context.num_columns; }

    protected:
        // Queued by CPLEX, which shows the point to the candidate callback
        // before taking it. That is CHECKFEAS: under NOCHECK a point the
        // callback would reject becomes the incumbent. The return code is
        // dropped: an abort here would end the search over an optional point.
        template <typename Point>
        void _post_solution(Point && point) {
            const std::size_t num_vars = solve_context.num_columns;
            _register_point(point, num_vars, [this](const int id) {
                return model->_var_handle(id);
            });
            tmp_indices.resize(num_vars);
            std::iota(tmp_indices.begin(), tmp_indices.end(), 0);
            double value = solve_context.objective_offset;
            for(std::size_t i = 0; i < num_vars; ++i)
                value += solve_context.objective[i] * tmp_scalars[i];
            CPX->callbackpostheursoln(context, static_cast<int>(num_vars),
                                      tmp_indices.data(), tmp_scalars.data(),
                                      value, CPXCALLBACKSOLUTION_CHECKFEAS);
        }
    };

public:
    class candidate_solution_callback_handle : public callback_handle_base {
    private:
        std::unique_ptr<double[]> _candidate;

        friend cplex_milp;
        candidate_solution_callback_handle(const cplex_api * api,
                                           CPXCALLBACKCONTEXTptr context_,
                                           callback_context & solve_context_)
            : callback_handle_base(api, context_, solve_context_) {}

        // the candidate in column order, read once per call
        const double * _candidate_values() {
            if(!_candidate) {
                const auto num_vars = solve_context.num_columns;
                _candidate = std::make_unique_for_overwrite<double[]>(num_vars);
                cbcheck(CPX->callbackgetcandidatepoint(
                    context, _candidate.get(), 0,
                    static_cast<int>(num_vars) - 1, nullptr));
            }
            return _candidate.get();
        }

        template <bool distinct, linear_constraint LC>
        void _add_lazy_constraint(LC && lc) {
            if constexpr(!distinct)
                _prepare_coalescing(solve_context.num_columns);
            _reset_cache();
            const auto native_id = [this](const variable & v) {
                return model->_native_id(v);
            };
            if constexpr(distinct)
                _register_raw_entries(lc.linear_terms(), native_id);
            else
                _register_coalescing_entries(lc.linear_terms(), native_id);
            detail::strip_zero_entries(tmp_indices, tmp_scalars);
            int matbegin = 0;
            const double b = lc.rhs();
            // CPXcallbackrejectcandidate rejects the candidate whatever the
            // row says: handed one the candidate satisfies, it loses that
            // candidate, and a feasible model can end infeasible. Written
            // `!(... > ...)` so that a row holding a NaN is not passed on.
            if(!(detail::row_violation(tmp_indices, tmp_scalars, lc.sense(), b,
                                       _candidate_values()) >
                 solve_context.feasibility_tolerance))
                return;
            const char sense = constraint_sense_to_cplex_sense(lc.sense());
            cbcheck(CPX->callbackrejectcandidate(
                context, 1, static_cast<int>(tmp_indices.size()), &b, &sense,
                &matbegin, tmp_indices.data(), tmp_scalars.data()));
        }

    public:
        template <linear_constraint LC>
        void add_lazy_constraint(LC && lc) {
            _add_lazy_constraint<false>(std::forward<LC>(lc));
        }
        template <linear_constraint LC>
        void add_lazy_constraint(distinct_variables_t, LC && lc) {
            _add_lazy_constraint<true>(std::forward<LC>(lc));
        }
        double get_solution_value() {
            double obj;
            // x == nullptr needs an empty [begin, end] range, else
            // CPXERR_NULL_POINTER
            cbcheck(
                CPX->callbackgetcandidatepoint(context, nullptr, 0, -1, &obj));
            return obj;
        }
        // Captures neither the handle nor the model: a point kept past the
        // call would read a dead frame through the first, and another
        // variable's value through the second once a variable is removed.
        auto get_solution() {
            const auto num_vars = solve_context.num_columns;
            auto solution = std::make_unique_for_overwrite<double[]>(num_vars);
            std::copy_n(_candidate_values(), num_vars, solution.get());
            return variable_mapping([native_id = model->_frozen_native_id(),
                                     solution = std::move(solution)](
                                        const variable & v) -> double & {
                return *(solution.get() + native_id(v));
            });
        }
        template <detail::point_of<variable, double> Point>
        void add_solution(Point && point) {
            _post_solution(point);
        }
    };

    class node_relaxation_callback_handle : public callback_handle_base {
    private:
        std::unique_ptr<double[]> _relaxation;

        friend cplex_milp;
        node_relaxation_callback_handle(const cplex_api * api,
                                        CPXCALLBACKCONTEXTptr context_,
                                        callback_context & solve_context_)
            : callback_handle_base(api, context_, solve_context_) {}

        // the relaxation point in column order, read once per call
        const double * _relaxation_values() {
            if(!_relaxation) {
                const auto num_vars = solve_context.num_columns;
                _relaxation =
                    std::make_unique_for_overwrite<double[]>(num_vars);
                cbcheck(CPX->callbackgetrelaxationpoint(
                    context, _relaxation.get(), 0,
                    static_cast<int>(num_vars) - 1, nullptr));
            }
            return _relaxation.get();
        }

        // CPLEX calls back again on a node whose relaxation its last cuts
        // did not move, which a cut can fail to do although the point
        // violates it as written: presolve may have aggregated its columns.
        // A separator that is a function of the point would offer the same
        // cuts again, and the node would never end.
        bool _repeats_the_last_call() {
            int thread = 0;
            CPXLONG node = -1;
            cbcheck(CPX->callbackgetinfoint(context, CPXCALLBACKINFO_THREADID,
                                            &thread));
            cbcheck(CPX->callbackgetinfolong(context, CPXCALLBACKINFO_NODEUID,
                                             &node));
            auto & last = solve_context.last_relaxation[thread];
            const double * point = _relaxation_values();
            const std::size_t num_vars = solve_context.num_columns;
            if(last.node == node && last.point.size() == num_vars &&
               std::equal(point, point + num_vars, last.point.begin()))
                return true;
            last.node = node;
            last.point.assign(point, point + num_vars);
            return false;
        }

        template <bool distinct, linear_constraint LC>
        void _add_user_cut(LC && lc) {
            if constexpr(!distinct)
                _prepare_coalescing(solve_context.num_columns);
            _reset_cache();
            const auto native_id = [this](const variable & v) {
                return model->_native_id(v);
            };
            if constexpr(distinct)
                _register_raw_entries(lc.linear_terms(), native_id);
            else
                _register_coalescing_entries(lc.linear_terms(), native_id);
            const auto cut = detail::prepare_user_cut(
                tmp_indices, tmp_scalars, lc.sense(),
                static_cast<double>(lc.rhs()), _relaxation_values());
            if(!cut) return;
            const int matbegin = 0;
            const char sense = constraint_sense_to_cplex_sense(cut->sense);
            const int purgeable = CPX_USECUT_PURGE;
            const int local = 0;
            // The return code is dropped. 1121 is CPLEX saying that it
            // cannot write the cut on its presolved model, and it then goes
            // without: aborting would end the search over an optional cut.
            CPX->callbackaddusercuts(
                context, 1, static_cast<int>(tmp_indices.size()), &cut->rhs,
                &sense, &matbegin, tmp_indices.data(), tmp_scalars.data(),
                &purgeable, &local);
        }

    public:
        template <linear_constraint LC>
        void add_user_cut(LC && lc) {
            _add_user_cut<false>(std::forward<LC>(lc));
        }
        template <linear_constraint LC>
        void add_user_cut(distinct_variables_t, LC && lc) {
            _add_user_cut<true>(std::forward<LC>(lc));
        }
        double get_solution_value() {
            double obj;
            // x == nullptr needs an empty [begin, end] range, else
            // CPXERR_NULL_POINTER
            cbcheck(
                CPX->callbackgetrelaxationpoint(context, nullptr, 0, -1, &obj));
            return obj;
        }
        // Captures neither the handle nor the model: a point kept past the
        // call would read a dead frame through the first, and another
        // variable's value through the second once a variable is removed.
        auto get_solution() {
            const auto num_vars = solve_context.num_columns;
            auto solution = std::make_unique_for_overwrite<double[]>(num_vars);
            std::copy_n(_relaxation_values(), num_vars, solution.get());
            return variable_mapping([native_id = model->_frozen_native_id(),
                                     solution = std::move(solution)](
                                        const variable & v) -> double & {
                return *(solution.get() + native_id(v));
            });
        }
        template <detail::point_of<variable, double> Point>
        void add_solution(Point && point) {
            _post_solution(point);
        }
    };

    // Only CPX_CALLBACKCONTEXT_GLOBAL_PROGRESS reads global values, and CPLEX
    // holds a lock across it. Any other context reads what one thread knows,
    // which may regress from one call to the next and run concurrently.
    class progress_callback_handle {
    private:
        friend cplex_milp;
        const cplex_api * CPX;
        CPXCALLBACKCONTEXTptr context;

        progress_callback_handle(const cplex_api * api,
                                 CPXCALLBACKCONTEXTptr context_)
            : CPX(api), context(context_) {}

        double _read(const CPXCALLBACKINFO what) {
            double value = 0;
            if(CPX->callbackgetinfodbl(context, what, &value) != 0)
                CPX->callbackabort(context);
            return detail::as_infinity(value, CPX_INFBOUND);
        }

    public:
        double get_solution_value() { return _read(CPXCALLBACKINFO_BEST_SOL); }
        double get_dual_bound() { return _read(CPXCALLBACKINFO_BEST_BND); }
        std::size_t get_node_count() {
            CPXLONG count = 0;
            if(CPX->callbackgetinfolong(context, CPXCALLBACKINFO_NODECOUNT,
                                        &count) != 0)
                CPX->callbackabort(context);
            return static_cast<std::size_t>(count);
        }
        void interrupt() { CPX->callbackabort(context); }
    };

private:
    std::function<void(candidate_solution_callback_handle &)>
        candidate_solution_callback;
    std::function<void(node_relaxation_callback_handle &)>
        node_relaxation_callback;
    std::function<void(progress_callback_handle &)> progress_callback;
    // set by the setter, cleared by the solve that honours it
    bool _candidate_callback_is_new = false;

    static int callback_fun(CPXCALLBACKCONTEXTptr context, CPXLONG contextid,
                            void * userhandle) {
        auto & solve_context = *static_cast<callback_context *>(userhandle);
        auto * model = solve_context.model;
        const std::scoped_lock lock(solve_context.mutex);
        // Each context names its callback: an else branch would hand any
        // context registered later to a callback that may not be set.
        if(contextid == CPX_CALLBACKCONTEXT_CANDIDATE) {
            candidate_solution_callback_handle handle(model->CPX, context,
                                                      solve_context);
            model->candidate_solution_callback(handle);
        } else if(contextid == CPX_CALLBACKCONTEXT_RELAXATION) {
            node_relaxation_callback_handle handle(model->CPX, context,
                                                   solve_context);
            if(handle._repeats_the_last_call()) return 0;
            model->node_relaxation_callback(handle);
        } else if(contextid == CPX_CALLBACKCONTEXT_GLOBAL_PROGRESS) {
            progress_callback_handle handle(model->CPX, context);
            model->progress_callback(handle);
        }
        return 0;
    }
    void _read_callback_context(callback_context & solve_context) {
        solve_context.num_columns = _num_var_native_ids();
        if((!candidate_solution_callback && !node_relaxation_callback) ||
           solve_context.num_columns == 0)
            return;
        solve_context.feasibility_tolerance = get_feasibility_tolerance();
        solve_context.objective.resize(solve_context.num_columns);
        check(CPX->getobj(env, lp, solve_context.objective.data(), 0,
                          static_cast<int>(solve_context.num_columns) - 1));
        check(CPX->getobjoffset(env, lp, &solve_context.objective_offset));
    }
    // For the duration of a branch-and-cut only. CPXlpopt and the conflict
    // refiner fail while a generic callback is registered, whatever it does
    // (CPXERR_UNSUPPORTED_OPERATION, measured on 22.1.1 and 22.1.2), and the
    // user data lives in solve()'s frame: registered by a setter, it would
    // dangle once the model moves and the next solve would crash.
    [[nodiscard]] auto _attached_callbacks(callback_context & solve_context) {
        const CPXLONG contexts =
            (candidate_solution_callback ? CPX_CALLBACKCONTEXT_CANDIDATE : 0) |
            (node_relaxation_callback ? CPX_CALLBACKCONTEXT_RELAXATION : 0) |
            (progress_callback ? CPX_CALLBACKCONTEXT_GLOBAL_PROGRESS : 0);
        if(contexts != 0)
            check(CPX->callbacksetfunc(env, lp, contexts, callback_fun,
                                       &solve_context));
        return detail::restore_guard([this, contexts] {
            if(contexts != 0)
                check(CPX->callbacksetfunc(env, lp, 0, nullptr, nullptr));
        });
    }

public:
    template <typename F>
    void set_candidate_solution_callback(F && f) {
        candidate_solution_callback = std::forward<F>(f);
        _candidate_callback_is_new = true;
    }
    template <typename F>
    void set_node_relaxation_callback(F && f) {
        node_relaxation_callback = std::forward<F>(f);
    }
    auto release_node_relaxation_callback() {
        return std::exchange(node_relaxation_callback, nullptr);
    }
    template <typename F>
    void set_progress_callback(F && f) {
        progress_callback = std::forward<F>(f);
    }
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// MIP start ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    template <typename ER>
    inline void _add_mip_start(ER && entries) {
        _reset_cache();
        _register_variables_entries<true>(entries);
        int beg = 0;
        int effort_level = CPX_MIPSTART_SOLVEMIP;
        check(CPX->addmipstarts(
            env, lp, 1, static_cast<int>(tmp_indices.size()), &beg,
            tmp_indices.data(), tmp_scalars.data(), &effort_level, nullptr));
    }

public:
    template <std::ranges::range ER>
    void add_mip_start(ER && entries) {
        _invalidate_results();
        _add_mip_start(entries);
    }
    void add_mip_start(
        std::initializer_list<std::pair<variable, scalar>> entries) {
        _invalidate_results();
        _add_mip_start(entries);
    }
    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////// Limits //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void set_node_limit(std::size_t count) {
        check(CPX->setintparam(env, CPXPARAM_MIP_Limits_Nodes,
                               static_cast<int>(count)));
    }
    auto get_node_limit() {
        int count;
        check(CPX->getintparam(env, CPXPARAM_MIP_Limits_Nodes, &count));
        return static_cast<std::size_t>(count);
    }
    void set_solution_limit(std::size_t count) {
        check(CPX->setintparam(env, CPXPARAM_MIP_Limits_Solutions,
                               static_cast<int>(count)));
    }
    auto get_solution_limit() {
        int count;
        check(CPX->getintparam(env, CPXPARAM_MIP_Limits_Solutions, &count));
        return static_cast<std::size_t>(count);
    }
    void set_memory_limit(memory_size<double, mebi> mb) {
        check(
            CPX->setdblparam(env, CPXPARAM_MIP_Limits_TreeMemory, mb.count()));
    }
    auto get_memory_limit() {
        double mb;
        check(CPX->getdblparam(env, CPXPARAM_MIP_Limits_TreeMemory, &mb));
        return memory_size<double, mebi>(mb);
    }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////// Tolerance parameters ///////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void set_optimality_tolerance(double tol) {
        check(CPX->setdblparam(env, CPXPARAM_MIP_Tolerances_MIPGap, tol));
    }
    double get_optimality_tolerance() {
        double tol;
        check(CPX->getdblparam(env, CPXPARAM_MIP_Tolerances_MIPGap, &tol));
        return tol;
    }
    // EpRHS, as on cplex_lp: what CPLEX lets a row or a bound be violated
    // by, in a MIP too. Not MIP.Tolerances.Linearization, the one tolerance
    // of the MIP group that reads like it: that is the epsilon of Concert's
    // reformulations, and the rows of a MIP stay at EpRHS whatever it is.
    void set_feasibility_tolerance(double tol) {
        check(CPX->setdblparam(env, CPXPARAM_Simplex_Tolerances_Feasibility,
                               tol));
    }
    double get_feasibility_tolerance() {
        double tol;
        check(CPX->getdblparam(env, CPXPARAM_Simplex_Tolerances_Feasibility,
                               &tol));
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
            status::optimal_face_unbounded,
            status::optimal_infeasible_unscaled,
            status::infeasible_or_unbounded,
            status::infeasible,
            status::unbounded,
            status::limit_reached,
            status::time_limit,
            status::iteration_limit,
            status::node_limit, 
            status::solution_limit,
            status::memory_limit,
            status::failed,
            status::numerical_failure,
            status::out_of_memory,
            status::interrupted>;

    status_variant _status = status::unknown{};

    status_variant _get_status_milp() {
        using namespace status;
        int status = CPX->getstat(env, lp);
        if(status == CPX_STAT_UNKNOWN) return unknown{};
        int pfeasind;
        check(CPX->solninfo(env, lp, nullptr, nullptr, &pfeasind, nullptr));
        const bool has_sol = pfeasind != 0;
        switch(status) {
            case CPXMIP_OPTIMAL:
            case CPXMIP_OPTIMAL_TOL:     return optimal{};
            case CPXMIP_OPTIMAL_INFEAS:  return optimal_infeasible_unscaled{};
            case CPXMIP_INForUNBD:       return infeasible_or_unbounded{};
            case CPXMIP_UNBOUNDED:       return unbounded{};
            case CPXMIP_INFEASIBLE:      return infeasible{};
            case CPXMIP_TIME_LIM_FEAS:
            case CPXMIP_TIME_LIM_INFEAS: return time_limit{has_sol};
            case CPXMIP_NODE_LIM_FEAS:
            case CPXMIP_NODE_LIM_INFEAS: return node_limit{has_sol};
            case CPXMIP_SOL_LIM:         return solution_limit{has_sol};
            case CPXMIP_MEM_LIM_FEAS:
            case CPXMIP_MEM_LIM_INFEAS:  return memory_limit{has_sol};
            case CPXMIP_FAIL_FEAS:       
            case CPXMIP_FAIL_INFEAS:     return failed{has_sol};
            case CPXMIP_FAIL_FEAS_NO_TREE:
            case CPXMIP_FAIL_INFEAS_NO_TREE: return out_of_memory{has_sol};
            case CPXMIP_ABORT_FEAS:      
            case CPXMIP_ABORT_INFEAS:        return interrupted{has_sol};
            // no dedicated status type: these collapse into limit_reached
            case CPXMIP_DETTIME_LIM_FEAS:
            case CPXMIP_DETTIME_LIM_INFEAS:  return limit_reached{has_sol};
            default:
                return unknown{};
        }
    }
    status_variant _get_status_lp() {
        using namespace status;
        switch(CPX->getstat(env, lp)) {
            case CPX_STAT_OPTIMAL:        return optimal{};
            case CPX_STAT_OPTIMAL_FACE_UNBOUNDED: 
                                          return optimal_face_unbounded{};
            case CPX_STAT_OPTIMAL_INFEAS: return optimal_infeasible_unscaled{};
            case CPX_STAT_INForUNBD:      return infeasible_or_unbounded{};
            case CPX_STAT_INFEASIBLE:     return infeasible{};
            case CPX_STAT_UNBOUNDED:      return unbounded{};
            case CPX_STAT_ABORT_TIME_LIM: return time_limit{};
            case CPX_STAT_ABORT_IT_LIM:   return iteration_limit{};
            case CPX_STAT_NUM_BEST:       return numerical_failure{true};
            case CPX_STAT_ABORT_USER:     return interrupted{};
            // no dedicated status type: these collapse into limit_reached
            case CPX_STAT_ABORT_OBJ_LIM:      return limit_reached{true};
            case CPX_STAT_ABORT_PRIM_OBJ_LIM: return limit_reached{};
            case CPX_STAT_ABORT_DUAL_OBJ_LIM: return limit_reached{};
            case CPX_STAT_ABORT_DETTIME_LIM:  return limit_reached{};
            case CPX_STAT_UNKNOWN:            return unknown{};
            default:
                return unknown{};
        }
    }
    // clang-format on
public:
    const status_variant & get_status() const {
        return _results_valid ? _status
                              : detail::unknown_status<status_variant>;
    }
    void reset_status() noexcept {
        _status = status::unknown{};
        _invalidate_results();
    }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////////// Solve //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void solve() {
        reset_status();  // a solve that throws reports unknown
        int probtype = CPX->getprobtype(env, lp);
        switch(probtype) {
            case CPXPROB_MILP: {
                // A callback is no change to CPLEX: after a finished solve,
                // CPXmipopt hands back the solution it holds without asking a
                // candidate callback set since, which may forbid it. Writing
                // an objective coefficient back is a change.
                if(std::exchange(_candidate_callback_is_new, false)) {
                    const int first = 0;
                    double coefficient;
                    check(CPX->getobj(env, lp, &coefficient, first, first));
                    check(CPX->chgobj(env, lp, 1, &first, &coefficient));
                }
                callback_context solve_context{.model = this,
                                               .num_columns = 0,
                                               .objective = {},
                                               .objective_offset = 0.0,
                                               .feasibility_tolerance = 0.0,
                                               .last_relaxation = {},
                                               .mutex = {}};
                _read_callback_context(solve_context);
                const auto detach = _attached_callbacks(solve_context);
                check(CPX->mipopt(env, lp));
                _status = _get_status_milp();
                _mark_results_valid();
                return;
            }
            case CPXPROB_LP:
                if(candidate_solution_callback)
                    throw std::runtime_error(
                        "cplex_milp: can't solve lp (no integer variables) "
                        "with candidate_solution_callback");
                check(CPX->lpopt(env, lp));
                _status = _get_status_lp();
                _mark_results_valid();
                return;
            default:
                throw std::runtime_error("cplex_milp: unknown problem type " +
                                         std::to_string(probtype));
        }
    }

    // The refiner replaces CPXgetstat with its own statuses, which do not
    // describe the held solution: the reported status is reset before the
    // native call, throw or return.
    auto compute_iis() {
        reset_status();
        return _compute_iis();
    }
    double get_solution_value() {
        _require_valid_results();
        double val;
        check(CPX->solution(env, lp, nullptr, &val, nullptr, nullptr, nullptr,
                            nullptr));
        return val;
    }
    auto get_solution() {
        _require_valid_results();
        auto solution =
            std::make_unique_for_overwrite<double[]>(_num_var_native_ids());
        check(CPX->solution(env, lp, nullptr, nullptr, solution.get(), nullptr,
                            nullptr, nullptr));
        return variable_mapping(
            [native_id = _frozen_native_id(),
             solution = std::move(solution)](const variable & v) {
                return *(solution.get() + native_id(v));
            });
    }
    double get_dual_bound() {
        _require_valid_results();
        if(CPX->getprobtype(env, lp) != CPXPROB_MILP)
            return get_solution_value();
        double bound;
        check(CPX->getbestobjval(env, lp, &bound));
        return bound;
    }
    std::size_t get_node_count() {
        _require_valid_results();
        if(CPX->getprobtype(env, lp) != CPXPROB_MILP) return 0u;
        return static_cast<std::size_t>(CPX->getnodecnt(env, lp));
    }
};

}  // namespace cplex::impl::v1
}  // namespace mippp
