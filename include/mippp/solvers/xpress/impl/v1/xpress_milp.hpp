#pragma once

#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <ranges>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>

#include "mippp/detail/restore_guard.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/xpress/impl/v1/xpress_base.hpp"

namespace mippp {
namespace xpress::impl::v1 {

class xpress_milp : public xpress_base {
public:
    [[nodiscard]] xpress_milp() : xpress_milp(xpress_api::load()) {}
    [[nodiscard]] explicit xpress_milp(const xpress_api & api)
        : xpress_base(api) {}

    using model_base<int, double>::add_integer_variable;
    using model_base<int, double>::add_integer_variables;
    using model_base<int, double>::add_binary_variable;
    using model_base<int, double>::add_binary_variables;

    void set_continuous(variable v) {
        _invalidate_results();
        int var_id = v.id();
        char type = 'C';
        check(XPRS->chgcoltype(prob, 1, &var_id, &type));
    }
    void set_integer(variable v) {
        _invalidate_results();
        int var_id = v.id();
        char type = 'I';
        check(XPRS->chgcoltype(prob, 1, &var_id, &type));
    }
    void set_binary(variable v) {
        _invalidate_results();
        int var_id = v.id();
        char type = 'B';
        check(XPRS->chgcoltype(prob, 1, &var_id, &type));
    }
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// Callbacks ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    class callback_handle_base : protected model_base<int, double> {
    protected:
        const xpress_api * XPRS;
        XPRSprob prob;
        const double objective_offset;

        void check(int error) {
            if(error == 0) return;
            char errmsg[512];
            XPRS->getlasterror(prob, errmsg);
            throw std::runtime_error("Xpress: error " + std::to_string(error) +
                                     ": " + errmsg);
        }

        callback_handle_base(const xpress_api * api, XPRSprob prob_,
                             const double obj_offset)
            : model_base<int, double>()
            , XPRS(api)
            , prob(prob_)
            , objective_offset(obj_offset) {}

    public:
        // XPRS_COLS counts the presolved matrix inside a callback; solutions
        // come back in the input space, sized XPRS_INPUTCOLS
        std::size_t num_variables() {
            int num_vars;
            check(XPRS->getintattrib(prob, XPRS_INPUTCOLS, &num_vars));
            return static_cast<std::size_t>(num_vars);
        }
    };

public:
    class candidate_solution_callback_handle : public callback_handle_base {
    private:
        int * _reject;

        friend xpress_milp;
        candidate_solution_callback_handle(const xpress_api * api,
                                           XPRSprob prob_,
                                           const double obj_offset,
                                           int * reject)
            : callback_handle_base(api, prob_, obj_offset), _reject(reject) {}

    public:
        void reject_solution() { *_reject = 1; }

        // the incumbent is not updated yet in preintsol: XPRS_MIPOBJVAL and
        // XPRSgetsolution would describe the previous one, not the candidate
        double get_solution_value() {
            double val;
            check(XPRS->getdblattrib(prob, XPRS_LPOBJVAL, &val));
            return objective_offset + val;
        }
        auto get_solution() {
            const auto num_vars = num_variables();
            auto solution = std::make_unique_for_overwrite<double[]>(num_vars);
            check(XPRS->getcallbacksolution(prob, nullptr, solution.get(), 0,
                                            static_cast<int>(num_vars) - 1));
            return variable_mapping(std::move(solution));
        }
    };

    // XPRS_MIPOBJVAL and XPRS_BESTBOUND leave out the wrapper's offset.
    class progress_callback_handle {
    private:
        friend xpress_milp;
        const xpress_api * XPRS;
        XPRSprob prob;
        const double objective_offset;

        progress_callback_handle(const xpress_api * api, XPRSprob prob_,
                                 const double obj_offset)
            : XPRS(api), prob(prob_), objective_offset(obj_offset) {}

        double _read(const int attribute) {
            double value;
            XPRS->_check(prob, XPRS->getdblattrib(prob, attribute, &value));
            return objective_offset +
                   detail::as_infinity(value, XPRS_PLUSINFINITY);
        }

    public:
        double get_solution_value() { return _read(XPRS_MIPOBJVAL); }
        double get_dual_bound() { return _read(XPRS_BESTBOUND); }
        std::size_t get_node_count() {
            int count;
            XPRS->_check(prob, XPRS->getintattrib(prob, XPRS_NODES, &count));
            return static_cast<std::size_t>(count);
        }
        void interrupt() {
            XPRS->_check(prob, XPRS->interrupt(prob, XPRS_STOP_USER));
        }
    };

private:
    std::function<void(candidate_solution_callback_handle &)>
        candidate_solution_callback;
    std::function<void(progress_callback_handle &)> progress_callback;

    // MUTEXCALLBACKS does not list miplog, which runs on the worker threads,
    // and Xpress promises nothing between its callbacks of different kinds:
    // without this mutex nothing says that two calls do not overlap. Lives
    // for one solve, as a member it would cost the model its moves.
    struct callback_context {
        xpress_milp * model;
        std::mutex mutex;
    };
    static void candidate_solution_callback_fun(
        XPRSprob cbprob, void * cbdata, [[maybe_unused]] int soltype,
        int * p_reject, [[maybe_unused]] double * p_cutoff) {
        auto & context = *static_cast<callback_context *>(cbdata);
        const std::scoped_lock lock(context.mutex);
        candidate_solution_callback_handle handle(
            context.model->XPRS, cbprob, context.model->objective_offset,
            p_reject);
        context.model->candidate_solution_callback(handle);
    }
    static void report_progress(XPRSprob cbprob, void * cbdata) {
        auto & context = *static_cast<callback_context *>(cbdata);
        const std::scoped_lock lock(context.mutex);
        progress_callback_handle handle(context.model->XPRS, cbprob,
                                        context.model->objective_offset);
        context.model->progress_callback(handle);
    }
    static int progress_miplog_fun(XPRSprob cbprob, void * cbdata) {
        report_progress(cbprob, cbdata);
        return 0;
    }
    // For the duration of a solve only. Each addcb adds one more call, so a
    // setter registering would run the callback once per registration. The
    // user data lives in solve()'s frame: registered by a setter, it would
    // dangle once the model moves and the next solve would crash. And the
    // MIP solves of compute_iis() would run the candidate callback, whose
    // rejection makes a feasible subsystem read infeasible and the answer
    // name the whole model.
    [[nodiscard]] auto _attached_callbacks(callback_context & context) {
        const bool candidate = static_cast<bool>(candidate_solution_callback);
        const bool progress = static_cast<bool>(progress_callback);
        if(candidate)
            check(XPRS->addcbpreintsol(prob, candidate_solution_callback_fun,
                                       &context, 1));
        if(progress) {
            check(XPRS->addcbmiplog(prob, progress_miplog_fun, &context, 0));
            check(XPRS->addcbintsol(prob, report_progress, &context, 0));
        }
        return detail::restore_guard([this, candidate, progress, &context] {
            if(candidate)
                check(XPRS->removecbpreintsol(
                    prob, candidate_solution_callback_fun, &context));
            if(progress) {
                check(
                    XPRS->removecbmiplog(prob, progress_miplog_fun, &context));
                check(XPRS->removecbintsol(prob, report_progress, &context));
            }
        });
    }

public:
    template <typename F>
    void set_candidate_solution_callback(F && f) {
        candidate_solution_callback = std::forward<F>(f);
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
        check(XPRS->addmipsol(prob, static_cast<int>(tmp_indices.size()),
                              tmp_scalars.data(), tmp_indices.data(), nullptr));
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
    ////////////////////////// Tolerance parameters ///////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void set_optimality_tolerance(double tol) {
        check(XPRS->setdblcontrol(prob, XPRS_MIPRELSTOP, tol));
    }
    double get_optimality_tolerance() {
        double tol;
        check(XPRS->getdblcontrol(prob, XPRS_MIPRELSTOP, &tol));
        return tol;
    }
    void set_integrality_tolerance(double tol) {
        check(XPRS->setdblcontrol(prob, XPRS_MIPTOL, tol));
    }
    double get_integrality_tolerance() {
        double tol;
        check(XPRS->getdblcontrol(prob, XPRS_MIPTOL, &tol));
        return tol;
    }
    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////// Limits //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void set_time_limit(std::chrono::duration<double> t) {
        check(XPRS->setdblcontrol(prob, XPRS_TIMELIMIT, t.count()));
    }
    auto get_time_limit() {
        double t;
        check(XPRS->getdblcontrol(prob, XPRS_TIMELIMIT, &t));
        return std::chrono::duration<double>(t);
    }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////// Solve status ///////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    // clang-format off
private:
    using status_variant = std::variant<
            status::unknown,
            status::optimal,
            status::infeasible,
            status::unbounded,
            status::limit_reached,
            status::time_limit,
            status::iteration_limit,
            status::node_limit, 
            status::solution_limit,
            status::failed,
            status::numerical_failure,
            status::out_of_memory,
            status::interrupted>;

    status_variant _status = status::unknown{};

    status_variant _get_status() {
        using namespace status;
        int stop_status;
        check(XPRS->getintattrib(prob, XPRS_STOPSTATUS, &stop_status));
        int lp_status;
        check(XPRS->getintattrib(prob, XPRS_MIPSTATUS, &lp_status)); 
        const bool has_sol = (lp_status ==  XPRS_MIP_SOLUTION);  
        switch(stop_status) {
            case XPRS_STOP_NONE:
            case XPRS_STOP_MIPGAP:
            case XPRS_STOP_SOLVECOMPLETE: {   
                switch(lp_status) {
                    case XPRS_MIP_OPTIMAL:  return optimal{};
                    case XPRS_MIP_INFEAS:   return infeasible{};
                    case XPRS_MIP_UNBOUNDED:    return unbounded{};
                    case XPRS_MIP_NOT_LOADED:
                    default:
                        return unknown{has_sol};
                } 
            }
            case XPRS_STOP_TIMELIMIT:      return time_limit{has_sol};
            case XPRS_STOP_ITERLIMIT:      return iteration_limit{has_sol};
            case XPRS_STOP_NODELIMIT:      return node_limit{has_sol};
            case XPRS_STOP_SOLLIMIT:       return solution_limit{has_sol};
            case XPRS_STOP_WORKLIMIT:      return limit_reached{has_sol};
            case XPRS_STOP_MEMORYERROR:    return out_of_memory{has_sol};
            case XPRS_STOP_NUMERICALERROR: return numerical_failure{has_sol};
            case XPRS_STOP_GENERICERROR:   return failed{has_sol};
            case XPRS_STOP_USER:
            case XPRS_STOP_LICENSELOST:
            case XPRS_STOP_CTRLC:          return interrupted{has_sol};
            default:
                return unknown{has_sol};
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
        if(_bounds_hold_no_value()) {
            _status = status::infeasible{};
            _mark_results_valid();
            return;
        }
        callback_context context{this, {}};
        const auto detach = _attached_callbacks(context);
        check(XPRS->mipoptimize(prob, nullptr));
        _status = _get_status();
        // A stopped search leaves the problem presolved, where XPRS_COLS and
        // the column getters and setters describe the presolved columns.
        // Postsolving keeps the incumbent but drops the search tree.
        check(XPRS->postsolve(prob));
        _mark_results_valid();
    }

    // The routine solves on its own and overwrites the status attributes and
    // the held solution: the reported status is reset before the first native
    // call, throw or return.
    // A model without integer columns is an LP, on which the routine names
    // the side of a ranged row reliably.
    auto compute_iis() {
        reset_status();
        int mip_entities;
        check(XPRS->getintattrib(prob, XPRS_ORIGINALMIPENTS, &mip_entities));
        return _compute_iis<detail::iis_sided_status,
                            detail::iis_whole_or_sided_status>(mip_entities >
                                                               0);
    }

    double get_solution_value() {
        _require_valid_results();
        double val;
        check(XPRS->getdblattrib(prob, XPRS_MIPOBJVAL, &val));
        return objective_offset + val;
    }
    auto get_solution() {
        _require_valid_results();
        const auto num_vars = num_variables();
        auto solution = std::make_unique_for_overwrite<double[]>(num_vars);
        check(XPRS->getsolution(prob, nullptr, solution.get(), 0,
                                static_cast<int>(num_vars) - 1));
        return variable_mapping(std::move(solution));
    }

private:
    bool _has_mip_entities() {
        int mip_entities;
        check(XPRS->getintattrib(prob, XPRS_ORIGINALMIPENTS, &mip_entities));
        return mip_entities > 0;
    }

public:
    double get_dual_bound() {
        _require_valid_results();
        if(!_has_mip_entities()) return get_solution_value();
        double bound;
        check(XPRS->getdblattrib(prob, XPRS_BESTBOUND, &bound));
        return objective_offset + bound;
    }
    std::size_t get_node_count() {
        _require_valid_results();
        if(!_has_mip_entities()) return 0u;
        int count;
        check(XPRS->getintattrib(prob, XPRS_NODES, &count));
        return static_cast<std::size_t>(count);
    }
};

}  // namespace xpress::impl::v1
}  // namespace mippp
