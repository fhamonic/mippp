#pragma once

#include <algorithm>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <ranges>
#include <utility>
#include <variant>

#include "mippp/detail/restore_guard.hpp"
#include "mippp/detail/row_preparation.hpp"
#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/copt/impl/v1/copt_base.hpp"

namespace mippp {
namespace copt::impl::v1 {

class copt_milp : public copt_base {
private:
    int _is_mip = 0;

public:
    [[nodiscard]] copt_milp() : copt_milp(copt_api::load()) {}
    [[nodiscard]] explicit copt_milp(const copt_api & api) : copt_base(api) {}

    using model_base<int, double>::add_integer_variable;
    using model_base<int, double>::add_integer_variables;
    using model_base<int, double>::add_binary_variable;
    using model_base<int, double>::add_binary_variables;

private:
    inline void _add_binary_variables(const std::size_t & count) {
        tmp_scalars.assign(count, 0.0);
        tmp_types.assign(count, COPT_BINARY);
        check(COPT->AddCols(prob, static_cast<int>(count), tmp_scalars.data(),
                            nullptr, nullptr, nullptr, nullptr,
                            tmp_types.data(), nullptr, nullptr, nullptr));
    }

public:
    void set_continuous(variable v) {
        _invalidate_results();
        int var_id = v.id();
        char type = COPT_CONTINUOUS;
        check(COPT->SetColType(prob, 1, &var_id, &type));
    }
    void set_integer(variable v) {
        _invalidate_results();
        int var_id = v.id();
        char type = COPT_INTEGER;
        check(COPT->SetColType(prob, 1, &var_id, &type));
    }
    void set_binary(variable v) {
        _invalidate_results();
        int var_id = v.id();
        char type = COPT_BINARY;
        check(COPT->SetColType(prob, 1, &var_id, &type));
    }
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// Callbacks ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    class candidate_solution_callback_handle
        : protected model_base<int, double> {
    private:
        const copt_api * COPT;
        copt_prob * prob;
        void * cbdata;
        const double feasibility_tolerance;
        std::unique_ptr<double[]> _candidate;

        friend copt_milp;
        candidate_solution_callback_handle(const copt_api * api,
                                           copt_prob * prob_, void * cbdata_,
                                           const double feasibility_tolerance_)
            : model_base<int, double>()
            , COPT(api)
            , prob(prob_)
            , cbdata(cbdata_)
            , feasibility_tolerance(feasibility_tolerance_) {}

        void check(const ret_code error) { COPT->_check(nullptr, error); }

        // the candidate in column order, read once per call
        const double * _candidate_values() {
            if(!_candidate) {
                _candidate =
                    std::make_unique_for_overwrite<double[]>(num_variables());
                check(COPT->GetCallbackInfo(cbdata, COPT_CBINFO_MIPCANDIDATE,
                                            _candidate.get()));
            }
            return _candidate.get();
        }

    public:
        std::size_t num_variables() {
            int num;
            check(COPT->GetIntAttr(prob, COPT_INTATTR_COLS, &num));
            return static_cast<std::size_t>(num);
        }

    private:
        template <bool distinct, linear_constraint LC>
        void _add_lazy_constraint(LC && lc) {
            if constexpr(!distinct) _prepare_coalescing(num_variables());
            _reset_cache();
            _register_variables_entries<distinct>(lc.linear_terms());
            // COPT gives a wrong optimum or a false infeasible on a callback
            // row holding an exact zero, and answers an empty one with an
            // error that check() would throw across COPT_Solve's frames.
            detail::strip_zero_entries(tmp_indices, tmp_scalars);
            if(tmp_indices.empty()) return;
            // COPT rejects the candidate whatever the row says: handed one
            // the candidate satisfies, it loses that candidate, and a
            // feasible model can end infeasible. Written `!(... > ...)` so
            // that a row holding a NaN is not passed on.
            if(!(detail::row_violation(tmp_indices, tmp_scalars, lc.sense(),
                                       static_cast<double>(lc.rhs()),
                                       _candidate_values()) >
                 feasibility_tolerance))
                return;
            check(COPT->AddCallbackLazyConstr(
                cbdata, static_cast<int>(tmp_indices.size()),
                tmp_indices.data(), tmp_scalars.data(),
                constraint_sense_to_copt_sense(lc.sense()), lc.rhs()));
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
            check(COPT->GetCallbackInfo(cbdata, COPT_CBINFO_MIPCANDOBJ, &obj));
            return obj;
        }
        auto get_solution() {
            auto num_vars = num_variables();
            auto solution = std::make_unique_for_overwrite<double[]>(num_vars);
            std::copy_n(_candidate_values(), num_vars, solution.get());
            return variable_mapping(std::move(solution));
        }
        // COPT takes the point at once, against the rows it holds, and never
        // shows it to this callback: a point a lazy constraint not added yet
        // would cut off becomes the incumbent. Only complete points count, an
        // undefined entry makes COPT ignore the call. The return code is
        // dropped: thrown from here, an error would cross COPT_Solve's frames.
        template <detail::point_of<variable, double> Point>
        void add_solution(Point && point) {
            _register_point(point, num_variables(),
                            [](const int id) { return variable(id); });
            double value;
            COPT->AddCallbackSolution(cbdata, tmp_scalars.data(), &value);
        }
    };

    class node_relaxation_callback_handle : protected model_base<int, double> {
    private:
        const copt_api * COPT;
        copt_prob * prob;
        void * cbdata;
        std::unique_ptr<double[]> _relaxation;

        friend copt_milp;
        node_relaxation_callback_handle(const copt_api * api, copt_prob * prob_,
                                        void * cbdata_)
            : model_base<int, double>()
            , COPT(api)
            , prob(prob_)
            , cbdata(cbdata_) {}

        void check(const ret_code error) { COPT->_check(nullptr, error); }

        // the relaxation point in column order, read once per call
        const double * _relaxation_values() {
            if(!_relaxation) {
                _relaxation =
                    std::make_unique_for_overwrite<double[]>(num_variables());
                check(COPT->GetCallbackInfo(cbdata, COPT_CBINFO_RELAXSOLUTION,
                                            _relaxation.get()));
            }
            return _relaxation.get();
        }

        template <bool distinct, linear_constraint LC>
        void _add_user_cut(LC && lc) {
            if constexpr(!distinct) _prepare_coalescing(num_variables());
            _reset_cache();
            _register_variables_entries<distinct>(lc.linear_terms());
            const auto cut = detail::prepare_user_cut(
                tmp_indices, tmp_scalars, lc.sense(),
                static_cast<double>(lc.rhs()), _relaxation_values());
            if(!cut) return;
            // The return code is dropped, as the cut is when COPT refuses
            // it: thrown from here, an error would cross COPT_Solve's frames.
            COPT->AddCallbackUserCut(
                cbdata, static_cast<int>(tmp_indices.size()),
                tmp_indices.data(), tmp_scalars.data(),
                constraint_sense_to_copt_sense(cut->sense), cut->rhs);
        }

    public:
        std::size_t num_variables() {
            int num;
            check(COPT->GetIntAttr(prob, COPT_INTATTR_COLS, &num));
            return static_cast<std::size_t>(num);
        }
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
            check(COPT->GetCallbackInfo(cbdata, COPT_CBINFO_RELAXSOLOBJ, &obj));
            return obj;
        }
        auto get_solution() {
            auto num_vars = num_variables();
            auto solution = std::make_unique_for_overwrite<double[]>(num_vars);
            std::copy_n(_relaxation_values(), num_vars, solution.get());
            return variable_mapping(std::move(solution));
        }
        // COPT takes the point at once, against the rows it holds, and never
        // shows it to the candidate callback: a point a lazy constraint not
        // added yet would cut off becomes the incumbent. The return code is
        // dropped: thrown from here, an error would cross COPT_Solve's frames.
        template <detail::point_of<variable, double> Point>
        void add_solution(Point && point) {
            _register_point(point, num_variables(),
                            [](const int id) { return variable(id); });
            double value;
            COPT->AddCallbackSolution(cbdata, tmp_scalars.data(), &value);
        }
    };

    // No get_node_count(): NodeCnt reads 0 inside a callback, with no error.
    class progress_callback_handle {
    private:
        friend copt_milp;
        const copt_api * COPT;
        copt_prob * prob;
        void * cbdata;

        progress_callback_handle(const copt_api * api, copt_prob * prob_,
                                 void * cbdata_)
            : COPT(api), prob(prob_), cbdata(cbdata_) {}

        double _read(const char * info) {
            double value;
            COPT->_check(nullptr, COPT->GetCallbackInfo(cbdata, info, &value));
            return detail::as_infinity(value, COPT_INFINITY);
        }

    public:
        double get_solution_value() { return _read(COPT_CBINFO_BESTOBJ); }
        double get_dual_bound() { return _read(COPT_CBINFO_BESTBND); }
        void interrupt() { COPT->_check(nullptr, COPT->Interrupt(prob)); }
    };

private:
    std::function<void(candidate_solution_callback_handle &)> solution_callback;
    std::function<void(node_relaxation_callback_handle &)>
        node_relaxation_callback;
    std::function<void(progress_callback_handle &)> progress_callback;
    // COPT drops the bound of a search stopped without an incumbent: BestBnd
    // and ObjBound then read +1e30 in either sense, which when minimizing
    // claims a bound no search proved. Only a callback sees it while the search
    // runs.
    double _search_bound = 0.0;

    // COPT's guide promises no serialization of its callbacks: none was seen
    // to overlap, and the mutex is what makes "never two at once" a promise.
    // It lives for one solve, as a member it would cost the model its moves,
    // and a static one would deadlock a solve nested in a callback.
    struct callback_context {
        copt_milp * model;
        // FeasTol, read before COPT_Solve
        double feasibility_tolerance;
        std::mutex mutex;
    };
    static int callback_func(copt_prob * prob, void * cbdata, int cbctx,
                             void * userdata) {
        auto & context = *static_cast<callback_context *>(userdata);
        auto * model = context.model;
        const std::scoped_lock lock(context.mutex);
        if(cbctx == COPT_CBCONTEXT_MIPSOL) {
            candidate_solution_callback_handle handle(
                model->COPT, prob, cbdata, context.feasibility_tolerance);
            model->solution_callback(handle);
            return 0;
        }
        // Returns before the path below, which is MIPNODE's and INCUMBENT's:
        // falling through would run the progress callback on every relaxation.
        if(cbctx == COPT_CBCONTEXT_MIPRELAX) {
            node_relaxation_callback_handle handle(model->COPT, prob, cbdata);
            model->node_relaxation_callback(handle);
            return 0;
        }
        model->check(model->COPT->GetCallbackInfo(cbdata, COPT_CBINFO_BESTBND,
                                                  &model->_search_bound));
        if(model->progress_callback) {
            progress_callback_handle handle(model->COPT, prob, cbdata);
            model->progress_callback(handle);
        }
        return 0;
    }
    // For the duration of a solve only. The user data lives in solve()'s
    // frame: registered by a setter, it would dangle once the model moves and
    // the next solve would crash. And the first solve of compute_iis() would
    // run the candidate callback, whose lazy constraints can make a feasible
    // model infeasible, and the routine then flags the whole model.
    // Progress stays off MIPSOL: that context alone acts as LazyConstraints=1,
    // which turns presolve reductions off and changes the search. MIPNODE,
    // which keeps _search_bound, leaves it alone and costs nothing measurable.
    [[nodiscard]] auto _attached_callbacks(callback_context & context) {
        int sense;
        check(COPT->GetIntAttr(prob, COPT_INTATTR_OBJSENSE, &sense));
        _search_bound = sense == COPT_MINIMIZE ? -COPT_INFINITY : COPT_INFINITY;
        const int contexts =
            COPT_CBCONTEXT_MIPNODE |
            (solution_callback ? COPT_CBCONTEXT_MIPSOL : 0) |
            (node_relaxation_callback ? COPT_CBCONTEXT_MIPRELAX : 0) |
            (progress_callback ? COPT_CBCONTEXT_INCUMBENT : 0);
        check(COPT->SetCallback(prob, callback_func, contexts, &context));
        return detail::restore_guard(
            [this] { check(COPT->SetCallback(prob, nullptr, 0, nullptr)); });
    }

public:
    template <typename F>
    void set_candidate_solution_callback(F && f) {
        solution_callback = std::forward<F>(f);
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
        check(COPT->AddMipStart(prob, static_cast<int>(tmp_indices.size()),
                                tmp_indices.data(), tmp_scalars.data()));
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
        check(COPT->SetDblParam(prob, COPT_DBLPARAM_RELGAP, tol));
    }
    double get_optimality_tolerance() {
        double tol;
        check(COPT->GetDblParam(prob, COPT_DBLPARAM_RELGAP, &tol));
        return tol;
    }
    void set_feasibility_tolerance(double tol) {
        check(COPT->SetDblParam(prob, COPT_DBLPARAM_FEASTOL, tol));
    }
    double get_feasibility_tolerance() {
        double tol;
        check(COPT->GetDblParam(prob, COPT_DBLPARAM_FEASTOL, &tol));
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
            status::time_limit,
            status::node_limit,
            status::numerical_failure,
            status::interrupted>;

    status_variant _status = status::unknown{};

    status_variant _get_status_lp() {
        using namespace status;
        int status_;
        check(COPT->GetIntAttr(prob, COPT_INTATTR_LPSTATUS, &status_));
        int has_sol;
        check(COPT->GetIntAttr(prob, COPT_INTATTR_HASLPSOL, &has_sol));
        switch(status_) {
            case COPT_LPSTATUS_OPTIMAL:     return optimal{};
            case COPT_LPSTATUS_INFEASIBLE:  return infeasible{};
            case COPT_LPSTATUS_UNBOUNDED:   return unbounded{};
            case COPT_LPSTATUS_TIMEOUT:     return time_limit{has_sol != 0};
            case COPT_LPSTATUS_NUMERICAL:
            case COPT_LPSTATUS_IMPRECISE:
            case COPT_LPSTATUS_UNFINISHED:  return numerical_failure{has_sol != 0};
            case COPT_LPSTATUS_INTERRUPTED: return interrupted{has_sol != 0};
            case COPT_LPSTATUS_UNSTARTED:   return unknown{};
            default:
                return unknown{has_sol != 0};
        }
    }
    status_variant _get_status_milp() {
        using namespace status;
        int status_;
        check(COPT->GetIntAttr(prob, COPT_INTATTR_MIPSTATUS, &status_));
        int has_sol;
        check(COPT->GetIntAttr(prob, COPT_INTATTR_HASMIPSOL, &has_sol));
        switch(status_) {
            case COPT_MIPSTATUS_OPTIMAL:     return optimal{};
            case COPT_MIPSTATUS_INF_OR_UNB:  return infeasible_or_unbounded{};
            case COPT_MIPSTATUS_INFEASIBLE:  return infeasible{};
            case COPT_MIPSTATUS_UNBOUNDED:   return unbounded{};
            case COPT_MIPSTATUS_TIMEOUT:     return time_limit{has_sol != 0};
            case COPT_MIPSTATUS_NODELIMIT:   return node_limit{has_sol != 0};
            case COPT_MIPSTATUS_UNFINISHED:  return numerical_failure{has_sol != 0};
            case COPT_MIPSTATUS_INTERRUPTED: return interrupted{has_sol != 0};
            default:
                return unknown{has_sol != 0};
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
        check(COPT->GetIntAttr(prob, COPT_INTATTR_ISMIP, &_is_mip));
        if(_is_mip) {
            callback_context context{this, get_feasibility_tolerance(), {}};
            const auto detach = _attached_callbacks(context);
            check(COPT->Solve(prob));
            _status = _get_status_milp();
        } else {
            check(COPT->SolveLp(prob));
            _status = _get_status_lp();
        }
        _mark_results_valid();
    }

    // On a MIP the wrapper solves the model first, up to its first incumbent
    // (see copt_base): the reported status is reset before the first native
    // call, throw or return, and the solution readable afterwards is that
    // solve's.
    auto compute_iis() {
        reset_status();
        check(COPT->GetIntAttr(prob, COPT_INTATTR_ISMIP, &_is_mip));
        return _compute_iis<detail::iis_whole_or_sided_status,
                            detail::iis_whole_or_sided_status>(_is_mip != 0);
    }

    double get_solution_value() {
        _require_valid_results();
        double val;
        if(_is_mip)
            check(COPT->GetDblAttr(prob, COPT_DBLATTR_BESTOBJ, &val));
        else
            check(COPT->GetDblAttr(prob, COPT_DBLATTR_LPOBJVAL, &val));
        return val;
    }
    auto get_solution() {
        _require_valid_results();
        auto solution =
            std::make_unique_for_overwrite<double[]>(num_variables());
        if(_is_mip)
            check(COPT->GetSolution(prob, solution.get()));
        else
            check(COPT->GetLpSolution(prob, solution.get(), nullptr, nullptr,
                                      nullptr));
        return variable_mapping(std::move(solution));
    }
    double get_dual_bound() {
        _require_valid_results();
        if(!_is_mip) return get_solution_value();
        if(is_a<status::stopped>(_status) &&
           !status::solution_available(_status))
            return _search_bound;
        double bound;
        check(COPT->GetDblAttr(prob, COPT_DBLATTR_BESTBND, &bound));
        return bound;
    }
    std::size_t get_node_count() {
        _require_valid_results();
        if(!_is_mip) return 0u;
        int count;
        check(COPT->GetIntAttr(prob, COPT_INTATTR_NODECNT, &count));
        return static_cast<std::size_t>(count);
    }
};

}  // namespace copt::impl::v1
}  // namespace mippp
