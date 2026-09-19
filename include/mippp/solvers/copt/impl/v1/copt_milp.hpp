#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <ranges>
#include <utility>
#include <variant>

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
        int var_id = v.id();
        char type = COPT_CONTINUOUS;
        check(COPT->SetColType(prob, 1, &var_id, &type));
    }
    void set_integer(variable v) {
        int var_id = v.id();
        char type = COPT_INTEGER;
        check(COPT->SetColType(prob, 1, &var_id, &type));
    }
    void set_binary(variable v) {
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

        friend copt_milp;
        candidate_solution_callback_handle(const copt_api * api,
                                           copt_prob * prob_, void * cbdata_)
            : model_base<int, double>()
            , COPT(api)
            , prob(prob_)
            , cbdata(cbdata_) {}

        void check(const ret_code error) { COPT->_check(nullptr, error); }

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
            check(COPT->GetCallbackInfo(cbdata, COPT_CBINFO_MIPCANDIDATE,
                                        solution.get()));
            return variable_mapping(std::move(solution));
        }
    };

private:
    std::function<void(candidate_solution_callback_handle &)> solution_callback;

    static int candidate_solution_callback_func(copt_prob * prob, void * cbdata,
                                                int cbctx, void * userdata) {
        if(cbctx != COPT_CBCONTEXT_MIPSOL) return 0;
        auto * model = static_cast<copt_milp *>(userdata);
        candidate_solution_callback_handle handle(model->COPT, prob, cbdata);
        model->solution_callback(handle);
        return 0;
    }

public:
    template <typename F>
    void set_candidate_solution_callback(F && f) {
        solution_callback = std::forward<F>(f);
        check(COPT->SetCallback(prob, candidate_solution_callback_func,
                                COPT_CBCONTEXT_MIPSOL, this));
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
        _add_mip_start(entries);
    }
    void add_mip_start(
        std::initializer_list<std::pair<variable, scalar>> entries) {
        _add_mip_start(entries);
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
        switch(status_) {
            case COPT_LPSTATUS_OPTIMAL:     return optimal{};
            case COPT_LPSTATUS_INFEASIBLE:  return infeasible{};
            case COPT_LPSTATUS_UNBOUNDED:   return unbounded{};
            case COPT_LPSTATUS_TIMEOUT:     return time_limit{};
            case COPT_LPSTATUS_NUMERICAL:
            case COPT_LPSTATUS_IMPRECISE:
            case COPT_LPSTATUS_UNFINISHED:  return numerical_failure{};
            case COPT_LPSTATUS_INTERRUPTED: return interrupted{};
            case COPT_LPSTATUS_UNSTARTED:   return unknown{};
            default:
                return unknown{};
        }
    }
    status_variant _get_status_milp() {
        using namespace status;
        int status_;
        check(COPT->GetIntAttr(prob, COPT_INTATTR_MIPSTATUS, &status_));
        switch(status_) {
            case COPT_MIPSTATUS_OPTIMAL:     return optimal{};
            case COPT_MIPSTATUS_INF_OR_UNB:  return infeasible_or_unbounded{};
            case COPT_MIPSTATUS_INFEASIBLE:  return infeasible{};
            case COPT_MIPSTATUS_UNBOUNDED:   return unbounded{};
            case COPT_MIPSTATUS_TIMEOUT:     return time_limit{};
            case COPT_MIPSTATUS_NODELIMIT:   return node_limit{};
            case COPT_MIPSTATUS_UNFINISHED:  return numerical_failure{};
            case COPT_MIPSTATUS_INTERRUPTED: return interrupted{};
            default:
                return unknown{};
        }
    }
    // clang-format on
public:
    const status_variant & get_status() const { return _status; }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////////// Solve //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void solve() {
        check(COPT->GetIntAttr(prob, COPT_INTATTR_ISMIP, &_is_mip));
        if(_is_mip) {
            check(COPT->Solve(prob));
            _status = _get_status_milp();
        } else {
            check(COPT->SolveLp(prob));
            _status = _get_status_lp();
        }
    }

    double get_solution_value() {
        double val;
        if(_is_mip)
            check(COPT->GetDblAttr(prob, COPT_DBLATTR_BESTOBJ, &val));
        else
            check(COPT->GetDblAttr(prob, COPT_DBLATTR_LPOBJVAL, &val));
        return val;
    }
    auto get_solution() {
        auto solution =
            std::make_unique_for_overwrite<double[]>(num_variables());
        if(_is_mip)
            check(COPT->GetSolution(prob, solution.get()));
        else
            check(COPT->GetLpSolution(prob, solution.get(), nullptr, nullptr,
                                      nullptr));
        return variable_mapping(std::move(solution));
    }
};

}  // namespace copt::impl::v1
}  // namespace mippp
