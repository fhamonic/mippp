#pragma once

#include <algorithm>
#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <ranges>
#include <string_view>
#include <utility>
#include <variant>

#include "mippp/detail/restore_guard.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/mosek/impl/v1/mosek_base.hpp"

namespace mippp {
namespace mosek::impl::v1 {

class mosek_milp : public mosek_base {
public:
    [[nodiscard]] mosek_milp() : mosek_milp(mosek_api::load()) {}
    [[nodiscard]] explicit mosek_milp(const mosek_api & api) : mosek_base(api) {
        check(MSK->putintparam(task, MSK_IPAR_OPTIMIZER,
                               MSK_OPTIMIZER_MIXED_INT));
    }

    using model_base<int, double>::add_integer_variable;
    using model_base<int, double>::add_integer_variables;
    using model_base<int, double>::add_binary_variable;
    using model_base<int, double>::add_binary_variables;

    void set_continuous(variable v) {
        _invalidate_results();
        check(MSK->putvartype(task, v.id(), MSK_VAR_TYPE_CONT));
    }
    void set_integer(variable v) {
        _invalidate_results();
        check(MSK->putvartype(task, v.id(), MSK_VAR_TYPE_INT));
    }
    void set_binary(variable v) {
        _invalidate_results();
        set_integer(v);
        set_variable_lower_bound(v, 0);
        set_variable_upper_bound(v, 1);
    }
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// Callbacks ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    // By name, at each solve: MOSEK numbers callback codes and information
    // items alphabetically, so a release that adds one shifts those after it.
    struct progress_codes {
        MSKcallbackcodee intermediate;
        MSKcallbackcodee new_incumbent;
        MSKint32t incumbent;
        MSKint32t num_incumbents;
        MSKint32t bound;
        MSKint32t bound_defined;
        MSKint32t solved_nodes;
        bool minimizing;
    };

public:
    // MOSEK has no value for "none yet": both read 0 until the flags are set.
    class progress_callback_handle {
    private:
        friend mosek_milp;
        const progress_codes & codes;
        const MSKrealt * douinf;
        const MSKint32t * intinf;
        bool interrupt_requested = false;

        progress_callback_handle(const progress_codes & codes_,
                                 const MSKrealt * douinf_,
                                 const MSKint32t * intinf_)
            : codes(codes_), douinf(douinf_), intinf(intinf_) {}

        double _infinity(const bool positive) const {
            const double infinity = std::numeric_limits<double>::infinity();
            return positive ? infinity : -infinity;
        }

    public:
        double get_solution_value() {
            if(intinf[codes.num_incumbents] == 0)
                return _infinity(codes.minimizing);
            return douinf[codes.incumbent];
        }
        double get_dual_bound() {
            if(intinf[codes.bound_defined] == 0)
                return _infinity(!codes.minimizing);
            return douinf[codes.bound];
        }
        std::size_t get_node_count() {
            return static_cast<std::size_t>(intinf[codes.solved_nodes]);
        }
        void interrupt() { interrupt_requested = true; }
    };

private:
    std::function<void(progress_callback_handle &)> progress_callback;
    progress_codes _progress_codes{};

    static MSKint32t progress_callback_func(MSKtask_t, MSKuserhandle_t usrptr,
                                            MSKcallbackcodee caller,
                                            const MSKrealt * douinf,
                                            const MSKint32t * intinf,
                                            const MSKint64t *) {
        auto * model = static_cast<mosek_milp *>(usrptr);
        const progress_codes & codes = model->_progress_codes;
        if(caller != codes.intermediate && caller != codes.new_incumbent)
            return 0;
        progress_callback_handle handle(codes, douinf, intinf);
        model->progress_callback(handle);
        return handle.interrupt_requested ? 1 : 0;
    }
    MSKcallbackcodee _callback_code(const std::string_view name) const {
        char code_name[MSK_MAX_STR_LEN];
        // past the last code it succeeds with "[UNDEFINED CODE]", and no name
        // matches
        for(int code = 0; code < 256; ++code) {
            const auto candidate = static_cast<MSKcallbackcodee>(code);
            if(MSK->callbackcodetostr(candidate, code_name) == MSK_RES_OK &&
               name == code_name)
                return candidate;
        }
        throw solver_error(
            "MOSEK lacks a callback code of the progress callback");
    }
    void _read_progress_codes() {
        progress_codes & codes = _progress_codes;
        codes.intermediate = _callback_code("MSK_CALLBACK_IM_MIO");
        codes.new_incumbent = _callback_code("MSK_CALLBACK_NEW_INT_MIO");
        const auto item = [this](MSKinftypee type, const char * name,
                                 MSKint32t & item_index) {
            check(MSK->getinfindex(task, type, name, &item_index));
        };
        item(MSK_INF_DOU_TYPE, "MSK_DINF_MIO_OBJ_INT", codes.incumbent);
        item(MSK_INF_INT_TYPE, "MSK_IINF_MIO_NUM_INT_SOLUTIONS",
             codes.num_incumbents);
        item(MSK_INF_DOU_TYPE, "MSK_DINF_MIO_OBJ_BOUND", codes.bound);
        item(MSK_INF_INT_TYPE, "MSK_IINF_MIO_OBJ_BOUND_DEFINED",
             codes.bound_defined);
        item(MSK_INF_INT_TYPE, "MSK_IINF_MIO_NUM_SOLVED_NODES",
             codes.solved_nodes);
        MSKobjsensee sense;
        check(MSK->getobjsense(task, &sense));
        codes.minimizing = sense == MSK_OBJECTIVE_SENSE_MINIMIZE;
    }
    // For the duration of a solve only. The user data is this: registered by a
    // setter, it dangles once the model moves and the next solve crashes.
    [[nodiscard]] auto _attached_callbacks() {
        const bool attach = static_cast<bool>(progress_callback);
        if(attach) {
            _read_progress_codes();
            check(MSK->putcallbackfunc(task, progress_callback_func, this));
        }
        return detail::restore_guard([this, attach] {
            if(attach) check(MSK->putcallbackfunc(task, nullptr, nullptr));
        });
    }

public:
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
        tmp_scalars.resize(num_variables());
        std::fill(tmp_scalars.begin(), tmp_scalars.end(), 0.0);
        for(auto && [var, coef] : entries) {
            tmp_scalars[var.uid()] += coef;
        }
        check(MSK->putxx(task, MSK_SOL_ITG, tmp_scalars.data()));
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
    ////////////////////////////// Solve status ///////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    // clang-format off
private:
    using status_variant = std::variant<
            status::unknown,
            status::optimal,
            status::infeasible_or_unbounded,
            status::infeasible,
            status::primal_and_dual_infeasible,
            status::unbounded,
            status::limit_reached,
            status::time_limit,
            status::iteration_limit,
            status::node_limit, 
            status::solution_limit,
            status::failed,
            status::numerical_failure,
            status::interrupted>;

    status_variant _status = status::unknown{};
    std::optional<MSKsoltypee> _solution;

    std::optional<MSKsoltypee> _pick_solution() const {
        if(_slot_state(MSK_SOL_ITG)) return MSK_SOL_ITG;
        return _pick_continuous_solution();
    }
    MSKsoltypee _require_solution() const {
        if(!_solution) throw solver_error("MOSEK holds no solution");
        return *_solution;
    }

    // whether a stopped solve left the solution get_solution() reads
    bool _has_solution() {
        if(!_solution) return false;
        MSKsolstae solsta;
        check(MSK->getsolsta(task, *_solution, &solsta));
        return _has_primal_solution(solsta);
    }

    status_variant _get_status(MSKrescodee trm) {
        using namespace status;
        switch(trm) {
            case MSK_RES_OK: {
                // MOSEK 11's mixed-integer optimizer defines no slot for an
                // empty task, nor for a column-less one with a violated row
                if(!_solution) {
                    if(num_variables() == 0 && num_constraints() == 0)
                        return optimal{};
                    return unknown{};
                }
                const MSKsoltypee soltype = *_solution;
                MSKprostae prosta;
                check(MSK->getprosta(task, soltype, &prosta));
                switch(prosta) {
                    // an integer solution has no dual, so an optimal MIP
                    // reports PRIM_FEAS, not PRIM_AND_DUAL_FEAS
                    case MSK_PRO_STA_PRIM_FEAS:
                    case MSK_PRO_STA_PRIM_AND_DUAL_FEAS: {
                        MSKsolstae solsta;
                        check(MSK->getsolsta(task, soltype, &solsta));
                        switch (solsta) {
                            case MSK_SOL_STA_OPTIMAL:
                            case MSK_SOL_STA_INTEGER_OPTIMAL:  return optimal{};
                            default: 
                                return unknown{};
                        }
                    }
                    case MSK_PRO_STA_PRIM_INFEAS_OR_UNBOUNDED:
                                                  return infeasible_or_unbounded{};
                    case MSK_PRO_STA_PRIM_INFEAS: return infeasible{};
                    case MSK_PRO_STA_PRIM_AND_DUAL_INFEAS:
                                                  return primal_and_dual_infeasible{};
                    case MSK_PRO_STA_DUAL_INFEAS: return unbounded{};
                    case MSK_PRO_STA_DUAL_FEAS:
                    case MSK_PRO_STA_ILL_POSED:
                    case MSK_PRO_STA_UNKNOWN:
                    default: 
                        return unknown{};
                }
            }
            case MSK_RES_TRM_MAX_TIME:          return time_limit{_has_solution()};
            case MSK_RES_TRM_MAX_ITERATIONS:    return iteration_limit{_has_solution()};
            case MSK_RES_TRM_MIO_NUM_BRANCHES:
            case MSK_RES_TRM_MIO_NUM_RELAXS:    return node_limit{_has_solution()};
            case MSK_RES_TRM_NUM_MAX_NUM_INT_SOLUTIONS:
                                                return solution_limit{_has_solution()};
            case MSK_RES_TRM_OBJECTIVE_RANGE:   return limit_reached{_has_solution()};
            case MSK_RES_TRM_USER_CALLBACK:     return interrupted{_has_solution()};
            case MSK_RES_TRM_NUMERICAL_PROBLEM:
            case MSK_RES_TRM_MAX_NUM_SETBACKS:
            case MSK_RES_TRM_STALL:             return numerical_failure{_has_solution()};
            case MSK_RES_TRM_LOST_RACE:
            case MSK_RES_TRM_INTERNAL:
            case MSK_RES_TRM_INTERNAL_STOP:     return failed{_has_solution()};
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
        _solution.reset();
        _invalidate_results();
    }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////////// Solve //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void solve() {
        reset_status();  // a solve that throws reports unknown
        // MSK_optimize reports a limit as an error code; MSK_optimizetrm
        // returns it as the termination code, and is the solve itself
        MSKrescodee trm = MSK_RES_OK;
        const auto detach = _attached_callbacks();
        check(MSK->optimizetrm(task, &trm));
        _solution = _pick_solution();
        _status = _get_status(trm);
        _mark_results_valid();
    }
    double get_solution_value() {
        _require_valid_results();
        double val = 0.0;
        if(num_variables() > 0)
            check(MSK->getprimalobj(task, _require_solution(), &val));
        return val;
    }
    auto get_solution() {
        _require_valid_results();
        const auto num_vars = num_variables();
        auto solution = std::make_unique_for_overwrite<double[]>(num_vars);
        if(num_vars > 0)
            check(MSK->getxx(task, _require_solution(), solution.get()));
        return variable_mapping(std::move(solution));
    }

private:
    bool _has_integer_variables() {
        MSKint32t num_int_vars;
        check(MSK->getnumintvar(task, &num_int_vars));
        return num_int_vars > 0;
    }

public:
    // Information items are read by name: MOSEK numbers them alphabetically,
    // so a release that adds one shifts the codes of those after it.
    double get_dual_bound() {
        _require_valid_results();
        if(!_has_integer_variables()) return get_solution_value();
        double bound;
        check(MSK->getnadouinf(task, "MSK_DINF_MIO_OBJ_BOUND", &bound));
        return bound;
    }
    std::size_t get_node_count() {
        _require_valid_results();
        if(!_has_integer_variables()) return 0u;
        MSKint32t count;
        check(MSK->getnaintinf(task, "MSK_IINF_MIO_NUM_SOLVED_NODES", &count));
        return static_cast<std::size_t>(count);
    }
};

}  // namespace mosek::impl::v1
}  // namespace mippp
