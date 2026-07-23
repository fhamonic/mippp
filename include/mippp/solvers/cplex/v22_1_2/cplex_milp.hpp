#pragma once

#include <functional>
#include <numeric>
#include <optional>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_entities.hpp"
#include "mippp/utility/memory_size.hpp"
#include "mippp/utility/solve_status.hpp"

#include "mippp/solvers/cplex/v22_1_2/cplex_base.hpp"

namespace mippp {
namespace cplex::v22_1_2 {

class cplex_milp : public cplex_base {
public:
    [[nodiscard]] explicit cplex_milp(const cplex_api & api)
        : cplex_base(api) {}

    variable add_integer_variable(
        const variable_params params = default_variable_params) {
        int var_id = _new_var_native_id();
        _add_variable(params, CPX_INTEGER);
        return _new_var_handle(var_id);
    }
    auto add_integer_variables(
        std::size_t count,
        variable_params params = default_variable_params) noexcept {
        const std::size_t handle_ids_begin =
            _add_variables(count, params, CPX_INTEGER);
        return _make_variables_view(handle_ids_begin, count);
    }
    template <typename IL>
    auto add_integer_variables(
        std::size_t count, IL && id_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t handle_ids_begin =
            _add_variables(count, params, CPX_INTEGER);
        return _make_indexed_variables_view(handle_ids_begin, count,
                                            std::forward<IL>(id_lambda));
    }

private:
    inline std::size_t _add_binary_variables(const std::size_t & count) {
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
    variable add_binary_variable() {
        int var_id = _new_var_native_id();
        _add_variable({.obj_coef = 0.0, .lower_bound = 0.0, .upper_bound = 1.0},
                      CPX_BINARY);
        return _new_var_handle(var_id);
    }
    auto add_binary_variables(std::size_t count) noexcept {
        const std::size_t handle_ids_begin = _add_binary_variables(count);
        return _make_variables_view(handle_ids_begin, count);
    }
    template <typename IL>
    auto add_binary_variables(std::size_t count, IL && id_lambda) noexcept {
        const std::size_t handle_ids_begin = _add_binary_variables(count);
        return _make_indexed_variables_view(handle_ids_begin, count,
                                            std::forward<IL>(id_lambda));
    }
    void set_continuous(variable v) noexcept {
        int var_id = _native_id(v);
        char type = CPX_CONTINUOUS;
        check(CPX->chgctype(env, lp, 1, &var_id, &type));
    }
    void set_integer(variable v) noexcept {
        int var_id = _native_id(v);
        char type = CPX_INTEGER;
        check(CPX->chgctype(env, lp, 1, &var_id, &type));
    }
    void set_binary(variable v) noexcept {
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
    void add_indicator_constraint(variable x, bool val,
                                  linear_constraint auto && lc) {
        _reset_cache(_num_var_native_ids());
        _register_entries(lc.linear_terms());
        check(CPX->addindconstr(env, lp, x.id(), static_cast<int>(!val),
                                static_cast<int>(tmp_indices.size()), lc.rhs(),
                                constraint_sense_to_cplex_sense(lc.sense()),
                                tmp_indices.data(), tmp_scalars.data(),
                                nullptr));
    }
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// Callbacks ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    class callback_handle_base : public model_base<int, double> {
    protected:
        const cplex_api * CPX;
        CPXCALLBACKCONTEXTptr context;
        cplex_milp * model;

        void cbcheck(int error) {
            if(error == 0) return;
            CPX->callbackabort(context);
        }

    public:
        callback_handle_base(const cplex_api * api,
                             CPXCALLBACKCONTEXTptr context_,
                             cplex_milp * model_)
            : model_base<int, double>()
            , CPX(api)
            , context(context_)
            , model(model_) {}

        std::size_t num_variables() {
            return static_cast<std::size_t>(
                CPX->getnumcols(model->env, model->lp));
        }
    };

public:
    class candidate_solution_callback_handle : public callback_handle_base {
    public:
        candidate_solution_callback_handle(const cplex_api * api,
                                           CPXCALLBACKCONTEXTptr context_,
                                           cplex_milp * model_)
            : callback_handle_base(api, context_, model_) {}

        void add_lazy_constraint(linear_constraint auto && lc) {
            _reset_cache(model->_num_var_native_ids());
            _register_entries(lc.linear_terms());
            if(model->_remap_ids) {
                for(auto & id : tmp_indices)
                    id = model->_native_ids_map[static_cast<std::size_t>(id)];
            }
            int matbegin = 0;
            const double b = lc.rhs();
            const char sense = constraint_sense_to_cplex_sense(lc.sense());
            cbcheck(CPX->callbackrejectcandidate(
                context, 1, static_cast<int>(tmp_indices.size()), &b, &sense,
                &matbegin, tmp_indices.data(), tmp_scalars.data()));
        }
        double get_solution_value() {
            double obj;
            cbcheck(
                CPX->callbackgetcandidatepoint(context, nullptr, 0, 0, &obj));
            return obj;
        }
        auto get_solution() {
            auto num_vars = model->_num_var_native_ids();
            auto solution = std::make_unique_for_overwrite<double[]>(num_vars);
            cbcheck(CPX->callbackgetcandidatepoint(
                context, solution.get(), 0, static_cast<int>(num_vars) - 1,
                nullptr));
            return variable_mapping(
                [this, solution = std::move(solution)](const variable & v) {
                    return *(solution.get() + model->_native_id(v));
                });
        }
    };

private:
    std::function<void(candidate_solution_callback_handle &)>
        candidate_solution_callback;

    static int candidate_solution_callback_fun(
        CPXCALLBACKCONTEXTptr context, [[maybe_unused]] CPXLONG contextid,
        void * userhandle) {
        auto * model = static_cast<cplex_milp *>(userhandle);
        candidate_solution_callback_handle handle(model->CPX, context, model);
        model->candidate_solution_callback(handle);
        return 0;
    }

public:
    template <typename F>
    void set_candidate_solution_callback(F && f) {
        candidate_solution_callback = std::forward<F>(f);
        check(CPX->callbacksetfunc(env, lp, CPX_CALLBACKCONTEXT_CANDIDATE,
                                   candidate_solution_callback_fun, this));
    }
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// MIP start ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    template <typename ER>
    inline void _add_mip_start(ER && entries) {
        _reset_raw_cache();
        _register_raw_entries(entries);
        int beg = 0;
        int effort_level = CPX_MIPSTART_SOLVEMIP;
        check(CPX->addmipstarts(
            env, lp, 1, static_cast<int>(tmp_indices.size()), &beg,
            tmp_indices.data(), tmp_scalars.data(), &effort_level, nullptr));
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
        check(CPX->setdblparam(env, CPXPARAM_MIP_Limits_TreeMemory, mb.count));
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
    void set_feasibility_tolerance(double tol) {
        check(
            CPX->setdblparam(env, CPXPARAM_MIP_Tolerances_Linearization, tol));
    }
    double get_feasibility_tolerance() {
        double tol;
        check(
            CPX->getdblparam(env, CPXPARAM_MIP_Tolerances_Linearization, &tol));
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

    status_variant _status;

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
            // unimplemented limits for now
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
            // unimplemented limits for now
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
    const status_variant & solve_status() const { return _status; }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////////// Solve //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void solve() {
        int probtype = CPX->getprobtype(env, lp);
        switch(probtype) {
            case CPXPROB_MILP:
                check(CPX->mipopt(env, lp));
                _status = _get_status_milp();
                return;
            case CPXPROB_LP:
                if(candidate_solution_callback)
                    throw std::runtime_error(
                        "cplex_milp: can't solve lp (no integer variables) "
                        "with candidate_solution_callback");
                check(CPX->lpopt(env, lp));
                _status = _get_status_lp();
                return;
            default:
                throw std::runtime_error("cplex_milp: unknowned problem type " +
                                         std::to_string(probtype));
        }
    }
    double get_solution_value() {
        double val;
        check(CPX->solution(env, lp, nullptr, &val, nullptr, nullptr, nullptr,
                            nullptr));
        return val;
        // double val;
        // check(CPX->getbestobjval(env, lp, &val));
        // return val;
    }
    auto get_solution() {
        auto solution =
            std::make_unique_for_overwrite<double[]>(_num_var_native_ids());
        check(CPX->solution(env, lp, nullptr, nullptr, solution.get(), nullptr,
                            nullptr, nullptr));
        return variable_mapping(
            [this, solution = std::move(solution)](const variable & v) {
                return *(solution.get() + _native_id(v));
            });
    }
};

}  // namespace cplex::v22_1_2
}  // namespace mippp
