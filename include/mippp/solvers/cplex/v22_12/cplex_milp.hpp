#ifndef MIPPP_CPLEX_v22_12_MILP_HPP
#define MIPPP_CPLEX_v22_12_MILP_HPP

#include <numeric>
#include <optional>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/cplex/v22_12/cplex_base.hpp"

namespace fhamonic::mippp {
namespace cplex::v22_12 {

class cplex_milp : public cplex_base {
public:
    [[nodiscard]] explicit cplex_milp(const cplex_api & api)
        : cplex_base(api) {}

    variable add_integer_variable(
        const variable_params params = default_variable_params) {
        int var_id = static_cast<int>(num_variables());
        _add_variable(params, CPX_INTEGER);
        return variable(var_id);
    }
    auto add_integer_variables(
        std::size_t count,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params, CPX_INTEGER);
        return _make_variables_range(offset, count);
    }
    template <typename IL>
    auto add_integer_variables(
        std::size_t count, IL && id_lambda,
        variable_params params = default_variable_params) noexcept {
        const std::size_t offset = num_variables();
        _add_variables(offset, count, params, CPX_INTEGER);
        return _make_indexed_variables_range(offset, count,
                                             std::forward<IL>(id_lambda));
    }

private:
    inline void _add_binary_variables(const std::size_t & offset,
                                      const std::size_t & count) {
        tmp_scalars.resize(2 * count);
        std::fill(tmp_scalars.begin(),
                  tmp_scalars.begin() + static_cast<std::ptrdiff_t>(count), 0);
        std::fill(tmp_scalars.begin() + static_cast<std::ptrdiff_t>(count),
                  tmp_scalars.end(), 1);
        tmp_types.resize(count);
        std::fill(tmp_types.begin(), tmp_types.end(), CPX_BINARY);
        check(CPX.newcols(
            env, lp, static_cast<int>(count), NULL, tmp_scalars.data(),
            tmp_scalars.data() + static_cast<std::ptrdiff_t>(count),
            tmp_types.data(), NULL));
    }

public:
    variable add_binary_variable() {
        int var_id = static_cast<int>(num_variables());
        _add_variable({.obj_coef = 0.0, .lower_bound = 0.0, .upper_bound = 1.0},
                      CPX_BINARY);
        return variable(var_id);
    }
    auto add_binary_variables(std::size_t count) noexcept {
        const std::size_t offset = num_variables();
        _add_binary_variables(offset, count);
        return _make_variables_range(offset, count);
    }
    template <typename IL>
    auto add_binary_variables(std::size_t count, IL && id_lambda) noexcept {
        const std::size_t offset = num_variables();
        _add_binary_variables(offset, count);
        return _make_indexed_variables_range(offset, count,
                                             std::forward<IL>(id_lambda));
    }

    void set_continuous(variable v) noexcept {
        int var_id = v.id();
        char type = CPX_CONTINUOUS;
        check(CPX.chgctype(env, lp, 1, &var_id, &type));
    }
    void set_integer(variable v) noexcept {
        int var_id = v.id();
        char type = CPX_INTEGER;
        check(CPX.chgctype(env, lp, 1, &var_id, &type));
    }
    void set_binary(variable v) noexcept {
        int var_id = v.id();
        char type = CPX_BINARY;
        check(CPX.chgctype(env, lp, 1, &var_id, &type));
        char lu[2] = {'L', 'U'};
        double bd[2] = {0.0, 1.0};
        check(CPX.chgbds(env, lp, 2, &var_id, lu, bd));
    }

    constraint add_indicator_constraint(variable x, bool val,
                                        linear_constraint auto && lc) {
        const int constr_id = static_cast<int>(num_constraints());
        _reset_cache(num_variables());
        _register_entries(lc.linear_terms());
        check(CPX.addindconstr(env, lp, x.id(), static_cast<int>(!val),
                               static_cast<int>(tmp_indices.size()), lc.rhs(),
                               constraint_sense_to_cplex_sense(lc.sense()),
                               tmp_indices.data(), tmp_scalars.data(), NULL));
        return constraint(constr_id);
    }

private:
    class callback_handle_base : public model_base<int, double> {
    protected:
        const cplex_api & CPX;
        CPXCALLBACKCONTEXTptr context;
        cplex_milp * model;

        void cbcheck(int error) {
            if(error == 0) return;
            CPX.callbackabort(context);
        }

    public:
        callback_handle_base(const cplex_api & api,
                             CPXCALLBACKCONTEXTptr context_,
                             cplex_milp * model_)
            : model_base<int, double>()
            , CPX(api)
            , context(context_)
            , model(model_) {}

        std::size_t num_variables() {
            return static_cast<std::size_t>(
                CPX.getnumcols(model->env, model->lp));
        }
    };

public:
    class candidate_solution_callback_handle : public callback_handle_base {
    public:
        candidate_solution_callback_handle(const cplex_api & api,
                                           CPXCALLBACKCONTEXTptr context_,
                                           cplex_milp * model_)
            : callback_handle_base(api, context_, model_) {}

        void add_lazy_constraint(linear_constraint auto && lc) {
            _reset_cache(num_variables());
            _register_entries(lc.linear_terms());
            int matbegin = 0;
            const double b = lc.rhs();
            const char sense = constraint_sense_to_cplex_sense(lc.sense());
            cbcheck(CPX.callbackrejectcandidate(
                context, 1, static_cast<int>(tmp_indices.size()), &b, &sense,
                &matbegin, tmp_indices.data(), tmp_scalars.data()));
        }

        double get_solution_value() {
            double obj;
            cbcheck(CPX.callbackgetcandidatepoint(context, NULL, 0, 0, &obj));
            return obj;
        }
        auto get_solution() {
            auto num_vars = num_variables();
            auto solution = std::make_unique_for_overwrite<double[]>(num_vars);
            cbcheck(CPX.callbackgetcandidatepoint(context, solution.get(), 0,
                                                static_cast<int>(num_vars) - 1,
                                                NULL));
            return variable_mapping(std::move(solution));
        }
    };

private:
    std::function<void(candidate_solution_callback_handle &)>
        candidate_solution_callback;

    static int candidate_solution_callback_fun(CPXCALLBACKCONTEXTptr context,
                                               CPXLONG contextid,
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
        check(CPX.callbacksetfunc(env, lp, CPX_CALLBACKCONTEXT_CANDIDATE,
                                  candidate_solution_callback_fun, this));
    }

    void set_optimality_tolerance(double tol) {
        check(CPX.setdblparam(env, CPXPARAM_MIP_Tolerances_MIPGap, tol));
    }
    double get_optimality_tolerance() {
        double tol;
        check(CPX.getdblparam(env, CPXPARAM_MIP_Tolerances_MIPGap, &tol));
        return tol;
    }

    void set_feasibility_tolerance(double tol) {
        check(CPX.setdblparam(env, CPXPARAM_MIP_Tolerances_Linearization, tol));
    }
    double get_feasibility_tolerance() {
        double tol;
        check(
            CPX.getdblparam(env, CPXPARAM_MIP_Tolerances_Linearization, &tol));
        return tol;
    }

    void solve() {
        int probtype = CPX.getprobtype(env, lp);
        switch(probtype) {
            case CPXPROB_MILP:
                check(CPX.mipopt(env, lp));
                return;
            case CPXPROB_LP:
                if(candidate_solution_callback)
                    throw std::runtime_error(
                        "cplex_milp: can't solve lp (no integer variables) "
                        "with candidate_solution_callback");
                check(CPX.lpopt(env, lp));
                return;
            default:
                throw std::runtime_error("cplex_milp: unknowned problem type " +
                                         std::to_string(probtype));
        }
    }

    double get_solution_value() {
        double val;
        check(CPX.solution(env, lp, NULL, &val, NULL, NULL, NULL, NULL));
        return val;
    }
    auto get_solution() {
        auto solution =
            std::make_unique_for_overwrite<double[]>(num_variables());
        check(CPX.solution(env, lp, NULL, NULL, solution.get(), NULL, NULL,
                           NULL));
        return variable_mapping(std::move(solution));
    }
};

}  // namespace cplex::v22_12
}  // namespace fhamonic::mippp

#endif  // MIPPP_CPLEX_v22_12_MILP_HPP